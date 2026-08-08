#!/usr/bin/env python3
"""Safe ACT rollout helper for PiPER + LeRobot.

Default mode is dry-run: load policy, read the robot/cameras, predict actions,
and print the limited command without sending it to the arm.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np
import torch


_SCRIPT_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _SCRIPT_DIR.parent
_LEROBOT_SRC = _REPO_ROOT / "src"
if str(_LEROBOT_SRC) not in sys.path:
    sys.path.insert(0, str(_LEROBOT_SRC))

from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.configs.policies import PreTrainedConfig
from lerobot.constants import POLICY_POSTPROCESSOR_DEFAULT_NAME, POLICY_PREPROCESSOR_DEFAULT_NAME
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from lerobot.robots.piper_follower.config_piper_follower import PiperFollowerConfig
from lerobot.robots.piper_follower.piper_follower import PiperFollower
from lerobot.scripts.server.helpers import (
    map_robot_keys_to_lerobot_features,
    raw_observation_to_observation,
)


ACTION_KEYS = [
    "joint1.pos",
    "joint2.pos",
    "joint3.pos",
    "joint4.pos",
    "joint5.pos",
    "joint6.pos",
    "gripper.pos",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint",
        default=str(_REPO_ROOT.parent / "checkpoints" / "piper_act_smoke_10ep_001000"),
        help="Local pretrained_model directory.",
    )
    parser.add_argument("--port", default="can1", help="Follower CAN interface.")
    parser.add_argument("--wrist-serial", default=os.environ.get("WRIST_REALSENSE_SERIAL", "238222076529"))
    parser.add_argument("--global-serial", default=os.environ.get("GLOBAL_REALSENSE_SERIAL", "142122071524"))
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--device", default="cpu", choices=["cpu", "cuda"], help="Policy inference device.")
    parser.add_argument("--steps", type=int, default=20, help="Number of control iterations.")
    parser.add_argument("--control-fps", type=float, default=5.0, help="Execution loop rate.")
    parser.add_argument(
        "--max-delta",
        type=float,
        default=2.0,
        help="Max normalized action delta per step for joints 1-6.",
    )
    parser.add_argument(
        "--max-gripper-delta",
        type=float,
        default=3.0,
        help="Max normalized gripper delta per step.",
    )
    parser.add_argument(
        "--disable-gripper",
        action="store_true",
        help="Hold gripper at current position while testing arm joints.",
    )
    parser.add_argument(
        "--ema-alpha",
        type=float,
        default=1.0,
        help=(
            "Exponential moving average for limited actions. "
            "1.0 disables smoothing; 0.3-0.6 is useful for reducing ACT chunk-boundary jitter."
        ),
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually send limited actions to the follower. Without this, dry-run only.",
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Allow PiperFollower.connect() to run parking(). Keep false for first tests.",
    )
    return parser.parse_args()


def require_checkpoint(path: Path) -> None:
    required = [
        "config.json",
        "model.safetensors",
        f"{POLICY_PREPROCESSOR_DEFAULT_NAME}.json",
        f"{POLICY_POSTPROCESSOR_DEFAULT_NAME}.json",
    ]
    missing = [name for name in required if not (path / name).exists()]
    if missing:
        raise FileNotFoundError(f"Checkpoint is incomplete at {path}. Missing: {missing}")


def make_robot(args: argparse.Namespace) -> PiperFollower:
    cameras = {
        "wrist": RealSenseCameraConfig(
            serial_number_or_name=args.wrist_serial,
            width=args.width,
            height=args.height,
            fps=args.fps,
        ),
        "global": RealSenseCameraConfig(
            serial_number_or_name=args.global_serial,
            width=args.width,
            height=args.height,
            fps=args.fps,
        ),
    }
    cfg = PiperFollowerConfig(port=args.port, id="act_safe_follower", cameras=cameras)
    return PiperFollower(cfg)


def load_policy(checkpoint: Path, device: str):
    cfg = PreTrainedConfig.from_pretrained(checkpoint)
    cfg.device = device
    policy_cls = get_policy_class(cfg.type)
    policy = policy_cls.from_pretrained(checkpoint, config=cfg, local_files_only=True)
    policy.to(device)
    policy.eval()
    policy.reset()

    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=cfg,
        pretrained_path=str(checkpoint),
        preprocessor_overrides={"device_processor": {"device": device}},
        postprocessor_overrides={"device_processor": {"device": "cpu"}},
    )
    return cfg, policy, preprocessor, postprocessor


def action_tensor_to_dict(action_tensor: torch.Tensor) -> dict[str, float]:
    action_tensor = action_tensor.detach().cpu().flatten()
    return {key: float(action_tensor[i].item()) for i, key in enumerate(ACTION_KEYS)}


def current_state_from_obs(obs: dict) -> dict[str, float]:
    return {key: float(obs[key]) for key in ACTION_KEYS}


def limit_action(
    predicted: dict[str, float],
    current: dict[str, float],
    max_delta: float,
    max_gripper_delta: float,
    disable_gripper: bool,
) -> dict[str, float]:
    limited = {}
    for key in ACTION_KEYS:
        delta_limit = max_gripper_delta if key == "gripper.pos" else max_delta
        abs_low, abs_high = (0.0, 100.0) if key == "gripper.pos" else (-100.0, 100.0)
        if key == "gripper.pos" and disable_gripper:
            limited[key] = float(np.clip(current[key], abs_low, abs_high))
            continue
        delta = np.clip(predicted[key] - current[key], -delta_limit, delta_limit)
        limited[key] = float(np.clip(current[key] + float(delta), abs_low, abs_high))
    return limited


def format_action(action: dict[str, float]) -> str:
    return "[" + ", ".join(f"{action[key]:7.2f}" for key in ACTION_KEYS) + "]"


def smooth_action(
    action: dict[str, float],
    previous: dict[str, float] | None,
    alpha: float,
) -> dict[str, float]:
    if previous is None or alpha >= 1.0:
        return dict(action)
    if alpha <= 0.0:
        return dict(previous)
    return {key: alpha * action[key] + (1.0 - alpha) * previous[key] for key in ACTION_KEYS}


def main() -> int:
    args = parse_args()
    if not 0.0 < args.ema_alpha <= 1.0:
        raise ValueError(f"--ema-alpha must be in (0, 1], got {args.ema_alpha}")
    checkpoint = Path(args.checkpoint).expanduser().resolve()
    require_checkpoint(checkpoint)

    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("Requested --device cuda, but torch.cuda.is_available() is false.")

    print(f"[policy] checkpoint={checkpoint}")
    print(f"[policy] device={args.device}")
    cfg, policy, preprocessor, postprocessor = load_policy(checkpoint, args.device)
    print(f"[policy] type={cfg.type}, image_features={list(cfg.image_features)}")

    robot = make_robot(args)
    print(f"[robot] connecting port={args.port}, execute={args.execute}, calibrate={args.calibrate}")
    robot.connect(calibrate=args.calibrate)
    lerobot_features = map_robot_keys_to_lerobot_features(robot)

    period = 1.0 / args.control_fps
    previous_limited = None
    try:
        for step in range(args.steps):
            loop_start = time.perf_counter()
            raw_obs = robot.get_observation()
            obs = raw_observation_to_observation(raw_obs, lerobot_features, cfg.image_features, args.device)
            obs = preprocessor(obs)

            with torch.inference_mode():
                raw_action = policy.select_action(obs)
                action = postprocessor.process_action(raw_action)

            predicted = action_tensor_to_dict(action)
            current = current_state_from_obs(raw_obs)
            limited = limit_action(
                predicted,
                current,
                args.max_delta,
                args.max_gripper_delta,
                args.disable_gripper,
            )
            smoothed = smooth_action(limited, previous_limited, args.ema_alpha)
            previous_limited = smoothed

            print(
                f"[{step:03d}] current={format_action(current)} "
                f"pred={format_action(predicted)} limited={format_action(limited)} "
                f"cmd={format_action(smoothed)}"
            )

            if args.execute:
                robot.send_action(smoothed)

            elapsed = time.perf_counter() - loop_start
            time.sleep(max(0.0, period - elapsed))
    except KeyboardInterrupt:
        print("\n[stop] Ctrl-C received.")
    finally:
        robot.disconnect(disable_torque=False)
        print("[done] disconnected.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
