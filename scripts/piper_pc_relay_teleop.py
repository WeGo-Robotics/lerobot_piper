#!/usr/bin/env python3
"""PC relay teleoperation for two PiPER arms on isolated CAN buses.

Topology:
    leader arm   -> USB-CAN A -> can0 -> PC
    follower arm -> USB-CAN B -> can1 -> PC

The script reads leader joint targets from can0 and sends rate-limited joint
targets to the follower on can1. It defaults to dry-run mode and will not move
the follower unless --execute is passed.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Literal

from piper_sdk import C_PiperInterface_V2


JointSource = Literal["auto", "feedback", "control"]

JOINT_LIMITS_MDEG = [
    (-150_000, 150_000),
    (0, 180_000),
    (-170_000, 0),
    (-100_000, 100_000),
    (-70_000, 70_000),
    (-120_000, 120_000),
]

GRIPPER_LIMIT_UM = (0, 68_000)


@dataclass
class ArmSample:
    joints: list[int]
    gripper: int
    joint_hz: float
    gripper_hz: float
    joint_timestamp: float
    source: str


def parse_float_list(raw: str, expected: int, name: str) -> list[float]:
    values = [float(item.strip()) for item in raw.split(",") if item.strip()]
    if len(values) != expected:
        raise ValueError(f"{name} must contain {expected} comma-separated values, got {len(values)}")
    return values


def connect_piper(can_name: str) -> C_PiperInterface_V2:
    piper = C_PiperInterface_V2(can_name=can_name)
    piper.ConnectPort(can_init=False, piper_init=False, start_thread=True)
    return piper


def feedback_sample(piper: C_PiperInterface_V2) -> ArmSample:
    joint_msg = piper.GetArmJointMsgs()
    gripper_msg = piper.GetArmGripperMsgs()
    joint = joint_msg.joint_state
    gripper = gripper_msg.gripper_state
    return ArmSample(
        joints=[
            int(joint.joint_1),
            int(joint.joint_2),
            int(joint.joint_3),
            int(joint.joint_4),
            int(joint.joint_5),
            int(joint.joint_6),
        ],
        gripper=int(gripper.grippers_angle),
        joint_hz=float(getattr(joint_msg, "Hz", 0.0)),
        gripper_hz=float(getattr(gripper_msg, "Hz", 0.0)),
        joint_timestamp=float(getattr(joint_msg, "time_stamp", 0.0)),
        source="feedback",
    )


def control_sample(piper: C_PiperInterface_V2) -> ArmSample:
    joint_msg = piper.GetArmJointCtrl()
    gripper_msg = piper.GetArmGripperCtrl()
    joint = joint_msg.joint_ctrl
    gripper = gripper_msg.gripper_ctrl
    return ArmSample(
        joints=[
            int(joint.joint_1),
            int(joint.joint_2),
            int(joint.joint_3),
            int(joint.joint_4),
            int(joint.joint_5),
            int(joint.joint_6),
        ],
        gripper=int(gripper.grippers_angle),
        joint_hz=float(getattr(joint_msg, "Hz", 0.0)),
        gripper_hz=float(getattr(gripper_msg, "Hz", 0.0)),
        joint_timestamp=float(getattr(joint_msg, "time_stamp", 0.0)),
        source="control",
    )


def read_leader(piper: C_PiperInterface_V2, source: JointSource) -> ArmSample:
    if source == "feedback":
        return feedback_sample(piper)
    if source == "control":
        return control_sample(piper)

    ctrl = control_sample(piper)
    if ctrl.joint_hz > 0 or ctrl.joint_timestamp > 0:
        return ctrl
    return feedback_sample(piper)


def clamp(value: int, lower: int, upper: int) -> int:
    return max(lower, min(upper, value))


def rate_limit(current: Iterable[int], target: Iterable[int], max_step: int) -> list[int]:
    limited = []
    for cur, tgt in zip(current, target, strict=True):
        delta = tgt - cur
        if delta > max_step:
            limited.append(cur + max_step)
        elif delta < -max_step:
            limited.append(cur - max_step)
        else:
            limited.append(tgt)
    return limited


def transform_joints(joints: list[int], signs: list[float], offsets_mdeg: list[float]) -> list[int]:
    return [int(round(joint * sign + offset)) for joint, sign, offset in zip(joints, signs, offsets_mdeg, strict=True)]


def bounded_joints(joints: list[int]) -> list[int]:
    return [clamp(joint, low, high) for joint, (low, high) in zip(joints, JOINT_LIMITS_MDEG, strict=True)]


def configure_follower_for_joint_control(piper: C_PiperInterface_V2, speed: int, high_follow: bool) -> None:
    mit_flag = 0xAD if high_follow else 0x00
    piper.MotionCtrl_2(0x01, 0x01, speed, mit_flag)
    time.sleep(0.05)
    piper.EnableArm()
    time.sleep(0.1)


def send_follower(
    piper: C_PiperInterface_V2,
    joints: list[int],
    gripper: int | None,
    speed: int,
    high_follow: bool,
    send_mode: bool,
) -> None:
    if send_mode:
        mit_flag = 0xAD if high_follow else 0x00
        piper.MotionCtrl_2(0x01, 0x01, speed, mit_flag)
    piper.JointCtrl(*joints)
    if gripper is not None:
        piper.GripperCtrl(clamp(abs(gripper), *GRIPPER_LIMIT_UM), 1000, 0x03, 0)


def fmt_degrees(values_mdeg: Iterable[int]) -> str:
    return "[" + ", ".join(f"{value / 1000.0:8.3f}" for value in values_mdeg) + "]"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--leader", default="can0", help="CAN interface connected to the leader arm")
    parser.add_argument("--follower", default="can1", help="CAN interface connected to the follower arm")
    parser.add_argument("--source", choices=["auto", "feedback", "control"], default="auto")
    parser.add_argument("--hz", type=float, default=10.0, help="Relay loop frequency")
    parser.add_argument("--speed", type=int, default=10, help="Follower speed percentage sent to MotionCtrl_2")
    parser.add_argument("--max-step-deg", type=float, default=1.0, help="Max follower joint step per cycle")
    parser.add_argument(
        "--mode-command-period",
        type=float,
        default=1.0,
        help="Seconds between repeated MotionCtrl_2 commands; 0 disables repeated mode commands",
    )
    parser.add_argument("--gripper-hz", type=float, default=5.0, help="Max gripper command frequency")
    parser.add_argument("--print-period", type=float, default=1.0, help="Seconds between status prints; 0 disables periodic status prints")
    parser.add_argument("--pause-file", default="", help="If this file exists, pause follower command sending")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run; 0 means until Ctrl-C")
    parser.add_argument("--signs", default="1,1,1,1,1,1", help="Per-joint sign mapping, comma-separated")
    parser.add_argument("--offset-deg", default="0,0,0,0,0,0", help="Per-joint offset in degrees, comma-separated")
    parser.add_argument("--include-gripper", action="store_true", help="Relay gripper target too")
    parser.add_argument("--high-follow", action="store_true", help="Use MotionCtrl_2 is_mit_mode=0xAD")
    parser.add_argument("--skip-follower-config", action="store_true", help="Do not send initial mode/enable commands")
    parser.add_argument("--execute", action="store_true", help="Actually send commands to the follower")
    args = parser.parse_args()

    if args.hz <= 0:
        raise ValueError("--hz must be positive")
    if args.max_step_deg <= 0:
        raise ValueError("--max-step-deg must be positive")
    if not 0 <= args.speed <= 100:
        raise ValueError("--speed must be in [0, 100]")

    signs = parse_float_list(args.signs, 6, "--signs")
    offsets_mdeg = [value * 1000.0 for value in parse_float_list(args.offset_deg, 6, "--offset-deg")]
    max_step_mdeg = int(round(args.max_step_deg * 1000))
    period = 1.0 / args.hz
    stop = False

    def handle_signal(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    print(f"Connecting leader={args.leader}, follower={args.follower}")
    leader = connect_piper(args.leader)
    follower = connect_piper(args.follower)
    time.sleep(0.5)

    if args.execute and not args.skip_follower_config:
        print("Configuring follower for joint control and enabling follower arm")
        configure_follower_for_joint_control(follower, args.speed, args.high_follow)

    start_time = time.monotonic()
    next_print = start_time
    last_mode_command = 0.0
    last_gripper_command = 0.0
    iterations = 0

    print(
        "Mode:",
        "EXECUTE: follower will move" if args.execute else "DRY-RUN: no follower commands will be sent",
    )
    print("Press Ctrl-C to stop.")

    try:
        while not stop:
            now = time.monotonic()
            if args.duration > 0 and now - start_time >= args.duration:
                break

            if args.pause_file and os.path.exists(args.pause_file):
                time.sleep(period)
                continue

            leader_sample = read_leader(leader, args.source)
            follower_sample = feedback_sample(follower)

            if leader_sample.joint_hz <= 0 and leader_sample.joint_timestamp <= 0:
                if args.print_period > 0 and now >= next_print:
                    print(
                        f"No leader {leader_sample.source} joint stream on {args.leader}; "
                        "not sending follower command."
                    )
                    next_print = now + args.print_period
                time.sleep(period)
                continue

            raw_target = transform_joints(leader_sample.joints, signs, offsets_mdeg)
            bounded_target = bounded_joints(raw_target)
            limited_target = rate_limit(follower_sample.joints, bounded_target, max_step_mdeg)
            send_gripper = (
                args.include_gripper
                and (args.gripper_hz <= 0 or now - last_gripper_command >= 1.0 / args.gripper_hz)
            )
            gripper = leader_sample.gripper if send_gripper else None
            send_mode = args.mode_command_period > 0 and now - last_mode_command >= args.mode_command_period

            if args.execute:
                send_follower(follower, limited_target, gripper, args.speed, args.high_follow, send_mode)
                if send_mode:
                    last_mode_command = now
                if send_gripper:
                    last_gripper_command = now

            iterations += 1
            if args.print_period > 0 and now >= next_print:
                print(
                    f"{iterations:06d} source={leader_sample.source} "
                    f"leader_hz={leader_sample.joint_hz:.1f} "
                    f"leader_ts={leader_sample.joint_timestamp:.3f} "
                    f"follower_hz={follower_sample.joint_hz:.1f}\n"
                    f"  leader   deg {fmt_degrees(leader_sample.joints)}\n"
                    f"  follower deg {fmt_degrees(follower_sample.joints)}\n"
                    f"  command  deg {fmt_degrees(limited_target)}"
                )
                next_print = now + args.print_period

            elapsed = time.monotonic() - now
            time.sleep(max(0.0, period - elapsed))
    finally:
        if args.execute:
            print("Stopping follower command stream; sending standby mode.")
            try:
                follower.MotionCtrl_2(0x00, 0x01, 0)
            except Exception as exc:  # noqa: BLE001
                print(f"Failed to send standby command: {exc}", file=sys.stderr)
        leader.DisconnectPort()
        follower.DisconnectPort()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
