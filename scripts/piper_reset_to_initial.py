#!/usr/bin/env python3
"""Move one or more PiPER arms to a shared initial joint posture.

Default target is the all-zero joint posture:
    joint1=0, joint2=0, joint3=0, joint4=0, joint5=0, joint6=0 deg

This script sends motion commands. Use only with the workspace clear.
"""

from __future__ import annotations

import argparse
import subprocess
import time

from piper_sdk import C_PiperInterface_V2


DEFAULT_TARGET_DEG = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def connect(can_name: str) -> C_PiperInterface_V2:
    piper = C_PiperInterface_V2(can_name=can_name)
    piper.ConnectPort(can_init=False, piper_init=False, start_thread=True)
    return piper


def can_is_up(can_name: str) -> bool:
    result = subprocess.run(
        ["ip", "link", "show", can_name],
        check=False,
        capture_output=True,
        text=True,
    )
    return result.returncode == 0 and "UP" in result.stdout


def print_arm_summary(port: str, piper: C_PiperInterface_V2) -> None:
    status = piper.GetArmStatus()
    joint_msg = piper.GetArmJointMsgs()
    enable_status = piper.GetArmEnableStatus()
    arm_status = status.arm_status
    print(
        f"[reset] {port}: "
        f"status_hz={float(getattr(status, 'Hz', 0.0)):.1f} "
        f"joint_hz={float(getattr(joint_msg, 'Hz', 0.0)):.1f} "
        f"ctrl_mode={getattr(arm_status, 'ctrl_mode', '?')} "
        f"teach_status={getattr(arm_status, 'teach_status', '?')} "
        f"err_code={getattr(arm_status, 'err_code', '?')} "
        f"enable={enable_status}"
    )


def parse_degrees(raw: str) -> list[int]:
    values = [float(item.strip()) for item in raw.split(",") if item.strip()]
    if len(values) != 6:
        raise ValueError("--target-deg must contain 6 comma-separated numbers")
    return [int(round(value * 1000)) for value in values]


def configure_arm(piper: C_PiperInterface_V2, speed: int, high_follow: bool) -> None:
    mit_flag = 0xAD if high_follow else 0x00
    piper.ModeCtrl(0x01, 0x01, speed, mit_flag)
    time.sleep(0.05)
    piper.MotionCtrl_2(0x01, 0x01, speed, mit_flag)
    time.sleep(0.05)
    for _ in range(20):
        piper.EnableArm()
        time.sleep(0.05)
        if all(piper.GetArmEnableStatus()):
            break


def send_target(piper: C_PiperInterface_V2, target_mdeg: list[int], gripper_um: int, speed: int, high_follow: bool) -> None:
    mit_flag = 0xAD if high_follow else 0x00
    piper.MotionCtrl_2(0x01, 0x01, speed, mit_flag)
    piper.JointCtrl(*target_mdeg)
    piper.GripperCtrl(abs(gripper_um), 1000, 0x03, 0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ports", nargs="+", default=["can0", "can1"])
    parser.add_argument("--target-deg", default=",".join(str(v) for v in DEFAULT_TARGET_DEG))
    parser.add_argument("--gripper-um", type=int, default=0)
    parser.add_argument("--speed", type=int, default=50)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--high-follow", action="store_true")
    parser.add_argument(
        "--master-home-ports",
        nargs="*",
        default=["can0"],
        help="Ports that should also receive the official master-arm home command. Use an empty value to disable.",
    )
    args = parser.parse_args()

    target_mdeg = parse_degrees(args.target_deg)
    period = 1.0 / args.hz

    print(f"[reset] ports={args.ports}")
    print(f"[reset] target_deg={[v / 1000 for v in target_mdeg]}, gripper_um={args.gripper_um}")
    print(f"[reset] master_home_ports={args.master_home_ports}")
    for port in args.ports:
        if not can_is_up(port):
            raise RuntimeError(f"{port} is not UP. Run: sudo ip link set {port} type can bitrate 1000000 && sudo ip link set {port} up")
    arms = {port: connect(port) for port in args.ports}

    try:
        time.sleep(0.3)
        for port, arm in arms.items():
            print_arm_summary(port, arm)
            configure_arm(arm, args.speed, args.high_follow)
            print_arm_summary(port, arm)
            if port in args.master_home_ports:
                print(f"[reset] sending ReqMasterArmMoveToHome(1) to {port}")
                arm.ReqMasterArmMoveToHome(1)
                time.sleep(0.1)

        start = time.monotonic()
        while time.monotonic() - start < args.duration:
            loop_start = time.monotonic()
            for arm in arms.values():
                send_target(arm, target_mdeg, args.gripper_um, args.speed, args.high_follow)
            time.sleep(max(0.0, period - (time.monotonic() - loop_start)))
    finally:
        for arm in arms.values():
            arm.DisconnectPort()

    print("[reset] done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
