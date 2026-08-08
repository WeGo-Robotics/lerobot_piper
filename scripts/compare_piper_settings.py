#!/usr/bin/env python3
"""Read and compare relevant PiPER settings on two CAN ports.

This script is read-only except for SDK parameter enquiry commands. It does not
enable motors and does not send motion commands.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass, asdict
from typing import Any

from piper_sdk import C_PiperInterface_V2


@dataclass
class PiperSettingsSnapshot:
    port: str
    status_hz: float
    joint_hz: float
    ctrl_hz: float
    control_mode: str
    arm_status: str
    mode_feed: str
    teach_status: str
    motion_status: str
    error_code: int
    joint_positions_mdeg: list[int]
    ctrl_positions_mdeg: list[int]
    mode_ctrl: str
    ctrl_151: str
    end_vel_acc: str
    crash_protection: str
    gripper_teaching: str
    enable_status: list[bool]


def connect(can_name: str) -> C_PiperInterface_V2:
    piper = C_PiperInterface_V2(can_name=can_name)
    piper.ConnectPort(can_init=False, piper_init=False, start_thread=True)
    return piper


def joint_feedback_values(piper: C_PiperInterface_V2) -> tuple[list[int], float]:
    msg = piper.GetArmJointMsgs()
    state = msg.joint_state
    return (
        [
            int(state.joint_1),
            int(state.joint_2),
            int(state.joint_3),
            int(state.joint_4),
            int(state.joint_5),
            int(state.joint_6),
        ],
        float(getattr(msg, "Hz", 0.0)),
    )


def joint_control_values(piper: C_PiperInterface_V2) -> tuple[list[int], float]:
    msg = piper.GetArmJointCtrl()
    ctrl = msg.joint_ctrl
    return (
        [
            int(ctrl.joint_1),
            int(ctrl.joint_2),
            int(ctrl.joint_3),
            int(ctrl.joint_4),
            int(ctrl.joint_5),
            int(ctrl.joint_6),
        ],
        float(getattr(msg, "Hz", 0.0)),
    )


def snapshot(can_name: str, settle: float) -> PiperSettingsSnapshot:
    piper = connect(can_name)
    try:
        time.sleep(settle)

        # Active enquiries. These request feedback but do not control motion.
        for query in (0x01, 0x02, 0x04):
            piper.ArmParamEnquiryAndConfig(query, 0x00, 0x00, 0x00, 0x03)
            time.sleep(0.1)

        time.sleep(settle)

        status = piper.GetArmStatus()
        joint_positions, joint_hz = joint_feedback_values(piper)
        ctrl_positions, ctrl_hz = joint_control_values(piper)
        mode_ctrl = piper.GetArmModeCtrl()
        ctrl_151 = piper.GetArmCtrlCode151()
        end_vel_acc = piper.GetCurrentEndVelAndAccParam()
        crash = piper.GetCrashProtectionLevelFeedback()
        gripper_teaching = piper.GetGripperTeachingPendantParamFeedback()

        arm_status = status.arm_status
        return PiperSettingsSnapshot(
            port=can_name,
            status_hz=float(getattr(status, "Hz", 0.0)),
            joint_hz=joint_hz,
            ctrl_hz=ctrl_hz,
            control_mode=str(getattr(arm_status, "ctrl_mode", "")),
            arm_status=str(getattr(arm_status, "arm_status", "")),
            mode_feed=str(getattr(arm_status, "mode_feed", "")),
            teach_status=str(getattr(arm_status, "teach_status", "")),
            motion_status=str(getattr(arm_status, "motion_status", "")),
            error_code=int(getattr(arm_status, "err_code", 0)),
            joint_positions_mdeg=joint_positions,
            ctrl_positions_mdeg=ctrl_positions,
            mode_ctrl=str(mode_ctrl),
            ctrl_151=str(ctrl_151),
            end_vel_acc=str(end_vel_acc),
            crash_protection=str(crash),
            gripper_teaching=str(gripper_teaching),
            enable_status=[bool(x) for x in piper.GetArmEnableStatus()],
        )
    finally:
        piper.DisconnectPort()


def print_snapshot(snap: PiperSettingsSnapshot) -> None:
    print(f"\n===== {snap.port} =====")
    print(f"status_hz={snap.status_hz:.1f} joint_hz={snap.joint_hz:.1f} ctrl_hz={snap.ctrl_hz:.1f}")
    print(f"control_mode={snap.control_mode} arm_status={snap.arm_status}")
    print(f"mode_feed={snap.mode_feed} teach_status={snap.teach_status} motion_status={snap.motion_status}")
    print(f"error_code={snap.error_code}")
    print(f"enable_status={snap.enable_status}")
    print(f"joint_positions_mdeg={snap.joint_positions_mdeg}")
    print(f"ctrl_positions_mdeg={snap.ctrl_positions_mdeg}")
    print("\n[mode_ctrl]")
    print(snap.mode_ctrl)
    print("[ctrl_151]")
    print(snap.ctrl_151)
    print("[end_vel_acc]")
    print(snap.end_vel_acc)
    print("[crash_protection]")
    print(snap.crash_protection)
    print("[gripper_teaching]")
    print(snap.gripper_teaching)


def compare(a: PiperSettingsSnapshot, b: PiperSettingsSnapshot) -> None:
    print("\n===== SUMMARY =====")
    keys: list[str] = [
        "status_hz",
        "joint_hz",
        "ctrl_hz",
        "control_mode",
        "arm_status",
        "mode_feed",
        "teach_status",
        "motion_status",
        "error_code",
        "enable_status",
    ]
    da: dict[str, Any] = asdict(a)
    db: dict[str, Any] = asdict(b)
    for key in keys:
        marker = "OK" if da[key] == db[key] else "DIFF"
        print(f"{marker:4} {key}: {a.port}={da[key]} | {b.port}={db[key]}")

    if a.joint_hz <= 0:
        print(f"WARN {a.port} has no joint feedback stream.")
    if b.joint_hz <= 0:
        print(f"WARN {b.port} has no joint feedback stream.")
    if a.ctrl_hz <= 0 and b.ctrl_hz <= 0:
        print("NOTE neither arm currently exposes a control-frame stream.")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ports", nargs=2, default=["can0", "can1"])
    parser.add_argument("--settle", type=float, default=0.5)
    args = parser.parse_args()

    snaps = [snapshot(port, args.settle) for port in args.ports]
    for snap in snaps:
        print_snapshot(snap)
    compare(snaps[0], snaps[1])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
