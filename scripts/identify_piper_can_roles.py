#!/usr/bin/env python3
"""Read-only helper to distinguish PiPER leader/follower CAN ports.

This script does not send motion commands. It connects to each CAN interface,
waits for feedback/control frames, and prints the SDK message counters.

Expected pattern from the PiPER SDK docs:
- master/leader arm: mainly sends control frames, visible via GetArmJointCtrl()
  and GetArmGripperCtrl().
- slave/follower arm: mainly sends feedback frames, visible via GetArmJointMsgs()
  and GetArmGripperMsgs().
"""

from __future__ import annotations

import argparse
import time

from piper_sdk import C_PiperInterface_V2


def sample_port(can_name: str, seconds: float) -> None:
    print(f"\n===== {can_name} =====")
    piper = C_PiperInterface_V2(can_name=can_name)
    piper.ConnectPort(can_init=False, piper_init=False, start_thread=True)
    time.sleep(seconds)

    joint_fb = piper.GetArmJointMsgs()
    gripper_fb = piper.GetArmGripperMsgs()
    joint_ctrl = piper.GetArmJointCtrl()
    gripper_ctrl = piper.GetArmGripperCtrl()
    status = piper.GetArmStatus()

    print("[feedback] joint:")
    print(joint_fb)
    print("[feedback] gripper:")
    print(gripper_fb)
    print("[control] joint:")
    print(joint_ctrl)
    print("[control] gripper:")
    print(gripper_ctrl)
    print("[status]:")
    print(status)

    piper.DisconnectPort()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ports", nargs="+", default=["can0", "can1"])
    parser.add_argument("--seconds", type=float, default=2.0)
    args = parser.parse_args()

    for port in args.ports:
        try:
            sample_port(port, args.seconds)
        except Exception as exc:
            print(f"\n===== {port} FAILED =====")
            print(repr(exc))

    print(
        "\nInterpretation hint: if one port has active control messages and the other "
        "has active feedback messages, the control-heavy port is likely the leader/master "
        "and the feedback-heavy port is likely the follower/slave."
    )


if __name__ == "__main__":
    main()
