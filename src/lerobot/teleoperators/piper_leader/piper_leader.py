#!/usr/bin/env python

# Copyright 2025 WeGo-Robotics Inc. EDU team. All rights reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import builtins
from pathlib import Path
from typing import Any
import logging
import time

import draccus

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.constants import HF_LEROBOT_CALIBRATION, TELEOPERATORS
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.piper import PiperMotorsBus

from ..teleoperator import Teleoperator
from .config_piper_leader import PipperLeaderConfig

logger = logging.getLogger(__name__)

class PiperLeader(Teleoperator):

    config_class = PipperLeaderConfig
    name = "piper_leader"

    def __init__(self, config: PipperLeaderConfig):
        self.id = config.id
        self.bus = PiperMotorsBus(
            port=self.config.port,
            motors={
                "joint1": Motor(1, "HTDW-5047", MotorNormMode.RANGE_M100_100),
                "joint2": Motor(2, "HTDW-5047", MotorNormMode.RANGE_M100_100),
                "joint3": Motor(3, "HTDW-5047", MotorNormMode.RANGE_M100_100),
                "joint4": Motor(4, "HTDW-5047", MotorNormMode.RANGE_M100_100),
                "joint5": Motor(5, "HTDW-5047", MotorNormMode.RANGE_M100_100),
                "joint6": Motor(6, "HTDW-5047", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(7, "HTDW-5047", MotorNormMode.RANGE_0_100),
            }
        )

    def __str__(self) -> str:
        return f"{self.id} {self.__class__.__name__}"

    @property
    def action_features(self) -> dict:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        self.bus.enable_torque()

    @property
    def is_calibrated(self) -> bool:
        pass

    def calibrate(self) -> None:
        pass

    def _load_calibration(self, fpath: Path | None = None) -> None:
        pass

    def _save_calibration(self, fpath: Path | None = None) -> None:
        pass

    def configure(self) -> None:
        """
        Apply any one-time or runtime configuration to the teleoperator.
        This may include setting motor parameters, control modes, or initial state.
        """
        pass

    def setup_motors(self) -> None:
        self.bus.connect()
        self.bus.set_slave()

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        return self.bus.get_action()

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """
        Send a feedback action command to the teleoperator.

        Args:
            feedback (dict[str, Any]): Dictionary representing the desired feedback. Its structure should match
                :pymeth:`feedback_features`.

        Returns:
            dict[str, Any]: The action actually sent to the motors potentially clipped or modified, e.g. by
                safety limits on velocity.
        """
        pass

    def disconnect(self) -> None:
        self.bus.disable_torque()
        self.bus.disconnect()
