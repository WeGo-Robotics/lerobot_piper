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
import logging
import time
from pathlib import Path
from functools import cached_property
from typing import Any

import draccus

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.constants import HF_LEROBOT_CALIBRATION, ROBOTS
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorNormMode, MotorCalibration

from ...motors.piper import PiperMotorsBus
from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_piper_follower import PiperFollowerConfig

logger = logging.getLogger(__name__)

class PiperFollower(Robot):

    # Set these in ALL subclasses
    config_class: PiperFollowerConfig
    name = "piper_follower"

    def __init__(self, config: PiperFollowerConfig):
        self.id = config.id
        self.port = config.port
        self.bus = PiperMotorsBus(
            id=config.id,
            port=config.port,
            motors={
                "joint1": Motor(1, "HTDW-5047", MotorNormMode.RANGE_M100_100),
                "joint2": Motor(2, "HTDW-5047", MotorNormMode.RANGE_M100_100),
                "joint3": Motor(3, "HTDW-5047", MotorNormMode.RANGE_M100_100),
                "joint4": Motor(4, "HTDW-5047", MotorNormMode.RANGE_M100_100),
                "joint5": Motor(5, "HTDW-5047", MotorNormMode.RANGE_M100_100),
                "joint6": Motor(6, "HTDW-5047", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(7, "HTDW-5047", MotorNormMode.RANGE_0_100),
            },
            calibration={
                "joint1": MotorCalibration(1, 0, 0, -150000, 150000),
                "joint2": MotorCalibration(2, 0, 0,       0, 180000),
                "joint3": MotorCalibration(3, 0, 0, -170000, 0     ),
                "joint4": MotorCalibration(4, 0, 0, -100000, 100000),
                "joint5": MotorCalibration(5, 0, 0,  -65000, 65000 ),
                "joint6": MotorCalibration(6, 0, 0, -100000, 130000),
                "gripper": MotorCalibration(7, 0, 0, -60000, 0),
            }
        )
        self.cameras = make_cameras_from_configs(config.cameras)

    def __str__(self) -> str:
        return f"{self.id} {self.__class__.__name__}"

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())
        # return self.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        self.bus.connect()
        self.bus.enable_torque()
        logger.info(f"{self} connected.")

        for cam in self.cameras.values():
            cam.connect()

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        return True
    
    def _load_calibration(self, fpath: Path | None = None) -> None:
        pass

    def _save_calibration(self, fpath: Path | None = None) -> None:
        pass

    def configure(self) -> None:
        pass

    def setup_motors(self) -> None:
        self.bus.connect()
        self.bus.set_slave()

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        obs_dict = {}

        # Read arm position
        start = time.perf_counter()
        obs_dict = self.bus.get_action()
        obs_dict = {f"{motor}.pos": val for motor, val in obs_dict.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        rlt = self.bus.set_action(action)

        return {f"{motor}.pos": val for motor, val in rlt.items()}

    def disconnect(self) -> None:
        self.bus.disable_torque()
        self.bus.disconnect()
