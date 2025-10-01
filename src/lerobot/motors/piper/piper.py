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


from ..motors_bus import Motor, MotorCalibration, MotorsBus, MotorNormMode, NameOrID, Value, get_address

import time
import logging
from typing import Any
# Import piper_sdk module
from piper_sdk import *
from wego_piper.port_handler import PortHandler
from .tables import (
    AVAILABLE_BAUDRATES,
    MODEL_BAUDRATE_TABLE,
    MODEL_CONTROL_TABLE,
    MODEL_ENCODING_TABLE,
    MODEL_NUMBER_TABLE,
    MODEL_RESOLUTION_TABLE,
    INITIALIZE_POSITION,
)

logger = logging.getLogger(__name__)


class PiperMotorsBus(MotorsBus):

    available_baudrates = [500_000, 1_000_000]
    default_timeout = 1000
    model_baudrate_table = MODEL_BAUDRATE_TABLE
    model_ctrl_table = MODEL_CONTROL_TABLE
    model_encoding_table = MODEL_ENCODING_TABLE
    model_number_table = MODEL_NUMBER_TABLE
    model_resolution_table = MODEL_RESOLUTION_TABLE
    normalized_data = ["Present_Position", "Goal_Position"]
    apply_drive_mode = False

    def __init__(
        self,
        id: str,
        port: str,
        motors: dict[str, Motor],
        calibration: dict[str, MotorCalibration] | None = None,
    ):
        super().__init__(port, motors, calibration)

        self.port_handler = PortHandler()
        self.id = id
        self.piper = C_PiperInterface_V2(port)
        logger.info(f"{id} : {port} is selected.")




    def _assert_protocol_is_compatible(self, instruction_name):
        pass

    def _handshake(self):
        pass
    
    def _find_single_motor(self, motor, initial_baudrate):
        pass

    def connect(self, handshake: bool = True) -> bool:
        self.port_handler.setupPort(self.piper)
        return self.port_handler.openPort()
    
    def clear_gripper(self):
        self.piper.GripperCtrl(0, 1000, 0x03, 0)

    def parking(self):
        timeout = 100 # 10sec
        self.set_action(INITIALIZE_POSITION)
        time.sleep(0.1)
        status = self.piper.GetArmStatus()

        while (status.arm_status.motion_status and timeout):
            self.set_action(INITIALIZE_POSITION)
            time.sleep(0.1)
            status = self.piper.GetArmStatus()
            timeout -= 1

    def disconnect(self, disable_torque: bool = False) -> None:
        if disable_torque:
            self.parking()
            self.piper.DisablePiper()

        self.port_handler.closePort()

    def write_calibration(self, calibration_dict: dict[str, MotorCalibration], cache: bool = True) -> None:
        pass

    def disable_torque(self, motors: int | str | list[str] | None = None, num_retry: int = 0) -> None:
        while(self.piper.DisablePiper() and num_retry):
            num_retry -= 1
            time.sleep(0.01)

    def _disable_torque(self, motor, model, num_retry):
        while(self.piper.DisableArm(motor) and num_retry):
            num_retry -= 1
            time.sleep(0.01)

    def _normalize(self, ids_values: dict[int, int]) -> dict[int, float]:
        if not self.calibration:
            raise RuntimeError(f"{self} has no calibration registered.")

        normalized_values = {}
        for id_, val in ids_values.items():
            # motor = self._id_to_name(id_)
            motor = id_
            min_ = self.calibration[motor].range_min
            max_ = self.calibration[motor].range_max
            drive_mode = self.apply_drive_mode and self.calibration[motor].drive_mode
            if max_ == min_:
                raise ValueError(f"Invalid calibration for motor '{motor}': min and max are equal.")

            bounded_val = min(max_, max(min_, val))
            if self.motors[motor].norm_mode is MotorNormMode.RANGE_M100_100:
                norm = (((bounded_val - min_) / (max_ - min_)) * 200) - 100
                normalized_values[id_] = -norm if drive_mode else norm
            elif self.motors[motor].norm_mode is MotorNormMode.RANGE_0_100:
                norm = ((bounded_val - min_) / (max_ - min_)) * 100
                normalized_values[id_] = 100 - norm if drive_mode else norm
            elif self.motors[motor].norm_mode is MotorNormMode.DEGREES:
                mid = (min_ + max_) / 2
                max_res = self.model_resolution_table[self._id_to_model(id_)] - 1
                normalized_values[id_] = (val - mid) * 360 / max_res
            else:
                raise NotImplementedError

        return normalized_values

    def _unnormalize(self, ids_values: dict[int, float]) -> dict[int, int]:
        if not self.calibration:
            raise RuntimeError(f"{self} has no calibration registered.")

        unnormalized_values = {}
        for id_, val in ids_values.items():
            motor = id_
            min_ = self.calibration[motor].range_min
            max_ = self.calibration[motor].range_max
            drive_mode = self.apply_drive_mode and self.calibration[motor].drive_mode
            if max_ == min_:
                raise ValueError(f"Invalid calibration for motor '{motor}': min and max are equal.")

            if self.motors[motor].norm_mode is MotorNormMode.RANGE_M100_100:
                val = -val if drive_mode else val
                bounded_val = min(100.0, max(-100.0, val))
                unnormalized_values[id_] = int(((bounded_val + 100) / 200) * (max_ - min_) + min_)
            elif self.motors[motor].norm_mode is MotorNormMode.RANGE_0_100:
                val = 100 - val if drive_mode else val
                bounded_val = min(100.0, max(0.0, val))
                unnormalized_values[id_] = int((bounded_val / 100) * (max_ - min_) + min_)
            elif self.motors[motor].norm_mode is MotorNormMode.DEGREES:
                mid = (min_ + max_) / 2
                max_res = self.model_resolution_table[self._id_to_model(id_)] - 1
                unnormalized_values[id_] = int((val * max_res / 360) + mid)
            else:
                raise NotImplementedError

        return unnormalized_values

    def enable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> bool:
        retry = 10
        while( not self.piper.EnablePiper() and retry):
            retry -= 1
            time.sleep(0.1)
        logger.info(f"{self.piper.GetArmEnableStatus()}")
        if not retry:
            return False
        logger.info(f"{self.id} torque on.")
        return True

    def get_action(self) -> dict[str, Any]:
        msg_joint = self.piper.GetArmJointMsgs()
        msg_gripr = self.piper.GetArmGripperMsgs()
        rlt = {
            "joint1"  : float(msg_joint.joint_state.joint_1),
            "joint2"  : float(msg_joint.joint_state.joint_2),
            "joint3"  : float(msg_joint.joint_state.joint_3),
            "joint4"  : float(msg_joint.joint_state.joint_4),
            "joint5"  : float(msg_joint.joint_state.joint_5),
            "joint6"  : float(msg_joint.joint_state.joint_6),
            "gripper" : float(msg_gripr.gripper_state.grippers_angle),
        }
        rlt = self._normalize(rlt)
        return rlt
    
    def get_control(self) -> dict[str :Any]:
        msg_joint = self.piper.GetArmJointCtrl()
        msg_gripr = self.piper.GetArmGripperCtrl()
        rlt = {
            "joint1"  : msg_joint.joint_ctrl.joint_1,
            "joint2"  : msg_joint.joint_ctrl.joint_2,
            "joint3"  : msg_joint.joint_ctrl.joint_3,
            "joint4"  : msg_joint.joint_ctrl.joint_4,
            "joint5"  : msg_joint.joint_ctrl.joint_5,
            "joint6"  : msg_joint.joint_ctrl.joint_6,
            "gripper" : msg_gripr.gripper_ctrl.grippers_angle,
        }
        return rlt

    def set_action(self, action : dict[str, Any]) -> dict[str, Any]:
        action_denormalzed = self._unnormalize(action)
        self.piper.ModeCtrl(0x01, 0x01, 30, 0x00)
        self.piper.JointCtrl( 
            int(action_denormalzed["joint1"]), 
            int(action_denormalzed["joint2"]), 
            int(action_denormalzed["joint3"]), 
            int(action_denormalzed["joint4"]),
            int(action_denormalzed["joint5"]),
            int(action_denormalzed["joint6"]),
        )
        self.piper.GripperCtrl(abs(int(action_denormalzed["gripper"])), 1000, 0x03, 0)
        return self.get_control()

    def _get_half_turn_homings(self, positions):
        pass

    def _encode_sign(self, data_name: str, ids_values: dict[int, int]) -> dict[int, int]:
        return ids_values

    def _decode_sign(self, data_name: str, ids_values: dict[int, int]) -> dict[int, int]:
        return ids_values
    
    def _split_into_byte_chunks(self, value, length):
        pass

    def write_calibration(self, calibration_dict: dict[str, MotorCalibration], cache: bool = True) -> None:
        pass

    @property
    def is_calibrated(self) -> bool:
        return True
    
    def set_slave(self):
        self.piper.MasterSlaveConfig(0xFC, 0, 0, 0)

    def set_master(self):
        self.piper.MasterSlaveConfig(0xFA, 0, 0, 0)

    def broadcast_ping(self, num_retry: int = 0, raise_on_error: bool = False) -> dict[int, int] | None:
        pass

    def configure_motors(self) -> None:
        pass

    def read_calibration(self) -> dict[str, MotorCalibration]:
        pass


if __name__ == "__main__":
    # Instantiate interface, the default parameters of the parameters are as follows
    #   can_name(str): can port name
    #   judge_flag(bool): Whether to enable the can module when creating this instance. 
    #                     If you use an unofficial module, please set it to False
    #   can_auto_init(bool): Whether to automatically initialize to open the can bus when creating this instance. 
    #                        If set to False, please set the can_init parameter to True in the ConnectPort parameter
    #   dh_is_offset([0,1] -> default 0x01): Whether the dh parameter used is the new version of dh or the old version of dh. 
    #                                       The old version is before S-V1.6-3, and the new version is after S-V1.6-3 firmware
    #           0 -> old
    #           1 -> new
    #   start_sdk_joint_limit(bool -> False): Whether to enable SDK joint angle limit, which will limit both feedback and control messages
    #   start_sdk_gripper_limit(bool -> False): Whether to enable SDK gripper position limit, which will limit both feedback and control messages
    #   logger_level(LogLevel -> default LogLevel.WARNING): Set the log level
    #         The following parameters are optional: 
    #               LogLevel.DEBUG
    #               LogLevel.INFO
    #               LogLevel.WARNING
    #               LogLevel.ERROR
    #               LogLevel.CRITICAL
    #               LogLevel.SILENT
    #   log_to_file(bool -> default False): Whether to enable the log writing function, True to enable, default to disable
    #   log_file_path(str -> default False): Set the path to write the log file, the default is the log folder under the sdk path
    piper = C_PiperInterface(can_name="can0",
                                judge_flag=False,
                                can_auto_init=True,
                                dh_is_offset=1,
                                start_sdk_joint_limit=False,
                                start_sdk_gripper_limit=False,
                                logger_level=LogLevel.WARNING,
                                log_to_file=False,
                                log_file_path=None)
    # Enable can send and receive threads
    piper.ConnectPort()
    # Loop and print messages. Note that the first frame of all messages is the default value. For example, the message content of the first frame of the joint angle message defaults to 0
    while True:
        print(piper.GetArmJointMsgs())
        time.sleep(0.005)# 200hz