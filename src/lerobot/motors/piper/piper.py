#!/usr/bin/env python

# =============================================================================
# 文件：PiperMotorsBus — LeRobot 与 PiPER 机械臂硬件之间的翻译器 + 搬运工
# =============================================================================
#
# 【这个文件在整个系统中的位置】
#
#   策略模型输出                     真实机械臂运动
#   [-100, 100] 数组                  物理关节转动
#         │                                ▲
#         │   _unnormalize()              │
#         ├──────────────────────────────►│  piper.JointCtrl()
#         │   [-100,100] → SDK原始整数     │
#         │                                │
#         │   _normalize()                 │
#         ◄──────────────────────────────┤  piper.GetArmJointMsgs()
#             SDK原始整数 → [-100,100]
#
# 【一句话概括】
#   PiperMotorsBus 负责两件事：
#     1. 把 SDK 原始整数（如 50000）翻译成归一化值（如 33.3）→ 给模型/数据集用
#     2. 把归一化值翻译回 SDK 原始整数 → 发给机械臂执行
#
# 【依赖关系】
#   - piper_sdk (C_PiperInterface_V2): AgileX 官方 SDK，负责 CAN 帧收发
#   - wego-piper (PortHandler): 第三方包，只用于 CAN 口的 setup/open/close
#   - .tables: 常量表（电机型号参数、分辨率、初始位姿等）
#   - ..motors_bus: LeRobot 框架的 MotorsBus 基类
#
# Copyright 2025 WeGo-Robotics Inc. EDU team. All rights reserved.
# Licensed under the Apache License, Version 2.0

# ── 标准库导入 ──
import time
import logging
from typing import Any

# ── LeRobot 框架基类和类型 ──
# Motor: 描述一个电机的元数据（id, 型号, 归一化模式）
# MotorCalibration: 一个电机的标定信息（id, 驱动模式, 范围 min/max）
# MotorsBus: 电机总线的抽象基类，提供了 _normalize/_unnormalize 的框架
# MotorNormMode: 枚举，决定归一化到 [-100,100] 还是 [0,100]
from ..motors_bus import Motor, MotorCalibration, MotorsBus, MotorNormMode, NameOrID, Value, get_address

# ── piper_sdk: AgileX 官方 SDK ──
# C_PiperInterface_V2 是主要接口，提供：
#   - ConnectPort()        连接 CAN 口并启动收发线程
#   - EnablePiper()        使能机械臂（上电、保持扭矩）
#   - DisablePiper()       失能机械臂（释放扭矩，手臂变松）
#   - GetArmJointMsgs()    读取反馈关节角度（编码器实际值）
#   - GetArmGripperMsgs()  读取反馈夹爪位置
#   - GetArmJointCtrl()    读取当前控制目标值
#   - GetArmGripperCtrl()  读取当前夹爪控制目标值
#   - GetArmStatus()       读取手臂状态（运动状态、错误码等）
#   - JointCtrl(j1..j6)    发送关节控制命令（6 个目标位置）
#   - GripperCtrl(pos, speed, mode, block) 发送夹爪控制命令
#   - ModeCtrl(...)        设置关节控制模式
#   - MotionCtrl_2(...)    设置运动控制模式和 MIT 标志
#   - MasterSlaveConfig(...) 配置硬件主从模式（你当前没用到）
from piper_sdk import *

# ── wego-piper: 第三方 CAN 口管理 ──
# PortHandler 只做三件事：
#   setupPort(piper)  设置 CAN 口参数
#   openPort()        打开 CAN 口
#   closePort()       关闭 CAN 口
# 等价于帮你调 piper.ConnectPort() 并检查状态
from wego_piper.port_handler import PortHandler

# ── 常量表（定义在 tables.py） ──
# INITIALIZE_POSITION: parking() 回零时的目标位置，如 {"joint1": 0, "joint2": 0, ...}
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


# =============================================================================
# PiperMotorsBus — 核心类
# =============================================================================
class PiperMotorsBus(MotorsBus):
    """
    PiPER 机械臂的电机总线实现。

    继承自 LeRobot 的 MotorsBus 基类，把 PiPER SDK 的原始接口包装成
    LeRobot 统一的"读位置 → 归一化 → 反归一化 → 写位置"接口。

    每只 PiPER 机械臂对应一个 PiperMotorsBus 实例。
    """

    # ── 类属性（会被父类方法引用） ──────────────────────────────────────────

    # PiPER 支持的 CAN 波特率（1M 和 500k）
    available_baudrates = [500_000, 1_000_000]

    # 超时时间（毫秒）
    default_timeout = 1000

    # 从 tables.py 导入的电机参数表
    model_baudrate_table = MODEL_BAUDRATE_TABLE         # 型号→波特率
    model_ctrl_table = MODEL_CONTROL_TABLE               # 控制表定义
    model_encoding_table = MODEL_ENCODING_TABLE           # 编码参数
    model_number_table = MODEL_NUMBER_TABLE               # 型号编号
    model_resolution_table = MODEL_RESOLUTION_TABLE       # 编码器分辨率（DEGREES 模式用）

    # 标记哪些数据字段需要归一化
    # Present_Position = 编码器的当前反馈值
    # Goal_Position     = 下发的目标位置值
    normalized_data = ["Present_Position", "Goal_Position"]

    # PiPER 不需要反转驱动模式（Dynamixel 舵机才需要）
    apply_drive_mode = False

    # =========================================================================
    # __init__ — 初始化
    # =========================================================================
    def __init__(
        self,
        id: str,                                     # 机器人标识，如 "pc_relay_follower"
        port: str,                                   # CAN 口名，如 "can1"
        motors: dict[str, Motor],                    # 电机定义字典
        calibration: dict[str, MotorCalibration] | None = None,  # 标定信息字典
    ):
        """
        创建一个与一只 PiPER 机械臂通信的总线对象。

        参数示例（来自 PiperFollower.__init__）：
          id = "pc_relay_follower"
          port = "can1"
          motors = {
              "joint1":  Motor(1, "AGILEX-M", MotorNormMode.RANGE_M100_100),
              "joint2":  Motor(2, "AGILEX-M", MotorNormMode.RANGE_M100_100),
              ...
              "gripper": Motor(7, "AGILEX-S", MotorNormMode.RANGE_0_100),
          }
          calibration = {
              "joint1":  MotorCalibration(1, 0, 0, -150000, 150000),
              "joint2":  MotorCalibration(2, 0, 0, 0, 180000),
              ...
              "gripper": MotorCalibration(7, 0, 0, 0, 68000),
          }

        关键：Motor 决定归一化到 [-100,100] 还是 [0,100]
             Calibration 决定 SDK 原始值的 min/max 范围
             两者共同决定 _normalize / _unnormalize 的映射公式
        """
        # 调父类构造函数，保存 motors 和 calibration 字典
        super().__init__(port, motors, calibration)

        # PortHandler: 来自 wego-piper，只管 CAN 口的 setup/open/close
        self.port_handler = PortHandler()

        self.id = id

        # ★ 核心对象：piper_sdk 的 CAN 接口
        # 传入 port 参数（如 "can1"），SDK 就知道跟哪个 CAN 口上的机械臂通信
        # 之后所有硬件操作都通过 self.piper 调用
        self.piper = C_PiperInterface_V2(port)

        logger.info(f"{id} : {port} is selected.")

    # =========================================================================
    # 连接 / 断开
    # =========================================================================

    def _assert_protocol_is_compatible(self, instruction_name):
        """PiPER 不需要协议兼容性检查（Dynamixel 舵机才需要），空实现。"""
        pass

    def _handshake(self):
        """PiPER 不需要握手协议，空实现。"""
        pass

    def _find_single_motor(self, motor, initial_baudrate):
        """PiPER 不需要逐电机扫描，空实现。"""
        pass

    def connect(self, handshake: bool = True) -> bool:
        """
        物理连接 CAN 口。

        注意：这里只负责 CAN 通信链路的建立。
        使能电机（上电保持扭矩）是由上层 PiperFollower.connect() 调用 enable_torque() 完成的。
        这是两层分离的设计：
          - Bus 层: 管通信
          - Robot 层: 管电机使能和初始化
        """
        self.port_handler.setupPort(self.piper)   # 设置 CAN 口参数（波特率等）
        return self.port_handler.openPort()        # 打开 CAN 口，返回 True/False

    def disconnect(self, disable_torque: bool = False) -> None:
        """
        断开 CAN 口。

        参数：
          disable_torque: 如果为 True，先回零位再失能（释放扭矩，手臂可以被人推动）
                          如果为 False，直接关 CAN 口（手臂保持当前位置，扭矩不释放）
        """
        if disable_torque:
            self.parking()               # 先回零位（安全位置）
            self.piper.DisablePiper()    # 再失能电机 → 电机松掉，可被人推动

        self.port_handler.closePort()    # 最后关闭 CAN 通信

    # =========================================================================
    # parking — 让机械臂回到预定义的初始安全位置
    # =========================================================================
    def clear_gripper(self):
        """张开夹爪（发送位置 0 = 全开）。"""
        self.piper.GripperCtrl(0, 1000, 0x03, 0)

    def parking(self):
        """
        让机械臂回到预定义的"初始姿态"（INITIALIZE_POSITION，定义在 tables.py）。

        机制：
          每 100ms 发送一次初始化位置，直到 GetArmStatus() 报告运动完成，
          最多等待 100 个周期（10 秒 = 100 × 100ms）。

        ★ 重要安全提示：
          PiperFollower.connect() 默认会调用 parking()（除非传 calibrate=False）。
          这意味着连接机械臂时它会自动运动到初始位姿！
          如果机械臂当前所在位置和初始位姿之间路径上有障碍物，必须先用
          connect(calibrate=False) 避免自动回零。
        """
        timeout = 100  # 100 个周期 = 最多等 10 秒
        self.set_action(INITIALIZE_POSITION)   # 发送初始位置作为目标
        time.sleep(0.1)
        status = self.piper.GetArmStatus()     # 读取运动状态

        # 如果还在运动中，持续重发目标位置直到停止或超时
        while status.arm_status.motion_status and timeout:
            self.set_action(INITIALIZE_POSITION)  # 重发（确保目标不丢）
            time.sleep(0.1)
            status = self.piper.GetArmStatus()
            timeout -= 1

    # =========================================================================
    # 归一化 / 反归一化（整个系统的数学核心）
    # =========================================================================
    #
    # 【为什么需要归一化】
    #   SDK 读到的关节值是原始整数（如 joint1 的范围是 [-150000, 150000]），
    #   不同关节的范围不一样（有的 ±150000，有的 0~180000，有的 -170000~0）。
    #   如果直接把原始值喂给模型，模型很难学习（各维度量纲不统一）。
    #   所以把所有值映射到统一的 [-100, 100]（关节）或 [0, 100]（夹爪）。
    #
    # 【映射公式】
    #   RANGE_M100_100（关节）:
    #     归一化: norm = ((raw - min) / (max - min)) * 200 - 100
    #     反归一化: raw = round(((norm + 100) / 200) * (max - min) + min)
    #     举例 joint1 [min=-150000, max=150000]:
    #       raw =      0 → norm =   0.0    (中位)
    #       raw = 150000 → norm = 100.0    (正向极限)
    #       raw =-150000 → norm =-100.0    (负向极限)
    #       raw =  75000 → norm =  50.0    (正向半程)
    #
    #   RANGE_0_100（夹爪）:
    #     归一化: norm = ((raw - min) / (max - min)) * 100
    #     反归一化: raw = round((norm / 100) * (max - min) + min)
    #     举例 gripper [min=0, max=68000]:
    #       raw =     0 → norm =   0.0    (全开)
    #       raw = 68000 → norm = 100.0    (全闭)
    #       raw = 34000 → norm =  50.0    (半开)

    def _normalize(self, ids_values: dict[int, int]) -> dict[int, float]:
        """
        归一化：SDK 原始整数 → [-100, 100] 或 [0, 100]

        输入：{1: 50000, 2: 90000, ..., 7: 34000}
              键 = motor id（整数），值 = SDK 原始整数值（关节/夹爪）

        输出：{1: 33.3, 2: 50.0, ..., 7: 50.0}
              键 = motor id（整数），值 = 归一化后的浮点数

        调用者：get_action() — 每帧采集时调用
        """
        if not self.calibration:
            raise RuntimeError(f"{self} has no calibration registered.")

        normalized_values = {}
        for id_, val in ids_values.items():
            motor = id_
            min_ = self.calibration[motor].range_min   # 从标定字典取最小原始值
            max_ = self.calibration[motor].range_max   # 从标定字典取最大原始值
            drive_mode = self.apply_drive_mode and self.calibration[motor].drive_mode
            # PiPER 上 apply_drive_mode 永远是 False，所以 drive_mode 永远是 False

            if max_ == min_:
                raise ValueError(
                    f"Invalid calibration for motor '{motor}': min and max are equal."
                )

            # 【安全钳位】将原始值限制在标定范围内，防止传感器异常导致越界
            bounded_val = min(max_, max(min_, val))

            # ── 分支 1: 关节模式 (joint1 ~ joint6) ──
            # MotorNormMode.RANGE_M100_100 → 映射到 [-100, 100]
            if self.motors[motor].norm_mode is MotorNormMode.RANGE_M100_100:
                # 核心公式: norm = ((raw - min) / (max - min)) * 200 - 100
                # 解释：
                #   (raw - min) / (max - min)  → 将 raw 映射到 [0, 1]
                #   * 200                        → 拉伸到 [0, 200]
                #   - 100                        → 平移到 [-100, 100]
                norm = (((bounded_val - min_) / (max_ - min_)) * 200) - 100
                normalized_values[id_] = -norm if drive_mode else norm

            # ── 分支 2: 夹爪模式 (gripper) ──
            # MotorNormMode.RANGE_0_100 → 映射到 [0, 100]
            elif self.motors[motor].norm_mode is MotorNormMode.RANGE_0_100:
                # 核心公式: norm = ((raw - min) / (max - min)) * 100
                norm = ((bounded_val - min_) / (max_ - min_)) * 100
                normalized_values[id_] = 100 - norm if drive_mode else norm

            # ── 分支 3: 角度制模式（PiPER 当前不使用） ──
            elif self.motors[motor].norm_mode is MotorNormMode.DEGREES:
                mid = (min_ + max_) / 2
                max_res = self.model_resolution_table[self._id_to_model(id_)] - 1
                normalized_values[id_] = (val - mid) * 360 / max_res

            else:
                raise NotImplementedError

        return normalized_values

    def _unnormalize(self, ids_values: dict[int, float]) -> dict[int, int]:
        """
        反归一化：[-100, 100] / [0, 100] → SDK 原始整数

        输入：{1: 42.5, 2: -10.3, ..., 7: 75.0}
              键 = motor id，值 = 归一化浮点数（通常来自策略模型输出）

        输出：{1: 63750, 2: -15450, ..., 7: 51000}
              键 = motor id，值 = SDK 原始整数（直接发给 JointCtrl/GripperCtrl）

        调用者：set_action() — 每次发送动作时调用

        ★ 这里也做了安全钳位：bounded_val 确保归一化值不超出 [-100,100]/[0,100]
        """
        if not self.calibration:
            raise RuntimeError(f"{self} has no calibration registered.")

        unnormalized_values = {}
        for id_, val in ids_values.items():
            motor = id_
            min_ = self.calibration[motor].range_min
            max_ = self.calibration[motor].range_max
            drive_mode = self.apply_drive_mode and self.calibration[motor].drive_mode

            if max_ == min_:
                raise ValueError(
                    f"Invalid calibration for motor '{motor}': min and max are equal."
                )

            # ── 分支 1: 关节 ([-100, 100] → 原始整数) ──
            if self.motors[motor].norm_mode is MotorNormMode.RANGE_M100_100:
                val = -val if drive_mode else val
                # ★ 第一层安全限幅：确保模型输出不超 [-100, 100]
                bounded_val = min(100.0, max(-100.0, val))
                # 逆公式: raw = round(((norm + 100) / 200) * (max - min) + min)
                # 解释：
                #   (norm + 100) / 200  → 将 [-100,100] 映射回 [0, 1]
                #   * (max - min)       → 拉伸到原始范围
                #   + min               → 平移回原始坐标系
                unnormalized_values[id_] = int(
                    ((bounded_val + 100) / 200) * (max_ - min_) + min_
                )

            # ── 分支 2: 夹爪 ([0, 100] → 原始整数) ──
            elif self.motors[motor].norm_mode is MotorNormMode.RANGE_0_100:
                val = 100 - val if drive_mode else val
                # ★ 第一层安全限幅：确保夹爪值不超 [0, 100]
                bounded_val = min(100.0, max(0.0, val))
                # 逆公式: raw = round((norm / 100) * (max - min) + min)
                unnormalized_values[id_] = int(
                    (bounded_val / 100) * (max_ - min_) + min_
                )

            # ── 分支 3: 角度制 ──
            elif self.motors[motor].norm_mode is MotorNormMode.DEGREES:
                mid = (min_ + max_) / 2
                max_res = self.model_resolution_table[self._id_to_model(id_)] - 1
                unnormalized_values[id_] = int((val * max_res / 360) + mid)

            else:
                raise NotImplementedError

        return unnormalized_values

    # =========================================================================
    # 写标定（PiPER 上不需要持久化写标定到硬件，空实现）
    # =========================================================================
    def write_calibration(
        self, calibration_dict: dict[str, MotorCalibration], cache: bool = True
    ) -> None:
        pass

    # =========================================================================
    # 使能 / 失能扭矩
    # =========================================================================

    def enable_torque(
        self, motors: str | list[str] | None = None, num_retry: int = 0
    ) -> bool:
        """
        使能机械臂电机（上电、保持扭矩）。

        通过 CAN 发送 EnablePiper 命令，告诉电机"我要开始控制你了"。
        使能后，电机处于伺服锁定状态，不能被人手轻易推动。

        ★ 如果无法使能：
          1. 检查 CAN 口是否正确 UP
          2. 检查机械臂是否已上电
          3. 检查是否被其他进程占用

        返回 True 表示使能成功，False 表示 10 次重试都失败。
        """
        retry = 10
        while not self.piper.EnablePiper() and retry:
            retry -= 1
            time.sleep(0.1)       # 每次重试间隔 100ms
        logger.info(f"{self.piper.GetArmEnableStatus()}")
        if not retry:
            return False           # 10 次全失败
        logger.info(f"{self.id} torque on.")
        return True

    def disable_torque(
        self, motors: int | str | list[str] | None = None, num_retry: int = 0
    ) -> None:
        """
        失能机械臂电机（释放扭矩）。

        使能后电机松掉，人手可以推动机械臂。
        ★ 使能前务必确保机械臂有支撑，否则会因重力下坠！
        """
        while self.piper.DisablePiper() and num_retry:
            num_retry -= 1
            time.sleep(0.01)

    def _disable_torque(self, motor, model, num_retry):
        """单个电机失能（PiPER 不支持逐电机操作，但保留接口）。"""
        while self.piper.DisableArm(motor) and num_retry:
            num_retry -= 1
            time.sleep(0.01)

    # =========================================================================
    # 读操作
    # =========================================================================

    def get_action(self) -> dict[str, Any]:
        """
        读取机械臂当前关节状态并归一化。

        这是数据采集和策略推理时最频繁调用的方法。每帧都调一次。

        流程：
          1. 通过 CAN 总线读取 6 个关节 + 1 个夹爪的编码器反馈值（SDK 原始整数）
          2. 调 _normalize() 把原始值映射到 [-100,100]（关节）或 [0,100]（夹爪）

        返回示例：
          {"joint1": 33.3, "joint2": 50.0, "joint3": -20.1,
           "joint4": 0.0, "joint5": 15.7, "joint6": -5.2,
           "gripper": 50.0}

        调用者：
          - PiperFollower.get_observation() → 数据集记录
          - PiperFollower.send_action() → 读取当前位置做安全限幅
        """
        # ① 从 SDK 读取原始反馈值
        msg_joint = self.piper.GetArmJointMsgs()       # 读关节编码器
        msg_gripr = self.piper.GetArmGripperMsgs()     # 读夹爪编码器

        # ② 从消息对象中提取各字段
        #    joint_state.joint_1 ~ joint_6: int 类型，SDK 原始值
        #    gripper_state.grippers_angle: int 类型，SDK 原始值
        rlt = {
            "joint1":  float(msg_joint.joint_state.joint_1),
            "joint2":  float(msg_joint.joint_state.joint_2),
            "joint3":  float(msg_joint.joint_state.joint_3),
            "joint4":  float(msg_joint.joint_state.joint_4),
            "joint5":  float(msg_joint.joint_state.joint_5),
            "joint6":  float(msg_joint.joint_state.joint_6),
            "gripper": float(msg_gripr.gripper_state.grippers_angle),
        }

        # ③ 归一化: 原始整数 → [-100, 100] / [0, 100]
        rlt = self._normalize(rlt)
        return rlt

    def get_control(self) -> dict[str, Any]:
        """
        读取当前控制目标值（不是编码器反馈值）。

        和 get_action() 的区别：
          - get_action() 读的是"机械臂实际在什么位置"（编码器反馈）
          - get_control() 读的是"上次发给机械臂的目标是什么"（控制值）

        返回的是原始整数，不做归一化。
        主要用于 set_action() 的返回值，确认命令已发出。

        ★ 注意：在 PC 中继遥操作中，piper_pc_relay_teleop.py 读主臂用的是
          GetArmJointCtrl() 而不是 GetArmJointMsgs()，因为主臂处于示教模式，
          GetArmJointMsgs() 可能没有反馈数据流。
        """
        msg_joint = self.piper.GetArmJointCtrl()
        msg_gripr = self.piper.GetArmGripperCtrl()
        rlt = {
            "joint1":  msg_joint.joint_ctrl.joint_1,
            "joint2":  msg_joint.joint_ctrl.joint_2,
            "joint3":  msg_joint.joint_ctrl.joint_3,
            "joint4":  msg_joint.joint_ctrl.joint_4,
            "joint5":  msg_joint.joint_ctrl.joint_5,
            "joint6":  msg_joint.joint_ctrl.joint_6,
            "gripper": msg_gripr.gripper_ctrl.grippers_angle,
        }
        return rlt

    # =========================================================================
    # 写操作：set_action — 把策略/模型输出发给真实机械臂
    # =========================================================================
    def set_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """
        发送关节和夹爪命令给机械臂。

        这是策略部署时最核心的方法。每帧调用一次（由 PiperFollower.send_action() 调用）。

        参数：
          action: 归一化值字典，形如
            {"joint1": 42.5, "joint2": -10.3, ..., "gripper": 75.0}

        流程：
          ① _unnormalize：归一化值 → SDK 原始整数
          ② ModeCtrl：设置关节位置控制模式
          ③ JointCtrl：发送 6 个关节的目标位置
          ④ GripperCtrl：发送夹爪目标位置
          ⑤ 返回 get_control() 确认命令已发出

        ★ 返回的是"当前控制目标"，不是"实际到达位置"。
          机械臂异步执行，实际到位需要时间。
        """
        # ① 反归一化: [-100,100]/[0,100] → SDK 原始整数
        #    如 joint1: 42.5 → 63750
        action_denormalzed = self._unnormalize(action)

        # ② 设置控制模式
        #    ModeCtrl(0x01, 0x01, 30, 0x00)
        #    参数含义（基于 piper_sdk 文档）：
        #      0x01 = 关节空间位置控制（Joint Position Control）
        #      0x01 = 在线控制模式
        #      30   = 速度百分比 30%（★ 安全考虑，比 relay 的 100% 慢）
        #      0x00 = 不使用 MIT 模式（普通位置控制）
        self.piper.ModeCtrl(0x01, 0x01, 30, 0x00)

        # ③ 发送关节目标位置（一次 CAN 消息包含全部 6 个关节）
        self.piper.JointCtrl(
            int(action_denormalzed["joint1"]),
            int(action_denormalzed["joint2"]),
            int(action_denormalzed["joint3"]),
            int(action_denormalzed["joint4"]),
            int(action_denormalzed["joint5"]),
            int(action_denormalzed["joint6"]),
        )

        # ④ 发送夹爪目标位置
        #    abs(...): 确保是正数（夹爪位置不能为负）
        #    1000:     速度参数
        #    0x03:     夹爪控制模式
        #    0:        非阻塞（不等待夹爪到达）
        self.piper.GripperCtrl(
            abs(int(action_denormalzed["gripper"])), 1000, 0x03, 0
        )

        # ⑤ 返回实际发出的命令值（用于日志/调试）
        return self.get_control()

    # =========================================================================
    # 硬件主从模式配置（你当前没用到）
    # =========================================================================
    #
    # PiPER SDK 支持硬件主从模式：两只臂连在同一 CAN 总线上，
    # 一只发命令、一只收命令。这是官方推荐的双臂方案。
    #
    # 你的实际方案是 PC 中继：
    #   两只臂各连一条独立 CAN（can0, can1）
    #   电脑从 can0 读主臂状态 → 通过 SDK 向 can1 发送从臂命令
    # 所以这两个方法虽然存在但当前未被调用。
    #
    # 如果以后想切换到硬件主从：
    #   1. 把两只臂的 CAN 线接到同一个 USB-CAN 模块
    #   2. 先后上电（先从臂后主臂）
    #   3. 调 set_slave() 和 set_master()
    #   4. 之后只需要读主臂状态，从臂会自动跟随

    def set_slave(self):
        """将本臂设置为从模式：接收并执行主臂的控制命令。"""
        self.piper.MasterSlaveConfig(0xFC, 0, 0, 0)

    def set_master(self):
        """将本臂设置为主模式：发送控制命令给从臂。"""
        self.piper.MasterSlaveConfig(0xFA, 0, 0, 0)

    # =========================================================================
    # 以下方法在 PiPER 上都是空实现（Dynamixel 舵机才需要这些）
    # =========================================================================

    def _get_half_turn_homings(self, positions):
        """PiPER 不需要半圈回零，空实现。"""
        pass

    def _encode_sign(
        self, data_name: str, ids_values: dict[int, int]
    ) -> dict[int, int]:
        """PiPER 不需要符号编码转换，直接返回原值。"""
        return ids_values

    def _decode_sign(
        self, data_name: str, ids_values: dict[int, int]
    ) -> dict[int, int]:
        """PiPER 不需要符号解码转换，直接返回原值。"""
        return ids_values

    def _split_into_byte_chunks(self, value, length):
        """PiPER 不需要字节拆分（SDK 内部处理 CAN 帧）。"""
        pass

    def broadcast_ping(
        self, num_retry: int = 0, raise_on_error: bool = False
    ) -> dict[int, int] | None:
        """PiPER 不需要广播 ping。"""
        pass

    def configure_motors(self) -> None:
        """PiPER 不需要额外电机配置。"""
        pass

    def read_calibration(self) -> dict[str, MotorCalibration]:
        """
        从硬件读取标定值。

        PiPER 的标定值硬编码在 PiperFollower.__init__ 中
        （每个关节的 range_min/range_max 是固定的），不需要从硬件读取。
        """
        pass

    @property
    def is_calibrated(self) -> bool:
        """PiPER 的标定是静态定义的，始终返回 True。"""
        return True

    # write_calibration 在上面已经定义过（空实现），
    # 这里重复定义也是一样的空实现
    def write_calibration(
        self, calibration_dict: dict[str, MotorCalibration], cache: bool = True
    ) -> None:
        pass


# =============================================================================
# 如果直接运行本文件：一个最小示例，展示 piper_sdk 的基本用法
# =============================================================================
if __name__ == "__main__":
    # 这是一个独立的最小示例，不依赖 LeRobot 框架。
    # 直接运行本文件可以快速验证 piper_sdk 是否能正常连接和读取。

    # C_PiperInterface 是 V1 版本的接口（PiperMotorsBus 中用 V2）
    # 参数说明（以下是默认值）：
    #   can_name(str):              CAN 口名称，如 "can0"
    #   judge_flag(bool):           是否校验 CAN 模块为正品，设为 False 跳过
    #   can_auto_init(bool):        是否在构造时自动初始化 CAN，True
    #   dh_is_offset(0|1):          DH 参数版本，1=新版 (S-V1.6-3 后的固件)
    #   start_sdk_joint_limit:      是否启用 SDK 关节限位，False
    #   start_sdk_gripper_limit:    是否启用 SDK 夹爪限位，False
    #   logger_level:               日志级别，WARNING 只用告警级别
    #   log_to_file:                是否写日志到文件，False
    #   log_file_path:              日志文件路径，None
    piper = C_PiperInterface(
        can_name="can0",
        judge_flag=False,               # 跳过 CAN 模块校验
        can_auto_init=True,             # 构造时自动初始化 CAN
        dh_is_offset=1,                 # 新版 DH 参数
        start_sdk_joint_limit=False,    # 不禁用 SDK 关节限位
        start_sdk_gripper_limit=False,  # 不禁用 SDK 夹爪限位
        logger_level=LogLevel.WARNING,
        log_to_file=False,
        log_file_path=None,
    )

    # ConnectPort 会启动 CAN 收发线程
    # ★ 注意：第一帧数据是默认值（全 0），之后才是真实值
    piper.ConnectPort()

    # 200Hz 循环打印关节状态
    # GetArmJointMsgs() 返回包含所有关节反馈和状态信息的完整消息对象
    while True:
        print(piper.GetArmJointMsgs())
        time.sleep(0.005)  # 5ms = 200Hz
