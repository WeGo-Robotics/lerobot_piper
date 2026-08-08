# 第3章 PiPER SDK 深度解析

本章深入剖析 PiPER 机械臂 Python SDK 的软件架构、通信协议和核心 API。我们将从硬件入口 `C_PiperInterface_V2` 出发，逐层解读 CAN 通信、线程模型、数据编解码、控制指令发送等关键机制。阅读本章后，你将具备直接通过 SDK 底层接口操控机械臂的能力，为后续的模仿学习数据采集和策略部署奠定坚实基础。

---

## 3.1 C_PiperInterface_V2 概述

### 3.1.1 文件位置与地位

整个 PiPER SDK 的核心入口类为 `C_PiperInterface_V2`，定义在：

```
piper_sdk/piper_sdk/interface/piper_interface_v2.py    (3216 行)
```

这个类是整个软件栈的中枢——所有对机械臂的操作（读关节角度、写目标位置、使能/失能电机、主从配置等）都必须经过这个类。它内部持有三个核心组件，分别负责硬件通信、协议编解码和运动学计算。

### 3.1.2 类架构总览

`C_PiperInterface_V2` 内部包含以下核心成员：

| 成员 | 类型 | 职责 |
|------|------|------|
| `__arm_can` | `C_STD_CAN` | 底层 SocketCAN 硬件端口，负责 CAN 帧的实际收发 |
| `__parser` | `C_PiperParserV2` | CAN 帧编解码器，实现 Python 对象与 8 字节 CAN data 之间的双向转换 |
| `__piper_fk` | `C_PiperForwardKinematics` | 前向运动学求解器，根据 6 个关节角度计算末端位姿及各连杆位姿 |

此外，还有参数管理器 `__piper_param_mag`（`C_PiperParamManager`）用于管理 SDK 层面的软限位参数。

### 3.1.3 单例模式

`C_PiperInterface_V2` 实现了基于 `can_name` 的单例模式，确保同一个 CAN 口只有一个连接实例。其核心机制如下：

```python
_instances = {}   # 类级别字典，以 can_name 为 key 缓存实例
_lock = threading.Lock()

def __new__(cls, can_name="can0", ...):
    key = (can_name)
    with cls._lock:
        if key not in cls._instances:
            instance = super().__new__(cls)
            instance._initialized = False
            cls._instances[key] = instance
    return cls._instances[key]
```

`__init__` 方法通过 `_initialized` 标志位防止重复初始化：

```python
def __init__(self, ...):
    if getattr(self, "_initialized", False):
        return  # 避免重复初始化
    # ... 执行一次性的初始化逻辑
    self._initialized = True
```

**注意**：单例的 key 仅由 `can_name` 决定。如果以相同 `can_name` 但不同其他参数（如 `start_sdk_joint_limit`）再次调用构造函数，将返回已缓存的实例，新参数**不会**生效。这是 SDK 使用中需要特别注意的设计约束。

### 3.1.4 构造参数详解

```python
def __init__(self,
             can_name: str = "can0",           # SocketCAN 接口名
             judge_flag: bool = True,          # 是否在初始化时检查 CAN 口是否正常工作
             can_auto_init: bool = True,       # 是否自动初始化 CAN 口
             dh_is_offset: int = 0x01,         # DH 参数中关节 1-2 是否有 2 度偏置
             start_sdk_joint_limit: bool = False,   # 是否启用 SDK 软限位
             start_sdk_gripper_limit: bool = False, # 是否启用夹爪软限位
             start_sdk_fk_cal: bool = False,        # 是否启用前向运动学计算
             logger_level: LogLevel = LogLevel.WARNING,
             log_to_file: bool = False,
             log_file_path = None)
```

各参数说明：

- **`can_name`**：Linux SocketCAN 接口名称，通常为 `"can0"`。对于多臂系统，可能为 `"can1"` 等。
- **`judge_flag`**：若为 `True`，构造时会调用 `JudgeCanInfo()` 检查 CAN 口是否存在且已 `ip link set up`。当使用 PCIe 转 CAN 模块时，由于硬件枚举时机问题，可能需要设为 `False`。
- **`can_auto_init`**：若为 `True`，构造时自动调用 `C_STD_CAN.Init()` 创建 `can.interface.Bus` 实例（基于 python-can 库）。
- **`dh_is_offset`**：PiPER 机械臂的 DH 参数中，关节 1-2 存在 2 度的装配偏置。`0x01` 表示应用偏置（`_theta` 使用 172.22 和 102.78 度），`0x00` 表示不应用（使用 174.22 和 100.78 度）。
- **`start_sdk_joint_limit`**：启用后，所有读取和写入的关节角度值会被 clamp 到 SDK 参数管理器中的软限位范围。这是一种纯软件层面的安全保护。
- **`start_sdk_gripper_limit`**：与上类似，对夹爪角度值施加软限位。
- **`start_sdk_fk_cal`**：启用后，每次收到 CAN 帧会额外计算前向运动学（6 个关节各自的位姿矩阵）。计算开销较大，仅在需要各连杆位姿时才启用。

---

## 3.2 后台线程详解

### 3.2.1 为什么需要后台线程

PiPER 机械臂的 CAN 通信采用**推模型（push model）**：机械臂主控板以固定频率（约 200Hz）主动向 CAN 总线上报关节角度、末端位姿、驱动器状态等反馈数据。SDK 必须持续从 SocketCAN 读取这些帧，否则内核缓冲区会溢出导致数据丢失。

为此，SDK 在 `ConnectPort()` 中启动两个 daemon 线程。

### 3.2.2 ReadCan 线程

`ReadCan` 是核心读取线程，在 `ConnectPort()` 中以内联函数定义并启动：

```python
def ReadCan():
    self.logger.info("[ReadCan] ReadCan Thread started")
    while not self.__read_can_stop_event.is_set():
        self.__fps_counter.increment("CanMonitor")
        try:
            read_status = self.__arm_can.ReadCanMessage()
        except can.CanOperationError:
            self.logger.error("[ReadCan] CAN is closed, stop ReadCan thread")
            break
        except Exception as e:
            self.logger.error("[ReadCan] 'error: %s'", e)
            break
```

关键流程：

1. **循环条件**：通过 `threading.Event` (`__read_can_stop_event`) 控制线程退出。
2. **FPS 计数**：每次迭代递增 `CanMonitor` 计数器，供帧率监测使用。
3. **`self.__arm_can.ReadCanMessage()`**：调用底层 `C_STD_CAN` 的读取方法。该方法内部调用 `self.bus.recv(timeout=0.001)` 从 SocketCAN 读取一帧。读取成功后，自动调用注册的回调函数 `self.ParseCANFrame`（在 `C_STD_CAN` 构造时通过 `callback_function` 参数传入）。
4. **异常处理**：若 CAN 口被关闭（`CanOperationError`），线程优雅退出。

**线程属性**：`daemon=True`，这意味着主程序退出时该线程会随进程终止，不会阻止程序退出。

### 3.2.3 CanMonitor 线程

`CanMonitor` 是 CAN 帧率监测线程：

```python
def CanMonitor():
    self.logger.info("[ReadCan] CanMonitor Thread started")
    while not self.__can_monitor_stop_event.is_set():
        try:
            self.__CanMonitor()
        except Exception as e:
            self.logger.error("CanMonitor() exception: %s", e)
            break
        self.__can_monitor_stop_event.wait(0.05)  # 每 50ms 检查一次
```

核心逻辑在 `__CanMonitor()` 中：

```python
def __CanMonitor(self):
    if self.__q_can_fps.full():
        self.__q_can_fps.get()          # 队列满时丢弃最旧数据
    self.__q_can_fps.put(self.GetCanFps())  # 压入当前帧率
    with self.__is_ok_mtx:
        if self.__q_can_fps.full() and all(x == 0 for x in self.__q_can_fps.queue):
            self.__is_ok = False    # 连续 5 次帧率均为 0 → CAN 总线卡死
        else:
            self.__is_ok = True
```

机制说明：
- 使用一个大小为 5 的 `Queue` 缓存最近 5 次帧率采样（每次间隔 50ms，共 250ms 窗口）。
- 若队列满且 5 次帧率**全部为零**，判定 CAN 总线卡死（`__is_ok = False`）。
- 上层可通过 `isOk()` 方法查询此状态，决定是否触发错误处理。

### 3.2.4 线程安全机制

SDK 为每个数据缓存配备了独立的 `threading.Lock`：

```python
self.__arm_joint_msgs_mtx = threading.Lock()    # 关节角度缓存锁
self.__arm_gripper_msgs_mtx = threading.Lock()  # 夹爪缓存锁
self.__arm_status_mtx = threading.Lock()        # 状态缓存锁
# ... 共 20+ 个锁
```

这种**细粒度锁**设计的优势：
- ReadCan 线程在回调中写数据时，只锁住正在更新的那个缓存，不影响其他缓存的读写。
- 用户线程调用 getter 方法时，只需获取对应缓存的锁，不会因为其他缓存的更新而阻塞。

---

## 3.3 核心读方法详解

本节详细介绍 `C_PiperInterface_V2` 提供的各类数据读取方法。所有 getter 方法均遵循统一的线程安全模式：获取对应锁 → 更新 Hz 字段 → 返回数据引用。

### 3.3.1 GetArmJointMsgs() — 关节角度反馈

```python
def GetArmJointMsgs(self):
    with self.__arm_joint_msgs_mtx:
        self.__arm_joint_msgs.Hz = self.__fps_counter.cal_average(
            self.__fps_counter.get_fps('ArmJoint_12'),
            self.__fps_counter.get_fps('ArmJoint_34'),
            self.__fps_counter.get_fps('ArmJoint_56'))
        return self.__arm_joint_msgs
```

**返回值类型**：`self.ArmJoint` —— 定义在 `C_PiperInterface_V2` 内部的嵌套类。

```python
class ArmJoint():
    def __init__(self):
        self.time_stamp: float = 0     # epoch 秒级时间戳
        self.Hz: float = 0             # 数据流频率
        self.joint_state = ArmMsgFeedBackJointStates()
```

**`joint_state`** (`ArmMsgFeedBackJointStates`) 的字段：

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `joint_1` | int | 0.001 度（毫度） | 关节 1 当前角度 |
| `joint_2` | int | 0.001 度 | 关节 2 当前角度 |
| `joint_3` | int | 0.001 度 | 关节 3 当前角度 |
| `joint_4` | int | 0.001 度 | 关节 4 当前角度 |
| `joint_5` | int | 0.001 度 | 关节 5 当前角度 |
| `joint_6` | int | 0.001 度 | 关节 6 当前角度 |

**数据来源**：CAN 帧 `0x2A5` (joint_1, joint_2)、`0x2A6` (joint_3, joint_4)、`0x2A7` (joint_5, joint_6)。每帧 8 字节，每关节占 4 字节小端 int32。

**编解码细节**（在 `C_PiperParserV2.DecodeMessage()` 中）：

```python
# 以 0x2A5 为例
msg.arm_joint_feedback.joint_1 = self.ConvertToNegative_32bit(
    self.ConvertBytesToInt(can_data, 0, 4))   # Byte 0-3 → int32
msg.arm_joint_feedback.joint_2 = self.ConvertToNegative_32bit(
    self.ConvertBytesToInt(can_data, 4, 8))   # Byte 4-7 → int32
```

`ConvertToNegative_32bit` 处理有符号整数的补码表示，确保负角度值正确解码。

**使用示例**：

```python
joint_msgs = piper.GetArmJointMsgs()
print(f"关节角度: J1={joint_msgs.joint_state.joint_1 * 0.001:.2f}°")
print(f"数据频率: {joint_msgs.Hz:.1f} Hz")
```

### 3.3.2 GetArmGripperMsgs() — 夹爪反馈

```python
def GetArmGripperMsgs(self):
    with self.__arm_gripper_msgs_mtx:
        self.__arm_gripper_msgs.Hz = self.__fps_counter.get_fps('ArmGripper')
        return self.__arm_gripper_msgs
```

**返回值类型**：`self.ArmGripper`。

```python
class ArmGripper():
    def __init__(self):
        self.time_stamp: float = 0
        self.Hz: float = 0
        self.gripper_state = ArmMsgFeedBackGripper()
```

**`gripper_state`** (`ArmMsgFeedBackGripper`) 的字段：

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `grippers_angle` | int | 0.001 度 | 夹爪当前角度 |
| `grippers_effort` | int | 0.001 N/m | 夹爪当前力矩 |
| `status_code` | int | 位掩码 | 夹爪状态码（见下文） |

**`status_code`** 是一个位掩码字段，通过 `foc_status` 属性提供分解后的布尔标志：

| 位 | 属性名 | 说明 |
|----|--------|------|
| bit[0] | `voltage_too_low` | 电源电压是否过低 |
| bit[1] | `motor_overheating` | 电机是否过温 |
| bit[2] | `driver_overcurrent` | 驱动器是否过流 |
| bit[3] | `driver_overheating` | 驱动器是否过温 |
| bit[4] | `sensor_status` | 传感器状态 |
| bit[5] | `driver_error_status` | 驱动器错误状态 |
| bit[6] | `driver_enable_status` | 驱动器使能状态（1=使能） |
| bit[7] | `homing_status` | 回零状态（1=已回零） |

状态码的设置通过 `@status_code.setter` 自动解析：

```python
@status_code.setter
def status_code(self, value: int):
    self._status_code = value
    self.foc_status.voltage_too_low      = bool(value & (1 << 0))
    self.foc_status.motor_overheating    = bool(value & (1 << 1))
    # ...
```

**数据来源**：CAN 帧 `0x2A8`。

**CAN 帧结构**（共 8 字节）：

| Byte | 内容 | 类型 |
|------|------|------|
| 0-3 | `grippers_angle` | int32，小端 |
| 4-5 | `grippers_effort` | int16，小端 |
| 6 | `status_code` | uint8 |
| 7 | 保留 | — |

### 3.3.3 GetArmJointCtrl() — 控制指令读取（主从模式专用）

**重要概念区分**：`GetArmJointMsgs()` 读取的是机械臂**实际反馈**的关节角度，而 `GetArmJointCtrl()` 读取的是 **CAN 总线上的控制指令帧**。在主从模式下，教学臂（主臂）会主动发送关节控制指令（CAN ID 0x155/0x156/0x157），运动臂（从臂）可以通过此方法**监听**主臂发送的目标角度。这是实现主从跟随的关键机制。

```python
def GetArmJointCtrl(self):
    with self.__arm_joint_ctrl_msgs_mtx:
        self.__arm_joint_ctrl_msgs.Hz = self.__fps_counter.cal_average(
            self.__fps_counter.get_fps('ArmJointCtrl_12'),
            self.__fps_counter.get_fps('ArmJointCtrl_34'),
            self.__fps_counter.get_fps('ArmJointCtrl_56'))
        return self.__arm_joint_ctrl_msgs
```

**返回值类型**：`self.ArmJointCtrl`，其 `joint_ctrl` 字段为 `ArmMsgJointCtrl`，结构同 `ArmMsgFeedBackJointStates`（`joint_1` 到 `joint_6`，单位 0.001 度）。

**数据来源**：CAN 帧 `0x155`、`0x156`、`0x157`。注意这两个 ID 系列在不同场景下的含义：

| CAN ID | 发送方 | 含义 |
|--------|--------|------|
| 0x155/156/157 | 主臂（教学臂） | 主臂发送的控制指令 |
| 0x2A5/2A6/2A7 | 任意臂 | 当前关节的实际反馈角度 |

### 3.3.4 GetArmGripperCtrl() — 夹爪控制指令读取

与 `GetArmJointCtrl()` 类似，读取主臂发送的夹爪控制指令。

```python
def GetArmGripperCtrl(self):
    with self.__arm_gripper_ctrl_msgs_mtx:
        self.__arm_gripper_ctrl_msgs.Hz = self.__fps_counter.get_fps("ArmGripperCtrl")
        return self.__arm_gripper_ctrl_msgs
```

**返回值类型**：`self.ArmGripperCtrl`，其 `gripper_ctrl` 字段为 `ArmMsgGripperCtrl`，包含 `grippers_angle`（int32）、`grippers_effort`（uint16）、`status_code`（uint8）、`set_zero`（uint8）。

**数据来源**：CAN 帧 `0x159`。

### 3.3.5 其他 Getter 方法一览

#### GetArmEndPoseMsgs()
获取机械臂末端位姿（欧拉角表示）。返回 `self.ArmEndPose`。

| 字段 | 单位 |
|------|------|
| `end_pose.X_axis` | 0.001 mm |
| `end_pose.Y_axis` | 0.001 mm |
| `end_pose.Z_axis` | 0.001 mm |
| `end_pose.RX_axis` | 0.001 度 |
| `end_pose.RY_axis` | 0.001 度 |
| `end_pose.RZ_axis` | 0.001 度 |

数据来源：CAN 帧 `0x2A2`/`0x2A3`/`0x2A4`（每帧传 2 个字段）。

#### GetArmStatus()
获取机械臂综合状态。返回 `self.ArmStatus`，其 `arm_status` 字段 (`ArmMsgFeedbackStatus`) 包含：

| 子字段 | 类型 | 说明 |
|--------|------|------|
| `ctrl_mode` | `CtrlMode` 枚举 | 控制模式（待机/CAN指令/示教/以太网/WiFi/遥控/联动输入/离线轨迹） |
| `arm_status` | `ArmStatus` 枚举 | 机械臂状态（正常/急停/无解/奇异点/超限/通信异常/碰撞等） |
| `mode_feed` | `ModeFeed` 枚举 | 当前运动模式（P/J/L/C/M） |
| `teach_status` | `TeachingState` 枚举 | 示教状态 |
| `motion_status` | `MotionStatus` 枚举 | 是否到达目标位置 |
| `err_code` | int (16位) | 故障码（包含各关节超限位和各关节通信异常标志） |

数据来源：CAN 帧 `0x2A1`。

#### GetArmHighSpdInfoMsgs()
获取 6 个关节电机的高速反馈信息。返回 `self.ArmMotorDriverInfoHighSpd`，包含 `motor_1` 到 `motor_6`（每个为 `ArmMsgFeedbackHighSpd` 实例）。

每电机字段：

| 字段 | 说明 |
|------|------|
| `motor_speed` | 电机转速 |
| `current` | 当前电流 |
| `pos` | 当前位置 |
| `effort` | 计算力矩（通过 `cal_effort()` 方法） |

数据来源：CAN 帧 `0x251`-`0x256`。

#### GetArmLowSpdInfoMsgs()
获取 6 个关节电机的低速反馈信息。返回 `self.ArmMotorDriverInfoLowSpd`。

每电机字段：

| 字段 | 单位 | 说明 |
|------|------|------|
| `vol` | 0.1 V | 驱动器电压 |
| `foc_temp` | 1 ℃ | 驱动器温度 |
| `motor_temp` | 1 ℃ | 电机温度 |
| `foc_status_code` | 位掩码 | 驱动器状态（含电源/过温/过流/碰撞/使能/堵转标志） |
| `bus_current` | 0.001 A | 母线电流 |

数据来源：CAN 帧 `0x261`-`0x266`。

**`foc_status` 各 bit 含义**：

| bit | 属性名 | 说明 |
|-----|--------|------|
| 0 | `voltage_too_low` | 电压过低 |
| 1 | `motor_overheating` | 电机过温 |
| 2 | `driver_overcurrent` | 驱动器过流 |
| 3 | `driver_overheating` | 驱动器过温 |
| 4 | `collision_status` | 碰撞保护状态 |
| 5 | `driver_error_status` | 驱动器错误 |
| 6 | `driver_enable_status` | 使能状态（1=已使能） |
| 7 | `stall_status` | 堵转保护状态 |

此方法在 `GetArmEnableStatus()` 中被用于检查当前使能状态。

#### GetFK()
获取前向运动学计算结果。返回 6 个 6 元素列表的列表 `[[x,y,z,rx,ry,rz]_1 ... [x,y,z,rx,ry,rz]_6]`，分别表示 1-6 号关节坐标系相对于 base_link 的位姿。需要 `start_sdk_fk_cal=True` 才会更新。

```python
def GetFK(self, mode: Literal["feedback", "control"] = "feedback"):
    if mode == "feedback":
        with self.__piper_feedback_fk_mtx:
            return self.__link_feedback_fk
    elif mode == "control":
        with self.__piper_ctrl_fk_mtx:
            return self.__link_ctrl_fk
```

- `"feedback"` 模式：基于实际反馈的关节角度计算。
- `"control"` 模式：基于控制指令中的目标关节角度计算（主从模式专用）。

---

## 3.4 核心写方法详解

发送控制指令遵循统一的流程：**构造消息对象 → 编码为 CAN 帧 → 调用 SendCanMessage → 检查发送状态**。

### 3.4.1 JointCtrl() — 关节角度控制

```python
def JointCtrl(self,
              joint_1: int, joint_2: int,
              joint_3: int, joint_4: int,
              joint_5: int, joint_6: int):
```

**参数**：6 个关节的目标角度，单位均为 **0.001 度（毫度）**。例如 `joint_1=45000` 表示目标 45.0 度。

**关节限位参考**：

| 关节 | 最小角度（度） | 最大角度（度） | 对应毫度范围 |
|------|--------------|--------------|-------------|
| Joint 1 | -150.0 | 150.0 | -150000 ~ 150000 |
| Joint 2 | -2.0 | 180.0 | -2000 ~ 180000 |
| Joint 3 | -170.0 | 2.0 | -170000 ~ 2000 |
| Joint 4 | -100.0 | 100.0 | -100000 ~ 100000 |
| Joint 5 | -70.0 | 70.0 | -70000 ~ 70000 |
| Joint 6 | -120.0 | 120.0 | -120000 ~ 120000 |

**内部执行流程**：

1. **SDK 软限位 clamp**：若启用了 `start_sdk_joint_limit`，先通过 `__CalJointSDKLimit()` 将每个角度 clamp 到软限位范围内。
2. **分发到三个私有方法**：
   - `__JointCtrl_12(joint_1, joint_2)` → CAN ID `0x155`
   - `__JointCtrl_34(joint_3, joint_4)` → CAN ID `0x156`
   - `__JointCtrl_56(joint_5, joint_6)` → CAN ID `0x157`
3. **每帧编码格式**（以 `__JointCtrl_12` 为例）：

```python
def __JointCtrl_12(self, joint_1, joint_2):
    tx_can = Message()
    joint_ctrl = ArmMsgJointCtrl(joint_1=joint_1, joint_2=joint_2)
    msg = PiperMessage(type_=ArmMsgType.PiperMsgJointCtrl_12, arm_joint_ctrl=joint_ctrl)
    self.__parser.EncodeMessage(msg, tx_can)   # Python 对象 → 8 字节 CAN data
    feedback = self.__arm_can.SendCanMessage(tx_can.arbitration_id, tx_can.data)
    if feedback is not self.__arm_can.CAN_STATUS.SEND_MESSAGE_SUCCESS:
        self.logger.error("JointCtrl_J12 send failed: SendCanMessage(%s)", feedback)
```

**CAN 帧编码细节**（在 `C_PiperParserV2.EncodeMessage()` 中）：

```python
elif(msg_type_ == ArmMsgType.PiperMsgJointCtrl_12):
    tx_can_frame.data = self.ConvertToList_32bit(msg.arm_joint_ctrl.joint_1) + \
                        self.ConvertToList_32bit(msg.arm_joint_ctrl.joint_2)
```

`ConvertToList_32bit` 将 int32 转为 4 字节小端列表。因此 `0x155` 的 8 字节布局为：

```
[joint_1 byte0] [joint_1 byte1] [joint_1 byte2] [joint_1 byte3]
[joint_2 byte0] [joint_2 byte1] [joint_2 byte2] [joint_2 byte3]
```

**注意事项**：
- `JointCtrl()` 会发送 3 个 CAN 帧（0x155, 0x156, 0x157），每个帧独立发送。
- 发送前必须通过 `ModeCtrl()` 设置为关节控制模式（`move_mode=0x01`）。
- 发送前必须通过 `EnablePiper()` 使能电机。

### 3.4.2 GripperCtrl() — 夹爪控制

```python
def GripperCtrl(self,
                gripper_angle: int = 0,                    # 单位 0.001 度
                gripper_effort: int = 0,                   # 单位 0.001 N/m，范围 0-5000
                gripper_code: Literal[0x00, 0x01, 0x02, 0x03] = 0,
                set_zero: Literal[0x00, 0xAE] = 0):
```

**参数详解**：

| 参数 | 取值 | 含义 |
|------|------|------|
| `gripper_code=0x00` | 失能 | 夹爪电机停止输出力矩 |
| `gripper_code=0x01` | 使能 | 夹爪电机使能 |
| `gripper_code=0x02` | 失能并清除错误 | 先清除错误标志再失能 |
| `gripper_code=0x03` | 使能并清除错误 | 先清除错误标志再使能 |
| `set_zero=0x00` | 无效 | 不执行零点设置 |
| `set_zero=0xAE` | 设零点 | 将当前位置设为夹爪零点 |

**CAN ID**：`0x159`

**CAN 帧编码**（8 字节）：

```python
tx_can_frame.data = self.ConvertToList_32bit(gripper_angle) + \
                    self.ConvertToList_16bit(gripper_effort, False) + \
                    self.ConvertToList_8bit(gripper_code, False) + \
                    self.ConvertToList_8bit(set_zero, False)
```

| Byte | 内容 |
|------|------|
| 0-3 | `gripper_angle` (int32, 小端) |
| 4-5 | `gripper_effort` (uint16, 小端) |
| 6 | `gripper_code` (uint8) |
| 7 | `set_zero` (uint8) |

### 3.4.3 ModeCtrl() — 模式控制

```python
def ModeCtrl(self,
             ctrl_mode: Literal[0x00, 0x01] = 0x01,
             move_mode: Literal[0x00, 0x01, 0x02, 0x03] = 0x01,
             move_spd_rate_ctrl: int = 50,
             is_mit_mode: Literal[0x00, 0xAD, 0xFF] = 0x00):
```

**参数详解**：

| 参数 | 取值 | 含义 |
|------|------|------|
| `ctrl_mode=0x00` | 待机模式 | 机械臂不响应控制指令 |
| `ctrl_mode=0x01` | CAN 指令控制模式 | 通过 CAN 总线发送控制指令 |
| `move_mode=0x00` | MOVE P | 末端位姿控制（笛卡尔空间） |
| `move_mode=0x01` | MOVE J | 关节空间控制 |
| `move_mode=0x02` | MOVE L | 直线路径控制 |
| `move_mode=0x03` | MOVE C | 圆弧路径控制 |
| `move_spd_rate_ctrl` | 0-100 | 运动速度百分比 |
| `is_mit_mode=0x00` | 位置速度模式 | 标准 PID 位置控制 |
| `is_mit_mode=0xAD` | MIT 模式 | 阻抗/导纳控制模式 |

`ModeCtrl()` 实际是 `MotionCtrl_2()` 的简化封装，内部直接调用：

```python
self.MotionCtrl_2(ctrl_mode, move_mode, move_spd_rate_ctrl, is_mit_mode)
```

**CAN ID**：`0x151`

**CAN 帧编码**（8 字节）：

```python
tx_can_frame.data = [ctrl_mode, move_mode, move_spd_rate_ctrl, is_mit_mode, 0x00, 0x00, 0x00, 0x00]
```

### 3.4.4 MasterSlaveConfig() — 主从模式配置

```python
def MasterSlaveConfig(self,
                      linkage_config: int,
                      feedback_offset: int,
                      ctrl_offset: int,
                      linkage_offset: int):
```

**参数详解**：

| 参数 | 取值 | 含义 |
|------|------|------|
| `linkage_config=0xFA` | 示教输入臂 | 该臂作为教学臂，主动发送控制指令 |
| `linkage_config=0xFC` | 运动输出臂 | 该臂作为运动臂，接收并执行控制指令 |
| `feedback_offset=0x00` | — | 反馈 ID 不偏移（基 ID 为 0x2Ax） |
| `feedback_offset=0x10` | — | 反馈 ID 偏移到 0x2Bx |
| `feedback_offset=0x20` | — | 反馈 ID 偏移到 0x2Cx |
| `ctrl_offset=0x00` | — | 控制指令基 ID 不偏移（0x15x） |
| `ctrl_offset=0x10` | — | 控制指令基 ID 偏移到 0x16x |
| `ctrl_offset=0x20` | — | 控制指令基 ID 偏移到 0x17x |
| `linkage_offset=0x00` | — | 联动目标地址不偏移 |
| `linkage_offset=0x10` | — | 联动目标地址偏移到 0x16x |
| `linkage_offset=0x20` | — | 联动目标地址偏移到 0x17x |

**CAN ID**：`0x470`

偏移机制的作用：在多臂系统中，避免不同机械臂的 CAN ID 冲突。教学臂的控制指令（原本 0x155-0x157）在加上偏移后变为 0x165-0x167，运动臂的反馈 ID 也随之偏移。

### 3.4.5 其他写方法一览

#### EndPoseCtrl()
笛卡尔空间末端位姿控制，发送欧拉角位姿指令。内部拆分为 3 帧：

| 调用 | CAN ID | 发送内容 |
|------|--------|---------|
| `__CartesianCtrl_XY(X, Y)` | 0x152 | X, Y 坐标（int32 × 2） |
| `__CartesianCtrl_ZRX(Z, RX)` | 0x153 | Z, RX 坐标（int32 × 2） |
| `__CartesianCtrl_RYRZ(RY, RZ)` | 0x154 | RY, RZ 坐标（int32 × 2） |

使用前需通过 `ModeCtrl(move_mode=0x00)` 切换为 MOVE P 模式。

#### EmergencyStop()
发送急停/恢复指令（CAN ID 0x150）。

```python
def EmergencyStop(self, emergency_stop=0x01):  # 0x01=急停, 0x02=恢复
    self.MotionCtrl_1(emergency_stop, 0x00, 0x00)
```

---

## 3.5 ConnectPort 和初始化流程

### 3.5.1 ConnectPort 完整流程

`ConnectPort()` 是机械臂连接的入口函数，完整的调用链为：

```
ConnectPort(can_init, piper_init, start_thread)
  ├── [can_init 或 首次连接] → __arm_can.Init()
  │     └── C_STD_CAN.Init() → can.interface.Bus(channel, bustype, bitrate)
  │           └── 创建 SocketCAN 总线实例
  ├── [start_thread] → 启动 ReadCan 线程 (daemon)
  ├── [start_thread] → 启动 CanMonitor 线程 (daemon)
  ├── [start_thread] → __fps_counter.start()  (启动帧率统计)
  └── [piper_init] → PiperInit()
        ├── SearchAllMotorMaxAngleSpd()    → 查询全部电机角度/速度限制
        ├── SearchAllMotorMaxAccLimit()    → 查询全部电机加速度限制
        └── SearchPiperFirmwareVersion()   → 查询固件版本
```

```python
def ConnectPort(self, can_init=False, piper_init=True, start_thread=True):
    # 步骤 1: CAN 初始化
    if(can_init or not self.__connected):
        init_status = self.__arm_can.Init()

    # 步骤 2: 设置连接状态
    with self.__lock:
        if self.__connected:
            return           # 防止重复连接
        self.__connected = True
        self.__read_can_stop_event.clear()
        self.__can_monitor_stop_event.clear()

    # 步骤 3: 启动后台线程
    if start_thread:
        self.__can_deal_th = threading.Thread(target=ReadCan, daemon=True)
        self.__can_deal_th.start()
        self.__can_monitor_th = threading.Thread(target=CanMonitor, daemon=True)
        self.__can_monitor_th.start()
        self.__fps_counter.start()

    # 步骤 4: 机械臂初始化查询
    if piper_init and self.__arm_can is not None:
        self.PiperInit()
```

### 3.5.2 PiperInit — 初始化查询序列

```python
def PiperInit(self):
    self.SearchAllMotorMaxAngleSpd()      # 查询 6 个电机各 1 次
    self.SearchAllMotorMaxAccLimit()      # 查询 6 个电机各 1 次
    self.SearchPiperFirmwareVersion()     # 查询固件版本
```

`SearchAllMotorMaxAngleSpd()` 和 `SearchAllMotorMaxAccLimit()` 分别对电机 1-6 逐一发送查询指令（CAN ID `0x472`），区别在于 `search_content` 参数：

```python
def SearchAllMotorMaxAngleSpd(self):
    for motor_num in [1, 2, 3, 4, 5, 6]:
        self.SearchMotorMaxAngleSpdAccLimit(motor_num, 0x01)  # 查询角度/速度限制

def SearchAllMotorMaxAccLimit(self):
    for motor_num in [1, 2, 3, 4, 5, 6]:
        self.SearchMotorMaxAngleSpdAccLimit(motor_num, 0x02)  # 查询加速度限制
```

这三个查询之所以必要，是因为：
- **角度限制**：获取每个电机的最大/最小角度限制和最大关节速度，用于后续的软限位保护。
- **加速度限制**：了解每个电机的最大加速度能力，用于运动规划。
- **固件版本**：获取机械臂主控板的固件版本字符串（如 `"S-V1.7.4"`），用于兼容性检查。

### 3.5.3 DisconnectPort — 断开流程

```python
def DisconnectPort(self, thread_timeout=0.1):
    with self.__lock:
        self.__connected = False
        self.__read_can_stop_event.set()  # 通知 ReadCan 线程退出

    # 等待 ReadCan 线程退出（超时 0.1 秒）
    if self.__can_deal_th and self.__can_deal_th.is_alive():
        self.__can_deal_th.join(timeout=thread_timeout)

    # 关闭 CAN 端口
    self.__arm_can.Close()
```

CanMonitor 线程通过 `__can_monitor_stop_event` 自然退出（每 50ms 检查一次），不需要显式 join。

---

## 3.6 Enable/Disable 流程

### 3.6.1 EnablePiper()

```python
def EnablePiper(self) -> bool:
    enable_list = self.GetArmEnableStatus()  # 读取当前使能状态
    self.EnableArm(7)                        # 发送使能指令
    return all(enable_list)                  # 返回使能前的状态
```

**调用链**：

1. `GetArmEnableStatus()` → 从 `GetArmLowSpdInfoMsgs()` 读取 6 个电机的 `foc_status.driver_enable_status`，返回布尔值列表。
2. `EnableArm(7)` → 构造 `ArmMsgMotorEnableDisableConfig(motor_num=7, enable_flag=0x02)`，编码后发送 CAN 帧 `0x471`。

### 3.6.2 DisablePiper()

```python
def DisablePiper(self) -> bool:
    enable_list = self.GetArmEnableStatus()
    self.DisableArm(7)
    return any(enable_list)  # 如果有任何电机之前是使能的，返回 True
```

`DisableArm(7)` 发送 `enable_flag=0x01`，其他逻辑与 EnableArm 相同。

### 3.6.3 motor_num=7 的含义

CAN 协议中，`motor_num` 取值 1-6 分别代表单独控制 1-6 号关节电机，取值 **7 代表全部电机**。这是一个 SDK 层面的约定（不是标准 CANopen 协议）。对应的 CAN 帧编码：

```python
# EnableArm(7):
tx_can_frame.data = [0x07, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
#                    ↑     ↑
#                    motor_num=7  enable_flag=0x02 (使能)
```

---

## 3.7 消息类型系统

### 3.7.1 系统分层架构

PiPER SDK 的消息处理分为四个层次：

```
┌─────────────────────────────────────────────┐
│  C_PiperInterface_V2                        │  ← 应用层 (读写 API)
├─────────────────────────────────────────────┤
│  PiperMessage                               │  ← 消息容器 (统一消息格式)
├─────────────────────────────────────────────┤
│  C_PiperParserV2.EncodeMessage/DecodeMessage│  ← 编解码层 (Python ↔ CAN bytes)
├─────────────────────────────────────────────┤
│  C_STD_CAN.SendCanMessage/ReadCanMessage    │  ← 硬件层 (SocketCAN)
└─────────────────────────────────────────────┘
```

### 3.7.2 PiperMessage — 通用消息容器

`PiperMessage` 类（`piper_msgs/msg_v2/arm_messages.py`）是整个 SDK 的统一消息容器。它聚合了**所有**可能的反馈消息和发送消息类型：

```python
class PiperMessage:
    def __init__(self, type_=None, time_stamp=0.0,
                 arm_status_msgs=None,           # 状态反馈
                 arm_joint_feedback=None,        # 关节角度反馈
                 gripper_feedback=None,          # 夹爪反馈
                 arm_end_pose=None,              # 末端位姿反馈
                 arm_high_spd_feedback=None,     # 高速驱动器反馈 ×6
                 arm_low_spd_feedback=None,      # 低速驱动器反馈 ×6
                 arm_motion_ctrl_1=None,         # 运动控制指令 1
                 arm_motion_ctrl_2=None,         # 运动控制指令 2
                 arm_joint_ctrl=None,            # 关节控制指令
                 arm_gripper_ctrl=None,          # 夹爪控制指令
                 arm_motor_enable=None,          # 电机使能/失能指令
                 # ... 共 30+ 种消息类型
                 ):
```

在解码阶段，`ParseCANFrame` 回调用同一个 `PiperMessage` 实例承载解码后的数据，然后通过 `type_` 字段路由到对应的 `__Update*` 方法更新内部缓存。

### 3.7.3 ArmMsgType — 消息类型枚举

`ArmMsgType`（`piper_msgs/msg_v2/arm_msg_type.py`）是一个枚举类，定义了所有可能的消息类型。其枚举值从 `auto()` 递增：

```python
class ArmMsgType(Enum):
    PiperMsgUnkonwn = 0x00
    PiperMsgStatusFeedback = auto()        # → 1
    PiperMsgEndPoseFeedback_1 = auto()     # → 2
    PiperMsgEndPoseFeedback_2 = auto()     # → 3
    # ...
    PiperMsgJointFeedBack_12 = auto()      # → 5
    PiperMsgJointFeedBack_34 = auto()      # → 6
    PiperMsgJointFeedBack_56 = auto()      # → 7
    PiperMsgGripperFeedBack = auto()       # → 8
    # ... (transmit)
    PiperMsgMotionCtrl_1 = auto()          # → ...
    PiperMsgJointCtrl_12 = auto()
    # ... 共 60+ 个枚举值
```

### 3.7.4 CanIDPiper 和 ArmMessageMapping — CAN ID 映射

`CanIDPiper`（`piper_msgs/msg_v2/can_id.py`）定义了所有 CAN ID 常量：

```python
class CanIDPiper(Enum):
    ARM_STATUS_FEEDBACK = 0x2A1         # 状态反馈
    ARM_END_POSE_FEEDBACK_1 = 0x2A2     # 末端位姿反馈 1
    ARM_JOINT_FEEDBACK_12 = 0x2A5       # 关节反馈 12
    ARM_GRIPPER_FEEDBACK = 0x2A8        # 夹爪反馈
    ARM_MOTION_CTRL_1 = 0x150           # 运动控制 1
    ARM_MOTION_CTRL_2 = 0x151           # 运动控制 2
    ARM_JOINT_CTRL_12 = 0x155           # 关节控制 12
    ARM_GRIPPER_CTRL = 0x159            # 夹爪控制
    ARM_MOTOR_ENABLE_DISABLE_CONFIG = 0x471  # 使能/失能
    ARM_FIRMWARE_READ = 0x4AF           # 固件读取
    # ... 共 40+ 个
```

`ArmMessageMapping` 提供 CAN ID 与 `ArmMsgType` 的双向映射（`id_to_type_mapping` 和 `type_to_id_mapping`）：

```python
class ArmMessageMapping:
    id_to_type_mapping = {
        0x2A1: ArmMsgType.PiperMsgStatusFeedback,
        0x2A5: ArmMsgType.PiperMsgJointFeedBack_12,
        0x155: ArmMsgType.PiperMsgJointCtrl_12,
        # ...
    }
    type_to_id_mapping = {v: k for k, v in id_to_type_mapping.items()}
```

这种设计使得编解码时可以通过 CAN ID 快速定位消息类型，反之亦然。

### 3.7.5 反馈消息 vs 发送消息

消息类型在代码组织上分为两个目录：

| 目录 | 内容 |
|------|------|
| `piper_msgs/msg_v2/feedback/` | 机械臂 → 主控的反馈消息 |
| `piper_msgs/msg_v2/transmit/` | 主控 → 机械臂的控制/配置指令 |

部分关键文件对照：

| 反馈类 | CAN ID | 发送类 | CAN ID |
|--------|--------|--------|--------|
| `ArmMsgFeedBackJointStates` | 0x2A5-7 | `ArmMsgJointCtrl` | 0x155-7 |
| `ArmMsgFeedBackGripper` | 0x2A8 | `ArmMsgGripperCtrl` | 0x159 |
| `ArmMsgFeedbackStatus` | 0x2A1 | `ArmMsgMotionCtrl_1` | 0x150 |
| `ArmMsgFeedBackEndPose` | 0x2A2-4 | `ArmMsgMotionCtrl_2` | 0x151 |

### 3.7.6 编解码完整流程

#### 解码（CAN → Python 对象）

```
C_STD_CAN.ReadCanMessage()
  └→ bus.recv(timeout=0.001)  → can.Message (arbitration_id + data)
      └→ callback_function = ParseCANFrame
          └→ parser.DecodeMessage(rx_can_frame, msg)
              ├→ 根据 arbitration_id 匹配 if/elif 分支
              ├→ 通过 ConvertBytesToInt + ConvertToNegative_XXbit 解码各字段
              ├→ 设置 msg.type_
              └→ 返回 True (ID 已识别) / False (未知 ID)
          └→ UpdateArmJointState(msg)  [以关节状态为例]
              └→ 加锁 → 更新 __arm_joint_msgs.joint_state.joint_X → 释放锁
```

#### 编码（Python 对象 → CAN）

```
C_PiperInterface_V2.JointCtrl(j1..j6)
  └→ ArmMsgJointCtrl(joint_1=j1, joint_2=j2)   # 创建消息对象
  └→ PiperMessage(type_=PiperMsgJointCtrl_12, arm_joint_ctrl=joint_ctrl)
  └→ parser.EncodeMessage(msg, tx_can_frame)
      ├→ 根据 msg.type_ 匹配 if/elif 分支
      ├→ ArmMessageMapping.get_mapping(msg_type=type_) → arbitration_id
      ├→ ConvertToList_32bit / 16bit / 8bit → 8 字节 data list
      └→ 返回 True
  └→ __arm_can.SendCanMessage(tx_can.arbitration_id, tx_can.data)
      └→ bus.send(Message(arbitration_id=id, data=data))
```

---

## 3.8 SDK 数值单位总结

PiPER SDK 广泛使用**整数表示法**来避免浮点精度问题。下表列出所有关键物理量在 SDK 内部的表示单位：

| 物理量 | SDK 内部单位 | 换算公式 | 示例 |
|--------|-------------|---------|------|
| 关节角度 | 0.001 度（毫度） | `angle_deg = raw / 1000` | `joint_1=45000` = 45.0 deg |
| 夹爪角度 | 0.001 度 | `gripper_deg = raw / 1000` | `gripper_angle=30000` = 30.0 deg |
| 夹爪力矩 | 0.001 N/m | `effort_nm = raw / 1000` | `effort=1000` = 1.0 N/m |
| 末端位置 (X, Y, Z) | 0.001 mm | `pos_mm = raw / 1000` | `x=500000` = 500 mm |
| 末端姿态 (RX, RY, RZ) | 0.001 度 | `euler_deg = raw / 1000` | `rx=90000` = 90.0 deg |
| 电机限制角度 | 0.1 度 | `limit_deg = raw / 10` | `max_angle=1500` = 150.0 deg |
| 关节最大速度 | 0.001 rad/s | `vel_rads = raw / 1000` | `max_spd=3000` = 3.0 rad/s |
| 关节最大加速度 | 0.01 rad/s^2 | `acc_rads2 = raw / 100` | `max_acc=500` = 5.0 rad/s^2 |
| 末端线速度 | 0.001 m/s | `vel_ms = raw / 1000` | `lin_vel=500` = 0.5 m/s |
| 末端角速度 | 0.001 rad/s | `ang_vel = raw / 1000` | `ang_vel=1000` = 1.0 rad/s |
| 末端线加速度 | 0.001 m/s^2 | `lin_acc = raw / 1000` | |
| 末端角加速度 | 0.001 rad/s^2 | `ang_acc = raw / 1000` | |
| 驱动器电压 | 0.1 V | `vol = raw / 10` | `vol=240` = 24.0 V |
| 驱动器/电机温度 | 1 degC | — | `foc_temp=45` = 45 degC |
| 母线电流 | 0.001 A | `current_A = raw / 1000` | `bus_current=1500` = 1.5 A |

**关键记忆口诀**：
- 角度类（关节/夹爪/末端姿态）→ `/1000` 得到度
- 位置类（末端 X/Y/Z）→ `/1000` 得到 mm
- 力矩类（夹爪力矩）→ `/1000` 得到 N/m
- 驱动器电压 → `/10` 得到 V，这是少数不按千分之一的特例

---

## 3.9 使用示例

下面展示一个最小但完整的 Python 脚本，演示如何使用 SDK 连接、使能、控制 PiPER 机械臂。

```python
#!/usr/bin/env python3
"""PiPER SDK 最小使用示例 —— 关节空间位置控制"""

import time
import math
from piper_sdk.interface.piper_interface_v2 import C_PiperInterface_V2

# ───────────────── 步骤 1: 创建接口实例 ─────────────────
# can_name 为 SocketCAN 接口名，通常为 "can0"
# 使用 PCIe CAN 模块时 judge_flag=False
piper = C_PiperInterface_V2(
    can_name="can0",
    judge_flag=True,
    can_auto_init=True,
    start_sdk_joint_limit=True,      # 启用 SDK 软限位保护
    start_sdk_gripper_limit=True,
    start_sdk_fk_cal=False,          # 不需要 FK 时可关闭以节省计算
)

# ───────────────── 步骤 2: 连接并使能 ─────────────────
print("[1] 正在连接 CAN 口并启动后台线程...")
piper.ConnectPort(can_init=True, piper_init=True, start_thread=True)
time.sleep(0.5)  # 等待初始化查询完成和首帧数据到达

print("[2] 正在使能机械臂...")
piper.EnablePiper()
time.sleep(0.3)

# 确认使能状态
enable_status = piper.GetArmEnableStatus()
print(f"    各关节使能状态: {enable_status}")

# ───────────────── 步骤 3: 设置控制模式 ─────────────────
print("[3] 切换到关节控制模式 (MOVE J)...")
piper.ModeCtrl(
    ctrl_mode=0x01,       # CAN 指令控制模式
    move_mode=0x01,       # MOVE J (关节空间)
    move_spd_rate_ctrl=30 # 30% 速度
)
time.sleep(0.1)

# ───────────────── 步骤 4: 读取当前关节位置 ─────────────────
joint_msgs = piper.GetArmJointMsgs()
current_joints = [
    joint_msgs.joint_state.joint_1,
    joint_msgs.joint_state.joint_2,
    joint_msgs.joint_state.joint_3,
    joint_msgs.joint_state.joint_4,
    joint_msgs.joint_state.joint_5,
    joint_msgs.joint_state.joint_6,
]
print(f"[4] 当前关节角度 (毫度): {current_joints}")
print(f"    当前关节角度 (度):   {[j * 0.001 for j in current_joints]}")

# ───────────────── 步骤 5: 发送目标位置 ─────────────────
# 注意：实际使用中应设置合理的运动范围，不要超出关节限位
print("[5] 发送目标关节角度...")
target_j1 =  45000   #  45.0 度
target_j2 =  90000   #  90.0 度
target_j3 = -90000   # -90.0 度
target_j4 =  30000   #  30.0 度
target_j5 =  45000   #  45.0 度
target_j6 =  60000   #  60.0 度

piper.JointCtrl(target_j1, target_j2, target_j3,
                target_j4, target_j5, target_j6)

# 等待机械臂运动到位（简单实现：固定等待；生产环境应检查 motion_status）
time.sleep(3.0)

# ───────────────── 步骤 6: 循环读取反馈 ─────────────────
print("[6] 循环读取反馈 (按 Ctrl+C 停止)...")
try:
    for i in range(100):
        joint_msgs = piper.GetArmJointMsgs()
        gripper_msgs = piper.GetArmGripperMsgs()
        status = piper.GetArmStatus()

        joints_deg = [
            joint_msgs.joint_state.joint_1 * 0.001,
            joint_msgs.joint_state.joint_2 * 0.001,
            joint_msgs.joint_state.joint_3 * 0.001,
            joint_msgs.joint_state.joint_4 * 0.001,
            joint_msgs.joint_state.joint_5 * 0.001,
            joint_msgs.joint_state.joint_6 * 0.001,
        ]

        print(f"[{i:03d}] 关节角度: {[f'{j:.2f}' for j in joints_deg]} 度 | "
              f"状态: {status.arm_status.motion_status} | "
              f"帧率: {joint_msgs.Hz:.1f} Hz")

        # 检查 CAN 是否正常
        if not piper.isOk():
            print("警告: CAN 总线可能已卡死!")

        time.sleep(0.05)  # 约 20Hz 打印

except KeyboardInterrupt:
    print("\n用户中断")

# ───────────────── 步骤 7: 失能并断开 ─────────────────
print("[7] 失能机械臂并断开连接...")
piper.DisablePiper()
time.sleep(0.2)
piper.DisconnectPort()
print("    已断开连接。")
```

### 示例关键说明

1. **`ConnectPort` 的参数**：
   - `can_init=True`：强制执行 CAN 口初始化（首次或重新连接时必需）。
   - `piper_init=True`：自动查询电机参数和固件版本。
   - `start_thread=True`：启动 ReadCan 和 CanMonitor 后台线程。

2. **模式切换顺序**：必须先 `EnablePiper()` 使能，再 `ModeCtrl()` 设置控制模式，最后才能发送 `JointCtrl()` 指令。

3. **`ModeCtrl(move_spd_rate_ctrl=30)`**：设置速度为满速的 30%，在调试阶段建议使用较低速度，降低碰撞风险。

4. **读取顺序无关**：由于后台线程持续更新缓存，`GetArmJointMsgs()` 等读方法随时返回最新的缓存值，无需担心阻塞。但需要注意首次调用前给足初始化时间（至少等待 0.3-0.5 秒）。

5. **线程安全**：所有 getter 方法内部使用独立锁，可以在主线程中安全调用，不受 ReadCan 线程影响。

6. **错误检查**：生产代码中应检查各方法的返回值、`isOk()` 状态以及 `arm_status` 中的错误码，实现健壮的错误处理。

### 进阶：主从模式设置

```python
# 在教学臂（主臂）上执行
master = C_PiperInterface_V2(can_name="can0")
master.ConnectPort()
master.EnablePiper()
master.ModeCtrl(ctrl_mode=0x01, move_mode=0x01)
# 配置为示教输入臂，控制指令偏移到 0x16x
master.MasterSlaveConfig(
    linkage_config=0xFA,    # 教学输入臂
    feedback_offset=0x00,
    ctrl_offset=0x10,       # 控制指令 ID 偏移 +0x10
    linkage_offset=0x10     # 联动目标地址偏移 +0x10
)

# 在运动臂（从臂）上执行
slave = C_PiperInterface_V2(can_name="can1")
slave.ConnectPort()
slave.EnablePiper()
# 配置为运动输出臂，读取偏移后的控制指令
slave.MasterSlaveConfig(
    linkage_config=0xFC,    # 运动输出臂
    feedback_offset=0x10,   # 反馈 ID 也偏移
    ctrl_offset=0x00,
    linkage_offset=0x00
)

# 从臂读取主臂的控制指令
ctrl = slave.GetArmJointCtrl()
print(f"主臂目标角度: J1={ctrl.joint_ctrl.joint_1 * 0.001:.2f}°")
```

---

## 本章小结

本章从源代码层面深入剖析了 PiPER SDK 的核心架构：

1. **`C_PiperInterface_V2`** 是整个系统的中枢，通过单例模式管理 CAN 连接，内部聚合了底层 CAN 硬件端口、协议编解码器和前向运动学求解器。

2. **双线程后台模型**（ReadCan + CanMonitor）解决了 CAN 总线推模型数据接收和总线健康监测的需求，通过细粒度锁机制保证线程安全。

3. **消息类型系统**（`PiperMessage` + `ArmMsgType` + `CanIDPiper` + `ArmMessageMapping`）提供了一套完整的 CAN 帧与 Python 对象之间的双向映射，所有数据以整数形式存储（毫度、毫牛米等），避免了浮点精度损失。

4. **统一的控制指令发送流程**：构造消息对象 → 编码为 CAN 帧 → 调用 `SendCanMessage` → 检查发送状态。

5. **主从模式**通过 CAN ID 偏移机制实现，教学臂主动发送控制指令，运动臂监听并执行，实现硬件级别的低延迟主从跟随。

掌握这些底层机制后，你可以自信地直接通过 SDK 接口操控 PiPER 机械臂，为后续的数据采集（第4章）和策略部署（第5章）做好准备。
