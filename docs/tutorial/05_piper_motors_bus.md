# 第五章 PiPER 电机总线适配层：PiperMotorsBus 完全解析

> "适配层的本质不是翻译数据格式，而是翻译通信模型——把 CAN 总线的推模型翻译成 LeRobot 框架的拉模型。"

## 本章导览

本章是整个教程最重要的一章。前四章你已经理解了硬件基础（CAN 总线、电机型号）、SDK 原理（线程模型、CAN 编解码）和 LeRobot 框架（MotorsBus 基类）。现在所有知识点汇聚到同一个文件——`piper.py`。这个文件只有约 700 行，但它完成了一件极富挑战的工作：让一个为 Dynamixel 串口舵机设计的模仿学习框架，无缝驱动一台完全不同的 CAN 总线机械臂。

学完本章后，你应该能够：

- 完整解释 `get_action()` 和 `set_action()` 的每一步调用链
- 手写归一化/反归一化公式，并带入任意关节参数进行验算
- 理解为什么某些方法是空实现——不是因为"偷懒"，而是 CAN 总线和串口的通信模型根本不同
- 根据 PiPER 的校准表判断任意关节角度的有效范围
- 画出一次完整数据帧（read-write cycle）的时序图

如果读完本章只能记住一句话，记住这句：

> PiperMotorsBus 是一个**适配器**，它把 CAN 线上毫度为单位的电机编码器整数，翻译成 LeRobot 策略模型期望的 [-100, 100] 浮点数组，反之亦然。

---

## 5.1 "适配"到底是什么

### 5.1.1 问题的根源

LeRobot 是 HuggingFace 为 Dynamixel 和 Feetech 串口舵机设计的模仿学习框架。这类舵机的通信模型非常直接：

```
         USB 转串口线
主机 ◄─────────────────► 舵机链 (Daisy Chain)
      发送指令包 / 接收状态包
      每个舵机有唯一 ID (0-252)
      广播写、同步读、Ping...
```

PiPER 机械臂则完全不同：

```
         USB-CAN 适配器
主机 ◄─────────────────► CAN 总线
      200Hz 主动上报帧（推模型）
      控制帧需要按 CAN ID 发送
      没有舵机 ID 概念，只有 CAN ID
      没有广播 Ping，没有同步读
```

两者的通信模型差异可以归纳为下表：

| 维度 | Dynamixel / Feetech | PiPER (CAN) |
|------|---------------------|-------------|
| 物理总线 | UART 串口（TTL/RS-485） | CAN 总线（差分信号） |
| 通信模型 | 拉模型：主机轮询每个舵机 | 推模型：机械臂主动上报 |
| 帧格式 | Dynamixel Protocol 2.0（可变长包） | CAN 2.0B（8 字节固定数据场） |
| 寻址方式 | 舵机 ID（0-252） | CAN ID（11 位或 29 位） |
| 同步操作 | `sync_read` / `sync_write` 广播指令 | 多帧发送，每帧对应不同 CAN ID |
| 电机发现 | `broadcast_ping()` 扫描所有 ID | 不需要——只连一只臂 |
| 握手协议 | Ping → 读型号 → 写配置 | CAN 口 UP + 主动上报即建立通信 |

### 5.1.2 解决方案：适配器模式

软件工程中有一个经典设计模式叫**适配器模式**（Adapter Pattern）：

> 将一个类的接口转换成客户期望的另一个接口。适配器让原本由于接口不兼容而不能一起工作的类可以合作。

在我们这个具体场景中：

- **客户期望的接口** = `MotorsBus` 基类（LeRobot 定义的统一接口）
- **被适配者** = `C_PiperInterface_V2`（SDK 的 CAN 通信接口）
- **适配器** = `PiperMotorsBus`

```
LeRobot 策略模型                   PiPER 机械臂硬件
┌─────────────────┐               ┌─────────────────┐
│ action_dict     │               │ CAN 帧发送       │
│ {"joint1": 42.5 │               │ CAN ID 0x155     │
│  "joint2":-10.3 │               │ Data: 63750...   │
│  ...}           │               │                  │
└───────┬─────────┘               └────────▲─────────┘
        │                                  │
        ▼                                  │
┌──────────────────────────────────────────┴───────┐
│                PiperMotorsBus                    │
│  ┌──────────────────────────────────────────┐   │
│  │  _unnormalize()  [-100,100] → 原始整数    │   │
│  │  JointCtrl()      原始整数 → CAN 帧       │   │
│  │  GripperCtrl()    原始整数 → CAN 帧       │   │
│  └──────────────────────────────────────────┘   │
│  ┌──────────────────────────────────────────┐   │
│  │  GetArmJointMsgs()  CAN 帧 → 原始整数     │   │
│  │  GetArmGripperMsgs() CAN 帧 → 原始整数    │   │
│  │  _normalize()       原始整数 → [-100,100] │   │
│  └──────────────────────────────────────────┘   │
└─────────────────────────────────────────────────┘
        │                                  ▲
        ▼                                  │
┌─────────────────┐               ┌─────────────────┐
│ observation     │               │ CAN 帧接收       │
│ {"joint1": 33.3 │               │ CAN ID 0x201     │
│  "joint2": 50.0 │               │ Data: 50000...   │
│  ...}           │               │                  │
└─────────────────┘               └─────────────────┘
```

### 5.1.3 适配的三层含义

"适配"在这个项目中不是简单的数据格式转换。它发生在三个层面：

**第一层：硬件通信层替代**

Dynamixel 用串口协议包，PiPER 用 CAN 帧。在基类 `MotorsBus` 中，`_read()` / `_write()` / `_sync_read()` / `_sync_write()` 等方法底层都依赖 `PortHandler` + `PacketHandler` 收发串口数据包。PiPER 把这些方法全部替换为 SDK 的 `GetArmJointMsgs()` / `JointCtrl()` 等 CAN 原生调用。

**第二层：数据格式转换**

这是整个系统最核心的数学运算。SDK 读到的关节值是整型的**毫度**（millidegrees）——joint1 的范围是 [-150000, 150000]，gripper 的范围是 [0, 68000]。LeRobot 框架期望的是统一归一化到 [-100, 100]（关节）或 [0, 100]（夹爪）的浮点值。`_normalize()` 和 `_unnormalize()` 完成这层映射。

**第三层：接口语义保留**

尽管底层通信方式完全不同，但上层调用者（`PiperFollower`）看到的接口和 Dynamixel 版本一模一样——`get_action()` 返回 `dict[str, float]`，`set_action(action_dict)` 接收 `dict[str, float]`。这种语义保留使得策略模型和数据集格式无需任何修改就可以在不同硬件平台上切换。

### 5.1.4 比喻：电源适配器

用一个日常比喻来总结：

> LeRobot 的 `MotorsBus` 接口就像 **USB-C 电源标准**（统一的电压/电流/协议）。
> PiPER 的 CAN 协议就像墙上 **220V 交流电插座**。
> `PiperMotorsBus` 就是那个 **充电头**：
> - 输入端（USB-C 口）接受标准的归一化值 [-100, 100]
> - 输出端（插头）输出 PiPER 特定的 CAN 协议（毫度整数 + CAN ID 帧）
> - 内部电路（`_normalize`/`_unnormalize`）完成交流→直流的数学转换

---

## 5.2 PiperMotorsBus 类结构

### 5.2.1 继承关系

```
abc.ABC
  └── MotorsBus (motors_bus.py)         ← LeRobot 框架的电机总线抽象基类
        └── PiperMotorsBus (piper.py)    ← PiPER CAN 总线实现
```

### 5.2.2 __init__ 参数详解

```python
def __init__(
    self,
    id: str,                                     # 机器人标识
    port: str,                                    # CAN 口名
    motors: dict[str, Motor],                     # 电机定义字典
    calibration: dict[str, MotorCalibration] | None = None,  # 标定信息字典
):
```

**`id: str`** — 机器人的唯一标识。在典型的 PC 中继遥操作（PC Relay Teleop）场景中，主臂 ID 为 `"pc_relay_leader"`，从臂 ID 为 `"pc_relay_follower"`。这个 ID 会被用于日志输出和校准文件路径规划。

**`port: str`** — Linux SocketCAN 接口名称，如 `"can0"` 或 `"can1"`。这个字符串直接传给 `C_PiperInterface_V2(port)` 构造函数，SDK 内部用它创建 `can.interface.Bus` 实例。

**`motors: dict[str, Motor]`** — 电机定义字典，键是关节名称（如 `"joint1"`），值是 `Motor` 数据类。每个 `Motor` 包含三个字段：

- `id: int` — 电机逻辑 ID（1~7）
- `model: str` — 电机型号字符串（`"AGILEX-M"` 或 `"AGILEX-S"`）
- `norm_mode: MotorNormMode` — 归一化模式，决定映射到 `[-100, 100]` 还是 `[0, 100]`

典型构造示例：
```python
motors = {
    "joint1":  Motor(1, "AGILEX-M", MotorNormMode.RANGE_M100_100),
    "joint2":  Motor(2, "AGILEX-M", MotorNormMode.RANGE_M100_100),
    "joint3":  Motor(3, "AGILEX-M", MotorNormMode.RANGE_M100_100),
    "joint4":  Motor(4, "AGILEX-S", MotorNormMode.RANGE_M100_100),
    "joint5":  Motor(5, "AGILEX-S", MotorNormMode.RANGE_M100_100),
    "joint6":  Motor(6, "AGILEX-S", MotorNormMode.RANGE_M100_100),
    "gripper": Motor(7, "AGILEX-S", MotorNormMode.RANGE_0_100),
}
```

**`calibration: dict[str, MotorCalibration] | None`** — 标定信息字典。每个 `MotorCalibration` 包含：

- `id: int` — 电机 ID
- `drive_mode: int` — 驱动模式（PiPER 不使用，始终为 0）
- `homing_offset: int` — 回零偏移（PiPER 不使用，始终为 0）
- `range_min: int` — 关节最小原始值（毫度）
- `range_max: int` — 关节最大原始值（毫度）

典型构造示例：
```python
calibration = {
    "joint1":  MotorCalibration(1, 0, 0, -150000, 150000),
    "joint2":  MotorCalibration(2, 0, 0, 0, 180000),
    "joint3":  MotorCalibration(3, 0, 0, -170000, 0),
    "joint4":  MotorCalibration(4, 0, 0, -100000, 100000),
    "joint5":  MotorCalibration(5, 0, 0, -65000, 65000),
    "joint6":  MotorCalibration(6, 0, 0, -100000, 130000),
    "gripper": MotorCalibration(7, 0, 0, 0, 68000),
}
```

### 5.2.3 关键对象属性

构造函数初始化以下几个核心属性，它们是整个文件正常运转的基础：

**`self.piper: C_PiperInterface_V2`** — 来自 AgileX 官方 SDK 的 CAN 接口对象。构造函数中通过 `C_PiperInterface_V2(port)` 创建，但**不立即连接**——连接操作推迟到 `connect()` 调用时。之后所有硬件操作都通过 `self.piper.XXX()` 调用。

**`self.port_handler: PortHandler`** — 来自 `wego_piper` 包的 CAN 口管理器。它只做三件事：
1. `setupPort(piper)` — 设置 CAN 口参数（波特率等）
2. `openPort()` — 打开 CAN 口
3. `closePort()` — 关闭 CAN 口

注意：`PortHandler` 来自第三方包 `wego_piper`，而 `C_PiperInterface_V2` 来自官方 `piper_sdk`。两者职责不同——`PortHandler` 只负责 CAN 口生命周期管理，`C_PiperInterface_V2` 负责 CAN 帧收发、编解码和控制命令。

**`self.id: str`** — 保存构造函数传入的机器人标识，用于日志和校准文件路径。

### 5.2.4 类属性（由父类方法引用）

PiperMotorsBus 定义了一些重要的类属性，它们会被父类 `MotorsBus` 的方法引用：

```python
available_baudrates = [500_000, 1_000_000]  # 支持的 CAN 波特率
default_timeout = 1000                       # 超时时间（毫秒）
apply_drive_mode = False                     # 不需要反转驱动模式
normalized_data = ["Present_Position", "Goal_Position"]  # 需要归一化的数据字段
```

**`apply_drive_mode = False`** 是一个关键设计选择。在 Dynamixel 系统中，如果电机的 `drive_mode` 设为反转模式，`_normalize()` 会对结果取反（`-norm`），`_unnormalize()` 也会对输入取反。但 PiPER 不支持这种硬件级别的方向反转，也不需要通过软件来反转——因为校准表中的 `range_min` / `range_max` 已经隐式定义了方向（如 joint3 的 min=-170000, max=0 就是负向范围）。因此 `apply_drive_mode` 始终为 `False`，`drive_mode` 相关的取反逻辑永远不会触发。

---

## 5.3 覆写的方法（硬件层）

PiperMotorsBus 覆写了基类中所有需要直接与硬件交互的方法。这些方法构成了本类的"对外 API"，是策略模型、数据采集等上层代码实际调用的入口。

### 5.3.1 connect() — 建立 CAN 通信链路

```python
def connect(self, handshake: bool = True) -> bool:
    self.port_handler.setupPort(self.piper)   # ① 设置 CAN 口参数
    return self.port_handler.openPort()        # ② 打开 CAN 口
```

**步骤 ① setupPort(self.piper)**

调用 `wego_piper` 的 `PortHandler.setupPort()`。这个方法内部通过 `piper_sdk` 提供的接口设置 CAN 口波特率等底层参数。传入 `self.piper` 对象让 `PortHandler` 知道该在哪个 SDK 实例上操作。

**步骤 ② openPort()**

打开 CAN 口。返回 `True` 表示成功，返回 `False` 表示失败。成功打开后，机械臂的 CAN 上报帧（约 200Hz）开始流经 SocketCAN，SDK 内部的 `ReadCan` 线程开始接收和解析数据。

**重要设计说明：connect() 只管通信链路，不管电机使能。**

这是两层分离的架构设计：

- **Bus 层**（本层）：管通信——CAN 口是否打开，数据是否能收发
- **Robot 层**（`PiperFollower`）：管电机——何时使能，何时回零，何时初始化

电机使能由上层 `PiperFollower.connect()` 通过调用 `enable_torque()` 完成，不是 `PiperMotorsBus.connect()` 的职责。这种分离遵循了 LeRobot 的原始设计哲学，也让每个层次的职责更加清晰。

**超时与重试**

`connect()` 没有内置重试机制。如果 `openPort()` 返回 `False`，调用者（通常是 `PiperFollower.connect()`）会收到 `False` 并决定如何处理。常见的失败原因包括：

- CAN 口名称错误（如写成 `"can0"` 但实际是 `"can1"`）
- CAN 口未 `ip link set up`
- 权限不足（当前用户不在 `dialout` 或对应 group 中）
- USB-CAN 适配器未插入或被其他进程占用

### 5.3.2 disconnect() — 安全关闭连接

```python
def disconnect(self, disable_torque: bool = False) -> None:
    if disable_torque:
        self.parking()               # ① 先回零位
        self.piper.DisablePiper()    # ② 再失能电机
    self.port_handler.closePort()    # ③ 最后关闭 CAN 口
```

**步骤 ① parking() — 回到安全位置**

调用 `parking()` 将所有关节移动到 `INITIALIZE_POSITION`（全零位姿）。这是安全措施——如果在机械臂处于极限位置时直接失能，手臂可能因重力猛然坠落，损坏硬件或伤及人员。

**步骤 ② DisablePiper() — 释放扭矩**

通过 CAN 发送失能命令，使电机进入自由状态。失能后，人可以轻松推动机械臂——这对于断电维护尤为重要。`DisablePiper()` 内部可能有重试逻辑（取决于 SDK 版本）。

**步骤 ③ closePort() — 关闭 CAN 口**

释放 SocketCAN 资源。关闭后，SDK 的 `ReadCan` 线程检测到 CAN 口关闭并退出。

**参数说明：**

- `disable_torque=False`（默认）：直接关闭 CAN 口，保持电机当前扭矩和位置。适用于程序临时重启（不改变机械臂姿态）。
- `disable_torque=True`：先回零再失能。适用于关机/维护场景。

### 5.3.3 get_action() — 读取当前关节状态（归一化）

这是整个系统调用最频繁的方法——每次数据采集（Record）、每次策略推理（Eval）、每次回放（Replay）都要调用。它完成从 CAN 帧到归一化字典的完整转换。

```python
def get_action(self) -> dict[str, Any]:
    # ① 从 SDK 读取原始反馈值
    msg_joint = self.piper.GetArmJointMsgs()       # 读关节编码器
    msg_gripr = self.piper.GetArmGripperMsgs()     # 读夹爪编码器

    # ② 从消息对象中提取各字段
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
```

**数据流全程追踪：**

```
机械臂编码器
    │
    ▼
[物理角度]  joint1=45.0°, joint2=90.0°, ..., gripper=34.0°
    │
    ▼  驱动器 ADC 采样，转换为毫度整数
[原始值]    joint1=45000, joint2=90000, ..., gripper=34000
    │
    ▼  驱动器组装 CAN 帧 (CAN ID 0x201-0x20A)
[CAN 帧]    8 字节 data field，200Hz 上报
    │
    ▼  SocketCAN 接收 → ReadCan 线程回调 → Parser 解码
[SDK 消息]  msg_joint.joint_state.joint_1 = 45000
    │          msg_joint.joint_state.joint_2 = 90000
    │          ...
    │          msg_gripr.gripper_state.grippers_angle = 34000
    │
    ▼  get_action() 提取各字段
[提取字典]  {"joint1": 45000, "joint2": 90000, ..., "gripper": 34000}
    │
    ▼  _normalize() 应用归一化公式
[归一化]    {"joint1": 30.0, "joint2": 50.0, ..., "gripper": 50.0}
```

**关键设计细节：**

1. **SDK 消息对象不是实时读取 CAN 帧**，而是读取 `ReadCan` 线程缓存的最新值。因此 `GetArmJointMsgs()` 几乎没有延迟（≈0ms），只是返回内存中已有的结构体。真正的延迟在 CAN 帧的采集到缓存更新之间。

2. **所有 6 个关节在一个消息对象中**，SDK 解析一帧 CAN 数据后同时更新全部 6 个关节的反馈值，这保证了关节状态的时间一致性。

3. **夹爪的读取是独立的**，因为它来自不同的 CAN ID（夹爪驱动器有自己的上报帧）。

4. **float() 转换**：SDK 返回的关节值是 `int` 类型，显式转为 `float` 是为了避免后续 `_normalize()` 中的整数除法问题（Python 中 `int / int` 会丢失精度）。

### 5.3.4 get_control() — 读取当前控制目标（未归一化）

```python
def get_control(self) -> dict[str, Any]:
    msg_joint = self.piper.GetArmJointCtrl()      # 读取控制目标值
    msg_gripr = self.piper.GetArmGripperCtrl()    # 读取夹爪控制目标值
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
```

**与 `get_action()` 的核心区别：**

| | get_action() | get_control() |
|---|---|---|
| 数据来源 | 编码器反馈（实际位置） | 控制命令值（目标位置） |
| SDK 方法 | `GetArmJointMsgs()` | `GetArmJointCtrl()` |
| 物理含义 | "机械臂现在在哪" | "上次我让它去哪" |
| 是否归一化 | 是（调用 `_normalize`） | 否（返回原始整数） |
| 用途 | 训练/推理/数据集记录 | 确认命令已发出、调试 |
| 在 PC 中继中的特殊用途 | — | 读主臂当前位置（主臂处于示教模式时 `GetArmJointMsgs()` 可能无数据） |

**重要：为什么 PC 中继遥操作中读主臂要用 `GetArmJointCtrl()`？**

在 PC 中继模式中，主臂处于**示教模式**（Teaching Mode），电机释放扭矩，人可以随意拖动。在这种状态下，主臂驱动器的工作方式和正常伺服控制不同——它可能不产生常规的编码器反馈流（`GetArmJointMsgs()` 返回的可能不是最新的编码器值），但控制命令接口（`GetArmJointCtrl()`）仍然是活跃的，跟踪着主臂的当前位置。因此 `piper_pc_relay_teleop.py` 中读主臂用的是 `GetArmJointCtrl()`：

```python
# piper_pc_relay_teleop.py 中的用法（示意）
leader_action = leader_bus.get_control()   # 读主臂控制值
follower_bus.set_action(leader_action)     # 发给从臂执行
```

### 5.3.5 set_action() — 发送动作命令给机械臂

这是策略部署时最核心的方法——把策略模型输出的归一化动作翻译成 CAN 控制帧。

```python
def set_action(self, action: dict[str, Any]) -> dict[str, Any]:
    # ① 反归一化: [-100,100]/[0,100] → SDK 原始整数
    action_denormalzed = self._unnormalize(action)

    # ② 设置控制模式
    self.piper.ModeCtrl(0x01, 0x01, 30, 0x00)

    # ③ 发送关节目标位置
    self.piper.JointCtrl(
        int(action_denormalzed["joint1"]),
        int(action_denormalzed["joint2"]),
        int(action_denormalzed["joint3"]),
        int(action_denormalzed["joint4"]),
        int(action_denormalzed["joint5"]),
        int(action_denormalzed["joint6"]),
    )

    # ④ 发送夹爪目标位置
    self.piper.GripperCtrl(
        abs(int(action_denormalzed["gripper"])), 1000, 0x03, 0
    )

    # ⑤ 返回实际发出的命令值（用于日志/调试）
    return self.get_control()
```

**参数：**

- `action: dict[str, Any]` — 归一化值字典，形如 `{"joint1": 42.5, "joint2": -10.3, ..., "gripper": 75.0}`。键是关节名称，值是归一化浮点数。

**返回值：**

- `dict[str, Any]` — 通过 `get_control()` 返回当前控制目标值（原始整数）。返回值用于日志记录和调试，不是机械臂的实际到达位置。机械臂的运动是异步的——`set_action()` 返回后，电机还在执行运动中。

**步骤 ① 反归一化 `_unnormalize()`**

将 [-100, 100] / [0, 100] 的浮点动作值转换为 SDK 期望的毫度整数。例如 joint1 的 42.5 → 63750 毫度。详见 5.6 节的公式推导。

**步骤 ② ModeCtrl(0x01, 0x01, 30, 0x00) — 设置控制模式**

这是整条 CAN 控制链路的"开关"。参数含义：

| 参数位置 | 值 | 含义 |
|---------|-----|------|
| 第 1 参数 | `0x01` | 关节空间位置控制（Joint Position Control） |
| 第 2 参数 | `0x01` | 在线控制模式 |
| 第 3 参数 | `30` | 速度百分比 30%（速度限制，安全考虑） |
| 第 4 参数 | `0x00` | 不使用 MIT 模式（标准位置控制） |

**速度参数为什么是 30%？**

这是安全设计。在 PC 中继遥操作场景（`piper_pc_relay_teleop.py`）中，速度通常设为 100%，因为主臂的动作由人实时控制，需要快速响应。而在策略部署场景（`lerobot-record` / `lerobot-eval`）中，策略模型的输出可能有抖动或不连续，30% 的速度限制可以减缓运动，降低碰撞风险。

**模式控制帧的 CAN 通信过程：**

`ModeCtrl(0x01, 0x01, 30, 0x00)` 内部会组装一条 CAN 帧并以 CAN ID `0x151` 发送到总线上。机械臂主控收到后，将控制模式切换到"关节位置控制"，并以 30% 的速度限制执行后续接收到的位置命令。

**当前代码的行为：每次 set_action() 都发送 ModeCtrl**

注意：在当前代码实现中，`ModeCtrl` 在每次 `set_action()` 调用中都发送一次。对于高速控制循环（如 30Hz 的策略推理），这意味着每秒发送 30 次相同的模式切换命令。虽然 CAN 总线设计上能够承受这种开销（每条 CAN 帧只有约 100 微秒的传输时间），但这仍然是不必要的冗余。

一个常见的优化是引入 `_mode_controlled` 标志位：

```python
# 优化方案（当前代码未实现，仅供参考）
if not self._mode_controlled:
    self.piper.ModeCtrl(0x01, 0x01, 30, 0x00)
    self._mode_controlled = True
```

这样 ModeCtrl 只在第一帧发送，后续帧直接跳到 JointCtrl。需要注意：如果发生连接断开、错误恢复等异常，需要重置该标志位。

**步骤 ③ JointCtrl(j1, j2, j3, j4, j5, j6) — 发送关节目标位置**

一次性发送 6 个关节的目标位置（毫度整数）。SDK 内部将这 6 个值分别打包到对应的 CAN 帧中：

| 关节 | CAN ID | 负载 |
|------|--------|------|
| joint1, joint2 | `0x155` | j1 (高字节 + 低字节), j2 (高字节 + 低字节) |
| joint3, joint4 | `0x156` | j3, j4 |
| joint5, joint6 | `0x157` | j5, j6 |

每个关节值占 2 字节（`int16`）。三条 CAN 帧依次发送，总传输时间约 3ms（在 1Mbps 波特率下，每条 8 字节 CAN 帧约 100 微秒，加上帧间隔和可能的仲裁等）。

**步骤 ④ GripperCtrl(angle, speed, mode, block) — 发送夹爪目标位置**

| 参数 | 当前值 | 含义 |
|------|--------|------|
| `angle` | `abs(int(action_denormalzed["gripper"]))` | 目标位置（毫度），使用 `abs()` 确保非负 |
| `speed` | `1000` | 速度参数 |
| `mode` | `0x03` | 夹爪控制模式 |
| `block` | `0` | 非阻塞——不等待夹爪到达位置 |

**为什么夹爪位置要用 `abs()`？**

`_unnormalize()` 对 `RANGE_0_100` 模式（夹爪）计算出的原始值：
```python
raw = int((bounded_val / 100) * (max - min) + min)
```
其中 `min=0, max=68000`，`bounded_val` 被钳位在 [0, 100]。理论上来讲结果一定非负。但为了防止浮点精度或异常输入导致微小负值（如 `-0.001` 经过 `int()` 变成 `-1`），加一层 `abs()` 保护是最稳妥的做法。

**步骤 ⑤ 返回 get_control()**

返回的不是机械臂的"实际位置"，而是刚发出的"目标命令值"。这提供了一个确认——命令确实已通过 CAN 发出。如果需要知道机械臂是否到达目标，应该在下一次循环中调用 `get_action()` 获取编码器反馈值，与期望值做比较。

### 5.3.6 Parking() — 回到初始安全位置

```python
def parking(self):
    timeout = 100                               # 最多等 100 个周期
    self.set_action(INITIALIZE_POSITION)         # 发送初始位置作为目标
    time.sleep(0.1)
    status = self.piper.GetArmStatus()           # 读取运动状态

    while status.arm_status.motion_status and timeout:
        self.set_action(INITIALIZE_POSITION)      # 重发目标
        time.sleep(0.1)
        status = self.piper.GetArmStatus()
        timeout -= 1
```

**`INITIALIZE_POSITION`** 定义在 `tables.py` 中，所有 7 个关节（6 个关节 + 1 个夹爪）的目标值均为 0（归一化值）。这对应机械臂的"中立姿态"——所有关节回到原始位置，夹爪全开。

**循环逻辑：**

`GetArmStatus()` 返回的 `status.arm_status.motion_status` 是一个布尔值，表示机械臂当前是否正在运动中。`parking()` 的逻辑是：
1. 先发送一次目标位置
2. 等待 100ms
3. 检查是否还在运动
4. 如果还在运动，重新发送目标位置（防止丢帧），再等 100ms
5. 重复，直到运动停止或超时

超时保护：`timeout = 100` 个周期，每周期 100ms，即最多等待 10 秒。如果 10 秒后机械臂仍未到达目标，方法退出但**不报错**——这留给上层调用者处理。

**安全提示：**

`PiperFollower.connect()` 默认会调用 `parking()`。这意味着连接机械臂时它会自动运动到初始位姿。如果机械臂当前所在位置和初始位姿之间路径上有障碍物，或者机械臂被卡住，自动回零可能导致碰撞。此时应该使用 `connect(calibrate=False)` 跳过自动回零。

### 5.3.7 clear_gripper() — 张开夹爪

```python
def clear_gripper(self):
    self.piper.GripperCtrl(0, 1000, 0x03, 0)
```

极其简单的方法：发送目标位置 0（毫度），即夹爪完全张开。参数 `1000` 是速度，`0x03` 是控制模式，`0` 表示非阻塞。

注意：`clear_gripper()` 直接调用 SDK 方法，跳过了 `set_action()` 的归一化流程——`0` 就是一个原始毫度值。这不会触发 `_mode_controlled` 标志位（如果存在的话）的检查。

### 5.3.8 enable_torque() — 使能电机

```python
def enable_torque(self, motors=None, num_retry=0) -> bool:
    retry = 10
    while not self.piper.EnablePiper() and retry:
        retry -= 1
        time.sleep(0.1)
    logger.info(f"{self.piper.GetArmEnableStatus()}")
    if not retry:
        return False
    logger.info(f"{self.id} torque on.")
    return True
```

使能是通过 SDK 的 `EnablePiper()` 方法完成的，它内部发送 CAN 命令到机械臂主控，使所有电机进入伺服锁定状态。使能成功后，电机保持当前角度，抵抗外力——即俗称的"上电锁住"。

**重试逻辑：**

最多尝试 10 次，每次间隔 100ms。所以总重试时间约为 1 秒。10 次全部失败返回 `False`。

**参数说明：**

- `motors` — 接受但不使用。接口保留是为了与基类 `MotorsBus` 的签名兼容（Dynamixel 版本支持逐电机使能）。PiPER 不支持逐个电机单独使能——要么全部使能，要么全不使能。
- `num_retry` — 同样接受但不使用。PiPER 版本使用自己内部的 `retry=10` 常量。

### 5.3.9 disable_torque() — 失能电机

```python
def disable_torque(self, motors=None, num_retry=0) -> None:
    while self.piper.DisablePiper() and num_retry:
        num_retry -= 1
        time.sleep(0.01)
```

失能所有电机，释放扭矩。失能后，电机处于自由状态，手臂可以被人手轻易推动。

**注意：** 外层代码（如 `disconnect()`) 通常在失能前会先调用 `parking()` 回到安全位置，防止自由落体。

### 5.3.10 主从模式配置

```python
def set_slave(self):
    self.piper.MasterSlaveConfig(0xFC, 0, 0, 0)

def set_master(self):
    self.piper.MasterSlaveConfig(0xFA, 0, 0, 0)
```

这两个方法用于配置硬件级主从模式——两只 PiPER 机械臂连在同一 CAN 总线上，一只设为主臂（`0xFA`），一只设为从臂（`0xFC`），从臂自动跟随主臂运动。

**当前项目中的应用：**

在 PC 中继遥操作方案中，主臂和从臂各连一条独立的 CAN 总线（`can0` 和 `can1`），电脑作为"中继站"从 `can0` 读取主臂状态，再通过 `can1` 发送命令给从臂。因此 `set_master()` / `set_slave()` 在当前实现中保留但未被调用。它们是为未来可能切换到纯硬件主从方案而预留的接口。

---

## 5.4 继承的方法（框架层）

以下方法和数据类直接继承自基类 `MotorsBus`，PiperMotorsBus 不做任何修改：

### 5.4.1 数据类

**`Motor`** — 描述一个电机的元数据：
```python
@dataclass
class Motor:
    id: int                     # 电机逻辑 ID
    model: str                  # 型号 ("AGILEX-M" 或 "AGILEX-S")
    norm_mode: MotorNormMode    # 归一化模式
```

**`MotorCalibration`** — 一个电机的标定信息：
```python
@dataclass
class MotorCalibration:
    id: int             # 电机 ID
    drive_mode: int     # 驱动模式（PiPER 不使用）
    homing_offset: int  # 回零偏移（PiPER 不使用）
    range_min: int      # 最小原始值（毫度）
    range_max: int      # 最大原始值（毫度）
```

**`MotorNormMode`** — 归一化模式枚举：
```python
class MotorNormMode(str, Enum):
    RANGE_0_100 = "range_0_100"       # 映射到 [0, 100]
    RANGE_M100_100 = "range_m100_100" # 映射到 [-100, 100]
    DEGREES = "degrees"                # 映射到角度制（PiPER 不使用）
```

### 5.4.2 继承的辅助方法

| 方法 | 功能 | PiPER 是否需要 |
|------|------|----------------|
| `_id_to_model(id_)` | 电机 ID → 型号字符串 | 是——用于 `_normalize` 中的 DEGREES 模式 |
| `_id_to_name(id_)` | 电机 ID → 名称 | 是——用于日志和调试 |
| `_get_motor_id(motor)` | 统一获取电机 ID（名称或 ID 均可） | 继承使用 |
| `_get_motor_model(motor)` | 统一获取电机型号 | 继承使用 |
| `_get_motors_list(motors)` | 参数归一化为电机名称列表 | 继承使用 |
| `is_connected` (property) | 检查 CAN 口是否已打开 | 继承使用（依赖 `port_handler.is_open`） |

### 5.4.3 sync_read() — 读取原始值

虽然 `get_action()` 是主要的读取方法，但 `sync_read()` 仍然通过继承保留，用于读取 `Present_Position` 等原始值（不做归一化）。在某些调试场景中很有用：

```python
# 读取原始编码器值（不归一化）
raw_positions = bus.sync_read("Present_Position", normalize=False)
# 返回: {"joint1": 45000, "joint2": 90000, ...}
```

---

## 5.5 空实现的方法（Dynamixel 遗留）

以下是 PiperMotorsBus 中所有空实现/no-op 的方法，以及为什么它们是空的。理解这些"为什么"比"是什么"更重要——它们揭示了 CAN 总线和 Dynamixel 串口在通信模型上的根本差异。

### 5.5.1 方法清单

| 方法 | 返回值 | 为什么是空实现 |
|------|--------|----------------|
| `_assert_protocol_is_compatible()` | `pass` | Dynamixel 需要检查协议版本兼容性（1.0 vs 2.0），CAN 没有"协议版本"概念 |
| `_handshake()` | `pass` | Dynamixel 需要通过 Ping 确认所有电机在线，CAN 的"握手"就是 `openPort()` 成功 |
| `_find_single_motor()` | `pass` | Dynamixel 需要逐个扫描 ID 找到刚出厂的电机（丢失 ID），CAN 上只有一只臂 |
| `broadcast_ping()` | `pass` | Dynamixel 通过广播地址扫描总线上所有舵机，CAN 没有广播寻址机制 |
| `configure_motors()` | `pass` | Dynamixel 需要设置回包延迟、加速度限制等参数，PiPER 的电机参数由 SDK 管理 |
| `read_calibration()` | `pass` | Dynamixel 的标定值存在电机 EEPROM 中需要读取，PiPER 的标定是 Python 代码中硬编码的 |
| `write_calibration()` | `pass` | 同上，PiPER 不需要向硬件写入标定值 |
| `_get_half_turn_homings()` | `pass` | Dynamixel 的半圈回零概念基于 12-bit 编码器（一圈 4096），PiPER 使用毫度单位 |
| `_encode_sign()` | 返回原值 | Dynamixel 需要对某些数据类型做符号编码（补码转换），PiPER 的值已是标准补码 |
| `_decode_sign()` | 返回原值 | 同上，PiPER 不需要额外的符号解码 |
| `_split_into_byte_chunks()` | `pass` | Dynamixel 需要手动拆分整数为字节数组组装串口包，CAN 帧由 SDK 内部处理 |

### 5.5.2 深入理解：为什么 CAN 不需要这些

**1. 不需要 Ping/握手/电机发现 (`_handshake`, `broadcast_ping`, `_find_single_motor`)**

在 Dynamixel 总线上，多个舵机以 Daisy Chain 方式串联。每个舵机有一个唯一的 ID（0-252），但出厂时可能都是 ID=1。因此需要一套"电机发现"协议：先广播 Ping，收到响应的就确认存在；如果冲突（两个舵机都是 ID=1），需要先逐个断电、改 ID，再全部接回。

PiPER 的 CAN 总线上只有一只机械臂。你不需要"发现"它——`openPort()` 成功就意味着通信链路畅通。每个关节数据的 CAN ID 是固定的（由 PiPER 协议定义），不存在"舵机 ID 冲突"问题。

**2. 不需要字节拆分 (`_split_into_byte_chunks`)**

Dynamixel Protocol 2.0 的串口数据包需要手动组装：header (0xFF 0xFF 0xFD) + reserved (0x00) + ID + length + instruction + parameters + CRC。其中 parameters 需要按 little-endian 拆分成字节数组。

PiPER 使用的是 CAN 2.0B 协议。`JointCtrl()` 内部调用 `C_PiperParserV2` 自动将 6 个 `int16` 关节值打包到 3 条 8 字节 CAN 帧中。开发者不需要关心字节序或帧格式。

**3. 不需要符号编解码 (`_encode_sign`, `_decode_sign`)**

某些 Dynamixel 型号的 `Present_Position` 使用特殊编码（如补码格式），需要软件层面做符号转换。PiPER 的关节值就是标准的 `int16` 毫度值，无需额外编码。

**4. 不需要写入硬件标定 (`write_calibration`, `read_calibration`)**

Dynamixel 舵机有 EEPROM 存储区，可以持久化保存标定参数（最小/最大位置限制、回零偏移等）。因此标定流程是：先 `read_calibration()` 从硬件读取已有标定 → 修改 → 再 `write_calibration()` 写回硬件。

PiPER 的标定值是固定的——每个关节的范围由机械设计决定，不会改变。因此标定信息直接以 Python 字典形式硬编码在 `PiperFollower.__init__()` 中，不需要读写硬件。

**5. 不需要角度制模式 (`DEGREES`)**

Dynamixel 的编码器分辨率因型号而异（如 XM430 是 4096 脉冲/圈，XL330 是 4096），`DEGREES` 模式通过 `(val - mid) * 360 / max_res` 公式将编码器脉冲转换为角度制。

PiPER 直接使用毫度单位，所有值已经是角度单位（只是放大了 1000 倍）。`DEGREES` 模式不适用，因此虽然 `_normalize()` 和 `_unnormalize()` 保留了 `DEGREES` 分支，但在 PiPER 上永远不会进入。

### 5.5.3 设计哲学

这些空实现不是"尚未完成"的 TODO，而是一种有意的设计选择——**接口继承 + 空实现**比"创建全新的基类"更好，原因是：

1. **框架兼容性**：`PiperFollower` 等上层代码调用 `bus.broadcast_ping()` 时不会报 `AttributeError`，而是静默跳过。这让 PiPER 版本可以在不修改 LeRobot 框架代码的情况下运行。

2. **遗留代码保护**：某些 LeRobot 工具脚本（如 `lerobot-find-port.py`、`calibrate.py`）可能会调用这些方法。空实现让这些脚本在不经过大改的情况下就能与 PiPER 共存。

3. **未来扩展性**：如果将来 PiPER 支持类似功能（比如通过 CAN 命令读取电机型号），只需在这些空方法中添加实际代码，接口保持不变。

---

## 5.6 归一化数学详解

这是整个系统最核心的数学部分。如果你只能从本章带走一个知识点，就是这一节。

### 5.6.1 为什么需要归一化

**原因一：统一接口**

不同机械臂的编码器使用不同的单位。Dynamixel 舵机用"脉冲"（0-4095），PiPER 用"毫度"（-150000 ~ 150000），UR5 用"弧度"。如果没有归一化，策略模型的输入/输出格式与机械臂型号强绑定，切换硬件平台等于重写全部代码。

归一化后，**所有机械臂的策略输入都是 [-100, 100]**。模型不需要知道底层用的是毫度还是脉冲还是弧度。

**原因二：数值稳定性**

神经网络训练对输入数值范围敏感。如果 joint1 的输入范围是 [-150000, 150000] 而 joint3 的是 [-170000, 0]，量纲差异巨大。优化器（如 Adam）对不同量纲的梯度缩放不均匀，导致训练困难。归一化到固定范围消除这种不均衡。

**原因三：迁移学习（理论）**

如果两个机械臂的结构相似（如都是 6 轴串联），在机械臂 A 上训练的策略原则上可以迁移到机械臂 B 上推理。归一化提供了这种可能性——策略只学习"相对位置"而不是"绝对编码器值"。

### 5.6.2 RANGE_M100_100 归一化（关节，6 个）

**公式：**

$$\text{norm} = \frac{\text{raw} - \text{min}}{\text{max} - \text{min}} \times 200 - 100$$

**分步解释：**

```
步骤 1: (raw - min) / (max - min)    → 映射到 [0, 1]    （归一化到 0-1 区间）
步骤 2: × 200                        → 映射到 [0, 200]  （拉伸到 0-200）
步骤 3: - 100                        → 映射到 [-100, 100]（平移到对称区间）
```

**边界验证：**

```
当 raw = min  →  norm = ((min - min) / (max - min)) × 200 - 100
                       = 0 × 200 - 100
                       = -100  ✓

当 raw = max  →  norm = ((max - min) / (max - min)) × 200 - 100
                       = 1 × 200 - 100
                       = 100   ✓

当 raw = (min + max) / 2  →  norm = (0.5 × 200) - 100
                                   = 100 - 100
                                   = 0     ✓  中位映射到 0
```

### 5.6.3 RANGE_M100_100 反归一化

**公式：**

$$\text{raw} = \left\lfloor \frac{\text{norm} + 100}{200} \times (\text{max} - \text{min}) + \text{min} \right\rceil$$

实际代码中 `round()` 由 `int()` 的截断实现（因为经过钳位后值为正且加上 min 后范围合理）：

```python
raw = int(((bounded_val + 100) / 200) * (max - min) + min)
```

**分步解释：**

```
步骤 1: (norm + 100) / 200    → 从 [-100,100] 映射回 [0, 1]
步骤 2: × (max - min)         → 拉伸到原始范围宽度
步骤 3: + min                 → 平移到原始坐标系
步骤 4: int(...)              → 转为整数（毫度）
```

**边界验证：**

```
当 norm = -100  →  raw = int(((-100 + 100) / 200) × (max - min) + min)
                        = int(0 + min)
                        = min  ✓

当 norm = 100   →  raw = int(((100 + 100) / 200) × (max - min) + min)
                        = int(1.0 × (max - min) + min)
                        = max  ✓

当 norm = 0     →  raw = int((100 / 200) × (max - min) + min)
                        = int(0.5 × (max - min) + min)
                        = (min + max) / 2  ✓  0 映射到中点
```

### 5.6.4 RANGE_0_100 归一化（夹爪，1 个）

**归一化公式：**

$$\text{norm} = \frac{\text{raw} - \text{min}}{\text{max} - \text{min}} \times 100$$

**反归一化公式：**

$$\text{raw} = \left\lfloor \frac{\text{norm}}{100} \times (\text{max} - \text{min}) + \text{min} \right\rceil$$

代码：
```python
# 归一化
norm = ((bounded_val - min_) / (max_ - min_)) * 100

# 反归一化
raw = int((bounded_val / 100) * (max_ - min_) + min_)
```

**与 RANGE_M100_100 的核心区别：** 输出范围是 [0, 100] 而不是 [-100, 100]。这是因为夹爪只有"开"和"闭"两个方向（开口大小是单向的），不像关节有正负两个运动方向。

**代码中的安全钳位：**

在归一化（正向）中：
```python
bounded_val = min(max_, max(min_, val))  # 将原始值钳位在 [min, max] 内
```

在反归一化（逆向）中：
```python
# RANGE_M100_100:
bounded_val = min(100.0, max(-100.0, val))  # 钳位在 [-100, 100]

# RANGE_0_100:
bounded_val = min(100.0, max(0.0, val))     # 钳位在 [0, 100]
```

两层安全钳位保证了即使输入值异常（传感器跳变、模型输出越界），也不会发出超出机械限位的命令。

### 5.6.5 带具体数值的完整计算示例

以 **joint1** 为例，走一遍完整的归一化+反归一化流程。

**给定参数：**

- `calibration["joint1"]`: `range_min = -150000`, `range_max = 150000`（即 ±150°）
- `motors["joint1"].norm_mode`: `RANGE_M100_100`
- 假设 SDK 读到的编码器反馈值：`raw = 45000`（即 45.0°）

**归一化计算：**

```
norm = ((45000 - (-150000)) / (150000 - (-150000))) × 200 - 100
     = (195000 / 300000) × 200 - 100
     = 0.65 × 200 - 100
     = 130 - 100
     = 30.0
```

**验证物理意义：** 45° 是 ±150° 范围的 65% 位置（从 -150° 算起：-150 + 300×0.65 = 45 ✓）。65% × 200 - 100 = 30 ✓。

**反归一化验证：**

```
raw = int(((30.0 + 100) / 200) × (150000 - (-150000)) + (-150000))
    = int((130 / 200) × 300000 - 150000)
    = int(0.65 × 300000 - 150000)
    = int(195000 - 150000)
    = int(45000)
    = 45000  ✓
```

往返一致：45000 → 30.0 → 45000。

**以 joint3 为例**（不对称、负向范围）：

- `range_min = -170000`, `range_max = 0`（范围为 [-170°, 0°]）
- 假设 `raw = -85000`（即 -85°，范围中点）

```
norm = ((-85000 - (-170000)) / (0 - (-170000))) × 200 - 100
     = (85000 / 170000) × 200 - 100
     = 0.5 × 200 - 100
     = 0.0
```

中点映射到 0.0 ✓。虽然 joint3 的原始值全是负数（-170000 到 0），但归一化后中间位置仍然是 0。

**以 gripper 为例**（RANGE_0_100 模式）：

- `range_min = 0`, `range_max = 68000`（范围为 [0°, 68°]）
- 假设 `raw = 34000`（即 34°，半开）

```
norm = ((34000 - 0) / (68000 - 0)) × 100
     = 0.5 × 100
     = 50.0
```

半开位置映射到 50.0 ✓。

### 5.6.6 归一化在数据 Pipeline 中的位置

```
机械臂编码器 (毫度整数)
    │
    ▼ _normalize()
[-100, 100] / [0, 100] 浮点
    │
    ▼ 存入数据集 (Parquet)
训练样本
    │
    ▼ 模型输入（归一化值）
策略模型
    │
    ▼ 模型输出（归一化值）
[-100, 100] / [0, 100] 浮点
    │
    ▼ _unnormalize()
机械臂执行 (毫度整数)
    │
    ▼ 驱动 CAN 帧
电机物理运动
```

关键洞察：**训练数据和推理数据使用完全相同的归一化公式**。如果训练时 joint1 的范围是 [-150000, 150000]，推理时也必须是这个范围。改变校准参数会导致"概念偏移"（concept drift）——模型学到的 30.0 对应 45°，但你改变校准后 30.0 可能对应不同的角度，机械臂的行为就会出错。

---

## 5.7 校准范围表

以下是 PiPER 机械臂所有 7 个驱动电机的校准范围。这些值来自 PiPER 的机械结构设计——由关节的物理限位和驱动器软限位共同决定。

### 5.7.1 完整校准表

| 关节 | 电机型号 | Motor ID | 归一化模式 | MIN (毫度) | MAX (毫度) | 角度范围 | 范围宽度 (度) |
|------|----------|----------|------------|-----------|-----------|----------|--------------|
| joint1 | AGILEX-M | 1 | RANGE_M100_100 | -150000 | 150000 | ±150° | 300° |
| joint2 | AGILEX-M | 2 | RANGE_M100_100 | 0 | 180000 | 0 ~ 180° | 180° |
| joint3 | AGILEX-M | 3 | RANGE_M100_100 | -170000 | 0 | -170° ~ 0° | 170° |
| joint4 | AGILEX-S | 4 | RANGE_M100_100 | -100000 | 100000 | ±100° | 200° |
| joint5 | AGILEX-S | 5 | RANGE_M100_100 | -65000 | 65000 | ±65° | 130° |
| joint6 | AGILEX-S | 6 | RANGE_M100_100 | -100000 | 130000 | -100° ~ 130° | 230° |
| gripper | AGILEX-S | 7 | RANGE_0_100 | 0 | 68000 | 0 ~ 68° | 68° |

### 5.7.2 电机型号分布

PiPER 机械臂使用两种型号的电机：

- **AGILEX-M**（3 个）：用于底座 3 个关节（joint1, joint2, joint3）——负载大，运动范围大，需要更大的扭矩。
- **AGILEX-S**（4 个）：用于腕部 3 个关节和夹爪（joint4, joint5, joint6, gripper）——负载小，对精度要求更高。

### 5.7.3 特殊范围解读

**joint2 为什么 min=0？**

joint2 是大臂的俯仰关节。它的机械结构不允许向后弯曲——大臂只能从垂直状态向前倾，不能向后仰（否则会撞到机械臂底座）。因此它的范围是单边正向 [0°, 180°]，min=0。在归一化时，0° 对应 norm=-100，180° 对应 norm=100。

**joint3 为什么全是负数？**

joint3 是小臂（肘关节）的俯仰关节。它的机械结构允许向下弯曲最多 170°，但无法向上弯曲到超过大臂延长线（0° 被定义为与大臂共线）。因此它的范围是 [-170°, 0°]。虽然全是负数，归一化后中位（-85°）仍然映射到 norm=0。

**joint4 为什么是 ±100°？**

joint4 是腕部旋转关节，负责手腕的 pronation/supination（旋前/旋后）。机械设计上可以有近 200° 的旋转范围。注意这个关节的旋转轴是沿小臂方向的，与其他俯仰关节不同。

**joint5 为什么范围最小（±65°）？**

joint5 是腕部俯仰关节。它的运动范围受限于腕部的机械结构和夹爪安装位置。过大的俯仰角度可能导致夹爪与机械臂本体碰撞。

**joint6 为什么不对称（-100° ~ 130°）？**

joint6 是腕部旋转关节（末端法兰旋转）。它的正负不对称可能是因为：
- 正向（+130°）旋转时不会拉扯内部线缆
- 负向（-100°）旋转时，内部线缆和气管的盘绕角度有限

**gripper 为什么使用 RANGE_0_100？**

夹爪只有"开"和"闭"两个方向，不像关节有正负两个运动方向。因此使用 RANGE_0_100：0 表示全开，100 表示全闭。夹爪范围宽度 68° 意味着手指从完全打开到完全闭合运动 68°（毫度 68000）。

### 5.7.4 归一化中点对应的物理角度

| 关节 | 归一化值 0 对应的原始值 | 对应的物理角度 |
|------|------------------------|----------------|
| joint1 | 0 | 0°（中立，朝正前方） |
| joint2 | 90000 | 90°（水平前伸） |
| joint3 | -85000 | -85°（半弯） |
| joint4 | 0 | 0°（中立旋转位） |
| joint5 | 0 | 0°（中立腕部） |
| joint6 | 15000 | 15°（微偏） |
| gripper | 34000 | 34°（半开） |

---

## 5.8 tables.py 内容

`tables.py` 是 PiPER 电机的常量定义文件，虽然只有约 90 行，但包含了电机控制所必需的全部参数表。

### 5.8.1 MODEL_RESOLUTION_TABLE — 编码器分辨率

```python
MODEL_RESOLUTION_TABLE = {
    "AGILEX-M": 4096,
    "AGILEX-S": 4096,
}
```

两种型号的编码器分辨率都是 4096 脉冲/转。这个值只在 `DEGREES` 归一化模式下使用（`max_res = model_resolution - 1 = 4095`），PiPER 当前不使用该模式，因此这个表虽定义但实际未参与计算。

### 5.8.2 MODEL_NUMBER_TABLE — 型号编号

```python
MODEL_NUMBER_TABLE = {
    "AGILEX-M": 1190,
    "AGILEX-S": 1191,
}
```

每个电机型号在 Dynamixel 协议中对应一个唯一的型号编号。虽然 PiPER 不使用 Dynamixel 协议，但保留这些定义是为了框架兼容性（`MotorsBus.__init__` 中的 `_validate_motors()` 和 `_model_nb_to_model_dict` 会用到）。

### 5.8.3 INITIALIZE_POSITION — 初始化位置

```python
INITIALIZE_POSITION = {
    "joint1": 0,
    "joint2": 0,
    "joint3": 0,
    "joint4": 0,
    "joint5": 0,
    "joint6": 0,
    "gripper": 0,
}
```

这是 `parking()` 方法使用的目标位置。所有值均为 0（归一化值），对应各关节的中立位置和夹爪的全开状态。注意：这里的 0 是归一化值，不是原始毫度值。在 `norm=0` 的情况下：

- joint1: 原始值 = 0 毫度 = 0°（中立）
- joint2: 原始值 = 90000 毫度 = 90°（前伸）
- joint3: 原始值 = -85000 毫度 = -85°（半弯）
- joint4: 原始值 = 0 毫度 = 0°
- joint5: 原始值 = 0 毫度 = 0°
- joint6: 原始值 = 15000 毫度 = 15°
- gripper: 原始值 = 0 毫度 = 0°（全开）

### 5.8.4 MODEL_BAUDRATE_TABLE — 波特率映射

```python
MODEL_BAUDRATE_TABLE = {
    1_000_000: 3,
    2_000_000: 4,
    3_000_000: 5,
    4_000_000: 6,
}
```

将波特率（bps）映射到 Dynamixel 协议中的波特率编号。PiPER 使用 CAN 总线，波特率固定为 1Mbps 或 500kbps，这个表保留为框架兼容性。

### 5.8.5 其他表

```python
AVAILABLE_BAUDRATES = [9600, 19200, ..., 4000000]  # 支持的全部波特率列表
MODEL_ENCODING_TABLE = {"AGILEX-M": {}, "AGILEX-S": {}}  # 编码参数（空）
MODEL_CONTROL_TABLE = {"AGILEX-M": 1190, "AGILEX-S": 1191}  # 控制表定义
MODEL_OPERATING_MODES = {
    "AGILEX-M": [0, 1, 3, 4, 5, 16],
    "AGILEX-S": [0, 1, 3, 4, 5, 16],
}  # 支持的操作模式
```

这些表中大部分是 Dynamixel 协议的遗留定义，保留在 PiPER 版本中是为了保持框架兼容性。`MODEL_CONTROL_TABLE` 实际上存储的是型号编号而非控制表定义，但变量名与 LeRobot 基类的 `model_ctrl_table` 保持一致。

---

## 5.9 get_action() vs set_action() 的完整时序

### 5.9.1 数据采集中一个帧周期的时序图

在 `lerobot-record`（数据采集）模式中，每一帧（约 30 FPS，即 33ms 一个周期）执行以下操作：

```
时间轴 →
  0ms                                                              33ms
  ├─────────────────────────────────────────────────────────────────┤
  │                                                                  │
  │ [读周期 ~0.2ms]                                                 │
  │  ├─ piper.GetArmJointMsgs()     ← 从 ReadCan 缓存读，≈0ms       │
  │  ├─ piper.GetArmGripperMsgs()   ← 同上                          │
  │  ├─ _normalize() × 7            ← 纯 CPU 计算，≈0.01ms          │
  │  └─ 组装 observation dict                                      │
  │                                                                  │
  │ [写周期 ~4ms]                                                   │
  │  ├─ _unnormalize() × 7          ← 纯 CPU 计算，≈0.01ms          │
  │  ├─ piper.ModeCtrl()            ← CAN 发送 0x151，≈0.1ms        │
  │  ├─ piper.JointCtrl()           ← CAN 发送 0x155/156/157，≈0.3ms│
  │  ├─ piper.GripperCtrl()         ← CAN 发送 0x159，≈0.1ms        │
  │  └─ get_control() 确认          ← 读缓存，≈0ms                  │
  │                                                                  │
  │ [相机周期 ~30ms（异步进行）]                                     │
  │  ├─ wrist_camera.async_read()   ← 硬件帧捕获 + USB 传输         │
  │  └─ global_camera.async_read()  ← 同上                          │
  │                                                                  │
  │ [等待] ~29ms 的空闲时间（等待下一帧触发）                        │
  │                                                                  │
  └─ 下一帧开始
```

**关键时间数据：**

| 操作 | 耗时 | 占比 |
|------|------|------|
| 读取关节状态（缓存读） | < 0.1ms | < 0.3% |
| 归一化 + 反归一化计算 | < 0.02ms | < 0.1% |
| CAN 帧发送（4 条） | ~0.5ms | ~1.5% |
| 相机帧捕获（异步） | ~30ms | ~91% |
| 空闲等待 | ~3ms | ~9% |

**瓶颈分析：**

显而易见，**相机采集是整个 Pipeline 的瓶颈**。CPU 计算（归一化/反归一化）和 CAN 通信的时间几乎可以忽略不计。这也是为什么 `get_action()` 中读取的是 ReadCan 线程的缓存值而不是实时触发一次 CAN 读取——在 33ms 的帧周期面前，缓存数据的新鲜度完全充足。

### 5.9.2 策略推理中一个帧周期的时序图

在 `lerobot-eval`（策略推理/部署）模式中，timeline 稍有不同：

```
时间轴 →
  0ms                                                              33ms
  ├─────────────────────────────────────────────────────────────────┤
  │                                                                  │
  │ [读周期] get_action() → 获取当前关节状态 + 相机图像              │
  │                                                                  │
  │ [推理] policy.inference(observation) → action_dict               │
  │        ★ 这是新增加的时间。取决于模型大小和硬件：                │
  │        ├─ ACT (GPU): ~5-10ms                                    │
  │        ├─ Diffusion Policy (GPU): ~20-50ms                      │
  │        └─ VQ-BeT (GPU): ~10-20ms                                │
  │                                                                  │
  │ [写周期] set_action(action_dict) → 发送命令给机械臂             │
  │                                                                  │
  └─ 下一帧开始
```

与 Record 模式的核心区别：
- Record 模式：写周期发送的是**来自主臂的动作**（通过 PC 中继转发），没有推理开销
- Eval 模式：写周期发送的是**策略模型的推理输出**，增加了推理时间

对于 Diffusion Policy 这种较大的模型（50ms 推理时间），33ms 帧周期可能不够。此时需要降帧率（如 15 FPS）或使用异步推理。这也是为什么 LeRobot 支持数据集的异步重放——某些策略模型达不到实时 30 FPS。

### 5.9.3 CAN 总线利用率分析

在 30 FPS 的帧率下，每秒发送的 CAN 帧数量：

| CAN 帧 | CAN ID | 每帧调用数/秒 | 每秒总帧数 | 每帧比特数 | 每秒比特数 |
|--------|--------|--------------|-----------|-----------|-----------|
| ModeCtrl | 0x151 | 30 | 30 | ~100 | 3,000 |
| JointCtrl | 0x155 | 30 | 30 | ~100 | 3,000 |
| JointCtrl | 0x156 | 30 | 30 | ~100 | 3,000 |
| JointCtrl | 0x157 | 30 | 30 | ~100 | 3,000 |
| GripperCtrl | 0x159 | 30 | 30 | ~100 | 3,000 |
| **主动上报帧** | 0x201-0x20A | 200 | 200×10 | ~100 | 200,000 |
| **合计** | | | 2,150 | | ~215,000 |

CAN 1Mbps 的带宽利用率 = 215,000 / 1,000,000 ≈ 21.5%。即使加上帧间隔（3 bit intermission）、填充位（bit stuffing）等开销，利用率也在 30% 以下，远未达到 CAN 总线的饱和点（通常 60-70% 为安全上限）。因此 CAN 带宽不是瓶颈。

---

## 5.10 完整代码走读

本节逐段走读 `piper.py` 的全部约 700 行代码，解释每一部分的用途、设计意图和与其他模块的交互。

### 5.10.1 文件头注释与版权 (第 1-32 行)

```
文件：PiperMotorsBus — LeRobot 与 PiPER 机械臂硬件之间的翻译器 + 搬运工
```

文件头的 ASCII 图示精确描述了本文件在整个系统中的位置——它是策略模型输出与真实机械臂运动之间的桥梁。依赖关系注释标注了四个关键外部模块：`piper_sdk`、`wego_piper`、`.tables` 和 `..motors_bus`。

### 5.10.2 导入部分 (第 34-81 行)

**标准库导入：** `time`（sleep 和超时控制）、`logging`（日志）、`typing.Any`（类型注解）。

**LeRobot 框架导入：**
```python
from ..motors_bus import Motor, MotorCalibration, MotorsBus, MotorNormMode, NameOrID, Value, get_address
```
从上层 `motors_bus.py` 导入基类和核心数据类型。`Motor` 和 `MotorCalibration` 是数据类，`MotorsBus` 是抽象基类，`MotorNormMode` 是枚举。

**PiPER SDK 导入：**
```python
from piper_sdk import *
```
导入所有 AgileX 官方 SDK 的公开符号。核心符号包括 `C_PiperInterface_V2`（主接口类）、`C_PiperInterface`（V1 版本）和 `LogLevel`（日志级别枚举）。

**第三方库导入：**
```python
from wego_piper.port_handler import PortHandler
```
`PortHandler` 来自 `wego_piper` 包，封装了 CAN 口的三个基本操作（setup/open/close）。

**常量表导入：**
```python
from .tables import (
    AVAILABLE_BAUDRATES, MODEL_BAUDRATE_TABLE, MODEL_CONTROL_TABLE,
    MODEL_ENCODING_TABLE, MODEL_NUMBER_TABLE, MODEL_RESOLUTION_TABLE,
    INITIALIZE_POSITION,
)
```

### 5.10.3 类定义与类属性 (第 89-120 行)

```python
class PiperMotorsBus(MotorsBus):
```

类属性的作用域是类级别（所有实例共享）。关键类属性：

- `available_baudrates = [500_000, 1_000_000]`：覆盖基类的长列表，因为 CAN 只支持这两种波特率。
- `default_timeout = 1000`：超时 1 秒。
- `apply_drive_mode = False`：核心设计选择，让 `_normalize` / `_unnormalize` 中的 `drive_mode` 分支永不触发。
- `normalized_data = ["Present_Position", "Goal_Position"]`：标记哪些数据字段名需要经过归一化/反归一化处理。
- `model_baudrate_table` / `model_ctrl_table` / `model_encoding_table` / `model_number_table` / `model_resolution_table`：从 `tables.py` 导入的电机参数表，赋值为类属性供父类方法引用。

### 5.10.4 __init__() (第 125-170 行)

```python
def __init__(self, id, port, motors, calibration=None):
    super().__init__(port, motors, calibration)
    self.port_handler = PortHandler()
    self.id = id
    self.piper = C_PiperInterface_V2(port)
```

**调用顺序分析：**

1. `super().__init__(port, motors, calibration)` — 先调用父类构造函数，它会：
   - 保存 `self.port`, `self.motors`, `self.calibration`
   - 构建 `_id_to_model_dict` 和 `_id_to_name_dict` 映射表
   - 调用 `_validate_motors()` 检查电机 ID 唯一性和控制表存在性
2. 创建 `PortHandler` 实例（暂不打开任何端口）
3. 保存 `self.id` 用于日志和校准文件路径
4. 创建 `C_PiperInterface_V2(port)` 实例（暂不连接 CAN 口）

**注意：** `C_PiperInterface_V2` 使用单例模式（按 `can_name`）。如果同一个 CAN 口已有其他代码创建了 `C_PiperInterface_V2` 实例，`C_PiperInterface_V2(port)` 会返回已缓存的实例。这种行为在单臂场景下没问题，但在双臂 PC 中继场景（`can0` 和 `can1` 各一个实例）中很重要——每个 `PiperMotorsBus` 持有对应 CAN 口的独立 SDK 实例。

### 5.10.5 连接与断开 (第 174-211 行)

```python
def _assert_protocol_is_compatible(self, instruction_name):
    pass  # PiPER 不需要协议兼容性检查

def _handshake(self):
    pass  # PiPER 不需要握手协议

def _find_single_motor(self, motor, initial_baudrate):
    pass  # PiPER 不需要逐电机扫描
```

三个空实现已在 5.5 节详细解释过。它们被保留是因为它们是 `MotorsBus` 的抽象方法，必须实现，但 PiPER 不需要这些功能。

```python
def connect(self, handshake: bool = True) -> bool:
    self.port_handler.setupPort(self.piper)
    return self.port_handler.openPort()
```

极其简洁的 `connect()`——只做 CAN 口的设置和打开。`handshake` 参数接受但不使用（保持接口兼容）。返回 `True`（成功）或 `False`（失败）。

```python
def disconnect(self, disable_torque: bool = False) -> None:
    if disable_torque:
        self.parking()
        self.piper.DisablePiper()
    self.port_handler.closePort()
```

`disable_torque` 默认为 `False`，意味着默认断开连接时**不**失能电机——这适用于程序临时重启场景。

### 5.10.6 parking() 与 clear_gripper() (第 216-244 行)

```python
def clear_gripper(self):
    self.piper.GripperCtrl(0, 1000, 0x03, 0)
```

跳过归一化流程，直接发送 SDK 原始命令。`0` 是目标毫度值（夹爪全开），`1000` 是速度参数，`0x03` 是控制模式。

```python
def parking(self):
    timeout = 100
    self.set_action(INITIALIZE_POSITION)
    time.sleep(0.1)
    status = self.piper.GetArmStatus()
    while status.arm_status.motion_status and timeout:
        self.set_action(INITIALIZE_POSITION)
        time.sleep(0.1)
        status = self.piper.GetArmStatus()
        timeout -= 1
```

使用 `set_action()` 而非直接调 SDK，是因为 `INITIALIZE_POSITION` 是归一化值（全 0），需要通过 `_unnormalize()` 转换为原始值再发送。循环重发的目的已在 5.3.6 节详细讨论。

### 5.10.7 _normalize() (第 274-332 行)

```python
def _normalize(self, ids_values: dict[int, int]) -> dict[int, float]:
```

这个方法的实现与父类 `MotorsBus._normalize()` 相同，但被覆写到了子类中。覆写的原因是两个细微但重要的差异：

1. **键的类型**：在父类中，`_normalize` 使用 `self._id_to_name(id_)` 将整数 ID 转为字符串名称来访问 `self.calibration`。在覆写版本中，直接使用整数 ID 访问 `self.calibration[motor]`——因为 PiPER 的 calibration 字典以整数 ID 为键。

2. **DEGREES 模式访问**：`self.model_resolution_table[self._id_to_model(id_)]` —— PiPER 版本使用整数 ID 而非字符串名称来查找模型。

方法内部逻辑（三个分支已在 5.6 节详细推导过）：
- `RANGE_M100_100`：`norm = ((bounded - min) / (max - min)) * 200 - 100`
- `RANGE_0_100`：`norm = ((bounded - min) / (max - min)) * 100`
- `DEGREES`：`norm = (val - mid) * 360 / max_res`（PiPER 不使用）

安全钳位 `bounded_val = min(max_, max(min_, val))` 在所有分支之前统一执行。

### 5.10.8 _unnormalize() (第 334-396 行)

```python
def _unnormalize(self, ids_values: dict[int, float]) -> dict[int, int]:
```

与 `_normalize()` 的对称方法。同样有三个分支，每个分支中先钳位输入值再计算输出：

- `RANGE_M100_100`：`bounded = clamp(val, -100, 100)`, `raw = int(((bounded + 100) / 200) * (max - min) + min)`
- `RANGE_0_100`：`bounded = clamp(val, 0, 100)`, `raw = int((bounded / 100) * (max - min) + min)`
- `DEGREES`：`raw = int((val * max_res / 360) + mid)`

反归一化中 `int()` 的截断行为：对于正数结果，`int()` 等价于 `floor()`，会产生最多 1 毫度（0.001°）的截断误差。这在实践中完全可以忽略。

### 5.10.9 enable_torque() / disable_torque() (第 410-453 行)

```python
def enable_torque(self, motors=None, num_retry=0) -> bool:
    retry = 10
    while not self.piper.EnablePiper() and retry:
        retry -= 1
        time.sleep(0.1)
    ...
    return True if retry else False
```

```python
def disable_torque(self, motors=None, num_retry=0) -> None:
    while self.piper.DisablePiper() and num_retry:
        num_retry -= 1
        time.sleep(0.01)
```

`_disable_torque(self, motor, model, num_retry)` 调用 `self.piper.DisableArm(motor)` 尝试禁用单个电机——但 PiPER SDK 实际上不支持逐个电机禁用（要么全使能，要么全失能），所以这个方法主要是接口占位。

### 5.10.10 get_action() (第 459-497 行)

已在 5.3.3 节完整讲解。核心数据流：`GetArmJointMsgs() / GetArmGripperMsgs()` → 提取字段 → `_normalize()` → 返回。

### 5.10.11 get_control() (第 499-525 行)

已在 5.3.4 节完整讲解。与 `get_action()` 的区别在于数据来源（`GetArmJointCtrl()` vs `GetArmJointMsgs()`）和是否归一化。

### 5.10.12 set_action() (第 530-583 行)

已在 5.3.5 节完整讲解。核心数据流：`_unnormalize()` → `ModeCtrl()` → `JointCtrl()` → `GripperCtrl()` → `get_control()`。

### 5.10.13 主从模式方法 (第 603-609 行)

```python
def set_slave(self):
    self.piper.MasterSlaveConfig(0xFC, 0, 0, 0)

def set_master(self):
    self.piper.MasterSlaveConfig(0xFA, 0, 0, 0)
```

已在 5.3.10 节讲解。`0xFC` 表示从臂模式，`0xFA` 表示主臂模式。

### 5.10.14 剩余空实现方法 (第 617-664 行)

```python
def _get_half_turn_homings(self, positions): pass
def _encode_sign(self, data_name, ids_values): return ids_values
def _decode_sign(self, data_name, ids_values): return ids_values
def _split_into_byte_chunks(self, value, length): pass
def broadcast_ping(self, ...): pass
def configure_motors(self): pass
def read_calibration(self): pass

@property
def is_calibrated(self) -> bool:
    return True

def write_calibration(self, ...): pass
```

所有空实现已在 5.5 节解释过。特别值得再次强调 `is_calibrated` 始终返回 `True`——PiPER 的标定是代码中硬编码的，不需要从硬件 EEPROM 读取，因此"是否已标定"这个问题永远回答"是"。

### 5.10.15 __main__ 测试代码 (第 670-705 行)

```python
if __name__ == "__main__":
    piper = C_PiperInterface(
        can_name="can0",
        judge_flag=False,
        can_auto_init=True,
        dh_is_offset=1,
        start_sdk_joint_limit=False,
        start_sdk_gripper_limit=False,
        logger_level=LogLevel.WARNING,
        log_to_file=False,
        log_file_path=None,
    )
    piper.ConnectPort()
    while True:
        print(piper.GetArmJointMsgs())
        time.sleep(0.005)  # 5ms = 200Hz
```

这是一个独立的最小示例，不依赖 LeRobot 框架。直接运行本文件可以快速验证 piper_sdk 是否能正常连接和读取。注意这里使用的是 `C_PiperInterface`（V1），不是 `PiperMotorsBus` 中使用的 `C_PiperInterface_V2`——两个版本的 API 大体相似。

**重要提示：** 第一帧 `GetArmJointMsgs()` 数据是全 0（默认值），之后才是真实编码器反馈。这是因为 SDK 的 `ConnectPort()` 需要启动 `ReadCan` 线程并等待第一帧 CAN 数据到达，期间结构体保持默认初始化值。

### 5.10.16 方法与调用关系总结

```
外部调用 (PiperFollower)
  │
  ├── connect()  ──────────────────────────────────────────────
  │     ├── port_handler.setupPort(piper)         设置 CAN 参数
  │     └── port_handler.openPort()               打开 CAN 口
  │
  ├── enable_torque()
  │     └── piper.EnablePiper()                   使能电机 (CAN 0x471)
  │
  ├── get_action()  ─── 每帧调用 ────────────────────────────────
  │     ├── piper.GetArmJointMsgs()               读关节反馈
  │     ├── piper.GetArmGripperMsgs()             读夹爪反馈
  │     └── _normalize(ids_values)                归一化
  │           └── 对每个 motor 应用公式
  │
  ├── set_action(action)  ─── 每帧调用 ────────────────────────
  │     ├── _unnormalize(action)                  反归一化
  │     │     └── 对每个 motor 应用公式
  │     ├── piper.ModeCtrl(...)                   设置控制模式 (CAN 0x151)
  │     ├── piper.JointCtrl(j1..j6)               发送关节目标 (CAN 0x155-157)
  │     ├── piper.GripperCtrl(...)                发送夹爪目标 (CAN 0x159)
  │     └── get_control()                         返回控制值确认
  │
  ├── parking()
  │     ├── set_action(INITIALIZE_POSITION)       发送初始位置
  │     ├── piper.GetArmStatus()                  检查运动状态
  │     └── 循环直到停止或超时
  │
  ├── disconnect(disable_torque)
  │     ├── parking()                             回零 (可选)
  │     ├── piper.DisablePiper()                  失能 (可选)
  │     └── port_handler.closePort()              关闭 CAN 口
  │
  └── clear_gripper()
        └── piper.GripperCtrl(0, 1000, 0x03, 0)  直接打开夹爪
```

---

## 5.11 本章总结

### 5.11.1 核心概念回顾

1. **适配器模式**：`PiperMotorsBus` 是一个硬件适配器，把 CAN 总线操作翻译成 LeRobot 期望的 Dynamixel 风格接口。适配发生在三个层面——硬件通信层（CAN vs 串口）、数据格式层（毫度整数 vs 归一化浮点）、接口语义层（保持 get/set_action 的调用契约不变）。

2. **归一化是数学核心**：`_normalize()` 和 `_unnormalize()` 完成了 `[-100, 100] / [0, 100]` 与毫度整数之间的双向映射。6 个关节使用 `RANGE_M100_100`（对称区间），夹爪使用 `RANGE_0_100`（单向区间）。

3. **空实现不是偷懒**：近一半的方法是空实现，这是因为 CAN 总线的通信模型和 Dynamixel 串口完全不同——不需要 Ping、不需要握手、不需要字节拆分、不需要发现电机。

4. **两层分离架构**：Bus 层（`connect`/`disconnect`）只管 CAN 通信链路，Robot 层（`enable_torque`/`disable_torque`/`parking`）管电机使能和初始化。

5. **标定是硬编码的**：PiPER 的关节范围由机械结构决定，不会改变。标定字典在 Python 代码中定义，不需要像 Dynamixel 那样从硬件 EEPROM 读写。因此 `is_calibrated` 始终返回 `True`。

### 5.11.2 关键公式速查

| 模式 | 方向 | 公式 |
|------|------|------|
| RANGE_M100_100 | 归一化 | `norm = ((raw - min) / (max - min)) * 200 - 100` |
| RANGE_M100_100 | 反归一化 | `raw = int(((norm + 100) / 200) * (max - min) + min)` |
| RANGE_0_100 | 归一化 | `norm = ((raw - min) / (max - min)) * 100` |
| RANGE_0_100 | 反归一化 | `raw = int((norm / 100) * (max - min) + min)` |

### 5.11.3 下一步

本章完整讲解了 `PiperMotorsBus`——整个项目中最重要的 700 行代码。你现在应该能够：

- 追踪从策略模型输出到电机物理运动的每一步
- 理解 CAN 总线和 Dynamixel 串口在通信模型上的根本差异
- 手写任意关节的归一化/反归一化计算
- 解释为什么某些方法是空实现

下一章将进入上层——`PiperFollower` 和 `PiperLeader` 类，它们基于 `PiperMotorsBus` 构建了完整的机器人控制逻辑，包括数据采集循环、策略推理循环和 PC 中继遥操作。
