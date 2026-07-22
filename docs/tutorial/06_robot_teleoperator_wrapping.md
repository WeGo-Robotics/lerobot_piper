# 第六章 Robot 与 Teleoperator 封装详解

本章深入剖析 PiPER 机械臂在 LeRobot 框架中的两层封装——PiperFollower（从臂/Robot）和 PiperLeader（主臂/Teleoperator）。阅读本章后，你将理解 LeRobot 如何将异构硬件抽象为统一的 `get_observation()` / `send_action()` 接口，掌握电机配置、校准、特征定义、注册机制等核心概念，并获得直接使用这些 Python 类编程的能力。

本章内容覆盖以下四个源文件：

| 文件 | 行数 | 角色 |
|------|------|------|
| `lerobot/robots/piper_follower/piper_follower.py` | 187 | 从臂驱动：采集观测、执行动作 |
| `lerobot/robots/piper_follower/config_piper_follower.py` | 43 | 从臂配置：CAN 口、相机、安全限幅 |
| `lerobot/teleoperators/piper_leader/piper_leader.py` | 111 | 主臂驱动：读取示教位置 |
| `lerobot/teleoperators/piper_leader/config_piper_leader.py` | 30 | 主臂配置：CAN 口、夹爪张开位置 |

---

## 6.1 Robot vs Teleoperator：概念辨析

### 6.1.1 物理角色的差异

在模仿学习场景中，一次完整的示教涉及两台机械臂，它们的物理角色截然不同：

```
┌──────────────────────────────────────────────────────────────────┐
│                        示教场景示意图                              │
│                                                                  │
│    ┌──────────────┐                      ┌──────────────┐        │
│    │   主臂 Leader  │  人手拖动，产生位置    │  从臂 Follower │        │
│    │  (Teleoperator) │ ◄══════════════════► │    (Robot)    │        │
│    │              │     PC 中继转发动作      │              │        │
│    │  无相机       │                      │  有相机        │        │
│    │  只输出动作    │                      │  输出观测+接收动作│        │
│    │  被操作者拖动  │                      │  被策略/主臂驱动│        │
│    └──────────────┘                      └──────────────┘        │
│                                                                  │
│     数据流向：主臂拖动 → PC读取主臂位置 → 发送给从臂执行           │
│     采集数据：从臂的关节角度(观测) + 相机图像 + 主臂的动作命令      │
└──────────────────────────────────────────────────────────────────┘
```

- **Robot（从臂 / Follower）**：执行任务的机械臂。它拥有完整的感知和执行闭环：通过电机编码器感知自身关节位置（proprioception），通过相机感知环境（exteroception），接收动作命令后驱动电机到达目标位置。在数据采集时，它是"被记录者"——我们记录它的关节状态和相机画面作为训练数据。
- **Teleoperator（主臂 / Leader）**：被人操作的示教臂。它没有相机，唯一的"传感器"是人手的拖动力。在示教过程中，人被允许直接拖拽主臂的各关节，主臂的编码器实时返回被拖拽到的位置，这个位置经过 PC 中继方案转换为从臂的目标位置。

### 6.1.2 在 LeRobot 中的抽象基类

LeRobot 用两个相互独立但又结构对称的抽象基类来建模这两个角色：

**Robot 基类**（`lerobot/robots/robot.py`，186 行）：

| 方法/属性 | 方向 | 语义 |
|-----------|------|------|
| `get_observation() → dict` | 从硬件**向外**读 | 读取机械臂当前状态（关节角度 + 相机图像），返回一个扁平字典 |
| `send_action(action) → dict` | 从外部**向硬件**写 | 向机械臂发送目标关节位置命令，返回实际执行的动作 |
| `observation_features` | 元数据 | 描述 `get_observation()` 返回字典的 schema（键名和数据类型/形状） |
| `action_features` | 元数据 | 描述 `send_action()` 期望接收字典的 schema |
| `connect()` | 生命周期 | 建立硬件连接、校准、使能力矩 |
| `disconnect()` | 生命周期 | 断开连接、失能力矩、清理相机 |

**Teleoperator 基类**（`lerobot/teleoperators/teleoperator.py`，182 行）：

| 方法/属性 | 方向 | 语义 |
|-----------|------|------|
| `get_action() → dict` | 从硬件**向外**读 | 读取主臂被拖拽到的当前关节位置 |
| `send_feedback(feedback) → None` | 从外部**向硬件**写（可选） | 向主臂发送力反馈（PiPER 主臂当前为空操作） |
| `action_features` | 元数据 | 描述 `get_action()` 返回字典的 schema |
| `feedback_features` | 元数据 | 描述 `send_feedback()` 期望接收字典的 schema |
| `connect()` | 生命周期 | 建立 CAN 连接、使能力矩 |
| `disconnect()` | 生命周期 | 失能力矩、断开连接 |

### 6.1.3 关键差异总结

| 特性 | Robot（PiperFollower） | Teleoperator（PiperLeader） |
|------|------------------------|------------------------------|
| **输入（从外部接收）** | `send_action(action)` — 目标关节位置 | `send_feedback(feedback)` — 力反馈（空实现） |
| **输出（向外部提供）** | `get_observation()` — 关节状态 + 相机图像 | `get_action()` — 被拖拽的关节位置 |
| **相机** | **有**。通常配置 1-2 个 RealSense | **无**。主臂不采集视觉 |
| **校准** | **有**。`MotorCalibration` 定义每个关节的物理范围 | **无**。不需要写入命令，不需要范围映射 |
| **连接后行为** | 连接 → 使能力矩 → parking（回初始位）→ 连接相机 | 连接 → 使能力矩（就这两步） |
| **断开行为** | 断开所有相机 → 断开 CAN → 可选失能力矩 | 失能力矩 → 断开 CAN |
| **电机型号** | joint1-3: AGILEX-M, joint4-6+夹爪: AGILEX-S | 全部 7 个电机: HTDW-5047 |
| **主从模式** | `set_slave()` | `set_master()` |

---

## 6.2 PiperFollower 完整解析

### 6.2.1 类定义和构造

```python
class PiperFollower(Robot):

    # Set these in ALL subclasses
    config_class: PiperFollowerConfig
    name = "piper_follower"

    def __init__(self, config: PiperFollowerConfig):
        self.config = config
        self.id = config.id
        self.port = config.port
        self.bus = PiperMotorsBus(
            id=config.id,
            port=config.port,
            motors={...},       # 电机型号配置
            calibration={...}   # 校准参数
        )
        self.cameras = make_cameras_from_configs(config.cameras)
```

两条类级别属性是强制约定：

- **`config_class`**：类型注解，指向 `PiperFollowerConfig`。LeRobot 框架通过 `draccus.ChoiceRegistry` 的多态机制，在运行时根据 `--robot.type=piper_follower` 自动选择对应的配置类并实例化。如果你忘写这个属性，框架将无法为你的 robot 类型找到正确的配置类。
- **`name = "piper_follower"`**：字符串标识，用于日志输出、注册表查找（`ROBOTS` 常量）、校准文件路径构造（`~/.cache/huggingface/lerobot/calibration/robots/piper_follower/`）。

构造函数的职责非常清晰——它只在内存中创建对象的数据结构，**不建立任何硬件连接**：

1. 保存配置引用 (`self.config`)。
2. 提取 `id` 和 `port` 作为便捷属性。
3. 创建 `PiperMotorsBus` 实例——这是整个机械臂操作的入口，封装了 CAN 通信、电机管理和校准逻辑。此处的 `motors` 参数定义电机型号和归一化模式，`calibration` 参数定义每个关节的物理角度范围。**这两个参数在构造时固化，之后不再改变**。
4. 调用 `make_cameras_from_configs(config.cameras)` 创建相机实例字典。这个工厂函数根据配置字典中的每个条目（指定了 `type`、`serial`、`width`、`height`、`fps`）实例化对应的相机驱动（如 `OpenCVCamera` 或 `RealSenseCamera`）。

### 6.2.2 电机配置（最关键的数据结构）

这是整个 PiperFollower 中最重要、也最容易被误解的代码段。让我们逐项拆解。

#### 电机型号与归一化

```python
self.bus = PiperMotorsBus(
    id=config.id,
    port=config.port,
    motors={
        "joint1": Motor(1, "AGILEX-M", MotorNormMode.RANGE_M100_100),
        "joint2": Motor(2, "AGILEX-M", MotorNormMode.RANGE_M100_100),
        "joint3": Motor(3, "AGILEX-M", MotorNormMode.RANGE_M100_100),
        "joint4": Motor(4, "AGILEX-S", MotorNormMode.RANGE_M100_100),
        "joint5": Motor(5, "AGILEX-S", MotorNormMode.RANGE_M100_100),
        "joint6": Motor(6, "AGILEX-S", MotorNormMode.RANGE_M100_100),
        "gripper": Motor(7, "AGILEX-S", MotorNormMode.RANGE_0_100),
    },
    ...
)
```

每个 `Motor` 对象包含三个参数：

- **`motor_id`**（第 1 参数）：CAN 总线上的电机 ID，范围 1-7。这个 ID 是物理的——它对应电机驱动器上拨码开关设置的 CAN ID。joint1 = ID 1, joint2 = ID 2, ..., gripper = ID 7。注意：这里的 ID 与字典的 key 名是独立的两件事——key 名只是逻辑标识符，ID 才是 CAN 帧中实际使用的地址。
- **`model`**（第 2 参数）：电机型号字符串。控制 SDK 内部如何解析该电机的反馈数据（如编码器分辨率、电流/速度反馈格式）。不同型号对应不同的物理电机。
- **`norm_mode`**（第 3 参数）：归一化模式。定义该电机的归一化目标区间。
  - `RANGE_M100_100`：归一化到 `[-100.0, 100.0]`。用于所有 6 个关节（joint1-6）。对称双极性区间，0 代表物理中位。
  - `RANGE_0_100`：归一化到 `[0.0, 100.0]`。仅用于夹爪（gripper）。单极性区间，0 代表完全闭合，100 代表完全张开。

> **为什么需要归一化？**
>
> 不同电机的物理角度范围差异很大。joint2 的运动范围是从 0 到约 180 度，joint6 是从约 -100 到 +130 度。如果神经网络直接使用物理角度值，它需要隐式地学习每个关节的不同尺度，这会使训练变得困难。归一化将所有关节统一映射到 `[-100, 100]`（或 `[0, 100]`）的公共数值空间，使得策略网络可以平等地对待所有关节，显著降低学习难度。

#### 校准参数

```python
calibration={
    "joint1": MotorCalibration(1, 0, 0, -150000, 150000),
    "joint2": MotorCalibration(2, 0, 0,       0, 180000),
    "joint3": MotorCalibration(3, 0, 0, -170000, 0     ),
    "joint4": MotorCalibration(4, 0, 0, -100000, 100000),
    "joint5": MotorCalibration(5, 0, 0,  -65000, 65000 ),
    "joint6": MotorCalibration(6, 0, 0, -100000, 130000),
    "gripper": MotorCalibration(7, 0, 0, 0, 68000),
}
```

`MotorCalibration` 的构造函数签名为：

```python
MotorCalibration(motor_id, offset_deg, direction, min, max)
```

| 参数 | 类型 | 含义 |
|------|------|------|
| `motor_id` | int | CAN 总线上的电机 ID（必须与 motors 字典中对应电机的 ID 一致） |
| `offset_deg` | float | 角度偏置（度）。PiPER 的出厂校准已完成，此处均为 0 |
| `direction` | int | 旋转方向。0 为默认方向（正向），1 为反向。此处均为 0（默认方向） |
| `min` | int | 该关节的**最小物理限制**，以 SDK 原始编码器单位表示 |
| `max` | int | 该关节的**最大物理限制**，以 SDK 原始编码器单位表示 |

`min` 和 `max` 是校准参数中真正关键的值。它们定义了每个关节的**原始编码器值的合法范围**。归一化过程就是将原始编码器值按此范围线性映射到归一化区间：

```
归一化值 = (原始值 - min) / (max - min) * (norm_max - norm_min) + norm_min
```

例如，对于 joint1，原始值 `0`（物理中位）对应归一化值 `0.0`（`[-100, 100]` 的中点）。

#### 完整参数对照表

| 关节 | Motor ID | 型号 | 校准范围 (min ~ max) | 归一化区间 | 物理含义 | 编码器分辨率量级 |
|------|----------|------|----------------------|-----------|----------|-----------------|
| joint1 | 1 | AGILEX-M | -150000 ~ 150000 | [-100, 100] | 底座旋转（J1），范围约 ±150° | ~1000 单位/度 |
| joint2 | 2 | AGILEX-M | 0 ~ 180000 | [-100, 100] | 大臂俯仰（J2），范围 0° ~ 180° | ~1000 单位/度 |
| joint3 | 3 | AGILEX-M | -170000 ~ 0 | [-100, 100] | 肘关节俯仰（J3），范围约 -170° ~ 0° | ~1000 单位/度 |
| joint4 | 4 | AGILEX-S | -100000 ~ 100000 | [-100, 100] | 腕部旋转（J4），范围约 ±100° | ~1000 单位/度 |
| joint5 | 5 | AGILEX-S | -65000 ~ 65000 | [-100, 100] | 腕部俯仰（J5），范围约 ±65° | ~1000 单位/度 |
| joint6 | 6 | AGILEX-S | -100000 ~ 130000 | [-100, 100] | 腕部末端旋转（J6），范围约 -100° ~ +130° | ~1000 单位/度 |
| gripper | 7 | AGILEX-S | 0 ~ 68000 | [0, 100] | 夹爪开合，0=闭合，100=完全张开 | ~680 单位（全程） |

> **为什么 joint1-3 使用 AGILEX-M，而 joint4-6 使用 AGILEX-S？**
>
> 这是 PiPER 机械臂的硬件设计选择。**AGILEX-M**（M = Medium/Large）是较大型的伺服电机，具有更高的额定扭矩和更大的输出功率，安装在底座、肩部和肘部（joint1-3）——这三个关节需要承受整个臂身的重量和末端负载。**AGILEX-S**（S = Small）是小型伺服电机，更紧凑、更轻量，安装在腕部的三个关节（joint4-6）——这些关节靠近末端执行器，需要低惯量以实现快速精确的腕部动作。夹爪也使用 AGILEX-S，因为夹爪的开合力需求相对较低。

> **为什么 gripper 的归一化区间是 [0, 100] 而不是 [-100, 100]？**
>
> 夹爪只有"开"和"合"两个方向，不存在对称的双向运动。使用 `[0, 100]` 更符合直觉：`0` 代表完全闭合（夹紧物体），`100` 代表完全张开（释放物体）。如果使用 `[-100, 100]`，神经网络可能会学到负值对应半开状态，反而引入不必要的复杂性。

### 6.2.3 observation_features 和 action_features

这两个属性定义了 LeRobot 数据集的 schema——它们告诉数据采集系统"这个 robot 会产生什么样的数据"。

#### _motors_ft —— 关节状态特征

```python
@property
def _motors_ft(self) -> dict[str, type]:
    return {f"{motor}.pos": float for motor in self.bus.motors}
```

遍历 `self.bus.motors` 字典的所有键（即 `"joint1"`, `"joint2"`, ..., `"gripper"`），为每个电机构造一个以 `.pos` 为后缀的键名，值类型为 `float`。生成的结果为：

```python
{
    "joint1.pos": float,
    "joint2.pos": float,
    "joint3.pos": float,
    "joint4.pos": float,
    "joint5.pos": float,
    "joint6.pos": float,
    "gripper.pos": float,
}
```

这里 `.pos` 后缀是 LeRobot 的数据集命名约定——它表示这是**位置（position）**特征。LeRobot 数据集（基于 HuggingFace Datasets）使用 features 字典来定义每帧数据的结构，`.pos` 后缀帮助下游的数据处理 pipeline 识别这是一个关节位置值。

#### _cameras_ft —— 相机图像特征

```python
@property
def _cameras_ft(self) -> dict[str, tuple]:
    return {
        cam: (self.cameras[cam].height, self.cameras[cam].width, 3)
        for cam in self.cameras
    }
```

遍历相机字典，为每个相机生成一个键（通常为 `"wrist"` 和 `"global"`），值是一个三元组 `(height, width, 3)`，表示 RGB 图像的形状。例如，如果配置了腕部和全局两个 640x480 相机：

```python
{
    "wrist":  (480, 640, 3),
    "global": (480, 640, 3),
}
```

注意顺序是 `(H, W, C)` —— height 在前，channels 在后，这是一些深度学习框架（如 PyTorch）的图像数据约定。`3` 固定代表 RGB 三通道。

#### 合并为完整 features

```python
@cached_property
def observation_features(self) -> dict:
    return {**self._motors_ft, **self._cameras_ft}

@cached_property
def action_features(self) -> dict:
    return self._motors_ft
```

两个关键观察：

1. **`observation_features` = 关节特征 + 相机特征**。这意味着 `get_observation()` 返回的字典包含两类信息：proprioceptive（本体感知，即关节角度）和 exteroceptive（外部感知，即相机图像）。
2. **`action_features` = 关节特征（仅有）**。这意味着发送给机械臂的动作命令只包含目标关节位置，不涉及相机。动作是纯 proprioceptive 的——因为相机是传感器（输入），不是执行器（输出）。神经网络策略根据观测中的图像和关节状态推理出目标关节位置，然后将这个目标关节位置发送给机械臂执行。

使用 `@cached_property`（`functools.cached_property`）而非 `@property` 是因为字典合并操作 `{**dict1, **dict2}` 每次调用都会创建新对象。缓存后，第一次访问计算一次，之后直接返回缓存值，避免了不必要的重复计算。

### 6.2.4 connect() 详细流程

```python
def connect(self, calibrate: bool = True) -> bool:
    if not self.bus.connect():
        return False
    logger.info(f"{self} connected.")
    while not self.bus.enable_torque():
        logger.info(f"{self} retry torque on.")
    logger.info(f"{self} go to origin.")
    if calibrate:
        self.bus.parking()

    for cam in self.cameras.values():
        cam.connect()
    return True
```

连接流程按顺序分为 4 个阶段：

**阶段 1：CAN 连接** — `self.bus.connect()`

这是最底层的连接操作。`PiperMotorsBus.connect()` 内部会调用 SDK 的 `C_PiperInterface_V2.ConnectPort()`，后者初始化 SocketCAN 接口、启动后台读取线程（ReadCan 线程，约 200Hz）、发送握手帧确认通信通路正常。

如果连接失败（例如 CAN 口不存在或机械臂未上电），返回 `False`，整个 `connect()` 返回 `False` 通知调用者。

**阶段 2：使能力矩** — `self.bus.enable_torque()`（带重试循环）

使能力矩意味着向每个电机驱动器发送 enable 命令，让电机进入转矩控制模式。在失能状态下，电机处于自由旋转模式（可以被外力轻松转动）。在使能状态下，电机会抵抗外力、保持位置、响应位置命令。

`while not ... : retry` 循环的意义：在实际硬件中，CAN 总线通信有时会因为电气噪声或驱动器状态问题导致第一帧使能命令未成功送达。重试循环确保最终使能成功（如果持续失败则陷入无限循环——这是需要注意的风险点）。

**阶段 3：parking（回初始位）** — `self.bus.parking()`（当 `calibrate=True`）

`parking()` 方法将机械臂的所有关节缓慢移动到预设的"初始位置"（parking position）。这个初始位置是机械臂的一个安全姿态——所有关节都位于其运动范围中段附近，不会碰到极限位，也不会碰到桌面或机械臂自身。

`calibrate` 参数默认为 `True`。如果你希望在连接后保持机械臂当前位置不变（例如在实验间隙快速重连），可以传入 `calibrate=False`。

**阶段 4：逐个连接相机** — `for cam in self.cameras.values(): cam.connect()`

遍历所有配置的相机，调用各自的 `connect()` 方法。对于 RealSense 相机，这会初始化 librealsense2 管道、启动彩色流、等待首帧到达。相机连接较慢（通常需要 1-3 秒），所以写在了 CAN 连接之后。

> **为什么相机连接放在最后？**
>
> 如果先连相机再连 CAN，一旦 CAN 连接失败（例如机械臂未开机、CAN 线松动），相机已经建立了连接但无法使用。将相机连接放在最后，如果 CAN 连接或 parking 失败，调用者可以快速重试而不必重复等待相机初始化。

### 6.2.5 get_observation() 详细流程

```python
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
```

这是一个典型的数据采集循环核心。每帧数据采集经过以下步骤：

**步骤 1：连接检查**

`is_connected` 属性返回 `self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())` ——同时检查 CAN 总线和所有相机都处于连接状态。如果不满足，抛出 `DeviceNotConnectedError`。

**步骤 2：读取关节位置**

`self.bus.get_action()` 调用 SDK 的底层读取接口，从后台 ReadCan 线程的共享缓冲区中获取最新的电机位置值。返回的字典格式为：

```python
{
    "joint1": -15.2,   # 归一化值，范围 [-100, 100]
    "joint2": 45.8,
    "joint3": -120.0,
    "joint4": 30.1,
    "joint5": 0.0,
    "joint6": -45.3,
    "gripper": 50.0,   # 归一化值，范围 [0, 100]
}
```

注意：这些值已经是归一化后的浮点数（经过 calibration 的 min/max 范围映射），直接可用于训练。

然后通过 `{f"{motor}.pos": val for motor, val in obs_dict.items()}` 为每个键添加 `.pos` 后缀，使其符合 LeRobot 数据集的特征命名约定：

```python
{
    "joint1.pos": -15.2,
    "joint2.pos": 45.8,
    ...
    "gripper.pos": 50.0,
}
```

**步骤 3：读取相机图像**

使用 `cam.async_read()` 从每个相机获取最新的 RGB 帧。`async_read()` 是非阻塞调用——它读取相机后台线程的最新缓冲帧，不等待新帧到达。这确保了数据采集循环不会被相机帧率（通常 30 FPS）所阻塞：即使相机还没产生新帧，我们也使用上一帧的值，保证整体采集的实时性。

**步骤 4：合并返回**

将关节位置字典和相机图像字典合并为一个扁平字典返回。一个完整的返回示例如下：

```python
{
    # 关节位置（proprioception）
    "joint1.pos": -15.2,       # float: 归一化关节位置
    "joint2.pos": 45.8,
    "joint3.pos": -120.0,
    "joint4.pos": 30.1,
    "joint5.pos": 0.0,
    "joint6.pos": -45.3,
    "gripper.pos": 50.0,

    # 相机图像（exteroception）
    "wrist":  np.ndarray(shape=(480, 640, 3), dtype=uint8),   # RGB 图像
    "global": np.ndarray(shape=(480, 640, 3), dtype=uint8),   # RGB 图像
}
```

> **为什么方法名叫 `get_observation()` 但内部调用的是 `self.bus.get_action()`？**
>
> 这是一个命名上的潜在混淆点。`PiperMotorsBus.get_action()` 的"action"是从电机驱动的角度而言的——电机把它当前的"行为"（当前角度）反馈出来。但从 Robot/LeRobot 的角度，这属于"观测"（observation），因为它反映了机械臂当前的状态。`get_action()` 这个命名源于 `PiperMotorsBus` 继承自更底层的 motors bus 接口，其中"action"是通用术语。

### 6.2.6 send_action() 详细流程

```python
def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
    if not self.is_connected:
        raise DeviceNotConnectedError(f"{self} is not connected.")

    goal_pos = {key.removesuffix(".pos"): val
                for key, val in action.items()
                if key.endswith(".pos")}

    # Cap goal position when too far away from present position.
    if self.config.max_relative_target is not None:
        present_pos = self.bus.sync_read("Present_Position")
        goal_present_pos = {key: (g_pos, present_pos[key])
                            for key, g_pos in goal_pos.items()}
        goal_pos = ensure_safe_goal_position(
            goal_present_pos, self.config.max_relative_target
        )

    rlt = self.bus.set_action(goal_pos)

    return {f"{motor}.pos": val for motor, val in rlt.items()}
```

**步骤 1：key 转换**

策略网络输出的动作字典使用 `.pos` 后缀作为键名（如 `"joint1.pos": 0.0`），这是 LeRobot 数据集的命名约定。但在发送给 `PiperMotorsBus.set_action()` 之前，需要去掉 `.pos` 后缀，因为 bus 层使用裸的电机名作为键。

```python
# 输入：
#   {"joint1.pos": 10.0, "joint2.pos": 20.0, ...}
# 经过 key.removesuffix(".pos") 转换后：
#   {"joint1": 10.0, "joint2": 20.0, ...}
```

注意 `if key.endswith(".pos")` 的过滤——如果动作字典中包含了非 `.pos` 后缀的键（例如某些外部添加的元数据），这些不会被传入 bus。

**步骤 2：安全限幅（可选）**

当 `config.max_relative_target` 不为 `None` 时，会启用相对位移安全检查：

```python
if self.config.max_relative_target is not None:
    present_pos = self.bus.sync_read("Present_Position")
    goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
    goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)
```

这是防止机械臂发生剧烈运动的最后一道防线。其工作原理如下：

1. **读取当前位置**：`self.bus.sync_read("Present_Position")` 向 CAN 总线发送同步读取命令，获取所有电机的当前实际位置（归一化值）。与 `get_action()` 不同，`sync_read` 是阻塞的——它会等待硬件返回最新数据后才返回，保证数据的一致性。

2. **构造 (目标, 当前) 对**：`goal_present_pos` 字典将目标位置和当前位置配对在一起：
   ```python
   {"joint1": (10.0, 5.0),   # (目标, 当前)
    "joint2": (20.0, 25.0),
    ...}
   ```

3. **限幅检查**：`ensure_safe_goal_position()` 函数对每个关节检查 `|目标 - 当前|` 是否超过 `max_relative_target`。如果超过，将目标值 clamp 到 `当前位置 ± max_relative_target`。

`max_relative_target` 支持两种配置形式：

- **全局标量**：`max_relative_target = 5.0` — 所有关节的单步最大移动量统一为 5.0（在归一化空间 [-100, 100] 中，5.0 约对应几度的物理角度）。
- **per-joint 字典**：`max_relative_target = {"joint1": 3.0, "joint2": 2.0, ...}` — 为每个关节单独设定限制。这对于某些运动范围较小的关节（如 joint5，物理范围只有 ±65°）特别有用。

> **性能注意**：注释中的警告 `/!\ Slower fps expected due to reading from the follower` 指出了这个安全检查的性能代价。每帧额外进行一次 `sync_read` 会增加约 2-5ms 的延迟（取决于 CAN 总线波特率和电机数量），从而降低数据采集或策略推理的帧率。在生产环境中，如果策略本身已经过充分训练且输出平滑，可以考虑将 `max_relative_target` 设为 `None` 以获取最高帧率。

**步骤 3：发送动作**

`self.bus.set_action(goal_pos)` 执行以下操作：

1. **反归一化**：将归一化的目标关节位置（`[-100, 100]` 范围）按 calibration 参数反向映射回原始编码器值。
2. **构造 CAN 帧**：为每个电机构造位置控制命令帧（CAN ID = `0x100 + motor_id`，8 字节数据负载包含目标位置编码）。
3. **发送到 CAN 总线**：通过 SocketCAN 的 `bus.send(msg)` 依次发送每个电机的命令帧。

由于 CAN 总线是广播式的，所有电机几乎同时收到命令帧，实现同步运动。

**步骤 4：返回实际动作**

`set_action()` 返回值是实际发送的归一化位置值（如果某些值被反归一化后超出物理范围会被 clamp）。函数再次添加 `.pos` 后缀后返回，保持与输入格式一致。

### 6.2.7 disconnect() —— 修复的 bug

这是一个值得单独讨论的修改。让我们对比原始版本和修复版本：

**原始版本（有 bug，`lerobot_piper-piper-original`）：**

```python
def disconnect(self, disable_torque: bool = False) -> None:
    self.bus.disconnect(disable_torque)
```

**修复版本（你们的版本）：**

```python
def disconnect(self, disable_torque: bool = False) -> None:
    for camera in self.cameras.values():
        if camera.is_connected:
            camera.disconnect()
    self.bus.disconnect(disable_torque)
```

**Bug 分析：**

原始版本在 `disconnect()` 时只断开了 CAN 总线，**没有断开相机**。这会导致以下问题：

1. **RealSense 相机资源泄漏**：librealsense2 的 pipeline 持有 USB 设备句柄和内核缓冲区。如果不调用 `pipeline.stop()` 而直接让 Python 进程退出，USB 设备可能保持在异常状态。
2. **Core dump 风险**：在某些 Linux 内核版本和 RealSense 固件组合下，未正确关闭的 RealSense 管道在进程退出时可能触发内核 USB 子系统的 core dump。
3. **下次连接失败**：如果相机驱动程序没有正确重置，下次调用 `connect()` 时可能无法正常打开设备（设备被标记为"忙"状态）。

修复的关键点：

- **先关相机，再关 CAN**：断开连接的顺序应当与连接顺序相反（LIFO 原则）。连接时是先 CAN 后相机，断开时先相机关 CAN。
- **检查 `is_connected`**：只断开已经连接的相机，避免对已断开的相机重复调用 `disconnect()`（虽然在大多数实现中是幂等的，但显式检查是更好的防御性编程）。
- **`disable_torque` 参数**：控制是否在断开前失能力矩。默认为 `False` 以保持机械臂的当前位置（避免断开后机械臂因重力下垂）。如果设为 `True`，电机会进入自由旋转模式。

---

## 6.3 PiperLeader 完整解析

### 6.3.1 与 PiperFollower 的关键差异

PiperLeader 继承自 `Teleoperator` 基类而非 `Robot` 基类，这带来了接口层面的根本性变化。同时，由于主臂的物理角色是"被人拖动"而非"自主运动"，其连接流程和电机配置也大不相同。

完整差异对照表：

| 特性 | PiperFollower | PiperLeader |
|------|---------------|-------------|
| **基类** | `Robot` | `Teleoperator` |
| **配置文件** | `PiperFollowerConfig` (RobotConfig 子类) | `PiperLeaderConfig` (TeleoperatorConfig 子类) |
| **注册装饰器** | `@RobotConfig.register_subclass("piper_follower")` | `@TeleoperatorConfig.register_subclass("piper_leader")` |
| **电机型号** | joint1-3: AGILEX-M, joint4-6+gripper: AGILEX-S | 全部 7 个: HTDW-5047 |
| **校准 (calibration)** | **有** — `MotorCalibration` 定义每个关节的范围 | **无** — 构造函数未传入 calibration 参数 |
| **相机** | **有** — `self.cameras = make_cameras_from_configs(...)` | **无** — 没有 cameras 属性 |
| **特征属性** | `observation_features` + `action_features` | `action_features` + `feedback_features` |
| **读方法** | `get_observation()` — 读关节 + 相机 | `get_action()` — 只读关节位置 |
| **写方法** | `send_action()` — 发送目标位置 | `send_feedback()` — 空实现（pass） |
| **setup_motors()** | `set_slave()` — 从模式 | `set_master()` — 主模式 |
| **连接后行为** | CAN 连接 → 使能力矩 → parking → 连接相机 | CAN 连接 → 使能力矩（结束） |
| **重试策略** | `bus.connect()` 失败后立即返回 False | `bus.connect()` 失败后 sleep(0.1) 重试（无限循环） |
| **is_calibrated** | 委托给 `bus.is_calibrated` | 硬编码返回 `True`（主臂不需要校准） |
| **disconnect** | 先断开所有相机 → 再断开 CAN | 先失能力矩 → 再断开 CAN |

### 6.3.2 电机配置

```python
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
    }
)
```

与 PiperFollower 的 motors 配置对比：

| 对比维度 | PiperFollower | PiperLeader |
|----------|---------------|-------------|
| 关节 1-3 型号 | AGILEX-M | HTDW-5047 |
| 关节 4-6 型号 | AGILEX-S | HTDW-5047 |
| 夹爪型号 | AGILEX-S | HTDW-5047 |
| 归一化模式（关节） | RANGE_M100_100 | RANGE_M100_100（相同） |
| 归一化模式（夹爪） | RANGE_0_100 | RANGE_0_100（相同） |

**HTDW-5047** 是教学臂专用的伺服电机型号。它的特点是：

- **低齿槽转矩（low cogging torque）**：电机内部设计的磁路经过优化，使得在失能状态下转子转动平滑、阻力小。这对主臂至关重要——人需要能够轻松拖拽主臂，如果电机阻转力矩太大，拖拽体验会很差。
- **高分辨率编码器**：可以提供与 AGILEX 系列相媲美的位置反馈精度，确保主臂读出的位置数据精度足够高。
- **不需要校准参数**：主臂不接收写入命令，因此不需要 `MotorCalibration` 来限定范围。归一化由 SDK 内部的默认范围（通常是电机的硬件限位）完成。

> **注意**：PiperLeader 的构造函数中**没有传入 `calibration` 参数**。这意味着 `PiperMotorsBus` 的 `calibration` 参数被设为 `None`（或默认空字典）。当没有 calibration 时，`set_action()` 的反归一化步骤无法执行——但这对主臂来说没有影响，因为主臂根本不调用 `set_action()`。

### 6.3.3 为什么主臂不需要校准？

这个问题的答案涉及 LeRobot 架构中数据和命令流的根本不对称性：

**从臂需要校准** 因为它需要写入（`set_action()`）：

```
策略/主臂输出归一化值
    ↓
send_action() 接收归一化值 (如 joint1.pos = 10.0)
    ↓
bus.set_action() 需要将归一化值反归一化为原始编码器值
    ↓
MotorCalibration(min, max) 定义了反归一化的映射公式
    ↓
CAN 帧包含原始编码器值，发送给电机驱动器
```

**主臂不需要校准** 因为它只读取（`get_action()`）：

```
人拖拽主臂
    ↓
编码器产生原始值
    ↓
SDK 的 ReadCan 线程读取原始值
    ↓
bus.get_action() 使用默认范围归一化（或 SDK 内部自带的范围）
    ↓
get_action() 返回归一化值
    ↓
（归一化值直接被中继程序转发给从臂）
```

主臂的归一化过程**不需要知道精确的物理角度范围**——只要归一化使用的映射在整条链路中保持一致即可。主臂读数归一化到 `[-100, 100]`，从臂也在 `[-100, 100]` 的归一化空间中接收命令，映射关系自动匹配。

### 6.3.4 setup_motors() 的特殊性

```python
# PiperFollower
def setup_motors(self) -> None:
    self.bus.connect()
    self.bus.set_slave()

# PiperLeader
def setup_motors(self) -> None:
    self.bus.connect()
    self.bus.set_master()
```

`setup_motors()` 是 LeRobot 框架中的一个生命周期钩子，在 `connect()` 之后被调用（具体由 `lerobot-teleoperate` 等 CLI 工具协调）。它的作用是配置电机的主从角色：

- **`set_slave()`**：将从臂的电机配置为从模式（slave mode）。在从模式下，电机接收来自外部（PC 中继程序或策略推理）的位置命令并通过 CAN 总线执行。对应 CAN 协议中的 `MasterSlaveConfig` 值 `0xFC...`。
- **`set_master()`**：将主臂的电机配置为主模式（master mode）。在主模式下，电机的编码器数据通过 CAN 总线输出，但不响应外部位置命令。对应 CAN 协议中的 `MasterSlaveConfig` 值 `0xFA...`。

> **PC 中继方案不使用主从模式**：在实际的 PC 中继遥操作方案中（`piper_pc_relay_teleop.py`），数据流是"主臂 SDK 读位置 → PC 转发 → 从臂 SDK 写位置"，即 PC 作为中继。此时两个臂之间的 CAN 总线是物理独立的（通常分属两个不同的 CAN 口），它们之间没有直接的 CAN 级主从同步。`set_master()` 和 `set_slave()` 仅在双臂直连的 CAN 主从同步方案中使用（该方案在本项目中不常用）。

### 6.3.5 connect() 对比

```python
# PiperFollower.connect()
def connect(self, calibrate: bool = True) -> bool:
    if not self.bus.connect():
        return False
    # ... torque enable with retry ...
    if calibrate:
        self.bus.parking()
    for cam in self.cameras.values():
        cam.connect()
    return True

# PiperLeader.connect()
def connect(self, calibrate: bool = True) -> None:
    while not self.bus.connect():
        logger.info(f"{self} connection failed.")
        time.sleep(0.1)
    logger.info(f"{self} connected.")
    self.bus.enable_torque()
    logger.info(f"{self} torque on.")
```

关键差异：

1. **重试策略**：PiperFollower 的连接失败后直接返回 `False`，由调用者决定如何处理。PiperLeader 则采用无限重试循环——连接失败后 sleep 0.1 秒再重试。这种差异反映了两种场景的不同需求：从臂的连接通常由脚本控制，失败后可以由用户手动排查后重跑脚本；主臂的连接通常在人开始示教前进行，无限重试意味着"好了，你现在可以去给主臂上电了，程序会等着你"。

2. **无 parking**：主臂不需要 parking。主臂的初始位置就是它当前被搁置的位置——通常比 parking 位置更适合人开始拖拽（parking 位置是一个紧致的直立姿态，不太利于人操作）。

3. **无相机连接**：主臂没有相机，connect() 流程更简短。

4. **使能力矩无重试**：`self.bus.enable_torque()` 只调用一次（无 while 循环）。这是因为主臂的 HTDW-5047 电机使能响应更快，几乎不需要重试。

### 6.3.6 get_action() 流程

```python
def get_action(self) -> dict[str, Any]:
    if not self.is_connected:
        raise DeviceNotConnectedError(f"{self} is not connected.")
    return self.bus.get_action()
```

这是整个 PiperLeader 中最简洁的方法——只有 2 行有效代码。`self.bus.get_action()` 返回的字典键名没有 `.pos` 后缀（直接是 `"joint1"`, `"joint2"` 等），这与 PiperFollower 的 `get_observation()` 不同。这是因为：

- PiperFollower 的观测数据需要写入 LeRobot 数据集，必须遵循数据集的 `.pos` 命名约定。
- PiperLeader 的动作数据由 PC 中继程序直接转发，不经过数据集序列化，因此可以使用更简短的键名。

### 6.3.7 send_feedback() —— 空实现

```python
def send_feedback(self, feedback: dict[str, Any]) -> None:
    pass
```

这个方法是一个**存根（stub）**——只有 `pass` 语句，什么都不做。在 LeRobot 的 Teleoperator 抽象中，`send_feedback()` 的目的是向主臂发送力反馈信号（例如在 VR 手柄或 haptic 设备中提供触觉反馈）。PiPER 的主臂是纯机械的被动示教臂，不具备力反馈执行器，因此该方法为空。

这个空实现的存在是为了满足 `Teleoperator` 抽象基类的接口约定——如果不在子类中实现 `send_feedback()`，Python 会在实例化时报 `TypeError: Can't instantiate abstract class`。

---

## 6.4 Config 类解析

### 6.4.1 PiperFollowerConfig

```python
@RobotConfig.register_subclass("piper_follower")
@dataclass(kw_only=True)
class PiperFollowerConfig(RobotConfig):
    port: str
    disable_torque_on_disconnect: bool = True
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    max_relative_target: float | dict[str, float] | None = None
```

各字段说明：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `port` | `str` | **（必填）** | CAN 口名称，如 `"can0"`、`"can1"`。这是唯一没有默认值的字段，必须在构造时显式提供 |
| `disable_torque_on_disconnect` | `bool` | `True` | 断开连接时是否失能力矩。设为 `True` 时，断开后电机自由旋转，人可以手动移动机械臂；设为 `False` 时，断开后电机保持当前位置 |
| `cameras` | `dict[str, CameraConfig]` | `{}`（空字典） | 相机配置字典。key 为相机名（如 `"wrist"`, `"global"`），value 为 `CameraConfig` 对象。空字典意味着不使用任何相机 |
| `max_relative_target` | `float \| dict[str, float] \| None` | `None` | 单步最大相对位移限幅。`None` 表示不限制；`float` 表示所有关节共用同一值；字典表示 per-joint 配置 |

从 `RobotConfig` 继承的字段（在 `RobotConfig.__init__` 中处理）：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `id` | `str \| None` | `None` | 机器人实例标识。同一台机器上连接多个同型号机械臂时用于区分 |
| `calibration_dir` | `Path \| None` | `None` | 校准文件存储目录。如果为 `None`，自动使用 `~/.cache/huggingface/lerobot/calibration/robots/piper_follower/` |

**`@dataclass(kw_only=True)`** 强制所有字段必须以关键字参数形式传递，避免位置参数导致的混淆（例如 `PiperFollowerConfig("can0", True)` 这种容易误读的写法被禁止，必须写作 `PiperFollowerConfig(port="can0", disable_torque_on_disconnect=True)`）。

**`RobotConfig.__post_init__` 的相机验证**（从 `RobotConfig` 继承）：

```python
def __post_init__(self):
    if hasattr(self, "cameras") and self.cameras:
        for _, config in self.cameras.items():
            for attr in ["width", "height", "fps"]:
                if getattr(config, attr) is None:
                    raise ValueError(
                        f"Specifying '{attr}' is required for the camera to be used in a robot"
                    )
```

如果在 cameras 字典中配置了相机但没有指定 `width`、`height` 或 `fps`，会在实例化时直接抛出 `ValueError`。这确保了配置的完整性。

### 6.4.2 PiperLeaderConfig

```python
@TeleoperatorConfig.register_subclass("piper_leader")
@dataclass
class PiperLeaderConfig(TeleoperatorConfig):
    port: str
    gripper_open_pos: float = 50.0
```

各字段说明：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `port` | `str` | **（必填）** | CAN 口名称 |
| `gripper_open_pos` | `float` | `50.0` | 夹爪张开位置（归一化值，范围 [0, 100]）。当主臂进入转矩模式时，夹爪电机会收到该位置值。如果人捏合夹爪后释放，夹爪会自动弹回此位置 |

`gripper_open_pos = 50.0` 的含义：在归一化范围 `[0, 100]` 中，50.0 对应夹爪半开状态。这提供了一种"弹性夹爪"的体验——人可以用力捏合夹爪，松开后夹爪自动回到半开位置（而不是全开或全闭）。这个默认值可以按任务需要调整。

与 PiperFollowerConfig 不同，PiperLeaderConfig **没有相机配置**（因为主臂无相机），**没有安全限幅配置**（因为主臂只读不写），**没有 `disable_torque_on_disconnect`**（主臂的 `disconnect()` 总是会失能力矩）。

**注意**：PiperLeaderConfig 使用 `@dataclass` 而非 `@dataclass(kw_only=True)`。这意味着它接受位置参数——`PiperLeaderConfig("can0")` 是合法写法（但不推荐）。

---

## 6.5 注册机制深入

### 6.5.1 `register_subclass` 的工作原理

两个配置类顶部都有类似的装饰器：

```python
@RobotConfig.register_subclass("piper_follower")        # PiperFollowerConfig
@TeleoperatorConfig.register_subclass("piper_leader")    # PiperLeaderConfig
```

这个机制是 LeRobot 实现**多态配置**的核心。它的工作流程如下：

**第 1 步：装饰器注册**

`RobotConfig` 继承自 `draccus.ChoiceRegistry`（见 `RobotConfig` 的定义）：

```python
@dataclass(kw_only=True)
class RobotConfig(draccus.ChoiceRegistry, abc.ABC):
    ...
```

`draccus.ChoiceRegistry` 维护一个全局的**注册表**（registry），将字符串名称映射到配置子类：

```
注册表 (内部):
{
    "piper_follower": PiperFollowerConfig,
    "so100":          So100Config,
    "koch":           KochConfig,
    ...
}
```

当 Python 解释器执行到 `@RobotConfig.register_subclass("piper_follower")` 时（类定义时刻），装饰器将 `"piper_follower"` → `PiperFollowerConfig` 的映射写入注册表。这个过程**在 `import` 时**就已完成，不需要等到实例化。

**第 2 步：CLI 参数解析**

用户在命令行使用 `lerobot-record --robot.type=piper_follower --robot.port=can1` 时：

1. `draccus`（基于 `dataclasses` + `argparse`/`jsonargparse` 的配置解析库）解析 `--robot.type=piper_follower`。
2. draccus 查询 `RobotConfig` 的注册表，找到 `"piper_follower"` 对应的子类 `PiperFollowerConfig`。
3. draccus 使用 `PiperFollowerConfig` 的字段定义来解析剩余的 `--robot.*` 参数（如 `--robot.port=can1`）。
4. 如果用户传入 `--robot.cameras.wrist.type=realsense --robot.cameras.wrist.serial=...`，draccus 递归地使用 `CameraConfig` 的注册表（也是 `ChoiceRegistry` 的子类）来解析相机子配置。

**第 3 步：类型属性验证**

配置文件实例化后，`PiperFollower.__init__` 接收这个配置对象。由于类定义中有 `config_class: PiperFollowerConfig` 的类型注解，IDE 和类型检查器可以验证类型正确性。不过，在运行时 Python 并不会强制执行这个类型约束——如果错误地将 `PiperLeaderConfig` 传入 `PiperFollower()`，只有在访问不存在的属性时才会报 `AttributeError`。

### 6.5.2 `type` 属性

两个配置类都定义了 `type` 属性：

```python
@property
def type(self) -> str:
    return self.get_choice_name(self.__class__)
```

`get_choice_name()` 是 `draccus.ChoiceRegistry` 提供的方法，它反向查找注册表——给定一个类，返回它被注册时的名称字符串。对于 `PiperFollowerConfig`，返回 `"piper_follower"`。

这个属性使得配置对象具有**自描述性**：给定任意一个配置对象，你可以通过 `config.type` 知道它属于哪种 robot/teleoperator。

---

## 6.6 完整使用示例

### 6.6.1 使用 PiperFollower 编程（不使用 CLI）

```python
from lerobot.robots.piper_follower import PiperFollower, PiperFollowerConfig
from lerobot.cameras import OpenCVCameraConfig

# 第 1 步：构造配置
config = PiperFollowerConfig(
    port="can1",
    cameras={
        "wrist": OpenCVCameraConfig(
            camera_index=0,
            width=640,
            height=480,
            fps=30,
        ),
    },
    max_relative_target=5.0,  # 安全限幅：单步不超过 5.0
)
print(f"Robot type: {config.type}")  # 输出: piper_follower

# 第 2 步：创建 robot 对象（此时无硬件连接）
robot = PiperFollower(config)
print(f"Features: {robot.action_features}")
# 输出: {'joint1.pos': float, 'joint2.pos': float, ...,
#         'gripper.pos': float, 'wrist': (480, 640, 3)}

# 第 3 步：连接硬件
robot.connect(calibrate=True)
print(f"Connected: {robot.is_connected}")  # 输出: True

# 第 4 步：读取观测
obs = robot.get_observation()
print(obs.keys())
# 输出: dict_keys(['joint1.pos', 'joint2.pos', 'joint3.pos',
#                   'joint4.pos', 'joint5.pos', 'joint6.pos',
#                   'gripper.pos', 'wrist'])
print(f"joint1.position = {obs['joint1.pos']}")
# 输出: joint1.position = -15.2 (示例值)

# 第 5 步：发送动作
action = {
    "joint1.pos": 10.0,
    "joint2.pos": 30.0,
    "joint3.pos": -50.0,
    "joint4.pos": 0.0,
    "joint5.pos": 20.0,
    "joint6.pos": -30.0,
    "gripper.pos": 80.0,
}
result = robot.send_action(action)
# 机械臂运动到目标位置，result 是实际执行的动作

# 第 6 步：断开连接
robot.disconnect(disable_torque=True)
print(f"Connected: {robot.is_connected}")  # 输出: False
```

### 6.6.2 使用 PiperLeader 编程（不使用 CLI）

```python
from lerobot.teleoperators.piper_leader import PiperLeader, PiperLeaderConfig

# 第 1 步：构造配置
config = PiperLeaderConfig(
    port="can0",
    gripper_open_pos=60.0,  # 夹爪默认处于较开位置
)

# 第 2 步：创建 leader 对象
leader = PiperLeader(config)
print(f"Teleoperator type: {config.type}")  # 输出: piper_leader

# 第 3 步：连接硬件
leader.connect()
print(f"Connected: {leader.is_connected}")  # 输出: True

# 第 4 步：读取主臂位置（循环读取，模拟遥操作）
import time
for i in range(100):
    action = leader.get_action()
    print(f"Frame {i}: {action}")
    # 输出: {'joint1': -10.5, 'joint2': 45.2, ..., 'gripper': 55.0}
    time.sleep(0.033)  # ~30 FPS

# 第 5 步：断开
leader.disconnect()
```

### 6.6.3 主从联动示例（PC 中继方案核心逻辑）

以下代码展示了 PC 中继遥操作中最核心的数据流——这实际上就是 `piper_pc_relay_teleop.py` 的简化版本：

```python
from lerobot.robots.piper_follower import PiperFollower, PiperFollowerConfig
from lerobot.teleoperators.piper_leader import PiperLeader, PiperLeaderConfig

# 分别配置主臂和从臂（它们连接不同的 CAN 口）
leader_config = PiperLeaderConfig(port="can0")
follower_config = PiperFollowerConfig(port="can1")

leader = PiperLeader(leader_config)
follower = PiperFollower(follower_config)

# 连接
leader.connect()
follower.connect()

try:
    while True:
        # 第 1 步：读取主臂被拖动到的位置
        action = leader.get_action()
        # action = {"joint1": 10.5, "joint2": 45.2, ..., "gripper": 55.0}

        # 第 2 步：添加 .pos 后缀，转发给从臂
        action_with_suffix = {f"{k}.pos": v for k, v in action.items()}
        # action_with_suffix = {"joint1.pos": 10.5, ...}

        # 第 3 步：从臂执行动作
        follower.send_action(action_with_suffix)

finally:
    # 清理
    leader.disconnect()
    follower.disconnect()
```

这个循环的核心逻辑只有 3 行代码，体现了 LeRobot 封装带来的简洁性：

1. `leader.get_action()` — 读主臂位置
2. 添加 `.pos` 后缀 — 格式转换
3. `follower.send_action()` — 发送给从臂执行

---

## 6.7 小结

本章我们逐行剖析了 PiperFollower 和 PiperLeader 两个核心类的完整实现。以下是关键收获：

1. **Robot 和 Teleoperator 是两个独立的抽象层次**。Robot 管理感知（观测）和执行（动作）的完整闭环，Teleoperator 只负责输出人类示教的位置数据。

2. **电机配置是整个系统的数据根基**。`Motor` 的型号和归一化模式 + `MotorCalibration` 的物理范围，共同决定了归一化和反归一化的映射关系。AGILEX-M 用于大扭矩关节（1-3），AGILEX-S 用于小扭矩关节（4-6+夹爪），HTDW-5047 用于教学臂——这背后是硬件设计的工程权衡。

3. **校准只对写操作有意义**。PiperFollower 需要校准来确保发送的目标值在物理限制内，PiperLeader 只读不写，因此不需要校准。

4. **`max_relative_target` 是最后一道安全防线**。它防止单步移动过大导致的机械臂抖动或碰撞，但会带来额外的 CAN 读取延迟。

5. **disconnect 的 bug 修复体现了防御性编程的重要性**。相机和 CAN 总线都是需要显式清理的资源，断开顺序应当与连接顺序相反。

6. **注册机制使多态配置成为可能**。`@RobotConfig.register_subclass("piper_follower")` 通过 draccus 的 ChoiceRegistry 实现了"字符串 → 类"的自动路由，使得 CLI 工具可以用 `--robot.type=piper_follower` 这样的简洁语法选择不同的硬件驱动。

在下一章，我们将介绍数据采集的完整流程——包括 PC 中继遥操作脚本、LeRobot 的 record 命令、以及数据是如何被写入 HuggingFace Datasets 格式的。
