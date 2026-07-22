# 第 4 章 LeRobot 框架深度解析

## 4.1 LeRobot 是什么

LeRobot 是 HuggingFace 于 2024 年开源的机器人学习框架，目标是为真实机器人数据采集、策略训练和部署评估提供一套统一的标准化流水线。其核心理念可以概括为三个关键词：

**统一的数据格式** —— 所有机器人，无论机械结构如何不同，它们采集出来的数据都遵循同一套命名规范和存储结构。这使得在不同的机器人硬件之间迁移算法和数据集成为可能。

**插件式硬件接入** —— 新增一种机器人型号，只需编写一个配置类和实现若干抽象方法，无需修改框架核心代码。框架通过 `draccus.ChoiceRegistry` 自动发现并加载新硬件。

**模块化策略训练** —— 训练算法（ACT、Diffusion Policy、Pi0、VQ-BeT、TD-MPC 等）与机器人硬件完全解耦。策略只看到归一化后的动作值，与底层舵机或 CAN 总线无关。

LeRobot 提供了三大 CLI 工具，覆盖从数据采集到训练再到部署的完整生命周期：

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│ lerobot-record   │ ──► │  lerobot-train    │ ──► │  lerobot-eval    │
│ 采集示教数据      │     │ 训练模仿学习策略    │     │ 部署并评估策略    │
└─────────────────┘     └──────────────────┘     └─────────────────┘
      ▲                                                   │
      │       ┌──────────────────────────────┐            │
      └────── │ --robot.type=piper_follower   │ ◄──────────┘
              │ --robot.port=can0              │
              │ --teleop.type=piper_leader     │
              │ --teleop.port=can1             │
              └──────────────────────────────┘
```

所有这些 CLI 工具都是通过 `pyproject.toml` 中的 `[project.scripts]` 注册为系统命令的。当你执行 `pip install -e ".[piper]"` 时，pip 会自动在你的 `$PATH` 中创建可执行脚本。下面是从 `pyproject.toml` 中摘录的实际配置：

```toml
[project.scripts]
lerobot-calibrate="lerobot.calibrate:main"
lerobot-find-cameras="lerobot.find_cameras:main"
lerobot-find-port="lerobot.find_port:main"
lerobot-record="lerobot.record:main"
lerobot-replay="lerobot.replay:main"
lerobot-setup-motors="lerobot.setup_motors:main"
lerobot-teleoperate="lerobot.teleoperate:main"
lerobot-eval="lerobot.scripts.eval:main"
lerobot-train="lerobot.scripts.train:main"
```

每一行等号左边是用户敲的命令，等号右边的 `模块路径:函数名` 告诉 pip 把哪个 Python 函数包装成可执行入口。

`pip install -e ".[piper]"` 实际上做了三件事：

1. **安装 piper_sdk 依赖**——从 `[project.optional-dependencies]` 中找到 `piper = ["piper_sdk>=0.4.1"]`，确保 PiPER 硬件的 CAN 通信库可用。
2. **注册 CLI 入口点**——将上表中的每个 `模块:函数` 映射创建为系统级可执行文件。
3. **触发模块导入**——`pip install -e` 以 editable 模式安装后，Python 在 `import lerobot` 时会遍历 `src/lerobot/` 下所有子包。由于每个子类配置文件中使用了 `@RobotConfig.register_subclass("piper_follower")` 装饰器，这些注册都在模块导入时执行完毕，ChoiceRegistry 的 `_subclasses` 字典在程序启动时就已经填好了。

---

## 4.2 LeRobot 的设计哲学：从硬件多样性到算法统一性

在理解了 LeRobot 的基本功能之后，我们有必要深入探讨一个问题：**为什么 LeRobot 要这样设计？** 这不仅仅是一个工程选择，而是反映了机器人学习领域对"如何让 AI 走进物理世界"这个问题的深层思考。

### 4.2.1 机器人学习的数据问题

深度学习的成功——从 ImageNet 到 GPT——很大程度上依赖于**大规模、高质量、标准化的数据集**。但机器人领域长期面临一个"鸡和蛋"的问题：

1. **没有统一的数据格式**：每个实验室、每个机器人平台都有自己的数据格式。MIT 的机器人可能用 ROS bag，Stanford 的可能用 HDF5 + 自定义 schema，工业机器人用专有格式。数据无法共享、无法复用。
2. **数据采集成本极高**：CV/NLP 的数据可以从互联网抓取（免费且海量），机器人数据需要真人操作物理硬件（昂贵且有限）。
3. **硬件碎片化**：机器人有 6 轴的、7 轴的、并联的、移动的，每种都有不同的自由度和控制接口。

LeRobot 的设计正是为了解决这三个问题：
- **统一格式** → 解决数据孤岛（类似于 ImageNet 统一了图像分类数据）
- **插件式架构** → 解决硬件碎片化（类似于 PyTorch 的 Module 统一了所有网络层）
- **标准化工具链** → 降低采集门槛（类似于 HuggingFace 的 Transformers 让非专家也能训练模型）

### 4.2.2 为什么归一化是设计的核心

在 [第 1 章](01_system_overview.md) 中我们介绍了 VLA 和 World Model 的概念。LeRobot 的归一化设计（所有动作统一为 [-100, 100] 或 [0, 100]）不仅是为了工程便利——它是实现 **跨机器人迁移学习** 的必要前提。

考虑两种可能的发展路径：

**路径 A（当前）**：PiPER 上的 ACT
```
PiPER 数据 → 训练 ACT → 部署到 PiPER
```

**路径 B（未来的 VLA）**：多种机器人数据联合训练
```
PiPER 数据 + So100 数据 + Koch 数据 + Reachy 数据 → 训练 VLA → 部署到任意机器人
```

路径 B 只有在所有机器人的动作空间被归一化到同一尺度时才有可能。一个 2 米长的工业机械臂和 30 厘米的桌面机械臂，它们的关节运动范围差了近一个数量级。但归一化到 [-100, 100] 后，对策略模型来说它们都是"相同的动作空间"，模型只需要关注"在这个视觉场景中应该如何移动"，而不是"这个特定机械臂的物理极限是多少毫米"。

这引出了一个更深层的概念——**embodiment-agnostic policy（体态无关策略）**：一个能理解"抓取"这个动作语义的策略，应该能在不同大小、不同自由度的机械臂上执行，就像人类既可以用左手也可以用右手拿杯子一样。LeRobot 的归一化框架正是通向这一愿景的一小步。

### 4.2.3 插件式设计如何支持 VLA 和 World Model 的演进

LeRobot 的三层架构（详见 4.3 节）看似只是经典的软件工程实践，但它实际上有更深远的考虑：

**MotorsBus 层的独立性** 意味着：
- 可以单独升级为**带有 World Model 的 MotorsBus**：在执行动作前，先用 World Model 预测该动作的后果，如果预测的结果不安全，降低速率或拒绝执行。
- 可以替换为**仿真 MotorsBus**：在仿真器中运行相同的策略，用于大规模的离线训练和评估（这就是 4.2.1 节提到的 3 条路径中的纯 RL + sim-to-real）。

**Robot/Teleoperator 层的独立性** 意味着：
- 可以添加**语言接口**：`connect()` 变为 `connect("pick up the red cube")`，将自然语言指令转换为任务上下文，策略据此调整行为（VLA 的前置需求）。
- 可以引入**多模态观测**：当前的 observation 包含 RGB 图像和关节角度。未来可以添加深度图、触觉力反馈、甚至麦克风音频——Robot 层的 `get_observation()` 接口天然支持这种扩展。

**Policy 层的独立性** 意味着：
- ACT 可以替换为 Pi0（一个 VLA 模型）或带 World Model 的 TD-MPC，而硬件层完全不用修改。
- 策略可以不直接输出最终动作，而是输出一个**planner 的中间表示**（如 sub-goal 坐标），再由一个下层的 motion planner 将 sub-goal 翻译为关节角度指令（分层策略，Hierarchical Policy）。

### 4.2.4 与更广泛的 AI 研究的关系

LeRobot 的设计受到了以下研究的深刻影响：

| 影响来源 | 对 LeRobot 的具体影响 |
|----------|----------------------|
| **ImageNet (2012)** | 证明了统一的大规模数据集可以催生算法革命。LeRobot 的统一数据格式意在创建"机器人的 ImageNet"。 |
| **HuggingFace Transformers (2019)** | 证明了统一的模型接口（AutoModel、AutoTokenizer）可以极大降低使用门槛。LeRobot 的 ChoiceRegistry 和 `make_robot_from_config()` 是同样的思想。 |
| **GPT 系列 (2018-2023)** | 证明了 scaling law——模型越大、数据越多，泛化能力越强。这推动了 VLA 的研究和 LeRobot 的标准化采集工具。 |
| **RT-2 / Octo / Pi0 (2023-2024)** | 证明了 VLA 模型可以在真实机器人上 zero-shot 执行指令。LeRobot 的设计就是为了让这类模型能方便地接入不同硬件。 |
| **JEPA / World Models (2022-2024)** | 证明了预测未来观测是学习世界知识的有效方式。LeRobot 保留 observation 和 action 的完整历史，为未来集成 World Model 提供了数据基础。 |

---

## 4.3 插件式设计：三层架构

LeRobot 采用了严格的**三层架构**，每层之间通过抽象基类定义契约，通过 ChoiceRegistry 实现自动发现。这种设计使得添加新硬件时，只需在对应的层实现具体子类，上层和数据流水线完全不受影响。

```
┌──────────────────────────────────────────────────────────────────────┐
│  CLI 层                                                              │
│  lerobot-record / lerobot-train / lerobot-eval / lerobot-teleoperate │
│                                                                      │
│  Example:                                                            │
│    $ lerobot-record --robot.type=piper_follower --robot.port=can0    │
│    $ lerobot-record --robot.type=koch_follower --robot.port=/dev/tty │
│    $ lerobot-record --robot.type=so100_follower --robot.port=...     │
│    $ lerobot-record --robot.type=bi_so100_follower                    │
│                                                                      │
│      draccus.ChoiceRegistry 根据 --robot.type 的值自动查找对应子类    │
├──────────────────────────────────────────────────────────────────────┤
│  Robot / Teleoperator 层                                             │
│                                                                      │
│  PiperFollower / PiperLeader / KochFollower / KochLeader /           │
│  SO100Follower / SO100Leader / LeKiwi / Reachy2Robot / Stretch3 /    │
│  BiSO100Follower / ViperX / HopeJr / HomunculusGlove / Phone / ...   │
│                                                                      │
│  每个 Robot 持有 1 个 MotorsBus 实例 (bimanual 机器人持有 2 个)        │
│  每个 Teleoperator 持有 1 个 MotorsBus 实例                           │
├──────────────────────────────────────────────────────────────────────┤
│  MotorsBus 层                                                        │
│                                                                      │
│  PiperMotorsBus / DynamixelMotorsBus / FeetechMotorsBus / ...        │
│                                                                      │
│  每个 MotorsBus 持有:                                                 │
│    - port_handler: 串口/CAN 端口的打开/关闭/参数设置                   │
│    - 若干 Motor 对象: 电机的 id、型号、归一化模式                      │
│    - 若干 MotorCalibration 对象: 运动范围 min/max                     │
├──────────────────────────────────────────────────────────────────────┤
│  硬件层                                                              │
│                                                                      │
│  piper_sdk (CAN 总线) / serial (USB 串口) / CAN bus / ...            │
│                                                                      │
│  真正的物理通信: CAN 帧收发、RS-485 协议、舵机控制表读写               │
└──────────────────────────────────────────────────────────────────────┘
```

### 为什么 LeRobot 能自动发现 PiperFollower？

关键语句在 `src/lerobot/robots/piper_follower/config_piper_follower.py` 的第 26 行：

```python
@RobotConfig.register_subclass("piper_follower")
@dataclass(kw_only=True)
class PiperFollowerConfig(RobotConfig):
    port: str
    ...
```

当 Python 执行 `import lerobot.robots.piper_follower` 时（这发生在 `lerobot-record` 启动阶段的 `make_robot_from_config` 或通过 draccus 解析 `--robot.type=piper_follower` 参数时），装饰器 `register_subclass("piper_follower")` 将字符串 `"piper_follower"` 与类 `PiperFollowerConfig` 的映射写入 ChoiceRegistry 的 `_subclasses` 字典中。

此后，当用户在命令行输入 `--robot.type=piper_follower`，draccus 的序列化引擎会：

1. 查找 `RobotConfig._subclasses["piper_follower"]`
2. 发现是 `PiperFollowerConfig`
3. 用剩余的 CLI 参数（如 `--robot.port=can0`）实例化 `PiperFollowerConfig(port="can0")`
4. 将这个 config 对象传递给 `PiperFollower(config)`

整个过程对用户完全透明——用户只需要知道机器人类型名和端口名。

---

## 4.4 draccus ChoiceRegistry 自动类型发现

### draccus 是什么

draccus 是 HuggingFace 团队开发的配置管理库（当前 LeRobot 固定依赖 `draccus==0.10.0`），它在标准 Python `dataclass` 的基础上增加了：

- **命令行参数自动解析**——dataclass 的字段自动映射为 CLI 参数（`--field=value` 或 `--field.subfield=value`）
- **ChoiceRegistry 子类注册机制**——基类维护一个全局注册表，子类通过装饰器注册，运行时按名称查找
- **嵌套配置支持**——一个 dataclass 可以包含另一个 dataclass，CLI 参数用 `.` 分隔嵌套层级
- **插件动态加载**——通过 `--xxx.discover_packages_path=my_package` 可以在运行时动态加载外部包中的子类

### ChoiceRegistry 机制剖析

`ChoiceRegistry` 是 draccus 提供的一个混入类（Mixin），它的核心工作方式是维护一个类级别的 `_subclasses` 字典：

```python
# draccus 内部逻辑的简化示意
class ChoiceRegistry:
    _subclasses: dict[str, type] = {}

    @classmethod
    def register_subclass(cls, name: str):
        """将子类注册到注册表中"""
        def decorator(subclass):
            cls._subclasses[name] = subclass
            return subclass
        return decorator

    @classmethod
    def get_choice_name(cls, subclass) -> str:
        """反向查找：给定子类，返回注册名"""
        for name, sub in cls._subclasses.items():
            if sub is subclass:
                return name
        raise KeyError(subclass)
```

在 LeRobot 中，`RobotConfig` 继承自 `draccus.ChoiceRegistry`：

```python
# src/lerobot/robots/config.py 第 23 行
@dataclass(kw_only=True)
class RobotConfig(draccus.ChoiceRegistry, abc.ABC):
    id: str | None = None
    calibration_dir: Path | None = None
```

`TeleoperatorConfig` 也采用完全相同的方式：

```python
# src/lerobot/teleoperators/config.py 第 23 行
@dataclass(kw_only=True)
class TeleoperatorConfig(draccus.ChoiceRegistry, abc.ABC):
    id: str | None = None
    calibration_dir: Path | None = None
```

### 完整调用链示例

假设用户在终端执行：

```bash
lerobot-record --robot.type=piper_follower --robot.port=can1 --robot.id=black --dataset.repo_id=my_user/my_data
```

整个类型发现和实例化过程如下：

```
1. shell 调用 lerobot-record 可执行文件
      │
2.    └─► Python 执行 lerobot.record:main()
      │   └─► @parser.wrap() 装饰器分析 main() 的第一个参数类型 → RecordConfig
      │       RecordConfig 的第一个字段是 robot: RobotConfig
      │
3.    parser 扫描 sys.argv，发现 --robot.type=piper_follower
      │   └─► 查找 RobotConfig._subclasses["piper_follower"]
      │       找到 PiperFollowerConfig
      │
4.    parser 发现 --robot.port=can1
      │   └─► 匹配 PiperFollowerConfig 的 port: str 字段
      │       值 = "can1"
      │
5.    parser 发现 --robot.id=black
      │   └─► 匹配 RobotConfig 基类的 id 字段
      │       值 = "black"
      │
6.    parser 发现 --dataset.repo_id=my_user/my_data
      │   └─► 匹配 RecordConfig.dataset 的 repo_id 字段
      │       值 = "my_user/my_data"
      │
7.    parser 用所有解析到的参数调用:
      │   RecordConfig(
      │       robot=PiperFollowerConfig(port="can1", id="black"),
      │       dataset=DatasetRecordConfig(repo_id="my_user/my_data"),
      │       teleop=None,
      │       policy=None,
      │   )
      │
8.    main() 收到 cfg: RecordConfig 对象
      │   └─► make_robot_from_config(cfg.robot)
      │       返回 PiperFollower(config=cfg.robot)
      │
9.    PiperFollower.__init__() 创建 PiperMotorsBus(...)
      │   └─► PiperMotorsBus.__init__() 调用 piper_sdk.C_PiperInterface_V2(can1)
      │
10.   进入主循环: record_loop(robot=PiperFollower, ...)
```

### 已注册的所有机器人类型

LeRobot 生态中目前已注册的机器人类型（通过在代码库中搜索 `@RobotConfig.register_subclass` 所得）：

| 注册名 | 配置类 | 底层总线 | 说明 |
|--------|--------|----------|------|
| `piper_follower` | PiperFollowerConfig | PiperMotorsBus (piper_sdk) | PiPER 双臂中的执行臂，CAN 通信 |
| `koch_follower` | KochFollowerConfig | DynamixelMotorsBus | Koch v1.1 低成本臂 |
| `so100_follower` | SO100FollowerConfig | FeetechMotorsBus | SO-100 低成本臂 |
| `so101_follower` | SO101FollowerConfig | FeetechMotorsBus | SO-101 改进版 |
| `bi_so100_follower` | BiSO100FollowerConfig | FeetechMotorsBus x2 | SO-100 双臂版 |
| `lekiwi` | LeKiwiConfig | FeetechMotorsBus | LeKiwi 教育机器人 |
| `lekiwi_client` | LeKiwiClientConfig | (远程) | LeKiwi 远程客户端 |
| `reachy2` | Reachy2Config | (reachy2_sdk) | Pollen Robotics 的 Reachy2 |
| `stretch3` | Stretch3Config | DynamixelMotorsBus | Hello Robot 的 Stretch 3 |
| `viperx` | ViperXConfig | DynamixelMotorsBus | Trossen Robotics ViperX |
| `hope_jr_arm` | HopeJrArmConfig | FeetechMotorsBus | Hope Jr 手臂 |
| `hope_jr_hand` | HopeJrHandConfig | FeetechMotorsBus | Hope Jr 灵巧手 |

对应的 Teleoperator 注册：

| 注册名 | 配置类 | 底层总线 | 说明 |
|--------|--------|----------|------|
| `piper_leader` | PiperLeaderConfig | PiperMotorsBus (piper_sdk) | PiPER 主臂（示教臂） |
| `koch_leader` | KochLeaderConfig | DynamixelMotorsBus | Koch 主臂 |
| `so100_leader` | SO100LeaderConfig | FeetechMotorsBus | SO-100 主臂 |
| `so101_leader` | SO101LeaderConfig | FeetechMotorsBus | SO-101 主臂 |
| `bi_so100_leader` | BiSO100LeaderConfig | FeetechMotorsBus x2 | SO-100 双臂版主臂 |
| `gamepad` | GamepadConfig | (HID) | 游戏手柄遥操作 |
| `keyboard` | KeyboardConfig | (pynput) | 键盘遥操作 |
| `keyboard_ee` | KeyboardEEConfig | (pynput) | 键盘末端执行器控制 |
| `widowx` | WidowXConfig | DynamixelMotorsBus | WidowX 主臂 |
| `stretch3` | Stretch3TeleopConfig | — | Stretch 3 游戏手柄 |
| `reachy2_teleoperator` | Reachy2TeleoperatorConfig | (reachy2_sdk) | Reachy2 遥操作 |
| `homunculus_glove` | HomunculusGloveConfig | — | 数据手套 |
| `homunculus_arm` | HomunculusArmConfig | — | 外骨骼臂 |
| `phone` | PhoneConfig | (hebi-py) | 手机遥操作 |

---

## 4.5 MotorsBus 基类详解

`MotorsBus` 是 LeRobot 框架中最接近物理硬件的抽象层，定义在 `src/lerobot/motors/motors_bus.py` 中（共 1220 行）。它封装了与一串串联电机（daisy-chained motors）通信的完整逻辑。

### 4.4.1 Motor 数据类

```python
# motors_bus.py 第 95-99 行
@dataclass
class Motor:
    id: int              # 电机 ID (1-7)
    model: str           # 电机型号 ("AGILEX-M", "AGILEX-S", "HTDW-5047")
    norm_mode: MotorNormMode  # 归一化模式
```

每个 `Motor` 对象描述一个物理电机的三要素元数据：

- **id**：电机在总线上的唯一标识。在 Dynamixel/Feetech 协议中，id 用于定址；在 PiPER 中，id 是逻辑编号（1=joint1, 2=joint2, ..., 7=gripper）。
- **model**：电机型号字符串。不同型号有不同的控制表（ctrl_table）、分辨率（resolution_table）和型号编码（model_number_table）。例如 AGILEX-M 是 PiPER 的大关节电机（1190），AGILEX-S 是小关节和夹爪电机（1191），HTDW-5047 是主臂示教电机的型号。
- **norm_mode**：归一化模式，决定了从原始编码器值映射到归一化值时的公式。

### 4.4.2 MotorNormMode 枚举

```python
# motors_bus.py 第 80-83 行
class MotorNormMode(str, Enum):
    RANGE_0_100 = "range_0_100"
    RANGE_M100_100 = "range_m100_100"
    DEGREES = "degrees"
```

| 模式 | 归一化范围 | 使用场景 | 公式 |
|------|-----------|---------|------|
| `RANGE_M100_100` | [-100, 100] | 旋转关节 | `((raw - min) / (max - min)) * 200 - 100` |
| `RANGE_0_100` | [0, 100] | 夹爪 | `((raw - min) / (max - min)) * 100` |
| `DEGREES` | 角度制 | 旋转关节（以度为单位） | `(raw - mid) * 360 / max_resolution` |

PiPER 执行臂的关节 1~6 使用 `RANGE_M100_100`，夹爪使用 `RANGE_0_100`。PiPER 不使用 `DEGREES` 模式（在主臂/执行臂的配置中均无此用法）。

### 4.4.3 MotorCalibration 数据类

```python
# motors_bus.py 第 86-92 行
@dataclass
class MotorCalibration:
    id: int
    drive_mode: int      # 驱动模式 (0=正向, 1=反向)
    homing_offset: int   # 回零偏置
    range_min: int       # 最小原始值
    range_max: int       # 最大原始值
```

这是理解归一化系统的关键数据结构。`range_min` 和 `range_max` 定义了每个关节在 SDK 原始坐标系中的运动范围。由于不同关节的物理运动范围不同，必须为每个关节分别设置。

以 PiperFollower 的标定为例（来自 `piper_follower.py` 第 60-68 行）：

```python
calibration={
    "joint1":  MotorCalibration(1, 0, 0, -150000,  150000),  # ±150k 对称
    "joint2":  MotorCalibration(2, 0, 0,       0,  180000),  # 只能正向 0~180°
    "joint3":  MotorCalibration(3, 0, 0, -170000,       0),  # 只能负向 -170°~0
    "joint4":  MotorCalibration(4, 0, 0, -100000,  100000),  # ±100k
    "joint5":  MotorCalibration(5, 0, 0,  -65000,   65000),  # ±65k
    "joint6":  MotorCalibration(6, 0, 0, -100000,  130000),  # 不对称
    "gripper": MotorCalibration(7, 0, 0,       0,   68000),  # 0~68k
}
```

如果不使用 `range_min/max` 而使用固定范围（如统一假设所有关节都是 [0, 4096]），那么 joint2（实际范围 [0, 180000]）的归一化会完全错误——一个微小的物理运动会被放大为巨大的归一化变化，反之亦然。`range_min/range_max` 确保每个关节的归一化都精确映射到它的真实物理运动范围。

### 4.4.4 MotorsBus 抽象类核心设计

`MotorsBus` 是一个 `abc.ABC` 抽象类（第 212 行），其构造函数接受：

```python
def __init__(
    self,
    port: str,                              # "can1", "/dev/ttyUSB0" 等
    motors: dict[str, Motor],               # {"joint1": Motor(1, "AGILEX-M", ...), ...}
    calibration: dict[str, MotorCalibration] | None = None,
):
```

构造函数执行后，内部会建立三个关键的映射字典（第 280-282 行）：

```python
self._id_to_model_dict = {m.id: m.model for m in self.motors.values()}
self._id_to_name_dict = {m.id: motor for motor, m in self.motors.items()}
self._model_nb_to_model_dict = {v: k for k, v in self.model_number_table.items()}
```

这些映射使得框架可以通过电机 ID（整数）快速查找到电机名称（字符串）和型号。

**核心方法清单：**

| 方法 | 功能 | 在 PiPER 中 |
|------|------|------------|
| `connect()` / `disconnect()` | 打开/关闭通信端口，可选握手 | 打开 CAN 口 / 关闭 CAN 口 |
| `get_action()` → `_normalize()` | 读取当前电机位置并归一化 | 调 piper_sdk 读编码器 → 归一化 |
| `set_action()` → `_unnormalize()` | 将归一化值反归一化后发送给电机 | 反归一化 → JointCtrl/GripperCtrl |
| `sync_read()` / `sync_write()` | 批量读/写多个电机的同一寄存器 | PiPER 不使用（CAN 是广播协议） |
| `parking()` | 机械臂回到预定义的安全位置 | 发送 INITIALIZE_POSITION |
| `enable_torque()` / `disable_torque()` | 使能/失能电机扭矩 | EnablePiper / DisablePiper |
| `read()` / `write()` | 读/写单个电机的单个寄存器 | PiPER 不使用（CAN 不走寄存器协议） |

**Dynamixel 特有方法（PiPER 中为空实现）：**

这些方法存在于抽象接口中是因为 DynamixelMotorsBus 和 FeetechMotorsBus 需要它们，但 PiperMotorsBus 将其全部实现为空或直通（pass-through）：

```python
# PiperMotorsBus 中
def _handshake(self):                # 不需要握手协议
    pass

def broadcast_ping(self, ...):       # 不需要广播 ping
    pass

def _find_single_motor(self, ...):   # 不需要逐电机扫描
    pass

def _get_half_turn_homings(self, ...): # 不需要半圈回零
    pass

def _encode_sign(self, data_name, ids_values):
    return ids_values                # 不需要符号编码，直通

def _decode_sign(self, data_name, ids_values):
    return ids_values                # 不需要符号解码，直通

def _split_into_byte_chunks(self, ...):
    pass                             # piper_sdk 内部处理 CAN 帧
```

这个设计说明了 LeRobot 抽象层次的权衡：MotorsBus 基类继承目标是覆盖 Dynamixel 和 Feetech 的通用需求，PiperMotorsBus 作为一个"非标准"实现，将不适用的方法空实现，而覆盖了核心方法（`_normalize`, `_unnormalize`, `connect`, `disconnect`, `enable_torque` 等）来接入自己的 SDK。

### 4.4.5 _normalize 和 _unnormalize 数学原理

`_normalize` 和 `_unnormalize` 是框架最核心的两个方法，定义在 MotorsBus 基类中（第 776-833 行），但 PiperMotorsBus 重写了它们以适配自己的 SDK 值传递方式。以下是它们的确切数学公式：

**RANGE_M100_100 模式（关节）：**

```
归一化:   norm = ((raw - min) / (max - min)) * 200 - 100
反归一化: raw  = round(((norm + 100) / 200) * (max - min) + min)

示例 joint1 [min=-150000, max=150000]:
  raw =      0 → norm =  0.0   (中位)
  raw = 150000 → norm = 100.0  (正向极限)
  raw =-150000 → norm = -100.0 (负向极限)
```

**RANGE_0_100 模式（夹爪）：**

```
归一化:   norm = ((raw - min) / (max - min)) * 100
反归一化: raw  = round((norm / 100) * (max - min) + min)

示例 gripper [min=0, max=68000]:
  raw =     0 → norm = 0.0   (全开)
  raw = 68000 → norm = 100.0 (全闭)
```

两个方法都在开头和结尾做了**双层安全钳位**：

1. `_normalize` 中：`bounded_val = min(max_, max(min_, val))`——防止编码器异常值越界
2. `_unnormalize` 中：`bounded_val = min(100.0, max(-100.0, val))`——防止策略模型输出超出预期范围

当 `apply_drive_mode` 为 `True` 且 `calibration.drive_mode` 为 `True` 时（仅 Dynamixel 舵机使用），公式中会多一次符号反转。PiPER 中 `apply_drive_mode = False`，所以这条路径永远不会被触发。

---

## 4.6 Robot 基类详解

`Robot` 抽象类定义在 `src/lerobot/robots/robot.py`（共 185 行），是所有 LeRobot 兼容机器人的最顶层抽象。

### 4.5.1 类属性

```python
# robot.py 第 42-44 行
config_class: builtins.type[RobotConfig]   # 关联的配置类类型
name: str                                  # 机器人名称，如 "piper_follower"
```

每个子类必须设置这两个属性。例如 PiperFollower：

```python
class PiperFollower(Robot):
    config_class: PiperFollowerConfig   # 类型注解
    name = "piper_follower"             # 实际值
```

`name` 被用于生成标定文件的存储路径：`HF_LEROBOT_HOME/calibration/robots/{name}/{id}.json`。

### 4.5.2 构造函数

```python
def __init__(self, config: RobotConfig):
    self.robot_type = self.name
    self.id = config.id
    self.calibration_dir = (
        config.calibration_dir if config.calibration_dir
        else HF_LEROBOT_CALIBRATION / ROBOTS / self.name
    )
    self.calibration_dir.mkdir(parents=True, exist_ok=True)
    self.calibration_fpath = self.calibration_dir / f"{self.id}.json"
    self.calibration: dict[str, MotorCalibration] = {}
    if self.calibration_fpath.is_file():
        self._load_calibration()
```

构造函数负责：
1. 确定标定文件的存储目录（可以自定义或使用默认的 `~/.cache/huggingface/lerobot/calibration/robots/{name}/`）
2. 如果已有标定文件，自动加载（`_load_calibration` 使用 draccus 的 JSON 反序列化，可直接将 JSON 字典转换回 `dict[str, MotorCalibration]`）

### 4.5.3 抽象属性与方法

| 抽象成员 | 类型 | 说明 |
|----------|------|------|
| `observation_features` | property → dict | 描述观测空间的 key 和类型。Key 如 `"joint1.pos"`, `"joint2.pos"`, `"wrist"`, `"top"`；Value 为 `float` 或 `(H, W, C)` 形状元组 |
| `action_features` | property → dict | 描述动作空间的 key 和类型。Key 如 `"joint1.pos"`...`"gripper.pos"`；Value 通常全为 `float` |
| `is_connected` | property → bool | 机器人当前是否已连接 |
| `is_calibrated` | property → bool | 机器人当前是否已标定 |
| `connect(calibrate)` | 方法 | 建立通信。`calibrate=True` 时自动执行标定 |
| `disconnect()` | 方法 | 断开通信并清理资源 |
| `calibrate()` | 方法 | 执行标定过程 |
| `configure()` | 方法 | 设置电机参数、控制模式等一次性配置 |
| `get_observation()` | 方法 | 读取当前观测（电机位置 + 相机图像），返回 dict |
| `send_action(action)` | 方法 | 发送动作命令给机器人，返回实际发出的动作 dict |

### 4.5.4 以 PiperFollower 为例的完整实现

PiperFollower 定义在 `src/lerobot/robots/piper_follower/piper_follower.py` 中（共 187 行）。以下是关键实现：

**构造函数（第 44-70 行）——创建 MotorsBus：**

```python
def __init__(self, config: PiperFollowerConfig):
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
        calibration={
            "joint1": MotorCalibration(1, 0, 0, -150000, 150000),
            # ... (见 4.4.3 节)
        }
    )
    self.cameras = make_cameras_from_configs(config.cameras)
```

注意 Joint 1-3 使用 `AGILEX-M`（大关节电机），Joint 4-6 使用 `AGILEX-S`（小关节电机），这与物理硬件的真实配置一致。

**observation_features（第 86-88 行）：**

```python
@cached_property
def observation_features(self) -> dict:
    return {**self._motors_ft, **self._cameras_ft}
```

其中 `_motors_ft` 生成 `{"joint1.pos": float, "joint2.pos": float, ..., "gripper.pos": float}`，`_cameras_ft` 生成 `{"wrist": (480, 640, 3), "top": (480, 640, 3), ...}`。两者合并为完整的观测特征字典。

**get_observation（第 133-153 行）：**

```python
def get_observation(self) -> dict[str, Any]:
    obs_dict = {}
    # 读电机位置（已归一化到 [-100,100] / [0,100]）
    obs_dict = self.bus.get_action()
    obs_dict = {f"{motor}.pos": val for motor, val in obs_dict.items()}
    # 读相机帧
    for cam_key, cam in self.cameras.items():
        obs_dict[cam_key] = cam.async_read()
    return obs_dict
```

`bus.get_action()` 返回的键是 `{"joint1": 33.5, ..., "gripper": 50.0}`，通过字典推导式加上 `.pos` 后缀变成 `{"joint1.pos": 33.5, ..., "gripper.pos": 50.0}`，符合 LeRobot 数据集规范。

**send_action（第 155-170 行）：**

```python
def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
    goal_pos = {key.removesuffix(".pos"): val
                for key, val in action.items() if key.endswith(".pos")}
    # 可选的安全限幅
    if self.config.max_relative_target is not None:
        present_pos = self.bus.sync_read("Present_Position")
        goal_pos = ensure_safe_goal_position(
            {key: (g_pos, present_pos[key])
             for key, g_pos in goal_pos.items()},
            self.config.max_relative_target
        )
    rlt = self.bus.set_action(goal_pos)
    return {f"{motor}.pos": val for motor, val in rlt.items()}
```

这里有一个重要的**安全保护机制**——`max_relative_target`：如果模型输出的目标位置与当前位置的差值超过阈值，会自动钳位到阈值范围。这可以防止策略模型输出异常大的动作导致机械臂剧烈运动。

### 4.5.5 标定框架

基类提供了 `_load_calibration` 和 `_save_calibration` 两个方法（第 125-145 行），使用 draccus 的 JSON 序列化功能：

```python
def _load_calibration(self, fpath: Path | None = None) -> None:
    fpath = self.calibration_fpath if fpath is None else fpath
    with open(fpath) as f, draccus.config_type("json"):
        self.calibration = draccus.load(dict[str, MotorCalibration], f)

def _save_calibration(self, fpath: Path | None = None) -> None:
    fpath = self.calibration_fpath if fpath is None else fpath
    with open(fpath, "w") as f, draccus.config_type("json"):
        draccus.dump(self.calibration, f, indent=4)
```

PiperFollower 将这两个方法覆盖为空实现（第 120-124 行），因为 PiPER 的标定值是硬编码在 `__init__` 中的静态值，不需要存取到文件。

---

## 4.7 Teleoperator 基类详解

`Teleoperator` 抽象类定义在 `src/lerobot/teleoperators/teleoperator.py`（共 181 行）。它与 `Robot` 的结构非常相似，但有几个关键区别。

### 4.6.1 Robot vs Teleoperator 对比

```
┌─────────────────────────────────┐  ┌──────────────────────────────────┐
│  Robot (执行臂)                  │  │  Teleoperator (主臂/示教臂)        │
├─────────────────────────────────┤  ├──────────────────────────────────┤
│  observation_features           │  │  (没有 observation_features)      │
│  ─ 电机位置 + 相机图像            │  │                                  │
│                                 │  │                                  │
│  action_features                │  │  action_features                 │
│  ─ 接收并执行动作                 │  │  ─ 输出动作（用户的示教动作）       │
│                                 │  │                                  │
│  get_observation() → obs        │  │  get_action() → action           │
│  ─ 读电机 + 读相机                │  │  ─ 读主臂关节状态                  │
│                                 │  │                                  │
│  send_action(action)            │  │  send_feedback(feedback)         │
│  ─ 执行动作                       │  │  ─ 可选的力反馈（PiPER 未实现）     │
│                                 │  │                                  │
│  feedback_features (无)         │  │  feedback_features               │
│                                 │  │  ─ 力反馈特征的描述                │
└─────────────────────────────────┘  └──────────────────────────────────┘
```

### 4.6.2 核心属性

| Teleoperator 成员 | 类型 | 说明 |
|-------------------|------|------|
| `config_class` | type | 关联的 TeleoperatorConfig 类型 |
| `name` | str | 唯一名称，如 `"piper_leader"` |
| `action_features` | property → dict | 主臂输出的动作特征，如 `{"joint1.pos": float, ...}` |
| `feedback_features` | property → dict | 力反馈特征（PiperLeader 返回 `{}`，未实现） |
| `is_connected` | property → bool | 主臂是否已连接 |
| `is_calibrated` | property → bool | 主臂是否已标定（PiperLeader 始终返回 `True`） |

### 4.6.3 以 PiperLeader 为例

PiperLeader 定义在 `src/lerobot/teleoperators/piper_leader/piper_leader.py`（共 111 行）。与 PiperFollower 的对比：

| 对比维度 | PiperFollower (执行臂) | PiperLeader (主臂) |
|----------|----------------------|-------------------|
| 电机型号 | AGILEX-M / AGILEX-S | HTDW-5047 |
| 电机数量 | 6 + 1 (关节 + 夹爪) | 6 + 1 (关节 + 夹爪) |
| 标定信息 | 硬编码完整的 range_min/max | 无标定 |
| connect 行为 | 连接 → 使能扭矩 → parking 回零 | 连接 → 使能扭矩 |
| 状态读取 | `get_observation()` 读编码器 + 相机 | `get_action()` 只读编码器 |
| 动作发送 | `send_action()` 调 `bus.set_action()` | 不发送动作 |
| 力反馈 | 无 | `send_feedback()` 空实现 |

关键区别在于**职责分离**：

- **PiperLeader** 的角色是"传感器"——把用户的示教动作转换成归一化数值输出给数据采集流水线。它不执行动作，只读状态。
- **PiperFollower** 的角色是"执行器"——接收策略模型的动作指令，转换成物理运动。它也提供传感器读数（编码器位置 + 相机图像）用于形成观测。

在 `lerobot-record` 的数据采集中，两者同时工作：

```
  用户手动拖动主臂
       │
       ▼
  PiperLeader.get_action() → raw_action → 存为数据集中的 action
       │
       │  (同一时刻)
       │
       ▼
  PiperFollower.get_observation() → raw_obs → 存为数据集中的 observation
```

注意：在这种采集模式下，执行臂（PiperFollower）的 `send_action()` 并没有被调用—它在采集中充当的是被动传感器，记录"在执行这个动作之前，机械臂在什么位置"。动作的来源是主臂（PiperLeader），而不是执行策略模型。

---

## 4.8 pyproject.toml 注册机制

### 4.7.1 入口点注册

`pyproject.toml` 中的 `[project.scripts]` 部分定义了 CLI 入口点：

```toml
[project.scripts]
lerobot-calibrate="lerobot.calibrate:main"
lerobot-find-cameras="lerobot.find_cameras:main"
lerobot-find-port="lerobot.find_port:main"
lerobot-record="lerobot.record:main"
lerobot-replay="lerobot.replay:main"
lerobot-setup-motors="lerobot.setup_motors:main"
lerobot-teleoperate="lerobot.teleoperate:main"
lerobot-eval="lerobot.scripts.eval:main"
lerobot-train="lerobot.scripts.train:main"
```

每个等号左边是用户在 Shell 中敲的命令，右边是 `包.模块:函数` 的 Python 导入路径。当 `pip install` 执行时，setuptools 会为每个入口生成一个操作系统级的可执行脚本（在 `$PREFIX/bin/` 下），脚本的内容大致是：

```python
from lerobot.record import main
if __name__ == "__main__":
    sys.exit(main())
```

### 4.7.2 可选依赖分组

```toml
[project.optional-dependencies]
# 电机驱动
feetech = ["feetech-servo-sdk>=1.0.0"]
dynamixel = ["dynamixel-sdk>=3.7.31"]
piper = ["piper_sdk>=0.4.1"]

# 机器人
gamepad = ["lerobot[pygame-dep]", "hidapi>=0.14.0"]
hopejr = ["lerobot[feetech]", "lerobot[pygame-dep]"]
lekiwi = ["lerobot[feetech]", "pyzmq>=26.2.1"]
reachy2 = ["reachy2_sdk>=1.0.14"]
# ...

# 策略
pi0 = ["lerobot[transformers-dep]"]
smolvla = ["lerobot[transformers-dep]", "num2words>=0.5.14", ...]
hilserl = ["lerobot[transformers-dep]", "gym-hil>=0.1.9", ...]

# 全部
all = ["lerobot[dynamixel]", "lerobot[gamepad]", "lerobot[hopejr]", ...]
```

这种分组设计的好处是：
- `pip install lerobot[piper]` 只安装 PiPER 需要的 `piper_sdk`，不安装 `dynamixel-sdk` 或 `feetech-servo-sdk`
- 在 Docker 镜像或 CI 环境中可以用 `pip install lerobot[all]` 安装全部依赖
- 依赖之间可以相互引用（如 `hopejr = ["lerobot[feetech]", "lerobot[pygame-dep]"]`）

### 4.7.3 完整的安装与发现流程

```
$ pip install -e ".[piper]"
   │
   ├─► 1. 安装 piper_sdk>=0.4.1 (CAN 总线通信库)
   │
   ├─► 2. setuptools 将 entry_points 注册到系统 PATH
   │
   └─► 3. 安装完成, Python 可以 import lerobot

$ lerobot-record --robot.type=piper_follower --robot.port=can0
   │
   ├─► shell 执行 lerobot-record → lerobot/record.py:main()
   │
   ├─► @parser.wrap() 装饰器触发参数解析
   │
   ├─► import lerobot.robots 触发所有机器人模块的 __init__.py
   │   └─► 执行 @RobotConfig.register_subclass("piper_follower")
   │       → RobotConfig._subclasses["piper_follower"] = PiperFollowerConfig
   │
   ├─► draccus 解析 --robot.type=piper_follower
   │   └─► 查找 RobotConfig._subclasses["piper_follower"]
   │       → 找到 PiperFollowerConfig → 用剩余 CLI 参数实例化
   │
   └─► make_robot_from_config(config) → PiperFollower(config)
       └─► PiperFollower.__init__() → PiperMotorsBus.__init__()
           └─► C_PiperInterface_V2("can0") → 开始通信
```

---

## 4.9 数据格式标准

### 4.8.1 LeRobot Dataset 目录结构

```
~/.cache/huggingface/lerobot/my_user/my_dataset/
├── meta/
│   └── info.json          # 数据集元信息
│       ├── robot_type: "piper_follower"
│       ├── features: {...}  # observation.state + action 的格式定义
│       ├── fps: 30
│       ├── total_episodes: 50
│       └── total_frames: ...
├── data/
│   ├── episode_000000.parquet
│   ├── episode_000001.parquet
│   └── ...
└── videos/
    ├── observation.images.wrist/
    │   ├── episode_000000.mp4
    │   └── episode_000001.mp4
    └── observation.images.top/
        ├── episode_000000.mp4
        └── episode_000001.mp4
```

### 4.8.2 Observation 和 Action 的 Key 命名规范

LeRobot 使用 `.` 作为层级分隔符，key 的命名遵循严格的语义约定：

```
observation.state          → 电机状态（位置、速度等）的集合
observation.images.wrist   → 腕部相机图像
observation.images.global  → 全局相机图像
observation.language       → 语言指令（多任务场景）
observation.environment_state → 环境状态（用于 Gym 环境）

action                     → 动作空间
```

在 PiperFollower 中，这些 key 是通过 `observation_features` 和 `action_features` 属性产生的：

**observation_features 示例：**

```python
{
    "joint1.pos": float,      # 关节 1 位置
    "joint2.pos": float,      # 关节 2 位置
    "joint3.pos": float,      # 关节 3 位置
    "joint4.pos": float,      # 关节 4 位置
    "joint5.pos": float,      # 关节 5 位置
    "joint6.pos": float,      # 关节 6 位置
    "gripper.pos": float,     # 夹爪位置
    "wrist": (480, 640, 3),   # 腕部相机图像 (H, W, C)
    "top": (480, 640, 3),     # 顶部相机图像 (H, W, C)
}
```

**action_features 示例：**

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

注意：`action_features` 只包含电机位置（由策略模型输出），不包含图像。模型不需要输出图像。

### 4.8.3 特征类型定义规范

在 `src/lerobot/constants.py` 中定义了数据集中使用的标准 key 常量：

```python
OBS_ENV_STATE = "observation.environment_state"
OBS_STATE = "observation.state"
OBS_IMAGE = "observation.image"
OBS_IMAGES = "observation.images"
OBS_LANGUAGE = "observation.language"
ACTION = "action"
REWARD = "next.reward"
TRUNCATED = "next.truncated"
DONE = "next.done"
```

PiperFollower 的 `get_observation()` 返回的是一个**扁平字典**（flat dict），每个 key 直接对应一个值或数组：

```python
{
    "joint1.pos": 33.5,
    "joint2.pos": 50.0,
    ...
    "gripper.pos": 50.0,
    "wrist": np.ndarray(shape=(480, 640, 3), dtype=uint8),
    "top": np.ndarray(shape=(480, 640, 3), dtype=uint8),
}
```

LeRobot 数据集框架负责将这个扁平字典序列化到 parquet 文件（电机状态）和 mp4 视频文件（图像）。parquet 中每一行是一帧，包含所有标量观测和动作值；视频文件存储对应帧的图像帧。

---

## 4.10 LeRobot 支持的其他机器人

下表汇总了 LeRobot 代码库中已注册的所有机器人类型，展示 PiPER 在整个生态系统中的位置：

| 类型名 | 厂商/型号 | 电机总线 | 自由度 | 价格区间 | 特点 |
|--------|----------|---------|--------|---------|------|
| `piper_follower` | AgileX PiPER | CAN (piper_sdk) | 6+1 | 中高端 | 双臂平台，工业级 CAN 通信 |
| `koch_follower` | Koch v1.1 | Dynamixel (RS-485) | 6+1 | 低成本 | 开源低成本的先驱设计 |
| `so100_follower` | SO-100 | Feetech (RS-485) | 6+1 | 低成本 | Koch 的改进版 |
| `so101_follower` | SO-101 | Feetech (RS-485) | 6+1 | 低成本 | SO-100 的升级版 |
| `bi_so100_follower` | SO-100 双臂 | Feetech (RS-485) x2 | 2x(6+1) | 低成本 | 双臂低成本方案 |
| `lekiwi` | LeKiwi | Feetech + ZeroMQ | 6 | 教育 | 面向教学的桌面机器人 |
| `reachy2` | Pollen Robotics | reachy2_sdk | 7+ | 高端 | 灵巧手 + 双臂 + 头部 |
| `stretch3` | Hello Robot | Dynamixel | 3+ | 中高端 | 移动操作平台 |
| `viperx` | Trossen Robotics | Dynamixel | 6+1 | 中端 | Interbotix 系列 |
| `hope_jr_arm` | Hope Jr. | Feetech | — | 教育 | 教育平台的手臂部分 |
| `hope_jr_hand` | Hope Jr. | Feetech | — | 教育 | 教育平台的灵巧手部分 |

### PiPER 在生态系统中的独特定位

与基于 Dynamixel 或 Feetech 舵机的低成本方案（Koch、SO-100）相比，PiPER 有以下几个独特特点：

1. **CAN 总线通信**——PiPER 使用 CAN（Controller Area Network）总线而非 RS-485 串口。CAN 支持更高的带宽（最高 1 Mbps）和更强的抗干扰能力，适合工业环境。但这也意味着 PiPER 的 MotorsBus 实现不能复用 Dynamixel/Feetech 的 `sync_read`/`sync_write` 协议栈，而是直接通过 `piper_sdk` 的高层 API（`GetArmJointMsgs`、`JointCtrl` 等）完成通信。

2. **非标电机**——PiPER 使用 AgileX 定制的 AGILEX-M 和 AGILEX-S 电机，它们不兼容 Dynamixel 的控制表协议。因此 PiperMotorsBus 不需要 `_handshake`、`broadcast_ping`、`_find_single_motor`、`configure_motors` 等方法，这些在 Dynamixel/Feetech 上至关重要的方法在 PiPER 上都是空实现。

3. **静态标定**——大多数 Dynamixel/Feetech 机器人需要执行动态标定过程（读取编码器、移动关节、计算范围）。PiPER 的关节运动范围在硬件层面是固定的，标定值直接硬编码在 `PiperFollower.__init__` 中。这简化了标定流程，但也意味着如果物理更换了一个运动范围不同的关节，需要手动修改源代码中的标定值。

4. **双臂原生支持**——PiPER 是设计为双臂平台的：一只臂作为 Leader（主臂，用户拖动示教），一只臂作为 Follower（执行臂，执行策略）。两只臂通过各自的独立 CAN 口连接（主臂 `can0`，执行臂 `can1`），LeRobot 框架将 Leader 注册为 Teleoperator 类型，Follower 注册为 Robot 类型。这种架构与 LeRobot 的 Teleoperator → Robot 数据流水线完美契合。

---

## 本章小结

本章从源代码层面深入剖析了 LeRobot 框架的四层架构：从 CLI 入口点到 Robot/Teleoperator 抽象层，再到 MotorsBus 硬件抽象层，最终到达 piper_sdk 的硬件通信层。

核心设计理念可以概括为：

- **ChoiceRegistry 的自动发现机制**使得新增硬件类型只需一个装饰器声明，无需修改框架核心代码
- **MotorsBus 的归一化/反归一化系统**将所有不同量纲的原始编码器值统一映射到 [-100, 100] 或 [0, 100]，使得策略模型可以跨硬件迁移
- **Robot 与 Teleoperator 的职责分离**使数据采集流水线中的"谁产生动作"和"谁记录状态"两个角色清晰独立
- **MotorCalibration 的 per-joint 标定**确保每个关节的归一化精确反映其真实物理运动范围

理解这一层抽象对于后续理解 `lerobot-record` 的数据采集流程（第 5 章）和策略训练流程（第 6 章）至关重要，因为这些流程都建立在 Robot 和 Teleoperator 这两个抽象接口之上。
