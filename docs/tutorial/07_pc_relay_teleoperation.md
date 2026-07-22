# 第七章 PC 中继遥操作

本章是 PiPER + LeRobot 模仿学习流水线中**最核心的日常操作章节**。你将深入理解 PC 中继方案的设计动机、架构原理、代码实现和操作流程。阅读完本章后，你应当能够：

- 理解为什么需要 PC 中继，以及它与硬件主从模式和 LeRobot 内置 teleoperate 的区别
- 掌握 `piper_pc_relay_teleop.py` 的每一行代码逻辑
- 掌握 `record_with_pc_relay.sh` 的完整采集流程
- 独立调试中继遥操作中的常见问题
- 根据实际需求调整关节变换、速率限制等参数

本章内容覆盖以下两个源文件：

| 文件 | 行数 | 角色 |
|------|------|------|
| `scripts/piper_pc_relay_teleop.py` | 309 | PC 中继引擎：读主臂、变换、限速、写从臂 |
| `record_with_pc_relay.sh` | 228 | 采集编排器：启动中继、配置相机、调用 lerobot-record |

---

## 7.1 为什么需要 PC 中继

### 7.1.1 问题场景：主臂和从臂在不同 CAN 总线上

在 PiPER 双臂协作机器人系统中，主臂（Leader）和从臂（Follower）各自连接一个独立的 USB-CAN 适配器，在操作系统中分别呈现为 `can0` 和 `can1` 两个网络接口：

```
┌──────────────────┐          ┌──────────────────┐
│   USB-CAN 适配器 A │          │   USB-CAN 适配器 B │
│   (can0)          │          │   (can1)          │
└────────┬─────────┘          └────────┬─────────┘
         │ CAN 总线                     │ CAN 总线
         │                              │
    ┌────┴────┐                    ┌────┴────┐
    │ 主臂    │                    │ 从臂    │
    │ Leader  │                    │ Follower│
    │ CAN ID: │                    │ CAN ID: │
    │ 0x2A5   │                    │ 0x155   │
    │ 0x2A6   │                    │ 0x156   │
    │ 0x2A7   │                    │ 0x157   │
    └─────────┘                    └─────────┘
```

主臂以 1kHz 频率通过 CAN ID `0x2A5`-`0x2A7` 上报 6 个关节的实时位置。从臂通过 CAN ID `0x155`-`0x157` 接收关节目标位置命令。这两个 CAN 总线在电气和协议层面完全隔离——主臂发出的 CAN 帧物理上无法到达从臂所在的 `can1` 总线，反之亦然。

这并非设计缺陷，而是一种**刻意为之的拓扑选择**：
- 每个 USB-CAN 适配器有其带宽上限（通常 1 Mbps），将两个臂分到不同总线可避免带宽争用
- 调试时可以独立插拔某一个臂的 CAN 线，不影响另一个臂的通信
- 如果将来需要扩展更多外设（如力传感器、额外的编码器），独立的 CAN 总线提供了物理隔离的扩展空间

但这也意味着：如果不在 PC 上编写软件桥接程序，主臂和从臂之间无法直接通信。

### 7.1.2 硬件主从模式的局限

PiPER 的电机驱动器（型号 HTDW-5047 和 AGILEX-M/S）在固件层面支持一种"硬件主从模式"：将主臂的某个 CAN ID 配置为将关节状态直接转发到从臂对应的 CAN ID。这种模式在理论上可以实现"零延迟"的关节跟随。

然而，硬件主从模式有若干无法回避的局限：

**1. 需要 CAN ID 偏移配置**

硬件主从模式要求主臂和从臂的 CAN ID 之间有一个固定的偏移量。例如，主臂上报 `0x2A5`，从臂监听 `0x2A5 - offset`。这个偏移量需要写入电机固件的配置寄存器，操作繁琐且不可逆（需要通过专门的配置工具修改）。

在 PiPER 的默认固件中，主臂的上报 ID（`0x2A5`-`0x2A7`）和从臂的接收 ID（`0x155`-`0x157`）之间并无简单的偏移关系，这意味着如果使用硬件主从模式，需要**重新烧写至少一个臂的 CAN ID 配置**——这是一个高风险操作，稍有不慎会导致电机无法通信。

**2. 无法做关节变换**

硬件主从模式将主臂的编码器值**原封不动**地写入从臂。但实际使用中，以下场景需要关节变换：

- **方向翻转**：主臂和从臂可能采用镜像安装，关节 1 的正方向在主臂上是"顺时针"，在从臂上却是"逆时针"。如果直接转发编码器值，两个臂的运动方向会相反。
- **零点偏置**：主臂和从臂的机械零点可能不一致。主臂关节 2 在 0 度时，从臂关节 2 可能处于 +15 度的位置。需要加一个固定的偏置来对齐。

硬件主从模式只能做"1:1 直通"，无法插入任何数学变换。

**3. 无法做速率限制**

硬件主从模式将主臂的位置以 CAN 总线的原生速率（1 kHz）转发。当人快速拖拽主臂时，从臂会收到一个瞬时的大幅度位置跳变。虽然电机驱动器内部有自身的速度环和电流环保护，但**位置指令的突变**仍可能导致从臂产生不期望的抖动甚至过流保护。

PC 中继可以在软件层面做**增量限幅（per-step delta clamping）**，确保每一步的位置变化不超过安全阈值。

**4. 无法与数据采集协调**

硬件主从模式是一个纯硬件通路：主臂动 → 从臂动。它完全不知道"数据采集"这回事。如果 lerobot-record 在采集过程中需要暂停（例如进入 reset 阶段提示操作者复位环境），硬件主从模式下从臂会继续跟随主臂运动，导致复位过程中从臂处于不可控状态。

### 7.1.3 PC 中继的优势

PC 中继方案在 PC 上运行一个 Python 进程，通过 piper_sdk 同时连接 `can0`（读主臂）和 `can1`（写从臂），在软件层面构建一条可编程的数据通路。相比硬件主从模式，PC 中继有以下优势：

| 能力 | 硬件主从 | PC 中继 |
|------|----------|---------|
| **关节方向翻转** | 不支持（需重新安装机械臂） | 支持（`--signs` 参数） |
| **零点偏置对齐** | 不支持（需机械调整） | 支持（`--offset-deg` 参数） |
| **速率限制** | 依赖驱动器自身保护 | 支持（`--max-step-deg` 参数） |
| **关节限位** | 依赖硬件限位开关 | 支持（软件层面 clamp 到关节允许范围） |
| **与数据采集协调** | 不支持 | 支持（pause-file 机制） |
| **安全开关** | 上电即跟随 | `--execute` 显式启用，默认 dry-run |
| **调试可见性** | 无 | 实时打印主臂/从臂/命令的角度值 |
| **退出保护** | 直接断电可能导致臂坠落 | 退出时自动发送 standby 模式命令 |

### 7.1.4 为什么不用 LeRobot 内置的 teleoperate 功能

LeRobot 框架提供了内置的遥操作机制（`lerobot-record` 的 `--teleoperator.type` 和 `--teleoperator.port` 参数）。它的设计假设是：

- 主臂（Teleoperator）和从臂（Robot）是**同一型号**的机械臂（如两个 SO-100 臂）
- 数据通路是 `主臂.get_action() → lerobot-record → 从臂.send_action()`，即 lerobot-record 进程同时连接主臂和从臂
- 主臂和从臂可能共享同一个通信总线（如两个 SO-100 的 Dynamixel 舵机串联在同一条 RS-485 总线上）

这个假设在 PiPER 场景中不成立：

1. **PiPER 的 CAN 总线是独立的**：`can0` 和 `can1` 是两条物理上分离的 CAN 总线。lerobot-record 的架构期望一个 Robot 对象和一个 Teleoperator 对象，但 PiperFollower 和 PiperLeader 各自只能绑定一个 CAN 口。如果在同一个进程中创建两个 `C_PiperInterface_V2` 实例分别连 `can0` 和 `can1`，**piper_sdk 的单例模式会阻止第二个实例的创建**——因为 SDK 内部以 `can_name` 为键维护全局单例，但两个不同的 CAN 口需要两个独立的连接。

2. **需要一个独立于录制进程的中继**：如果中继逻辑嵌入在 lerobot-record 进程内，那么 lerobot-record 的崩溃（例如磁盘满导致写入失败）会导致中继停止，从臂失去控制。将中继放在独立进程中可以做到**故障隔离**——录制进程挂了，中继进程继续工作，从臂仍然跟随主臂。

3. **复位协调需要进程间通信**：lerobot-record 在 episode 之间需要暂停中继以执行复位操作。pause-file 是一种简单的进程间协调机制（文件系统作为信号量），两个独立进程可以通过文件的存在/不存在来同步状态。如果中继嵌入在 lerobot-record 内，这种协调会变得复杂（需要内部状态机）。

因此在 PiPER 的 PC 中继方案中：
- **lerobot-record 只负责录制**：它连接从臂（`--robot.type=piper_follower --robot.port=can1`），读取从臂的关节状态和相机图像，但**不连接主臂也不发送动作命令**。注意 `record_with_pc_relay.sh` 中**没有** `--teleoperator.type` 参数。
- **piper_pc_relay_teleop.py 负责中继**：它独立运行，连接 `can0`（读）和 `can1`（写），负责从主臂到从臂的全量数据转发和变换。

```
┌────────────────────────────────────────────────────────────────────┐
│                        PC 中继方案架构                               │
│                                                                    │
│  lerobot-record 进程                  piper_pc_relay_teleop.py 进程 │
│  ┌──────────────────────┐            ┌──────────────────────────┐  │
│  │ PiperFollower(can1)  │            │ Leader SDK(can0)         │  │
│  │   get_observation()  │            │   GetArmJointMsgs()      │  │
│  │   (只读，不写命令)     │            │        │                  │  │
│  │        ▲             │            │   关节变换 + 速率限制      │  │
│  │        │              │            │        ▼                  │  │
│  │   相机 + 关节状态      │            │ Follower SDK(can1)       │  │
│  │                      │            │   JointCtrl()            │  │
│  └──────────────────────┘            │   GripperCtrl()          │  │
│                                      └──────────────────────────┘  │
│  功能: 录制观测数据                       功能: 转发主臂动作到从臂       │
│  崩溃影响: 丢失当前 episode              崩溃影响: 从臂停住，不丢失数据  │
└────────────────────────────────────────────────────────────────────┘
```

---

## 7.2 架构设计

### 7.2.1 整体数据流

以下 ASCII 图展示了 PC 中继方案的完整数据流。请仔细跟踪每一条箭头——它刻画了从"人的手拖动主臂"到"从臂执行动作"再到"数据写入硬盘"的全过程。

```
                        CAN 总线 0 (can0)                    CAN 总线 1 (can1)
                     ════════════════════                ════════════════════
                     ║                  ║                ║                  ║
  ┌─────────┐        ║                  ║                ║                  ║        ┌─────────┐
  │ 主臂    │        ║                  ║                ║                  ║        │ 从臂    │
  │ Leader  │────────╬──► 0x2A5-0x2A7   ║                ║  0x155-0x157 ──╬───────►│ Follower│
  │         │        ║   (关节位置上报)   ║                ║  (关节目标命令)  ║        │         │
  │ 电机:   │        ║   1 kHz          ║                ║  100 Hz         ║        │ 电机:   │
  │ HTDW-   │        ║                  ║                ║                 ║        │ AGILEX  │
  │ 5047×7  │        ║                  ║                ║                 ║        │ M/S×7   │
  └─────────┘        ║                  ║                ║                 ║        └─────────┘
                     ║                  ║                ║                 ║
                     ║    USB-CAN A     ║                ║    USB-CAN B    ║
                     ╚══════╤═══════════╝                ╚══════╤═══════════╝
                            │                                   │
                            │ socketcan                         │ socketcan
                            ▼                                   ▼
  ┌─────────────────────────────────────────────────────────────────────────┐
  │                          PC (Ubuntu 22.04)                              │
  │                                                                         │
  │  ┌──────────────────────────────────────────────────────────────────┐   │
  │  │                   piper_pc_relay_teleop.py                        │   │
  │  │                                                                  │   │
  │  │  ┌───────────────────┐          ┌───────────────────┐            │   │
  │  │  │ C_PiperInterface  │          │ C_PiperInterface  │            │   │
  │  │  │ _V2("can0")       │          │ _V2("can1")       │            │   │
  │  │  │                   │          │                   │            │   │
  │  │  │ read_leader()     │          │ send_follower()   │            │   │
  │  │  │  ├─ feedback_sample│          │  ├─ JointCtrl()   │            │   │
  │  │  │  │  GetArmJointMsgs│          │  ├─ GripperCtrl() │            │   │
  │  │  │  │  GetArmGripperMsgs          │  └─ MotionCtrl_2 │            │   │
  │  │  │  └─ control_sample │          │                   │            │   │
  │  │  │     GetArmJointCtrl│          │                   │            │   │
  │  │  │     GetArmGripperCtrl          │                   │            │   │
  │  │  └────────┬──────────┘          └────────▲──────────┘            │   │
  │  │           │                              │                        │   │
  │  │           │    ┌─────────────────────┐    │                        │   │
  │  │           │    │   关节处理流水线      │    │                        │   │
  │  │           │    │                     │    │                        │   │
  │  │           ├───►│ ① transform_joints  │    │                        │   │
  │  │           │    │    sign * val       │    │                        │   │
  │  │           │    │    + offset_mdeg    │    │                        │   │
  │  │           │    │                     │    │                        │   │
  │  │           │    │ ② bounded_joints    │    │                        │   │
  │  │           │    │    clamp to         │    │                        │   │
  │  │           │    │    joint limits     │    │                        │   │
  │  │           │    │                     │    │                        │   │
  │  │           │    │ ③ rate_limit        │    │                        │   │
  │  │           │    │    clamp(delta,     │    │                        │   │
  │  │           │    │    ±max_step_mdeg)  │    │                        │   │
  │  │           │    └──────────┬──────────┘    │                        │   │
  │  │           │               └────────────────┘                        │   │
  │  │           │                                                        │   │
  │  │           │    ┌─────────────────────┐                              │   │
  │  │           │    │   暂停检测           │                              │   │
  │  │           │    │   os.path.exists(   │                              │   │
  │  │           │    │     pause_file)     │                              │   │
  │  │           │    └─────────────────────┘                              │   │
  │  └──────────────────────────────────────────────────────────────────┘   │
  │                                                                         │
  │  ┌──────────────────────────────────────────────────────────────────┐   │
  │  │                   record_with_pc_relay.sh                        │   │
  │  │                                                                  │   │
  │  │  ┌─────────────────┐          ┌─────────────────────────────┐    │   │
  │  │  │ 后台进程:       │          │ 前台进程:                    │    │   │
  │  │  │ relay_teleop    │          │ lerobot-record              │    │   │
  │  │  │ (PID=$RELAY_PID)│          │                             │    │   │
  │  │  │                 │          │ --robot.type=piper_follower │    │   │
  │  │  │ 通过 trap EXIT  │          │ --robot.port=can1           │    │   │
  │  │  │ 确保退出时清理   │          │ (无 --teleoperator.type)    │    │   │
  │  │  └─────────────────┘          └─────────────────────────────┘    │   │
  │  └──────────────────────────────────────────────────────────────────┘   │
  └─────────────────────────────────────────────────────────────────────────┘
```

### 7.2.2 关键设计决策

**决策 1：中继进程和录制进程分离**

中继进程（Python 脚本）和录制进程（lerobot-record CLI）是两个独立的操作系统进程。它们之间唯一的通信渠道是文件系统（pause-file）。这样做的原因：

- **故障隔离**：如果 lerobot-record 因为磁盘满、相机断连等原因崩溃，中继进程不受影响，从臂仍然安全跟随主臂。如果中继进程崩溃，lerobot-record 会记录到"从臂不再移动"，但已写入硬盘的数据不会丢失。
- **独立生命周期**：中继进程需要在整个采集会话期间持续运行（可能数小时），而录制进程在每个 episode 之间会进入不同的状态（录制态 → 复位态 → 录制态）。独立进程使得各自的启动、停止、异常处理逻辑互不干扰。
- **可替换性**：你可以单独升级中继脚本而不影响 lerobot-record，反之亦然。你也可以在调试时手动启停中继进程，而不影响已录制的数据。

**决策 2：默认 dry-run 模式（`--execute` 显式启用）**

中继脚本的默认行为是**不发送任何命令给从臂**（dry-run 模式）。只有显式传入 `--execute` 参数后，才会真正通过 CAN 总线向从臂发送 JointCtrl / GripperCtrl / MotionCtrl_2 命令。

这是一个关键的安全设计：如果你忘记插从臂的 CAN 线，或者在错误的配置下运行脚本，dry-run 模式确保不会发生意外运动。你可以在 dry-run 模式下观察控制台打印的主臂位置、变换后位置和限速后位置，确认一切正确后再加 `--execute` 真正控制从臂。

**决策 3：基于反馈位置（feedback position）的速率限制**

速率限制算法使用**从臂当前的实际位置**（通过 `feedback_sample()` 读取）作为基线，而不是上一次发送的命令位置。这样做的原因是：从臂可能因为外部阻力、电机力矩不足、通信延迟等原因没有精确到达上一次命令指定的位置。如果基于"命令位置"做限速，累积误差会越来越大。基于"实际反馈位置"做限速，可以天然消除累积误差——每一步的增量都是相对于"从臂真正到达的位置"来计算的。

---

## 7.3 piper_pc_relay_teleop.py 逐段解析

本节按照脚本的代码结构，从上到下解析每一个模块和函数。建议在阅读时保持源码在另一个窗口打开，对照阅读。

### 7.3.1 导入和常量定义

```python
from piper_sdk import C_PiperInterface_V2
```

脚本的唯一外部依赖是 `piper_sdk`，它提供了 CAN 通信的底层封装。`C_PiperInterface_V2` 是一个 C++ 扩展类（通过 pybind11 绑定），内部管理着一个后台线程持续收发 CAN 帧。注意 `V2` 后缀——这是 PiPER SDK 的第二个大版本，与第一版在 API 上有显著差异（例如 `ConnectPort` 的参数从位置参数变为关键字参数）。

```python
JOINT_LIMITS_MDEG = [
    (-150_000, 150_000),   # joint 1: ±150°
    (0, 180_000),          # joint 2: 0° ~ 180°
    (-170_000, 0),         # joint 3: -170° ~ 0°
    (-100_000, 100_000),   # joint 4: ±100°
    (-70_000, 70_000),     # joint 5: ±70°
    (-120_000, 120_000),   # joint 6: ±120°
]

GRIPPER_LIMIT_UM = (0, 68_000)
```

关节限位值以**毫度（mdeg，millidegree）**为单位。1 mdeg = 0.001 度，即 1000 mdeg = 1 度。使用整数的原因是 CAN 协议中角度值以编码器脉冲为单位传输，1 度对应 1000 个编码器单位。

为什么要存这些限位值？因为 PC 中继的 `bounded_joints()` 函数会在发送命令前将目标位置 clamp 到这些范围内。即使主臂的操作者将关节拖到了超出从臂物理极限的角度，从臂也不会尝试执行这个超出范围的目标——它会被 clamp 到最近的安全边界。这是一种纯软件层面的**二次安全保护**（一次保护是电机驱动器内部的限位）。

夹爪的限位单位是**微米（um）**，范围 0 到 68,000 um = 68 mm。这对应了夹爪平行开闭的行程。

### 7.3.2 ArmSample 数据类

```python
@dataclass
class ArmSample:
    joints: list[int]       # 6 个关节的编码器值 (mdeg)
    gripper: int            # 夹爪位置 (um)
    joint_hz: float         # 关节数据刷新率
    gripper_hz: float      # 夹爪数据刷新率
    joint_timestamp: float  # 关节数据时间戳
    source: str             # 数据来源: "feedback" 或 "control"
```

`ArmSample` 是一个纯数据容器，封装了从机械臂读取的一组完整的关节和夹爪状态。`joint_hz` 和 `joint_timestamp` 是从 CAN 消息的元数据中提取的，用于判断数据流是否正常——如果 `joint_hz <= 0` 且 `joint_timestamp <= 0`，说明 CAN 总线上没有有效的关节数据（可能 CAN 口未 up、波特率不匹配、或机械臂未上电）。

### 7.3.3 读取函数：三种数据来源

脚本提供了三种读取主臂位置的方式，由 `--source` 参数控制：

```python
JointSource = Literal["auto", "feedback", "control"]
```

#### 方式 1：feedback_sample — 读取反馈位置

```python
def feedback_sample(piper: C_PiperInterface_V2) -> ArmSample:
    joint_msg = piper.GetArmJointMsgs()
    gripper_msg = piper.GetArmGripperMsgs()
    joint = joint_msg.joint_state
    gripper = gripper_msg.gripper_state
    return ArmSample(
        joints=[int(joint.joint_1), ..., int(joint.joint_6)],
        gripper=int(gripper.grippers_angle),
        ...
        source="feedback",
    )
```

`GetArmJointMsgs()` 返回的是电机编码器**实际测量到的当前位置**（即反馈位置）。`GetArmGripperMsgs()` 同理。这是最"真实"的数据——它反映的是机械臂物理上所处的角度，而不是任何命令期望的角度。

使用场景：如果你希望从臂严格跟随主臂的**实际物理位置**（而不是主臂最后一次接收到的命令位置），使用 `feedback` 源。

#### 方式 2：control_sample — 读取控制位置

```python
def control_sample(piper: C_PiperInterface_V2) -> ArmSample:
    joint_msg = piper.GetArmJointCtrl()
    gripper_msg = piper.GetArmGripperCtrl()
    ...
    source="control",
```

`GetArmJointCtrl()` 返回的是**上一次发送给该臂的关节目标命令值**。在主臂（Leader）上，通常不会有外部进程向它发送命令——主臂的电机处于被拖动状态，电机驱动器持续采样编码器并更新此值。在某些固件版本中，`GetArmJointCtrl()` 返回的值可能比 `GetArmJointMsgs()` 更平滑（经过了驱动器内部的滤波）。

`GetArmGripperCtrl()` 同理，返回的是上一次发送给夹爪的目标位置。

使用场景：录制脚本 `record_with_pc_relay.sh` 默认使用 `RELAY_SOURCE=control`。

#### 方式 3：auto — 自动选择

```python
def read_leader(piper: C_PiperInterface_V2, source: JointSource) -> ArmSample:
    if source == "feedback":
        return feedback_sample(piper)
    if source == "control":
        return control_sample(piper)
    # auto
    ctrl = control_sample(piper)
    if ctrl.joint_hz > 0 or ctrl.joint_timestamp > 0:
        return ctrl
    return feedback_sample(piper)
```

`auto` 模式先尝试 `control_sample()`，如果控制数据流有效（`joint_hz > 0` 或 `joint_timestamp > 0`），则使用控制数据；否则回退到反馈数据。这是一种"优先使用更平滑的控制数据，但确保不会用无效数据"的策略。

#### 三种方式的对比

| 来源 | 读取的 API | 值的含义 | 平滑度 | 推荐场景 |
|------|-----------|---------|--------|---------|
| `feedback` | `GetArmJointMsgs()` | 编码器实测角度 | 可能有微小的测量噪声 | 需要最真实的物理角度 |
| `control` | `GetArmJointCtrl()` | 上一次命令角度 | 经过驱动器滤波，更平滑 | 数据采集（默认） |
| `auto` | 先 control 后 feedback | 自动选择 | 取决于可用数据 | 不确定时使用 |

### 7.3.4 关节变换：transform_joints

```python
def transform_joints(
    joints: list[int],
    signs: list[float],
    offsets_mdeg: list[float],
) -> list[int]:
    return [
        int(round(joint * sign + offset))
        for joint, sign, offset in zip(joints, signs, offsets_mdeg, strict=True)
    ]
```

变换公式：

```
cmd_val[i] = round(leader_val[i] × sign[i] + offset[i])
```

其中：
- `sign[i]` 是关节 `i` 的方向符号。`1.0` 表示方向相同，`-1.0` 表示方向反转。
- `offset[i]` 是以**毫度**为单位的偏置值。

**为什么需要方向符号？**

主臂和从臂的关节可能采用不同的安装方向。例如，如果主臂的关节 1 正方向是逆时针旋转（从顶部看），而从臂的关节 1 正方向是顺时针旋转。当人将主臂关节 1 逆时针旋转 30 度时，如果没有符号翻转，从臂也会逆时针旋转 30 度——但因为在它自己的坐标系中"逆时针"对应了相反的方向，实际效果是从臂向错误方向运动。

设置 `--signs "1,-1,1,1,1,1"` 即可只翻转关节 2 的方向。

**为什么需要偏置？**

主臂和从臂的机械零点可能不一致。例如，主臂关节 3 在编码器值为 0 时指向正前方，但从臂关节 3 在编码器值为 0 时指向下方 15 度。设置 `--offset-deg "0,0,-15,0,0,0"` 即可将关节 3 的零点偏移 -15 度，使两个臂在物理空间中对齐。

### 7.3.5 关节限位：bounded_joints

```python
def bounded_joints(joints: list[int]) -> list[int]:
    return [
        clamp(joint, low, high)
        for joint, (low, high) in zip(joints, JOINT_LIMITS_MDEG, strict=True)
    ]
```

该函数将变换后的目标关节位置 clamp 到 `JOINT_LIMITS_MDEG` 定义的每个关节的物理允许范围内。这一步在速率限制**之前**执行——如果变换后的目标已经超出了关节限位，先把它拉回安全范围，再做速率限制，避免速率限制在非法值上运算。

### 7.3.6 速率限制：rate_limit

这是整个中继脚本中最核心的安全机制。

```python
def rate_limit(
    current: Iterable[int],    # 从臂当前的反馈位置
    target: Iterable[int],     # 变换+限位后的目标位置
    max_step: int,             # 单步最大允许变化量 (mdeg)
) -> list[int]:
    limited = []
    for cur, tgt in zip(current, target, strict=True):
        delta = tgt - cur
        if delta > max_step:
            limited.append(cur + max_step)
        elif delta < -max_step:
            limited.append(cur - max_step)
        else:
            limited.append(tgt)
    return limited
```

算法逻辑（对每个关节独立执行）：

```
delta = target[i] - current[i]

if delta > +max_step:
    output[i] = current[i] + max_step    # 向目标方向移动一步，但不超过 max_step
elif delta < -max_step:
    output[i] = current[i] - max_step    # 向目标方向移动一步，但不超过 max_step
else:
    output[i] = target[i]                # 目标在安全范围内，直接采用
```

**关键参数：max_step_mdeg**

```python
max_step_mdeg = int(round(args.max_step_deg * 1000))
```

`--max-step-deg` 的默认值是 1.0 度。在 `record_with_pc_relay.sh` 中，`RELAY_MAX_STEP_DEG` 默认设为 10 度，`RELAY_HZ` 默认设为 20 Hz。这意味着每步最多允许变化 10 度，即最大关节速度为 10 度/步 × 20 步/秒 = 200 度/秒。这个速度对于桌面级协作机械臂来说已经非常快，同时仍然在安全范围内。

如果你发现从臂在跟随主臂快速运动时有"追不上"的感觉（命令位置与实际位置之间有显著的持续偏差），可以适当增大 `--max-step-deg`。如果你发现从臂抖动明显（关节位置在目标附近来回震荡），可以适当减小 `--max-step-deg`。

**为什么必须做速率限制？**

1. **防止瞬时跳变**：如果主臂的 CAN 数据因为电磁干扰出现了一个异常值（例如关节 1 突然从 30 度跳到 300 度），没有速率限制的话，从臂会尝试以最大速度冲向 300 度，可能导致碰撞。速率限制将这一步的增量限制在 max_step 以内，从臂不会"相信"单步的突变。
2. **平滑 CAN 数据流**：主臂以 1 kHz 上报位置，但中继循环可能以 10-20 Hz 运行（受限于 USB-CAN 适配器的写入带宽和 Python GIL）。两个相邻的中继周期之间，主臂可能已经移动了相当大的角度。速率限制将这个大增量拆分为多个小步，使从臂的运动更平滑。
3. **防止操作者失误**：如果操作者不小心快速甩动主臂，速率限制确保从臂不会以同样危险的速度甩动。

### 7.3.7 从臂配置和命令发送

**配置函数：configure_follower_for_joint_control**

```python
def configure_follower_for_joint_control(
    piper: C_PiperInterface_V2, speed: int, high_follow: bool
) -> None:
    mit_flag = 0xAD if high_follow else 0x00
    piper.MotionCtrl_2(0x01, 0x01, speed, mit_flag)
    time.sleep(0.05)
    piper.EnableArm()
    time.sleep(0.1)
```

`MotionCtrl_2` 的四个参数：
- `0x01`: 控制模式 = 关节位置控制
- `0x01`: 使能状态 = 使能
- `speed`: 速度百分比（0-100）。在 `record_with_pc_relay.sh` 中默认设为 100。
- `mit_flag`: MIT 模式标志。`0xAD` 启用高跟随模式（higher torque bandwidth），`0x00` 为普通模式。`record_with_pc_relay.sh` 默认启用 `RELAY_HIGH_FOLLOW=1`。

然后调用 `EnableArm()` 使能电机力矩，等待 100ms 使能生效。如果不调用 `EnableArm()`，即使发送了 `JointCtrl` 命令，从臂的电机也不会输出力矩——关节处于自由状态。

如果你希望在脚本启动时**不自动使能从臂**（例如你想手动确认一切正常后再使能），可以使用 `--skip-follower-config` 参数。这在调试阶段非常有用。

**命令发送函数：send_follower**

```python
def send_follower(
    piper: C_PiperInterface_V2,
    joints: list[int],        # 6 个关节的目标位置 (mdeg)
    gripper: int | None,      # 夹爪目标位置 (um)，None 表示不发送夹爪命令
    speed: int,               # 速度百分比
    high_follow: bool,        # MIT 模式
    send_mode: bool,          # 本周期是否同时发送 MotionCtrl_2
) -> None:
    if send_mode:
        mit_flag = 0xAD if high_follow else 0x00
        piper.MotionCtrl_2(0x01, 0x01, speed, mit_flag)
    piper.JointCtrl(*joints)
    if gripper is not None:
        piper.GripperCtrl(
            clamp(abs(gripper), *GRIPPER_LIMIT_UM), 1000, 0x03, 0
        )
```

三个 CAN 命令的发送顺序：
1. **MotionCtrl_2**（条件发送）：设置控制模式和使能状态。不每周期发送，由 `--mode-command-period` 控制发送间隔（例如每 1.0 秒发送一次），因为控制模式不需要每帧重复设置。
2. **JointCtrl**（每周期必发）：发送 6 个关节的目标位置。`*joints` 将列表解包为 6 个位置参数。
3. **GripperCtrl**（条件发送）：发送夹爪的目标位置和速度/力矩参数。由 `--gripper-hz` 控制发送频率（默认 5 Hz），因为夹爪不需要像关节那样高的控制频率。

`GripperCtrl` 的参数：
- `clamp(abs(gripper), 0, 68000)`: 夹爪目标位置 clamp 到安全范围
- `1000`: 夹爪速度参数
- `0x03`: 夹爪控制模式
- `0`: 力矩参数

### 7.3.8 主循环逐行解析

让我们逐行分析 `main()` 函数中的主循环。这是整个 PC 中继的核心，它在一个无限循环中执行"读主臂 → 变换 → 限幅 → 写从臂"的流水线。

```python
while not stop:
    now = time.monotonic()
```

`time.monotonic()` 返回单调递增的时钟（不受系统时间调整影响），用于计算每个循环迭代的耗时。

```python
    if args.duration > 0 and now - start_time >= args.duration:
        break
```

`--duration` 参数可以设置脚本自动退出（单位秒）。设为 0 表示无限运行直到 Ctrl-C。

```python
    if args.pause_file and os.path.exists(args.pause_file):
        time.sleep(period)
        continue
```

**暂停机制的核心**：如果 `--pause-file` 指定了一个文件路径，并且该文件在当前时刻存在，则跳过本周期——不读取主臂位置，不发送从臂命令。脚本 sleep `period` 秒后直接进入下一个循环。

注意 `sleep(period)` 的设计：它保证即使在暂停期间，循环频率也大致保持在 `hz` 的水平，不会因为 `continue` 而进入忙等待（busy loop）消耗 CPU。

暂停文件的**创建和删除**不由此脚本负责——它只是被动检查。文件的创建和删除由 `lerobot-record` 的 `run_reset_command_if_configured()` 函数管理（见 7.4.4 节）。

```python
    leader_sample = read_leader(leader, args.source)
    follower_sample = feedback_sample(follower)
```

读取主臂和从臂的当前状态。注意：主臂的数据来源由 `--source` 决定，但从臂总是使用 `feedback_sample()`（读取实际位置）。这是因为速率限制需要基于从臂的**真实当前位置**，而不是它的控制目标位置（见 7.2.2 决策 3）。

```python
    if leader_sample.joint_hz <= 0 and leader_sample.joint_timestamp <= 0:
        ... # 打印警告，skip 本周期
        time.sleep(period)
        continue
```

如果主臂的 CAN 数据流无效（频率和时间戳均为 0），跳过发送。这处理了主臂未上电、CAN 线松动等异常情况。

```python
    raw_target = transform_joints(leader_sample.joints, signs, offsets_mdeg)
    bounded_target = bounded_joints(raw_target)
    limited_target = rate_limit(follower_sample.joints, bounded_target, max_step_mdeg)
```

三步流水线（顺序很重要）：
1. **transform_joints**：应用方向符号和零点偏置
2. **bounded_joints**：clamp 到关节物理限位内
3. **rate_limit**：基于从臂当前反馈位置，限制单步增量的幅度

> 顺序不可颠倒。如果先 rate_limit 再 bounded_joints，则 rate_limit 可能在越界值上计算（例如 target 为 200,000 mdeg，远超 limit 的 150,000 mdeg），导致 rate_limit 产生一个不合理的大步长。正确的顺序是先 clamp 到合法范围，再在合法范围内做增量限幅。

```python
    send_gripper = (
        args.include_gripper
        and (args.gripper_hz <= 0 or now - last_gripper_command >= 1.0 / args.gripper_hz)
    )
    gripper = leader_sample.gripper if send_gripper else None
    send_mode = (
        args.mode_command_period > 0
        and now - last_mode_command >= args.mode_command_period
    )
```

夹爪命令和模式命令的节流逻辑：
- 夹爪只有在 `--include-gripper` 启用且距上次发送超过 `1/gripper_hz` 秒时才发送。如果 `--gripper-hz` 设为 0，则每个周期都发。
- `MotionCtrl_2` 模式命令只有在距上次发送超过 `--mode-command-period` 秒时才附带发送。

```python
    if args.execute:
        send_follower(follower, limited_target, gripper, args.speed, args.high_follow, send_mode)
        if send_mode:
            last_mode_command = now
        if send_gripper:
            last_gripper_command = now
```

只有 `--execute` 启用时，才真正向从臂发送命令。如果没有 `--execute`，脚本仍然执行读取、变换、限速的全部计算逻辑，只是不发送——这是 dry-run 模式。

```python
    elapsed = time.monotonic() - now
    time.sleep(max(0.0, period - elapsed))
```

频率控制：计算本周期已经消耗的时间，sleep 剩余时间以保持 `--hz` 的标称频率。`max(0.0, ...)` 防止在循环超时（elapsed > period）时 sleep 负数。

这个简单的前馈 sleep 方案（而非 PID 调节的变步长控制）在循环耗时稳定的情况下可以达到很好的频率精度。如果 `--hz` 设得太高（例如 100 Hz，period = 0.01s），但单次循环耗时超过 0.01s，则实际频率会低于标称值——`elapsed > period` 导致 sleep 0 秒，循环以其最快速度运行。

### 7.3.9 退出时的安全处理

```python
finally:
    if args.execute:
        print("Stopping follower command stream; sending standby mode.")
        try:
            follower.MotionCtrl_2(0x00, 0x01, 0)
        except Exception as exc:
            print(f"Failed to send standby command: {exc}", file=sys.stderr)
    leader.DisconnectPort()
    follower.DisconnectPort()
```

`finally` 块保证无论脚本如何退出（Ctrl-C、异常、正常结束），以下清理动作都会执行：

1. **发送 standby 命令**：`MotionCtrl_2(0x00, 0x01, 0)` — 模式=`0x00`（standby/失能），使能=`0x01`（使能），速度=`0`。这会让从臂的电机退出关节位置控制模式，进入一种"无力矩但保持位置"的 standby 状态。如果不发送此命令，从臂可能在断开 CAN 连接后仍然保持最后一次接收到的力矩命令，导致关节在重力作用下缓慢下坠。
2. **断开 CAN 连接**：`DisconnectPort()` 释放 piper_sdk 的后台线程和 socketcan 资源。

### 7.3.10 命令行参数完整参考

```python
parser.add_argument("--leader", default="can0",
    help="CAN interface connected to the leader arm")
parser.add_argument("--follower", default="can1",
    help="CAN interface connected to the follower arm")
parser.add_argument("--source", choices=["auto", "feedback", "control"], default="auto",
    help="Leader joint data source")
parser.add_argument("--hz", type=float, default=10.0,
    help="Relay loop frequency")
parser.add_argument("--speed", type=int, default=10,
    help="Follower speed percentage sent to MotionCtrl_2")
parser.add_argument("--max-step-deg", type=float, default=1.0,
    help="Max follower joint step per cycle (degrees)")
parser.add_argument("--mode-command-period", type=float, default=1.0,
    help="Seconds between repeated MotionCtrl_2 commands; 0 disables")
parser.add_argument("--gripper-hz", type=float, default=5.0,
    help="Max gripper command frequency")
parser.add_argument("--print-period", type=float, default=1.0,
    help="Seconds between status prints; 0 disables")
parser.add_argument("--pause-file", default="",
    help="If this file exists, pause follower command sending")
parser.add_argument("--duration", type=float, default=0.0,
    help="Seconds to run; 0 means until Ctrl-C")
parser.add_argument("--signs", default="1,1,1,1,1,1",
    help="Per-joint sign mapping, comma-separated")
parser.add_argument("--offset-deg", default="0,0,0,0,0,0",
    help="Per-joint offset in degrees, comma-separated")
parser.add_argument("--include-gripper", action="store_true",
    help="Relay gripper target too")
parser.add_argument("--high-follow", action="store_true",
    help="Use MotionCtrl_2 is_mit_mode=0xAD")
parser.add_argument("--skip-follower-config", action="store_true",
    help="Do not send initial mode/enable commands")
parser.add_argument("--execute", action="store_true",
    help="Actually send commands to the follower")
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--leader` | str | `can0` | 主臂连接的 CAN 接口名 |
| `--follower` | str | `can1` | 从臂连接的 CAN 接口名 |
| `--source` | choice | `auto` | 主臂数据来源：`auto`（优先 control）/ `feedback`/ `control` |
| `--hz` | float | `10.0` | 中继循环频率。实际频率受限于 USB-CAN 带宽和 Python 耗时 |
| `--speed` | int | `10` | 从臂速度百分比 (0-100)。通过 MotionCtrl_2 发送 |
| `--max-step-deg` | float | `1.0` | 单步最大角度变化量（度）。核心安全参数 |
| `--mode-command-period` | float | `1.0` | MotionCtrl_2 命令的重复发送间隔（秒）。0 表示只发送一次 |
| `--gripper-hz` | float | `5.0` | 夹爪命令的最大发送频率。0 表示每周期都发送 |
| `--print-period` | float | `1.0` | 控制台状态打印间隔（秒）。0 表示不打印 |
| `--pause-file` | str | `""` | 暂停文件的路径。空字符串表示不使用暂停机制 |
| `--duration` | float | `0.0` | 运行时长（秒）。0 表示无限运行直到 Ctrl-C |
| `--signs` | str | `"1,1,1,1,1,1"` | 每个关节的方向符号，逗号分隔。`1`=同向, `-1`=反向 |
| `--offset-deg` | str | `"0,0,0,0,0,0"` | 每个关节的角度偏置（度），逗号分隔 |
| `--include-gripper` | flag | False | 是否同时中继夹爪动作 |
| `--high-follow` | flag | False | 是否启用 MIT 高跟随模式 (0xAD) |
| `--skip-follower-config` | flag | False | 是否跳过从臂初始配置（不发送模式命令和使能） |
| `--execute` | flag | False | **是否真正发送命令**。不传则为 dry-run 模式 |

---

## 7.4 record_with_pc_relay.sh 逐段解析

`record_with_pc_relay.sh` 是 PC 中继方案的操作入口。它是一个 bash 脚本，负责：

1. 加载配置（相机序列号、CAN 口、采集参数）
2. 以后台进程启动中继脚本
3. 以前台进程启动 lerobot-record
4. 在录制结束时清理中继进程

### 7.4.1 目录和 Conda 环境配置

```bash
PIPER_ROOT="${PIPER_ROOT:-/home/liyuqi/Documents/Piper}"
LEROBOT_DIR="${LEROBOT_DIR:-${PIPER_ROOT}/lerobot_piper-piper}"
CONDA_ENV="${CONDA_ENV:-lerobot}"
```

所有路径都通过 bash 参数扩展提供默认值，可以使用环境变量覆盖：

```bash
PIPER_ROOT=/workspace CONDA_ENV=my_env ./record_with_pc_relay.sh
```

脚本使用 `${VAR:-default}` 语法（带冒号），这意味着只有当变量**未设置或为空**时才使用默认值。

### 7.4.2 相机配置加载

```bash
if [[ -f "${LEROBOT_DIR}/camera_config_realsense.env" ]]; then
  source "${LEROBOT_DIR}/camera_config_realsense.env"
fi
```

`camera_config_realsense.env` 文件的内容示例：

```bash
export WRIST_REALSENSE_SERIAL="238222076529"
export GLOBAL_REALSENSE_SERIAL="142122071524"
export ROBOT_CAMERAS='{
  wrist: {type: intelrealsense, serial_number_or_name: "WRIST_SERIAL", width: 640, height: 480, fps: 15},
  global: {type: intelrealsense, serial_number_or_name: "GLOBAL_SERIAL", width: 640, height: 480, fps: 15}
}'
```

> 注意：`ROBOT_CAMERAS` 中使用了占位符 `WRIST_SERIAL` 和 `GLOBAL_SERIAL`。这是设计上的考量——相机配置模板保持通用性，实际序列号在脚本中通过字符串替换注入。

序列号替换逻辑：

```bash
if [[ -n "${WRIST_REALSENSE_SERIAL:-}" ]]; then
  ROBOT_CAMERAS="${ROBOT_CAMERAS//WRIST_SERIAL/${WRIST_REALSENSE_SERIAL}}"
fi
if [[ -n "${GLOBAL_REALSENSE_SERIAL:-}" ]]; then
  ROBOT_CAMERAS="${ROBOT_CAMERAS//GLOBAL_SERIAL/${GLOBAL_REALSENSE_SERIAL}}"
fi
```

`${ROBOT_CAMERAS//WRIST_SERIAL/...}` 是 bash 的全局替换语法。替换后，配置变为：

```yaml
wrist: {type: intelrealsense, serial_number_or_name: "238222076529", width: 640, height: 480, fps: 15}
global: {type: intelrealsense, serial_number_or_name: "142122071524", width: 640, height: 480, fps: 15}
```

如果替换后仍然存在占位符（说明相机配置文件没有正确提供序列号），脚本会检测并报错退出：

```bash
if [[ "${ROBOT_CAMERAS}" == *"WRIST_SERIAL"* || "${ROBOT_CAMERAS}" == *"GLOBAL_SERIAL"* ]]; then
  echo "[record] ERROR: ROBOT_CAMERAS still contains placeholder serials:" >&2
  exit 1
fi
```

### 7.4.3 FPS 对齐

```bash
ROBOT_CAMERAS="$(ROBOT_CAMERAS="${ROBOT_CAMERAS}" DATASET_FPS="${DATASET_FPS}" python - <<'PY'
import json, os, yaml

camera_cfg = yaml.safe_load(os.environ["ROBOT_CAMERAS"])
dataset_fps = int(os.environ["DATASET_FPS"])
if isinstance(camera_cfg, dict):
    for cfg in camera_cfg.values():
        if isinstance(cfg, dict) and "fps" in cfg:
            cfg["fps"] = dataset_fps
print(json.dumps(camera_cfg))
PY
)"
```

这段内联 Python 脚本将相机配置中的所有 `fps` 字段强制对齐到 `DATASET_FPS`（默认 30）。为什么要这样做？

在 LeRobot 的数据集中，所有相机必须以**相同的帧率**录制。如果腕部相机以 15 fps 运行而全局相机以 30 fps 运行，数据对齐会出现问题（某一帧的关节状态对应哪个相机图像？）。通过将所有相机 fps 统一到 `DATASET_FPS`，保证每个时间戳下所有数据（关节状态 + 所有相机帧）是一一对应的。

随后还有一个验证步骤：

```python
mismatches = {
    name: cfg.get("fps")
    for name, cfg in camera_cfg.items()
    if isinstance(cfg, dict) and cfg.get("fps") is not None and int(cfg.get("fps")) != dataset_fps
}
if mismatches:
    raise SystemExit(f"ERROR: camera fps must match DATASET_FPS. mismatches={mismatches}")
```

这确保了（在替换之后）没有任何相机 fps 与目标 fps 不一致。

### 7.4.4 数据集路径处理

```bash
DATASET_BASE_DIR="${DATASET_BASE_DIR:-${DATASET_ROOT:-${PIPER_ROOT}/datasets}}"
DATASET_NAME="${DATASET_NAME:-piper_pc_relay_smoke}"
HF_USER="${HF_USER:-${HF_USER_DEFAULT:-local}}"
DATASET_REPO_ID="${DATASET_REPO_ID:-${HF_USER}/${DATASET_NAME}}"
DATASET_OUTPUT_ROOT="${DATASET_OUTPUT_ROOT:-${DATASET_BASE_DIR}/${DATASET_REPO_ID}}"

if [[ -e "${DATASET_OUTPUT_ROOT}" && "${AUTO_SUFFIX_DATASET_ROOT}" == "1" ]]; then
  DATASET_OUTPUT_ROOT="${DATASET_OUTPUT_ROOT}_$(date +%Y%m%d_%H%M%S)"
fi
```

数据集默认路径为 `${PIPER_ROOT}/datasets/${HF_USER}/${DATASET_NAME}`。如果路径已存在且 `AUTO_SUFFIX_DATASET_ROOT=1`（默认启用），自动追加时间戳后缀（如 `_20260622_143025`），防止意外覆盖已有数据。

如果你要手动指定数据集目录，设置 `DATASET_OUTPUT_ROOT` 即可：

```bash
DATASET_OUTPUT_ROOT=/data/my_experiment ./record_with_pc_relay.sh
```

### 7.4.5 后台启动中继进程

```bash
RELAY_PAUSE_FILE="${RELAY_PAUSE_FILE:-/tmp/piper_pc_relay.pause}"

relay_cmd=(
  "${PIPER_ROOT}/piper_sdk/piper/bin/python"
  "${LEROBOT_DIR}/scripts/piper_pc_relay_teleop.py"
  --leader "${LEADER_CAN}"
  --follower "${FOLLOWER_CAN}"
  --source "${RELAY_SOURCE}"
  --hz "${RELAY_HZ}"
  --speed "${RELAY_SPEED}"
  --max-step-deg "${RELAY_MAX_STEP_DEG}"
  --mode-command-period "${RELAY_MODE_COMMAND_PERIOD}"
  --gripper-hz "${RELAY_GRIPPER_HZ}"
  --print-period "${RELAY_PRINT_PERIOD}"
  --pause-file "${RELAY_PAUSE_FILE}"
  --execute
)
```

关键默认值（在脚本顶部定义）：

| 变量 | 默认值 | 含义 |
|------|--------|------|
| `RELAY_HZ` | `20` | 中继频率 20 Hz |
| `RELAY_SPEED` | `100` | 从臂速度百分比 100% |
| `RELAY_MAX_STEP_DEG` | `10` | 单步最大 10 度（相当于 200 度/秒） |
| `RELAY_SOURCE` | `control` | 使用控制位置数据 |
| `RELAY_INCLUDE_GRIPPER` | `1` | 中继夹爪 |
| `RELAY_HIGH_FOLLOW` | `1` | 启用 MIT 高跟随模式 |
| `RELAY_MODE_COMMAND_PERIOD` | `1.0` | 每秒重发一次 MotionCtrl_2 |
| `RELAY_GRIPPER_HZ` | `5` | 夹爪命令 5 Hz |
| `RELAY_PRINT_PERIOD` | `0` | 不打印状态（设为 1 或更大可以开启调试打印） |

注意：`record_with_pc_relay.sh` 中使用的是**项目自带的 Python 解释器**（`${PIPER_ROOT}/piper_sdk/piper/bin/python`），而不是系统 Python 或 Conda 环境中的 Python。这是因为 `piper_sdk` 是一个编译好的 C++ 扩展包，对 Python 版本和 ABI 有严格依赖，使用自带的解释器可以避免版本冲突。

```bash
trap cleanup EXIT INT TERM
rm -f "${RELAY_PAUSE_FILE}" 2>/dev/null || true

echo "[relay] ${relay_cmd[*]}"
"${relay_cmd[@]}" &
RELAY_PID="$!"
sleep 2
```

`trap cleanup EXIT INT TERM` 注册了退出处理器，确保脚本被 Ctrl-C 或 kill 时，`cleanup` 函数一定会执行（删除暂停文件、kill 中继进程）。

`sleep 2` 给中继进程 2 秒的启动时间（piper_sdk 需要建立 CAN 连接和启动后台线程），然后才启动 lerobot-record。

### 7.4.6 复位协调机制

这是 PC 中继方案与 lerobot-record 深度集成的关键部分。

```bash
if [[ "${AUTO_RESET_ARMS}" == "1" || "${AUTO_RESET_ARMS}" == "true" ]]; then
  reset_ports=("${FOLLOWER_CAN}")
  master_home_args=()
  if [[ "${AUTO_RESET_LEADER}" == "1" || "${AUTO_RESET_LEADER}" == "true" ]]; then
    reset_ports=("${LEADER_CAN}" "${FOLLOWER_CAN}")
    master_home_args=(--master-home-ports "${LEADER_CAN}")
  fi
  export LEROBOT_RESET_COMMAND="${PIPER_ROOT}/piper_sdk/piper/bin/python ${LEROBOT_DIR}/scripts/piper_reset_to_initial.py --ports ${reset_ports[*]} ${master_home_args[*]} --target-deg ${RESET_TARGET_DEG} --speed ${RESET_SPEED} --hz ${RESET_HZ} --duration ${RESET_DURATION} --high-follow"
  export LEROBOT_RELAY_PAUSE_FILE="${RELAY_PAUSE_FILE}"
fi
```

两个关键的环境变量被导出：
- `LEROBOT_RESET_COMMAND`：复位命令的完整 shell 命令字符串。在每个 episode 结束后，lerobot-record 会执行此命令。
- `LEROBOT_RELAY_PAUSE_FILE`：暂停文件的路径。lerobot-record 在执行复位命令前 touch 此文件，执行后删除。

lerobot-record 内部的处理逻辑（来自 `src/lerobot/record.py`）：

```python
def run_reset_command_if_configured() -> None:
    command = os.environ.get("LEROBOT_RESET_COMMAND")
    if not command:
        return
    pause_file = os.environ.get("LEROBOT_RELAY_PAUSE_FILE")
    try:
        if pause_file:
            Path(pause_file).parent.mkdir(parents=True, exist_ok=True)
            Path(pause_file).touch()          # ① 创建暂停文件
        subprocess.run(shlex.split(command), check=False)  # ② 执行复位
    finally:
        if pause_file:
            Path(pause_file).unlink(missing_ok=True)  # ③ 删除暂停文件
```

这个三段式协调保证：
1. **创建暂停文件** → 中继进程在下一次循环中检测到文件存在 → 停止向从臂发送命令
2. **执行复位** → `piper_reset_to_initial.py` 将双臂移动到初始位置。此时从臂不受中继进程控制（因为暂停已生效），而是由复位脚本直接控制
3. **删除暂停文件** → 中继进程检测到文件消失 → 恢复向从臂发送命令

如果不使用 pause-file 机制，复位脚本和中继脚本会**同时**向从臂发送 CAN 命令——两个进程竞争同一个 CAN 总线，导致从臂的行为不可预测。

### 7.4.7 启动 lerobot-record

```bash
lerobot-record \
  --robot.type=piper_follower \
  --robot.port="${FOLLOWER_CAN}" \
  --robot.cameras="${ROBOT_CAMERAS}" \
  --robot.id=pc_relay_follower \
  --dataset.root="${DATASET_OUTPUT_ROOT}" \
  --dataset.repo_id="${DATASET_REPO_ID}" \
  --dataset.fps="${DATASET_FPS}" \
  --dataset.num_episodes="${NUM_EPISODES}" \
  --dataset.episode_time_s="${EPISODE_TIME_S}" \
  --dataset.reset_time_s="${RESET_TIME_S}" \
  --dataset.single_task="${TASK}" \
  --dataset.push_to_hub="${PUSH_TO_HUB}" \
  --display_data="${DISPLAY_DATA}" \
  --play_sounds="${PLAY_SOUNDS}" \
  --manual_step="${MANUAL_STEP}"
```

注意这里**没有 `--teleoperator.type` 参数**。在标准的 LeRobot 录制备置中，通常会指定一个 Teleoperator 类型（如 `--teleoperator.type=piper_leader`），让 lerobot-record 同时管理主臂和从臂。但在 PC 中继方案中：

- 主臂的数据通路由独立的 `piper_pc_relay_teleop.py` 进程管理
- lerobot-record 只需要连接从臂（`--robot.type=piper_follower --robot.port=can1`）来采集观测数据
- 数据集中的 `action` 字段直接使用从臂的当前关节状态（因为从臂正在跟随主臂运动，它的关节状态反映了主臂的动作）

默认采集参数：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `NUM_EPISODES` | `5` | 采集 5 个 episode |
| `EPISODE_TIME_S` | `20` | 每个 episode 录制 20 秒 |
| `RESET_TIME_S` | `10` | 每个 episode 之间 10 秒复位时间 |
| `DATASET_FPS` | `30` | 数据集帧率 30 fps |
| `MANUAL_STEP` | `true` | 手动步进模式（按回车进入下一 episode） |

### 7.4.8 清理函数

```bash
cleanup() {
  rm -f "${RELAY_PAUSE_FILE}" 2>/dev/null || true
  if [[ -n "${RELAY_PID:-}" ]] && kill -0 "${RELAY_PID}" 2>/dev/null; then
    kill "${RELAY_PID}" 2>/dev/null || true
    wait "${RELAY_PID}" 2>/dev/null || true
  fi
}
```

清理函数被 `trap cleanup EXIT INT TERM` 绑定到脚本退出事件，确保无论脚本如何退出：
1. 暂停文件被删除（防止下一次运行时中继误认为暂停）
2. 中继进程被终止（`kill` + `wait` 确保进程完全退出）
3. 中继进程的 `finally` 块会自动发送 standby 命令给从臂

---

## 7.5 PC 中继的完整工作流时序

以下时序图展示了一次完整的录制循环（从一个 episode 开始到下一个 episode 开始）中，人、主臂、中继进程、从臂和 lerobot-record 之间的交互。

```
时间 ──────────────────────────────────────────────────────────────────────►

人         : [手拖动主臂，执行任务]................[听到"Reset"，松手]........[等待复位完成]....
            :                                                  :
主臂       : [编码器上报 0x2A5-0x2A7 @1kHz]......[继续上报]....[被复位脚本驱动回零位]...
(can0)     :                                                  :
            :                                                  :
中继进程    : [读取→变换→限速→JointCtrl @20Hz].[检测到 pause_file].[sleep等待]...[检测 pause_file 删除→恢复发送]
(独立进程)  :                                                  :
            :                                                  :
从臂       : [跟随主臂运动，执行任务]..........[停在上一个位置].[被复位脚本驱动回零位]...[恢复跟随主臂]
(can1)     :                                                  :
            :                                                  :
lerobot-   : [录制 observation + action @30fps]..[episode 结束]................[下一 episode 开始]
record     :                                    :              :
(前台进程)  :                                    :              :
            :                                    :              :
            :                          run_reset_command_if_configured():
            :                          ① touch /tmp/piper_pc_relay.pause
            :                          ② piper_reset_to_initial.py --ports can1
            :                          ③ rm /tmp/piper_pc_relay.pause
            :                                    :              :
            :                          [reset_time_s 倒计时或手动按 Enter]
            :                          "Reset the environment"
            :

Episode N 录制 (20s)              Reset 阶段 (10s)         Episode N+1 录制 (20s)
├────────────────────────────────┼──────────────────────┼─────────────────────────►
```

### 关键时序说明

1. **录制阶段**（episode N）：人拖拽主臂执行任务，中继进程以 20 Hz 将主臂位置中继到从臂（经过变换和限速），从臂跟随主臂实时运动。lerobot-record 以 30 fps 记录从臂的关节状态和相机图像。此时 pause_file 不存在，中继进程正常运行。

2. **episode 结束时刻**：lerobot-record 的录制计时器到达 `episode_time_s`（默认 20 秒）。系统播放 "Reset the environment" 语音提示。`run_reset_command_if_configured()` 被调用。

3. **暂停中继**（关键步骤）：函数首先 `touch /tmp/piper_pc_relay.pause`。在中继进程的下一个循环周期内（最多 `1/20 = 0.05 秒`），`os.path.exists(pause_file)` 返回 True，中继进程停止向从臂发送命令。**从臂停在最后接收到的位置，不再跟随主臂运动。**

4. **执行复位**：`piper_reset_to_initial.py` 脚本运行，连接 `can1`（从臂），将关节移动到目标零点位（默认 `0,0,0,0,0,0` 度）。如果 `AUTO_RESET_LEADER=1`，也同时复位主臂。

5. **恢复中继**：复位脚本执行完毕后，`finally` 块删除 pause_file。中继进程在下一个循环周期检测到文件消失，恢复向从臂发送命令。人重新开始拖拽主臂，进入下一 episode。

这个协调机制的优雅之处在于：
- 不需要进程间通信（IPC），文件系统充当信号量
- 中继进程不需要知道"为什么"暂停——它只检查文件存在性
- 暂停和恢复的延迟上限为 `1/RELAY_HZ` 秒（默认 0.05 秒），足够快
- 如果复位脚本执行失败（返回非零退出码），`finally` 块仍然会删除 pause_file，中继不会永久卡在暂停状态

---

## 7.6 安全设计

PC 中继方案从多个层面确保了操作安全。理解这些安全机制不仅有助于避免事故，也有助于在遇到异常行为时快速定位问题。

### 7.6.1 速率限制（增量限幅）

这是最核心的安全层。`rate_limit()` 函数基于从臂**当前实际反馈位置**，对每一步的目标增量进行 clamp：

```
output[i] = clamp(target[i], current[i] - max_step, current[i] + max_step)
```

关键性质：
- **增量限幅，不是绝对位置限幅**：速率限制不关心目标位置的绝对值是多少，只关心"这一步的变化量"。即使主臂报告了一个跳跃了 90 度的非法值，从臂也只会移动 `max_step_deg` 度（默认 1 度，在采集脚本中为 10 度）。
- **基于反馈位置，不自累积误差**：每一周期的基线是 `feedback_sample()` 读取的从臂实际位置。如果上一周期从臂因为某种原因没有到达目标（例如遇到障碍物），本周期会从"它实际在的位置"开始计算增量，不会在"未到达的旧目标"上叠加新的增量。
- **独立于硬件保护**：速率限制是纯软件层面的保护，与电机驱动器的速度环/电流环保护正交。两者同时生效，形成纵深防御。

### 7.6.2 关节限位（bounded_joints）

在速率限制之前，`bounded_joints()` 将变换后的目标位置 clamp 到 `JOINT_LIMITS_MDEG` 定义的物理范围内。这确保了即使 `--signs` 和 `--offset-deg` 的配置错误导致变换后的目标超出了物理极限，从臂也不会收到越界命令。

### 7.6.3 暂停文件机制

暂停文件提供了一种**带外（out-of-band）**的紧急停止能力。即使中继进程正常运行，只要在文件系统中创建 pause_file（`touch /tmp/piper_pc_relay.pause`），中继就会在 0.05 秒内停止发送命令。你可以通过以下方式手动触发暂停：

```bash
# 手动暂停中继
touch /tmp/piper_pc_relay.pause

# 手动恢复中继
rm /tmp/piper_pc_relay.pause
```

这在以下场景中非常有用：
- 从臂出现异常抖动，需要立即停止
- 工作区有障碍物，需要先移开再继续操作
- 调试关节变换参数，需要观察从臂停在某位置的行为

### 7.6.4 进程隔离

中继进程和录制进程是独立的操作系统进程。这种隔离带来了以下安全属性：

| 故障场景 | 影响 | 安全性 |
|----------|------|--------|
| lerobot-record 崩溃（磁盘满） | 当前 episode 数据丢失，但中继继续工作 | 从臂仍然受控 |
| 中继进程崩溃（Python 异常） | 从臂停在最后位置，lerobot-record 继续运行（见不到新数据） | 从臂不会意外运动 |
| 两个进程同时崩溃 | 从臂停在最后位置（USB-CAN 保持最后输出） | 从臂静止 |
| 操作者按 Ctrl-C | bash trap 触发 → kill 中继 → 中继 finally 发送 standby → 从臂进入无力矩保持 | 安全停止 |

### 7.6.5 Dry-run 模式

中继脚本的默认行为是**不发送任何命令**。`--execute` 必须显式传入。这意味着：

- 第一次启动时，你可以在 dry-run 模式下观察控制台输出，确认关节变换和速率限制的逻辑正确
- 如果你忘记了正确的参数组合，dry-run 模式给了你试错的空间
- 在演示或培训场景中，可以先 dry-run 一遍，确认所有参数无误后再加 `--execute`

### 7.6.6 退出时的 Standby 命令

中继进程退出时（无论是正常退出还是被信号杀死），`finally` 块会向从臂发送 `MotionCtrl_2(0x00, 0x01, 0)`——进入 standby 模式但不失能。这比直接断开 CAN 连接更安全：

- 如果直接断开 CAN，从臂可能保持上一次的力矩命令，在重力作用下关节缓慢移动
- Standby 模式让驱动器切换到位置保持状态（无力矩输出但保持当前位置），关节不会下坠
- 如果你需要完全失能力矩（例如检修），可以在 standby 之后再手动发送失能命令

---

## 7.7 常见问题和调试

### 7.7.1 中继不工作：从臂完全不动

**可能原因及排查步骤**：

**1. 检查 CAN 口是否 UP**

```bash
ip link show can0
ip link show can1
```

输出中应当看到 `state UP`。如果状态是 `DOWN`：

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

> PiPER 的 CAN 波特率是 1 Mbps（1,000,000 bps）。注意不是 500k 或 125k。

**2. 检查是否传了 `--execute`**

在没有 `--execute` 时，脚本是 dry-run 模式。检查控制台输出：第一行会打印 `Mode: DRY-RUN: no follower commands will be sent` 或 `Mode: EXECUTE: follower will move`。

**3. 检查主臂是否上电且 CAN 线连接**

在 dry-run 模式下观察控制台输出。如果看到 `No leader ... joint stream on can0`，说明主臂的 CAN 数据没有到达 PC。检查主臂电源和 USB-CAN 适配器的连接。

**4. 检查 USB-CAN 适配器的指示灯**

- 绿灯常亮：适配器已上电，socketcan 已绑定
- 绿灯闪烁：CAN 总线上有数据帧传输
- 红灯：总线错误（可能是终端电阻缺失或波特率不匹配）

若红灯常亮，请检查 CAN 总线的 120 欧终端电阻是否正确安装。

### 7.7.2 从臂抖动

**可能原因和解决方案**：

**1. `--max-step-deg` 过大**

如果 `max-step-deg` 设置过高（例如 50 度/步），从臂会在目标位置附近来回超调。降低此值：

```bash
# 在 record_with_pc_relay.sh 调用时覆盖
RELAY_MAX_STEP_DEG=3 ./record_with_pc_relay.sh
```

推荐从 3 度/步开始测试，逐步增大直到找到既不抖动又能跟上的最佳值。

**2. `--hz` 过高**

如果中继频率设置过高（例如 100 Hz），USB-CAN 适配器可能无法以这个频率稳定发送命令。尝试降低：

```bash
RELAY_HZ=10 ./record_with_pc_relay.sh
```

**3. 主臂传感器噪声**

尝试切换 `--source`：

```bash
# 如果当前用的是 feedback，尝试 control
RELAY_SOURCE=control ./record_with_pc_relay.sh

# 如果当前用的是 control，尝试 feedback
RELAY_SOURCE=feedback ./record_with_pc_relay.sh
```

通常 `control` 源的数据比 `feedback` 源更平滑（经过了驱动器内部滤波）。

**4. CAN 总线电磁干扰**

检查 CAN 线是否靠近大功率设备（电机驱动器、开关电源）。尝试重新走线，使 CAN 线远离干扰源。确保 CAN 线使用双绞线，且屏蔽层正确接地。

### 7.7.3 主臂和从臂运动方向不一致

**问题**：人把主臂往某个方向拖动，但从臂往相反方向运动。

**原因**：主臂和从臂的某个（或某几个）关节安装方向相反。

**解决方案**：修改 `--signs` 参数。例如关节 2 方向反转：

```python
# 直接在命令行测试（dry-run 模式）
python scripts/piper_pc_relay_teleop.py --signs "1,-1,1,1,1,1" --print-period 1

# 确认正确后，在 record_with_pc_relay.sh 中设置环境变量（当前脚本未暴露此参数，
# 需要手动修改 relay_cmd 数组或在脚本中添加环境变量支持）
```

如何确定哪个关节需要反转：
1. 将主臂的**单个关节**从零位向正方向缓慢移动（例如只动关节 1，其他关节保持不动）
2. 观察从臂对应关节的运动方向
3. 如果方向相反，将该关节的 sign 改为 `-1`
4. 重复以上步骤，逐个关节验证

### 7.7.4 从臂零点与主臂不对齐

**问题**：主臂和从臂在相同姿态下，从臂的关节角度有明显偏置。

**解决方案**：使用 `--offset-deg` 参数：

```python
# 例如关节 3 需要偏移 -15 度
python scripts/piper_pc_relay_teleop.py --offset-deg "0,0,-15,0,0,0" --print-period 1
```

**如何确定偏置值**：

1. 将主臂手动拖到零位姿态（或某个已知姿态）
2. 读取从臂的当前关节角度（dry-run 模式的控制台会打印 follower deg）
3. 计算差值：`offset = master_pos - follower_pos`
4. 将差值作为 `--offset-deg` 的值

### 7.7.5 频率跟不上（实际频率低于设定频率）

**现象**：控制台打印的 hz 信息显示实际中继频率显著低于 `--hz` 的设定值。

**原因**：
- USB-CAN 适配器的写入带宽不足（廉价适配器可能无法做高频写入）
- Python GIL 导致 piper_sdk 的后台线程得不到足够的 CPU 时间
- 系统负载过高（其他进程占用 CPU）

**解决方案**：

1. **降低中继频率**：
   ```bash
   RELAY_HZ=10 ./record_with_pc_relay.sh
   ```
   10-20 Hz 对大多数操作任务来说已经足够。人类操作者的手部动作频率通常在 2-5 Hz 范围内。

2. **关闭不必要的后台进程**：特别是浏览器、IDE 等占用 CPU 和内存的应用。

3. **检查 USB-CAN 适配器**：使用 `candump` 检查总线负载：
   ```bash
   candump can0   # 观察主臂的上报频率
   ```
   应当看到约 1000 帧/秒（1 kHz × 3 个 CAN ID）。如果实际频率远低于此，可能是 USB-CAN 适配器硬件性能不足。

### 7.7.6 录制数据中从臂动作不连续

**现象**：录制的数据集中，从臂的关节轨迹出现跳变或不连续。

**可能原因**：

1. **复位阶段数据未过滤**：在 reset 阶段，从臂被 `piper_reset_to_initial.py` 驱动回零位，这个运动会被 lerobot-record 记录（如果 reset 阶段也在录制）。这是预期行为——reset 阶段的数据在训练时通常会被跳过。

2. **中继频率与录制频率不一致**：中继以 20 Hz 发送命令，录制以 30 fps 采样。在两次中继命令之间，从臂可能已经到达了目标位置，录制的相邻两帧关节值可能相同。这也是预期行为——训练时模型学到的是"动作可以重复"。

3. **CAN 通信丢帧**：使用 `candump` 检查从臂 CAN 总线是否有错误帧：
   ```bash
   cat /sys/class/net/can1/statistics/rx_errors
   cat /sys/class/net/can1/statistics/tx_errors
   ```
   如果错误计数器持续增长，说明 CAN 物理层有问题（终端电阻、线缆质量、接地等）。

### 7.7.7 夹爪不跟随主臂

**问题**：主臂夹爪的开合不影响从臂夹爪。

**可能原因**：
1. 中继脚本没有加 `--include-gripper` 参数。`record_with_pc_relay.sh` 默认设置了 `RELAY_INCLUDE_GRIPPER=1`，如果手动运行中继脚本需要显式添加。
2. 从臂夹爪的通信协议与主臂夹爪不同（主臂用 HTDW-5047 的夹爪通道，从臂用 AGILEX-S 的夹爪通道）。这是硬件层面的已知差异——PC 中继方案暂不支持主臂夹爪到从臂夹爪的转发。如需夹爪控制，请使用 lerobot-record 中配置的夹爪动作直接控制从臂夹爪。

---

## 7.8 操作检查清单

在每次开始数据采集前，建议按以下清单逐项确认：

```
□ 1. 硬件检查
   □ 主臂和从臂的电源线已连接并上电
   □ USB-CAN A 连接 can0（主臂），USB-CAN B 连接 can1（从臂）
   □ CAN 终端电阻已安装（120 欧）
   □ 相机 USB 线已连接（腕部和全局）
   □ 机械臂工作区内无障碍物

□ 2. CAN 总线检查
   □ ip link show can0 → state UP
   □ ip link show can1 → state UP
   □ can0 波特率 = 1,000,000
   □ can1 波特率 = 1,000,000

□ 3. 相机检查
   □ lerobot-find-cameras realsense 返回两个相机
   □ camera_config_realsense.env 中的序列号与实际相机匹配

□ 4. 中继脚本 dry-run 测试
   □ python scripts/piper_pc_relay_teleop.py --print-period 1
   □ 控制台打印 leader deg 数值随拖拽主臂实时变化
   □ 控制台打印 follower deg 数值与从臂实际姿态一致
   □ 主臂和从臂运动方向一致（如不一致，调整 --signs）
   □ 从臂零点与主臂对齐（如有偏置，调整 --offset-deg）

□ 5. 采集配置确认
   □ DATASET_NAME 已设置为有意义的数据集名称
   □ NUM_EPISODES 和 EPISODE_TIME_S 满足需求
   □ TASK 描述准确描述了当前采集的任务

□ 6. 开始采集
   □ ./record_with_pc_relay.sh
   □ 等待 "Press Enter to continue..." 提示，按 Enter 开始第一个 episode
   □ 操作过程中如遇异常，Ctrl-C 安全退出
```

---

## 7.9 本章小结

PC 中继是 PiPER 双臂系统中连接"人的操作"和"机器的执行"的核心桥梁。本章涵盖了以下关键知识点：

1. **为什么需要 PC 中继**：主臂和从臂在不同 CAN 总线上，硬件主从模式无法满足关节变换、速率限制和采集协调的需求。LeRobot 内置的 teleoperate 机制假设双臂同型号、同总线，不适用于 PiPER。

2. **架构设计**：中继进程和录制进程分离，通过文件系统（pause-file）协调。进程隔离提供了故障隔离，dry-run 模式提供了安全试错空间。

3. **数据处理流水线**：`读取主臂 → 关节变换（sign × val + offset）→ 限位（clamp to limits）→ 速率限制（clamp delta）→ 发送从臂`。三步顺序不可颠倒。

4. **录制编排**：`record_with_pc_relay.sh` 负责加载相机配置、启动后台中继、配置复位命令、启动前台录制、退出时清理。

5. **安全机制**：速率限制（增量限幅）、关节限位、暂停文件、进程隔离、dry-run 默认模式、退出 standby 命令——六层安全保护形成纵深防御。

6. **调试方法**：通过 dry-run 模式的 console 输出来验证关节变换和速率限制的正确性，通过 `candump` 和 `ip link show` 来诊断 CAN 通信问题。

掌握了 PC 中继方案后，你就掌握了 PiPER 双臂系统日常数据采集的完整工作流。下一章将进入**策略训练**——如何使用采集到的数据训练行为克隆模型。

---

> **源码参考**：
> - `scripts/piper_pc_relay_teleop.py` (309 行) — PC 中继引擎
> - `record_with_pc_relay.sh` (228 行) — 采集编排脚本
> - `src/lerobot/record.py` — `run_reset_command_if_configured()` 函数（第 146-159 行）
> - `scripts/piper_reset_to_initial.py` — 双臂复位脚本，由 `LEROBOT_RESET_COMMAND` 调用
