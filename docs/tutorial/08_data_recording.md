# 第八章 数据录制：从示教到数据集

本章全面讲解 PiPER 机械臂在 LeRobot 框架中的数据录制管线。从 `lerobot-record` CLI 入口点出发，逐层深入录制循环、时间控制、双相机配置、语音提示系统和数据集输出结构，最后覆盖数据验证（Replay）和常见故障排查。阅读本章后，你将能够独立完成从硬件接线到产出可训练数据集的全流程操作。

本章内容覆盖以下源文件：

| 文件 | 行数 | 角色 |
|------|------|------|
| `src/lerobot/record.py` | 596 | 录制主逻辑：CLI 入口、录制循环、手动步进、复位命令 |
| `src/lerobot/utils/utils.py` | 503 | 语音提示系统：TTS 引擎链、提示音生成、预生成播放 |
| `src/lerobot/cameras/realsense/camera_realsense.py` | 556 | RealSense 相机驱动：同步/异步读取、超时控制 |
| `src/lerobot/replay.py` | 134 | 数据回放：逐帧重放已录制的动作 |
| `camera_config_realsense.env` | 16 | 双相机环境变量配置模板 |

---

## 8.1 lerobot-record 完整流程

### 8.1.1 入口点与命令结构

`lerobot-record` 是一个通过 `pyproject.toml` 注册的 CLI 命令，其执行入口为 `src/lerobot/record.py:main()`。`main()` 函数极为简洁——它只做一件事：调用 `record()`。

```python
# src/lerobot/record.py 第 591-592 行
def main():
    record()
```

`record()` 函数被 `@parser.wrap()` 装饰器包裹，这个装饰器负责三件事：

1. **解析命令行参数**：将 `--robot.type=piper_follower` 这类 key=value 参数解析为嵌套的 dataclass 实例。
2. **自动生成帮助文档**：基于 dataclass 的字段类型和默认值自动构建 `--help` 输出。
3. **类型校验与转换**：将字符串参数自动转换为对应的 Python 类型（如 `"30"` → `30`）。

### 8.1.2 命令行参数结构：RecordConfig → DatasetRecordConfig

录制命令的参数体系由三层 dataclass 嵌套组成：

```
RecordConfig
├── robot: RobotConfig                    # 从臂配置
│   ├── type: str = "piper_follower"      #   机器人类型
│   ├── port: str = "can0"                #   CAN 接口
│   ├── cameras: dict                     #   相机配置
│   └── ...
├── dataset: DatasetRecordConfig          # 数据集配置
│   ├── repo_id: str                      #   数据集标识符（如 "local/my_task"）
│   ├── single_task: str                  #   任务描述
│   ├── root: str | Path | None           #   数据集存储根目录
│   ├── fps: int = 30                     #   录制帧率
│   ├── episode_time_s: int | float = 60  #   每个 episode 的录制时长（秒）
│   ├── reset_time_s: int | float = 60    #   每个 episode 之间的复位等待时间（秒）
│   ├── num_episodes: int = 50            #   录制 episode 数量
│   ├── video: bool = True                #   是否将图像编码为视频
│   ├── push_to_hub: bool = True          #   录制完成后是否上传到 Hugging Face Hub
│   ├── private: bool = False             #   上传时是否设为私有仓库
│   ├── tags: list[str] | None            #   Hub 标签
│   ├── num_image_writer_processes: int = 0   # 图像写入子进程数
│   ├── num_image_writer_threads_per_camera: int = 4  # 每相机的图像写入线程数
│   └── video_encoding_batch_size: int = 1       # 视频批量编码大小
├── teleop: TeleoperatorConfig | None     # 主臂配置（可选）
├── policy: PreTrainedConfig | None       # 策略配置（可选，替代遥操作）
├── display_data: bool = False            # 是否使用 rerun 实时可视化
├── play_sounds: bool = True              # 是否启用语音/提示音播报
├── manual_step: bool = False             # 是否启用手动步进模式
└── resume: bool = False                  # 是否在已有数据集上续录
```

**关键参数详解**：

| 参数路径 | 类型 | 默认值 | 说明 |
|----------|------|--------|------|
| `--robot.type` | str | — | 从臂类型，PiPER 项目始终用 `piper_follower` |
| `--robot.port` | str | — | 从臂 CAN 接口，如 `can0` 或 `can1` |
| `--robot.cameras` | dict | — | 相机配置字典，支持 opencv 和 intelrealsense 两种类型 |
| `--robot.id` | str | — | 从臂标识名（如 `black`），用于区分多台机械臂 |
| `--teleoperator.type` | str | — | 主臂类型，PiPER 项目用 `piper_leader` |
| `--teleoperator.port` | str | — | 主臂 CAN 接口 |
| `--dataset.repo_id` | str | — | 数据集路径标识，如 `local/piper_pick_cube` |
| `--dataset.num_episodes` | int | 50 | 要录制的 episode 总数 |
| `--dataset.fps` | int | 30 | 目标录制帧率（帧/秒） |
| `--dataset.episode_time_s` | int | 60 | 每个 episode 的录制时长 |
| `--dataset.single_task` | str | — | 任务文本描述（必填） |
| `--control.time_s` | — | — | 注意：实际参数名为 `--dataset.episode_time_s`，旧版文档可能引用 `control.time_s` |
| `--display_data` | bool | false | 启用 rerun 可视化面板 |
| `--manual_step` | bool | false | 启用手动步进：每 episode 前后等待 Enter 确认 |
| `--resume` | bool | false | 断点续录模式 |

### 8.1.3 完整执行流程

`record()` 函数的执行可划分为 8 个阶段：

```
                         ┌───────────────────────┐
                         │  1. 解析 CLI 参数       │
                         │  @parser.wrap() 装饰器   │
                         │  → RecordConfig 实例     │
                         └───────────┬───────────┘
                                     │
                         ┌───────────▼───────────┐
                         │  2. 创建 Robot 实例      │
                         │  make_robot_from_config  │
                         │  → PiperFollower 对象    │
                         └───────────┬───────────┘
                                     │
                         ┌───────────▼───────────┐
                         │  3. 创建 Teleoperator    │
                         │  (本项目中设为 None)      │
                         │  因为 PiperLeader 直接    │
                         │  CAN 中继到 Follower     │
                         └───────────┬───────────┘
                                     │
                         ┌───────────▼───────────┐
                         │  4. 创建/打开数据集       │
                         │  LeRobotDataset.create() │
                         │  或 resume 模式下的打开   │
                         └───────────┬───────────┘
                                     │
                         ┌───────────▼───────────┐
                         │  5. robot.connect()    │
                         │  → CAN 连接 + 电机使能    │
                         │  → robot.calibrate()    │
                         │  → 相机连接与预热         │
                         └───────────┬───────────┘
                                     │
                         ┌───────────▼───────────┐
                         │  6. 进入录制循环         │
                         │  while recorded < total:│
                         │    ├─ record_loop()     │
                         │    ├─ 复位等待/reset     │
                         │    ├─ 处理 rerecord      │
                         │    └─ dataset.save_ep()  │
                         └───────────┬───────────┘
                                     │
                         ┌───────────▼───────────┐
                         │  7. robot.disconnect()  │
                         │  → 失能力矩 + CAN 断开   │
                         │  → 相机停止 + 断开       │
                         └───────────┬───────────┘
                                     │
                         ┌───────────▼───────────┐
                         │  8. 上传到 Hub (可选)    │
                         │  dataset.push_to_hub()  │
                         └───────────────────────┘
```

**阶段 2 — 创建 Robot 实例**：
```python
# record.py 第 443 行
robot = make_robot_from_config(cfg.robot)
```
`make_robot_from_config` 根据 `cfg.robot.type` 的值（此处为 `"piper_follower"`）从注册表中查找对应的 dataclass 并实例化。`PiperFollowerConfig` 在此过程中被构造为包含 CAN 端口、相机列表、关节限幅等信息。

**阶段 3 — 创建 Teleoperator**：在本项目的 PC 中继方案中，主臂（PiperLeader）通过 CAN 直接向从臂（PiperFollower）发送目标位置，PC 端不需要再创建 teleoperator 对象来转发动作，因此此处 `teleop = None`。不过代码中保留了 teleoperator 的完整加载路径，以便其他机器人类型使用。

**阶段 4 — 创建数据集**：
```python
# record.py 第 480-490 行
dataset = LeRobotDataset.create(
    cfg.dataset.repo_id,
    cfg.dataset.fps,
    root=cfg.dataset.root,
    robot_type=robot.name,
    features=dataset_features,
    use_videos=cfg.dataset.video,
    ...
)
```
`LeRobotDataset.create()` 在磁盘上创建以下结构：
- `meta/` 目录，包含 `info.json`（元数据）和 `stats.json`（在后续训练步骤中填入）
- `data/` 目录，包含分块（chunk）的 parquet 文件
- `videos/` 目录，按相机名称分为子目录

数据集的 `features`（特征 schema）由两部分合并而来：
1. **动作特征**（`robot.action_features`）：从臂关节的维度信息
2. **观测特征**（`robot.observation_features`）：关节角度 + 相机图像信息

**阶段 5 — robot.connect()**：这一步骤执行以下子操作：
1. 打开 CAN 接口，与从臂的 7 个电机建立通信
2. 切换电机到使能（enable）状态，施加力矩
3. 执行 `calibrate()`：将电机驱动到 parking 位置（初始位姿）。此过程约需 2-3 秒
4. 连接所有配置的相机（如两个 RealSense），启动预热读取

预热期间，相机被连续读取若干帧以确保自动曝光、白平衡等参数稳定。具体代码：
```python
# camera_realsense.py 第 184-191 行
if warmup:
    time.sleep(1)  # RealSense 硬件需要短时间稳定
    start_time = time.time()
    while time.time() - start_time < self.warmup_s:
        self.read()
        time.sleep(0.1)
```

---

## 8.2 录制循环详解

### 8.2.1 record_loop() 函数完整流程

`record_loop()` 是数据录制的核心函数，它被调用两次：
- **录制阶段**：以 `episode_time_s` 为时长，同时采集数据并写入数据集
- **复位阶段**：以 `reset_time_s` 为时长，只采集观测并发送动作（不写入数据集），给操作者时间复位环境

函数签名：

```python
def record_loop(
    robot: Robot,
    events: dict,
    fps: int,
    teleop_action_processor: RobotProcessorPipeline,
    robot_action_processor: RobotProcessorPipeline,
    robot_observation_processor: RobotProcessorPipeline,
    dataset: LeRobotDataset | None = None,
    teleop: Teleoperator | None = None,
    policy: PreTrainedPolicy | None = None,
    preprocessor: PolicyProcessorPipeline | None = None,
    postprocessor: PolicyProcessorPipeline | None = None,
    control_time_s: int | None = None,
    single_task: str | None = None,
    display_data: bool = False,
):
```

**每帧的执行步骤如下**：

```
for timestamp in range(0, control_time_s, 1/fps):   # 实际上用 while + time.perf_counter()
    ┌──────────────────────────────────────────────────────────┐
    │ Step 1: 检查事件                                         │
    │   - events["exit_early"] → 立即终止当前 episode         │
    │   - events["stop_recording"] → 由外层循环处理            │
    │   - events["rerecord_episode"] → 由外层循环处理          │
    ├──────────────────────────────────────────────────────────┤
    │ Step 2: robot.get_observation()                          │
    │   → 读取从臂 7 个电机当前角度 + 2 个相机图像              │
    │   → 返回扁平 dict，键如 "observation/arm/joint_1",       │
    │     "observation/images/wrist", ...                      │
    ├──────────────────────────────────────────────────────────┤
    │ Step 3: robot_observation_processor(obs)                 │
    │   → 默认是 IdentityProcessor（不做处理）                  │
    │   → 可配置为数据增强、归一化等                            │
    ├──────────────────────────────────────────────────────────┤
    │ Step 4: 获取动作                                         │
    │   分支 A (有 policy): predict_action(observation, policy) │
    │   分支 B (有 teleop): teleop.get_action()               │
    │   分支 C (PiPER 中继): 见下方详解                         │
    ├──────────────────────────────────────────────────────────┤
    │ Step 5: 动作处理                                         │
    │   teleop_action_processor((action, obs))                 │
    ├──────────────────────────────────────────────────────────┤
    │ Step 6: 构建 frame dict → dataset.add_frame()            │
    │   frame = {                                              │
    │     **observation_frame,   # 观测数据                     │
    │     **action_frame,       # 动作数据                      │
    │     "task": single_task,  # 任务文本                      │
    │   }                                                      │
    ├──────────────────────────────────────────────────────────┤
    │ Step 7: display_data (可选)                              │
    │   → log_rerun_data(obs_processed, sent_action)           │
    │   → rerun 面板实时显示相机画面和关节状态                  │
    ├──────────────────────────────────────────────────────────┤
    │ Step 8: 帧率控制                                         │
    │   dt_s = time.perf_counter() - start_loop_t              │
    │   busy_wait(1/fps - dt_s)                                │
    │   → 如果 dt_s > 1/fps，busy_wait 为负数，立即进入下一帧  │
    └──────────────────────────────────────────────────────────┘
```

### 8.2.2 PiPER 的动作获取特殊路径

由于 PiPER 采用 PC 中继方案（主臂 CAN 直接中继到从臂 CAN），`teleop` 在本项目中为 `None`，动作来源不是遥操作臂也不是策略，而是直接从观测值中提取。对应代码：

```python
# record.py 第 389-391 行（PiPER 专用分支）
act_processed_teleop = teleop_action_processor((obs, obs))
action_values = act_processed_teleop
```

这里将 `obs` （从臂的当前关节角度）直接作为动作值记录。背后的原理是：在 PC 中继方案中，主臂发出的目标位置命令被 CAN 中继软件（如 `cangw`）直接转发给从臂，从臂的反馈控制会将其实际位置驱动到目标位置。当 PC 通过 `get_observation()` 读取从臂关节角度时，这个角度近似等于主臂被拖拽到的位置（忽略电机的跟踪延迟）。因此，**记录"从臂的实际关节角度"等价于记录"主臂的命令位置"**。

这一设计使得记录命令极简：不需要 `--teleoperator` 参数，不需要 `teleop.connect()`，整个录制流程只需要一台从臂的 CAN 连接。

### 8.2.3 时间控制机制

录制循环的时间控制是理解 FPS 精度的关键。代码中使用 `time.perf_counter()` 获取高精度时间戳（单位：秒），并用两种时间尺度管理流程：

```
┌─────────────────────────────────────────────────────────────────┐
│  episode 级别的时间线（以 episode_time_s=30, fps=30 为例）        │
│                                                                 │
│  start_episode_t ─────────────────────────────────────────────► │
│  │                                                              │
│  │  帧0  帧1  帧2  帧3  .........  帧899                        │
│  │  │    │    │    │               │                            │
│  │  0s  33ms 67ms 100ms          30s                           │
│  │                                                              │
│  │  timestamp = perf_counter() - start_episode_t                │
│  └─────────────────────────────────────────────────────────────┘
│
│  每帧级别的时间控制：
│
│  start_loop_t = perf_counter()
│  │
│  ├── 采集观测 (~5ms)
│  ├── 处理管线 (~1ms)
│  ├── 写入数据集 (~3ms)
│  │
│  dt_s = perf_counter() - start_loop_t    # 例如 9ms
│  sleep_time = 1/30 - 0.009 = 0.033 - 0.009 = 0.024s
│  busy_wait(0.024)                        # 等待到 33ms 满
```

**busy_wait 的实现**（来自 `src/lerobot/utils/robot_utils.py`）：

```python
def busy_wait(seconds):
    if seconds <= 0:
        return  # 如果已超时，立即进入下一帧
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        pass
```

关键行为：
- **不掉帧**：如果某帧耗时超过 `1/fps`（例如 40ms > 33ms），`busy_wait` 接收负数直接返回，下一帧立即开始。不会因为试图"追赶"而累积延迟。
- **有效 FPS 可能低于目标**：如果大多数帧耗时都超过目标间隔（通常因为相机读取慢或磁盘 I/O 阻塞），实际录制帧率会低于 `--dataset.fps` 的设置值。

---

## 8.3 附加功能详解

以原始 LeRobot 录制管线为基础，本项目添加了三个实用增强功能：手动步进模式、可配置复位脚本和有效 FPS 统计。

### 8.3.1 manual_step 模式

**问题背景**：原始录制流程中，每个 episode 之间自动等待固定的 `reset_time_s` 秒后立即开始下一个 episode。但在实际使用中，操作者需要摆放操作物品、清理桌面、调整相机角度——这些操作的耗时不确定。固定的等待时间要么太短（操作者还没准备好），要么太长（浪费时间）。

**解决方案**：新增 `manual_step` 参数。

```python
# RecordConfig 第 218 行
manual_step: bool = False
```

当 `manual_step=True` 时，录制流程的行为变化如下：

```
                    ┌─── manual_step=False（默认） ──┐
                    │                                │
  Episode N 结束     │  Episode N+1 开始              │
  ──────────────────►│◄──────────────────────────────►
                    │   固定等待 reset_time_s 秒       │
                    │                                │
                    └────────────────────────────────┘

                    ┌─── manual_step=True ───────────┐
                    │                                │
  Episode N 结束     │  Episode N+1 开始              │
  ──────────────────►│◄─────── 等待用户按 Enter ─────►│
                    │  "Press Enter to start..."     │
                    │                                │
                    └────────────────────────────────┘
```

代码实现分为两处：

**(1) Episode 开始前等待**：
```python
# record.py 第 519-520 行
if cfg.manual_step:
    input(f"\n[record] Press Enter to start episode {dataset.num_episodes}...")
else:
    log_say(f"Recording episode {dataset.num_episodes}", cfg.play_sounds, blocking=True)
```

**(2) Episode 结束后等待**：
```python
# record.py 第 543-551 行
should_reset = not events["stop_recording"] and (
    (recorded_episodes < cfg.dataset.num_episodes - 1) or events["rerecord_episode"]
)
if should_reset and cfg.manual_step:
    run_reset_command_if_configured()
    input("[record] Episode finished. Reset complete or manually adjusted. Press Enter to continue...")
elif should_reset:
    log_say("Reset the environment", cfg.play_sounds, blocking=True)
    run_reset_command_if_configured()
    # 自动模式下使用固定 reset_time_s 等待
    record_loop(robot=robot, ..., control_time_s=cfg.dataset.reset_time_s, ...)
```

注意：在 `manual_step` 模式下，复位阶段**不再调用 record_loop 来占用固定时间**，而是直接等待用户输入。这大大提高了工作效率——操作者只需花费实际复位所需的时间。

### 8.3.2 run_reset_command_if_configured()

**问题背景**：许多操作环境在每轮录制后需要自动化复位——例如通过另一个脚本驱动传送带将新工件送到工位，或者控制气动执行器将物品推回初始位置。这些操作可能是系统级别的（需要 root 权限或调用其他硬件驱动），不便写入 LeRobot Python 代码中。

**解决方案**：通过环境变量注入任意复位命令，在录制循环的复位阶段自动执行。

```python
# record.py 第 146-159 行
def run_reset_command_if_configured() -> None:
    command = os.environ.get("LEROBOT_RESET_COMMAND")
    if not command:
        return
    pause_file = os.environ.get("LEROBOT_RELAY_PAUSE_FILE")
    print(f"[record] Running reset command: {command}")
    try:
        if pause_file:
            Path(pause_file).parent.mkdir(parents=True, exist_ok=True)
            Path(pause_file).touch()          # 创建暂停文件 → relay 暂停
        subprocess.run(shlex.split(command), check=False)  # 执行复位脚本
    finally:
        if pause_file:
            Path(pause_file).unlink(missing_ok=True)  # 删除暂停文件 → relay 恢复
```

**设计要点分析**：

1. **环境变量驱动，零代码侵入**：只需在启动录制前 `export LEROBOT_RESET_COMMAND="..."` 即可激活，不需要修改 Python 代码。适合快速切换不同工位的复位逻辑。

2. **暂停文件协调机制**：`LEROBOT_RELAY_PAUSE_FILE` 是一个可选的路径。当指定时，执行复位命令前该文件被 `touch` 创建，命令执行完后被 `unlink` 删除。PC 中继程序（`cangw` 或等价组件）可以监控这个文件的存在性——文件存在时暂停 CAN 中继，防止复位命令引起的从臂运动被误认为是主臂的命令输入。

3. **try/finally 确保清理**：无论复位命令成功、失败还是被 Ctrl+C 中断，`finally` 块都会执行，确保暂停文件一定被删除。这是关键的可靠性保证——如果暂停文件残留在磁盘上，relay 将被永久暂停。

4. **非零返回码不中断流程**：`check=False` 意味着即使复位脚本返回错误，录制流程也会继续。原因是在录制的容错性优于单次复位的成功——操作者可以在终端看到错误信息并手动干预，而不是整个录制会话崩溃。

**使用示例**：

```bash
# 场景：录制 pick-and-place，每轮结束后传送带送新物品
export LEROBOT_RESET_COMMAND="python3 /home/user/scripts/advance_conveyor.py --steps 1"
export LEROBOT_RELAY_PAUSE_FILE="/tmp/lerobot_relay_pause"

lerobot-record \
    --robot.type=piper_follower --robot.port=can0 \
    --dataset.repo_id=local/pick_place \
    --dataset.num_episodes=10 \
    --dataset.episode_time_s=30 \
    --dataset.single_task="Pick the object and place it in the bin"
```

### 8.3.3 有效 FPS 统计

**问题背景**：`--dataset.fps=30` 表示**目标**帧率，但实际录制帧率取决于硬件性能。如果相机读取、数据处理、磁盘写入的总耗时超过 33ms（1/30秒），实际帧率就会低于 30。用户需要知道实际帧率来判断录制质量。

**解决方案**：在每个 episode 结束时打印有效 FPS 统计。

```python
# record.py 第 424-432 行
elapsed_s = time.perf_counter() - start_episode_t
if dataset is not None and elapsed_s > 0:
    logging.info(
        "Recorded %d frames in %.2fs, effective fps %.2f / requested fps %s",
        num_frames,
        elapsed_s,
        num_frames / elapsed_s,
        fps,
    )
```

**输出示例**：

```
INFO Recorded 897 frames in 30.12s, effective fps 29.78 / requested fps 30
```

如果有效 FPS 显著低于目标 FPS（例如 22.5 / requested fps 30），说明存在性能瓶颈。常见的瓶颈排查方法：

| 瓶颈来源 | 症状 | 排查手段 |
|----------|------|----------|
| 相机读取慢 | 单帧耗时 > 40ms | 降低相机分辨率、减少 fps 设置 |
| USB 带宽不足 | 双相机时明显 | 将两个相机插到不同的 USB 控制器上 |
| 磁盘 I/O 慢 | 写入峰值时丢帧 | 使用 SSD、减少 `num_image_writer_threads` |
| CPU 忙于图像编码 | CPU 使用率 > 90% | 增大 `video_encoding_batch_size` 推迟编码 |

---

## 8.4 双 RealSense 配置

### 8.4.1 物理布局

本项目使用两台 Intel RealSense 相机从不同视角采集操作场景：

```
┌──────────────────────────────────────────────────────────────┐
│                      工位俯视图                                │
│                                                              │
│    ┌──────────────────────────────────────────┐              │
│    │              global 相机 (D435)            │              │
│    │         固定在工位上方，俯视整个操作区域      │              │
│    │         序列号: 142122071524               │              │
│    │         分辨率: 640×480, 30fps             │              │
│    └──────────────────────────────────────────┘              │
│                          │ 向下拍摄                           │
│                          ▼                                   │
│    ┌──────────┐     ┌─────────────┐     ┌──────────┐        │
│    │  主臂     │     │   操作区域    │     │  从臂     │        │
│    │ (Leader)  │     │  (物品堆放)   │     │(Follower) │        │
│    │           │     │             │     │           │        │
│    └──────────┘     └─────────────┘     └──────────┘        │
│                                               │              │
│                                          wrist 相机 (D435I)  │
│                                          序列号: 238222076529│
│                                          分辨率: 640×480,    │
│                                          颜色30fps, 深度15fps│
│                                          安装在从臂末端，      │
│                                          跟随末端运动          │
└──────────────────────────────────────────────────────────────┘
```

- **wrist（腕部相机）**：Intel RealSense D435I，安装在从臂末端执行器附近，随机械臂运动。提供近距离的操作对象精细视角，适合精确抓取任务。
- **global（全局相机）**：Intel RealSense D435，固定在工位上方约 1.2m 处，视角不变。提供整个操作空间的全景视图，适合定位物体和判断大范围运动。

### 8.4.2 camera_config_realsense.env 配置详解

```bash
# 相机序列号——每台 RealSense 出厂时固化，USB 口换了也不影响识别
export WRIST_REALSENSE_SERIAL="238222076529"
export GLOBAL_REALSENSE_SERIAL="142122071524"

# LeRobot 框架使用的 Robot Cameras 配置
export ROBOT_CAMERAS='{
  wrist: {
    type: intelrealsense,
    serial_number_or_name: "238222076529",
    width: 640, height: 480, fps: 15,
  },
  global: {
    type: intelrealsense,
    serial_number_or_name: "142122071524",
    width: 640, height: 480, fps: 15,
  }
}'
```

**序列号区分的好处**：
- RealSense 相机在 Linux 下的 `/dev/video*` 编号可能因 USB 口插拔而变化
- 序列号是出厂烧录的，永久不变
- 使用 `serial_number_or_name` 参数可确保无论相机插在哪个 USB 口，都能正确识别

**获取相机序列号**：
```bash
# 使用 LeRobot 内置工具
lerobot-find-cameras realsense

# 或直接使用 realsense-viewer
realsense-viewer
# GUI 中会显示每个相机的序列号
```

**fps 设置说明**：虽然相机硬件支持 30fps，但此处配置为 15fps。这是因为：
1. 两台相机共享同一个 USB 控制器的带宽，总带宽有限
2. 模仿学习任务通常不需要 30fps 的视频数据（每帧之间的变化很小）
3. 降低 fps 可减少数据集的存储体积，加速训练

### 8.4.3 async_read 超时修改

**原始代码**：
```python
# camera_realsense.py 第 489 行（修改前）
def async_read(self, timeout_ms: float = 200) -> np.ndarray:
```

**修改后**：
```python
# camera_realsense.py 第 489 行（修改后）
def async_read(self, timeout_ms: float = 1000) -> np.ndarray:
```

**修改原因分析**：

`async_read` 的工作机制如下：
1. 后台线程 `_read_loop` 持续从 RealSense 读取帧，存入 `latest_frame`，并通过 `new_frame_event` 事件通知
2. `async_read` 被主线程调用时，等待 `new_frame_event` 被设置（最多等 `timeout_ms` 毫秒）
3. 如果超时仍未等到新帧，抛出 `TimeoutError`

原始值 `200ms` 的矛盾：
- RealSense 相机以 30fps 运行，帧间隔约 33ms
- 但进程调度、图像传输、后台线程切换都会引入额外延迟
- 当**两台**相机同时运行时，主线程需要在两个相机之间切换读取，可能出现"刚读了一个相机，另一个相机的后台线程还在等待新帧"的情况
- 200ms 对于单相机足够，但双相机时偶尔会因为 CPU 调度而超时

改为 `1000ms` 后：
- 给予充足的等待时间（30fps 下约可完成 30 帧的读取周期）
- 几乎消除了因调度延迟导致的误报超时
- 代价极低：正常情况下几毫秒就能等到新帧，超时只是一个安全上限

---

## 8.5 语音提示系统

语音提示系统帮助操作者在不需要看屏幕的情况下感知录制状态。这在操作环境中非常重要——操作者的注意力应该集中在机械臂和操作物品上，而不是终端输出。

### 8.5.1 TTS 引擎降级链

```python
# utils.py 第 202-232 行
def say(text: str, blocking: bool = False):
    system = platform.system()

    if system == "Darwin":                  # macOS: 系统内置 say 命令
        cmd = ["say", text]

    elif system == "Linux":
        if shutil.which("espeak-ng"):       # 首选: eSpeak NG（优秀的中文支持）
            cmd = ["espeak-ng", text]
        elif shutil.which("espeak"):        # 备选: eSpeak（旧版）
            cmd = ["espeak", text]
        else:                               # 兜底: speech-dispatcher
            cmd = ["spd-say", text]
            if blocking:
                cmd.append("--wait")

    elif system == "Windows":
        cmd = ["PowerShell", "-Command",
               "Add-Type -AssemblyName System.Speech; "
               f"(New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak('{text}')"]
```

**引擎对比**：

| 引擎 | 中文发音质量 | 安装方式 | 说明 |
|------|-------------|----------|------|
| espeak-ng | 良好（支持拼音和声调） | `sudo apt install espeak-ng` | 首选推荐。支持多语言，中文合成效果明显优于 spd-say |
| espeak | 一般 | `sudo apt install espeak` | espeak-ng 的前身，中文支持较弱 |
| spd-say | 差 | Ubuntu 自带的 speech-dispatcher | 英文尚可，中文发音非常机械，不建议用于中文提示 |

**为什么需要更换**：原版 LeRobot 只使用 `spd-say` 作为 Linux TTS 方案。在英文环境下尚可接受，但在中文操作提示场景下（如"开始录制"、"请复位环境"），`spd-say` 的中文输出难以辨认。升级到 `espeak-ng` 显著改善了中文语音的可懂度。

安装依赖：
```bash
sudo apt update
sudo apt install espeak-ng
```

### 8.5.2 play_cached_voice_prompt() — 预生成语音播放

**问题背景**：TTS 实时合成存在两个问题：
1. **延迟大**：`espeak-ng` 合成一段短文本约需 200~500ms，这个延迟会在录制循环中累积
2. **质量不稳定**：同一段文本每次合成的韵律可能略有不同

**解决方案**：预先用高质量的云 TTS（如 Azure Neural TTS）生成语音文件，录制时直接播放这些文件。

语音文件存放在 `audio_prompts/` 目录下，按语言和语音名称组织：

```
audio_prompts/
├── en-US-AriaNeural/              # 英文女声 (Azure Neural)
│   ├── recording_episode_0.wav    # "Recording episode 0"
│   ├── recording_episode_1.wav    # "Recording episode 1"
│   ├── ...
│   ├── recording_episode_19.wav   # "Recording episode 19"
│   ├── reset_the_environment.wav  # "Reset the environment"
│   ├── re_record_episode.wav      # "Re-record episode"
│   ├── stop_recording.wav         # "Stop recording"
│   └── exiting.wav                # "Exiting"
│
└── zh-CN-XiaoxiaoNeural/          # 中文女声 (Azure Neural, 晓晓)
    ├── recording_episode_0.mp3    # "正在录制第0个episode"
    ├── recording_episode_1.mp3
    ├── ...
    ├── recording_episode_50.mp3
    ├── reset_the_environment.mp3  # "请复位操作环境"
    ├── re_record_episode.mp3      # "重新录制这个episode"
    └── stop_recording.mp3         # "停止录制"
```

**语音目录选择**：通过环境变量 `LEROBOT_VOICE_PROMPT_DIR` 指定。例如：
```bash
# 使用中文语音
export LEROBOT_VOICE_PROMPT_DIR="audio_prompts/zh-CN-XiaoxiaoNeural"

# 使用英文语音
export LEROBOT_VOICE_PROMPT_DIR="audio_prompts/en-US-AriaNeural"
```

**播放机制**：
```python
# utils.py 第 253-283 行
def play_cached_voice_prompt(text: str) -> bool:
    prompt_dir = os.environ.get("LEROBOT_VOICE_PROMPT_DIR")
    if not prompt_dir:
        return False                               # 未配置目录，回退到 TTS

    key = _voice_prompt_key(text)
    if key is None:
        return False                               # 文本无法映射到预生成文件

    wav_path = Path(prompt_dir) / f"{key}.wav"
    if wav_path.exists() and shutil.which("aplay"):
        subprocess.run(["aplay", "-q", str(wav_path)], check=False)
        return True                                # WAV 用 aplay 播放（低延迟 ~5ms）

    path = Path(prompt_dir) / f"{key}.mp3"
    if not path.exists():
        return False                               # 文件不存在，回退到 TTS

    try:
        import pygame
        if not pygame.mixer.get_init():
            pygame.mixer.init()
        pygame.mixer.music.load(str(path))
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():       # 阻塞等待播放完成
            time.sleep(0.02)
        return True
    except Exception as exc:
        logging.warning("Failed to play cached voice prompt %s: %s", path, exc)
        return False                               # 播放失败，回退到 TTS
```

**文件格式选择**：
- `.wav` 格式优先：Linux 下 `aplay` 直接播放，延迟约 5ms，无额外依赖
- `.mp3` 格式备选：需要 pygame 库，播放延迟约 10ms，但文件体积更小

**文本到文件名的映射**（`_voice_prompt_key`）：
```python
# utils.py 第 239-250 行
def _voice_prompt_key(text: str) -> str | None:
    # "Recording episode 3" → "recording_episode_3"
    match = re.fullmatch(r"Recording episode (\d+)", text)
    if match:
        return f"recording_episode_{match.group(1)}"

    # 固定文本映射
    mapping = {
        "Reset the environment": "reset_the_environment",
        "Re-record episode": "re_record_episode",
        "Stop recording": "stop_recording",
        "Exiting": "exiting",
    }
    return mapping.get(text)
```

### 8.5.3 play_phase_beep() — 不同阶段的提示音

语音播放适合告知"正在发生什么"，但有时候操作者只需要一个简短的声音信号来判断状态变化。提示音系统为不同事件分配不同频率的蜂鸣音：

| 事件 | 频率 | 模式 | 含义 |
|------|------|------|------|
| 开始录制 episode | 880 Hz | 单声（0.16s） | 高音短促——"准备好了" |
| 复位环境 | 440 Hz | 双声（0.12s × 2） | 中音双声——"请复位" |
| 重新录制 episode | 660 Hz | 三声（0.1s × 3） | 中高音三连——"重来" |
| 停止/退出 | 220 Hz | 单长声（0.3s） | 低音长——"结束" |

```python
# utils.py 第 308-327 行
def play_phase_beep(text: str):
    if not _truthy_env("LEROBOT_PHASE_BEEP"):
        return                    # 环境变量未启用，跳过

    lower = text.lower()
    if "recording episode" in lower:
        pattern = [(880, 0.16)]                    # 高音单声
    elif "reset" in lower:
        pattern = [(440, 0.12), (440, 0.12)]      # 中音双声
    elif "re-record" in lower:
        pattern = [(660, 0.1), (660, 0.1), (660, 0.1)]  # 中高音三声
    elif "stop" in lower or "exiting" in lower:
        pattern = [(220, 0.3)]                     # 低音长声
    else:
        pattern = [(660, 0.1)]                     # 默认中高音

    for index, (freq, duration) in enumerate(pattern):
        if index > 0:
            time.sleep(0.08)         # 多声之间 80ms 间隔
        _play_tone(freq, duration)
```

**计算机声学设计原理**：
- **高音 (880Hz)** 对应开始：高频声音具有"唤醒"和"注意"的心理暗示，类似比赛开始时的哨声
- **中音 (440Hz/660Hz)** 对应中间状态：不突兀，提示但不打断注意力
- **低音 (220Hz)** 对应结束：低频声音具有"终止"和"确认"的感觉，类似心跳声的结束
- 频率选择避开了 8 度重复（如 440Hz 与 880Hz 相差一个八度），确保操作者能清晰分辨不同阶段的提示

**提示音生成实现**（`_play_tone`）：

```python
# utils.py 第 286-305 行
def _play_tone(freq_hz: int, duration_s: float, volume: float = 0.35):
    if not shutil.which("aplay"):
        return                          # aplay 不可用则跳过

    sample_rate = 16_000                # 16kHz 采样率（语音品质）
    samples = int(sample_rate * duration_s)
    amplitude = int(32767 * volume)     # 35% 音量，避免刺耳

    path = Path("/tmp/lerobot_phase_beep.wav")

    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(1)             # 单声道
        wav.setsampwidth(2)             # 16-bit 采样
        wav.setframerate(sample_rate)

        frames = bytearray()
        for i in range(samples):
            # 生成正弦波：y(t) = A * sin(2π * f * t)
            sample = int(amplitude * math.sin(2 * math.pi * freq_hz * i / sample_rate))
            frames.extend(sample.to_bytes(2, byteorder="little", signed=True))
        wav.writeframes(frames)

    subprocess.run(["aplay", "-q", str(path)], check=False)
```

关键技术选择：
- **16kHz 采样率**：奈奎斯特频率为 8kHz，880Hz 的最高频率远在其下，16kHz 足够且文件极小
- **直接生成 WAV 而非调用外部合成器**：避免额外依赖，生成耗时约 2ms
- **`/tmp/lerobot_phase_beep.wav`**：固定临时路径，每次覆写，不积累文件垃圾

**启用提示音**：
```bash
export LEROBOT_PHASE_BEEP=1
```

### 8.5.4 log_say() — 统一的语音播报接口

```python
# utils.py 第 330-337 行
def log_say(text: str, play_sounds: bool = True, blocking: bool = False):
    logging.info(text)                           # 始终写日志

    if play_sounds:
        if not play_cached_voice_prompt(text):   # 优先级 1: 预生成文件
            say(text, blocking)                  # 优先级 2: 实时 TTS
    if blocking:
        play_phase_beep(text)                    # 关键节点播提示音
```

**调用场景分析**：

`log_say` 在录制流程中被调用 8 次，每次的语义和策略不同：

| 调用位置 | text 内容 | blocking | 说明 |
|----------|-----------|----------|------|
| 录制 episode 前 | "Recording episode N" | True | 阻断：确保操作者听到提示后再开始录制 |
| 自动复位开始 | "Reset the environment" | True | 阻断：提示操作者开始复位动作 |
| 重新录制 | "Re-record episode" | True | 阻断：操作者需要知道发生了重录 |
| 停止录制 | "Stop recording" | True | 阻断：最终确认，避免操作者误以为还在录制 |
| 退出程序 | "Exiting" | False | 非阻断：退出时不需要等待 |

`blocking=True` 的含义是：语音播报会阻塞主线程直到播放完成（通常 1~3 秒），确保操作者在录制开始前已收到完整提示。`blocking=False` 则异步播放，不阻塞录制流程。

---

## 8.6 录制启动脚本

### 8.6.1 直连 CAN 方案（简单版）

当主臂和从臂通过 CAN 硬件中继直接通信（不需要 PC 中继软件）时，录制命令最简洁：

```bash
#!/bin/bash
# 4__record.sh — 直连 CAN 录制脚本

source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot

HF_USER=$(hf auth whoami | head -n 1)
echo "HuggingFace user: $HF_USER"

lerobot-record \
    --robot.type=piper_follower \
    --robot.port=can0 \
    --robot.cameras='{
        top: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30},
        left: {type: opencv, index_or_path: 4, width: 640, height: 480, fps: 30}
    }' \
    --robot.id=black \
    --dataset.num_episodes=50 \
    --dataset.single_task="Grab the yellow car and put in the box" \
    --display_data=true \
    --dataset.repo_id=${HF_USER}/piper_pick_yellow_car
```

**关键点**：
- `robot.port=can0`：从臂连接到 CAN0 接口
- 不需要 `--teleoperator` 参数：主臂的命令由 CAN 硬件中继直接转发
- 相机使用 `opencv` 类型（USB 摄像头）或 `intelrealsense` 类型（RealSense）
- `display_data=true`：打开 rerun 可视化面板，实时查看相机和关节状态
- `dataset.repo_id` 使用 HuggingFace 用户名作为前缀

### 8.6.2 PC 中继方案（record_with_pc_relay.sh）

当使用 PC 中继方案时，录制前需要：
1. 加载相机环境变量配置
2. 启动 PC 中继程序（另开终端）

```bash
# 终端 1：启动 PC 中继
./cangw --leader can0 --follower can1

# 终端 2：加载相机配置并开始录制
source ./camera_config_realsense.env
source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot

# 启用语音提示（中文）和提示音
export LEROBOT_VOICE_PROMPT_DIR="audio_prompts/zh-CN-XiaoxiaoNeural"
export LEROBOT_PHASE_BEEP=1

# 可选：配置自动复位命令
export LEROBOT_RESET_COMMAND="python3 /home/user/scripts/reset_env.py"
export LEROBOT_RELAY_PAUSE_FILE="/tmp/lerobot_relay_pause"

lerobot-record \
    --robot.type=piper_follower \
    --robot.port=can1 \
    --robot.cameras="${ROBOT_CAMERAS}" \
    --robot.id=black \
    --dataset.repo_id=local/piper_bimanual_task \
    --dataset.num_episodes=30 \
    --dataset.episode_time_s=60 \
    --dataset.fps=30 \
    --dataset.single_task="Pick the red block and place it on the blue platform" \
    --display_data=true \
    --manual_step=true \
    --play_sounds=true
```

注意：`--robot.cameras="${ROBOT_CAMERAS}"` 使用了从 `camera_config_realsense.env` 中加载的环境变量，这样 camera 配置就可以独立于启动脚本管理。

---

## 8.7 数据集输出结构

### 8.7.1 完整目录树

录制完成后，数据集在磁盘上的完整结构如下：

```
datasets/local/piper_task/                    # <root>/<repo_id>
│
├── meta/
│   ├── info.json                             # 数据集元信息
│   │   ├── "fps": 30                         #   录制帧率
│   │   ├── "total_episodes": 10              #   episode 总数
│   │   ├── "total_frames": 8990              #   总帧数
│   │   ├── "total_tasks": 1                  #   任务数
│   │   ├── "features": {...}                 #   特征 schema
│   │   └── "robot_type": "piper_follower"    #   机器人类型
│   │
│   ├── stats.json                            # 归一化统计量
│   │   ├── "observation.state": {            #   关节角度
│   │   │   "mean": [...], "std": [...],      #
│   │   │   "min": [...], "max": [...]        #
│   │   │   }                                 #
│   │   └── "action": {                       #   动作命令
│   │       "mean": [...], "std": [...],      #
│   │       "min": [...], "max": [...]        #
│   │       }                                 #
│   │
│   └── tasks.jsonl                           # 任务描述（每行一个 episode）
│       ├── {"task_index": 0, "task": "Pick the red block..."}
│       ├── {"task_index": 1, "task": "Pick the red block..."}
│       └── ...
│
├── data/
│   └── chunk-000/                            # 第一个数据块
│       ├── episode_000000.parquet            # Episode 0 所有帧的非图像数据
│       ├── episode_000001.parquet            # Episode 1
│       ├── ...
│       └── episode_000009.parquet            # 每个 chunk 可包含多个 episode
│
└── videos/
    ├── observation.images.wrist/             # 腕部相机视频
    │   ├── episode_000000.mp4               # Episode 0 的视频
    │   ├── episode_000001.mp4
    │   └── ...
    │
    └── observation.images.global/            # 全局相机视频
        ├── episode_000000.mp4
        ├── episode_000001.mp4
        └── ...
```

### 8.7.2 关键文件说明

**info.json — 数据集元信息**：

```json
{
    "fps": 30,
    "total_episodes": 10,
    "total_frames": 8990,
    "total_tasks": 1,
    "total_chunks": 1,
    "chunks_size": 1000,
    "features": {
        "observation.state": {
            "dtype": "float32",
            "shape": [7],
            "names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper"]
        },
        "observation.images.wrist": {
            "dtype": "video",
            "shape": [3, 480, 640],
            "names": ["channel", "height", "width"]
        },
        "action": {
            "dtype": "float32",
            "shape": [7],
            "names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper"]
        },
        "task": {
            "dtype": "string",
            "shape": [1],
            "names": null
        }
    },
    "robot_type": "piper_follower"
}
```

**stats.json**：此文件在录制阶段仅包含占位值，真正的统计量在训练脚本的 `compute_stats()` 步骤中计算。它包含每个浮点特征维度的均值、标准差、最小值和最大值，用于训练时的数据归一化。

**tasks.jsonl**：每行一个 JSON 对象，记录该 episode 的任务文本。如果所有 episode 使用相同任务，每行内容相同。

**parquet 文件**：每个 episode 的表格数据以 Apache Parquet 格式存储。每行对应一帧，列包括：
- `observation.state` (float32[7])：从臂 7 个关节的实际角度
- `action` (float32[7])：记录的动作（在 PiPER 方案中等于从臂角度）
- `task` (string)：任务描述文本
- `episode_index` (int64)：episode 编号
- `frame_index` (int64)：帧在 episode 内的编号
- `timestamp` (float32)：相对于 episode 开始的时间戳（秒）
- `index` (int64)：全局帧编号

**视频文件**：每个 camera key 对应一个视频子目录。视频以 MP4 (H.264) 格式编码，帧率与数据集 fps 一致。视频帧索引与 parquet 文件的 `frame_index` 列对齐。

### 8.7.3 数据读取示例

录制完成后，可以用以下 Python 代码快速检查数据质量：

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# 加载本地数据集
dataset = LeRobotDataset("local/piper_task")

# 查看元信息
print(f"FPS: {dataset.fps}")
print(f"Total episodes: {dataset.num_episodes}")
print(f"Total frames: {dataset.num_frames}")
print(f"Features: {dataset.features}")

# 读取某个 episode 的所有帧
episode_0 = dataset.hf_dataset.filter(
    lambda x: x["episode_index"] == 0
)
print(f"Episode 0 has {len(episode_0)} frames")

# 查看第一帧的数据
first_frame = dataset[0]
print(f"Joint angles: {first_frame['observation.state']}")
print(f"Task: {first_frame['task']}")
```

---

## 8.8 Replay 功能 — 数据验证

### 8.8.1 功能概述

`lerobot-replay` 是一个独立的数据验证工具，功能是读取已录制的数据集，将保存的动作命令逐帧发送给从臂，让从臂"重放"示教过程。它的核心应用场景：

1. **验证数据质量**：重放能展示数据中是否存在不合理的大幅度跳跃、抖动或噪声。如果从臂在重放时剧烈摇摆，说明原始数据有质量问题。
2. **验证动作可行性**：不是所有从数据中学到的行为都能在物理机器人上执行。Replay 在真实硬件上验证记录的关节轨迹是否平滑、可达。
3. **调试硬件问题**：如果 replay 中动作表现与录制时不一致（例如末端位置偏移），可以定位是电机校准、CAN 延迟还是关节零位漂移问题。
4. **演示和展示**：无需操作者再次示教，机器人可以自动重复之前的操作过程。

### 8.8.2 命令用法

```bash
lerobot-replay \
    --robot.type=piper_follower \
    --robot.port=can0 \
    --robot.id=black \
    --dataset.repo_id=local/piper_pick_yellow_car \
    --dataset.episode=0
```

关键参数：
- `--robot.type/port/id`：与录制时相同的从臂配置
- `--dataset.repo_id`：已录制数据集的路径标识
- `--dataset.episode`：要回放的具体 episode 编号

### 8.8.3 回放循环

```python
# replay.py 第 107-125 行
log_say("Replaying episode", cfg.play_sounds, blocking=True)

for idx in range(len(episode_frames)):
    start_episode_t = time.perf_counter()

    # 从数据集中取出第 idx 帧的动作
    action_array = actions[idx]["action"]
    action = {}
    for i, name in enumerate(dataset.features["action"]["names"]):
        action[name] = action_array[i]

    # 读取当前观测（用于 action processor 的处理上下文）
    robot_obs = robot.get_observation()

    # 处理动作
    processed_action = robot_action_processor((action, robot_obs))

    # 发送动作到从臂
    _ = robot.send_action(processed_action)

    # 帧率控制
    dt_s = time.perf_counter() - start_episode_t
    busy_wait(1 / dataset.fps - dt_s)

robot.disconnect()
```

与录制循环的核心区别：
- **不需要相机**：Replay 只发送关节命令，不需要读取相机图像
- **动作来自数据集而非主臂或策略**：逐帧读取 parquet 文件中的 `action` 列
- **不需要写数据集**：纯回放，无数据写入
- **同样的帧率控制**：保持与录制时相同的帧率，确保回放速度一致

---

## 8.9 常见问题与排查

### 8.9.1 FPS 不稳定 / 有效 FPS 远低于目标值

**现象**：终端输出 `effective fps 18.34 / requested fps 30`，录制的视频明显卡顿。

**原因与解决方案**：

| 可能原因 | 排查方法 | 解决方案 |
|----------|----------|----------|
| 相机 USB 带宽不足 | 查看 `dmesg \| grep -i usb` 是否有带宽错误 | 将两个相机插到不同的 USB 控制器（查看 `lsusb -t` 确认总线拓扑） |
| 相机分辨率/帧率过高 | 计算带宽：`640×480×3×30×2相机 ≈ 165 MB/s` | 降低到 `640×480, fps=15` 或 `320×240, fps=30` |
| 磁盘写入慢 | `iotop -o` 观察写入速率 | 使用 SSD；增大 `video_encoding_batch_size` 推迟视频编码 |
| 图像写入线程过多 | 线程竞争导致主线程阻塞 | 减少 `num_image_writer_threads_per_camera` 到 2 |
| CPU 负载高 | `htop` 观察 CPU 利用率 | 增加 `num_image_writer_processes` 利用多核并行 |

**系统性优化命令**（使用前先备份数据）：

```bash
lerobot-record \
    --robot.type=piper_follower --robot.port=can0 \
    --robot.cameras='{wrist: {type: intelrealsense, serial_number_or_name: "238222076529", width: 320, height: 240, fps: 15}}' \
    --dataset.repo_id=local/test_optimized \
    --dataset.fps=15 \
    --dataset.num_image_writer_threads_per_camera=2 \
    --dataset.video_encoding_batch_size=10 \
    ...
```

### 8.9.2 录制中断 / 电机失能

**现象**：录制过程中从臂突然失能，终端输出 CAN 通信错误。

**排查清单**：

1. **CAN 物理连接**：
   ```bash
   # 检查 CAN 接口是否 up
   ip link show can0
   # 应显示 "state UP"，如果显示 "state DOWN"：
   sudo ip link set can0 up type can bitrate 1000000
   ```

2. **CAN 线缆**：检查 DB9 或 M12 接头是否松动。双臂系统通常有 4 个 CAN 节点（主臂 × 2 口、从臂 × 2 口），每个节点都需要可靠连接。

3. **终端电阻**：CAN 总线两端必须有 120Ω 终端电阻。如果使用 USB-CAN 适配器，确认适配器是否已内置终端电阻。

4. **电机温度保护**：电机长时间工作可能过热触发保护。如果从臂突然失能，等待 5 分钟冷却后再试。

5. **电源不足**：7 个 AGILEX 电机（尤其是 joint1-3 的大扭矩 AGILEX-M）对电源要求高。确认 24V 电源额定电流 >= 15A。

### 8.9.3 相机不工作

**现象**：`lerobot-find-cameras realsense` 找不到相机，或连接时超时。

**排查步骤**：

```bash
# 1. 确认 USB 设备被系统识别
lsusb | grep -i intel
# 应输出类似：Bus 003 Device 005: ID 8086:0b07 Intel Corp. Intel(R) RealSense(TM) Depth Camera 435

# 2. 确认序列号
lerobot-find-cameras realsense
# 对比输出的序列号与 camera_config_realsense.env 中的序列号是否一致

# 3. 检查 USB 线缆
# RealSense 对 USB 线缆质量敏感。建议使用：
# - 原装线缆 或 高质量的 USB 3.0 线缆
# - 长度不超过 2m（过长会导致信号衰减）
# - 避免使用 USB Hub（直接插主机 USB 口）

# 4. 重置 USB 设备
sudo usbreset <bus>/<device>  # 例如 sudo usbreset 003/005
```

### 8.9.4 语音不播放 / 中文发音差

**排查步骤**：

```bash
# 1. 确认 espeak-ng 已安装
which espeak-ng          # 应输出 /usr/bin/espeak-ng
espeak-ng "测试中文"      # 听听是否正常

# 2. 安装 espeak-ng（如果未安装）
sudo apt update && sudo apt install espeak-ng

# 3. 确认语音提示目录正确
ls audio_prompts/zh-CN-XiaoxiaoNeural/
# 应列出 .mp3 文件

# 4. 确认环境变量已设置
echo $LEROBOT_VOICE_PROMPT_DIR
# 应输出 audio_prompts/zh-CN-XiaoxiaoNeural（或完整路径）

# 5. 确认 pygame 已安装（用于播放 mp3）
python3 -c "import pygame; print('ok')"
```

### 8.9.5 磁盘空间不足

**现象**：录制到一半报错 `No space left on device`。

**空间估算**：以 640×480, 30fps, 60s episode 为例：

| 数据类型 | 单帧大小 | 一个 episode (1800 帧) | 50 个 episode |
|----------|----------|------------------------|---------------|
| Parquet (关节数据) | ~200 bytes | ~0.36 MB | ~18 MB |
| 视频 (H.264, 1 相机) | ~15 KB | ~27 MB | ~1.35 GB |
| 视频 (H.264, 2 相机) | ~30 KB | ~54 MB | ~2.7 GB |
| 原始 png (录制过程中) | ~150 KB | ~270 MB | — (编码后转为视频) |
| **总计（2相机, 50ep）** | — | — | **约 3-5 GB** |

**预防措施**：
```bash
# 录制前检查磁盘空间
df -h datasets/

# 建议至少保留录制预估空间的 3 倍余量（考虑临时 png 文件）
```

### 8.9.6 录制但未生成视频

**现象**：`videos/` 目录为空，只有 png 文件残留。

**原因**：视频编码在 episode 保存后异步执行。如果录制提前终止或编码进程被杀，视频可能未完成编码。

**解决方案**：
```bash
# 确认 video encoding batch size 设置
# 如果设得过大（如 50），所有 episode 录制完后才编码
# 建议设为 1（每个 episode 录完立即编码）
# 或设为合理值如 5（每录完 5 个 episode 批量编码一次）

lerobot-record ... --dataset.video_encoding_batch_size=1
```

---

## 8.10 章节总结

本章从 CLI 入口点 `lerobot-record` 出发，完整剖析了 PiPER 机械臂的数据录制管线。核心要点回顾：

1. **管线的入口和参数体系**：`record.py:main()` → `record()`，通过嵌套的 `RecordConfig` dataclass 管理所有参数，`@parser.wrap()` 装饰器自动完成解析和校验。

2. **录制循环的逐帧时序**：`record_loop()` 每帧依次执行事件检查 → `get_observation()` → 处理管线 → 动作获取 → `dataset.add_frame()` → 帧率控制。PiPER 的特殊之处在于动作直接从从臂观测中提取（因为主臂命令通过 CAN 中继已在从臂上体现为实际位置）。

3. **手动步进模式**（`manual_step=True`）：将固定时间的复位等待改为等待操作者按 Enter，更适合需要手动摆放物品的场景。

4. **可配置复位命令**（`LEROBOT_RESET_COMMAND`）：通过环境变量注入外部脚本，配合暂停文件机制协调 PC 中继。

5. **双 RealSense 配置**：利用序列号区分相机，wrist（腕部）和 global（全局）两个视角互补。`async_read` 超时从 200ms 增加到 1000ms 解决了双相机读取的调度问题。

6. **语音提示系统**：包括 TTS 引擎降级链（espeak-ng → espeak → spd-say）、预生成高质量语音文件播放、以及不同频率的蜂鸣提示音。整体设计让操作者不需要看屏幕即可知晓录制状态。

7. **数据集输出结构**：`meta/`（info.json + stats.json + tasks.jsonl）、`data/`（parquet 文件）、`videos/`（mp4 文件）三层结构，支持 HuggingFace datasets 库直接加载。

8. **Replay 回放验证**：`lerobot-replay` 逐帧读取数据集中的动作并重放到从臂上，用于验证数据质量和动作可行性。

9. **常见问题排查**：覆盖了 FPS 不稳定、录制中断、相机不工作、语音播放失败、磁盘空间不足等实际使用中的高频问题。
