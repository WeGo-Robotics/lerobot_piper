# 附录 A：脚本参考手册

本附录为 PiPER 双臂机器人 + LeRobot 模仿学习流水线中所有核心脚本提供完整的参考文档。每个脚本均涵盖用途说明、参数列表、使用示例和关键注意事项。

---

## A.1 `scripts/piper_pc_relay_teleop.py` — PC 中继遥操作脚本

### 用途

将主臂（leader）的关节指令通过 PC 实时转发给从臂（follower），实现主从遥操作。典型拓扑如下：

```
主臂   -> USB-CAN A -> can0 -> PC
从臂   -> USB-CAN B -> can1 -> PC
```

脚本从 `can0` 读取主臂关节目标，经过符号映射、偏移补偿、速率限制和安全限位后，将指令发送到 `can1` 上的从臂。**默认为干运行（dry-run）模式，不会驱动物理臂**，需要显式指定 `--execute` 才会实际发送控制指令。

### 环境要求

- 两条 CAN 总线均已激活（`ip link set can0 up` / `can1 up`），波特率 1 Mbps
- 主臂与从臂均已上电，且分别连接至对应的 USB-CAN 适配器
- `piper_sdk` 已安装，CAN 接口权限正常（通常需要 `sudo` 或 udev 规则）
- Python 解释器：`piper_sdk/piper/bin/python`（SDK 自带虚拟环境）

### 完整参数列表

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--leader` | `str` | `"can0"` | 连接主臂的 CAN 接口名称 |
| `--follower` | `str` | `"can1"` | 连接从臂的 CAN 接口名称 |
| `--source` | `str` | `"auto"` | 读取主臂关节数据的数据源。可选值：`auto`（优先 control 流，其次 feedback 流）、`feedback`（仅读反馈帧）、`control`（仅读控制帧） |
| `--hz` | `float` | `10.0` | 中继循环频率（Hz）。必须为正数 |
| `--speed` | `int` | `10` | 从臂运动速度百分比，写入 `MotionCtrl_2` 命令。取值范围 [0, 100] |
| `--max-step-deg` | `float` | `1.0` | 单个控制周期内从臂每个关节的最大步进量（度）。用于速率限制，避免从臂突跳 |
| `--mode-command-period` | `float` | `1.0` | 重复发送 `MotionCtrl_2` 模式命令的间隔（秒）。设为 0 则禁用重复模式命令 |
| `--gripper-hz` | `float` | `5.0` | 夹爪指令的最大发送频率（Hz）。设为 0 表示不限频 |
| `--print-period` | `float` | `1.0` | 状态打印间隔（秒）。设为 0 禁用周期性打印 |
| `--pause-file` | `str` | `""` | 暂停文件路径。当该文件存在时暂停向从臂发送指令；删除后恢复发送 |
| `--duration` | `float` | `0.0` | 运行时长（秒）。设为 0 表示持续运行直到 Ctrl-C |
| `--signs` | `str` | `"1,1,1,1,1,1"` | 每关节符号映射（6 个逗号分隔的浮点数）。例如 `-1` 表示反转该关节方向 |
| `--offset-deg` | `str` | `"0,0,0,0,0,0"` | 每关节偏移量（度），6 个逗号分隔的浮点数 |
| `--include-gripper` | `flag` | `False` | 是否同时中继夹爪目标 |
| `--high-follow` | `flag` | `False` | 启用高跟随模式（`MotionCtrl_2` 的 `is_mit_mode=0xAD`） |
| `--skip-follower-config` | `flag` | `False` | 跳过初始模式/使能配置，不发送初始 `MotionCtrl_2` 和 `EnableArm` 命令 |
| `--execute` | `flag` | `False` | **实际发送指令**。不指定则脚本仅打印状态，不驱动物理臂（干运行模式） |

### 使用示例

```bash
# 干运行（安全预览，不驱动物理臂）
python scripts/piper_pc_relay_teleop.py

# 实际执行遥操作，包含夹爪，高跟随模式
python scripts/piper_pc_relay_teleop.py \
  --leader can0 --follower can1 \
  --hz 20 --speed 80 --max-step-deg 5.0 \
  --include-gripper --high-follow \
  --source control \
  --execute

# 干运行，反转关节 2 和关节 4 方向
python scripts/piper_pc_relay_teleop.py --signs "1,-1,1,-1,1,1"
```

### 关键说明

- **安全机制**：默认不执行（dry-run）；关节限位由 `JOINT_LIMITS_MDEG` 和 `GRIPPER_LIMIT_UM` 定义；速率限制通过 `--max-step-deg` 控制单步增量；`--pause-file` 提供外部暂停机制
- **数据源选择**：`auto` 模式优先读取控制帧（`GetArmJointCtrl`），当控制帧无数据时回退到反馈帧（`GetArmJointMsgs`）；`control` 模式在主臂实际发送控制指令时更实时
- **退出行为**：收到 `SIGINT`/`SIGTERM` 后，如果处于执行模式，脚本会向从臂发送待机命令（`MotionCtrl_2(0x00, 0x01, 0)`），然后断开连接

---

## A.2 `scripts/piper_act_safe_rollout.py` — ACT 策略安全部署脚本

### 用途

加载训练好的 ACT（Action Chunking Transformer）策略，读取机器人状态和相机图像，预测动作，并对从臂执行安全限幅后的动作命令。**默认模式为干运行（dry-run）**：加载策略、读取传感器、推理动作、打印结果，但不向机械臂发送任何控制指令。

### 干运行模式说明

干运行（`--execute` 未指定）是本脚本的核心安全设计：

1. 加载完整的策略检查点（配置文件 + 模型权重 + 预处理器/后处理器）
2. 连接从臂和 RealSense 相机
3. 每步读取当前观测（关节位置 + 图像），运行策略推理
4. 计算预测动作，应用 delta 限幅和 EMA 平滑
5. 打印当前状态、预测值和限幅后的指令，**不驱动物理臂**
6. 上下文内容

此模式用于：验证检查点完整性、检查推理速度、评估动作范围合理性、在真实部署前做最后的 dry-run 验证。

### 完整参数列表

#### 必需参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--checkpoint` | `str` | `"../checkpoints/piper_act_smoke_10ep_001000"` | 预训练模型目录路径，需包含 `config.json`、`model.safetensors`、`preprocessor.json`、`postprocessor.json` |

#### 硬件/相机参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--port` | `str` | `"can1"` | 从臂（执行策略输出的臂）的 CAN 接口 |
| `--wrist-serial` | `str` | 环境变量 `WRIST_REALSENSE_SERIAL` 或 `"238222076529"` | 腕部 RealSense 相机序列号 |
| `--global-serial` | `str` | 环境变量 `GLOBAL_REALSENSE_SERIAL` 或 `"142122071524"` | 全局 RealSense 相机序列号 |
| `--width` | `int` | `640` | 相机采集宽度 |
| `--height` | `int` | `480` | 相机采集高度 |
| `--fps` | `int` | `15` | 相机采集帧率 |

#### 推理参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--device` | `str` | `"cpu"` | 策略推理设备。可选：`cpu`、`cuda` |
| `--steps` | `int` | `20` | 控制迭代步数 |
| `--control-fps` | `float` | `5.0` | 执行循环频率（Hz） |

#### 安全限幅参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--max-delta` | `float` | `2.0` | 每步关节 1-6 最大归一化动作变化量 |
| `--max-gripper-delta` | `float` | `3.0` | 每步夹爪最大归一化动作变化量 |
| `--disable-gripper` | `flag` | `False` | 保持夹爪当前位置不变（仅测试臂关节时使用） |
| `--ema-alpha` | `float` | `1.0` | 动作 EMA（指数移动平均）平滑系数。`1.0` 禁用平滑；`0.3-0.6` 可减少 ACT chunk 边界抖动。取值范围 (0, 1] |

#### 执行控制

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--execute` | `flag` | `False` | 实际向从臂发送限幅动作。不指定则为干运行 |
| `--calibrate` | `flag` | `False` | 允许 `PiperFollower.connect()` 执行 `parking()` 校准。首次测试建议关闭 |

### 使用示例

```bash
# 干运行验证（推荐首次使用）
python scripts/piper_act_safe_rollout.py \
  --checkpoint /path/to/checkpoints/piper_act_10ep_001000 \
  --device cuda --steps 30

# 实际执行，带 EMA 平滑和安全限幅
python scripts/piper_act_safe_rollout.py \
  --checkpoint /path/to/checkpoints/piper_act_10ep_001000 \
  --device cuda --steps 50 --execute \
  --max-delta 1.5 --ema-alpha 0.5 \
  --port can1

# 仅测试臂关节，禁用夹爪
python scripts/piper_act_safe_rollout.py \
  --checkpoint /path/to/checkpoints --execute \
  --disable-gripper --max-delta 1.0
```

### 安全特性

- **默认干运行**：不指定 `--execute` 则绝不驱动物理臂
- **Delta 限幅**：`--max-delta` 限制单步关节目标变化，`--max-gripper-delta` 限制夹爪变化。动作被 clamp 到归一化范围 [-100, 100]（关节）和 [0, 100]（夹爪）
- **EMA 平滑**：`--ema-alpha` 通过指数移动平均消除 ACT chunk 边界的动作抖动
- **检查点完整性校验**：启动时检查 `config.json`、`model.safetensors`、`preprocessor.json`、`postprocessor.json` 四个文件是否齐全
- **异常退出保护**：`Ctrl-C` 后保持 torque 不禁用（`disable_torque=False`），防止突然掉力
- **范围校验**：`--ema-alpha` 必须在 (0, 1] 范围内，超出则报错退出

---

## A.3 `scripts/piper_reset_to_initial.py` — 机械臂复位到初始位置

### 用途

将一个或多个 PiPER 机械臂移动到统一的初始关节姿态，用于任务间复位。默认目标姿态为全零位姿（6 个关节均为 0 度）。

### 完整参数列表

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--ports` | `nargs="+"` | `["can0", "can1"]` | 需要复位的 CAN 端口列表 |
| `--target-deg` | `str` | `"0.0,0.0,0.0,0.0,0.0,0.0"` | 目标关节角度（度），6 个逗号分隔的浮点数 |
| `--gripper-um` | `int` | `0` | 目标夹爪位置（微米）。0 表示闭合 |
| `--speed` | `int` | `50` | 运动速度百分比 |
| `--hz` | `float` | `10.0` | 控制发送频率 |
| `--duration` | `float` | `5.0` | 命令发送持续时间（秒） |
| `--high-follow` | `flag` | `False` | 启用高跟随模式（`mit_flag=0xAD`） |
| `--master-home-ports` | `nargs="*"` | `["can0"]` | 需要额外发送 `ReqMasterArmMoveToHome(1)` 主臂归位命令的端口列表。设为空值可禁用 |

### 使用示例

```bash
# 双臂同时复位到零位
python scripts/piper_reset_to_initial.py --ports can0 can1

# 仅复位从臂，使用自定义目标位置
python scripts/piper_reset_to_initial.py \
  --ports can1 \
  --target-deg "10.0,-20.0,45.0,0.0,0.0,0.0" \
  --speed 30 --duration 3.0

# 主臂发出归位命令
python scripts/piper_reset_to_initial.py \
  --ports can0 can1 \
  --master-home-ports can0
```

### 关键说明

- 脚本在发送目标指令前会先发送 `ModeCtrl` 和 `MotionCtrl_2` 配置命令，然后尝试最多 20 次使能（`EnableArm`），直到所有臂均已使能
- `--master-home-ports` 中的端口会额外接收 `ReqMasterArmMoveToHome(1)`，这是 PiPER SDK 中主臂的专用归位协议
- 需确保 CAN 接口状态为 UP，否则脚本会抛出错误并给出激活命令
- 复位期间持续指定时长内按指定频率发送关节和夹爪目标指令

---

## A.4 `scripts/realsense_live_view.py` — 双 RealSense 实时预览

### 用途

实时显示一个或两个 Intel RealSense 彩色相机流，支持 Web 浏览器查看和 OpenCV 窗口查看两种模式。可用于验证相机安装位置、对焦和帧率。

### 完整参数列表

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--wrist-serial` | `str` | `"238222076529"` | 腕部相机序列号 |
| `--global-serial` | `str` | `"142122071524"` | 全局相机序列号 |
| `--only` | `str` | `"both"` | 显示哪些相机。可选：`both`、`wrist`、`global` |
| `--width` | `int` | `640` | 彩色流宽度 |
| `--height` | `int` | `480` | 彩色流高度 |
| `--fps` | `int` | `30` | 彩色流帧率 |
| `--timeout-ms` | `int` | `1000` | 取帧超时（毫秒） |
| `--layout` | `str` | `"horizontal"` | 双画面布局。可选：`horizontal`（水平并排）、`vertical`（垂直堆叠） |
| `--viewer` | `str` | `"web"` | 查看模式。可选：`web`（浏览器 MJPEG 流）、`opencv`（本地 OpenCV 窗口） |
| `--host` | `str` | `"127.0.0.1"` | Web 模式下的 HTTP 绑定地址 |
| `--port` | `int` | `8765` | Web 模式下的 HTTP 端口 |
| `--snapshot-dir` | `str` | `"../camera_check/live_view"` | 截图保存目录 |

### 截图功能

- **Web 模式**：访问 `http://<host>:<port>/snapshot` 可触发截图，保存为 `realsense_live_<timestamp>.jpg`
- **OpenCV 模式**：按 `s` 键保存截图，文件名格式同上，保存至 `--snapshot-dir` 指定目录
- 截图输出目录会自动创建（不存在时）

### 使用示例

```bash
# 浏览器预览（默认），打开 http://127.0.0.1:8765
python scripts/realsense_live_view.py

# OpenCV 窗口预览，仅腕部相机
python scripts/realsense_live_view.py --viewer opencv --only wrist

# 自定义分辨率和帧率
python scripts/realsense_live_view.py --width 1280 --height 720 --fps 15

# 垂直布局双画面
python scripts/realsense_live_view.py --layout vertical
```

### 控制快捷键（OpenCV 模式）

| 按键 | 功能 |
|------|------|
| `q` / `Esc` | 退出 |
| `s` | 保存截图 |

### 关键说明

- Web 模式启动一个轻量 HTTP 服务器，页面通过 MJPEG 流推送实时画面，局域网内其他设备也可访问（将 `--host` 设为 `0.0.0.0`）
- 画面左上角显示相机名称和实时帧率（FPS）
- 如果相机暂时无数据，会显示 `waiting for <name>` 提示，并显示上一帧或黑画面

---

## A.5 `scripts/compare_piper_settings.py` — 双臂参数对比

### 用途

只读方式读取并对比两个 CAN 端口上 PiPER 臂的关键设置和状态参数。不使能电机，不发送运动指令。

### 完整参数列表

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--ports` | `nargs=2` | `["can0", "can1"]` | 需要对比的两个 CAN 端口（必须恰好 2 个） |
| `--settle` | `float` | `0.5` | 连接后等待 SDK 数据流稳定的时间（秒） |

### 对比内容

脚本读取并逐行对比以下参数：

| 参数 | 来源 | 说明 |
|------|------|------|
| `status_hz` | `GetArmStatus` | 状态反馈频率 |
| `joint_hz` | `GetArmJointMsgs` | 关节反馈流频率 |
| `ctrl_hz` | `GetArmJointCtrl` | 控制帧频率 |
| `control_mode` | arm_status | 控制模式 |
| `arm_status` | arm_status | 臂状态 |
| `mode_feed` | arm_status | 模式反馈 |
| `teach_status` | arm_status | 示教状态 |
| `motion_status` | arm_status | 运动状态 |
| `error_code` | arm_status | 错误码 |
| `enable_status` | `GetArmEnableStatus` | 使能状态（6 个 bool 值列表） |

此外还单独打印每个臂的详细信息：关节反馈位置（mdeg）、关节控制目标位置（mdeg）、ModeCtrl 设置、CtrlCode151、末端速度加速度参数、碰撞保护等级反馈、夹爪示教器参数反馈。

### 输出格式

```
===== can0 =====
status_hz=100.0 joint_hz=100.0 ctrl_hz=100.0
control_mode=...
...
[gripper_teaching]
...

===== can1 =====
...

===== SUMMARY =====
 OK  status_hz: can0=100.0 | can1=100.0
DIFF joint_hz: can0=100.0 | can1=0.0
 OK  control_mode: ...
...
WARN can1 has no joint feedback stream.
NOTE neither arm currently exposes a control-frame stream.
```

- `OK`：两侧参数一致
- `DIFF`：两侧参数不一致，需关注
- `WARN`：某个臂无关节反馈流（可能未上电、CAN 未激活或连接问题）
- `NOTE`：两个臂均无控制帧暴露（正常，取决于 SDK 当前控制模式）

### 使用示例

```bash
# 对比默认的 can0 和 can1
python scripts/compare_piper_settings.py

# 指定自定义端口
python scripts/compare_piper_settings.py --ports can0 can2

# 增加等待稳定时间
python scripts/compare_piper_settings.py --settle 1.0
```

### 关键说明

- 完全只读：通过 `ArmParamEnquiryAndConfig` 发送参数查询请求（query=0x01, 0x02, 0x04），不改变任何设置
- 适用于故障排查：快速判断两个臂的控制模式、使能状态、反馈流是否一致
- 典型使用场景：遥操作前验证主臂和从臂处于匹配状态；记录故障前的参数快照

---

## A.6 `scripts/identify_piper_can_roles.py` — CAN 端口角色识别

### 用途

只读方式识别每个 CAN 端口上连接的 PiPER 臂是主臂（leader/master）还是从臂（follower/slave）。不发送任何运动指令。

### 工作原理

根据 PiPER SDK 通信协议：

- **主臂（leader/master）**：主要发送控制帧，可通过 `GetArmJointCtrl()` 和 `GetArmGripperCtrl()` 看到活跃的控制消息
- **从臂（follower/slave）**：主要发送反馈帧，可通过 `GetArmJointMsgs()` 和 `GetArmGripperMsgs()` 看到活跃的反馈消息

脚本连接每个 CAN 端口，等待 SDK 数据流积累一段时间，然后分别打印反馈消息和控制消息的完整内容。根据哪个端口有活跃的控制消息即可判断主臂所在。

### 完整参数列表

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--ports` | `nargs="+"` | `["can0", "can1"]` | 需要识别的 CAN 端口列表 |
| `--seconds` | `float` | `2.0` | 每个端口连接后等待数据积累的时间（秒） |

### 使用示例

```bash
# 识别默认的 can0 和 can1 上的角色
python scripts/identify_piper_can_roles.py

# 更长的等待时间以获得更准确的结果
python scripts/identify_piper_can_roles.py --ports can0 can1 --seconds 5.0
```

### 输出格式

```
===== can0 =====
[feedback] joint:
<JointMsg detail>
[feedback] gripper:
<GripperMsg detail>
[control] joint:
<JointCtrl detail>
[control] gripper:
<GripperCtrl detail>
[status]:
<Status detail>

===== can1 =====
[feedback] joint:
...
[control] joint:
...

Interpretation hint: if one port has active control messages and the other
has active feedback messages, the control-heavy port is likely the leader/master
and the feedback-heavy port is likely the follower/slave.
```

### 关键说明

- 完全只读：仅连接和读取，不使能电机，不发送运动命令
- 输出包含底层 SDK 消息对象的 `__repr__` 完整打印，包括消息计数器等详细信息
- 如果某个端口连接失败（CAN 未激活、设备未连接），会打印异常信息但不中断其余端口检测

---

## A.7 `scripts/server_train_act_from_dataset.sh` — 服务端 ACT 训练脚本

### 用途

在训练服务器上，从本地 LeRobot 数据集目录启动 ACT 策略训练。需在 `lerobot` conda 环境下运行，通过环境变量配置训练参数。

### 所有环境变量

#### 路径类

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `PIPER_ROOT` | `/root/autodl-tmp/Piper` | 项目根目录 |
| `LEROBOT_DIR` | `${PIPER_ROOT}/lerobot_piper-piper` | LeRobot 代码目录 |
| `DATASET_DIR` | （必填，无默认值） | 数据集目录，需包含 `data/`、`meta/`、`videos/` |
| `DATASET_NAME` | `DATASET_DIR` 的 `basename` | 数据集名称 |
| `DATASET_REPO_ID` | `local/${DATASET_NAME}` | 数据集仓库 ID |
| `OUTPUT_BASE` | `${PIPER_ROOT}/outputs/train` | 训练输出根目录 |
| `OUTPUT_DIR` | `${OUTPUT_BASE}/${JOB_NAME}_<timestamp>` | 具体输出目录（自动加时间戳） |
| `JOB_NAME` | `piper_act` | 训练任务名称 |

#### 训练超参数

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `POLICY_DEVICE` | `cuda` | 策略训练/推理设备 |
| `BATCH_SIZE` | `8` | 批次大小 |
| `STEPS` | `20000` | 总训练步数 |
| `LOG_FREQ` | `100` | 日志打印频率（步） |
| `SAVE_FREQ` | `5000` | 检查点保存频率（步） |
| `EVAL_FREQ` | `0` | 评估频率（步），0 表示不评估 |
| `NUM_WORKERS` | `4` | 数据加载工作进程数 |
| `CHUNK_SIZE` | `30` | ACT 的 chunk 大小 |
| `N_ACTION_STEPS` | `30` | 每次执行的动作步数 |
| `VIDEO_BACKEND` | `pyav` | 视频解码后端 |
| `PRETRAINED_BACKBONE_WEIGHTS` | `ResNet18_Weights.IMAGENET1K_V1` | 预训练骨干网络权重。设为 `null`/`None`/`none` 则随机初始化 |

### 训练前检查

脚本在启动训练前执行以下检查：

1. `DATASET_DIR` 是否为空（必填）
2. `LEROBOT_DIR` 目录是否存在
3. 数据集目录下 `meta/info.json` 是否存在
4. `lerobot-train` 命令是否可用（验证 conda 环境）
5. 通过 Python 脚本验证数据集可加载，并打印 `num_frames`、`num_episodes`、`fps` 及首帧 `action`、`state`、图像特征 shape

### 使用示例

```bash
# 在服务器上运行
DATASET_DIR=/path/to/dataset/piper_task_v1 \
STEPS=50000 \
BATCH_SIZE=16 \
  bash scripts/server_train_act_from_dataset.sh
```

### 关键说明

- 使用 `set -euo pipefail`，任何命令失败都会中止脚本
- 训练参数通过 `lerobot-train` 命令行传递，wandb 默认关闭（`--wandb.enable=false`）
- 输出目录自动加时间戳以避免覆盖
- 训练完成后打印最新检查点文件列表

---

## A.8 `scripts/package_lerobot_dataset.sh` — 数据集打包脚本

### 用途

将本地 LeRobot 数据集目录打包为 `.tar.gz` 压缩包并计算 SHA-256 校验和，方便传输到训练服务器。

### 环境变量

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `DATASET_DIR` | （必填，无默认值） | 待打包的数据集目录，需包含 `data/`、`meta/` 子目录和 `meta/info.json` |
| `OUT_DIR` | `/home/liyuqi/Documents/Piper/exported_datasets` | 压缩包输出目录 |

### 输入

- `DATASET_DIR` 下的完整数据集目录（包含 `data/`、`meta/` 及可选的 `videos/`）

### 输出

- `<OUT_DIR>/<数据集名>.tar.gz` — 压缩包
- `<OUT_DIR>/<数据集名>.tar.gz.sha256` — SHA-256 校验文件

### 使用示例

```bash
DATASET_DIR=/home/liyuqi/Documents/Piper/datasets/local/piper_task_v1 \
  bash scripts/package_lerobot_dataset.sh

# 自定义输出目录
DATASET_DIR=/path/to/dataset OUT_DIR=/mnt/share/datasets \
  bash scripts/package_lerobot_dataset.sh
```

### 关键说明

- 使用 `set -euo pipefail` 严格错误处理
- 打包前检查 `meta/info.json`、`data/`、`meta/` 是否齐全
- 使用 `readlink -f` 解析绝对路径，避免相对路径问题
- 打包后打印文件大小和 SHA-256 校验值

---

## A.9 `record_with_pc_relay.sh` — 主数据采集编排器

### 用途

这是**整个数据采集流程的核心编排脚本**。它同时启动 PC 中继遥操作进程（`piper_pc_relay_teleop.py`）和 LeRobot 录制进程（`lerobot-record`），实现"主臂遥操作 + 从臂状态记录"的完整数据采集流水线。

### 系统拓扑

```
主臂   -> USB-CAN A -> can0 -> [PC 中继] -> can1 -> 从臂
                                   ↑
                              lerobot-record 读取从臂观测 + 相机图像
                              → 存储为 LeRobot 数据集（从臂状态=action）
```

中继进程将主臂指令发送到从臂驱动运动；`lerobot-record` 仅读取从臂的观测和相机数据，并将从臂状态作为动作（action）存储。

### 所有环境变量

#### 路径类

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `PIPER_ROOT` | `/home/liyuqi/Documents/Piper` | 项目根目录 |
| `LEROBOT_DIR` | `${PIPER_ROOT}/lerobot_piper-piper` | LeRobot 代码目录 |
| `CONDA_ENV` | `lerobot` | Conda 环境名称 |
| `VOICE_PROMPT_DIR` | `${LEROBOT_DIR}/audio_prompts/en-US-AriaNeural` | 语音提示音频目录 |

#### 相机配置

| 环境变量 | 说明 |
|----------|------|
| `ROBOT_CAMERAS` | 相机配置 YAML 字符串。为空时自动尝试从 `camera_config_realsense.env` 读取 |
| `WRIST_REALSENSE_SERIAL` | 腕部 RealSense 序列号（由 `camera_config_realsense.env` 提供） |
| `GLOBAL_REALSENSE_SERIAL` | 全局 RealSense 序列号（由 `camera_config_realsense.env` 提供） |

#### 中继参数

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `LEADER_CAN` | `can0` | 主臂 CAN 接口 |
| `FOLLOWER_CAN` | `can1` | 从臂 CAN 接口 |
| `RELAY_HZ` | `20` | 中继循环频率 |
| `RELAY_SPEED` | `100` | 从臂运动速度百分比 |
| `RELAY_MAX_STEP_DEG` | `10` | 最大关节步进量（度） |
| `RELAY_SOURCE` | `control` | 主臂数据源（`control`/`feedback`/`auto`） |
| `RELAY_INCLUDE_GRIPPER` | `1` | 是否中继夹爪（`1`=是） |
| `RELAY_HIGH_FOLLOW` | `1` | 高跟随模式（`1`=启用） |
| `RELAY_MODE_COMMAND_PERIOD` | `1.0` | 模式命令重复间隔（秒） |
| `RELAY_GRIPPER_HZ` | `5` | 夹爪指令频率 |
| `RELAY_PRINT_PERIOD` | `0` | 状态打印间隔（0=禁用） |
| `RELAY_PAUSE_FILE` | `/tmp/piper_pc_relay.pause` | 暂停文件路径 |

#### 数据集录制参数

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `DATASET_BASE_DIR` / `DATASET_ROOT` | `${PIPER_ROOT}/datasets` | 数据集存储根目录 |
| `DATASET_NAME` | `piper_pc_relay_smoke` | 数据集名称 |
| `DATASET_FPS` | `30` | 录制帧率 |
| `NUM_EPISODES` | `5` | 录制轮次（episode）数 |
| `EPISODE_TIME_S` | `20` | 每轮时长（秒） |
| `RESET_TIME_S` | `10` | 轮间复位时间（秒） |
| `TASK` | `"Teleoperate the follower PiPER with the leader PiPER"` | 任务描述文本 |
| `DISPLAY_DATA` | `false` | 是否显示录制数据 |
| `PUSH_TO_HUB` | `false` | 是否推送到 Hugging Face Hub |
| `PLAY_SOUNDS` | `false` | 是否播放提示音 |
| `HF_USER` | `hf auth whoami` 结果或 `local` | Hugging Face 用户名 |
| `DATASET_REPO_ID` | `${HF_USER}/${DATASET_NAME}` | 数据集仓库 ID |
| `DATASET_OUTPUT_ROOT` | `${DATASET_BASE_DIR}/${DATASET_REPO_ID}` | 实际输出目录 |
| `AUTO_SUFFIX_DATASET_ROOT` | `1` | 自动在已存在的输出目录名后加时间戳 |

#### 行为控制

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `PHASE_BEEP` | `true` | 是否播放阶段提示音 |
| `MANUAL_STEP` | `true` | 手动步进模式（需按键确认每轮开始） |
| `AUTO_RESET_ARMS` | `true` | 每轮间自动复位从臂 |
| `AUTO_RESET_LEADER` | `false` | 每轮间是否也复位主臂 |

#### 复位参数

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `RESET_SPEED` | `50` | 复位运动速度 |
| `RESET_DURATION` | `5` | 复位命令持续时间（秒） |
| `RESET_HZ` | `10` | 复位控制频率 |
| `RESET_TARGET_DEG` | `0,0,0,0,0,0` | 复位目标关节角度 |

### 执行流程

```
1. 加载 conda 环境 (lerobot)
2. 读取 camera_config_realsense.env（如果存在）解析相机序列号
3. 验证相机配置中无占位符 (WRIST_SERIAL / GLOBAL_SERIAL)
4. 将相机帧率统一为 DATASET_FPS
5. 设置 trap 清理函数（退出时自动杀 relay 进程、删 pause 文件）
6. 启动 piper_pc_relay_teleop.py 后台进程（带 --execute）
7. 等待 2 秒让中继稳定
8. 配置 LEROBOT_PHASE_BEEP（阶段提示音）
9. 配置 LEROBOT_RESET_COMMAND（自动复位命令，可选）
10. 启动 lerobot-record 录制进程
11. 录制完毕后进入 cleanup 清理
```

### 使用示例

```bash
# 无相机冒烟测试
cd /home/liyuqi/Documents/Piper/lerobot_piper-piper
bash record_with_pc_relay.sh

# 带双 RealSense 相机的真实数据采集
ROBOT_CAMERAS='{ wrist: {type: realsense, serial: WRIST_SERIAL, width: 640, height: 480, fps: 30}, global: {type: realsense, serial: GLOBAL_SERIAL, width: 640, height: 480, fps: 30} }' \
NUM_EPISODES=10 EPISODE_TIME_S=60 \
RELAY_SPEED=80 RELAY_HZ=30 \
  bash record_with_pc_relay.sh

# 自定义数据集名称和低位关注参数
DATASET_NAME=piper_pour_water_v2 \
NUM_EPISODES=30 EPISODE_TIME_S=30 \
RELAY_MAX_STEP_DEG=5.0 \
  bash record_with_pc_relay.sh
```

### 关键说明

- **安全机制**：退出时自动执行 cleanup，发送 SIGTERM 给中继进程（中继进程收到后发送待机命令并断开连接），删除 pause 文件
- **复位机制**：通过设置 `LEROBOT_RESET_COMMAND` 和 `LEROBOT_RELAY_PAUSE_FILE` 环境变量，让 `lerobot-record` 在轮间自动暂停中继、复位机械臂、再恢复中继
- **相机占位符检查**：如果 `ROBOT_CAMERAS` 中仍包含 `WRIST_SERIAL` 或 `GLOBAL_SERIAL` 占位符字面量，脚本会中止并提示先配置真实序列号
- **键盘控制**（在 `lerobot-record` 录制过程中）：右键 = 完成当前 episode 进入复位阶段；左键 = 重新录制当前 episode；Esc = 停止录制

---

## A.10 根目录编号 Shell 脚本（`1__init_can.sh` ~ `8__run_client.sh`）

这些脚本是流水线的快速入口，按编号顺序执行可实现从 CAN 初始化到策略部署的完整流程。每个脚本均为单行或数行命令的快捷封装。

### `1__init_can.sh` — CAN 总线初始化

**角色**：激活 CAN0 总线，波特率 1 Mbps。

```bash
bash ~/piper_sdk/piper_sdk/can_activate.sh can0 1000000
```

**使用时机**：每次系统启动后、连接机械臂之前执行。如果使用双臂（can0 + can1），需额外执行 `can1` 的初始化。

---

### `2__find_camera.sh` — 查找可用相机

**角色**：列出系统中所有可用的相机设备，并打开文件管理器查看之前捕获的图像。

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate lerobot
python ~/lerobot/src/lerobot/find_cameras.py
nautilus ~/lerobot/outputs/captured_images
```

**使用时机**：连接新相机后验证系统是否识别；确认相机索引号。

---

### `3__set_camera.sh` — 设置相机属性

**角色**：调整相机曝光、增益等属性参数。

```bash
python ./src/lerobot/camera_prop.py
```

**使用时机**：调整相机参数以获得最佳图像质量。通常在 `2__find_camera.sh` 确认相机可用之后执行。

---

### `4__record.sh` — 录制数据集（独立采集）

**角色**：使用 `lerobot-record` 录制数据集，采用**独立采集**模式（无 PC 中继遥操作）。机械臂由人手动操作（直接示教），lerobot 读取从臂状态和相机图像作为数据。

关键参数示例（脚本中的硬编码值）：
- 机器人类型：`piper_follower`，端口 `can0`
- 2 个 OpenCV 相机（`top` 索引 0，`left` 索引 4），640x480@30fps
- 50 个 episode

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate lerobot
HF_USER=$(hf auth whoami | head -n 1)
lerobot-record \
  --robot.type=piper_follower --robot.port=can0 \
  --robot.cameras="{ ... }" --robot.id=black \
  --dataset.num_episodes=50 \
  --dataset.single_task="Grab the yellow car and put in the box" \
  --display_data=true \
  --dataset.repo_id=${HF_USER}/piper_pick_yellow_car
```

**使用时机**：手动示教数据采集模式，适用于不需要主从遥操作的简单任务。

---

### `5__replay.sh` — 数据集重放

**角色**：从已有数据集中回放录制的动作到机械臂，用于验证数据集质量。

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate lerobot
HF_USER=$(hf auth whoami | head -n 1)
lerobot-replay \
  --robot.type=piper_follower --robot.port=can0 \
  --robot.id=black \
  --dataset.repo_id=${HF_USER}/piper_pick_yellow_car \
  --dataset.episode=0
```

**使用时机**：数据采集后验证动作序列是否合理；演示训练数据中的标准轨迹。

---

### `6__train.sh` — 训练策略（SmolVLA）

**角色**：在服务器上使用 `lerobot-train` 训练 SmolVLA 策略。

关键参数（脚本中硬编码）：
- 策略类型：`smolvla`
- 设备：`cuda`，批次大小 64
- 训练步数 20000，评估频率每 5000 步

```bash
lerobot-train \
  --policy.device=cuda --policy.type=smolvla \
  --dataset.repo_id=wego-hansu/piper_pick_yellow_car \
  --dataset.video_backend=pyav --batch_size=64 \
  --steps=20000 --eval_freq=5000 \
  --output_dir=outputs/train/piper_smolvla_pick_yellow_cars_new \
  --job_name=piper_smolvla_yellow_car --wandb.enable=false
```

**使用时机**：数据集准备好后，在 GPU 服务器上启动训练。注意这是 SmolVLA 训练模板，需替换 `dataset.repo_id` 和 `output_dir` 为实际值。

---

### `7__run_server.sh` — 启动策略服务器

**角色**：启动 LeRobot 的 `policy_server.py`，在本地端口提供策略推理服务。

```bash
python src/lerobot/scripts/server/policy_server.py --host=127.0.0.1 --port=8080
```

**使用时机**：训练完成后，在部署机械臂的机器上启动策略服务器，等待 `robot_client` 连接。

---

### `8__run_client.sh` — 启动机器人客户端

**角色**：启动 LeRobot 的 `robot_client.py`，连接策略服务器并驱动机器人执行策略推理后的动作。

关键参数示例：
- 策略服务器地址：`127.0.0.1:8080`
- 机器人类型：`piper_follower`，端口 `can0`
- 2 个 OpenCV 相机，任务描述，策略类型 `smolvla`
- 策略路径：本地训练的检查点目录

```bash
python src/lerobot/scripts/server/robot_client.py \
  --server_address=127.0.0.1:8080 \
  --robot.type=piper_follower --robot.port=can0 \
  --robot.id=black --robot.cameras="{ ... }" \
  --task="Grasp the object and put it in the bin" \
  --policy_type=smolvla \
  --pretrained_name_or_path=outputs/train/.../checkpoints/020000/pretrained_model \
  --policy_device=cuda --actions_per_chunk=50 \
  --chunk_size_threshold=0.5 \
  --aggregate_fn_name=weighted_average \
  --debug_visualize_queue_size=True
```

**使用时机**：策略服务器运行后，在连接机械臂的机器上启动客户端执行策略部署。注意需替换 `pretrained_name_or_path` 为实际检查点路径。

---

### 编号脚本使用流程概览

```
[开发机]                              [服务器]
1__init_can.sh    (CAN 激活)
2__find_camera.sh (查找相机)
3__set_camera.sh  (配置相机)
4__record.sh      (录制数据集)
  或
record_with_pc_relay.sh (中继采集)
     ↓
5__replay.sh      (数据回放验证)
     ↓
打包传输数据集到服务器 →
                           6__train.sh  (训练策略)
                                ↓
                           下载检查点到开发机
     ↓
7__run_server.sh  (启动策略服务器)
8__run_client.sh  (策略部署)
```

---

> **文档版本**：附录 A v1.0
> **最后更新**：2026-06-22
> **对应代码库**：`lerobot_piper-piper`
