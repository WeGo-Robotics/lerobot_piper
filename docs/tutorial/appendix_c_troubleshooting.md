# 附录 C 故障排除指南

本附录汇总了 PiPER + LeRobot 教程完整流程中可能遇到的常见问题，按故障类别分组，每组提供症状识别、诊断命令和解决方案。

---

## C.1 CAN 总线问题

### C.1.1 CAN 端口未出现

**症状**：执行 `ifconfig -a` 或 `ip link show` 看不到任何 `can0` 接口。

**诊断命令**：
```bash
# 检查 USB 设备是否被识别
lsusb | grep -i "can\|gs_usb\|peak\|kvaser\|socketcan"

# 检查 dmesg 中的 USB 设备日志
dmesg | tail -30 | grep -i "usb\|can"

# 检查是否有待加载的内核模块
lsmod | grep can

# 查看所有网络接口
ip link show
```

**解决方案**：
1. **USB CAN 适配器未识别**：重新拔插 USB 连接线，尝试不同 USB 端口（优先使用 USB 2.0 端口，部分 USB 3.0 控制器存在兼容性问题）。
2. **驱动未加载**：对于 gs_usb 适配器（如 CANable），执行：
   ```bash
   sudo modprobe gs_usb
   ```
   对于 MCP251x SPI 转 CAN：执行 `sudo modprobe mcp251x`。
3. **固件问题**：部分 CANable 适配器需要刷入 candleLight 固件才能被 gs_usb 驱动识别。参考 [candleLight_fw](https://github.com/candle-usb/candleLight_fw)。
4. **内核版本过低**：确保 Linux 内核版本 >= 5.4（`uname -r` 检查），旧内核可能缺少 SocketCAN 支持。

### C.1.2 CAN 端口无法启动 (bring up)

**症状**：执行 `sudo ip link set can0 up type can bitrate 1000000` 时报错 `Cannot find device "can0"` 或 `RTNETLINK answers: Operation not permitted`。

**诊断命令**：
```bash
# 检查 CAN 设备是否在 /sys 中注册
ls /sys/class/net/ | grep can

# 查看内核 CAN 日志
dmesg | grep -i can | tail -20

# 检查当前 CAN 状态
ip -details link show can0 2>&1
```

**解决方案**：
1. **设备名不对**：检查实际的接口名称（可能是 `can1`、`slcan0` 等）。执行 `ip link show | grep -E "^[0-9]+:"` 查看全部网络接口。
2. **设备未初始化**：
   ```bash
   # 先设置类型和比特率再启动
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set can0 up
   ```
3. **终端电阻未接**：确认 CAN 总线两端各有一个 120 Ω 终端电阻。缺少终端电阻会导致无法正常通信甚至控制器报错。
4. **权限不足**：确保使用 `sudo`，或将用户加入 `dialout` 组：`sudo usermod -aG dialout $USER`。

### C.1.3 CAN 端口已启动但无数据

**症状**：`candump can0` 没有任何输出，或只偶尔出现零星报文。

**诊断命令**：
```bash
# 实时监测 CAN 数据
candump can0

# 查看接口统计信息（错误计数器）
ip -details -statistics link show can0

# 发送测试帧检查 CAN 回路是否正常
cansend can0 123#DEADBEEF

# 检查总线负载
canbusload can0@1000000
```

**解决方案**：
1. **波特率不匹配**：PiPER 机械臂 CAN 总线默认为 **1 Mbps (1000000)**。确认与机械臂固件波特率一致。
2. **机械臂未上电**：PiPER 机械臂需外部供电（通常 24V DC），仅 USB 连接不足以驱动电机控制器。
3. **终端电阻问题**：缺少终端电阻会导致信号反射，测量总线 CAN_H 和 CAN_L 间静态电阻应约为 60 Ω（两个 120 Ω 并联）。
4. **CAN 控制器 bus-off**：如果 `ip link show can0` 显示 `ERROR-PASSIVE` 或 `BUS-OFF`（错误计数器 txerr/rxerr >= 96/128），需要重启：
   ```bash
   sudo ip link set can0 down
   sudo ip link set can0 up type can bitrate 1000000
   ```
5. **电机未使能**：机械臂上电后不会主动发送数据，需要先通过 0x471 指令使能电机，才会收到 0x2A5~0x2A8 的反馈数据。

---

## C.2 机械臂连接问题

### C.2.1 SDK 无法连接机械臂

**症状**：运行 `detect_arm.py` 或任何 SDK 代码时报错：`ConnectionError`、`TimeoutError`、`Cannot connect to arm` 或程序卡死无响应。

**诊断命令**：
```bash
# 确认 CAN 端口工作正常
candump can0 -n 5

# 检查 Python 环境和 SDK 安装
python3 -c "import piper_sdk; print(piper_sdk.__version__)"

# 运行 SDK 自带的探测脚本
python3 -m piper_sdk.demo.detect_arm

# 检查是否有进程占用 CAN 接口
sudo lsof /dev/ttyUSB* 2>/dev/null
```

**解决方案**：
1. **CAN 端口未启动**：按 C.1 节检查 CAN 连接。
2. **机械臂供电不足**：PiPER 机械臂建议使用 24V / 10A 以上的直流电源，电压不足时控制器无法正常启动。
3. **SDK 版本不匹配**：
   ```bash
   pip show piper_sdk
   # 确认版本号与机械臂固件版本兼容
   ```
4. **CAN 接口被其他程序占用**：确保同一时间只有一个进程使用 CAN 接口（SocketCAN 支持多进程同时读写，但某些 SDK 实现可能要求独占）。
5. **连接超时**：检查 SDK 初始化代码中的 `can_interface` 参数是否正确：
   ```python
   piper = Piper(interface="can0", bitrate=1000000)
   ```

### C.2.2 扭矩无法使能

**症状**：调用 `piper.EnableArm(7)`（或等效的使能函数）后，电机无响应，关节仍处于自由状态。

**诊断命令**：
```bash
# 直接发送使能 CAN 指令测试
cansend can0 471#FF02000000000000

# 查看反馈数据确认使能状态
candump can0,2A1:7FF  # 监听状态反馈

# 检查是否有错误/警告码
python3 -c "
import piper_sdk
p = piper_sdk.Piper()
print(p.GetArmStatus())
"
```

**解决方案**：
1. **使能顺序**：PiPER 要求先设置控制模式 (0x151)，再使能电机 (0x471)：
   ```python
   # 正确顺序
   piper.MotionCtrl_2(ctrl_mode=0x01, move_mode=0x01, move_spd_rate_ctrl=50)
   piper.EnableArm(7)  # 使能全部电机
   ```
2. **急停状态未清除**：如果之前触发过急停，需要先解除：
   ```python
   piper.MotionCtrl_1(emergency_stop=0x02)  # 恢复急停
   ```
3. **软限位触发**：如果关节当前位置超出软限位，部分电机拒绝使能。可以先关闭软限位测试：
   ```python
   piper.init_soft_joint_limit_on()  # 开启软限位保护
   piper.init_soft_joint_limit_off() # 关闭软限位保护（调试用）
   ```
4. **硬件错误锁存**：通过查询清除错误：
   ```python
   # 使用 0x475 清除关节错误
   piper.JointConfig(joint_motor_num=7, clear_joint_err=0xAE)
   ```

### C.2.3 机械臂不响应控制指令

**症状**：使能成功、发送关节控制指令后，机械臂不运动。

**诊断命令**：
```bash
# 检查实际发送的控制指令
candump can0 | grep "155\|156\|157"

# 检查反馈角度是否在变化
python3 -c "
import piper_sdk
p = piper_sdk.Piper()
states = p.GetArmJointMsgs()
print(states)  # 查看反馈角度
"
```

**解决方案**：
1. **运动速度设置过低**：0x151 Byte 2 速度百分比过低会导致运动极慢。设为 50~100。
2. **控制模式未正确设置**：确认 `ctrl_mode = 0x01` (CAN 指令控制模式)。
3. **目标角度超出限位**：发送的目标角度超出软限位范围时，机械臂拒绝执行。可以查询当前限位：
   ```python
   piper.SearchAllMotorAngleLimitMaxSpd()
   limits = piper.GetAllMotorAngleLimitMaxSpd()
   # 比较目标角度是否在 min_angle_limit 和 max_angle_limit 之间
   ```
4. **指令编码错误**：角度以 0.001° 为单位编码为 int32。0.151 弧度 ≈ 9°，编码值约为 9000。如果编码值小于 10（即目标角度小于 0.01°），可能无可见运动。
5. **电机处于失能状态**：调用 `EnableArm(7, 0x02)` 重新使能。

---

## C.3 摄像头问题

### C.3.1 摄像头未检测到

**症状**：LeRobot 报告 `No camera found`、`Cannot open video device`。

**诊断命令**：
```bash
# 列出所有视频设备
ls /dev/video*

# 检查 USB 摄像头
lsusb | grep -i "camera\|webcam\|imaging"

# 使用 OpenCV 测试打开摄像头
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
print('Opened:', cap.isOpened())
cap.release()
"

# 使用 v4l2 查看设备
v4l2-ctl --list-devices

# 使用 lerobot 内置工具
lerobot-find-cameras
```

**解决方案**：
1. **设备索引不对**：`/dev/video0` 不一定是实际的摄像头，可能被内置摄像头或虚拟设备占用。用 `v4l2-ctl --list-devices` 确认正确索引。
2. **权限不足**：将用户加入 `video` 组：
   ```bash
   sudo usermod -aG video $USER
   # 重新登录或执行 newgrp video
   ```
3. **USB 供电不足**：摄像头通过无源 USB Hub 连接可能供电不足。直接连接到主机 USB 端口。
4. **UVC 驱动问题**：
   ```bash
   sudo modprobe uvcvideo
   dmesg | grep uvcvideo
   ```
5. **LeRobot 配置**：确认录制命令中的 `--robot.cameras` 参数正确：
   ```bash
   lerobot-record --robot.type piper_follower --robot.cameras '0' '1'
   ```

### C.3.2 摄像头超时

**症状**：录制时频繁出现 `Camera timeout` 警告，或帧获取延迟很大。

**诊断命令**：
```bash
# 测试摄像头 FPS
python3 -c "
import cv2, time
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 30)
t0 = time.time()
for i in range(100):
    ret, frame = cap.read()
print(f'Actual FPS: {100 / (time.time() - t0):.1f}')
cap.release()
"
```

**解决方案**：
1. **USB 带宽饱和**：多个摄像头共用同一 USB 控制器的带宽。将摄像头分散到不同的 USB 端口（物理上位于不同 USB Host Controller 上）。查看 USB 树状拓扑：
   ```bash
   lsusb -t
   ```
2. **降低分辨率**：在 LeRobot 配置中降低摄像头分辨率（如 640x480）：
   ```python
   # 在 robot config 中设置
   CameraConfig(width=640, height=480, fps=30)
   ```
3. **关闭自动曝光/自动白平衡**：
   ```python
   cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # 手动曝光
   cap.set(cv2.CAP_PROP_EXPOSURE, -6)       # 设定曝光值
   ```

### C.3.3 帧率过低 (Low FPS)

**症状**：录制显示实际 FPS 远低于目标 FPS（如目标 30 FPS，实际仅 5-10 FPS）。

**诊断命令**：
```bash
# 使用 v4l2 查看摄像头支持的格式和帧率
v4l2-ctl -d /dev/video0 --list-formats-ext

# LeRobot 录制时观察 FPS 日志
lerobot-record --robot.type piper_follower --robot.cameras '0' 2>&1 | grep -i fps
```

**解决方案**：
1. **MJPEG vs YUYV**：MJPEG 格式通常可获得更高 FPS。在配置中指定：
   ```python
   CameraConfig(fps=30, color_mode='rgb', pixel_format='MJPEG')
   ```
2. **系统负载过高**：录制过程中避免运行其他计算密集型任务。
3. **硬件瓶颈**：USB 2.0 摄像头理论最大带宽 480 Mbps，单个 1080p@30FPS YUYV 流约占 140 MB/s=1120 Mbps，超出 USB 2.0 带宽。此时必须降低分辨率或使用 MJPEG 压缩格式。

---

## C.4 录制问题

### C.4.1 帧率下降

**症状**：录制开始时 FPS 正常，几分钟后出现明显下降。

**诊断命令**：
```bash
# 监控磁盘 I/O
iostat -x 1

# 检查磁盘剩余空间
df -h

# 监控 CPU 使用率
htop

# 检查录制文件大小增长速率
watch -n 1 'du -sh ~/.cache/lerobot/'
```

**解决方案**：
1. **磁盘 I/O 瓶颈**：录制数据写入速度不足。将数据目录迁移到 SSD（固态硬盘）：
   ```bash
   export LEROBOT_HOME=/path/to/ssd/lerobot_data
   ```
2. **内存缓冲区不足**：录制时 Python 进程内存使用过高触发 GC，导致主循环阻塞。增加系统可用内存，或减少同时录制的摄像头数量。
3. **编码/压缩开销**：如果录像编码格式设为原始 YUYV，磁盘写入量大，考虑使用 MJPEG 或降低分辨率。
4. **CPU 过热降频**：检查 CPU 温度 (`sensors`)，如果过热触发了降频保护，改善散热条件。

### C.4.2 数据帧丢失/帧不完整

**症状**：录制完成后检查 HDF5 文件，时间戳不均匀或关节角度数据出现跳变/缺失。

**诊断命令**：
```bash
# 检查 HDF5 数据集完整性
python3 -c "
import h5py, numpy as np
f = h5py.File('path/to/episode_0.h5', 'r')
for key in f['data'].keys():
    arr = f['data'][key][:]
    print(f'{key}: shape={arr.shape}, NaN={np.isnan(arr).sum()}, Zero={np.sum(arr==0)}')
f.close()
"

# 检查时间戳间隔
python3 -c "
import h5py, numpy as np
f = h5py.File('path/to/episode_0.h5', 'r')
ts = f['observation/timestamp'][:]
diffs = np.diff(ts.squeeze())
print(f'Mean dt={diffs.mean():.3f}s, Std={diffs.std():.3f}s, Max gap={diffs.max():.3f}s')
f.close()
"
```

**解决方案**：
1. **CAN 数据丢包**：如果 CAN 总线上同时有大量报文，可能丢包。检查 `ip -details -statistics link show can0` 中的 `dropped` 计数。如持续增长，说明内核 SocketCAN 缓冲区不足：
   ```bash
   sudo ip link set can0 txqueuelen 1000
   # 或增大内核接收缓冲区（需要重新加载 can 模块）
   ```
2. **ROS/中间件竞争**：如果同时运行 ROS 节点与 LeRobot，确保它们使用不同的 CAN 接口或同一接口的 SocketCAN 多进程模式。
3. **线程同步问题**：LeRobot 数据采集使用多线程（图像采集线程 + 主控制线程）。如果关节读取在图像线程之后但时间戳取自不同的时钟，会导致时间戳不一致。可考虑使用统一的 monotonic clock。

### C.4.3 磁盘空间不足

**症状**：录制中途报错 `No space left on device` 或 `OSError: [Errno 28]`。

**诊断命令**：
```bash
# 查看磁盘使用情况
df -h

# 查看 LeRobot 数据目录大小
du -sh ~/.cache/lerobot/

# 预估一次录制的磁盘占用
# 2 cameras × 640×480×3 bytes × 30 FPS × 60s ≈ 3.3 GB (原始) / 0.5 GB (MJPEG)
```

**解决方案**：
1. **清理旧数据**：
   ```bash
   rm -rf ~/.cache/lerobot/old_episodes/
   ```
2. **更改数据存储路径**到更大磁盘：
   ```bash
   export LEROBOT_HOME=/mnt/large_disk/lerobot_data
   mkdir -p $LEROBOT_HOME
   ```
3. **减小录制参数**：
   - 降低摄像头分辨率
   - 降低目标帧率
   - 减少录制时长
4. **使用视频压缩格式**：MJPEG 比原始 YUYV 省磁盘空间约 5~10 倍。

---

## C.5 主从跟随（Relay）问题

### C.5.1 从机械臂不跟随运动

**症状**：Leader 臂可自由拖动，但 Follower 臂保持静止。

**诊断命令**：
```bash
# 检查 Leader 臂是否在发送数据
candump can0  # 检查 0x2A1~0x2A8 是否有持续反馈

# 检查 Follower 臂是否收到控制指令
candump can1 | grep "155\|156\|157"

# 确认 Follower 臂使能状态
python3 -c "
import piper_sdk
p = piper_sdk.Piper(interface='can1')
p.SearchAllMotorMaxAngleSpdAccLimit()
status = p.GetArmStatus()
print(status)
"
```

**解决方案**：
1. **Follower 臂未使能**：确保 Follower 臂的 EnableArm 已调用且成功。
2. **CAN 接口混淆**：Leader 和 Follower 必须连接到不同的 CAN 接口（如 `can0` 和 `can1`）。确认物理连接和 SDK 初始化参数一致。
3. **主从模式未设置**：在 PC Relay 模式下需要正确配置主从关系。确保两支机械臂分别发送了正确的主从配置指令：
   - Leader：`0x470 [0xFA]`（示教输入臂）
   - Follower：`0x470 [0xFC]`（运动输出臂）
4. **ID 偏移配置错误**：如果两支臂共用一个 CAN 总线（不推荐），必须设置 ID 偏移以区分数据。参见附录 B.7.3。
5. **关节映射错误**：确认 Leader 的 J1~J6 关节角度被正确映射到 Follower 的对应关节。某些安装方式下需要做角度取反。
6. **软件逻辑**：检查 PC Relay 脚本的循环是否在正常运行（打印调试信息/日志）。

### C.5.2 Follower 臂运动抖动严重

**症状**：Follower 臂在跟随 Leader 时出现明显的抖动/震颤/高频振动。

**诊断命令**：
```bash
# 观察反馈角度的时间变化
candump can1 | grep "2A5\|2A6\|2A7" | head -50

# 检查控制帧的发送频率
timeout 5 candump can1 | grep "155\|156\|157" | wc -l
```

**解决方案**：
1. **控制频率不稳定**：控制循环的频率不均导致抖动。使用固定频率的控制循环（如 `time.perf_counter()` + `sleep` 或 RT 线程）：
   ```python
   import time
   period = 1.0 / 100  # 100 Hz
   while True:
       t_start = time.perf_counter()
       # 发送控制指令
       elapsed = time.perf_counter() - t_start
       time.sleep(max(0, period - elapsed))
   ```
2. **角度噪声放大**：Leader 臂反馈的角度本身有少量噪声（~0.1°），直接映射会放大为抖动。在控制回路中加入低通滤波：
   ```python
   alpha = 0.2  # 滤波系数 (0~1)
   filtered_angle = alpha * raw_angle + (1 - alpha) * filtered_angle
   ```
3. **PID 参数过激**：如果使用额外 PID 控制，P 增益过大会导致超调和振荡。降低 Kp 值。
4. **加速度限制过低**：机械臂默认的加速度限制可能过高，导致速度突变。通过 0x475 适度降低最大加速度。

### C.5.3 Follower 臂运动方向与预期相反

**症状**：Leader 臂向上抬时 Follower 向下压，或者旋转方向相反。

**诊断命令**：
```bash
# 对比 Leader 和 Follower 的同关节角度
python3 -c "
import piper_sdk
leader = piper_sdk.Piper(interface='can0')
follower = piper_sdk.Piper(interface='can1')
l = leader.GetArmJointMsgs()
f = follower.GetArmJointMsgs()
for i in range(6):
    print(f'J{i+1}: Leader={getattr(l.joint_state, f\"joint_{i+1}\")*0.001:.1f}°, '
          f'Follower={getattr(f.joint_state, f\"joint_{i+1}\")*0.001:.1f}°')
"
```

**解决方案**：
1. **安装姿态差异**：两支臂的物理安装方向不同（如一支水平正装，一支倒装）。检查 `installation_pos` (0x151 Byte 5) 设置，并在软件中对需要翻转的关节做取反处理。
2. **关节编号不一致**：确认两支臂的 J1~J6 关节物理定义一致。必要时在代码中建立关节映射表。
3. **解决方案代码示例**：
   ```python
   # 关节方向取反示例
   joint_offsets = [+1, +1, -1, +1, +1, +1]  # 1 表示同向，-1 表示反向
   for i, direction in enumerate(joint_offsets):
       follower_cmd[i] = leader_pos[i] * direction
   ```

---

## C.6 训练问题

### C.6.1 显存不足 (OOM)

**症状**：训练启动后报错 `RuntimeError: CUDA out of memory`。

**诊断命令**：
```bash
# 查看 GPU 显存使用
nvidia-smi

# 训练启动前确认显存余量
python3 -c "import torch; print(f'Total: {torch.cuda.get_device_properties(0).total_mem/1e9:.1f} GB, '
                                  f'Free: {torch.cuda.memory_reserved(0)/1e9:.1f} GB')"
```

**解决方案**：
1. **减小 batch size**：在训练配置中将 `batch_size` 从默认值减半或更小。
2. **减少图像分辨率**：训练所用图像分辨率直接决定显存占用量。将分辨率从 640x480 降至 320x240 或 224x224。
3. **减少上下文长度**：如果使用 ACT 或类似序列模型，减小 `chunk_size` 或 `horizon` 参数。
4. **使用混合精度训练**：启用 `torch.amp` (自动混合精度)：
   ```python
   # 在配置中添加
   use_amp = true
   ```
5. **梯度累积**：增大 `gradient_accumulation_steps` 以在较小 batch size 下模拟较大 batch size。
6. **减小模型容量**：减少 transformer 层数、注意力头数或隐藏维度。

### C.6.2 损失不下降

**症状**：训练了多个 epoch，损失曲线基本持平或只有微小下降。

**诊断命令**：
```bash
# 查看训练曲线
tensorboard --logdir outputs/train/

# 检查数据分布
python3 -c "
import h5py, numpy as np
f = h5py.File('episode_0.h5', 'r')
actions = f['action'][:]
print(f'Action mean: {actions.mean(axis=0)}')
print(f'Action std: {actions.std(axis=0)}')
print(f'Action range: {actions.min(axis=0)} to {actions.max(axis=0)}')
f.close()
"
```

**解决方案**：
1. **数据量不足**：模仿学习需要大量示教数据。单任务至少 50 条 episode，复杂任务可能需要 100 条以上。
2. **数据质量差**：示教动作不一致、抖动、停顿过长。重新录制更流畅的示教数据。
3. **学习率设置不当**：尝试减小或增大学习率一个数量级（默认 1e-4，尝试 5e-5 或 5e-4）。
4. **归一化问题**：确认动作和观测的归一化统计量正确（mean/std）。LeRobot 会自动计算并使用数据集统计量。
5. **任务难度与模型容量不匹配**：简单的策略用大模型会过拟合，复杂任务用小模型则欠拟合。调整模型大小。
6. **检查数据加载**：确认摄像头图像和关节角度的时间对齐正确，没有偏移。

### C.6.3 检查点无法加载

**症状**：训练中断后恢复时报错 `KeyError: 'model_state_dict'` 或 `Checkpoint corrupted`。

**诊断命令**：
```bash
# 列出 checkpoint 文件
ls -lh outputs/train/*/checkpoints/

# 检查 checkpoint 文件完整性
python3 -c "
import torch
ckpt = torch.load('outputs/train/.../checkpoints/ckpt_1000.pth', map_location='cpu')
print('Keys:', list(ckpt.keys()))
print('Epoch:', ckpt.get('epoch', 'N/A'))
print('Loss:', ckpt.get('loss', 'N/A'))
"
```

**解决方案**：
1. **版本不兼容**：checkpoint 可能由不同版本的 LeRobot 或 PyTorch 生成。尝试安装与训练时相同的依赖版本。
2. **模型结构变化**：如果修改了模型代码，旧的 checkpoint 结构不再匹配。从头训练，或用兼容的模型结构加载后迁移。
3. **文件损坏**：如果训练被强制终止（kill -9），checkpoint 文件可能写入不完整。使用上一个完整的 checkpoint 恢复：
   ```bash
   # 检查文件大小异常的 checkpoint
   ls -lhS outputs/train/*/checkpoints/
   ```
4. **恢复训练的命令**：
   ```bash
   lerobot-train \
       --config.output_dir outputs/train/my_experiment \
       --resume 1
   ```

---

## C.7 部署问题

### C.7.1 策略部署后机械臂不运动

**症状**：训练完成的策略成功加载，但机械臂不动。

**诊断命令**：
```bash
# 确认策略输出值范围
python3 -c "
import torch
model = torch.load('policy.pth')
# 打印模型输出归一化参数
print('Action mean:', model.get('action_mean', 'N/A'))
print('Action std:', model.get('action_std', 'N/A'))
"

# 观察实际控制指令流
candump can0 | grep "155\|156\|157" | head -20
```

**解决方案**：
1. **动作反归一化**：策略输出的动作值是归一化后的（通常均值 0，标准差 1），需要反向映射到实际的关节角度编码值。确认反归一化步骤正确：
   ```python
   action_raw = model_output  # 范围 ~ [-1, 1]
   action_denorm = action_raw * action_std + action_mean  # 反归一化
   joint_cmd = (action_denorm * 1000).astype(int)  # 转为 0.001° 编码
   ```
2. **动作空间维度**：确认策略输出的维度和机械臂可接受的关节数量一致（6 个关节 + 1 个夹爪 = 7 维）。
3. **机械臂未使能**：检查 Follower 臂是否已处于使能状态和控制模式下。
4. **控制模式**：确认控制模式为 CAN 指令控制（ctrl_mode = 0x01）。
5. **安全限位阻止**：策略输出的目标角度可能超出机械臂当前设置的软限位。

### C.7.2 运动抖动/不平滑

**症状**：策略推理产生的运动有明显的抖动、卡顿或步进感。

**诊断命令**：
```bash
# 检查推理频率
python3 -c "
import time
times = []
for _ in range(100):
    t0 = time.perf_counter()
    # 模拟一次推理
    time.sleep(0.01)
    times.append(time.perf_counter() - t0)
print(f'Mean inference time: {np.mean(times)*1000:.1f}ms, Max: {np.max(times)*1000:.1f}ms')
"
```

**解决方案**：
1. **推理频率不足**：如果模型一次推理耗时超过控制周期，会导致帧间间隙过大。优化模型推理：
   - 使用 ONNX Runtime 或 TensorRT 加速推理
   - 减小模型输入分辨率
   - 使用更轻量级的模型架构
2. **动作平滑（Action Chunking）**：如果使用 ACT 模型，它本身会输出一个动作块（chunk），可以减少帧间跳变。确认 `chunk_size` 设置合理（通常 10~100）。
3. **时间平滑（Temporal Smoothing）**：在后处理中加入指数移动平均平滑输出：
   ```python
   current_action = 0.8 * raw_output + 0.2 * previous_action
   ```
4. **观测跳变**：摄像头图像不稳定或有噪点导致策略输出抖动。确保摄像头固定、光照条件稳定。

### C.7.3 部署时程序崩溃/异常退出

**症状**：部署脚本运行一段时间后崩溃，或按 Ctrl+C 后机械臂不释放/锁死。

**诊断命令**：
```bash
# 查看崩溃前的 Python 异常
python3 deploy.py 2>&1 | tee deploy.log

# 检查核心转储
coredumpctl list

# 检查系统日志
journalctl -xe | tail -30
```

**解决方案**：
1. **异常处理不完整**：部署脚本缺少 `try-finally` 保证安全退出的逻辑。必须包装：
   ```python
   try:
       # 部署循环
       while running:
           action = policy.predict(obs)
           piper.set_joints(action)
   except KeyboardInterrupt:
       print("Interrupted, cleaning up...")
   finally:
       piper.disable_all()  # 失能所有电机
       piper.close()        # 关闭 CAN 连接
   ```
2. **CAN 通信超时**：长时间运行时 CAN 通信偶尔超时。添加重试逻辑：
   ```python
   import time
   def send_with_retry(piper, cmd, max_retries=3):
       for attempt in range(max_retries):
           try:
               piper.send(cmd)
               return True
           except TimeoutError:
               time.sleep(0.01)
       return False
   ```
3. **内存泄漏**：长时间运行的推理可能导致 GPU 内存缓慢增长。在循环中定期清空 CUDA 缓存：
   ```python
   if step % 1000 == 0:
       torch.cuda.empty_cache()
   ```
4. **紧急停止按钮失效**：部署环境必须确保物理急停按钮可用。测试急停响应是否正常。
5. **程序退出后机械臂未失能**：如果异常处理未能执行，可以手动发送急停指令或切断机械臂电源。也可以在启动脚本中加入 trap：
   ```bash
   #!/bin/bash
   trap "cansend can0 150#0100000000000000" EXIT  # 退出时发送急停
   python3 deploy.py
   ```

---

## C.8 环境与依赖问题速查

| 症状 | 可能原因 | 解决方向 |
|---|---|---|
| `ImportError: No module named 'piper_sdk'` | SDK 未安装或不在 PYTHONPATH | `pip install -e /path/to/piper_sdk` |
| `ModuleNotFoundError: No module named 'lerobot'` | LeRobot 未安装 | `pip install -e ".[piper]"` |
| `ImportError: libpython3.x.so` | Python 动态库缺失 | `sudo apt install libpython3.x-dev` |
| `can0: unknown interface` | CAN 驱动未加载 | `sudo modprobe can; sudo modprobe can_raw; sudo modprobe gs_usb` |
| `RTNETLINK answers: Operation not supported` | CAN 配置参数错误 | 检查 bitrate 参数拼写：`bitrate` 不是 `bit_rate` |
| `socket.error: [Errno 1] Operation not permitted` | 权限不足 | `sudo` 或加入 `dialout` 组 |
| `AttributeError: 'NoneType' object has no attribute 'x'` | SDK 未正确初始化 | 检查 `Piper()` 构造函数参数，确认 can_interface 正确 |
| HDF5 文件读取慢 | 磁盘 I/O 瓶颈 | 迁移到 SSD，使用数据子集训练 |
| 机器人动作幅度太小 | 动作归一化范围问题 | 检查 `action_mean` / `action_std` 是否正确计算 |
| 环境噪音导致误触发 | 工作室噪音/振动 | 改善环境光照，固定机械臂底座，减少背景变化 |

---

## C.9 获取更多帮助

1. **CAN 总线调试工具推荐**：
   - `can-utils`：Linux 标准 SocketCAN 工具集 (candump/cansend/cangen)
   - `python-can`：Python CAN 接口库，可用于编写自定义调试脚本
   - `busmaster`：开源 CAN 总线分析工具（带 GUI）

2. **日志收集**：在提交 Issue 前，请收集以下信息：
   ```bash
   # 环境信息
   uname -a                          # 内核版本
   python3 --version                 # Python 版本
   pip list | grep -E "piper|lerobot|torch|can"  # 依赖版本
   nvidia-smi                        # GPU 信息（如适用）
   lsusb                             # USB 设备列表
   ip link show                      # 网络接口（含 CAN）
   ```

3. **联系渠道**：
   - PiPER SDK Issue：查看 SDK 项目仓库的 Issues 页面
   - LeRobot Discord：[HuggingFace Discord](https://discord.gg/huggingface) 中的 `#lerobot` 频道
   - PiPER 技术支持：联系设备供应商获取固件更新和技术文档
