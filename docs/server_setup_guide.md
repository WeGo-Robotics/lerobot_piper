# PiPER ACT 训练服务器环境配置指南

## 0. 服务器选择

### 推荐配置

| 项目 | 最低要求 | 推荐 |
|------|---------|------|
| GPU | RTX 3060 12GB | RTX 3090/4090 24GB |
| 显存 | ≥8GB | ≥16GB |
| CPU | 4核 | 8核+ |
| 内存 | 16GB | 32GB |
| 磁盘 | 50GB | 100GB+ |
| 系统 | Ubuntu 22.04/24.04 | Ubuntu 22.04 |

### 国内平台推荐

- **AutoDL** (https://www.autodl.com) — 按小时计费，RTX 3090 约 1-2 元/小时
- **恒源云** (https://www.hengyuangpu.com)
- 选择带 `miniconda` 或 `cuda 12.x` 预装的镜像可跳过第 1 节

---

## 1. 系统环境初始化

SSH 登录服务器后执行。如果平台预装了 CUDA 和 miniconda，跳到第 2 节。

### 1.1 基础工具

```bash
sudo apt update && sudo apt install -y build-essential git curl wget tmux htop
```

### 1.2 CUDA 驱动（如果未预装）

```bash
# 检查是否已有 CUDA
nvidia-smi

# 如果没有，安装 CUDA 12.x。以 Ubuntu 22.04 为例：
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-toolkit-12-6
```

> 训练用 PyTorch 2.7.1，需要 CUDA ≥12.1。`nvidia-smi` 显示的 CUDA 版本是驱动 API 版本，只要 ≥12.1 即可。

### 1.3 Miniconda（如果未预装）

```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b -p $HOME/miniconda3
source $HOME/miniconda3/etc/profile.d/conda.sh
conda init bash
# 重新登录或 source ~/.bashrc
```

---

## 2. 创建 Python 环境

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda create -n lerobot python=3.10 -y
conda activate lerobot
```

---

## 3. 克隆 & 安装 LeRobot

### 3.1 克隆项目

```bash
# 创建工作目录
PIPER_ROOT=~/Piper
mkdir -p $PIPER_ROOT
cd $PIPER_ROOT

# 克隆（你的 fork）
git clone -b piper https://github.com/oSaintJusto/lerobot_piper.git lerobot_piper-piper
```

### 3.2 安装依赖

```bash
cd $PIPER_ROOT/lerobot_piper-piper

# 安装 LeRobot + ACT 训练所需依赖（不需要 piper_sdk/realsense 等硬件驱动）
pip install --upgrade pip
pip install -e ".[intelrealsense]"

# 关键训练依赖（如果上面命令报错就手动装）
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
pip install datasets diffusers huggingface-hub
pip install einops av opencv-python-headless
pip install draccus safetensors wandb
```

> 注意：服务器不需要 `piper_sdk`、`pyrealsense2`、`feetech-servo-sdk`、`dynamixel-sdk` 等硬件驱动。如果 `pip install -e ".[piper]"` 失败，只装训练相关依赖即可。

### 3.3 验证

```bash
python -c "import torch; print(f'CUDA: {torch.cuda.is_available()}'); print(f'GPU: {torch.cuda.get_device_name(0)}')"
# 应输出: CUDA: True, GPU: NVIDIA GeForce RTX 3090 (或你的 GPU 型号)
```

---

## 4. 传输数据集

### 4.1 打包数据集（在你本地机器上执行）

```bash
cd ~/Documents/Piper/lerobot_piper-piper
DATASET_NAME=piper_merged_81eps bash scripts/package_lerobot_dataset.sh
# 生成: ~/Documents/Piper/exported_datasets/piper_merged_81eps.tar.gz
```

### 4.2 上传到服务器（在你本地机器上执行）

```bash
# AutoDL 默认端口 60000，恒源云 40000。替换 YOUR_HOST 和 YOUR_PORT
scp -P YOUR_PORT \
    ~/Documents/Piper/exported_datasets/piper_merged_81eps.tar.gz \
    root@YOUR_HOST:/root/autodl-tmp/
```

### 4.3 解压数据集（在服务器上执行）

```bash
cd /root/autodl-tmp  # 或你的数据目录
tar xzf piper_merged_81eps.tar.gz
ls piper_merged_81eps/
# 应该看到: data/ meta/ videos/
```

> **重要**：AutoDL 的 `/root/autodl-tmp` 是高速数据盘，系统盘 `/root` 很小。数据必须放数据盘。

---

## 5. 开始训练

### 5.1 使用 tmux 保持会话（防止 SSH 断开后训练中断）

```bash
tmux new -s train
conda activate lerobot
```

### 5.2 运行训练

```bash
PIPER_ROOT=~/Piper
DATASET_DIR=/root/autodl-tmp/piper_merged_81eps
OUTPUT_DIR=/root/autodl-tmp/outputs/train

cd $PIPER_ROOT/lerobot_piper-piper

export PIPER_ROOT=$PIPER_ROOT
export DATASET_DIR=$DATASET_DIR
export DATASET_NAME=piper_merged_81eps
export OUTPUT_DIR=$OUTPUT_DIR

# 使用预置的训练脚本
bash scripts/server_train_act_from_dataset.sh
```

### 5.3 自定义参数示例

```bash
lerobot-train \
  --policy.type=act \
  --policy.device=cuda \
  --policy.chunk_size=30 \
  --policy.n_action_steps=30 \
  --policy.pretrained_backbone_weights=ResNet18_Weights.IMAGENET1K_V1 \
  --dataset.repo_id="local/piper_merged_81eps" \
  --dataset.root="/root/autodl-tmp/piper_merged_81eps" \
  --dataset.video_backend=pyav \
  --batch_size=8 \
  --steps=20000 \
  --log_freq=100 \
  --save_freq=5000 \
  --num_workers=4 \
  --output_dir="/root/autodl-tmp/outputs/train/piper_act_$(date +%Y%m%d_%H%M%S)" \
  --job_name=piper_act \
  --wandb.enable=false
```

### 参数调整建议

| 参数 | 默认 | 调优方向 |
|------|------|---------|
| `batch_size` | 8 | 显存不够就降到 4，显存充裕可以 16 |
| `steps` | 20000 | 数据多可以加（40000-80000），观察 loss 收敛 |
| `policy.chunk_size` | 30 | 和你的数据帧率匹配，30fps×30=1秒的动作块 |
| `policy.pretrained_backbone_weights` | ResNet18 | 可选 null（从头训练）或 ResNet50 |
| `save_freq` | 5000 | 每 N 步保存 checkpoint |

### 5.4 tmux 操作

```bash
# 离开会话（训练继续后台运行）
Ctrl+B 然后按 D

# 重新连接
tmux attach -t train

# 查看训练日志
tail -f /root/autodl-tmp/outputs/train/piper_act_*/logs/*.log
```

---

## 6. 下载训练结果

### 6.1 找到输出目录

```bash
ls /root/autodl-tmp/outputs/train/piper_act_*/
# 含有 checkpoints/ 和 logs/
```

### 6.2 监控训练进度

```bash
# 查看最新 loss
grep "loss" /root/autodl-tmp/outputs/train/piper_act_*/logs/*.log | tail -20

# 查看 checkpoint
ls /root/autodl-tmp/outputs/train/piper_act_*/checkpoints/
```

### 6.3 下载到本地

在你本地机器上执行：

```bash
# 下载最新的 checkpoint
scp -P YOUR_PORT -r \
    root@YOUR_HOST:/root/autodl-tmp/outputs/train/piper_act_*/checkpoints/last \
    ~/Documents/Piper/outputs/train/

# 或下载整个训练输出
scp -P YOUR_PORT -r \
    root@YOUR_HOST:/root/autodl-tmp/outputs/train/piper_act_* \
    ~/Documents/Piper/outputs/train/
```

---

## 7. 完整流程速查

```bash
# ─── 服务器上 ───

# 1. 环境
source ~/miniconda3/etc/profile.d/conda.sh && conda activate lerobot

# 2. 解压数据
tar xzf /root/autodl-tmp/piper_merged_81eps.tar.gz -C /root/autodl-tmp/

# 3. 开始训练
tmux new -s train
cd ~/Piper/lerobot_piper-piper
DATASET_DIR=/root/autodl-tmp/piper_merged_81eps bash scripts/server_train_act_from_dataset.sh

# Ctrl+B, D 离开

# ─── 本地机器上 ───

# 4. 打包上传
cd ~/Documents/Piper/lerobot_piper-piper
DATASET_NAME=piper_merged_81eps bash scripts/package_lerobot_dataset.sh
scp -P PORT ~/Documents/Piper/exported_datasets/piper_merged_81eps.tar.gz root@HOST:/root/autodl-tmp/

# 5. 下载结果
scp -P PORT -r root@HOST:/root/autodl-tmp/outputs/train/piper_act_*/checkpoints/last ~/Documents/Piper/outputs/train/
```

---

## 8. 常见问题

### CUDA Out of Memory

```bash
# 降低 batch_size 和 num_workers
lerobot-train ... --batch_size=4 --num_workers=2
```

### pip install 失败

```bash
# 只安装训练核心依赖，跳过硬件驱动
pip install torch torchvision torchcodec -f https://download.pytorch.org/whl/cu121
pip install datasets diffusers accelerate av opencv-python-headless safetensors einops draccus wandb pyyaml huggingface-hub
```

### 数据集加载报错

```bash
# 检查目录结构
ls /root/autodl-tmp/piper_merged_81eps/
# 必须包含: data/ meta/ videos/
# meta/ 必须包含: info.json episodes/

cat /root/autodl-tmp/piper_merged_81eps/meta/info.json | python -m json.tool | head -20
```

### SSH 断开后找不到 tmux 会话

```bash
tmux ls          # 列出所有会话
tmux attach -t train  # 重新连接
```
