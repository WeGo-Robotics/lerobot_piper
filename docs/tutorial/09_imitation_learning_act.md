# 第九章 模仿学习与 Action Chunking Transformer (ACT)

> 学习目标：理解模仿学习的基本原理，掌握 Action Chunking Transformer (ACT) 的架构设计、训练流程与部署方法，能够将 ACT 策略应用于双臂机器人操作任务。

---

## 9.1 模仿学习基础

### 什么是模仿学习

模仿学习 (Imitation Learning) 是机器人学习中最直观的范式之一。其核心思想可以概括为一句话：**让人来演示任务，让机器人学习模仿人的行为**。从机器学习的角度看，模仿学习本质上是一种监督学习——只是输入和输出都活在物理世界中。

具体而言，模仿学习将机器人控制问题转化为一个映射问题：

- **输入 (observation)**：机器人当前感知到的所有信息，包括相机拍摄的 RGB 图像、机械臂各关节的当前位置、夹爪的开合状态等。
- **输出 (action)**：机器人下一步应该执行的动作，通常表示为各关节的目标角度位置和夹爪的目标开度。
- **训练数据**：由人类操作者通过遥操作采集的 (observation, action) 对，按时间顺序排列形成若干段演示 (demonstration episodes)。

与强化学习 (Reinforcement Learning) 相比，模仿学习最大的区别在于：**模仿学习不需要设计 reward 函数**。reward 函数的设计是强化学习中最困难的部分之一——对于一个复杂的双臂操作任务，如何用数学定义"做得好"往往比直接演示几次要困难得多。模仿学习通过人类演示绕过了这个问题，让数据本身定义什么是好的行为。

### 行为克隆 (Behavioral Cloning)

行为克隆是模仿学习中最简单、最直接的方法。它的数学形式非常朴素：

$$\min_{\pi} \mathbb{E}_{(o_t, a_t) \sim \mathcal{D}} \left[ \|\pi(o_t) - a_t\|^2 \right]$$

其中 $\pi$ 是我们要学习的策略网络，$\mathcal{D}$ 是人类的演示数据集，$o_t$ 是 $t$ 时刻的观测，$a_t$ 是 $t$ 时刻人类执行的动作。换句话说，行为克隆的目标就是让策略网络对每一个观测下的预测动作，尽可能接近人类在该观测下执行的动作。

然而，行为克隆存在两个根本性的挑战：

**问题一：复合误差 (Compounding Errors)**。假设策略每次预测的误差是 $\epsilon$。在开环执行中，第一步会产生一个小偏差，这导致机器人到达的状态略微偏离训练分布；第二步的策略在这个"没见过的状态"上进行预测，可能产生更大的偏差；如此往复，误差随时间呈指数级累积，最终机器人可能完全偏离演示过的轨迹。这是模仿学习最核心的问题——策略的误差分布与训练时的误差分布不同（分布偏移，distribution shift）。

**问题二：多模态 (Multi-modality)**。对于同一个初始状态，可能存在多种同样正确的操作方式。例如，把桌上的杯子拿起来，可以从左边接近也可以从右边接近。如果用一个简单的回归模型（输出单一的动作向量），它会学习输出所有可能动作的平均值——而这个平均值恰恰可能是一个无效动作（比如夹爪从中间穿过了杯子）。标准的 L2 回归无法处理这种一个输入对应多个合理输出的情况。

本章介绍的 ACT 算法通过预测动作序列 (action chunking) 和 Transformer 架构，有效缓解了上述两个问题。

### LeRobot 数据集格式

在深入算法细节之前，我们首先需要理解训练数据的组织方式。LeRobot 框架使用了一种统一的数据格式来存储演示数据。一个数据集包含多个 episode（每次人类遥操作录制一段称为一个 episode），每个 episode 是一个时间序列：

```
Episode k:
  t=0: observation_0 → action_0
  t=1: observation_1 → action_1
  ...
  t=T: observation_T → action_T
```

每个时间步的 observation 和 action 的具体结构如下：

```python
observation = {
    "observation.state": tensor([joint1.pos, joint2.pos, ..., joint6.pos, gripper.pos]),  # (7,)
    "observation.images.wrist":  tensor(480, 640, 3),   # RGB 图像，腕部相机
    "observation.images.global": tensor(480, 640, 3),   # RGB 图像，全局相机
}

action = tensor([joint1.pos, joint2.pos, ..., joint6.pos, gripper.pos])  # (7,)
```

注意 action 和 observation.state 共享相同的维度结构（都是 7 维：6 个关节角度 + 1 个夹爪开度），但含义不同：observation.state 是当前关节的实际位置，而 action 是人类演示的目标位置。训练时，模型学习从 observation.state 映射到 action 的偏移关系。

在代码中，action 的键名列表定义在 `piper_act_safe_rollout.py` 中：

```python
ACTION_KEYS = [
    "joint1.pos",
    "joint2.pos",
    "joint3.pos",
    "joint4.pos",
    "joint5.pos",
    "joint6.pos",
    "gripper.pos",
]
```

---

### 为什么 ACT 选择关节空间而不是像素空间

在第一章中，我们讨论了关节空间控制和笛卡尔空间控制的分野。现在我们从 ACT 算法的具体实现角度，审视这个设计选择如何影响学习问题的难度和策略的性能。

#### ACT 的输入和输出

回顾 ACT 的实际数据流：

```
输入:
  ● 腕部相机图像 (640×480×3 = 921,600 维) ──→ ResNet-18 ──→ 特征向量 (512 维)
  ● 全局相机图像 (640×480×3 = 921,600 维) ──→ ResNet-18 ──→ 特征向量 (512 维)
  ● 关节当前角度 + 夹爪开度 (7 维) ──────────────────────────→ 直接拼接

输出:
  ● 未来 30 步的关节目标位置: 30 × 7 = 210 维
    (每步: joint1.pos, joint2.pos, ..., joint6.pos, gripper.pos)
```

ACT 输出的不是像素空间的动作（比如"在图像坐标 (320, 240) 处抓取"），不是末端笛卡尔坐标（xyz + 姿态），而是**关节角度**。这个选择与 Behavior Transformer (BeT) 和 Diffusion Policy 等主流模仿学习算法完全一致。

#### 为什么：动作空间维度的指数级差异

这可能是模仿学习中最重要的"免费午餐"——动作空间的维度差异：

```
动作空间类型           维度           学习难度
─────────────────────────────────────────────────
像素空间动作           921,600         极高 (语义鸿沟)
  (预测下一帧像素)

笛卡尔轨迹             6 + IK          中等 (需要 IK 求解)
  (x, y, z, roll, pitch, yaw)

关节空间动作           7               低 (直接拟合)
  (joint1..6, gripper)  ← ACT 的选择
```

从 921,600 维到 7 维，维度降低了超过 10 万倍。在机器学习中，**学习一个 7→7 的映射远容易于学习一个 921,600→921,600 的映射**。这不仅体现在需要的数据量上，也体现在训练的稳定性和预测的精确度上。

#### 更根本的原因：避免"中间表示"的累积误差

如果策略输出的是笛卡尔坐标（末端 xyz + 姿态），轨迹需要经过 IK 求解器才能转化为电机命令：

```
策略 → [x, y, z, roll, pitch, yaw] → IK 求解器 → [θ₁, ..., θ₆] → 电机
                                       ↑
                                  多解性、奇异性、
                                  数值迭代误差
```

IK 求解器在以下情况下可能失效：
- 目标位姿在奇异点附近（Jacobian 病态，关节速度趋于无穷）
- 目标位姿有多个解，IK 选了"错误"的那个（比如导致关节 3 撞到桌面）
- 数值 IK 不收敛（目标位姿稍微超出工作空间边界）

当 IK 求解器出错时，策略不知道自己犯了什么错——它只能看到"机械臂没有到达我指定的位置"，但无法知道是 IK 的问题还是自己的问题。这个反馈回路断裂使得 end-to-end 学习的梯度无法从动作效果回传到策略参数。

在关节空间中，这个问题完全不存在：

```
策略 → [θ₁, ..., θ₆, gripper] → 直接发给电机
```

**策略的输出和电机的输入是同一种数据类型**，不需要任何中间变换。这意味着：
- 策略可以精确控制每个关节的运动量
- 训练信号（L2 loss）直接反映策略参数的优化方向
- 没有奇异性、多解性、收敛性等"黑箱"问题

#### 为什么这不是一个"偷懒"的选择

有人可能会问：笛卡尔空间控制更"通用"——策略与机械臂解耦，同一个策略可以用于不同的机械臂。为什么不做笛卡尔？

答案是：**在当前模仿学习的发展阶段，让一个机械臂可靠地完成 3-5 个桌面任务，比让一个策略跨机械臂泛化更紧迫**。先让一台机械臂"好用"，再考虑跨硬件的泛化。跨硬件泛化是 VLA 路线的目标——用海量多机器人数据训练一个通用策略——而不是 ACT 要解决的问题。

ACT 的务实哲学：**在低维关节空间中，用 50-200 episodes 的数据，让一个真实的机械臂学会做一件事**。这个目标在当前阶段是合理且可实现的。

---

### ACT 与 VLA 的关系

#### ACT 是视觉模仿学习，VLA 是其多模态扩展

ACT (Action Chunking Transformer) 是一种**视觉模仿学习（Visual Imitation Learning）**方法。它的核心能力是从图像和关节状态直接预测动作序列，但策略不知道"任务是什么"——它只是复现训练数据中的行为模式。

从 ACT 升级到 VLA 的核心变化是**引入语言条件**：

```
ACT:                          VLA:
┌──────────────────┐          ┌──────────────────────────┐
│  image_t         │          │  "拿起红色杯子"           │
│  joint_state_t   │          │  image_t                  │
│       ↓          │          │  joint_state_t            │
│  ResNet + Trans. │          │       ↓                   │
│       ↓          │          │  LangEnc + VisEnc + Trans. │
│  actions_{t:t+30}│          │       ↓                   │
└──────────────────┘          │  actions_{t:t+N}           │
                              └──────────────────────────┘

"不知道任务是什么，      "知道任务是'拿起红色杯子'，
 只知道看到X→做Y"          可以根据语言切换行为"
```

#### 如何将 ACT 改造为 VLA

从架构角度看，将 ACT 改造为 VLA 需要做以下扩展：

```python
# 当前 ACT：仅视觉输入
class ACTPolicy(nn.Module):
    def forward(self, image, joint_state):
        visual_features = self.vision_encoder(image)         # ResNet-18
        state_features = self.state_encoder(joint_state)     # MLP
        fused = torch.cat([visual_features, state_features], dim=-1)
        actions = self.transformer_decoder(fused)            # 输出 30×7
        return actions

# 改造后的 VLA：
class VLAPolicy(nn.Module):
    def forward(self, image, joint_state, language_instruction):
        visual_features = self.vision_encoder(image)         # SigLIP/ViT (更大的视觉 backbone)
        text_features = self.language_encoder(language)      # T5/Gemma (新增语言编码器)
        state_features = self.state_encoder(joint_state)     # MLP
        # 通过交叉注意力融合三种模态
        fused = self.multimodal_transformer(
            visual_features, text_features, state_features
        )
        actions = self.action_decoder(fused)
        return actions
```

这就是 RT-1（Google Robotics）、RT-2（Google DeepMind）、Octo（Stanford）等模型的核心架构思路——在模仿学习的视觉-动作流水线上，增加一个语言编码分支，通过交叉注意力机制让语言语义影响动作生成。

#### LeRobot 的模块化设计使 ACT → VLA 的迁移变得简单

本项目的核心设计优势在于：**数据采集流水线与策略实现是解耦的**。

```
                    ┌─────────────────────────┐
                    │   数据采集流水线 (不变)    │
                    │   - 相同的硬件 (PiPER)    │
                    │   - 相同的双相机配置      │
                    │   - 相同的 CAN 中继       │
                    │   - 相同的数据集格式      │
                    └───────────┬─────────────┘
                                │
                    ┌───────────▼─────────────┐
                    │   策略实现 (可替换)       │
                    │                           │
                    │   今天: ACTPolicy         │
                    │   明天: VLAPolicy         │
                    │   后天: WorldModelPlanner │
                    │                           │
                    │   统一的接口:              │
                    │   predict(observation) → action
                    └───────────────────────────┘
```

你今天在 PiPER 上采集的每一条数据（双相机 RGB 图像 + 关节状态 + 动作），只要再配上语言标签（如 "拿起红色方块"），就是一条标准的 VLA 训练数据。不需要换硬件，不需要改相机布局，不需要重新设计数据格式。

语言标签可以来自：
- 录制时的语音输入（你已经集成了语音提示功能）
- 手动标注（在 dataset metadata 中添加 `"task": "pick up red block"`）
- 自动标注（通过 VLM 在回放视频时自动生成描述）

#### 现在采集的数据就是"VLA-ready"

这是理解本项目价值的一个关键视角：你每天的 ACT 实验数据并不仅仅服务于当前的 ACT 训练。只要 episode 的数据结构是完整的（observation + action + metadata），这些数据在未来可以直接合并到更大的 VLA 训练集中。这就是为什么 LeRobot 的标准数据格式把 metadata 设为可选但强烈推荐——它为未来扩展预留了空间。

---

### ACT 与世界模型的关系

#### ACT 是 Model-Free，世界模型是 Model-Based

这是强化学习和机器人学习中两个根本不同的范式：

```
┌────────────────────────────────────────────────────────────────┐
│  Model-Free (ACT 所属的范式)                                  │
│                                                                │
│  策略 = f(observation) → action                               │
│                                                                │
│  直接学习"看到什么 → 做什么"的映射。                            │
│  不构建环境模型，不做预测，不"思考"未来。                       │
│                                                                │
│  代表方法: ACT, Diffusion Policy, Behavior Transformer,       │
│            SAC, PPO, DrQ-v2                                   │
└────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────┐
│  Model-Based (世界模型所在的范式)                              │
│                                                                │
│  世界模型 = f(s_t, a_t) → s_{t+1}                             │
│  策略     = argmax_a [奖励(s_t, a) + V(世界模型(s_t, a))]     │
│                                                                │
│  先学一个内部的世界模型（环境的预测器），                        │
│  然后在世界模型内部做规划，选出最优动作。                       │
│                                                                │
│  代表方法: Dreamer (v1/v2/v3), TD-MPC (v1/v2),                │
│            MuZero, PlaNet                                     │
└────────────────────────────────────────────────────────────────┘
```

#### 为什么 Model-Free 目前赢了（在真实机器人上）

截至 2025-2026 年，真实机器人操作任务的 SOTA 方法仍然主要是 model-free 的（ACT, Diffusion Policy 等）。原因在于：

**世界模型在视觉空间中积累预测误差。** 每一步的微小预测误差会复合增长：

```
初始状态 s_0
  → 预测 s_1 (误差 ε)
    → 以 s_1 (有误差) 为起点预测 s_2 (误差 2ε)
      → 以 s_2 (有误差) 为起点预测 s_3 (误差 ≈ 4ε)
        → ...
          → 第 30 步的预测已经与实际无关
```

对于 ACT 的 30 步（chunk_size = 30）这样的时间跨度，显式世界模型的视觉预测通常已经崩溃。没有可靠的未来预测，基于世界模型的规划就失去了基础。

**世界模型需要比策略更多的训练数据。** 一个行为克隆策略只需要学习 observation → action 的映射（监督信号是动作的 L2 loss）。一个世界模型需要学习 (observation, action) → next_observation 的映射，输出空间是完整的观测（包括高维图像），这本质上是一个更难的问题。

#### ACT 的 Action Chunking 作为隐式世界模型

ACT 的一个精妙之处在于：**它的 action chunking 在功能上等价于一种有限的、隐式的世界模型**。

```
显式世界模型的预测过程:
  s_0, a_0 → world_model → s_1_pred
  s_1_pred, a_1 → world_model → s_2_pred
  ...

ACT 的动作分块预测:
  observation_t → ACT → [a_t, a_{t+1}, a_{t+2}, ..., a_{t+29}]
  
  这些动作序列本身编码了"接下来会发生什么"的信息——
  不需要预测像素，但序列结构反映了任务的时间逻辑：
  
  a_t    ≈ 接近目标物体
  a_{t+5} ≈ 到达抓取位置  
  a_{t+8} ≈ 闭合夹爪
  a_{t+15}≈ 抬起物体
  a_{t+25}≈ 移动到目标位置
  a_{t+29}≈ 释放物体
```

ACT 不做视觉预测，不做显式的状态转移，但它的 30 步动作序列**隐式地将时间结构融入了策略输出**。这既得到了时间抽象的好处（平滑、减少累积误差、抑制高频抖动），又避免了显式世界模型的视觉预测困难。

这是一种务实的折中——"我们不预测世界会变成什么样，但我们预测机械臂应该怎么动"。

#### 但 ACT 不能做什么：长期规划

隐式世界模型（action chunking）的局限在于：它只能预测"按计划进行时"的动作，无法推理"如果出错了怎么办"。

考虑以下场景：

```
任务: 抓取杯子并放到左边

情况 A (训练数据中的):
  接近杯子 → 合拢夹爪 → 杯子在夹爪中 → 移动到左边 → 释放
  ACT 可以完美处理（这是训练分布内的）

情况 B (分布外):
  接近杯子 → 合拢夹爪 → 杯子滑落了！(夹爪中空无一物)
  ACT 会继续执行后续动作: "移动到左边 → 释放"
  它"不知道"杯子已经掉了——因为它没有世界模型来检查
  "夹爪中还有杯子吗？"
```

一个完整的世界模型 + 规划系统可以处理情况 B：在每一步根据预测的未来状态进行重新规划（re-planning）。如果世界模型预测"再这样下去杯子会掉"，规划器可以调整抓取力度或改变接近角度。

这是一种不同的能力层级：
- **ACT (model-free)**：适用于"如果一切按预期进行"的 tasks，对分布外情况无抵抗力
- **世界模型 (model-based)**：理论上可以推理"如果...会怎样"，能够响应意外情况
- **中间地带**：ACT + 高频重规划（在 ACT 输出的 30 步中，每执行 5 步就用新观测重新调用 ACT 预测下一个 30 步）——这是一种折中，利用高频的 model-free 推理来近似 re-planning

#### 为什么理解这层关系很重要

理解 ACT 和世界模型的关系，有助于理解本项目架构的演进空间：

```
今天:
  PiPER 双臂 + LeRobot + ACT = 可靠的桌面级任务模仿学习

明天:
  同样的硬件 + 同样的数据 + WorldModelPolicy =
  能做在线规划的"更聪明"的机器人

后天:
  同样的硬件 + 更多用户的共享数据 + VLA + World Model =
  通用语言条件、能做长期规划、自适应意外情况的机器人
```

LeRobot 的设计保证了你不需要为这个演进重写任何数据采集代码——只需要替换 `Policy` 的实现。这是框架层面"为未来设计"的范例。

## 9.2 Action Chunking Transformer (ACT) 原理

### 核心思想

ACT 的核心创新在于：**不是预测下一个时刻的单个动作，而是预测未来一段时间的完整动作轨迹（chunk）**。

具体来说，传统的行为克隆策略在时刻 $t$ 接收观测 $o_t$，输出单个动作 $a_t$。而 ACT 在时刻 $t$ 接收观测 $o_t$，一次性输出 $K$ 个未来动作：

$$\pi(o_t) = [\hat{a}_t, \hat{a}_{t+1}, \hat{a}_{t+2}, \ldots, \hat{a}_{t+K-1}]$$

其中 $K$ 称为 `chunk_size`，典型值为 30（在 30 fps 下对应 1 秒的动作序列）。

**为什么要预测多步动作？**

1. **减少高频抖动**：单步预测容易受到传感器噪声和模型微小扰动的影响，导致输出在相邻帧之间发生剧烈变化。多步预测天然地编码了动作的时间平滑性——chunk 内的动作序列是模型在同一次推理中生成的，彼此之间是协调的。

2. **利用时间一致性**：机器人操作中的连续动作高度相关。例如执行一个抓取动作，整个过程中各关节的运动轨迹通常是一条平滑的曲线。一次性预测整条曲线比逐帧预测每个点更能保证轨迹的连贯性。

3. **降低推理频率需求**：如果 `n_action_steps` 设置为 30，意味着策略推理一次后，可以将输出的 30 步动作逐帧发送给机器人执行整整 1 秒，而无需每帧都进行推理。这大大降低了对推理速度的要求。不过在实践中，为了获得更好的控制效果，通常 `n_action_steps` 会结合 temporal ensemble 使用。

### 网络架构

ACT 的网络结构可以拆解为三个主要模块：

```
输入模块:
  - 腕部图像 (wrist)    ──► ResNet-18 backbone ──► 图像特征 (512维)
  - 全局图像 (global)   ──► ResNet-18 backbone ──► 图像特征 (512维)
  - 关节位置 (obs.state) ──► MLP ────────────────► 关节特征 (512维)

处理模块 (Transformer Encoder + Decoder):
  - 图像特征 + 关节特征 + 位置编码 ──► Transformer Encoder
  - K 个可学习 query tokens ───────► Transformer Decoder (cross-attend to encoder output)

输出模块:
  - K 个动作向量, 每个 7 维 (joint1..6 + gripper)
```

整个数据流可以描述为以下步骤：

**步骤一：视觉特征提取**。两个相机各自拍摄的 $480 \times 640 \times 3$ 图像分别送入一个 ResNet-18 网络。每个 ResNet 的输入经过 conv1 到 layer4 的逐层处理后，得到一张空间尺寸缩小的特征图，再经过全局平均池化 (Global Average Pooling) 压缩为一个 512 维的向量。两个相机的特征向量拼接 (concatenate) 后得到 1024 维的视觉特征。

**步骤二：本体感知特征提取**。当前的 7 维关节位置通过一个线性层投影到 512 维的空间中，与视觉特征的维度对齐。

**步骤三：位置编码**。在进入 Transformer Encoder 之前，每个 token 都需要加上位置编码以保留空间或序列信息：

- 对于一维的 token（如 latent、robot state），使用可学习的 Embedding 作为位置编码。
- 对于二维的图像特征图，使用正弦位置编码（2D Sinusoidal Position Embedding），对每个像素位置 $(x, y)$ 生成唯一的编码向量。这种编码方式借鉴了 Transformer 论文中的思路：对偶数维度使用正弦函数，对奇数维度使用余弦函数，不同维度具有不同的频率。

**步骤四：Transformer Encoder**。Encoder 由 4 层 Transformer Encoder Layer 堆叠而成。每层包含：
- Multi-head Self-Attention（8 个头，每个头 64 维）
- Feed-forward Network（将 512 维扩展到 3200 维，再压缩回 512 维）
- Layer Normalization + Residual Connection

Encoder 对所有输入 token（latent + state + 各像素的图像特征）进行 self-attention，让模型学习不同模态之间的交互关系——例如图像中的"杯子"应该和关节状态中的"夹爪靠近杯子"产生关联。

**步骤五：Transformer Decoder**。Decoder 的输入是 $K$ 个可学习的 query token（`decoder_pos_embed`），每个 query 对应未来一个时间步的动作预测。Decoder 通过 cross-attention 机制从 Encoder 的输出中"查询"相关信息。具体来说：

- Self-Attention：query 之间互相通信，确保预测的动作序列内部保持一致。
- Cross-Attention：query 去 attend Encoder 的输出，从中提取与当前时间步最相关的视觉和状态信息。

Decoder 只有 1 层（这是一个历史遗留问题——原始 ACT 实现宣称有 7 层但因代码 bug 实际上只用到了第 1 层，LeRobot 的实现保留了这个设定）。

**步骤六：动作回归头**。Decoder 的每个输出 token（维度 512）通过一个线性层 (`action_head`) 投影到动作空间（7 维），得到最终的 $K \times 7$ 的动作 chunk。

### ResNet-18 Backbone

ACT 选择 ResNet-18 作为视觉 backbone，这并非因为 ResNet-18 是"最好"的视觉模型，而是出于精确的工程考虑：

- **轻量**：ResNet-18 只有约 11M 参数，远小于 ResNet-50（25M）或 ViT（86M+）。在实际部署中，推理速度直接影响控制回路的延迟。
- **ImageNet 预训练**：通过加载 `ResNet18_Weights.IMAGENET1K_V1` 预训练权重，ResNet-18 已经具备了通用的视觉特征提取能力——它在 ImageNet 上学会了识别边缘、纹理、形状等基础视觉元素，这些能力可以直接迁移到机器人操作场景中。
- **冻结 BatchNorm**：在 LeRobot 的实现中，ResNet 的 BatchNorm 层被替换为 FrozenBatchNorm2d（`norm_layer=FrozenBatchNorm2d`），意味着在训练过程中 BatchNorm 的统计量始终使用 ImageNet 预训练的值，不会根据机器人数据更新。这有助于在小数据集上稳定训练。
- **可选的膨胀卷积**：`replace_final_stride_with_dilation` 参数允许将 ResNet 最后一层的 2x2 stride 替换为膨胀卷积 (dilated convolution)，以保留更高的空间分辨率。这在对位置敏感的精细操作任务中可能会有帮助，但默认保持关闭以降低计算量。

### Transformer 详解

Transformer 是 ACT 的核心处理模块。理解它对于理解 ACT 为什么有效至关重要。

**Self-Attention 的本质**。Self-Attention 允许输入序列中的每个元素（token）与序列中的所有其他元素进行交互。在 ACT 中：

- Encoder 的 Self-Attention 让不同来源的信息——来自腕部相机的像素特征、来自全局相机的像素特征、当前关节状态——彼此"沟通"。例如，腕部相机中看到"夹爪旁边有一个红色积木"，这个视觉 token 可以和"关节 4 的角度是 45 度"这个状态 token 通过 attention 建立关联。
- Decoder 的 Self-Attention 让不同时间步的 query 之间相互协调。例如，第 5 步的 query 知道第 4 步预测了什么动作，从而保证动作序列的时序一致性。

**Cross-Attention**。Decoder 中的 Cross-Attention 是实现"条件生成"的关键。Decoder 的 query（代表"我想知道第 t 步该做什么"）去 attend Encoder 的所有输出 token，从中提取相关的信息。注意力权重 $\alpha_{i,j}$ 表示第 $i$ 个 query 对第 $j$ 个 encoder token 的关注程度：

$$\text{Attention}(Q, K, V) = \text{softmax}\left(\frac{QK^T}{\sqrt{d_k}}\right)V$$

其中 $Q$ 来自 Decoder query，$K$ 和 $V$ 来自 Encoder 输出。$\sqrt{d_k}$ 是缩放因子，防止点积过大导致 softmax 梯度消失。

**VAE (Variational Auto-Encoder)**。ACT 在训练时使用了一个额外的 Transformer（称为 VAE Encoder，不要和主 Transformer Encoder 混淆）来引入变分目标。VAE Encoder 的作用是：

1. 将未来的真实动作序列编码为一个低维的 latent 向量（32 维）。
2. 训练时，主 Transformer 的解码器基于这个 latent 向量来重建动作序列。
3. 同时，最小化 latent 分布与标准正态分布之间的 KL 散度，使 latent 空间更加规整。

这样做的目的是让模型不仅仅学习"给定观测，输出动作"的确定性映射，还能学习到动作序列的概率分布——这有助于处理多模态问题：同一个观测下可能有多种合理动作，VAE 的随机采样可以覆盖多种模式。

然而，VAE 相关代码在推理时并不使用（推理时 latent 设置为全零向量），因此 VAE 更像是一个训练正则化手段而非推理时的必要组件。`use_vae` 参数可以关闭 VAE。

### 损失函数

ACT 的损失函数由两部分组成：

**L1 损失（Smooth L1 / Huber）**。与标准的行为克隆使用 MSE（L2 Loss）不同，ACT 使用 L1 Loss：

$$\mathcal{L}_{L1} = \frac{1}{K \cdot d} \sum_{k=0}^{K-1} \sum_{j=0}^{d-1} |\hat{a}_{t+k}^{(j)} - a_{t+k}^{(j)}|$$

其中 $K$ 是 chunk_size，$d=7$ 是动作维度。使用 L1 Loss 而非 L2 Loss 的原因是：L1 对异常值（outlier）不敏感。在人类演示数据中，偶尔会有突兀的大幅动作（如突然松开夹爪），L2 Loss 会将这些异常值的影响放大（平方效应），而 L1 Loss 的梯度是常数，不会被个别异常值主导。

在代码中，L1 损失的计算还会考虑 padding 掩码：数据集中不同 episode 的长度不同，短的 episode 会被 padding 到统一长度，这些 padding 位置不参与损失计算：

```python
l1_loss = (
    F.l1_loss(batch[ACTION], actions_hat, reduction="none") * ~batch["action_is_pad"].unsqueeze(-1)
).mean()
```

**KL 散度损失（仅在 use_vae=True 时生效）**：

$$\mathcal{L}_{KL} = -\frac{1}{2} \sum_{j=1}^{L} \left(1 + \log\sigma_j^2 - \mu_j^2 - \sigma_j^2\right)$$

其中 $L=32$ 是 latent 维度，$\mu$ 和 $\log\sigma^2$ 是 VAE Encoder 输出的分布参数。KL 散度衡量 latent 分布与标准正态分布 $\mathcal{N}(0, I)$ 的差异。总损失为：

$$\mathcal{L} = \mathcal{L}_{L1} + \beta \cdot \mathcal{L}_{KL}$$

其中 $\beta$ 是 `kl_weight`，默认值为 10.0。

### 关键超参数

以下是在 PiPER 项目中使用的典型 ACT 超参数配置：

| 参数 | 典型值 | 含义 |
|------|--------|------|
| `chunk_size` | 30 | 每次推理预测多少步未来动作 |
| `n_action_steps` | 30 | 每次推理后实际执行多少步（$\leq$ chunk_size） |
| `learning_rate` | 1e-5 | AdamW 优化器的学习率 |
| `optimizer_weight_decay` | 1e-4 | 权重衰减系数 |
| `batch_size` | 8 | 每次训练的 batch 大小，取决于 GPU 显存 |
| `steps` | 20000 | 总训练步数 |
| `n_encoder_layers` | 4 | Transformer Encoder 的层数 |
| `n_decoder_layers` | 1 | Transformer Decoder 的层数 |
| `dim_model` | 512 | Transformer 的隐藏维度 |
| `n_heads` | 8 | Multi-head Attention 的头数 |
| `dim_feedforward` | 3200 | Feed-forward 层的扩展维度 |
| `latent_dim` | 32 | VAE 的 latent 空间维度 |
| `dropout` | 0.1 | Transformer 中的 Dropout 率 |
| `kl_weight` | 10.0 | KL 散度在总损失中的权重 |
| `n_obs_steps` | 1 | 每次推理使用多少个历史观测帧 |
| `temporal_ensemble_coeff` | 0.01 | 时间集成的指数衰减系数 |

---

## 9.3 时间集成 (Temporal Ensemble)

### 问题：Chunk 边界的不连续性

虽然 ACT 一次性预测一个完整的动作 chunk（30 步），但在实际部署中，我们可能希望在 chunk 之间重新推理以获得更好的控制效果。假设 `n_action_steps=1`（每帧都推理）：

- 在 $t=0$ 时刻，策略输出 chunk $\mathbf{A} = [a_0, a_1, \ldots, a_{29}]$，执行 $a_0$。
- 在 $t=1$ 时刻，策略输入新的观测，输出新 chunk $\mathbf{B} = [b_0, b_1, \ldots, b_{29}]$。

问题在于：$a_1$（来自第一次推理，预测的是时刻 1 的动作）和 $b_0$（来自第二次推理，预测的也是时刻 1 的动作）可能不相等。由于两次推理的输入观测略有不同，或者模型本身的不确定性，这两个预测值之间会有差异。如果我们直接在当前帧无缝切换到新 chunk 的第一个动作，就可能产生动作跳变 (jump)。

### 解决：加权平均重叠时间步

时间集成 (Temporal Ensemble) 的核心思想是：**对同一时刻来自不同推理的多组预测进行指数加权平均**。这正是 ACT 原论文中 Algorithm 2 描述的方法。

具体而言，对于时刻 $t$ 的实际执行动作，它是所有"覆盖了时刻 $t$ 的预测 chunk"中对应位置动作的加权平均：

$$\text{executed\_action}[t] = \frac{\sum_i w_i \cdot \text{predicted\_action}_i[t]}{\sum_i w_i}$$

其中 $i$ 是推理的索引（$i=0$ 表示最新的一次推理），权重 $w_i$ 按指数衰减：

$$w_i = \exp(-\lambda \cdot i)$$

$\lambda$ 是 `temporal_ensemble_coeff`（默认 0.01）。注意这个系数的含义是：**$i$ 越大代表推理越旧**，`temporal_ensemble_coeff` 为正时，旧推理的权重更高（`exp(-0.01 * i)` 接近 1 时权重更大）。这看似反直觉——为什么旧推理权重要更高？原论文的实验表明，过高的新推理权重会削弱 action chunking 的时间平滑优势，让动作序列变得过于 responsive 而失去连贯性。

### 在线计算算法

在代码实现中（`ACTTemporalEnsembler` 类），时间集成并非在每次推理时重新计算整个加权平均（那样需要缓存所有历史预测 chunk，内存开销太大），而是使用在线递推方式：

```python
class ACTTemporalEnsembler:
    def __init__(self, temporal_ensemble_coeff, chunk_size):
        self.chunk_size = chunk_size
        # 预计算所有权重和累积权重
        self.ensemble_weights = torch.exp(-temporal_ensemble_coeff * torch.arange(chunk_size))
        self.ensemble_weights_cumsum = torch.cumsum(self.ensemble_weights, dim=0)

    def update(self, actions):
        if self.ensembled_actions is None:
            # 第一次推理：直接存储整个 chunk
            self.ensembled_actions = actions.clone()
            self.ensembled_actions_count = torch.ones((chunk_size, 1))
        else:
            # 在线更新：将现有 ensemble 乘以旧权重，加入新预测乘以新权重
            # （实际实现有更精细的处理，包括处理 chunk 的截断和拼接）
            ...

        # 弹出并返回第一个动作
        action = self.ensembled_actions[:, 0]
        self.ensembled_actions = self.ensembled_actions[:, 1:]
        return action
```

### 使用条件

要启用时间集成，需要同时满足：
- `temporal_ensemble_coeff` 不为 `None`（典型值 0.01）。
- `n_action_steps` 必须等于 1——因为时间集成需要每帧都进行推理以形成 ensemble。当 `n_action_steps > 1` 时策略内置的 action queue 接管（参见下文"动作队列"）。

### 动作队列 (Action Queue)

当不使用时间集成时（`temporal_ensemble_coeff` 为 `None`），ACT 使用更简单的动作队列机制：推理一次得到 $K$ 步动作后，全部放入一个队列，然后逐帧弹出执行，直到队列耗尽再触发下一次推理。这适用于对推理延迟敏感的场景——推理一次管 30 步，大大降低了 GPU 占用。

---

## 9.4 训练流程

### server_train_act_from_dataset.sh 详解

PiPER 项目提供了一个完整的训练脚本 `server_train_act_from_dataset.sh`，用于在训练服务器上启动 ACT 训练。下面逐段解析该脚本。

#### 1. 环境配置

```bash
PIPER_ROOT="${PIPER_ROOT:-/root/autodl-tmp/Piper}"
LEROBOT_DIR="${LEROBOT_DIR:-${PIPER_ROOT}/lerobot_piper-piper}"
DATASET_DIR="${DATASET_DIR:-}"
OUTPUT_BASE="${OUTPUT_BASE:-${PIPER_ROOT}/outputs/train}"
JOB_NAME="${JOB_NAME:-piper_act}"
OUTPUT_DIR="${OUTPUT_DIR:-${OUTPUT_BASE}/${JOB_NAME}_$(date +%Y%m%d_%H%M%S)}"
```

脚本使用 Bash 的 `${var:-default}` 语法，为每个关键路径提供默认值。`OUTPUT_DIR` 自动加入时间戳以避免多次训练相互覆盖。核心可调参数通过环境变量传入：

```bash
POLICY_DEVICE="${POLICY_DEVICE:-cuda}"
BATCH_SIZE="${BATCH_SIZE:-8}"
STEPS="${STEPS:-20000}"
CHUNK_SIZE="${CHUNK_SIZE:-30}"
N_ACTION_STEPS="${N_ACTION_STEPS:-30}"
VIDEO_BACKEND="${VIDEO_BACKEND:-pyav}"
PRETRAINED_BACKBONE_WEIGHTS="${PRETRAINED_BACKBONE_WEIGHTS:-ResNet18_Weights.IMAGENET1K_V1}"
```

#### 2. 数据集验证

训练开始前，脚本通过一段内联 Python 代码验证数据集是否可以正确加载：

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ds = LeRobotDataset(
    repo_id="${DATASET_REPO_ID}",
    root="${DATASET_DIR}",
    video_backend="${VIDEO_BACKEND}",
)
print("num_frames:", ds.num_frames)
print("num_episodes:", ds.num_episodes)
print("fps:", ds.fps)
sample = ds[0]
print("action:", tuple(sample["action"].shape))
print("state:", tuple(sample["observation.state"].shape))
for key in sorted(k for k in sample if k.startswith("observation.images.")):
    print(key + ":", tuple(sample[key].shape))
```

这段代码输出数据集的帧数、episode 数、帧率，以及第一个样本中各模态的形状。如果数据集的 `meta/info.json` 缺失或格式不符合预期，这一步会报错并中止训练——避免了训练到一半才发现数据问题的情况。

#### 3. 训练命令

```bash
lerobot-train \
    --policy.type=act \
    --policy.device="${POLICY_DEVICE}" \
    --policy.push_to_hub=false \
    --policy.chunk_size="${CHUNK_SIZE}" \
    --policy.n_action_steps="${N_ACTION_STEPS}" \
    --dataset.repo_id="${DATASET_REPO_ID}" \
    --dataset.root="${DATASET_DIR}" \
    --dataset.video_backend="${VIDEO_BACKEND}" \
    --batch_size="${BATCH_SIZE}" \
    --steps="${STEPS}" \
    --log_freq="${LOG_FREQ}" \
    --save_freq="${SAVE_FREQ}" \
    --eval_freq="${EVAL_FREQ}" \
    --num_workers="${NUM_WORKERS}" \
    --output_dir="${OUTPUT_DIR}" \
    --job_name="${JOB_NAME}" \
    --wandb.enable=false
```

各参数的含义：
- `--policy.type=act`：指定使用 ACT 策略。
- `--policy.device=cuda`：在 GPU 上训练。
- `--policy.push_to_hub=false`：不自动推送到 HuggingFace Hub（本地训练场景）。
- `--dataset.repo_id` 和 `--dataset.root`：分别指定数据集在 HuggingFace Hub 上的标识符和本地存储路径。
- `--wandb.enable=false`：默认关闭 Weights & Biases 日志（可以按需开启）。

ResNet 预训练权重的处理有一条特殊逻辑：

```bash
if [[ "${PRETRAINED_BACKBONE_WEIGHTS}" == "null" || ... ]]; then
  ARGS+=(--policy.pretrained_backbone_weights=null)
else
  ARGS+=(--policy.pretrained_backbone_weights="${PRETRAINED_BACKBONE_WEIGHTS}")
fi
```

这允许用户选择从头训练 ResNet（传入 `null`）或使用 ImageNet 预训练权重（默认）。

### 训练监控

训练过程中，以下参数控制日志和模型保存的频率：

- `--log_freq=100`：每 100 步输出一次训练指标（loss、学习率等）。
- `--save_freq=5000`：每 5000 步保存一个 checkpoint。
- `--eval_freq=0`：默认为 0 表示不在训练过程中进行评估（评估通常离线进行）。
- 输出目录结构：
  ```
  outputs/train/piper_act_20250101_120000/
  ├── checkpoints/      # 模型权重和配置文件
  │   ├── 0005000/
  │   ├── 0010000/
  │   └── ...
  └── logs/             # 训练日志
  ```

每个 checkpoint 目录包含：
- `config.json`：策略的完整配置
- `model.safetensors`：模型权重
- `preprocessor.json`：预处理 pipeline 配置（归一化参数等）
- `postprocessor.json`：后处理 pipeline 配置（反归一化参数等）

### 训练配置的底层机制

在 LeRobot 框架中，`TrainPipelineConfig`（定义在 `src/lerobot/configs/train.py`）是整个训练流程的总配置类。它使用 `draccus` 库将命令行参数与 `dataclass` 字段绑定，实现了类型安全的参数解析。训练 pipeline 配置的核心字段包括：

```python
@dataclass
class TrainPipelineConfig(HubMixin):
    dataset: DatasetConfig
    policy: PreTrainedConfig | None = None
    output_dir: Path | None = None
    job_name: str | None = None
    resume: bool = False
    seed: int | None = 1000
    num_workers: int = 4
    batch_size: int = 8
    steps: int = 100_000
    eval_freq: int = 20_000
    log_freq: int = 200
    save_freq: int = 20_000
```

当 `use_policy_training_preset=True` 时，优化器和学习率调度器的配置直接从策略类中获取（`self.policy.get_optimizer_preset()` 和 `self.policy.get_scheduler_preset()`），确保训练超参数与策略设计者的建议保持一致。对于 ACT，`get_optimizer_preset` 返回 `AdamWConfig(lr=1e-5, weight_decay=1e-4)`，`get_scheduler_preset` 返回 `None`（不使用学习率调度器）。

---

## 9.5 策略部署

### piper_act_safe_rollout.py 详解

训练完成后，需要将策略部署到真实机器人上执行推理。PiPER 项目提供了 `piper_act_safe_rollout.py` 脚本，其名称中的 "safe" 暗示了其核心设计理念——**安全第一，先验证后执行**。

#### 整体架构

脚本的执行流程可以概括为以下几个阶段：

```
初始化阶段:
  1. 解析命令行参数 (parse_args)
  2. 验证 checkpoint 完整性 (require_checkpoint)
  3. 加载 ACT 策略和预处理/后处理 pipeline (load_policy)
  4. 创建 PiperFollower 实例并连接从臂 (make_robot + robot.connect)
  5. 映射 robot 的键名到 LeRobot 标准名称 (map_robot_keys_to_lerobot_features)

推理循环 (每步):
  1. robot.get_observation() → 原始观测 (图像 + 关节位置)
  2. 将原始观测转换为 LeRobot 格式 (raw_observation_to_observation)
  3. 预处理 (preprocessor): 归一化 + 加 batch 维度 + 移入设备
  4. policy.select_action(obs) → K 步动作 chunk
  5. 后处理 (postprocessor.process_action): 反归一化 + 移回 CPU
  6. limit_action(): 增量限幅安全检查
  7. smooth_action(): EMA 平滑
  8. 如果是执行模式，robot.send_action(smoothed) → 发送到从臂
```

#### 干运行模式 (--dry-run)

脚本的默认行为是干运行 (dry run)——不添加 `--execute` 参数时，策略会完整地执行推理、限幅、平滑等所有计算，但**不会**将动作发送给机器人。终端输出如下格式：

```
[000] current=[  30.00,   45.00,  ...,   50.00] pred=[  32.15,   46.20,  ...,   51.30] limited=[  32.15,   46.20,  ...,   51.30] cmd=[  32.15,   46.20,  ...,   51.30]
```

每列的含义：
- `current`：机械臂当前的实际关节位置
- `pred`：策略的原始预测值
- `limited`：经过增量限幅 (limit_action) 后的值
- `cmd`：经过 EMA 平滑 (smooth_action) 后的最终命令值

通过对比 `pred` 和 `cmd`，可以直观地评估限幅和平滑是否对策略输出产生了显著影响。如果两者差异很大，说明策略输出的幅度超出了安全阈值。

#### limit_action() — 增量限幅

增量限幅是保障安全的第一道防线。它限制每一步的动作变化量不超过预设阈值：

```python
def limit_action(predicted, current, max_delta, max_gripper_delta, disable_gripper):
    limited = {}
    for key in ACTION_KEYS:
        delta_limit = max_gripper_delta if key == "gripper.pos" else max_delta
        abs_low, abs_high = (0.0, 100.0) if key == "gripper.pos" else (-100.0, 100.0)
        if key == "gripper.pos" and disable_gripper:
            limited[key] = float(np.clip(current[key], abs_low, abs_high))
            continue
        delta = np.clip(predicted[key] - current[key], -delta_limit, delta_limit)
        limited[key] = float(np.clip(current[key] + float(delta), abs_low, abs_high))
    return limited
```

工作原理：
1. 计算预测动作与当前位置的差值 $\Delta = a_{\text{pred}} - a_{\text{current}}$。
2. 将 $\Delta$ clamp 到 $[-\text{max\_delta}, +\text{max\_delta}]$ 范围内。
3. 最终动作 $a_{\text{cmd}} = a_{\text{current}} + \text{clamp}(\Delta)$。
4. 再对绝对值进行一次 clamp（关节角度的合法范围是 [-100, 100] 度，夹爪是 [0, 100]）。

`--max-delta` 的默认值为 2.0（关节角度单位），这意味着每步（0.2 秒，如果 control_fps=5）关节最多移动 2 个单位。这样即使策略输出了一个极端的预测值（例如由于 chunk 切换时的 jump 或模型对异常观测的不合理响应），机器人也不会突然大幅度运动。

夹爪有独立的 `--max-gripper-delta` 参数（默认 3.0），因为夹爪的开合动作通常比关节转动更剧烈。此外，`--disable-gripper` 开关允许在测试手臂关节时固定夹爪位置，避免夹爪意外开合导致抓取的物体掉落。

#### smooth_action() — EMA 平滑

EMA（指数移动平均）是额外的平滑层，用于降低动作序列中的高频抖动：

```python
def smooth_action(action, previous, alpha):
    if previous is None or alpha >= 1.0:
        return dict(action)
    if alpha <= 0.0:
        return dict(previous)
    return {key: alpha * action[key] + (1.0 - alpha) * previous[key] for key in ACTION_KEYS}
```

EMA 的递推公式为：

$$a_{\text{smooth}}^{(t)} = \alpha \cdot a_{\text{new}}^{(t)} + (1 - \alpha) \cdot a_{\text{smooth}}^{(t-1)}$$

其中 $\alpha$ 是 `--ema-alpha` 参数（默认 1.0 表示不启用平滑）。当 $\alpha < 1$ 时，当前执行的命令不仅是当前策略预测的结果，还"记忆"了上一步的命令——$\alpha$ 越小，对历史命令的依赖越强，输出越平滑，但响应也越慢。

**EMA 与 Temporal Ensemble 的区别**：
- Temporal Ensemble 由策略内部实现（`ACTTemporalEnsembler`），在动作空间中对多个推理 chunk 的重叠部分进行加权平均——解决的是跨 chunk 的一致性问题。
- EMA 是在策略输出已经确定之后，在命令发送给机器人之前额外施加的时间平滑——解决的是帧间高频抖动问题。

两者可以同时使用，形成双层平滑。在 chunk 边界（即从队列中弹出下一帧动作时），EMA 能够显著减少相邻两帧之间的不连续性。

#### 完整推理循环

推理循环的核心代码结构如下（简化版）：

```python
previous_limited = None
period = 1.0 / args.control_fps  # 控制周期，如 0.2 秒

for step in range(args.steps):
    loop_start = time.perf_counter()

    # 1. 获取观测
    raw_obs = robot.get_observation()
    obs = raw_observation_to_observation(raw_obs, lerobot_features,
                                          cfg.image_features, args.device)
    obs = preprocessor(obs)

    # 2. 策略推理
    with torch.inference_mode():
        raw_action = policy.select_action(obs)
        action = postprocessor.process_action(raw_action)

    # 3. 转换格式
    predicted = action_tensor_to_dict(action)
    current = current_state_from_obs(raw_obs)

    # 4. 安全检查
    limited = limit_action(predicted, current,
                           args.max_delta, args.max_gripper_delta,
                           args.disable_gripper)
    smoothed = smooth_action(limited, previous_limited, args.ema_alpha)
    previous_limited = smoothed

    # 5. 执行（仅在 --execute 模式下）
    if args.execute:
        robot.send_action(smoothed)

    # 6. 帧率控制
    elapsed = time.perf_counter() - loop_start
    time.sleep(max(0.0, period - elapsed))
```

关键设计要点：
- `torch.inference_mode()` 上下文管理器禁用梯度计算和 autograd 跟踪，减少推理时的内存和计算开销。
- `time.perf_counter()` 精确测量每步耗时，通过 `time.sleep` 补足到目标控制周期，保持稳定的控制帧率。
- `robot.disconnect(disable_torque=False)` 在脚本退出时（包括 Ctrl+C 中断）确保从臂不断电（保持当前姿态），防止因突然断电导致机械臂坠落。

---

## 9.6 为什么 ACT 适合双臂操作

双臂操作 (Bimanual Manipulation) 是机器人学习中最具挑战性的场景之一。ACT 的设计恰好针对了双臂操作的几个关键需求：

**协调性**：双臂操作要求左右臂的动作严格协调——例如双手配合拧开瓶盖，左臂固定瓶身、右臂旋开瓶盖，两个动作在时序上有精确的依赖关系。如果只预测单步动作，模型需要隐式地学习这种协调性；而 ACT 预测完整的动作序列，可以在 chunk 内部通过 Decoder 的 Self-Attention 显式地协调不同时间步的动作。

**时间尺度匹配**：典型的操作子任务——如抓起物体、移动到目标位置、放下——通常持续 1 到数秒。`chunk_size=30`（在 30 fps 下为 1 秒）恰好覆盖了这些短时操作子任务的时间跨度。如果只需要 0.5 秒的子任务，30 步 chunk 提供了足够的余量；如果需要 2 秒的复杂操作，可以在 chunk 之间重新推理以更新策略。

**推理频率降低**：双臂操作的物理约束（惯性、摩擦、电机转速）意味着不需要每毫秒都调整控制指令。`n_action_steps=30` 配置下，模型推理一次可以管 1 秒的动作，远比双臂物理系统的时间常数长。这降低了对 GPU 推理延迟的要求，也减少了计算开销。

**对演示噪声的鲁棒性**：人类演示者在遥操作时不可避免地会产生手抖、犹豫、纠正等噪声。ACT 的 L1 Loss 和 action chunking 机制对这类噪声具有天然的鲁棒性——L1 不会被个别的突兀动作主导损失，而 chunking 让模型关注的是整段轨迹的"形状"而非每个点的精确值。

---

## 9.7 ACT 的局限性

尽管 ACT 在双臂操作任务中表现出色，但它并非没有局限：

**数据需求大**：ACT 通常需要 50-200 个 episode 的高质量演示才能学到可靠的行为。数据采集是遥操作机器人项目中最耗时的环节——每个 episode 可能需要数分钟的人工操作，而且需要覆盖不同的初始条件、物体位置和光照变化。数据不足时，ACT 学到的策略容易过拟合训练场景中的特定视觉模式。

**对分布外 (Out-of-Distribution) 状态的泛化弱**：ACT 是一个确定性策略（忽略 VAE 的随机采样），当机器人偏离训练数据中的状态分布时（例如物体被移动到了训练时从未见过的位置），策略的输出可能变得不可靠。这是所有模仿学习方法的通病——当策略逐步偏离演示轨迹时，它面临的是训练数据中从未出现过的观测。

**不建模物理约束**：ACT 的神经网络完全不理解物理世界的规律——它不知道机械臂有最大扭矩限制，不知道夹爪不能穿透物体，不知道桌面是一个刚性平面。这些约束全部依赖人类演示数据中的"常识"来隐式编码。当策略输出一个违反物理规律的动作时，唯一的安全保障是部署脚本中的 `limit_action()` 和 `smooth_action()` 等启发式规则。

**只能复现见过的行为模式**：如果训练数据中只有"从左向右推"的演示，ACT 无法自主学会"从右向左拉"。它不具备组合泛化或推理能力——不会有类似"既然学会了推，倒过来应该也能拉"的 generalize。这使得 ACT 更像是一个动作回放引擎（有条件的回放）而非一个真正"理解"任务的智能体。

**长序列的 VAE 编码效率**：32 维的 latent 向量需要编码 30 步 $\times$ 7 维 = 210 个浮点数的动作序列信息，压缩比接近 7:1。对于复杂的、变化丰富的动作序列，这个压缩可能导致信息丢失。

---

## 9.8 其他 LeRobot 支持的策略

LeRobot 框架不仅支持 ACT，还集成了多种策略算法，适用于不同的机器人和任务场景。以下是简要对比：

| 策略 | 类型 | 核心特点 | 适用场景 |
|------|------|----------|----------|
| **ACT** | 模仿学习 | Action Chunking + Transformer + VAE | 双臂操作、精细任务 |
| **Diffusion Policy** | 模仿学习 | 扩散模型生成动作分布，天然处理多模态 | 高精度位置控制、需要多模态输出的任务 |
| **TDMPC** | 模型预测控制 | 基于模型的规划，在隐空间中搜索最优动作序列 | 需要长期规划的任务、接触丰富的操作 |
| **VQ-BET** | 模仿学习 | 使用 Vector Quantization 对动作离散化，显式建模多模态 | 行为模式多样的任务（如多种抓取策略） |
| **SmolVLA** | 视觉语言动作 | 跨任务泛化，支持语言指令 | 需要跨任务泛化、多任务学习的场景 |
| **SAC** | 强化学习 | Actor-Critic 架构，最大化累积 reward | 仿真环境中的训练（需要 reward 函数） |

选择策略时的主要考量：
- 如果有高质量的演示数据且任务相对单一，**ACT** 是一个稳定和可靠的首选。
- 如果同一状态存在多种同样正确的动作（例如抓取不同部位），**Diffusion Policy** 或 **VQ-BET** 对多模态的处理更好。
- 如果任务涉及接触物理、需要精确的力控或轨迹规划，**TDMPC** 的模型预测能力可能更合适。
- 如果希望一个策略处理多种不同的任务，**SmolVLA** 的多任务能力值得尝试。

---

## 9.9 常见问题

### 策略输出抖动

**现象**：机器人动作不流畅，关节在相邻帧之间来回振荡，尤其是在 chunk 边界处。

**原因**：连续两次推理产生的 action chunk 在重叠位置不完全一致，或者模型本身对观测的微小变化过于敏感。

**解决方案**：
1. 降低 `--max-delta`（如从 2.0 降到 1.0），增加增量限幅的强度。
2. 降低 `--ema-alpha`（如从 1.0 降到 0.5），启用更激进的 EMA 平滑。
3. 在策略配置中启用 temporal ensemble（设置 `temporal_ensemble_coeff=0.01` 且 `n_action_steps=1`）。

### 策略不跟随

**现象**：机械臂几乎不动，或者动作与人类期望完全不符。

**可能原因**：
1. 加载了错误的 checkpoint（例如不同任务或不同机器人的模型）。
2. 数据集统计信息（归一化参数）与 checkpoint 不匹配。
3. 相机没有正确初始化或图像格式不匹配。

**排查步骤**：
1. 检查 checkpoint 目录中是否包含 `config.json`、`model.safetensors`、`preprocessor.json`、`postprocessor.json` 四个文件——脚本中 `require_checkpoint()` 会验证这一点。
2. 启用干运行模式（默认行为），观察 `pred` 列的值是否有意义——如果所有预测值都接近零或某个常数，可能是归一化/反归一化出了问题。
3. 确认数据集使用的 `DATASET_REPO_ID` 与训练时一致。
4. 确认相机分辨率 (`--width`, `--height`) 与训练时的分辨率一致。

### Out of Memory (OOM)

**现象**：训练时报错 `CUDA out of memory`。

**原因**：batch_size 或 chunk_size 超出了 GPU 显存容量。ACT 的内存占用主要来自：
- 图像 batch（$B \times 2 \times 3 \times H \times W$）
- Transformer 的激活值（与 $K$ 和序列长度成正比）

**解决方案**：
1. 减小 `--batch_size`（如从 8 降到 4 或 2）。
2. 减小 `--chunk_size`（如从 30 降到 15）——但这会改变策略的语义，需要重新训练。
3. 降低图像分辨率（如从 640x480 降到 320x240）。
4. 如果 GPU 显存实在不足，可以使用 `--policy.device=cpu` 进行纯 CPU 训练（非常慢，仅用于验证流程）。

### 推理太慢

**现象**：每步推理耗时过长，导致控制帧率无法达到设定值（如 5 fps）。

**原因**：ResNet-18 虽然是轻量 backbone，但在 CPU 上推理 640x480 的双相机图像仍然较慢；或者 GPU 推理时显存不足导致内存交换。

**解决方案**：
1. 使用 GPU 推理 (`--device cuda`) 而非 CPU。
2. 降低相机分辨率（`--width 320 --height 240`），注意需要与训练时的分辨率一致，或重新训练。
3. 增大 `n_action_steps`（如设为 30），让策略推理一次后使用较长时间的动作队列，减少推理频率。
4. 如果图像预处理 pipeline 耗时长，可以将其与推理解耦到单独的线程中。

### 机械臂在退出时坠落

**现象**：停止脚本后机械臂失去力矩 (torque) 而自然下落。

**原因**：`robot.disconnect()` 被调用时传入了 `disable_torque=True`，或者在异常退出时没有正确调用 `disconnect()`。

**解决方案**：
- 脚本的 `finally` 块中使用了 `robot.disconnect(disable_torque=False)`，确保从臂在断开连接时保持力矩。不要修改这个参数。
- 如果机械臂仍然坠落，检查从臂的物理连接（CAN 总线线缆是否松动）和电源状态。

---

## 小结

本章从模仿学习的基本概念出发，深入讲解了 Action Chunking Transformer (ACT) 的原理、架构、训练流程和部署方法。ACT 通过 action chunking 机制解决了传统行为克隆中的复合误差问题，通过 Transformer 架构实现了多模态信息的融合和时序动作的协调生成，通过时间集成 (temporal ensemble) 保证了推理时的动作平滑性。

在实际应用中，ACT 需要对硬件安全有充分的考虑——本章介绍的 `piper_act_safe_rollout.py` 脚本通过干运行模式、增量限幅和 EMA 平滑三道防线，确保策略的输出在发送给物理机器人之前经过了充分的约束和过滤。

至此，本教程的理论部分完结。后续章节将聚焦于具体实验、性能分析和进阶优化。
