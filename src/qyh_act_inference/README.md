# QYH ACT Inference

ACT (Action Chunking Transformer) 推理部署包，用于具身智能机器人的实时模型推理和控制。

## 📋 功能特性

- ✅ **ACT 模型推理**：加载训练好的 ACT 模型，实时推理
- ✅ **动作安全处理**：缩放、平滑、限位、夹爪二值化
- ✅ **任务引擎集成**：作为技能节点集成到 `qyh_task_engine`
- ✅ **多种配置模板**：右手单臂、双臂协同、调试模式
- ✅ **ROS2 服务接口**：启动/停止/重置/加载模型

## 🏗️ 架构

```
qyh_act_inference/
├── qyh_act_inference/
│   ├── __init__.py
│   ├── inference_config.py    # 配置类（调参核心）
│   ├── act_policy.py          # ACT 模型加载和推理
│   ├── action_executor.py     # 动作执行器（安全处理）
│   └── act_skill_node.py      # 任务引擎技能节点
├── scripts/
│   ├── act_inference_node.py  # ROS2 推理节点
│   └── test_act_inference.py  # 测试脚本
├── launch/
│   └── act_inference.launch.py
├── config/
│   └── act_inference.yaml
├── CMakeLists.txt
└── package.xml
```

## 🚀 快速开始

### 1. 编译

```bash
cd ~/qyh-robot-system/qyh_jushen_ws
colcon build --packages-select qyh_act_inference
source install/setup.bash
```

### 2. 准备模型

将训练好的模型放到指定目录：

```bash
# 创建模型目录
mkdir -p ~/qyh-robot-system/models/pickup_cube

# 复制模型文件
cp /path/to/policy.pt ~/qyh-robot-system/models/pickup_cube/
cp /path/to/normalization.npz ~/qyh-robot-system/models/pickup_cube/
```

### 3. 启动推理节点

```bash
# 基本启动
ros2 launch qyh_act_inference act_inference.launch.py

# 指定模型路径
ros2 launch qyh_act_inference act_inference.launch.py \
    model_path:=~/qyh-robot-system/models/pickup_cube/policy.pt

# 调试模式（保守参数）
ros2 launch qyh_act_inference act_inference.launch.py \
    action_scale:=0.2 \
    device:=cpu
```

### 4. 控制推理

```bash
# 加载模型
ros2 service call /act_inference_node/load_model std_srvs/srv/Trigger

# 启动推理
ros2 service call /act_inference_node/start std_srvs/srv/SetBool "{data: true}"

# 停止推理
ros2 service call /act_inference_node/start std_srvs/srv/SetBool "{data: false}"

# 重置
ros2 service call /act_inference_node/reset std_srvs/srv/Trigger
```

## ⚙️ 调参指南

### 关键参数（按重要性排序）

| 参数 | 默认值 | 说明 | 调参建议 |
|-----|-------|------|---------|
| `action_scale` | 0.4 | 动作缩放因子 | 初始 0.2-0.3，稳定后 0.5-1.0 |
| `smoothing_alpha` | 0.3 | EMA 平滑系数 | 0.2（更平滑）- 0.5（更灵敏）|
| `max_joint_delta` | 0.05 | 单步最大变化 (rad) | 保持默认或更小 |
| `action_steps` | 3 | 每次执行动作数 | 2-5，越大越快 |
| `gripper_threshold` | 0.6 | 夹爪阈值 | 根据任务调整 |

### 调参顺序

1. **离线 Replay**：用录制的数据验证推理方向
2. **空载测试**：机械臂悬空，action_scale=0.1
3. **慢速模式**：action_scale=0.2，hz=10
4. **正常模式**：逐步提高 scale 和频率

### 常见问题

1. **动作抖动**
   - 降低 `smoothing_alpha` (0.2)
   - 降低 `action_scale`
   - 增加 `max_joint_delta` 限制

2. **夹爪抖动**
   - 增加 `gripper_hysteresis` (0.15)
   - 检查训练数据中夹爪标注

3. **动作太慢**
   - 提高 `action_scale`
   - 增加 `action_steps`
   - 提高 `control_frequency`

## 🎯 任务引擎集成

### 在任务 JSON 中使用 ACT

```json
{
    "type": "ACTExecute",
    "params": {
        "model_name": "pickup_cube",
        "max_duration": 30.0,
        "action_scale": 0.4
    }
}
```

### 完整任务示例

```json
{
    "name": "ACT夹取方块",
    "root": {
        "type": "Sequence",
        "children": [
            {
                "type": "ArmMoveJ",
                "params": { "side": "right", "pose_name": "home" }
            },
            {
                "type": "GripperControl",
                "params": { "side": "right", "action": "open" }
            },
            {
                "type": "ACTExecute",
                "params": {
                    "model_name": "pickup_cube",
                    "max_duration": 20.0
                }
            }
        ]
    }
}
```

## 📁 模型目录结构

```
~/qyh-robot-system/models/
├── pickup_cube/
│   ├── policy.pt              # 模型权重
│   ├── normalization.npz      # 归一化参数
│   └── config.yaml            # 训练配置（可选）
├── place_cube/
│   └── ...
└── fold_clothes/
    └── ...
```

## 🔧 ROS2 接口

### 服务

| 服务名 | 类型 | 说明 |
|-------|------|------|
| `/act_inference_node/load_model` | `Trigger` | 加载模型 |
| `/act_inference_node/start` | `SetBool` | 启动/停止推理 |
| `/act_inference_node/reset` | `Trigger` | 重置状态 |

### 订阅话题

| 话题 | 类型 | 说明 |
|-----|------|------|
| `/right_arm/joint_states` | `JointState` | 右臂关节状态 |
| `/left_arm/joint_states` | `JointState` | 左臂关节状态 |
| `/camera/head/color/image_raw` | `Image` | 头部相机图像 |

### 发布话题

| 话题 | 类型 | 说明 |
|-----|------|------|
| `/act_inference_node/status` | `String` | 推理状态 |

## 📊 性能参考

| 配置 | 推理延迟 | 控制频率 |
|-----|---------|---------|
| CUDA (RTX 3080) | ~10ms | 50 Hz |
| CUDA (Jetson Orin) | ~30ms | 20 Hz |
| CPU (i7-12700) | ~100ms | 10 Hz |

## 🔗 相关链接

- [ACT 论文](https://arxiv.org/abs/2304.13705)
- [qyh_task_engine 文档](../qyh_task_engine/README.md)
- [数据采集文档](../../DATA_COLLECTION_VERIFICATION.md)
