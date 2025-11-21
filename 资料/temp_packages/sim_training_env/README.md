# 仿真训练环境 (Simulation Training Environment)

用于双臂机械臂模仿学习（ACT/ACT++）的仿真训练环境。

## 项目目标

在没有实体摄像头和传感器的情况下，通过Gazebo仿真实现：
1. 双7轴机械臂抓取零件
2. 将零件口朝上放置到指定位置
3. 录制演示数据用于ACT/ACT++训练

## 目录结构

```
sim_training_env/
├── launch/              # 启动文件
│   ├── training_sim.launch.py      # 主仿真启动文件
│   └── record_demo.launch.py       # 数据录制启动文件
├── worlds/              # Gazebo世界文件
│   └── pick_place_training.world   # 抓取放置训练场景
├── models/              # 3D模型
│   └── target_part/                # 待抓取零件模型
├── config/              # 配置文件
│   └── recorder_config.yaml        # 数据录制配置
├── scripts/             # Python脚本
│   ├── data_recorder.py            # 数据录制节点
│   ├── teleop_control.py           # 遥操作控制
│   └── data_processor.py           # 数据后处理
└── README.md
```

## 完整实施计划

### 阶段一：搭建仿真环境 ✅

**目标**：创建包含桌面、零件、相机的Gazebo场景

**任务**：
- [x] 创建sim_training_env包
- [x] 创建world文件（桌子、零件、目标区域）
- [ ] 添加RGBD相机插件
- [ ] 配置Gazebo物理参数
- [ ] 测试场景加载

**验收标准**：
```bash
ros2 launch sim_training_env training_sim.launch.py
# 能看到双臂机器人、桌子、零件和相机视角
```

### 阶段二：配置机械臂控制 ��

**目标**：确保仿真中机械臂可控

**任务**：
- [ ] 在URDF中添加ros2_control插件
- [ ] 配置Gazebo的joint trajectory controller
- [ ] 测试左右臂独立控制
- [ ] 验证关节状态反馈

**验收标准**：
```bash
# 能够通过命令控制机械臂
ros2 topic pub /right_arm_controller/joint_trajectory ...
```

### 阶段三：开发Teleoperation控制

**目标**：实现人工示教控制界面

**任务**：
- [ ] 创建键盘/GUI控制节点
- [ ] 实现末端位姿控制
- [ ] 添加夹爪开合控制
- [ ] 实现底盘移动（如需要）

**示例控制方式**：
- 键盘WASD：控制选中机械臂末端XY移动
- QE：Z轴上下
- 空格：切换左右臂
- G：夹爪开合

### 阶段四：数据录制系统

**目标**：同步录制多模态演示数据

**任务**：
- [ ] 实现数据录制节点（data_recorder.py）
- [ ] 同步RGB、深度、关节状态
- [ ] 保存为结构化格式
- [ ] 添加任务标注功能

**录制数据格式**：
```
demo_0001/
├── rgb/
│   ├── frame_00001.jpg
│   └── ...
├── depth/
│   ├── frame_00001.png
│   └── ...
├── joint_states.json
└── meta.yaml
```

### 阶段五：数据处理Pipeline

**目标**：将录制数据转换为训练格式

**任务**：
- [ ] 实现数据对齐和同步
- [ ] 转换为npz/hdf5格式
- [ ] 提取图像特征
- [ ] 生成动作序列标签

**输出格式**（ACT/ACT++兼容）：
```python
{
    'observations': {
        'images': [T, H, W, 3],      # RGB图像序列
        'depth': [T, H, W],           # 深度图序列
        'qpos': [T, 14]               # 双臂关节位置（7+7）
    },
    'actions': [T, 14],               # 目标关节位置
    'episode_ends': [indices]         # 每个episode的结束帧
}
```

### 阶段六：ACT/ACT++训练框架

**目标**：实现模仿学习训练

**任务**：
- [ ] 搭建ACT模型结构
- [ ] 实现Behavior Cloning训练
- [ ] 添加action chunking机制
- [ ] 实现模型评估

**训练流程**：
```bash
# 1. 录制50-100条演示
ros2 launch sim_training_env record_demo.launch.py

# 2. 处理数据
python scripts/data_processor.py --input demos/ --output dataset.npz

# 3. 训练模型
python train_act.py --data dataset.npz --epochs 1000

# 4. 在仿真中评估
python eval_policy.py --checkpoint model.pth
```

## 快速开始

### 1. 编译包
```bash
cd ~/qyh_jushen_ws
colcon build --packages-select sim_training_env
source install/setup.bash
```

### 2. 启动仿真
```bash
ros2 launch sim_training_env training_sim.launch.py
```

### 3. 录制演示（待实现）
```bash
# 终端1：启动仿真
ros2 launch sim_training_env training_sim.launch.py

# 终端2：启动录制
ros2 run sim_training_env data_recorder.py

# 终端3：控制机械臂
ros2 run sim_training_env teleop_control.py
```

## 技术栈

- **仿真**：Gazebo Classic / Gazebo Ignition
- **机器人框架**：ROS 2 Humble
- **控制**：ros2_control + joint_trajectory_controller
- **视觉**：RGB-D相机插件
- **训练**：PyTorch + ACT/ACT++

## 参考资料

- Mobile ALOHA论文：https://arxiv.org/abs/2401.02117
- ALOHA项目：https://tonyzhaozh.github.io/aloha/
- ACT代码：https://github.com/tonyzhaozh/act

## 当前进度

- [x] 创建项目结构
- [x] 编写world文件
- [ ] 测试仿真启动
- [ ] 配置控制器
- [ ] 实现数据录制
- [ ] 开发训练代码

## 下一步工作

1. 完成world文件中的相机配置
2. 确保Dual-Arms URDF包含ros2_control插件
3. 测试整个仿真环境能否正常启动
4. 开发简单的teleop控制来验证机械臂可控性
