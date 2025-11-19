# 快速开始指南

## ✅ 当前进度

- [x] 创建sim_training_env仿真环境包
- [x] 添加ros2_control和gazebo_ros2_control插件到URDF
- [x] 配置joint_trajectory_controller
- [x] 创建完整的launch文件
- [ ] 测试仿真启动和控制器

## 🚀 立即测试

### 1. 启动仿真环境

```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
source install/setup.bash
ros2 launch sim_training_env training_sim.launch.py
```

**预期结果**：
- Gazebo窗口打开
- 看到桌子、红色零件和目标区域（绿色标记）
- 双臂机器人出现在场景中
- 约5秒后，控制器自动加载

### 2. 验证控制器（新终端）

```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
source install/setup.bash

# 查看已加载的控制器
ros2 control list_controllers

# 应该看到：
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# right_arm_controller[joint_trajectory_controller/JointTrajectoryController] active
# left_arm_controller[joint_trajectory_controller/JointTrajectoryController] active
```

### 3. 测试机械臂控制（新终端）

```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
source install/setup.bash

# 运行测试脚本（会让右臂第一个关节移动）
python3 src/sim_training_env/scripts/test_control.py
```

**预期结果**：右臂应该会摆动一下然后回到原位

### 4. 手动发送控制命令

```bash
# 发送简单的位置命令给右臂
ros2 topic pub --once /right_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['r-j1', 'r-j2', 'r-j3', 'r-j4', 'r-j5', 'r-j6', 'r-j7'],
  points: [
    {
      positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 2}
    }
  ]
}"
```

## 📂 项目结构

```
qyh_jushen_ws/
├── test_simulation.sh              # 一键测试脚本
├── QUICK_START.md                  # 本文件
├── 仿真训练项目计划.md              # 完整项目计划
└── src/
    ├── Dual-Arms/
    │   ├── urdf/
    │   │   ├── Dual-Arms-gazebo.urdf          # ✨ 已添加ros2_control
    │   │   └── Dual-Arms-gazebo.urdf.backup   # 原始备份
    │   └── config/
    │       └── dual_arms_controllers.yaml     # 控制器配置
    └── sim_training_env/
        ├── launch/
        │   └── training_sim.launch.py         # ✨ 主启动文件
        ├── worlds/
        │   └── pick_place_training.world      # Gazebo场景
        ├── scripts/
        │   └── test_control.py                # 控制测试脚本
        └── README.md
```

## �� 故障排查

### 问题1：Gazebo无法启动
**解决方案**：
```bash
killall gzserver gzclient
```

### 问题2：控制器未加载
**检查**：
```bash
ros2 control list_controllers
```
如果为空，等待5-10秒再试

### 问题3：机器人没有出现
**检查**：
```bash
ros2 topic echo /robot_description
```
应该能看到URDF内容

### 问题4：关节不动
**检查joint_states**：
```bash
ros2 topic echo /joint_states
```
应该能看到14个关节的状态

## 📋 下一步工作

### 阶段三：开发Teleoperation控制

创建键盘控制节点，让操作者可以直接控制机械臂：

```python
# 下一步要创建的文件
src/sim_training_env/scripts/teleop_keyboard.py
src/sim_training_env/scripts/teleop_gui.py
```

**功能**：
- 键盘WASD控制末端XY移动
- QE控制Z轴
- 空格切换左右臂
- R键开始/停止录制

### 阶段四：数据录制

创建数据录制节点：
```python
src/sim_training_env/scripts/data_recorder.py
```

**录制内容**：
- RGB图像 (30Hz)
- 深度图像 (30Hz)
- 关节状态 (100Hz)
- TF变换

## 🎯 验收标准

### 当前阶段（阶段二）验收：
- [x] Gazebo能正常启动
- [x] 机器人正确生成
- [ ] 控制器成功加载（需要实际测试）
- [ ] 能发送命令控制关节（需要实际测试）

### 完整项目验收：
- [ ] 能流畅示教操作
- [ ] 录制50+条演示数据
- [ ] 数据转换为训练格式
- [ ] ACT模型训练收敛
- [ ] 仿真内成功率>70%

## 📞 需要帮助？

查看详细文档：
- 完整项目计划：`仿真训练项目计划.md`
- 包文档：`src/sim_training_env/README.md`
- Mobile ALOHA论文：https://arxiv.org/abs/2401.02117

---

**当前状态**：✅ 阶段一完成，✅ 阶段二配置完成，🔄 等待测试验证
