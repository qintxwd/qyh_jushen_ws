# Teleoperation 和数据录制指南

## 🎮 键盘控制使用说明

### 启动步骤

#### 方法1：分步启动（推荐用于调试）

```bash
# 终端1: 启动仿真
source install/setup.bash
ros2 launch sim_training_env training_sim.launch.py

# 终端2: 启动数据录制节点
source install/setup.bash
python3 src/sim_training_env/scripts/data_recorder_enhanced.py

# 终端3: 启动键盘控制
source install/setup.bash
python3 src/sim_training_env/scripts/teleop_keyboard.py
```

#### 方法2：一键启动（用于正式录制）

```bash
# 终端1: 启动仿真和录制
source install/setup.bash
ros2 launch sim_training_env record_demo.launch.py

# 终端2: 启动键盘控制
source install/setup.bash
python3 src/sim_training_env/scripts/teleop_keyboard.py
```

---

## ⌨️ 键盘控制按键说明

### 基础控制
- **数字键 1-7**: 选择要控制的关节 (J1-J7)
  - J1: 基座旋转
  - J2-J7: 各关节
  
- **Q**: 增加选中关节角度 (+0.1 弧度)
- **A**: 减少选中关节角度 (-0.1 弧度)

- **空格**: 切换左右臂
  - RIGHT ARM (右臂)
  - LEFT ARM (左臂)

### 录制控制
- **R**: 开始/停止录制
  - 第一次按R: 🔴 开始录制
  - 第二次按R: ⏹️ 停止录制并保存

### 辅助功能
- **H**: 显示帮助信息
- **ESC**: 退出程序

---

## 📝 录制演示数据的标准流程

### 第1步：准备环境
```bash
# 启动仿真和录制系统
ros2 launch sim_training_env record_demo.launch.py
# 等待Gazebo完全加载（约10秒）
```

### 第2步：启动控制
```bash
# 新终端
python3 src/sim_training_env/scripts/teleop_keyboard.py
```

### 第3步：测试控制
在开始正式录制前，先测试控制：
1. 按 `1` 选择J1关节
2. 按 `Q` 几次，观察右臂是否移动
3. 按 `空格` 切换到左臂
4. 再次测试左臂控制

### 第4步：录制演示
1. **按 R 键开始录制** - 看到 "🔴 录制开始"
2. **执行完整任务**：
   - 移动右臂到零件位置
   - 抓取零件（模拟夹爪闭合）
   - 移动到目标位置
   - 调整姿态使零件口朝上
   - 放下零件（模拟夹爪打开）
3. **再按 R 键停止录制** - 看到 "⏹️ 录制停止"
4. 数据自动保存到 `~/demo_recordings/demo_XXXX_timestamp/`

### 第5步：重复
- 每次录制会自动编号（demo_0001, demo_0002, ...）
- 建议录制 **50-100 条演示**
- 每条演示保持动作连贯

---

## 📊 录制数据结构

每次录制会生成如下结构：

```
~/demo_recordings/
└── demo_0001_20251111_174500/
    ├── rgb/
    │   ├── frame_00001.jpg
    │   ├── frame_00002.jpg
    │   └── ...
    ├── depth/
    │   ├── frame_00001.png
    │   ├── frame_00002.png
    │   └── ...
    ├── joint_states.json      # 所有关节状态（100Hz）
    └── meta.yaml              # 元数据（任务名、时长等）
```

### meta.yaml 示例
```yaml
task_name: pick_place_upright
demo_number: 1
success: true
frame_count: 150
joint_state_count: 500
duration: 5.2
```

---

## 🎯 录制技巧和最佳实践

### 动作要求
1. **平滑连贯**: 避免突然的大幅度移动
2. **速度适中**: 不要太快或太慢
3. **完整任务**: 从初始状态到完成状态

### 多样性建议
为了提高模型泛化能力，尝试：
- 不同的起始位置
- 稍微不同的运动路径
- 不同的抓取角度
- 轻微的速度变化

### 质量控制
- **成功演示**: 零件最终口朝上且稳定放置
- **失败演示**: 也可以保留一部分，标记为失败
- 每10个演示检查一次录制数据

---

## 🐛 常见问题

### Q1: 控制节点无响应
**解决**: 
```bash
# 检查joint_states是否发布
ros2 topic echo /joint_states
# 如果没有输出，重启仿真
```

### Q2: 录制节点没有保存数据
**解决**:
```bash
# 检查topic是否存在
ros2 topic list | grep camera
# 确保相机数据在发布
```

### Q3: 机械臂移动不连贯
**解决**:
- 减小步长：修改 `joint_step = 0.05` （在teleop_keyboard.py中）
- 或使用更慢的控制频率

### Q4: 想修改保存目录
**解决**:
```bash
ros2 launch sim_training_env record_demo.launch.py \
    save_dir:=/path/to/your/directory
```

---

## 📈 后续步骤

录制完成后，下一步是数据处理：

1. **验证数据**
```bash
ls -lh ~/demo_recordings/
# 确保有50+个demo目录
```

2. **数据处理**（下一阶段）
```bash
python3 scripts/data_processor.py \
    --input ~/demo_recordings \
    --output dataset.npz
```

3. **训练ACT模型**（最后阶段）
```bash
python3 train_act.py --data dataset.npz
```

---

## 💡 提示

- 建议第一次先录制 5-10 个演示，验证流程
- 然后批量录制剩余的 40-90 个
- 保持良好的工作节奏，避免疲劳
- 定期检查数据质量

---

**快捷命令汇总**：
```bash
# 启动
ros2 launch sim_training_env record_demo.launch.py  # 终端1
python3 src/sim_training_env/scripts/teleop_keyboard.py  # 终端2

# 录制
R - 开始录制
[执行动作]
R - 停止录制

# 重复50-100次
```

**相关文档**：
- `QUICK_START.md` - 快速开始
- `仿真训练项目计划.md` - 完整计划
