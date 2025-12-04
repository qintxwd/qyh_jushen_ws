# VR Clutch Mode 测试指南

## 📋 已完成的修改

### 1. 核心Clutch逻辑 (Python)
- `qyh_vr_calibration/vr_clutch_controller.py` - Clutch状态机
- `qyh_vr_calibration/vr_clutch_node.py` - ROS2节点
- `qyh_vr_calibration/config/vr_clutch_params.yaml` - 配置文件
- `qyh_vr_calibration/launch/vr_clutch.launch.py` - Launch文件
- `qyh_vr_calibration/setup.py` - 入口点注册

### 2. 消息定义
- `qyh_teleoperation_msgs/msg/TeleopStatus.msg` - 添加clutch字段

### 3. teleoperation_controller适配 (C++)
- `teleoperation_node.cpp` - 添加clutch状态订阅

### 4. 前端网页
- `backend/app/api/vr_teleoperation.py` - VR API
- `backend/app/ros2_bridge/bridge.py` - VR状态订阅
- `backend/app/main.py` - 路由注册
- `frontend/src/components/panels/VRTeleoperationPanel.vue` - 前端组件
- `frontend/src/components/panels/PanelContainer.vue` - 面板注册
- `frontend/src/stores/layout.ts` - 布局定义

---

## 🧪 测试步骤

### Step 1: 编译ROS2包

```bash
cd ~/qyh_jushen_ws
colcon build --packages-select qyh_vr_calibration qyh_teleoperation_msgs qyh_teleoperation_controller
source install/setup.bash
```

### Step 2: 测试Clutch节点 (无VR硬件)

启动节点：
```bash
ros2 run qyh_vr_calibration vr_clutch_node
```

检查话题：
```bash
ros2 topic list | grep vr
```

应该看到：
- `/vr/left_target_pose`
- `/vr/right_target_pose`
- `/vr/left_clutch_engaged`
- `/vr/right_clutch_engaged`

### Step 3: 模拟VR输入测试

终端1 - 启动节点：
```bash
ros2 run qyh_vr_calibration vr_clutch_node
```

终端2 - 发布模拟机器人状态：
```bash
ros2 topic pub /jaka/robot_state qyh_jaka_control_msgs/msg/RobotState "{
  connected: true,
  left_cartesian_pose: {position: {x: 0.3, y: 0.0, z: 0.5}},
  right_cartesian_pose: {position: {x: 0.3, y: 0.0, z: 0.5}}
}" --rate 50
```

终端3 - 发布模拟VR位姿：
```bash
ros2 topic pub /vr/left_hand/pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'vr_origin'},
  pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
}" --rate 50
```

终端4 - 发布模拟VR按键（模拟grip按下）：
```bash
# 按下grip (值 > 0.8)
ros2 topic pub /vr/left_hand/joy sensor_msgs/msg/Joy "{
  axes: [0.0, 0.0, 0.0, 0.9]
}" --once

# 松开grip (值 < 0.2)
ros2 topic pub /vr/left_hand/joy sensor_msgs/msg/Joy "{
  axes: [0.0, 0.0, 0.0, 0.1]
}" --once
```

终端5 - 监控输出：
```bash
ros2 topic echo /vr/left_clutch_engaged
ros2 topic echo /vr/left_target_pose
```

### Step 4: 完整系统测试

使用launch文件启动整个遥操作系统：
```bash
ros2 launch qyh_vr_calibration vr_clutch.launch.py
```

### Step 5: 前端测试

1. 启动后端：
```bash
cd qyh_jushen_web/backend
python -m uvicorn app.main:app --reload
```

2. 启动前端：
```bash
cd qyh_jushen_web/frontend
npm run dev
```

3. 打开浏览器访问 `http://localhost:5173`

4. 从侧边栏打开 "VR遥操作" 面板

5. 检查：
   - VR连接状态显示
   - 左/右手Clutch状态指示灯
   - Grip值进度条
   - 配置保存功能

---

## ⚙️ 配置参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `clutch.engage_threshold` | 0.8 | Grip值 > 此值时接合Clutch |
| `clutch.release_threshold` | 0.2 | Grip值 < 此值时释放Clutch |
| `clutch.position_scale` | 1.0 | VR位移 → 机器人位移的缩放 |
| `clutch.rotation_scale` | 1.0 | VR旋转 → 机器人旋转的缩放 |
| `clutch.max_position_delta` | 0.05m | 单步最大位移限制 |
| `clutch.max_rotation_delta` | 0.1rad | 单步最大旋转限制 |
| `clutch.axis_mapping` | [0,1,2] | VR轴→机器人轴映射 |
| `clutch.axis_signs` | [1,1,1] | 轴方向符号 |

---

## 🔧 故障排除

### 1. vr_clutch_node启动失败
检查依赖：
```bash
ros2 pkg list | grep qyh
pip list | grep scipy
```

### 2. 没有收到VR数据
检查vr_bridge是否运行：
```bash
ros2 node list | grep vr
ros2 topic hz /vr/left_hand/pose
```

### 3. 前端无法连接
检查ROS2桥接状态：
```bash
curl http://localhost:8000/api/v1/vr/status
```

---

## 📅 下一步：真机测试

1. 确保JAKA机械臂已连接且使能
2. 确保VR头盔和手柄已连接
3. 在开阔空间进行测试
4. 准备急停按钮
5. 先用低速参数测试：
   - `position_scale: 0.5`
   - `rotation_scale: 0.5`
   - `max_position_delta: 0.02`

---

**文档版本**: v1.0  
**创建日期**: 2025-01-21
