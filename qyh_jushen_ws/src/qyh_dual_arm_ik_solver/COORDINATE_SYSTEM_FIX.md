# 坐标系修复说明

## 🐛 问题描述

**原始问题**：IK求解器没有正确处理坐标系转换

### 原实现问题
1. ❌ coordinate_mapper发布的是`vr_origin`坐标系的位姿
2. ❌ IK求解器直接使用，没有转换到`base_link_left/right`
3. ❌ JAKA SDK期望的是相对于`base_link_left/right`的位姿
4. ❌ 导致坐标系不匹配，机械臂运动异常

## ✅ 修复方案

### 选择：方案B - IK求解器负责TF转换

**原因**：
1. ✅ 符合架构分层：coordinate_mapper处理VR空间，IK求解器处理机器人空间
2. ✅ 保持解耦：coordinate_mapper无需知道机器人模型
3. ✅ 利用TF系统：自动处理完整变换链（包括动态零位校准）
4. ✅ 性能足够：TF查询在125Hz下完全可行

## 📝 代码修改

### 1. dual_arm_ik_solver_node.cpp

#### 初始化部分
```cpp
// ⭐ 始终初始化TF buffer和listener
tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

// use_tf_lookup默认为true
declare_parameter<bool>("use_tf_lookup", true);
```

#### solveLeftArmIK() 修改
```cpp
bool solveLeftArmIK()
{
    // === 步骤1: TF坐标系转换 ⚠️ 关键 ===
    geometry_msgs::msg::PoseStamped target_in_base_left;
    
    try {
        // 自动处理完整TF链: vr_origin → teleop_base → base_link → base_link_left
        target_in_base_left = tf_buffer_->transform(
            *left_target_,  // 输入: vr_origin坐标系
            "base_link_left",  // 输出: base_link_left坐标系
            tf2::durationFromSec(0.1)
        );
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "左臂TF转换失败: %s", ex.what());
        return false;
    }
    
    // === 步骤2: 末端坐标系校正 ===
    // human_hand: [X前, Y左, Z上] → lt: [X左, Y上, Z后]
    tf2::Quaternion q_correction;
    q_correction.setRPY(0, 0, -M_PI_2);  // 绕Z轴-90度
    
    tf2::Quaternion q_corrected = q_correction * q_base_left;
    tf2::Vector3 pos_corrected = R_correction * pos_base_left;
    
    // === 步骤3: 转换到JAKA格式并调用IK ===
    // ... (单位转换 m→mm, quat→euler)
    robot_->kine_inverse(0, ref_joints, &target_pose, &ik_result);
}
```

#### solveRightArmIK() 同样修改
- TF转换到`base_link_right`
- 末端校正使用+90度（镜像）

### 2. ik_solver_params.yaml

```yaml
# ⭐ 必须使用TF查询进行坐标系转换
use_tf_lookup: true  # 必须为true
```

### 3. README.md

更新了重要说明部分，详细说明坐标系处理流程。

## 🔄 完整数据流

```
VR设备
  ↓
[vr_bridge] 发布 vr_origin → vr_left_controller
  ↓
[coordinate_mapper] 
  - 读取: vr_origin → vr_left_controller
  - 处理: 滤波、缩放、速度限制
  - 发布: /teleop/left_hand/target (frame_id="vr_origin") ⭐
  ↓
[dual_arm_ik_solver] ⭐ 本次修复
  - 接收: /teleop/left_hand/target (vr_origin坐标系)
  - TF转换: vr_origin → base_link_left (通过TF查询)
  - 末端校正: human_hand [X前] → lt [X左]
  - IK求解: 输入base_link_left坐标系的位姿
  - 发布: /left_arm/joint_command
  ↓
[qyh_jaka_control] 伺服控制
  ↓
机械臂运动
```

## 📐 坐标系关系

### 完整TF链
```
vr_origin (VR零位)
  ↓ [teleop_manager发布]
teleop_base (人体语义坐标系)
  ↓ [静态TF]
base_link (机器人中心)
  ↓ [URDF + 校准偏移]
base_link_left (左臂基座)
  ↓ [JAKA运动学]
lt (左臂末端)
```

### 坐标系方向
- `vr_origin`: [X前, Y左, Z上] (VR空间)
- `human_hand`: [X前, Y左, Z上] (人手语义)
- `base_link_left`: [X前, Y左, Z上] (机器人坐标系)
- `lt`: [X左, Y上, Z后] (JAKA末端) ⚠️ 需要校正

## ✅ 验证清单

### 编译测试
```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
colcon build --packages-select qyh_dual_arm_ik_solver
```

### 运行测试
```bash
# 终端1: 启动TF发布（需要完整TF树）
ros2 launch qyh_dual_arms_description display.launch.py

# 终端2: 启动VR数据
ros2 launch qyh_vr_bridge vr_bridge.launch.py

# 终端3: 启动遥操作（包括teleop_manager和coordinate_mapper）
ros2 launch qyh_dual_arm_teleop teleop.launch.py

# 终端4: 启动JAKA控制
ros2 launch qyh_jaka_control jaka_control.launch.py

# 终端5: 启动IK求解器
ros2 launch qyh_dual_arm_ik_solver ik_solver.launch.py
```

### TF验证
```bash
# 检查TF链是否完整
ros2 run tf2_ros tf2_echo vr_origin base_link_left

# 查看TF树
ros2 run tf2_tools view_frames
```

### 功能验证
- [ ] IK求解器能正常启动
- [ ] 没有TF转换失败的警告
- [ ] 接收到目标位姿后能成功求解IK
- [ ] 发布的关节指令合理（没有跳变）
- [ ] 机械臂能正确跟随VR手柄运动

## 📚 相关文档

- [TF_design.md](../TF_design.md) - 完整坐标系设计
- [DATA_FLOW.md](../DATA_FLOW.md) - 数据流文档
- [README.md](README.md) - 本包说明

## 🔧 故障排除

### 问题：TF转换失败
```
"左臂TF转换失败: Lookup would require extrapolation"
```
**解决**：
1. 确认所有TF发布节点都在运行
2. 检查TF树是否完整：`ros2 run tf2_tools view_frames`
3. 确认teleop_manager正在发布`teleop_base → vr_origin`

### 问题：IK求解失败率高
**可能原因**：
1. 末端坐标系校正不正确（检查旋转角度）
2. 目标位姿超出机械臂工作空间
3. IK参考关节角度不合理

### 问题：机械臂运动方向错误
**检查**：
1. 末端坐标系校正旋转方向（左臂-90°，右臂+90°）
2. TF链中的各个变换是否正确
3. URDF中的机械臂安装偏移

---

**修复日期**：2025-12-16
**修复人**：GitHub Copilot
**状态**：✅ 已完成，待测试验证
