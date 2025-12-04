# VR/遥操作设备与机械臂对齐方案调研报告

## 📌 问题描述

**核心问题**: 遥操作启动时，VR手柄位置与机械臂末端位置存在较大差异

```
VR手柄位置: (0.5, 0.3, 1.2) m  (在操作者前方)
机械臂末端: (0.2, 0.1, 0.8) m  (在机器人workspace内)

初始差异: Δx ≈ 0.5m
```

**直接跟随的问题**:
1. 差分IK计算出巨大的关节速度: `dq = J⁻¹ * Δx/dt`
2. 超过安全速度限制 → 安全停止
3. 即使不触发安全限制，机械臂也会产生剧烈运动

---

## 🔍 开源项目方案总结

### 1️⃣ Leader-Follower (主从机械臂)

**代表项目**: ALOHA, Mobile ALOHA, GELLO

**原理**:
```
┌─────────────┐     1:1映射      ┌─────────────┐
│  Leader臂   │  ──────────────>│  Follower臂 │
│ (人操作的)   │   关节角度直通   │  (执行任务)  │
└─────────────┘                 └─────────────┘
```

**实现方式** (ALOHA代码):
```python
# aloha_scripts/one_side_teleop.py
def one_side_teleop(side):
    master = InterbotixManipulatorXS("master_" + side)
    puppet = InterbotixManipulatorXS("puppet_" + side)
    
    while True:
        # 直接读取master的关节角度
        master_positions = master.arm.get_joint_positions()
        
        # 直接发送给puppet
        puppet.arm.set_joint_positions(master_positions)
```

**优点**:
- ✅ **天然对齐**: Leader和Follower运动学完全相同，无对齐问题
- ✅ **直觉性强**: 1:1映射，手感自然
- ✅ **延迟低**: 无需IK计算
- ✅ **力反馈**: Leader臂天然能感受到Follower臂的阻力

**缺点**:
- ❌ **成本高**: 需要额外一套机械臂作为Leader
- ❌ **空间占用大**: 需要两套机械臂的物理空间
- ❌ **不适用VR**: 无法与VR头显系统集成
- ❌ **臂展限制**: 操作范围受Leader臂限制

**适用场景**: 工业场景、需要高精度/力反馈的任务

---

### 2️⃣ SpaceMouse 增量控制

**代表项目**: Diffusion Policy, robomimic, many industrial systems

**原理**:
```
SpaceMouse输出: (dx, dy, dz, droll, dpitch, dyaw)  # 相对增量
                         │
                         ▼
机械臂末端位姿: pose_new = pose_current + delta * scale
```

**实现方式**:
```python
# 典型的SpaceMouse控制循环
while True:
    # SpaceMouse返回相对运动 (6个轴的增量)
    dx, dy, dz, drx, dry, drz = spacemouse.get_state()
    
    # 缩放因子
    pos_scale = 0.001  # m per tick
    rot_scale = 0.01   # rad per tick
    
    # 增量更新
    current_pose = robot.get_ee_pose()
    target_pose = current_pose.copy()
    target_pose[:3] += np.array([dx, dy, dz]) * pos_scale
    target_pose[3:6] += np.array([drx, dry, drz]) * rot_scale
    
    robot.move_to(target_pose)
```

**优点**:
- ✅ **无对齐问题**: 始终是相对增量，不存在绝对位置差异
- ✅ **精确控制**: 可以精细调节scale实现微米级控制
- ✅ **成本低**: SpaceMouse ~$300-500
- ✅ **简单可靠**: 实现简单，稳定性好

**缺点**:
- ❌ **直觉性差**: 不是自然的手臂映射
- ❌ **学习曲线**: 需要学习6自由度同时操作
- ❌ **速度受限**: 大范围移动需要多次推动
- ❌ **无沉浸感**: 无法提供VR的沉浸式体验

**适用场景**: 实验室数据采集、精细操作、快速原型验证

---

### 3️⃣ VR手势追踪 + Retargeting

**代表项目**: Open-TeleVision, AnyTeleop, dex-retargeting

**原理**:
```
┌───────────────┐     重定向      ┌───────────────┐
│ VR手/手指位姿  │ ───────────────>│ 机器人手臂/手  │
│ (人体尺度)     │  优化/映射算法   │ (机器人尺度)   │
└───────────────┘                └───────────────┘
```

**关键技术: Retargeting (重定向)**

AnyTeleop/dex-retargeting 使用优化算法进行重定向:
```python
# dex-retargeting核心算法
class VectorRetargeting:
    """基于向量的重定向"""
    def retarget(self, human_keypoints):
        # 目标: 保持人手关键点之间的相对方向向量
        # 而非绝对位置
        
        human_vectors = self.compute_vectors(human_keypoints)
        
        # 优化机器人关节角度，使机器人的向量与人手向量对齐
        robot_joints = self.optimizer.solve(
            target_vectors=human_vectors,
            initial_guess=self.robot_current_joints
        )
        return robot_joints
```

Open-TeleVision的实现:
```python
# teleop/teleop_hand.py
class TeleOpHand:
    def step(self, hand_pose):
        # 使用手腕位姿直接映射
        wrist_pos = hand_pose[:3]
        wrist_rot = hand_pose[3:7]
        
        # 坐标变换: VR空间 -> 机器人空间
        robot_target = self.transform_to_robot_frame(wrist_pos, wrist_rot)
        
        # 发送到机器人
        self.robot.set_ee_pose(robot_target)
```

**Open-TeleVision的对齐策略** (根据代码和论文分析):
- 使用**相对位姿变化**而非绝对位置
- 系统启动时记录初始参考位姿
- 后续跟踪相对于初始位姿的变化

**优点**:
- ✅ **沉浸感强**: VR提供身临其境的体验
- ✅ **双手自然操作**: 直接映射双手动作
- ✅ **适用灵巧手**: 可以映射手指动作
- ✅ **远程操作**: 支持跨地域操作

**缺点**:
- ❌ **需要对齐策略**: 必须解决坐标系对齐问题
- ❌ **重定向延迟**: 优化算法有计算开销
- ❌ **臂展差异**: 人与机器人臂展不同需要缩放

**适用场景**: 灵巧手操作、VR沉浸式遥操作、人形机器人

---

### 4️⃣ UMI手持夹爪

**代表项目**: Universal Manipulation Interface (UMI)

**原理**: 完全不同的思路 - 人手持夹爪直接示教，然后相机位姿重建
```
┌─────────────────┐                ┌─────────────────┐
│ 人手持GoPro夹爪 │  ──SLAM重建──> │  轨迹 + 相机图  │
└─────────────────┘                └─────────────────┘
                                           │
                                           ▼
                                   ┌─────────────────┐
                                   │  机器人执行策略 │
                                   └─────────────────┘
```

**关键特点**:
- **100%无标定**: 不需要VR/机器人坐标系对齐
- **相机中心动作表示**: 所有动作相对于腕部相机
- **跨机器人部署**: 同一策略可在不同机器人上使用

**优点**:
- ✅ **无对齐问题**: 不使用实时遥操作
- ✅ **便携性强**: 可以在任何环境采集数据
- ✅ **泛化性好**: 训练数据来自真实世界
- ✅ **数据采集快**: 30秒/示教

**缺点**:
- ❌ **非实时**: 不支持实时遥操作
- ❌ **无力反馈**: 示教时无机器人反馈
- ❌ **依赖SLAM**: 需要GoPro SLAM精度

**适用场景**: 大规模数据采集、in-the-wild训练数据、任务学习

---

## 🎯 VR遥操作对齐方案详细对比

针对我们的PICO4 VR + JAKA双臂系统，以下是专门针对VR遥操作的对齐方案:

### 方案A: Clutch Mode (离合器模式)

**原理**: 按下按钮时建立VR-机器人位姿映射关系

```cpp
// 伪代码
class ClutchController {
    bool clutch_engaged = false;
    Pose vr_reference;      // 离合器接合时的VR位姿
    Pose robot_reference;   // 离合器接合时的机器人位姿
    
    void update(const Pose& vr_pose, bool clutch_button) {
        if (clutch_button && !clutch_engaged) {
            // 离合器刚接合
            clutch_engaged = true;
            vr_reference = vr_pose;
            robot_reference = robot.get_ee_pose();
        }
        else if (!clutch_button) {
            clutch_engaged = false;
            // 不控制机器人
        }
        else {
            // 离合器接合中 - 跟随增量
            Pose delta = vr_pose - vr_reference;  // 相对移动
            Pose target = robot_reference + delta; // 机器人目标
            robot.move_to(target);
        }
    }
};
```

**操作流程**:
1. 操作者将手移动到舒适位置
2. 按下Clutch按钮（如Grip键）
3. 系统记录VR和机器人当前位姿作为参考
4. 松开按钮时可以调整手的位置但不影响机器人
5. 再次按下继续操作

**优点**:
- ✅ 完全消除初始位置差异
- ✅ 操作者可以随时调整姿势
- ✅ 适合长时间操作，手累了可以松开休息
- ✅ 实现简单

**缺点**:
- ❌ 需要按住按钮操作，手指可能疲劳
- ❌ 松开瞬间可能有位置跳变（需要处理）
- ❌ 不如连续跟踪直觉

**代码实现复杂度**: ⭐⭐ (简单)

---

### 方案B: Incremental Mode (纯增量模式)

**原理**: 始终只跟随VR位姿的增量变化

```cpp
// 伪代码
class IncrementalController {
    Pose prev_vr_pose;
    bool initialized = false;
    double scale = 1.0;
    
    void update(const Pose& vr_pose) {
        if (!initialized) {
            prev_vr_pose = vr_pose;
            initialized = true;
            return;
        }
        
        // 计算VR位姿增量
        Pose delta = vr_pose - prev_vr_pose;
        prev_vr_pose = vr_pose;
        
        // 应用到机器人（带缩放）
        Pose current = robot.get_ee_pose();
        Pose target = current + delta * scale;
        robot.move_to(target);
    }
};
```

**优点**:
- ✅ 无需任何按钮操作
- ✅ 启动即可操作
- ✅ 实现最简单

**缺点**:
- ❌ **累积误差**: VR抖动会导致机器人漂移
- ❌ **无法到达绝对位置**: 如果VR丢失追踪，无法恢复
- ❌ **直觉性稍差**: 不是"我手在哪机器人手就在哪"

**代码实现复杂度**: ⭐ (最简单)

---

### 方案C: Initial Alignment Mode (初始对齐模式)

**原理**: 启动时进行一次性对齐校准

```cpp
// 伪代码
class InitialAlignmentController {
    Transform vr_to_robot;  // 坐标变换
    bool calibrated = false;
    
    void calibrate(const Pose& vr_pose) {
        Pose robot_pose = robot.get_ee_pose();
        // 计算从VR空间到机器人空间的变换
        vr_to_robot = compute_transform(vr_pose, robot_pose);
        calibrated = true;
    }
    
    void update(const Pose& vr_pose) {
        if (!calibrated) {
            // 等待按钮触发校准
            return;
        }
        // VR位姿直接变换到机器人空间
        Pose target = vr_to_robot * vr_pose;
        robot.move_to(target);
    }
};
```

**操作流程**:
1. 启动系统
2. 将手移动到与机器人末端"对应"的位置
3. 按下校准按钮
4. 系统建立坐标变换关系
5. 之后直接跟随

**优点**:
- ✅ 校准后直觉性最好
- ✅ 可以实现"我手在哪机器人手就在哪"
- ✅ 只需校准一次

**缺点**:
- ❌ 初始手位置需要与机器人对应（操作者需要"想象"机器人位置）
- ❌ 如果VR丢失追踪，重新校准
- ❌ 人与机器人臂展不同可能导致后续跟踪不自然

**代码实现复杂度**: ⭐⭐ (简单)

---

### 方案D: Workspace Mapping (工作空间映射)

**原理**: 将人的工作空间整体映射到机器人工作空间

```cpp
// 伪代码
class WorkspaceMappingController {
    // 人的工作空间（经验值或标定）
    BoundingBox human_workspace = {{-0.5, -0.5, 0.8}, {0.5, 0.5, 1.5}};  // 在人前方
    
    // 机器人工作空间
    BoundingBox robot_workspace = {{-0.3, -0.3, 0.1}, {0.3, 0.3, 0.6}};  // 在桌面上
    
    Pose mapPose(const Pose& vr_pose) {
        // 归一化到0-1
        Vector3d normalized = (vr_pose.position - human_workspace.min) 
                            / (human_workspace.max - human_workspace.min);
        
        // 映射到机器人空间
        Vector3d robot_pos = robot_workspace.min 
                           + normalized * (robot_workspace.max - robot_workspace.min);
        
        return {robot_pos, vr_pose.orientation};
    }
};
```

**优点**:
- ✅ 无需按钮，启动即可
- ✅ 全范围映射，适合大工作空间
- ✅ 可以处理人/机器人臂展差异

**缺点**:
- ❌ **比例失真**: 映射会改变实际移动距离
- ❌ 需要预先定义工作空间
- ❌ 边界处理困难

**代码实现复杂度**: ⭐⭐⭐ (中等)

---

### 方案E: Smooth Transition Mode (平滑过渡模式)

**原理**: 启动时让机器人缓慢移动到VR位置

```cpp
// 伪代码
class SmoothTransitionController {
    enum State { TRANSITIONING, TRACKING };
    State state = TRANSITIONING;
    double transition_speed = 0.02;  // m/s
    double threshold = 0.01;  // m
    
    void update(const Pose& vr_pose) {
        Pose current = robot.get_ee_pose();
        Pose target_robot = vr_to_robot_transform(vr_pose);
        
        if (state == TRANSITIONING) {
            double distance = (target_robot.position - current.position).norm();
            
            if (distance < threshold) {
                state = TRACKING;
            } else {
                // 缓慢向VR位置移动
                Vector3d direction = (target_robot.position - current.position).normalized();
                Pose move_target = current;
                move_target.position += direction * transition_speed * dt;
                robot.move_to(move_target);
            }
        } else {
            // 正常跟踪
            robot.move_to(target_robot);
        }
    }
};
```

**优点**:
- ✅ 无需任何按钮
- ✅ 启动后自动对齐
- ✅ 直觉性好

**缺点**:
- ❌ **过渡时间长**: 如果差异大，需要等待
- ❌ **操作者需等待**: 不能立即操作
- ❌ **安全问题**: 机器人自动移动可能碰撞

**代码实现复杂度**: ⭐⭐ (简单)

---

## 📊 方案对比矩阵

| 方案 | 直觉性 | 安全性 | 实现复杂度 | 操作便捷性 | 推荐度 |
|------|--------|--------|------------|------------|--------|
| **A. Clutch Mode** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| B. Incremental Mode | ⭐⭐ | ⭐⭐⭐⭐ | ⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| C. Initial Alignment | ⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ |
| D. Workspace Mapping | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ |
| E. Smooth Transition | ⭐⭐⭐⭐ | ⭐⭐ | ⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ |

---

## 🔧 推荐实现: Clutch Mode + 增量融合

**推荐采用 Clutch Mode 作为主要方案**，原因：
1. 业界VR遥操作的事实标准
2. 安全性最高
3. 实现简单，易于调试
4. 灵活性好，操作者可以随时调整

### 详细实现方案

```cpp
// vr_clutch_controller.hpp
class VRClutchController {
public:
    struct Config {
        double position_scale = 1.0;       // 位置缩放
        double rotation_scale = 1.0;       // 旋转缩放
        double max_position_delta = 0.05;  // 单步最大位移 (m)
        double max_rotation_delta = 0.1;   // 单步最大旋转 (rad)
        double release_smoothing = 0.95;   // 松开时的平滑系数
    };
    
    enum class State {
        IDLE,           // 未按住Clutch
        ENGAGING,       // 刚按下Clutch，建立参考
        TRACKING,       // 正在跟踪
        RELEASING       // 刚松开，平滑过渡
    };
    
private:
    Config config_;
    State state_ = State::IDLE;
    
    // 参考位姿
    Eigen::Isometry3d vr_reference_;
    Eigen::Isometry3d robot_reference_;
    
    // 平滑用
    Eigen::Isometry3d last_target_;
    
public:
    Eigen::Isometry3d update(
        const Eigen::Isometry3d& vr_pose,
        const Eigen::Isometry3d& robot_current,
        bool clutch_pressed,
        double dt) 
    {
        switch (state_) {
            case State::IDLE:
                if (clutch_pressed) {
                    // 建立参考
                    vr_reference_ = vr_pose;
                    robot_reference_ = robot_current;
                    last_target_ = robot_current;
                    state_ = State::ENGAGING;
                }
                return robot_current;  // 保持当前位置
                
            case State::ENGAGING:
                state_ = State::TRACKING;
                // fall through
                
            case State::TRACKING:
                if (!clutch_pressed) {
                    state_ = State::RELEASING;
                    return last_target_;
                }
                {
                    // 计算VR相对于参考的增量
                    Eigen::Isometry3d vr_delta = vr_reference_.inverse() * vr_pose;
                    
                    // 应用缩放
                    Eigen::Vector3d position_delta = vr_delta.translation() * config_.position_scale;
                    Eigen::AngleAxisd rotation_delta(vr_delta.rotation());
                    rotation_delta = Eigen::AngleAxisd(
                        rotation_delta.angle() * config_.rotation_scale, 
                        rotation_delta.axis());
                    
                    // 限制增量
                    clampDelta(position_delta, rotation_delta);
                    
                    // 计算目标
                    Eigen::Isometry3d target = robot_reference_;
                    target.translate(position_delta);
                    target.rotate(rotation_delta);
                    
                    last_target_ = target;
                    return target;
                }
                
            case State::RELEASING:
                if (clutch_pressed) {
                    // 重新按下，建立新参考
                    vr_reference_ = vr_pose;
                    robot_reference_ = robot_current;
                    state_ = State::TRACKING;
                }
                // 松开时保持最后位置
                return last_target_;
        }
        return robot_current;
    }
    
private:
    void clampDelta(Eigen::Vector3d& pos, Eigen::AngleAxisd& rot) {
        // 限制位置增量
        double pos_norm = pos.norm();
        if (pos_norm > config_.max_position_delta) {
            pos = pos / pos_norm * config_.max_position_delta;
        }
        
        // 限制旋转增量
        if (rot.angle() > config_.max_rotation_delta) {
            rot = Eigen::AngleAxisd(config_.max_rotation_delta, rot.axis());
        }
    }
};
```

### 集成到 qyh_teleoperation_controller

```cpp
// teleoperation_node.cpp 中的使用
class TeleoperationNode : public rclcpp::Node {
    VRClutchController left_clutch_;
    VRClutchController right_clutch_;
    
    void vrCallback(const VRData::SharedPtr msg) {
        // 左手
        auto left_target = left_clutch_.update(
            poseToIsometry(msg->left_hand_pose),
            robot_->getLeftEEPose(),
            msg->left_grip_pressed,
            dt_
        );
        
        // 右手
        auto right_target = right_clutch_.update(
            poseToIsometry(msg->right_hand_pose),
            robot_->getRightEEPose(),
            msg->right_grip_pressed,
            dt_
        );
        
        // 发送到差分IK
        diff_ik_->solve(left_target, right_target);
    }
};
```

---

## 📝 结论

1. **主流开源项目** (ALOHA, LeRobot, robomimic) 多使用 **Leader-Follower** 或 **SpaceMouse**，这些设备**天然不存在对齐问题**

2. **VR遥操作项目** (Open-TeleVision, AnyTeleop) 使用 **相对位姿变化** + **坐标变换**

3. **推荐方案**: **Clutch Mode**
   - 按住Grip键建立VR-Robot位姿映射
   - 跟随VR相对增量
   - 松开时保持位置
   
4. **实现优先级**:
   1. 先实现基础Clutch Mode
   2. 添加增量限制和平滑
   3. 可选：添加Workspace Mapping作为辅助

---

## 📚 参考资料

- [ALOHA](https://github.com/tonyzhaozh/aloha) - Leader-Follower双臂遥操作
- [Mobile ALOHA](https://mobile-aloha.github.io/) - 移动双臂遥操作
- [GELLO](https://wuphilipp.github.io/gello_site/) - 低成本Leader臂方案
- [Open-TeleVision](https://robot-tv.github.io/) - VR沉浸式遥操作
- [AnyTeleop](https://yzqin.github.io/anyteleop/) - 通用VR遥操作框架
- [UMI](https://umi-gripper.github.io/) - 手持夹爪数据采集
- [dex-retargeting](https://github.com/dexsuite/dex-retargeting) - 手势重定向
- [Diffusion Policy](https://diffusion-policy.cs.columbia.edu/) - 使用SpaceMouse采集

---

**文档版本**: v1.0  
**创建日期**: 2025-01-21  
**作者**: qyh_jushen_ws team
