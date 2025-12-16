/**
 * @file dual_arm_ik_solver_node.cpp
 * @brief 双臂IK求解节点 - VR遥操作专用
 * 
 * 功能：
 * - 作为第二个客户端连接到JAKA控制器
 * - 订阅VR目标位姿：/teleop/left_hand/target, /teleop/right_hand/target
 * - 高频调用IK求解（100Hz+）
 * - 发布关节指令供伺服控制使用
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <JAKAZuRobot.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <memory>
#include <array>

using namespace std::chrono_literals;

// ========== JAKA Zu7 关节限位和速度限制 ==========
// 来源：JAKA官方手册
struct JointLimits {
    double pos_min;  // 负限位（弧度）
    double pos_max;  // 正限位（弧度）
    double vel_max;  // 速度限制（弧度/秒）
};

const std::array<JointLimits, 7> JAKA_ZU7_LIMITS = {{
    {-6.2832, 6.2832, 1.5708},   // 关节1: ±360°, 90°/s
    {-1.8326, 1.8326, 1.5708},   // 关节2: ±105°, 90°/s
    {-6.2832, 6.2832, 2.0944},   // 关节3: ±360°, 120°/s
    {-2.5307, 0.5236, 2.0944},   // 关节4: -145°~30°, 120°/s
    {-6.2832, 6.2832, 2.6180},   // 关节5: ±360°, 150°/s
    {-1.8326, 1.8326, 2.6180},   // 关节6: ±105°, 150°/s
    {-6.2832, 6.2832, 2.6180}    // 关节7: ±360°, 150°/s
}};

const double SAFETY_MARGIN_POS = 0.0873;  // 5° 安全裕度
const double SAFETY_MARGIN_VEL = 0.8;     // 速度降到80%

class DualArmIKSolverNode : public rclcpp::Node
{
public:
    DualArmIKSolverNode() : Node("dual_arm_ik_solver_node")
    {
        // 参数声明
        declare_parameter<std::string>("robot_ip", "192.168.2.200");
        declare_parameter<double>("ik_rate", 125.0);  // 125Hz匹配伺服频率
        declare_parameter<bool>("auto_connect", true);
        // ⭐ 必须使用TF查询，因为coordinate_mapper发布的是vr_origin坐标系
        // 需要转换到base_link_left/right才能调用JAKA IK
        declare_parameter<bool>("use_tf_lookup", true);  // 默认启用TF查询
        declare_parameter<bool>("publish_debug_tf", true);  // 发布调试TF
        
        robot_ip_ = get_parameter("robot_ip").as_string();
        ik_rate_ = get_parameter("ik_rate").as_double();
        auto_connect_ = get_parameter("auto_connect").as_bool();
        use_tf_lookup_ = get_parameter("use_tf_lookup").as_bool();
        publish_debug_tf_ = get_parameter("publish_debug_tf").as_bool();
        
        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "  双臂IK求解节点启动");
        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "控制器IP: %s", robot_ip_.c_str());
        RCLCPP_INFO(get_logger(), "IK求解频率: %.1f Hz", ik_rate_);
        RCLCPP_INFO(get_logger(), "使用TF查询: %s", use_tf_lookup_ ? "是" : "否");
        RCLCPP_INFO(get_logger(), "发布调试TF: %s", publish_debug_tf_ ? "是" : "否");
        
        // 初始化JAKA SDK
        robot_ = std::make_unique<JAKAZuRobot>();
        
        // ⭐ TF监听器 - 必须初始化，用于坐标系转换
        // coordinate_mapper发布vr_origin坐标系，需要转换到base_link_left/right
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        
        // TF广播器 - 用于发布调试坐标系
        if (publish_debug_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }
        
        if (!use_tf_lookup_) {
            RCLCPP_WARN(get_logger(), "⚠️ use_tf_lookup=false 不推荐！");
            RCLCPP_WARN(get_logger(), "coordinate_mapper发布vr_origin坐标系，必须使用TF转换");
        }
        
        // 订阅VR目标位姿
        left_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/teleop/left_hand/target", 10,
            std::bind(&DualArmIKSolverNode::leftTargetCallback, this, std::placeholders::_1));
        
        right_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/teleop/right_hand/target", 10,
            std::bind(&DualArmIKSolverNode::rightTargetCallback, this, std::placeholders::_1));
        
        // 发布关节指令
        left_joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/left_arm/joint_command", 10);
        
        right_joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/right_arm/joint_command", 10);
        
        // 发布IK状态
        ik_status_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/ik_solver/status", 10);
        
        // 订阅实际关节状态 (用作IK参考)
        joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&DualArmIKSolverNode::jointStatesCallback, this, std::placeholders::_1));
        
        // 自动连接
        if (auto_connect_) {
            connectToRobot();
        }
        
        // 定时器 - 高频IK求解
        auto period = std::chrono::duration<double>(1.0 / ik_rate_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&DualArmIKSolverNode::ikSolverCallback, this));
        
        RCLCPP_INFO(get_logger(), "✓ IK求解节点初始化完成");
        RCLCPP_INFO(get_logger(), "等待VR目标位姿输入...");
    }
    
    ~DualArmIKSolverNode()
    {
        if (connected_) {
            RCLCPP_INFO(get_logger(), "断开JAKA连接...");
            robot_->login_out();
        }
    }

private:
    void connectToRobot()
    {
        RCLCPP_INFO(get_logger(), "连接到JAKA控制器 %s (第二个客户端)...", robot_ip_.c_str());
        
        errno_t ret = robot_->login_in(robot_ip_.c_str());
        if (ret == ERR_SUCC) {
            connected_ = true;
            RCLCPP_INFO(get_logger(), "✅ IK求解节点成功连接！");
            RCLCPP_INFO(get_logger(), "📊 多客户端连接已验证");
            
            // 初始化参考关节位置（零位）
            initReferenceJoints();
        } else {
            connected_ = false;
            RCLCPP_ERROR(get_logger(), "❌ 连接失败！错误码: %d", ret);
            RCLCPP_ERROR(get_logger(), "请确认qyh_jaka_control已运行");
        }
    }
    
    void initReferenceJoints()
    {
        // 左右臂零位参考（全零或张开姿态）
        for (int i = 0; i < 7; i++) {
            ref_left_joints_.jVal[i] = 0.0;
            ref_right_joints_.jVal[i] = 0.0;
        }
        // 可选：使用张开姿态作为参考
        ref_left_joints_.jVal[1] = -1.0472;  // -60度
        ref_right_joints_.jVal[1] = -1.0472;
        
        RCLCPP_INFO(get_logger(), "✓ 参考关节位置初始化完成");
    }
    
    void leftTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        left_target_ = msg;
        has_left_target_ = true;
    }
    
    void rightTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        right_target_ = msg;
        has_right_target_ = true;
    }
    
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 更新当前机械臂关节位置（用作IK参考）
        if (msg->position.size() >= 14) {  // 双臂14个关节
            // 左臂: 前7个关节
            for (int i = 0; i < 7; i++) {
                current_left_joints_.jVal[i] = msg->position[i];
            }
            has_current_left_ = true;
            
            // 右臂: 后7个关节
            for (int i = 0; i < 7; i++) {
                current_right_joints_.jVal[i] = msg->position[i + 7];
            }
            has_current_right_ = true;
        }
    }
    
    void ikSolverCallback()
    {
        if (!connected_) {
            return;
        }
        
        // 记录时间戳供速度检查使用
        last_solve_time_ = now();
        
        bool left_success = false;
        bool right_success = false;
        
        // 求解左臂IK
        if (has_left_target_) {
            left_success = solveLeftArmIK();
        }
        
        // 求解右臂IK
        if (has_right_target_) {
            right_success = solveRightArmIK();
        }
        
        // 发布状态
        auto status_msg = std_msgs::msg::Bool();
        status_msg.data = (left_success || right_success);
        ik_status_pub_->publish(status_msg);
        
        // 统计
        if (left_success) left_success_count_++;
        if (right_success) right_success_count_++;
        
        solve_count_++;
        
        // 每秒打印一次统计
        if (solve_count_ % static_cast<int>(ik_rate_) == 0) {
            double left_rate = 100.0 * left_success_count_ / solve_count_;
            double right_rate = 100.0 * right_success_count_ / solve_count_;
            RCLCPP_INFO(get_logger(), 
                "📊 IK统计: 左臂成功率=%.1f%%, 右臂成功率=%.1f%%, 总计=%d次",
                left_rate, right_rate, solve_count_);
        }
    }
    
    bool solveLeftArmIK()
    {
        // === 步骤1: TF坐标系转换 ⚠️ 关键 ===
        // 输入: left_target_ 在 vr_origin 坐标系下
        // 需要: 转换到 base_link_left 坐标系
        // TF自动处理完整链: vr_origin → teleop_base → base_link → base_link_left
        
        geometry_msgs::msg::PoseStamped target_in_base_left;
        
        try {
            // 使用TF转换到base_link_left坐标系
            target_in_base_left = tf_buffer_->transform(
                *left_target_, 
                "base_link_left",
                tf2::durationFromSec(0.1)  // 100ms超时
            );
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "左臂TF转换失败: %s", ex.what());
            return false;
        }
        
        // 提取转换后的位姿（已经在base_link_left坐标系下）
        tf2::Quaternion q_base_left(
            target_in_base_left.pose.orientation.x,
            target_in_base_left.pose.orientation.y,
            target_in_base_left.pose.orientation.z,
            target_in_base_left.pose.orientation.w
        );
        
        tf2::Vector3 pos_base_left(
            target_in_base_left.pose.position.x,
            target_in_base_left.pose.position.y,
            target_in_base_left.pose.position.z
        );
        
        // === 步骤2: 应用末端坐标系校正 ⚠️ 关键 ===
        // human_hand坐标系: [X前, Y左, Z上] (人手语义)
        // lt坐标系: [X左, Y上, Z后] (JAKA末端)
        // 
        // 坐标轴映射：
        //   lt的X = human的Y  (左→左，方向一致)
        //   lt的Y = human的Z  (上→上，方向一致)
        //   lt的Z = human的X  (前→前，方向一致) ✅
        // 旋转矩阵: R_lt_human = [[0,1,0], [0,0,1], [1,0,0]]
        
        // 从旋转矩阵创建tf2::Matrix3x3
        // ⭐ 正确映射：所有轴方向一致，无需取反
        tf2::Matrix3x3 R_correction(
            0.0,  1.0,  0.0,   // 第1行: lt.X = human.Y (左→左)
            0.0,  0.0,  1.0,   // 第2行: lt.Y = human.Z (上→上)
            1.0,  0.0,  0.0    // 第3行: lt.Z = human.X (前→前) ✅
        );
        
        // 从旋转矩阵提取四元数
        tf2::Quaternion q_correction;
        R_correction.getRotation(q_correction);
        
        // 应用旋转校正到姿态（注意：只旋转姿态，不旋转位置）
        // ⭐ 变换链：base_link_left → human_left_hand → lt
        // 右乘：R_lt = R_human * R_human_to_lt
        tf2::Matrix3x3 R_base_left(q_base_left);
        tf2::Matrix3x3 R_corrected = R_base_left * R_correction;  // ✅ 右乘！
        
        tf2::Quaternion q_corrected;
        R_corrected.getRotation(q_corrected);
        q_corrected.normalize();
        
        // 位置不变（位置已经在base_link_left坐标系下，不需要旋转）
        tf2::Vector3 pos_corrected = pos_base_left;
        
        // === 发布调试TF（可选）===
        if (publish_debug_tf_) {
            geometry_msgs::msg::TransformStamped debug_tf;
            debug_tf.header.stamp = now();
            debug_tf.header.frame_id = "base_link_left";
            debug_tf.child_frame_id = "left_hand_corrected";
            debug_tf.transform.translation.x = pos_corrected.x();
            debug_tf.transform.translation.y = pos_corrected.y();
            debug_tf.transform.translation.z = pos_corrected.z();
            debug_tf.transform.rotation.x = q_corrected.x();
            debug_tf.transform.rotation.y = q_corrected.y();
            debug_tf.transform.rotation.z = q_corrected.z();
            debug_tf.transform.rotation.w = q_corrected.w();
            tf_broadcaster_->sendTransform(debug_tf);
        }
        
        // === 步骤2: 转换到JAKA格式 ===
        CartesianPose target_pose;
        target_pose.tran.x = pos_corrected.x() * 1000.0;  // m -> mm
        target_pose.tran.y = pos_corrected.y() * 1000.0;
        target_pose.tran.z = pos_corrected.z() * 1000.0;
        
        // 四元数转欧拉角（RPY，弧度）
        tf2::Matrix3x3 m(q_corrected);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        target_pose.rpy.rx = roll;
        target_pose.rpy.ry = pitch;
        target_pose.rpy.rz = yaw;
        
        // 选择IK参考: 优先使用当前实际关节位置，否则使用初始参考
        JointValue* ref_joints = has_current_left_ ? &current_left_joints_ : &ref_left_joints_;
        
        // 调用IK求解 - robot_id=0表示左臂
        // 输入位姿已经在base_link_left坐标系下，且已应用末端校正
        JointValue ik_result;
        errno_t ret = robot_->kine_inverse(0, ref_joints, &target_pose, &ik_result);
        
        if (ret == ERR_SUCC) {
            // === 步骤3: 安全检查 ===
            // 检查关节位置限位
            if (!checkJointLimits(ik_result, "左臂")) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "左臂IK结果超出关节限位，跳过本次指令");
                return false;
            }
            
            // 检查关节速度
            if (has_prev_left_) {
                double dt = (now() - last_solve_time_).seconds();
                if (!checkJointVelocity(ik_result, prev_left_joints_, dt, "左臂")) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "左臂关节速度超限，跳过本次指令");
                    return false;
                }
            }
            
            // 记录当前关节值供下次速度检查
            prev_left_joints_ = ik_result;
            has_prev_left_ = true;
            
            // 注意: 不再更新ref_joints，因为使用实际joint_states作为参考
            
            // 发布关节指令
            publishJointCommand(ik_result, left_joint_pub_);
            return true;
        } else {
            if (left_error_count_++ % 100 == 0) {
                RCLCPP_WARN(get_logger(), "左臂IK失败 (错误计数: %d, 错误码: %d)", 
                    left_error_count_, ret);
            }
            return false;
        }
    }
    
    bool solveRightArmIK()
    {
        // === 步骤1: TF坐标系转换 ⚠️ 关键 ===
        // 输入: right_target_ 在 vr_origin 坐标系下
        // 需要: 转换到 base_link_right 坐标系
        
        geometry_msgs::msg::PoseStamped target_in_base_right;
        
        try {
            // 使用TF转换到base_link_right坐标系
            target_in_base_right = tf_buffer_->transform(
                *right_target_, 
                "base_link_right",
                tf2::durationFromSec(0.1)  // 100ms超时
            );
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "右臂TF转换失败: %s", ex.what());
            return false;
        }
        
        // 提取转换后的位姿
        tf2::Quaternion q_base_right(
            target_in_base_right.pose.orientation.x,
            target_in_base_right.pose.orientation.y,
            target_in_base_right.pose.orientation.z,
            target_in_base_right.pose.orientation.w
        );
        
        tf2::Vector3 pos_base_right(
            target_in_base_right.pose.position.x,
            target_in_base_right.pose.position.y,
            target_in_base_right.pose.position.z
        );
        
        // === 步骤2: 应用末端坐标系校正 ⚠️ 关键 ===
        // human_hand坐标系: [X前, Y左, Z上]
        // rt坐标系: [X左, Y上, Z后] (与lt相同，只是左右臂镜像安装)
        // 
        // 坐标轴映射：
        //   rt的X = human的Y  (左→左，方向一致)
        //   rt的Y = human的Z  (上→上，方向一致)
        //   rt的Z = human的X  (前→前，方向一致) ✅
        // 使用与左臂相同的旋转矩阵
        // R_rt_human = [[0,1,0], [0,0,1], [1,0,0]]
        
        // ⭐ 所有轴方向一致，无需取反
        tf2::Matrix3x3 R_correction(
            0.0,  1.0,  0.0,
            0.0,  0.0,  1.0,
            1.0,  0.0,  0.0   // rt.Z = human.X (前→前) ✅
        );
        
        tf2::Quaternion q_correction;
        R_correction.getRotation(q_correction);
        
        // 应用旋转校正到姿态
        // ⭐ 变换链：base_link_right → human_right_hand → rt
        // 右乘：R_rt = R_human * R_human_to_rt
        tf2::Matrix3x3 R_base_right(q_base_right);
        tf2::Matrix3x3 R_corrected = R_base_right * R_correction;  // ✅ 右乘！
        
        tf2::Quaternion q_corrected;
        R_corrected.getRotation(q_corrected);
        q_corrected.normalize();
        
        // 位置不变
        tf2::Vector3 pos_corrected = pos_base_right;
        
        // === 发布调试TF（可选）===
        if (publish_debug_tf_) {
            geometry_msgs::msg::TransformStamped debug_tf;
            debug_tf.header.stamp = now();
            debug_tf.header.frame_id = "base_link_right";
            debug_tf.child_frame_id = "right_hand_corrected";
            debug_tf.transform.translation.x = pos_corrected.x();
            debug_tf.transform.translation.y = pos_corrected.y();
            debug_tf.transform.translation.z = pos_corrected.z();
            debug_tf.transform.rotation.x = q_corrected.x();
            debug_tf.transform.rotation.y = q_corrected.y();
            debug_tf.transform.rotation.z = q_corrected.z();
            debug_tf.transform.rotation.w = q_corrected.w();
            tf_broadcaster_->sendTransform(debug_tf);
        }
        
        // === 步骤2: 转换到JAKA格式 ===
        CartesianPose target_pose;
        target_pose.tran.x = pos_corrected.x() * 1000.0;  // m -> mm
        target_pose.tran.y = pos_corrected.y() * 1000.0;
        target_pose.tran.z = pos_corrected.z() * 1000.0;
        
        // 四元数转欧拉角
        tf2::Matrix3x3 m(q_corrected);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        target_pose.rpy.rx = roll;
        target_pose.rpy.ry = pitch;
        target_pose.rpy.rz = yaw;
        
        // 选择IK参考: 优先使用当前实际关节位置，否则使用初始参考
        JointValue* ref_joints = has_current_right_ ? &current_right_joints_ : &ref_right_joints_;
        
        // 调用IK求解 - robot_id=1表示右臂
        // 输入位姿已经在base_link_right坐标系下，且已应用末端校正
        JointValue ik_result;
        errno_t ret = robot_->kine_inverse(1, ref_joints, &target_pose, &ik_result);
        
        if (ret == ERR_SUCC) {
            // === 步骤3: 安全检查 ===
            // 检查关节位置限位
            if (!checkJointLimits(ik_result, "右臂")) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "右臂IK结果超出关节限位，跳过本次指令");
                return false;
            }
            
            // 检查关节速度
            if (has_prev_right_) {
                double dt = (now() - last_solve_time_).seconds();
                if (!checkJointVelocity(ik_result, prev_right_joints_, dt, "右臂")) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "右臂关节速度超限，跳过本次指令");
                    return false;
                }
            }
            
            // 记录当前关节值供下次速度检查
            prev_right_joints_ = ik_result;
            has_prev_right_ = true;
            
            // 注意: 不再更新ref_joints，因为使用实际joint_states作为参考
            
            // 发布关节指令
            publishJointCommand(ik_result, right_joint_pub_);
            return true;
        } else {
            if (right_error_count_++ % 100 == 0) {
                RCLCPP_WARN(get_logger(), "右臂IK失败 (错误计数: %d, 错误码: %d)", 
                    right_error_count_, ret);
            }
            return false;
        }
    }
    
    /**
     * @brief 检查关节位置是否在安全范围内
     * @param joints 待检查的关节值
     * @param arm_name 机械臂名称（用于日志）
     * @return true表示安全，false表示超限
     */
    bool checkJointLimits(const JointValue& joints, const std::string& arm_name)
    {
        bool safe = true;
        for (int i = 0; i < 7; i++) {
            double pos = joints.jVal[i];
            double min_safe = JAKA_ZU7_LIMITS[i].pos_min + SAFETY_MARGIN_POS;
            double max_safe = JAKA_ZU7_LIMITS[i].pos_max - SAFETY_MARGIN_POS;
            
            if (pos < min_safe || pos > max_safe) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "⚠️ %s 关节%d 超限: %.3f rad (安全范围: [%.3f, %.3f])",
                    arm_name.c_str(), i+1, pos, min_safe, max_safe);
                safe = false;
            }
        }
        return safe;
    }
    
    /**
     * @brief 检查关节速度是否在安全范围内
     * @param joints 当前关节值
     * @param prev_joints 上一次关节值
     * @param dt 时间间隔（秒）
     * @param arm_name 机械臂名称（用于日志）
     * @return true表示安全，false表示超速
     */
    bool checkJointVelocity(const JointValue& joints, const JointValue& prev_joints, 
                           double dt, const std::string& arm_name)
    {
        if (dt <= 0.0) return true;
        
        bool safe = true;
        for (int i = 0; i < 7; i++) {
            double vel = std::abs((joints.jVal[i] - prev_joints.jVal[i]) / dt);
            double max_safe_vel = JAKA_ZU7_LIMITS[i].vel_max * SAFETY_MARGIN_VEL;
            
            if (vel > max_safe_vel) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "⚠️ %s 关节%d 超速: %.3f rad/s (限制: %.3f rad/s)",
                    arm_name.c_str(), i+1, vel, max_safe_vel);
                safe = false;
            }
        }
        return safe;
    }
    
    void publishJointCommand(const JointValue& joints, 
                            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub)
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = now();
        msg.name = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"};
        msg.position = {
            joints.jVal[0], joints.jVal[1], joints.jVal[2],
            joints.jVal[3], joints.jVal[4], joints.jVal[5],
            joints.jVal[6]
        };
        pub->publish(msg);
    }

    // ROS相关
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;  // 订阅实际关节状态
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_joint_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ik_status_pub_;
    
    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // JAKA SDK
    std::unique_ptr<JAKAZuRobot> robot_;
    std::string robot_ip_;
    bool connected_{false};
    bool auto_connect_{true};
    
    // 目标位姿
    geometry_msgs::msg::PoseStamped::SharedPtr left_target_;
    geometry_msgs::msg::PoseStamped::SharedPtr right_target_;
    bool has_left_target_{false};
    bool has_right_target_{false};
    
    // 参考关节位置（初始化时的零位，用作fallback）
    JointValue ref_left_joints_;
    JointValue ref_right_joints_;
    
    // 当前实际关节位置（从/joint_states获取，用作IK参考）
    JointValue current_left_joints_;
    JointValue current_right_joints_;
    bool has_current_left_{false};
    bool has_current_right_{false};
    
    // 上次关节位置（用于速度检查）
    JointValue prev_left_joints_;
    JointValue prev_right_joints_;
    bool has_prev_left_{false};
    bool has_prev_right_{false};
    rclcpp::Time last_solve_time_;
    
    // 配置
    double ik_rate_;
    bool use_tf_lookup_;
    bool publish_debug_tf_;
    
    // 统计
    int solve_count_{0};
    int left_success_count_{0};
    int right_success_count_{0};
    int left_error_count_{0};
    int right_error_count_{0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualArmIKSolverNode>();
    
    RCLCPP_INFO(node->get_logger(), " ");
    RCLCPP_INFO(node->get_logger(), "🚀 双臂IK求解器运行中...");
    RCLCPP_INFO(node->get_logger(), "📌 作为第二个客户端连接到JAKA控制器");
    RCLCPP_INFO(node->get_logger(), " ");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
