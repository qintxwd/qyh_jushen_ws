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
#include <qyh_jaka_control_msgs/msg/robot_state.hpp>
#include <JAKAZuRobot.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <memory>
#include <array>
#include <cmath>

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
const double SAFETY_MARGIN_VEL = 1.0;     // 允许超过标称速度20%（考虑IK求解的突变）

// # left joint = 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000
// # right joint = 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000
// # left pos = -0.626130, 989.737160, 219.885132, 1.572874, -0.000000, -3.141593
// # right pos = -0.679880, -989.449941, 217.950931, 1.574990, 0.000000, 0.000000
// //定义一个最初的参考的关节位置结构体
// std::array<double, 7> JAKA_ZU7_REF_DEFAULT_JOINT_LEFT =
// {
//     0.268,-59.017,0.195,-80.121,-0.109,80.051,0.014 //单位为度
// };
// std::array<double, 7> JAKA_ZU7_REF_DEFAULT_JOINT_RIGHT =
// {
//     -0.089,-65.010,-0.34,-79.964,0.263,-99.974,-0.016 //单位为度
// };

// //定义一个最初的参考的关节位置结构体
// std::array<double, 7> JAKA_ZU7_REF_DEFAULT_JOINT_LEFT =
// {
//     0.,0.,0.,0.,0.,0.,0., //单位为弧度
// };
// std::array<double, 7> JAKA_ZU7_REF_DEFAULT_JOINT_RIGHT =
// {
//     0.,0.,0.,0.,0.,0.,0., //单位为弧度
// };

//定义一个最初的参考的关节位置结构体
// # left joint = 0.004677, -1.030041, 0.003351, -1.398358, -0.001902, 1.397188, 0.000262
// # right joint = -0.001571, -1.134639, -0.005952, -1.395653, 0.004590, -1.744875, -0.000279
std::array<double, 7> JAKA_ZU7_REF_DEFAULT_JOINT_LEFT =
{
    0.004677, -1.030041, 0.003351, -1.398358, -0.001902, 1.397188, 0.000262, //单位为弧度
};
std::array<double, 7> JAKA_ZU7_REF_DEFAULT_JOINT_RIGHT =
{
    -0.001571, -1.134639, -0.005952, -1.395653, 0.004590, -1.744875, -0.000279, //单位为弧度
};

static inline double deg2rad(double d) { return d * M_PI / 180.0; }

// 归一化角度到[-π, π]范围
static inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

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
        declare_parameter<bool>("publish_debug_tf", true);  // 发布调试TF
        declare_parameter<bool>("target_x_left", false);  // 发给我们的目标是否x轴向左的，默认发给我们的是x轴向前的，我们需要旋转，所以默认false，如果发给我们的是x轴向左的，就改成true
        declare_parameter<bool>("has_z_offset", true);  // 是否有Z轴偏移
        declare_parameter<double>("left_z_offset", 0.219885132);  // 左臂Z轴偏移
        declare_parameter<double>("right_z_offset", 0.217950931); // 右臂Z轴偏移

        robot_ip_ = get_parameter("robot_ip").as_string();
        ik_rate_ = get_parameter("ik_rate").as_double();
        auto_connect_ = get_parameter("auto_connect").as_bool();
        publish_debug_tf_ = get_parameter("publish_debug_tf").as_bool();
        target_x_left_ = get_parameter("target_x_left").as_bool();
        has_z_offset_ = get_parameter("has_z_offset").as_bool();
        left_z_offset_ = get_parameter("left_z_offset").as_double();
        right_z_offset_ = get_parameter("right_z_offset").as_double();
        
        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "  双臂IK求解节点启动");
        RCLCPP_INFO(get_logger(), "===========================================");
        RCLCPP_INFO(get_logger(), "控制器IP: %s", robot_ip_.c_str());
        RCLCPP_INFO(get_logger(), "IK求解频率: %.1f Hz", ik_rate_);
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
        
        // 订阅机械臂真实位姿状态
        robot_state_sub_ = create_subscription<qyh_jaka_control_msgs::msg::RobotState>(
            "/jaka/robot_state", 10,
            std::bind(&DualArmIKSolverNode::robotStateCallback, this, std::placeholders::_1));
        
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
        // 使用默认参考关节（从度转换为弧度）
        for (int i = 0; i < 7; i++) {
            ref_left_joints_.jVal[i] = JAKA_ZU7_REF_DEFAULT_JOINT_LEFT[i];//deg2rad(JAKA_ZU7_REF_DEFAULT_JOINT_LEFT[i]);
            ref_right_joints_.jVal[i] = JAKA_ZU7_REF_DEFAULT_JOINT_RIGHT[i];//deg2rad(JAKA_ZU7_REF_DEFAULT_JOINT_RIGHT[i]);
        }

        RCLCPP_INFO(get_logger(), "✓ 参考关节位置已设置为默认值（度->弧度）");
        RCLCPP_INFO(get_logger(), "  左臂参考关节位置:");
        for (int i = 0; i < 7; i++) {
            RCLCPP_INFO(get_logger(), "    关节 %d: %.4f rad", i + 1, ref_left_joints_.jVal[i]);
        }
        RCLCPP_INFO(get_logger(), "  右臂参考关节位置:");
        for (int i = 0; i < 7; i++) {
            RCLCPP_INFO(get_logger(), "    关节 %d: %.4f rad", i + 1, ref_right_joints_.jVal[i]);
        }
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
    
    void robotStateCallback(const qyh_jaka_control_msgs::msg::RobotState::SharedPtr msg)
    {
        // 更新机械臂真实位姿
        left_real_pose_ = msg->left_cartesian_pose;
        right_real_pose_ = msg->right_cartesian_pose;
        has_left_real_pose_ = true;
        has_right_real_pose_ = true;
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

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "✓ 接收到当前关节状态 (左臂第1关节: %.3f rad, 右臂第1关节: %.3f rad)",
                current_left_joints_.jVal[0], current_right_joints_.jVal[0]);
        }
    }
    
    void ikSolverCallback()
    {
        if (!connected_) {
            return;
        }

        if(!has_left_target_ && !has_right_target_) {
            // 没有目标位姿，跳过求解
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
        
        // 检查消息新鲜度（避免使用过期的VR目标）
        auto msg_age = (now() - left_target_->header.stamp).seconds();
        if (msg_age > 1.0) {
            // 消息超过1秒，可能是clutch已松开或VR断开
            // 重置标志位，避免反复处理过期消息
            has_left_target_ = false;
            return false;
        }
        
        try {
            // 使用最新的TF变换（而不是消息时间戳对应的变换）
            // 这样即使消息稍旧，也能用当前TF树进行转换
            geometry_msgs::msg::PoseStamped input_pose = *left_target_;
            input_pose.header.stamp = rclcpp::Time(0);
            target_in_base_left = tf_buffer_->transform(
                input_pose, 
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

        // 如果有Z轴偏移，应用偏移
        if(has_z_offset_) {
            pos_base_left.setZ(pos_base_left.z() + left_z_offset_);
        }
        
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

        if(target_x_left_) {
            // 如果目标本来就是x轴向左的，则不需要旋转，使用单位矩阵
            R_correction = tf2::Matrix3x3(
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            );
        }
        
        // 从旋转矩阵提取四元数
        tf2::Quaternion q_correction;
        R_correction.getRotation(q_correction);
        
        // 应用旋转校正到姿态（注意：只旋转姿态，不旋转位置）
        // ⭐ 变换链：base_link_left → human_left_hand → lt
        // 右乘：R_lt = R_human * R_human_to_lt
        tf2::Matrix3x3 R_base_left(q_base_left);
        tf2::Matrix3x3 R_corrected = R_base_left * R_correction.transpose(); 
        
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
            
            // 检查关节速度（相对于当前机器人真实位置）
            if (has_current_left_) {
                double dt = (now() - last_solve_time_).seconds();
                if (dt > 0.0 && !checkJointVelocity(ik_result, current_left_joints_, dt, "左臂")) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "左臂关节速度超限，跳过本次指令");
                    return false;
                }
            }
            
            // 注意: 不再更新ref_joints，因为使用实际joint_states作为参考
            
            // 发布关节指令
            publishJointCommand(ik_result, left_joint_pub_);
            return true;
        } else {
            if (left_error_count_++ % 100 == 0) {
                RCLCPP_WARN(get_logger(), "左臂IK失败 (错误计数: %d, 错误码: %d)", 
                    left_error_count_, ret);
                
                // 输出参考关节位置
                RCLCPP_WARN(get_logger(), "当前参考关节位置:");
                for(int i=0; i<7; i++) {
                    RCLCPP_WARN(get_logger(), "  关节 %d: %.4f rad", i+1, ref_joints->jVal[i]);
                }
                
                // 输出目标位姿
                tf2::Quaternion q_target;
                q_target.setRPY(target_pose.rpy.rx, target_pose.rpy.ry, target_pose.rpy.rz);
                RCLCPP_WARN(get_logger(), "目标位姿 (target_pose, 在lt坐标系):");
                RCLCPP_WARN(get_logger(), "  位置: x=%.2f mm, y=%.2f mm, z=%.2f mm", 
                    target_pose.tran.x, target_pose.tran.y, target_pose.tran.z);
                RCLCPP_WARN(get_logger(), "  姿态RPY: rx=%.4f rad, ry=%.4f rad, rz=%.4f rad", 
                    target_pose.rpy.rx, target_pose.rpy.ry, target_pose.rpy.rz);
                RCLCPP_WARN(get_logger(), "  姿态Quat: [%.4f, %.4f, %.4f, %.4f]",
                    q_target.x(), q_target.y(), q_target.z(), q_target.w());
                
                // 输出真实位姿并计算偏差
                if (has_left_real_pose_) {
                    // real_pose原始四元数（可能在base_link_left坐标系）
                    tf2::Quaternion q_real_raw(
                        left_real_pose_.orientation.x,
                        left_real_pose_.orientation.y,
                        left_real_pose_.orientation.z,
                        left_real_pose_.orientation.w
                    );
                    
                    // ⚠️ 关键：应用R_correction变换到lt坐标系
                    // real_pose_lt = real_pose_base * R_correction^T
                    tf2::Matrix3x3 R_correction_left(
                        0.0,  1.0,  0.0,
                        0.0,  0.0,  1.0,
                        1.0,  0.0,  0.0
                    );
                    if(target_x_left_) {
                        R_correction_left = tf2::Matrix3x3(
                            1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0
                        );
                    }
                    tf2::Matrix3x3 R_real_raw(q_real_raw);
                    tf2::Matrix3x3 R_real_corrected = R_real_raw * R_correction_left.transpose();
                    tf2::Quaternion q_real;
                    R_real_corrected.getRotation(q_real);
                    q_real.normalize();
                    
                    tf2::Matrix3x3 m_real(q_real);
                    double real_roll, real_pitch, real_yaw;
                    m_real.getRPY(real_roll, real_pitch, real_yaw);
                    
                    RCLCPP_WARN(get_logger(), "机械臂真实位姿 (real_pose, 原始):");
                    RCLCPP_WARN(get_logger(), "  位置: x=%.2f mm, y=%.2f mm, z=%.2f mm", 
                        left_real_pose_.position.x * 1000.0, left_real_pose_.position.y * 1000.0, left_real_pose_.position.z * 1000.0);
                    double raw_roll, raw_pitch, raw_yaw;
                    tf2::Matrix3x3(q_real_raw).getRPY(raw_roll, raw_pitch, raw_yaw);
                    RCLCPP_WARN(get_logger(), "  姿态Quat(原始): [%.4f, %.4f, %.4f, %.4f]",
                        q_real_raw.x(), q_real_raw.y(), q_real_raw.z(), q_real_raw.w());
                    RCLCPP_WARN(get_logger(), "  姿态RPY(原始): rx=%.4f, ry=%.4f, rz=%.4f rad", raw_roll, raw_pitch, raw_yaw);
                    RCLCPP_WARN(get_logger(), "机械臂真实位姿 (real_pose, 变换到lt坐标系后):");
                    RCLCPP_WARN(get_logger(), "  姿态Quat(lt): [%.4f, %.4f, %.4f, %.4f]",
                        q_real.x(), q_real.y(), q_real.z(), q_real.w());
                    RCLCPP_WARN(get_logger(), "  姿态RPY(lt): rx=%.4f rad, ry=%.4f rad, rz=%.4f rad", 
                        real_roll, real_pitch, real_yaw);
                    
                    // 计算位置偏差（real_pose单位是m，target_pose单位是mm）
                    double dx = target_pose.tran.x - left_real_pose_.position.x * 1000.0;
                    double dy = target_pose.tran.y - left_real_pose_.position.y * 1000.0;
                    double dz = target_pose.tran.z - left_real_pose_.position.z * 1000.0;
                    double pos_error = std::sqrt(dx*dx + dy*dy + dz*dz);
                    
                    // 计算RPY姿态偏差（归一化到[-π,π]）
                    double drx_norm = normalizeAngle(target_pose.rpy.rx - real_roll);
                    double dry_norm = normalizeAngle(target_pose.rpy.ry - real_pitch);
                    double drz_norm = normalizeAngle(target_pose.rpy.rz - real_yaw);
                    
                    // 计算四元数旋转误差（更准确）
                    tf2::Quaternion q_target;
                    q_target.setRPY(target_pose.rpy.rx, target_pose.rpy.ry, target_pose.rpy.rz);
                    tf2::Quaternion q_error = q_real.inverse() * q_target;
                    q_error.normalize();
                    double error_angle = 2.0 * std::acos(std::min(1.0, std::abs(q_error.w())));
                    tf2::Vector3 error_axis(q_error.x(), q_error.y(), q_error.z());
                    double axis_len = error_axis.length();
                    if (axis_len > 1e-6) {
                        error_axis /= axis_len;
                    }
                    
                    RCLCPP_WARN(get_logger(), "位姿偏差:");
                    RCLCPP_WARN(get_logger(), "  位置偏差: dx=%.2f mm, dy=%.2f mm, dz=%.2f mm, 总偏差=%.2f mm", 
                        dx, dy, dz, pos_error);
                    RCLCPP_WARN(get_logger(), "  RPY偏差(归一化): drx=%.4f rad (%.2f°), dry=%.4f rad (%.2f°), drz=%.4f rad (%.2f°)", 
                        drx_norm, drx_norm*180.0/M_PI, dry_norm, dry_norm*180.0/M_PI, drz_norm, drz_norm*180.0/M_PI);
                    RCLCPP_WARN(get_logger(), "  四元数旋转误差: %.4f rad (%.2f°), 旋转轴=[%.3f, %.3f, %.3f]",
                        error_angle, error_angle*180.0/M_PI, error_axis.x(), error_axis.y(), error_axis.z());
                } else {
                    RCLCPP_WARN(get_logger(), "⚠️  未收到机械臂真实位姿数据");
                }
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
        
        // 检查消息新鲜度
        auto msg_age = (now() - right_target_->header.stamp).seconds();
        if (msg_age > 1.0) {
            // 重置标志位，避免反复处理过期消息
            has_right_target_ = false;
            return false;
        }
        
        try {
            // 使用最新的TF变换
            geometry_msgs::msg::PoseStamped input_pose = *right_target_;
            input_pose.header.stamp = rclcpp::Time(0);
            target_in_base_right = tf_buffer_->transform(
                input_pose, 
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

        // 如果有Z轴偏移，应用偏移
        if(has_z_offset_) {
            pos_base_right.setZ(pos_base_right.z() + right_z_offset_);
        }
        
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

        if(target_x_left_) {
            // 如果目标本来就是x轴向左的，则不需要旋转，使用单位矩阵
            R_correction = tf2::Matrix3x3(
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0
            );
        }
        
        tf2::Quaternion q_correction;
        R_correction.getRotation(q_correction);
        
        // 应用旋转校正到姿态
        // ⭐ 变换链：base_link_right → human_right_hand → rt
        // 右乘：R_rt = R_human * R_human_to_rt
        tf2::Matrix3x3 R_base_right(q_base_right);
        tf2::Matrix3x3 R_corrected = R_base_right * R_correction.transpose();  
        
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
            
            // 检查关节速度（相对于当前机器人真实位置）
            if (has_current_right_) {
                double dt = (now() - last_solve_time_).seconds();
                if (dt > 0.0 && !checkJointVelocity(ik_result, current_right_joints_, dt, "右臂")) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "右臂关节速度超限，跳过本次指令");
                    return false;
                }
            }
            
            // 注意: 不再更新ref_joints，因为使用实际joint_states作为参考
            
            // 发布关节指令
            publishJointCommand(ik_result, right_joint_pub_);
            return true;
        } else {
            if (right_error_count_++ % 100 == 0) {
                RCLCPP_WARN(get_logger(), "右臂IK失败 (错误计数: %d, 错误码: %d)", 
                    right_error_count_, ret);
                
                // 输出参考关节位置
                RCLCPP_WARN(get_logger(), "当前参考关节位置:");
                for(int i=0; i<7; i++) {
                    RCLCPP_WARN(get_logger(), "  关节 %d: %.4f rad", i+1, ref_joints->jVal[i]);
                }
                
                // 输出目标位姿
                tf2::Quaternion q_target_right;
                q_target_right.setRPY(target_pose.rpy.rx, target_pose.rpy.ry, target_pose.rpy.rz);
                RCLCPP_WARN(get_logger(), "目标位姿 (target_pose, 在rt坐标系):");
                RCLCPP_WARN(get_logger(), "  位置: x=%.2f mm, y=%.2f mm, z=%.2f mm", 
                    target_pose.tran.x, target_pose.tran.y, target_pose.tran.z);
                RCLCPP_WARN(get_logger(), "  姿态RPY: rx=%.4f rad, ry=%.4f rad, rz=%.4f rad", 
                    target_pose.rpy.rx, target_pose.rpy.ry, target_pose.rpy.rz);
                RCLCPP_WARN(get_logger(), "  姿态Quat: [%.4f, %.4f, %.4f, %.4f]",
                    q_target_right.x(), q_target_right.y(), q_target_right.z(), q_target_right.w());
                
                // 输出真实位姿并计算偏差
                if (has_right_real_pose_) {
                    // real_pose原始四元数（可能在base_link_right坐标系）
                    tf2::Quaternion q_real_raw(
                        right_real_pose_.orientation.x,
                        right_real_pose_.orientation.y,
                        right_real_pose_.orientation.z,
                        right_real_pose_.orientation.w
                    );
                    
                    // ⚠️ 关键：应用R_correction变换到rt坐标系
                    // real_pose_rt = real_pose_base * R_correction^T
                    tf2::Matrix3x3 R_correction_right(
                        0.0,  1.0,  0.0,
                        0.0,  0.0,  1.0,
                        1.0,  0.0,  0.0
                    );
                    if(target_x_left_) {
                        R_correction_right = tf2::Matrix3x3(
                            1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0
                        );
                    }
                    tf2::Matrix3x3 R_real_raw(q_real_raw);
                    tf2::Matrix3x3 R_real_corrected = R_real_raw * R_correction_right.transpose();
                    tf2::Quaternion q_real;
                    R_real_corrected.getRotation(q_real);
                    q_real.normalize();
                    
                    tf2::Matrix3x3 m_real(q_real);
                    double real_roll, real_pitch, real_yaw;
                    m_real.getRPY(real_roll, real_pitch, real_yaw);
                    
                    RCLCPP_WARN(get_logger(), "机械臂真实位姿 (real_pose, 原始):");
                    RCLCPP_WARN(get_logger(), "  位置: x=%.2f mm, y=%.2f mm, z=%.2f mm", 
                        right_real_pose_.position.x * 1000.0, right_real_pose_.position.y * 1000.0, right_real_pose_.position.z * 1000.0);
                    double raw_roll, raw_pitch, raw_yaw;
                    tf2::Matrix3x3(q_real_raw).getRPY(raw_roll, raw_pitch, raw_yaw);
                    RCLCPP_WARN(get_logger(), "  姿态Quat(原始): [%.4f, %.4f, %.4f, %.4f]",
                        q_real_raw.x(), q_real_raw.y(), q_real_raw.z(), q_real_raw.w());
                    RCLCPP_WARN(get_logger(), "  姿态RPY(原始): rx=%.4f, ry=%.4f, rz=%.4f rad", raw_roll, raw_pitch, raw_yaw);
                    RCLCPP_WARN(get_logger(), "机械臂真实位姿 (real_pose, 变换到rt坐标系后):");
                    RCLCPP_WARN(get_logger(), "  姿态Quat(rt): [%.4f, %.4f, %.4f, %.4f]",
                        q_real.x(), q_real.y(), q_real.z(), q_real.w());
                    RCLCPP_WARN(get_logger(), "  姿态RPY(rt): rx=%.4f rad, ry=%.4f rad, rz=%.4f rad", 
                        real_roll, real_pitch, real_yaw);
                    
                    // 计算位置偏差（real_pose单位是m，target_pose单位是mm）
                    double dx = target_pose.tran.x - right_real_pose_.position.x * 1000.0;
                    double dy = target_pose.tran.y - right_real_pose_.position.y * 1000.0;
                    double dz = target_pose.tran.z - right_real_pose_.position.z * 1000.0;
                    double pos_error = std::sqrt(dx*dx + dy*dy + dz*dz);
                    
                    // 计算RPY姿态偏差（归一化到[-π,π]）
                    double drx_norm = normalizeAngle(target_pose.rpy.rx - real_roll);
                    double dry_norm = normalizeAngle(target_pose.rpy.ry - real_pitch);
                    double drz_norm = normalizeAngle(target_pose.rpy.rz - real_yaw);
                    
                    // 计算四元数旋转误差（更准确，现在都在rt坐标系）
                    tf2::Quaternion q_error = q_real.inverse() * q_target_right;
                    q_error.normalize();
                    double error_angle = 2.0 * std::acos(std::min(1.0, std::abs(q_error.w())));
                    tf2::Vector3 error_axis(q_error.x(), q_error.y(), q_error.z());
                    double axis_len = error_axis.length();
                    if (axis_len > 1e-6) {
                        error_axis /= axis_len;
                    }
                    
                    RCLCPP_WARN(get_logger(), "位姿偏差:");
                    RCLCPP_WARN(get_logger(), "  位置偏差: dx=%.2f mm, dy=%.2f mm, dz=%.2f mm, 总偏差=%.2f mm", 
                        dx, dy, dz, pos_error);
                    RCLCPP_WARN(get_logger(), "  RPY偏差(归一化): drx=%.4f rad (%.2f°), dry=%.4f rad (%.2f°), drz=%.4f rad (%.2f°)", 
                        drx_norm, drx_norm*180.0/M_PI, dry_norm, dry_norm*180.0/M_PI, drz_norm, drz_norm*180.0/M_PI);
                    RCLCPP_WARN(get_logger(), "  四元数旋转误差: %.4f rad (%.2f°), 旋转轴=[%.3f, %.3f, %.3f]",
                        error_angle, error_angle*180.0/M_PI, error_axis.x(), error_axis.y(), error_axis.z());
                } else {
                    RCLCPP_WARN(get_logger(), "⚠️  未收到机械臂真实位姿数据");
                }
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
                    "⚠️ %s 关节%d 超速: %.3f rad/s (限制: %.3f rad/s), dt=%.3f s, joint_delta=%.3f rad",
                    arm_name.c_str(), i+1, vel, max_safe_vel, dt, std::abs(joints.jVal[i] - prev_joints.jVal[i]));
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
    rclcpp::Subscription<qyh_jaka_control_msgs::msg::RobotState>::SharedPtr robot_state_sub_;  // 订阅机械臂真实位姿
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
    
    // 机械臂真实位姿（从/jaka/robot_state获取）
    geometry_msgs::msg::Pose left_real_pose_;
    geometry_msgs::msg::Pose right_real_pose_;
    bool has_left_real_pose_{false};
    bool has_right_real_pose_{false};
    
    // 上次关节位置（用于速度检查）
    JointValue prev_left_joints_;
    JointValue prev_right_joints_;
    bool has_prev_left_{false};
    bool has_prev_right_{false};
    rclcpp::Time last_solve_time_;
    
    // 配置
    double ik_rate_;
    bool publish_debug_tf_;
    bool target_x_left_;  // 目标是否为x轴向左的
    bool has_z_offset_;
    double left_z_offset_;
    double right_z_offset_;
    
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
