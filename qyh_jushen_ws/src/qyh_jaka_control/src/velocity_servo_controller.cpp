#include "qyh_jaka_control/velocity_servo_controller.hpp"
#include <urdf/model.h>
#include <fstream>
#include <iostream>

namespace qyh_jaka_control {

VelocityServoController::VelocityServoController(rclcpp::Node::SharedPtr node, const std::string& arm_prefix)
    : node_(node), arm_prefix_(arm_prefix) {
    // 参数已在 JakaControlNode 中声明，这里直接读取
    // （避免重复声明导致 ParameterAlreadyDeclaredException）
    dt_ = node_->get_parameter("velocity_control.dt").as_double();
    linear_gain_ = node_->get_parameter("velocity_control.linear_gain").as_double();
    angular_gain_ = node_->get_parameter("velocity_control.angular_gain").as_double();
    max_linear_vel_ = node_->get_parameter("velocity_control.max_linear_vel").as_double();
    max_angular_vel_ = node_->get_parameter("velocity_control.max_angular_vel").as_double();
    joint_vel_limit_ = node_->get_parameter("velocity_control.joint_vel_limit").as_double();
    q_dot_min_ = node_->get_parameter("velocity_control.q_dot_min").as_double();
    max_delta_q_ = node_->get_parameter("velocity_control.max_delta_q").as_double();
    lambda_min_ = node_->get_parameter("velocity_control.lambda_min").as_double();
    position_deadzone_ = node_->get_parameter("velocity_control.position_deadzone").as_double();
    orientation_deadzone_ = node_->get_parameter("velocity_control.orientation_deadzone").as_double();
    joint_pos_min_ = node_->get_parameter("velocity_control.joint_pos_min").as_double_array();
    joint_pos_max_ = node_->get_parameter("velocity_control.joint_pos_max").as_double_array();
    
    RCLCPP_INFO(node_->get_logger(), "[VelCtrl] Parameters loaded: dt=%.3f, vel_limit=%.2f, q_dot_min=%.1e",
        dt_, joint_vel_limit_, q_dot_min_);
}

VelocityServoController::~VelocityServoController() {}

void VelocityServoController::setJointLimits(const std::vector<double>& pos_min, const std::vector<double>& pos_max) {
    if (pos_min.size() != 7 || pos_max.size() != 7) {
        RCLCPP_ERROR(node_->get_logger(), "[VelCtrl] Invalid joint limits size (expected 7)");
        return;
    }
    joint_pos_min_ = pos_min;
    joint_pos_max_ = pos_max;
    RCLCPP_INFO(node_->get_logger(), "[VelCtrl] Joint limits configured for %s arm", arm_prefix_.c_str());
}

bool VelocityServoController::initialize(const std::string& urdf_path, const std::string& base_link, const std::string& tip_link) {
    // 🔥 读取URDF文件内容（TracIK需要XML字符串，不是文件路径）
    std::ifstream urdf_file(urdf_path);
    if (!urdf_file.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "[VelCtrl] Failed to open URDF file: %s", urdf_path.c_str());
        return false;
    }
    std::string urdf_xml((std::istreambuf_iterator<char>(urdf_file)), std::istreambuf_iterator<char>());
    urdf_file.close();
    
    if (urdf_xml.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "[VelCtrl] URDF file is empty: %s", urdf_path.c_str());
        return false;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromFile(urdf_path, tree)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to construct KDL tree from URDF file: %s", urdf_path.c_str());
        return false;
    }

    if (!tree.getChain(base_link, tip_link, chain_)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get KDL chain from %s to %s", base_link.c_str(), tip_link.c_str());
        return false;
    }

    jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);
    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
    
    // 🔥 初始化TracIK求解器（支持seed state，避免多解跳变）
    // 注意：TracIK构造函数需要URDF XML字符串，不是文件路径
    tracik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(
        base_link, tip_link, urdf_xml, 
        0.01,   // timeout: 10ms求解时间（放宽以提高成功率）
        1e-4,   // epsilon: 0.1mm位置误差容限（放宽以减少震荡）
        TRAC_IK::Distance  // 同时优化位置和姿态
    );
    
    if (!tracik_solver_->getKDLChain(chain_)) {
        RCLCPP_ERROR(node_->get_logger(), "[VelCtrl] TracIK failed to get KDL chain");
        return false;
    }
    
    current_q_.resize(chain_.getNrOfJoints());
    integrated_q_.resize(chain_.getNrOfJoints(), 0.0);
    
    // 预分配Jacobian对象，避免每次计算时重新分配内存
    jac_ = KDL::Jacobian(chain_.getNrOfJoints());
    
    RCLCPP_INFO(node_->get_logger(), "[VelCtrl] Initialized with %d joints (using TracIK)", chain_.getNrOfJoints());
    initialized_ = true;
    return true;
}

void VelocityServoController::updateRobotState(const std::vector<double>& current_joints) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!initialized_) return;
    
    if (current_joints.size() != chain_.getNrOfJoints()) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
            "Joint count mismatch: expected %d, got %zu", chain_.getNrOfJoints(), current_joints.size());
        return;
    }

    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
        current_q_(i) = current_joints[i];
    }
    
    // If this is the first update or reset, sync integrated state with real robot
    if (first_update_) {
        integrated_q_ = current_joints;
        first_update_ = false;
        has_initialized_command_ = true;  // 🔧 标记已有有效指令，立即进入静止状态
        RCLCPP_DEBUG(node_->get_logger(), "[VelCtrl] Initialized integrated state from robot (ready for hold)");
    }
}

void VelocityServoController::setTargetPose(const geometry_msgs::msg::PoseStamped& target_pose) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    target_pose_ = target_pose;
    has_target_ = true;
}

bool VelocityServoController::computeNextCommand(std::vector<double>& next_joints) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!initialized_) return false;
    
    // 🔧 关键修复：当没有VR目标时，返回上次的积分状态（保持静止）
    // 这样可以避免用真实关节位置（带抖动）发送指令，防止误差累积
    if (!has_target_) {
        if (has_initialized_command_) {
            next_joints = integrated_q_;
            return true;  // 返回上次的固定指令，保持静止
        }
        return false;  // 还没有初始化过指令
    }

    // 1️⃣ 当前末端位姿（用真实关节状态FK）
    // CRITICAL: Use real robot state (current_q_) for FK and Jacobian, not integrated state
    KDL::Frame current_pose;
    fk_solver_->JntToCart(current_q_, current_pose);

    KDL::Frame target_kdl = poseToKDL(target_pose_.pose);
    KDL::Twist twist = KDL::diff(current_pose, target_kdl);

    // 2️⃣ 位置/姿态误差
    double position_error = twist.vel.Norm();
    double orientation_error = twist.rot.Norm();

    if (position_error < position_deadzone_ && orientation_error < orientation_deadzone_) {
        next_joints = integrated_q_;  // 静止保持
        return true;
    }

    unsigned int n_joints = chain_.getNrOfJoints();
    
    // 2️⃣ 积分状态漂移修正：避免长期偏离真实关节（分关节阈值）
    for (unsigned int i = 0; i < n_joints; ++i) {
        // 末端wrist关节(4-6)更敏感，用更小阈值；大臂关节(0-3)用较大阈值
        double drift_thresh = (i >= 4) ? 0.02 : 0.05;  // wrist: 1.15°, arm: 2.86°
        if (std::abs(integrated_q_[i] - current_q_(i)) > drift_thresh) {
            integrated_q_[i] = current_q_(i);
            RCLCPP_DEBUG(node_->get_logger(), "[%s] Joint %d drift corrected", arm_prefix_.c_str(), i);
        }
    }

    // 3️⃣ 🔥 误差缩放与目标插值（借鉴旧版，防止冲过头导致震荡）
    // 根据误差大小动态调整接近速度
    double linear_scale = std::min(1.0, max_linear_vel_ * dt_ / (position_error + 1e-6));
    double angular_scale = std::min(1.0, max_angular_vel_ * dt_ / (orientation_error + 1e-6));
    double approach_factor = std::min(linear_scale, angular_scale);
    
    // 对目标位姿进行插值，渐进接近
    KDL::Frame interpolated_target;
    interpolated_target.p = current_pose.p + (target_kdl.p - current_pose.p) * approach_factor;
    
    // 姿态插值（四元数球面线性插值的简化版）
    KDL::Rotation rot_diff = current_pose.M.Inverse() * target_kdl.M;
    KDL::Vector rot_axis;
    double rot_angle = rot_diff.GetRotAngle(rot_axis);
    
    // 🔧 关键保护：极小角度时保持当前姿态，避免轴不稳定导致wrist抖动
    if (rot_angle > 1e-3) {  // >0.057°才插值
        interpolated_target.M = current_pose.M * KDL::Rotation::Rot(rot_axis, rot_angle * approach_factor);
    } else {
        interpolated_target.M = current_pose.M;  // 姿态误差极小，保持不动
    }

    // 4️⃣ 使用TracIK求解插值后的目标（利用seed state避免多解跳变）
    KDL::JntArray seed_state(n_joints);
    for (unsigned int i = 0; i < n_joints; ++i) {
        seed_state(i) = integrated_q_[i];
    }
    
    KDL::JntArray result_joints(n_joints);
    int rc = tracik_solver_->CartToJnt(seed_state, interpolated_target, result_joints);
    
    if (rc < 0) {
        // 🔄 IK失败，保持当前积分状态（不重置，避免卡顿）
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "[%s] TracIK failed (code=%d), maintaining current position", arm_prefix_.c_str(), rc);
        next_joints = integrated_q_;
        return true;
    }
    
    // 5️⃣ 计算增量并限制步长（简化限速策略，避免过度平滑）
    next_joints.resize(n_joints);
    double max_joint_delta = 0.0;
    
    for (unsigned int i = 0; i < n_joints; ++i) {
        double delta_q = result_joints(i) - integrated_q_[i];
        max_joint_delta = std::max(max_joint_delta, std::abs(delta_q));
        
        // 单步增量保护（防止跳变）
        delta_q = std::clamp(delta_q, -max_delta_q_, max_delta_q_);
        
        integrated_q_[i] += delta_q;
        
        // 🔒 硬限位保护
        if (i < joint_pos_min_.size() && i < joint_pos_max_.size()) {
            integrated_q_[i] = std::clamp(integrated_q_[i], joint_pos_min_[i], joint_pos_max_[i]);
        }
        
        next_joints[i] = integrated_q_[i];
    }
    
    // 🔍 调试：检测大幅跳变
    if (max_joint_delta > 0.1) {
        RCLCPP_WARN(node_->get_logger(),
            "[%s] Large TracIK delta: %.3f rad (%.1f deg)", 
            arm_prefix_.c_str(), max_joint_delta, max_joint_delta * 57.3);
    }
    
    has_initialized_command_ = true;  // 标记已有有效的静止指令

    return true;
}

void VelocityServoController::reset() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    has_target_ = false;
    first_update_ = true;
    has_initialized_command_ = false;  // 重置时清除指令初始化标志
    // Note: integrated_q_ will be re-initialized from robot state on next updateRobotState()
    // This ensures smooth restart without jumps
}

KDL::Frame VelocityServoController::poseToKDL(const geometry_msgs::msg::Pose& pose) {
    return KDL::Frame(
        KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        KDL::Vector(pose.position.x, pose.position.y, pose.position.z)
    );
}

Eigen::MatrixXd VelocityServoController::dampedPseudoInverse(const Eigen::MatrixXd& J, double lambda) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::VectorXd singularValuesInv = singularValues;
    
    for (int i = 0; i < singularValues.size(); ++i) {
        singularValuesInv(i) = singularValues(i) / (singularValues(i) * singularValues(i) + lambda * lambda);
    }
    
    return svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
}

} // namespace qyh_jaka_control
