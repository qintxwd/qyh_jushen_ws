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
    
    current_q_.resize(chain_.getNrOfJoints());
    integrated_q_.resize(chain_.getNrOfJoints(), 0.0);
    
    // 预分配Jacobian对象，避免每次计算时重新分配内存
    jac_ = KDL::Jacobian(chain_.getNrOfJoints());
    
    RCLCPP_INFO(node_->get_logger(), "[VelCtrl] Initialized with %d joints", chain_.getNrOfJoints());
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

    // 3️⃣ 误差缩放，防止远目标积分过大
    double linear_scale = std::min(1.0, max_linear_vel_ / (position_error + 1e-6));
    double angular_scale = std::min(1.0, max_angular_vel_ / (orientation_error + 1e-6));

    twist.vel = twist.vel * linear_gain_ * linear_scale;
    twist.rot = twist.rot * angular_gain_ * angular_scale;

    // 4️⃣ 计算雅可比（CRITICAL: 使用真实关节状态）
    // 使用预分配的jac_对象，避免每次重新分配内存
    jac_solver_->JntToJac(current_q_, jac_);
    Eigen::MatrixXd J = jac_.data;

    // 5️⃣ 关节速度计算（动态阻尼伪逆）
    Eigen::VectorXd v_cart(6);
    v_cart << twist.vel.x(), twist.vel.y(), twist.vel.z(),
              twist.rot.x(), twist.rot.y(), twist.rot.z();

    // 动态阻尼：lambda随最大奇异值变化，防止奇异点时关节速度爆炸
    // 添加下限保护，防止s.maxCoeff()太小导致lambda接近0
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd s = svd.singularValues();
    double lambda = std::max(0.01 * s.maxCoeff(), lambda_min_);
    Eigen::VectorXd s_inv = s.array() / (s.array().square() + lambda * lambda);
    Eigen::MatrixXd J_pinv = svd.matrixV() * s_inv.asDiagonal() * svd.matrixU().transpose();

    Eigen::VectorXd q_dot = J_pinv * v_cart;

    // 6️⃣ 积分与安全限制
    next_joints.resize(chain_.getNrOfJoints());
    
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
        // 限制关节速度
        if (q_dot(i) > joint_vel_limit_) q_dot(i) = joint_vel_limit_;
        if (q_dot(i) < -joint_vel_limit_) q_dot(i) = -joint_vel_limit_;

        // 微小速度置零（防止死区边界抖动导致漂移）
        if (std::abs(q_dot(i)) < q_dot_min_) q_dot(i) = 0.0;

        // 积分
        double delta_q = q_dot(i) * dt_;

        // 积分步长饱和（双重安全保护）
        if (delta_q > max_delta_q_) delta_q = max_delta_q_;
        if (delta_q < -max_delta_q_) delta_q = -max_delta_q_;

        integrated_q_[i] += delta_q;
        
        // 🔒 关键安全：限制积分结果在关节物理范围内，防止长时间运行漂移
        if (i < joint_pos_min_.size() && i < joint_pos_max_.size()) {
            if (integrated_q_[i] > joint_pos_max_[i]) {
                integrated_q_[i] = joint_pos_max_[i];
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                    "[VelCtrl] Joint %d exceeded max limit, clamped to %.3f", i, joint_pos_max_[i]);
            }
            if (integrated_q_[i] < joint_pos_min_[i]) {
                integrated_q_[i] = joint_pos_min_[i];
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                    "[VelCtrl] Joint %d exceeded min limit, clamped to %.3f", i, joint_pos_min_[i]);
            }
        }
        
        next_joints[i] = integrated_q_[i];
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
