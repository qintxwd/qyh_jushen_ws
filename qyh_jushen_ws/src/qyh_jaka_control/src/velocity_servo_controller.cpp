#include "qyh_jaka_control/velocity_servo_controller.hpp"
#include <urdf/model.h>
#include <fstream>
#include <iostream>

namespace qyh_jaka_control {

VelocityServoController::VelocityServoController(rclcpp::Node::SharedPtr node, const std::string& arm_prefix)
    : node_(node), arm_prefix_(arm_prefix) {
    // Load parameters if needed
    node_->declare_parameter("velocity_control.linear_gain", 2.0);
    node_->declare_parameter("velocity_control.angular_gain", 1.0);
    node_->declare_parameter("velocity_control.max_linear_vel", 0.5);
    node_->declare_parameter("velocity_control.max_angular_vel", 1.0);
    
    linear_gain_ = node_->get_parameter("velocity_control.linear_gain").as_double();
    angular_gain_ = node_->get_parameter("velocity_control.angular_gain").as_double();
    max_linear_vel_ = node_->get_parameter("velocity_control.max_linear_vel").as_double();
    max_angular_vel_ = node_->get_parameter("velocity_control.max_angular_vel").as_double();
}

VelocityServoController::~VelocityServoController() {}

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
    
    RCLCPP_INFO(node_->get_logger(), "VelocityServoController initialized with %d joints", chain_.getNrOfJoints());
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
        RCLCPP_INFO(node_->get_logger(), "[VelCtrl] Initialized integrated state from robot");
    }
}

void VelocityServoController::setTargetPose(const geometry_msgs::msg::PoseStamped& target_pose) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    target_pose_ = target_pose;
    has_target_ = true;
}

bool VelocityServoController::computeNextCommand(std::vector<double>& next_joints) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!initialized_ || !has_target_) return false;

    // 1️⃣ 当前末端位姿（用真实关节状态FK）
    // CRITICAL: Use real robot state (current_q_) for FK and Jacobian, not integrated state
    KDL::Frame current_pose;
    fk_solver_->JntToCart(current_q_, current_pose);

    KDL::Frame target_kdl = poseToKDL(target_pose_.pose);
    KDL::Twist twist = KDL::diff(current_pose, target_kdl);

    // 2️⃣ 位置/姿态误差
    double position_error = twist.vel.Norm();
    double orientation_error = twist.rot.Norm();
    const double POSITION_DEADZONE = 0.001;   // 1mm
    const double ORIENTATION_DEADZONE = 0.017; // ~1 degree

    if (position_error < POSITION_DEADZONE && orientation_error < ORIENTATION_DEADZONE) {
        next_joints = integrated_q_;  // 静止保持
        return true;
    }

    // 3️⃣ 误差缩放，防止远目标积分过大
    double linear_scale = std::min(1.0, max_linear_vel_ / (position_error + 1e-6));
    double angular_scale = std::min(1.0, max_angular_vel_ / (orientation_error + 1e-6));

    twist.vel = twist.vel * linear_gain_ * linear_scale;
    twist.rot = twist.rot * angular_gain_ * angular_scale;

    // 4️⃣ 计算雅可比（CRITICAL: 使用真实关节状态）
    KDL::Jacobian jac(chain_.getNrOfJoints());
    jac_solver_->JntToJac(current_q_, jac);
    Eigen::MatrixXd J = jac.data;

    // 5️⃣ 关节速度计算（动态阻尼伪逆）
    Eigen::VectorXd v_cart(6);
    v_cart << twist.vel.x(), twist.vel.y(), twist.vel.z(),
              twist.rot.x(), twist.rot.y(), twist.rot.z();

    // 动态阻尼：lambda随最大奇异值变化，防止奇异点时关节速度爆炸
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd s = svd.singularValues();
    double lambda = 0.01 * s.maxCoeff();
    Eigen::VectorXd s_inv = s.array() / (s.array().square() + lambda * lambda);
    Eigen::MatrixXd J_pinv = svd.matrixV() * s_inv.asDiagonal() * svd.matrixU().transpose();

    Eigen::VectorXd q_dot = J_pinv * v_cart;

    // 6️⃣ 积分与安全限制
    next_joints.resize(chain_.getNrOfJoints());
    const double Q_DOT_MIN = 1e-4;  // 微小速度死区，防止积分漂移
    const double MAX_DELTA_Q = 0.02; // 最大单步积分（~1.1度），防止抖动或异常大步长
    
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
        // 限制关节速度
        if (q_dot(i) > joint_vel_limit_) q_dot(i) = joint_vel_limit_;
        if (q_dot(i) < -joint_vel_limit_) q_dot(i) = -joint_vel_limit_;

        // 微小速度置零（防止死区边界抖动导致漂移）
        if (std::abs(q_dot(i)) < Q_DOT_MIN) q_dot(i) = 0.0;

        // 积分
        double delta_q = q_dot(i) * dt_;

        // 积分步长饱和（双重安全保护）
        if (delta_q > MAX_DELTA_Q) delta_q = MAX_DELTA_Q;
        if (delta_q < -MAX_DELTA_Q) delta_q = -MAX_DELTA_Q;

        integrated_q_[i] += delta_q;
        next_joints[i] = integrated_q_[i];
    }

    return true;
}

void VelocityServoController::reset() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    has_target_ = false;
    first_update_ = true;
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
