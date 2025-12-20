#include "qyh_jaka_control/velocity_servo_controller.hpp"
#include <urdf/model.h>
#include <fstream>
#include <iostream>
#include <algorithm>

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
    double default_vel_limit = node_->get_parameter("velocity_control.joint_vel_limit").as_double();
    joint_vel_limit_.resize(7, default_vel_limit);  // 初始化为7个相同值
    q_dot_min_ = node_->get_parameter("velocity_control.q_dot_min").as_double();
    max_delta_q_ = node_->get_parameter("velocity_control.max_delta_q").as_double();
    
    // 🎯 工业级参数建议（JAKA实测优化值）
    // joint_vel_limit: 0.6 rad/s（避免速度报警）
    // q_dot_min: 0.005（死区放大，减少微抖）
    // servo_kp: 0.4（关键：降低增益避免震荡）
    // max_delta_q: 0.02（单步保护，已优化）
    lambda_min_ = node_->get_parameter("velocity_control.lambda_min").as_double();
    position_deadzone_ = node_->get_parameter("velocity_control.position_deadzone").as_double();
    orientation_deadzone_ = node_->get_parameter("velocity_control.orientation_deadzone").as_double();
    joint_pos_min_ = node_->get_parameter("velocity_control.joint_pos_min").as_double_array();
    joint_pos_max_ = node_->get_parameter("velocity_control.joint_pos_max").as_double_array();
    
    // 读取目标更新周期，并自动调整跳变阈值
    target_update_dt_ = node_->get_parameter("velocity_control.target_update_dt").as_double();
    
    // 自动计算跳变阈值：允许在更新周期内以最大速度运动，并给予3倍裕度
    // 这样可以防止快速运动时触发IK不连续保护
    double max_jump = default_vel_limit * target_update_dt_;
    single_joint_jump_thresh_ = max_jump * 3.0; 
    total_jump_thresh_ = single_joint_jump_thresh_ * 3.0;
    
    RCLCPP_INFO(node_->get_logger(), "[VelCtrl] Parameters loaded: dt=%.3f, update_dt=%.3f, jump_thresh=%.3f",
        dt_, target_update_dt_, single_joint_jump_thresh_);
}

VelocityServoController::~VelocityServoController() {}

void VelocityServoController::setJointLimits(const std::vector<double>& pos_min, const std::vector<double>& pos_max, const std::vector<double>& vel_limit) {
    if (pos_min.size() != 7 || pos_max.size() != 7 || vel_limit.size() != 7) {
        RCLCPP_ERROR(node_->get_logger(), "[VelCtrl] Invalid joint limits size (expected 7)");
        return;
    }
    joint_pos_min_ = pos_min;
    joint_pos_max_ = pos_max;
    joint_vel_limit_ = vel_limit;
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
    joint_target_.resize(chain_.getNrOfJoints(), 0.0);
    joint_target_ref_.resize(chain_.getNrOfJoints(), 0.0);
    integrated_q_.resize(chain_.getNrOfJoints(), 0.0);
    sat_count_.resize(chain_.getNrOfJoints(), 0);
    
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
    
    // If this is the first update or reset, sync both targets with real robot
    if (first_update_) {
        joint_target_ = current_joints;
        joint_target_ref_ = current_joints;
        integrated_q_ = current_joints;
        first_update_ = false;
        has_initialized_command_ = true;
        RCLCPP_DEBUG(node_->get_logger(), "[VelCtrl] Initialized joint_target & integrated_q from robot");
    }
}

void VelocityServoController::setJointTarget(const std::vector<double>& joint_target) {
    setJointTargetRef(joint_target);
}

void VelocityServoController::setJointTargetRef(const std::vector<double>& ik_joints) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (ik_joints.size() != chain_.getNrOfJoints()) {
        RCLCPP_ERROR(node_->get_logger(), "[VelCtrl] Invalid joint target size: %zu", ik_joints.size());
        return;
    }
    
    // 直接更新参考目标（Target Governor 会处理平滑）
    for (size_t i = 0; i < ik_joints.size(); ++i) {
        joint_target_ref_[i] = ik_joints[i];
    }
    has_target_ = true;
}

bool VelocityServoController::getIntegratedQ(std::vector<double>& q_out) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!initialized_ || !has_initialized_command_) return false;
    
    q_out = integrated_q_;
    return true;
}

bool VelocityServoController::checkIKContinuity(const std::vector<double>& seed, const std::vector<double>& result) {
    if (seed.size() != result.size()) return false;

    double sum_jump = 0.0;
    for (size_t i = 0; i < seed.size(); ++i) {
        double d = std::abs(result[i] - seed[i]);
        if (d > single_joint_jump_thresh_) {
            return false;   // ❌ 单关节跳解
        }
        sum_jump += d;
    }
    return sum_jump < total_jump_thresh_;
}

void VelocityServoController::updateTargetGovernor() {
    unsigned int n_joints = chain_.getNrOfJoints();
    for (size_t i = 0; i < n_joints; ++i) {
        // 限制目标变化率 (Target Governor)
        // 这里的 dt_ 是 Servo 周期 (0.008s)
        // 允许的最大步长 = 0.8 * max_vel * dt (提高到 0.8 以允许更快跟随)
        double max_step = 0.8 * joint_vel_limit_[i] * dt_;
        
        double diff = joint_target_ref_[i] - joint_target_[i];
        joint_target_[i] += std::clamp(diff, -max_step, max_step);
    }
}





void VelocityServoController::holdCurrent() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // IK失败时冻结在当前位置
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
        joint_target_ref_[i] = current_q_(i);
        // joint_target_ 也会在 Governor 中慢慢追过来
    }
    RCLCPP_DEBUG(node_->get_logger(), "[VelCtrl] Holding current position");
    has_target_ = true;
}

// ============================================================
// 🧠 IK求解接口（暴露给VR回调使用）
// 功能：位姿 + seed → 关节角度
// ============================================================
bool VelocityServoController::solveIK(
    const geometry_msgs::msg::Pose& target_pose,
    const std::vector<double>& seed_joints,
    std::vector<double>& result_joints)
{
    // ⚠️ IK求解不加锁，避免阻塞 Servo 线程 (125Hz)
    // TracIK 和 KDL 是线程安全的（只要不修改成员变量）
    // std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!initialized_) {
        RCLCPP_ERROR(node_->get_logger(), "[VelCtrl] Controller not initialized");
        return false;
    }
    
    unsigned int n_joints = chain_.getNrOfJoints();
    
    if (seed_joints.size() != n_joints) {
        RCLCPP_ERROR(node_->get_logger(), "[VelCtrl] Invalid seed size: %zu", seed_joints.size());
        return false;
    }
    
    // 转换为KDL格式
    KDL::Frame target_kdl = poseToKDL(target_pose);
    
    KDL::JntArray seed_state(n_joints);
    for (unsigned int i = 0; i < n_joints; ++i) {
        seed_state(i) = seed_joints[i];
    }
    
    // TracIK求解
    KDL::JntArray ik_result(n_joints);
    int rc = tracik_solver_->CartToJnt(seed_state, target_kdl, ik_result);
    
    if (rc < 0) {
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "[%s] IK failed (code=%d)", arm_prefix_.c_str(), rc);
        return false;
    }
    
    // 输出结果（带限位保护）
    result_joints.resize(n_joints);
    for (unsigned int i = 0; i < n_joints; ++i) {
        result_joints[i] = ik_result(i);
        
        // 硬限位保护
        if (i < joint_pos_min_.size() && i < joint_pos_max_.size()) {
            result_joints[i] = std::clamp(result_joints[i], joint_pos_min_[i], joint_pos_max_[i]);
        }
    }
    
    return true;
}

// ============================================================
// ⚙️ Servo层（高频 125Hz，纯追踪）
// 功能：追踪joint_target_（来自VR回调的IK结果）
// ============================================================
bool VelocityServoController::computeNextCommand(std::vector<double>& next_joints) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!initialized_) return false;
    
    // Hold模式：无关节目标时保持当前位置
    if (!has_target_) {
        if (has_initialized_command_) {
            next_joints = integrated_q_;
            return true;
        }
        return false;
    }
    
    // ⭐ 核心：Target Governor (10Hz logic executed in servo loop)
    updateTargetGovernor();

    unsigned int n_joints = chain_.getNrOfJoints();
    next_joints.resize(n_joints);
    
    for (unsigned int i = 0; i < n_joints; ++i) {
        // Soft sync integrated_q_ to real joint (anti drift)
        // 防止积分器长期漂移，将其限制在真实位置的邻域内
        double sync_thresh = (i >= 4) ? 0.02 : 0.05;
        integrated_q_[i] = std::clamp(
            integrated_q_[i],
            current_q_(i) - sync_thresh,
            current_q_(i) + sync_thresh
        );

        // Hybrid Velocity Servo: 追 integrated_q -> target
        double error = joint_target_[i] - integrated_q_[i];
        
        // 速度生成 (虚拟一阶系统)
        double qdot = error / follow_time_;
        qdot = std::clamp(qdot, -joint_vel_limit_[i], joint_vel_limit_[i]);
        
        // 微小速度死区
        if (std::abs(qdot) < q_dot_min_) {
            qdot = 0.0;
        }
        
        // 位置积分
        double delta = qdot * dt_;
        delta = std::clamp(delta, -max_delta_q_, max_delta_q_);
        
        double cmd = integrated_q_[i] + delta;
        
        // 🔪 刀3：Servo层最终安全钳（Final Safety Clamp）
        // 相对真实位置再钳一次（防 Following Error）
        double real_delta = cmd - current_q_(i);
        real_delta = std::clamp(real_delta, -max_delta_q_, max_delta_q_);
        
        // 反算最终指令
        next_joints[i] = current_q_(i) + real_delta;
        
        // 更新积分器（Anti-windup，保持同步）
        integrated_q_[i] = next_joints[i];
        
        // 硬限位保护
        if (i < joint_pos_min_.size() && i < joint_pos_max_.size()) {
            integrated_q_[i] = std::clamp(integrated_q_[i], joint_pos_min_[i], joint_pos_max_[i]);
            next_joints[i] = integrated_q_[i];
        }
    }
    
    has_initialized_command_ = true;
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
