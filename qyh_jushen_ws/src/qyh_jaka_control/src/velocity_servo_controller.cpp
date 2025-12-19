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
    joint_target_.resize(chain_.getNrOfJoints(), 0.0);
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
        integrated_q_ = current_joints;
        first_update_ = false;
        has_initialized_command_ = true;
        RCLCPP_DEBUG(node_->get_logger(), "[VelCtrl] Initialized joint_target & integrated_q from robot");
    }
}

void VelocityServoController::setJointTarget(const std::vector<double>& joint_target) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (joint_target.size() != chain_.getNrOfJoints()) {
        RCLCPP_ERROR(node_->get_logger(), "[VelCtrl] Invalid joint target size: %zu", joint_target.size());
        return;
    }
    
    // � 刀1：IK目标平滑（Target Smoothing）
    // 限制目标值的变化率，避免IK解突变直接传给Servo
    // 即使IK解是合法的，如果变化太快，Servo也追不上，会导致饱和
    const double max_target_step = 0.02; // rad, 允许IK目标每帧最大跳变 (30Hz -> 0.6 rad/s)
    
    for (size_t i = 0; i < joint_target.size(); ++i) {
        // 如果是第一次或者没有目标，就以当前位置为基准
        double prev_target = has_target_ ? joint_target_[i] : current_q_(i);
        
        joint_target_[i] = std::clamp(
            joint_target[i],
            prev_target - max_target_step,
            prev_target + max_target_step
        );
    }
    
    has_target_ = true;
}

void VelocityServoController::holdCurrent() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // IK失败时冻结在当前位置，防止Servo层追旧目标
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
        joint_target_[i] = current_q_(i);
    }
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
    std::lock_guard<std::mutex> lock(state_mutex_);
    
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
    
    unsigned int n_joints = chain_.getNrOfJoints();
    
    // ✅ 漂移修正：软限制积分器与真实位置的偏差
    // 改为Clamp模式，避免Hard Reset导致的指令突变（速度冲击）
    for (unsigned int i = 0; i < n_joints; ++i) {
        double drift_thresh = (i >= 4) ? 0.02 : 0.05;  // 腕部更严格
        
        // 将积分器限制在当前位置的邻域内
        // 这样即使跑偏，也只是"拖着走"，而不会"跳回"
        if (integrated_q_[i] > current_q_(i) + drift_thresh) {
            integrated_q_[i] = current_q_(i) + drift_thresh;
        } else if (integrated_q_[i] < current_q_(i) - drift_thresh) {
            integrated_q_[i] = current_q_(i) - drift_thresh;
        }
    }
    
    // ⚙️ Servo层：简单P控制（连续、可预测、JAKA喜欢）
    next_joints.resize(n_joints);
    
    for (unsigned int i = 0; i < n_joints; ++i) {
        // P控制：计算位置误差
        double error = joint_target_[i] - current_q_(i);
        
        // 速度指令（比例控制）
        double cmd_vel = servo_kp_ * error;
        
        // 速度限制
        cmd_vel = std::clamp(cmd_vel, -joint_vel_limit_, joint_vel_limit_);
        
        // 微小速度死区（防止目标附近抖动）
        if (std::abs(cmd_vel) < q_dot_min_) {
            cmd_vel = 0.0;
        }
        
        // 位置积分
        double delta_q = cmd_vel * dt_;
        
        // 🔪 刀3：Servo层最终安全钳（Final Safety Clamp）
        // 无论积分器怎么算，发给机器人的指令绝对不能超过当前位置的 max_delta_q_
        // 这是避免 "Following Error" 的最后一道防线
        
        // 预期的新位置（基于积分器）
        double expected_q = integrated_q_[i] + delta_q;
        
        // 计算相对于真实位置的偏差
        double real_delta = expected_q - current_q_(i);
        
        // 强制钳位到安全范围 (max_delta_q_ = 0.02 rad)
        real_delta = std::clamp(real_delta, -max_delta_q_, max_delta_q_);
        
        // 🔪 刀4：饱和计数器（Saturation Counter）
        // 如果指令长期贴着 max_delta_q_ 走，说明 Servo 已经跟不上了
        // JAKA 会认为这是"外部轨迹异常"，所以我们需要自己先熔断
        if (std::abs(real_delta) > 0.9 * max_delta_q_) {
            sat_count_[i]++;
        } else {
            sat_count_[i] = 0;
        }
        
        if (sat_count_[i] > 15) { // ~120ms @ 125Hz
             // 熔断保护：冻结该关节
             real_delta = 0.0;
        }
        
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
