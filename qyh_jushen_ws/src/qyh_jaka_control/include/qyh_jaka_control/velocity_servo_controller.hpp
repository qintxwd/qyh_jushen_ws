#ifndef VELOCITY_SERVO_CONTROLLER_HPP
#define VELOCITY_SERVO_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <trac_ik/trac_ik.hpp>
#include <vector>
#include <string>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/SVD>

namespace qyh_jaka_control {

class VelocityServoController {
public:
    VelocityServoController(rclcpp::Node::SharedPtr node, const std::string& arm_prefix);
    ~VelocityServoController();

    /**
     * @brief Initialize the controller with URDF and chain information
     * @param urdf_path Path to the URDF file
     * @param base_link Name of the base link
     * @param tip_link Name of the end-effector link
     * @return true if initialization successful
     */
    bool initialize(const std::string& urdf_path, const std::string& base_link, const std::string& tip_link);
    
    /**
     * @brief Update current robot state (from SDK callback)
     * @param current_joints Current joint positions in radians
     */
    void updateRobotState(const std::vector<double>& current_joints);
    
    /**
     * @brief Set joint target (from IK solver, called in VR callback)
     * @param joint_target Target joint positions in radians
     */
    void setJointTarget(const std::vector<double>& joint_target);

    /**
     * @brief Set joint target reference (slow IK reference)
     * @param ik_joints Target joint positions from IK
     */
    void setJointTargetRef(const std::vector<double>& ik_joints);

    /**
     * @brief Check if IK result is continuous with seed state
     * @param seed Seed joint positions
     * @param result IK result joint positions
     * @return true if continuous (safe)
     */
    bool checkIKContinuity(const std::vector<double>& seed, const std::vector<double>& result);
    
    /**
     * @brief Hold current position (used when IK fails)
     */
    void holdCurrent();
    
    /**
     * @brief Solve IK for target pose (exposes IK capability to caller)
     * @param target_pose Target pose in base frame
     * @param seed_joints Seed state for IK solver
     * @param result_joints Output joint positions
     * @return true if IK succeeded
     */
    bool solveIK(const geometry_msgs::msg::Pose& target_pose,
                 const std::vector<double>& seed_joints,
                 std::vector<double>& result_joints);
    
    /**
     * @brief Get current integrated joint positions (Servo's internal state)
     * @param q_out Output vector
     * @return true if initialized
     */
    bool getIntegratedQ(std::vector<double>& q_out);

    /**
     * @brief Calculate next joint command (called at 125Hz)
     * 
     * Features:
     * - Error scaling: prevents large integration steps for distant targets
     * - Dynamic damping: lambda adapts to singular values to avoid joint velocity explosion
     * - Velocity deadzone: prevents drift from micro-velocities near target
     * - Integration saturation: double protection on both velocity and step size
     * 
     * @param next_joints Output vector for next joint positions
     * @return true if valid command generated
     */
    bool computeNextCommand(std::vector<double>& next_joints);

    /**
     * @brief Reset controller state
     */
    void reset();
    
    /**
     * @brief Set joint position limits
     * @param pos_min Minimum joint positions (rad)
     * @param pos_max Maximum joint positions (rad)
     */
    void setJointLimits(const std::vector<double>& pos_min, const std::vector<double>& pos_max, const std::vector<double>& vel_limit);

private:
    rclcpp::Node::SharedPtr node_;
    std::string arm_prefix_;
    
    // KDL objects
    KDL::Chain chain_;
    std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<TRAC_IK::TRAC_IK> tracik_solver_;  // TracIK求解器（支持seed state）
    
    // State
    KDL::JntArray current_q_;
    KDL::Frame current_pose_kdl_;
    geometry_msgs::msg::PoseStamped target_pose_;
    bool has_target_ = false;
    bool initialized_ = false;
    bool first_update_ = true;
    
    // 🔥 两层结构状态量
    std::vector<double> joint_target_;     // Servo内部连续目标 (Governor output)
    std::vector<double> joint_target_ref_; // IK参考目标 (IK output, slow)
    std::vector<double> integrated_q_;     // Servo层输出：积分后的指令
    
    // 🔥 关键：target更新标志（只在新target到达时跑IK）
    std::atomic<bool> target_updated_{false};
    
    // Control parameters
    double dt_ = 0.008; // 125Hz
    double linear_gain_ = 2.0;
    double angular_gain_ = 1.0;
    double max_linear_vel_ = 0.5;
    double max_angular_vel_ = 1.0;
    double target_update_dt_ = 0.066; // 目标更新周期 (s)
    
    // Branch-Safe Hybrid Servo Parameters
    double single_joint_jump_thresh_ = 0.5;  // 放宽以适应15Hz更新
    double total_jump_thresh_ = 1.5;         // 放宽以适应15Hz更新
    double follow_time_ = 0.25;

    // 安全参数（可配置）
    double q_dot_min_ = 1e-4;        // 微小速度死区 (rad/s)
    double max_delta_q_ = 0.02;      // 最大积分步长 (rad)
    double lambda_min_ = 1e-4;       // 阻尼系数下限
    double position_deadzone_ = 0.001;    // 位置死区 (m)
    double orientation_deadzone_ = 0.017; // 姿态死区 (rad)
    
    // Servo层P控制增益
    double servo_kp_ = 5.0;          // 关节位置误差增益
    
    // 关节限位（防止积分漂移）
    std::vector<double> joint_pos_min_;
    std::vector<double> joint_pos_max_;
    std::vector<double> joint_vel_limit_;
    
    // 预分配Jacobian对象（性能优化）
    KDL::Jacobian jac_;
    
    std::mutex state_mutex_;
    bool has_initialized_command_ = false;  // 是否已有有效的静止指令（避免抖动累积）
    
    // 饱和计数器（防止长期贴着max_delta_q_走）
    std::vector<int> sat_count_;

    // Helper to convert Pose msg to KDL Frame
    KDL::Frame poseToKDL(const geometry_msgs::msg::Pose& pose);
    
    // Damped Pseudo Inverse
    Eigen::MatrixXd dampedPseudoInverse(const Eigen::MatrixXd& J, double lambda = 0.01);

    // Target Governor (10Hz logic executed in servo loop)
    void updateTargetGovernor();
};

} // namespace qyh_jaka_control

#endif // VELOCITY_SERVO_CONTROLLER_HPP
