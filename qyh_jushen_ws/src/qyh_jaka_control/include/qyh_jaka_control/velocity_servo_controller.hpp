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
     * @brief Set target pose (from VR)
     * @param target_pose Target pose in base frame
     */
    void setTargetPose(const geometry_msgs::msg::PoseStamped& target_pose);
    
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

private:
    rclcpp::Node::SharedPtr node_;
    std::string arm_prefix_;
    
    // KDL objects
    KDL::Chain chain_;
    std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    
    // State
    KDL::JntArray current_q_;
    KDL::Frame current_pose_kdl_;
    geometry_msgs::msg::PoseStamped target_pose_;
    bool has_target_ = false;
    bool initialized_ = false;
    bool first_update_ = true;
    
    // Integrated position (internal state for smooth output)
    std::vector<double> integrated_q_;
    
    // Control parameters
    double dt_ = 0.008; // 125Hz
    double linear_gain_ = 2.0;
    double angular_gain_ = 1.0;
    double max_linear_vel_ = 0.5;
    double max_angular_vel_ = 1.0;
    double joint_vel_limit_ = 1.5;
    
    std::mutex state_mutex_;
    
    // Helper to convert Pose msg to KDL Frame
    KDL::Frame poseToKDL(const geometry_msgs::msg::Pose& pose);
    
    // Damped Pseudo Inverse
    Eigen::MatrixXd dampedPseudoInverse(const Eigen::MatrixXd& J, double lambda = 0.01);
};

} // namespace qyh_jaka_control

#endif // VELOCITY_SERVO_CONTROLLER_HPP
