#ifndef QYH_HEAD_MOTOR_CONTROL__HEAD_MOTOR_NODE_HPP_
#define QYH_HEAD_MOTOR_CONTROL__HEAD_MOTOR_NODE_HPP_

#include <memory>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "qyh_head_motor_control/bus_servo_protocol.hpp"

namespace qyh_head_motor_control
{

/**
 * @brief 头部电机控制节点
 * 
 * 功能:
 * - 设置电机ID
 * - 发布电机位置 (JointState)
 * - 控制电机位置 [0-1000]
 * - 控制电机位置归一化 [-1, 1]
 * 
 * 话题:
 *   订阅:
 *     - ~/cmd_position (std_msgs/Float64MultiArray): 归一化位置命令 [-1, 1]
 *     - ~/cmd_position_raw (std_msgs/Int32MultiArray): 原始位置命令 [0-1000]
 *   发布:
 *     - ~/joint_states (sensor_msgs/JointState): 关节状态
 * 
 * 服务:
 *     - ~/enable_torque (std_srvs/SetBool): 使能/失能扭矩
 *     - ~/set_motor_id (qyh_head_motor_control/SetMotorId): 设置电机ID
 */
class HeadMotorNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     */
    explicit HeadMotorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    /**
     * @brief 析构函数
     */
    ~HeadMotorNode() override;

private:
    /**
     * @brief 初始化参数
     */
    void initParameters();
    
    /**
     * @brief 初始化通信
     */
    bool initCommunication();
    
    /**
     * @brief 初始化话题和服务
     */
    void initTopicsAndServices();
    
    /**
     * @brief 定时器回调 - 读取并发布位置
     */
    void publishCallback();
    
    /**
     * @brief 归一化位置命令回调 [-1, 1]
     */
    void cmdPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    
    /**
     * @brief 原始位置命令回调 [0-1000]
     */
    void cmdPositionRawCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    
    /**
     * @brief 使能扭矩服务回调
     */
    void enableTorqueCallback(
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response);
    
    /**
     * @brief 归一化位置转换为原始位置
     * @param normalized 归一化位置 [-1, 1]
     * @param motor_index 电机索引
     * @return 原始位置 [0-1000]
     */
    uint16_t normalizedToRaw(double normalized, size_t motor_index) const;
    
    /**
     * @brief 弧度转换为原始位置
     * @param radian 弧度
     * @param motor_index 电机索引
     * @return 原始位置 [0-1000]
     */
    uint16_t radianToRaw(double radian, size_t motor_index) const;
    
    /**
     * @brief 原始位置转换为归一化位置
     * @param raw 原始位置 [0-1000]
     * @param motor_index 电机索引
     * @return 归一化位置 [-1, 1]
     */
    double rawToNormalized(uint16_t raw, size_t motor_index) const;
    
    /**
     * @brief 原始位置转换为弧度
     * @param raw 原始位置 [0-1000]
     * @param motor_index 电机索引
     * @return 弧度
     */
    double rawToRadian(uint16_t raw, size_t motor_index) const;

    // 串口通信
    std::unique_ptr<BusServoProtocol> protocol_;
    
    // 参数
    std::string serial_port_;
    int baudrate_;
    std::vector<int64_t> motor_ids_;
    std::vector<std::string> joint_names_;
    std::vector<double> position_min_rad_;
    std::vector<double> position_max_rad_;
    std::vector<int64_t> position_center_raw_;
    std::vector<int64_t> position_min_raw_;
    std::vector<int64_t> position_max_raw_;
    int publish_rate_;
    int move_duration_ms_;
    
    // 当前位置
    std::vector<uint16_t> current_positions_;
    
    // ROS 接口
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr cmd_position_raw_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_torque_srv_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace qyh_head_motor_control

#endif  // QYH_HEAD_MOTOR_CONTROL__HEAD_MOTOR_NODE_HPP_
