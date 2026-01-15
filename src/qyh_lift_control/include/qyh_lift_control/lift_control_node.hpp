/*
 * 升降电机控制节点
 * 通过 Modbus TCP 与 PLC 通信
 */

#ifndef QYH_LIFT_CONTROL__LIFT_CONTROL_NODE_HPP_
#define QYH_LIFT_CONTROL__LIFT_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <mutex>

#include "qyh_lift_msgs/msg/lift_state.hpp"
#include "qyh_lift_msgs/srv/lift_control.hpp"
#include "modbus/modbus.hpp"

namespace qyh_lift_control
{

/**
 * Modbus 地址定义
 * 线圈地址从 1000 开始 (PLC地址 01001)
 * 寄存器地址从 1000 开始 (PLC地址 41001)
 */
struct ModbusAddress
{
  // 线圈地址 (Coils) - 基地址 1000
  static constexpr int COIL_BASE = 1000;
  static constexpr int COIL_ENABLE = 0;           // 使能
  static constexpr int COIL_SET_ORIGIN = 1;       // 设置原点 (不使用)
  static constexpr int COIL_MOVE_UP = 2;          // 正转(上升)
  static constexpr int COIL_MOVE_DOWN = 3;        // 反转(下降)
  static constexpr int COIL_RESET = 4;            // 复位
  static constexpr int COIL_ALARM = 5;            // 报警 (PLC通知)
  static constexpr int COIL_GO_STOP = 6;          // 绝对位置GO_STOP
  static constexpr int COIL_GO_POSITION = 10;     // 绝对位置GO
  static constexpr int COIL_POSITION_REACHED = 20; // 绝对位置到达 (PLC通知)

  // 寄存器地址 (Holding Registers) - 基地址 1000
  static constexpr int REG_BASE = 1000;
  static constexpr int REG_TARGET_POSITION = 0;   // 目标绝对位置
  static constexpr int REG_SPEED = 20;            // 绝对速度
  static constexpr int REG_CURRENT_POSITION = 40; // 当前绝对位置

  // 增加一个电磁铁的开关线圈，地址是64518
  static constexpr int COIL_ELECTROMAGNET = 64518 - 1000; // 电磁铁控制线圈
};

class LiftControlNode : public rclcpp::Node
{
public:
  explicit LiftControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LiftControlNode();

private:
  // Modbus 连接
  bool connect_modbus();
  void disconnect_modbus();

  // 定时器回调 - 读取状态并发布
  void timer_callback();

  // 重连定时器回调
  void reconnect_callback();

  // 读取状态
  bool read_lift_state();

  // 线圈操作
  bool write_coil(int offset, bool value);
  bool read_coil(int offset);

  // 寄存器操作
  bool write_register(int offset, uint16_t value);
  bool write_register_float(int offset, float value);
  uint16_t read_register(int offset);
  float read_register_float(int offset);

  // 控制功能
  bool enable_lift(bool enable);
  bool set_speed(float speed);
  bool go_to_position(float position);
  bool manual_move(bool up, bool hold);
  bool reset_alarm();
  bool stop_move();
  bool set_electromagnet(bool enable);
  // 服务回调
  void handle_control(
    const qyh_lift_msgs::srv::LiftControl::Request::SharedPtr request,
    qyh_lift_msgs::srv::LiftControl::Response::SharedPtr response);

  // 参数
  std::string plc_ip_;
  int plc_port_;
  double publish_rate_;

  // Modbus 连接
  std::unique_ptr<modbus::ModbusTCP> modbus_ctx_;
  std::mutex modbus_mutex_;
  bool is_connected_;

  // 状态
  qyh_lift_msgs::msg::LiftState current_state_;
  bool is_enabled_;

  // ROS2 接口
  rclcpp::Publisher<qyh_lift_msgs::msg::LiftState>::SharedPtr state_pub_;
  rclcpp::Service<qyh_lift_msgs::srv::LiftControl>::SharedPtr control_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;  // 重连定时器
};

}  // namespace qyh_lift_control

#endif  // QYH_LIFT_CONTROL__LIFT_CONTROL_NODE_HPP_
