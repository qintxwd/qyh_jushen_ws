/*
 * 关机控制节点实现
 * 支持两种关机触发方式：
 * 1. 硬件按钮 - PLC设置M100=1，节点检测到后触发关机
 * 2. 软件命令 - 网页调用服务，节点写M100=1并触发关机
 */

#include "modbus/modbus.hpp"
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <std_srvs/srv/trigger.hpp>
#include "qyh_shutdown_msgs/msg/shutdown_state.hpp"

namespace qyh_shutdown
{

class QyhShutdownNode : public rclcpp::Node
{
public:
  explicit QyhShutdownNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~QyhShutdownNode();

private:
  // Modbus 连接
  bool connect_modbus();
  void disconnect_modbus();

  // 定时器回调 - 读取状态并发布
  void timer_callback();

  // 重连定时器回调
  void reconnect_callback();

  // 读取PLC状态
  bool read_plc_state();

  // 线圈操作
  bool write_coil(int offset, bool value);

  // 控制功能
  bool request_shutdown(uint8_t source);  // 请求系统关机，source: 1=硬件, 2=软件
  void execute_system_shutdown();         // 执行系统关机命令

  // 服务回调
  void handle_control(const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std_srvs::srv::Trigger::Response::SharedPtr response);

  // 参数
  std::string plc_ip_;
  int plc_port_;
  double publish_rate_;
  int shutdown_coil_address_;  // 关机线圈地址
  int shutdown_delay_seconds_; // 关机延迟秒数

  // Modbus 连接
  std::unique_ptr<modbus::ModbusTCP> modbus_ctx_;
  std::mutex modbus_mutex_;
  bool is_connected_;

  // 状态
  qyh_shutdown_msgs::msg::ShutdownState current_state_;
  std::atomic<bool> shutdown_in_progress_;
  std::atomic<int> countdown_seconds_;
  uint8_t trigger_source_;  // 0=无, 1=硬件, 2=软件

  // ROS2 接口
  rclcpp::Publisher<qyh_shutdown_msgs::msg::ShutdownState>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr control_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

QyhShutdownNode::QyhShutdownNode(const rclcpp::NodeOptions &options)
    : Node("qyh_shutdown_node", options)
    , modbus_ctx_(nullptr)
    , is_connected_(false)
    , shutdown_in_progress_(false)
    , countdown_seconds_(-1)
    , trigger_source_(0)
{
  // 声明参数
  this->declare_parameter<std::string>("plc_ip", "192.168.1.88");
  this->declare_parameter<int>("plc_port", 502);
  this->declare_parameter<double>("publish_rate", 10.0);
  this->declare_parameter<int>("shutdown_coil_address", 100);  // M100
  this->declare_parameter<int>("shutdown_delay_seconds", 3);   // 3秒延迟

  // 获取参数
  plc_ip_ = this->get_parameter("plc_ip").as_string();
  plc_port_ = this->get_parameter("plc_port").as_int();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  shutdown_coil_address_ = this->get_parameter("shutdown_coil_address").as_int();
  shutdown_delay_seconds_ = this->get_parameter("shutdown_delay_seconds").as_int();

  RCLCPP_INFO(this->get_logger(), "Starting qyh_shutdown_node");
  RCLCPP_INFO(this->get_logger(), "PLC IP: %s, Port: %d", plc_ip_.c_str(), plc_port_);
  RCLCPP_INFO(this->get_logger(), "Shutdown coil address: M%d", shutdown_coil_address_);
  RCLCPP_INFO(this->get_logger(), "Shutdown delay: %d seconds", shutdown_delay_seconds_);

  // 连接 Modbus TCP
  if (!connect_modbus())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to PLC");
  }

  // 创建发布者 - 使用自定义消息
  state_pub_ = this->create_publisher<qyh_shutdown_msgs::msg::ShutdownState>(
      "shutdown_state", 10);

  // 创建服务
  control_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "qyh_shutdown", 
      std::bind(&QyhShutdownNode::handle_control, this, 
                std::placeholders::_1, std::placeholders::_2));

  // 创建定时器 - 状态发布 (1Hz)
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&QyhShutdownNode::timer_callback, this));

  // 创建重连定时器 - 10秒间隔
  reconnect_timer_ = this->create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&QyhShutdownNode::reconnect_callback, this));

  RCLCPP_INFO(this->get_logger(), "QyhShutdown Control Node started");
}

QyhShutdownNode::~QyhShutdownNode()
{
  disconnect_modbus();
}

bool QyhShutdownNode::connect_modbus()
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  try
  {
    modbus_ctx_ = std::make_unique<modbus::ModbusTCP>(plc_ip_, plc_port_);
    modbus_ctx_->set_response_timeout(1, 0);  // 1秒超时
    modbus_ctx_->connect();

    RCLCPP_INFO(this->get_logger(), "Successfully connected to PLC at %s:%d", 
                plc_ip_.c_str(), plc_port_);
    is_connected_ = true;
    return true;
  }
  catch (const modbus::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Modbus connection failed: %s", e.what());
    modbus_ctx_.reset();
    is_connected_ = false;
    return false;
  }
}

void QyhShutdownNode::disconnect_modbus()
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (modbus_ctx_)
  {
    try
    {
      modbus_ctx_->close();
    }
    catch (...)
    {
      // Ignore disconnect errors
    }
    modbus_ctx_.reset();
  }
  is_connected_ = false;
}

void QyhShutdownNode::reconnect_callback()
{
  if (is_connected_)
  {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Attempting to reconnect to PLC...");
  connect_modbus();
}

void QyhShutdownNode::timer_callback()
{
  // 读取PLC状态
  if (is_connected_)
  {
    read_plc_state();
  }

  // 更新并发布状态
  current_state_.plc_connected = is_connected_;
  current_state_.shutdown_in_progress = shutdown_in_progress_.load();
  current_state_.trigger_source = trigger_source_;
  current_state_.countdown_seconds = countdown_seconds_.load();

  state_pub_->publish(current_state_);
}

bool QyhShutdownNode::read_plc_state()
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (!modbus_ctx_)
  {
    return false;
  }

  try
  {
    // 读取关机线圈状态 M100
    auto coils = modbus_ctx_->read_coils(shutdown_coil_address_, 1);
    bool m100_state = (coils[0] != 0);

    // 检测硬件按钮触发的关机请求
    // 只有当M100=1且当前不在关机中时，才认为是硬件触发
    if (m100_state && !shutdown_in_progress_.load() && trigger_source_ == 0)
    {
      RCLCPP_WARN(this->get_logger(), "Hardware shutdown button pressed! M100=1");
      request_shutdown(qyh_shutdown_msgs::msg::ShutdownState::SOURCE_HARDWARE);
    }
    
    return true;
  }
  catch (const modbus::Exception &e)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                          "Failed to read PLC state: %s", e.what());
    is_connected_ = false;
    return false;
  }
}

bool QyhShutdownNode::write_coil(int offset, bool value)
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (!modbus_ctx_)
  {
    return false;
  }

  try
  {
    modbus_ctx_->write_coil(offset, value);
    return true;
  }
  catch (const modbus::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to write coil M%d: %s", offset, e.what());
    return false;
  }
}

bool QyhShutdownNode::request_shutdown(uint8_t source)
{
  // 已经在关机中，不重复触发
  if (shutdown_in_progress_.load())
  {
    RCLCPP_INFO(this->get_logger(), "Shutdown already in progress");
    return true;
  }

  const char* source_name = (source == qyh_shutdown_msgs::msg::ShutdownState::SOURCE_HARDWARE) 
                            ? "HARDWARE" : "SOFTWARE";
  RCLCPP_WARN(this->get_logger(), "Shutdown requested from %s!", source_name);

  // 软件触发时需要写M100通知PLC
  if (source == qyh_shutdown_msgs::msg::ShutdownState::SOURCE_SOFTWARE)
  {
    if (!write_coil(shutdown_coil_address_, true))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to write M%d to PLC", shutdown_coil_address_);
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Written M%d=1 to PLC", shutdown_coil_address_);
  }

  // 更新状态
  trigger_source_ = source;
  shutdown_in_progress_.store(true);
  countdown_seconds_.store(shutdown_delay_seconds_);

  // 在新线程中执行关机
  std::thread(&QyhShutdownNode::execute_system_shutdown, this).detach();

  return true;
}

void QyhShutdownNode::execute_system_shutdown()
{
  RCLCPP_WARN(this->get_logger(), "System will shutdown in %d seconds...", 
              shutdown_delay_seconds_);

  // 倒计时
  for (int i = shutdown_delay_seconds_; i > 0; --i)
  {
    countdown_seconds_.store(i);
    RCLCPP_WARN(this->get_logger(), "Shutdown countdown: %d", i);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  countdown_seconds_.store(0);
  RCLCPP_WARN(this->get_logger(), "System shutdown NOW!");

  // 执行系统关机命令
  int result = std::system("sudo shutdown -h now");

  if (result != 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute shutdown command, code: %d", result);
    // 关机失败，重置状态
    shutdown_in_progress_.store(false);
    trigger_source_ = 0;
    countdown_seconds_.store(-1);
  }
}

void QyhShutdownNode::handle_control(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
  // 软件触发关机
  if (request_shutdown(qyh_shutdown_msgs::msg::ShutdownState::SOURCE_SOFTWARE))
  {
    response->success = true;
    response->message = "System shutdown initiated (software trigger)";
  }
  else
  {
    response->success = false;
    response->message = "Failed to initiate shutdown";
  }
}

}  // namespace qyh_shutdown

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<qyh_shutdown::QyhShutdownNode>();

  RCLCPP_INFO(node->get_logger(), "Qyh Shutdown Control Node is running...");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
