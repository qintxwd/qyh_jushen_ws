#include <rclcpp/rclcpp.hpp>
#include <qyh_jaka_control_msgs/msg/jaka_dual_joint_servo.hpp>
#include <qyh_jaka_control_msgs/msg/jaka_dual_cartesian_servo.hpp>
#include <qyh_jaka_control_msgs/msg/jaka_servo_status.hpp>
#include <qyh_jaka_control_msgs/srv/start_servo.hpp>
#include <qyh_jaka_control_msgs/srv/stop_servo.hpp>
#include <qyh_jaka_control_msgs/srv/zero_ft.hpp>
#include <qyh_jaka_control_msgs/srv/set_ft_config.hpp>
#include <qyh_jaka_control_msgs/srv/set_filter.hpp>
#include <chrono>
#include <atomic>

using namespace std::chrono_literals;

class JakaServoNode : public rclcpp::Node {
public:
  JakaServoNode() : rclcpp::Node("jaka_servo_node"), running_(false) {
    declare_parameter<double>("cycle_time_ms", 1.0);
    declare_parameter<bool>("use_cartesian", false);
    cycle_time_ms_ = get_parameter("cycle_time_ms").as_double();

    status_pub_ = create_publisher<qyh_jaka_control_msgs::msg::JakaServoStatus>("/jaka/servo/status", 10);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    joint_sub_ = create_subscription<qyh_jaka_control_msgs::msg::JakaDualJointServo>(
        "/jaka/servo/joint_cmd", qos,
        [this](qyh_jaka_control_msgs::msg::JakaDualJointServo::SharedPtr msg) {
          last_joint_ = *msg;
          last_time_ = now();
        });
    cartesian_sub_ = create_subscription<qyh_jaka_control_msgs::msg::JakaDualCartesianServo>(
        "/jaka/servo/cartesian_cmd", qos,
        [this](qyh_jaka_control_msgs::msg::JakaDualCartesianServo::SharedPtr msg) {
          last_cartesian_ = *msg;
          last_time_ = now();
        });

    srv_start_ = create_service<qyh_jaka_control_msgs::srv::StartServo>(
        "/jaka/servo/start",
        [this](const auto, auto res) {
          running_ = true;
          res->success = true;
          res->message = "servo started";
        });
    srv_stop_ = create_service<qyh_jaka_control_msgs::srv::StopServo>(
        "/jaka/servo/stop",
        [this](const auto, auto res) {
          running_ = false;
          res->success = true;
          res->message = "servo stopped";
        });
    srv_zero_ft_ = create_service<qyh_jaka_control_msgs::srv::ZeroFT>(
        "/jaka/servo/zero_ft",
        [this](const auto, auto res) {
          res->success = true;
          res->message = "zero ft requested";
        });
    srv_set_ft_ = create_service<qyh_jaka_control_msgs::srv::SetFTConfig>(
        "/jaka/servo/set_ft_config",
        [this](const auto, auto res) {
          res->success = true;
          res->message = "ft config set";
        });
    srv_set_filter_ = create_service<qyh_jaka_control_msgs::srv::SetFilter>(
        "/jaka/servo/set_filter",
        [this](const auto, auto res) {
          res->success = true;
          res->message = "filter set";
        });

    auto period = std::chrono::milliseconds(static_cast<int>(cycle_time_ms_));
    timer_ = create_wall_timer(period, std::bind(&JakaServoNode::loop, this));
  }

private:
  void loop() {
    auto msg = qyh_jaka_control_msgs::msg::JakaServoStatus();
    msg.mode = use_cartesian_ ? std::string("cartesian") : std::string("joint");
    msg.is_abs = true; // 默认ABS，真实模式由最后一条指令决定
    msg.cycle_time_ns = static_cast<int32_t>(cycle_time_ms_ * 1e6);
    msg.publish_rate_hz = 1000.0 / cycle_time_ms_;
    msg.latency_ms = 0.0;
    msg.packet_loss_rate = 0.0;
    msg.error_code = 0;

    if (!running_) {
      status_pub_->publish(msg);
      return;
    }
    // 这里对接SDK：edg_recv → 根据use_cartesian_选择 edg_servo_j / edg_servo_p → edg_send
    // 当前骨架仅发布状态以打通链路
    status_pub_->publish(msg);
  }

  // pubs/subs/services
  rclcpp::Publisher<qyh_jaka_control_msgs::msg::JakaServoStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<qyh_jaka_control_msgs::msg::JakaDualJointServo>::SharedPtr joint_sub_;
  rclcpp::Subscription<qyh_jaka_control_msgs::msg::JakaDualCartesianServo>::SharedPtr cartesian_sub_;
  rclcpp::Service<qyh_jaka_control_msgs::srv::StartServo>::SharedPtr srv_start_;
  rclcpp::Service<qyh_jaka_control_msgs::srv::StopServo>::SharedPtr srv_stop_;
  rclcpp::Service<qyh_jaka_control_msgs::srv::ZeroFT>::SharedPtr srv_zero_ft_;
  rclcpp::Service<qyh_jaka_control_msgs::srv::SetFTConfig>::SharedPtr srv_set_ft_;
  rclcpp::Service<qyh_jaka_control_msgs::srv::SetFilter>::SharedPtr srv_set_filter_;

  // state
  qyh_jaka_control_msgs::msg::JakaDualJointServo last_joint_;
  qyh_jaka_control_msgs::msg::JakaDualCartesianServo last_cartesian_;
  rclcpp::Time last_time_;
  rclcpp::TimerBase::SharedPtr timer_;
  double cycle_time_ms_;
  bool use_cartesian_ {false};
  std::atomic<bool> running_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JakaServoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}