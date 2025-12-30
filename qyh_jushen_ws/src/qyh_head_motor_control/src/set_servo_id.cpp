/**
 * @file set_servo_id.cpp
 * @brief 舵机ID设置工具
 * 
 * 使用方法:
 *   ros2 run qyh_head_motor_control set_servo_id --ros-args -p new_id:=2 -p port:=/dev/ttyUSB0
 * 
 * 注意: 使用此工具时，必须只连接一个舵机，否则会报错退出
 */

#include <rclcpp/rclcpp.hpp>
#include "qyh_head_motor_control/bus_servo_protocol.hpp"

#include <iostream>
#include <thread>
#include <chrono>

class SetServoIdNode : public rclcpp::Node
{
public:
    SetServoIdNode() : Node("set_servo_id")
    {
        // 声明参数
        this->declare_parameter<int>("new_id", -1);
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 1000000);
        
        // 获取参数
        new_id_ = this->get_parameter("new_id").as_int();
        port_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();
        
        // 参数检查
        if (new_id_ < 0 || new_id_ > 253) {
            RCLCPP_ERROR(this->get_logger(), 
                "请指定有效的新ID (0-253)，使用参数: -p new_id:=<id>");
            RCLCPP_ERROR(this->get_logger(), 
                "示例: ros2 run qyh_head_motor_control set_servo_id --ros-args -p new_id:=2");
            rclcpp::shutdown();
            return;
        }
        
        // 执行ID设置
        runSetId();
    }

private:
    void runSetId()
    {
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "       舵机ID设置工具");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "串口: %s", port_.c_str());
        RCLCPP_INFO(this->get_logger(), "波特率: %d", baudrate_);
        RCLCPP_INFO(this->get_logger(), "目标ID: %d", new_id_);
        RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        
        // 创建协议对象
        qyh_head_motor_control::BusServoProtocol servo(port_, baudrate_);
        
        // 打开串口
        if (!servo.open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s", port_.c_str());
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "串口已打开，正在扫描舵机...");
        
        // 等待串口稳定
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 扫描连接的舵机数量
        std::vector<uint8_t> found_ids;
        
        // 方法1: 使用广播读取ID (只适用于单个舵机)
        uint8_t broadcast_id = 0;
        bool broadcast_success = servo.readServoId(broadcast_id);
        
        if (broadcast_success) {
            RCLCPP_INFO(this->get_logger(), "广播读取到舵机ID: %d", broadcast_id);
            found_ids.push_back(broadcast_id);
        }
        
        // 方法2: 逐个ID扫描 (ID 0-255 范围，可以扩大)
        RCLCPP_INFO(this->get_logger(), "正在扫描ID 0-255...");
        for (uint8_t id = 0; id <= 255; id++) {
            uint16_t position = 0;
            if (servo.readPosition(id, position)) {
                // 检查是否已经在列表中
                bool already_found = false;
                for (uint8_t fid : found_ids) {
                    if (fid == id) {
                        already_found = true;
                        break;
                    }
                }
                if (!already_found) {
                    RCLCPP_INFO(this->get_logger(), "扫描发现舵机ID: %d, 位置: %d", id, position);
                    found_ids.push_back(id);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        
        RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        RCLCPP_INFO(this->get_logger(), "扫描完成，发现 %zu 个舵机", found_ids.size());
        
        // 检查舵机数量
        if (found_ids.empty()) {
            RCLCPP_ERROR(this->get_logger(), "❌ 未发现任何舵机！");
            RCLCPP_ERROR(this->get_logger(), "请检查:");
            RCLCPP_ERROR(this->get_logger(), "  1. 舵机是否正确连接");
            RCLCPP_ERROR(this->get_logger(), "  2. 串口设备是否正确 (当前: %s)", port_.c_str());
            RCLCPP_ERROR(this->get_logger(), "  3. 波特率是否匹配 (当前: %d)", baudrate_);
            RCLCPP_ERROR(this->get_logger(), "  4. 舵机是否上电");
            servo.close();
            rclcpp::shutdown();
            return;
        }
        
        if (found_ids.size() > 1) {
            RCLCPP_ERROR(this->get_logger(), "❌ 发现多个舵机 (%zu 个)！", found_ids.size());
            RCLCPP_ERROR(this->get_logger(), "发现的ID列表:");
            for (uint8_t id : found_ids) {
                RCLCPP_ERROR(this->get_logger(), "  - ID: %d", id);
            }
            RCLCPP_ERROR(this->get_logger(), "");
            RCLCPP_ERROR(this->get_logger(), "⚠️  设置ID时必须只连接一个舵机！");
            RCLCPP_ERROR(this->get_logger(), "请断开其他舵机后重试。");
            servo.close();
            rclcpp::shutdown();
            return;
        }
        
        // 只有一个舵机，可以设置ID
        uint8_t old_id = found_ids[0];
        
        if (old_id == new_id_) {
            RCLCPP_WARN(this->get_logger(), "舵机当前ID已经是 %d，无需更改", new_id_);
            servo.close();
            rclcpp::shutdown();
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "准备将舵机ID从 %d 修改为 %d", old_id, new_id_);
        RCLCPP_INFO(this->get_logger(), "");
        
        // 设置新ID
        RCLCPP_INFO(this->get_logger(), "正在设置新ID...");
        
        if (!servo.setServoId(old_id, static_cast<uint8_t>(new_id_))) {
            RCLCPP_ERROR(this->get_logger(), "❌ 设置ID失败！");
            servo.close();
            rclcpp::shutdown();
            return;
        }
        
        // 等待舵机处理
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 验证新ID
        RCLCPP_INFO(this->get_logger(), "正在验证新ID...");
        uint16_t verify_position = 0;
        if (servo.readPosition(static_cast<uint8_t>(new_id_), verify_position)) {
            RCLCPP_INFO(this->get_logger(), "========================================");
            RCLCPP_INFO(this->get_logger(), "✅ ID设置成功！");
            RCLCPP_INFO(this->get_logger(), "   旧ID: %d -> 新ID: %d", old_id, new_id_);
            RCLCPP_INFO(this->get_logger(), "   当前位置: %d", verify_position);
            RCLCPP_INFO(this->get_logger(), "========================================");
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️  ID可能已设置，但验证失败");
            RCLCPP_WARN(this->get_logger(), "建议重新上电后检查");
        }
        
        servo.close();
        rclcpp::shutdown();
    }
    
    int new_id_;
    std::string port_;
    int baudrate_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SetServoIdNode>();
    
    // 节点会在构造函数中完成任务并调用 shutdown
    rclcpp::spin(node);
    
    return 0;
}
