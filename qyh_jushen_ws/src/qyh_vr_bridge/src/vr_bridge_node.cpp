#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <cstring>
#include <iostream>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

// 简化的 TF 树:
//   vr_origin (ROS坐标系)
//       ├── vr_head
//       ├── vr_left_hand (含握持补偿)
//       └── vr_right_hand (含握持补偿)
//
// VR坐标系 (PICO): X-右, Y-上, -Z-前
// ROS坐标系: X-前, Y-左, Z-上
// 位置映射: ros_x = -vr_z, ros_y = -vr_x, ros_z = vr_y
// 四元数映射: ros_qx = -vr_qz, ros_qy = -vr_qx, ros_qz = vr_qy, ros_qw = vr_qw

#pragma pack(push, 1)
struct ControllerDataPacket {
    int64_t timestamp;           // 8 bytes
    
    // Head Pose
    float head_position[3];      // 12 bytes
    float head_orientation[4];   // 16 bytes: x, y, z, w

    // Left controller
    uint8_t left_active;         // 1 byte
    float left_position[3];      // 12 bytes
    float left_orientation[4];   // 16 bytes
    float left_joystick[2];      // 8 bytes
    float left_trigger;          // 4 bytes
    float left_grip;             // 4 bytes
    
    // Right controller
    uint8_t right_active;        // 1 byte
    float right_position[3];     // 12 bytes
    float right_orientation[4];  // 16 bytes
    float right_joystick[2];     // 8 bytes
    float right_trigger;         // 4 bytes
    float right_grip;            // 4 bytes
    
    // Buttons and touches
    uint32_t buttons_bitmask;    // 4 bytes
    uint32_t touches_bitmask;    // 4 bytes
};
#pragma pack(pop)

class VRBridgeNode : public rclcpp::Node
{
public:
    VRBridgeNode() : Node("vr_bridge_node")
    {
        // Parameters
        this->declare_parameter("udp_port", 9999);
        this->declare_parameter("grip_offset_deg", 35.0);  // 默认35度握持补偿
        
        udp_port_ = this->get_parameter("udp_port").as_int();
        grip_offset_deg_ = this->get_parameter("grip_offset_deg").as_double();
        
        // 预计算握持补偿四元数 (绕Y轴旋转，pitch方向)
        grip_offset_q_.setRPY(0, grip_offset_deg_ * M_PI / 180.0, 0);
        grip_offset_q_.normalize();

        // Publishers
        head_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vr/head/pose", 10);
        left_hand_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vr/left_hand/pose", 10);
        left_hand_joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/vr/left_hand/joy", 10);
        left_hand_active_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vr/left_hand/active", 10);
        right_hand_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vr/right_hand/pose", 10);
        right_hand_joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/vr/right_hand/joy", 10);
        right_hand_active_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vr/right_hand/active", 10);

        // TF Broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        RCLCPP_INFO(this->get_logger(), "VR Bridge - Simplified coordinate mapping");
        RCLCPP_INFO(this->get_logger(), "Grip offset: %.1f deg (pitch)", grip_offset_deg_);

        // Initialize UDP
        if (!init_udp()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize UDP socket");
            return;
        }

        // Start receive thread
        running_ = true;
        recv_thread_ = std::thread(&VRBridgeNode::receive_loop, this);
        
        RCLCPP_INFO(this->get_logger(), "Listening on UDP port %d", udp_port_);
    }

    ~VRBridgeNode()
    {
        running_ = false;
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
        close_udp();
    }

private:
    // VR -> ROS 位置坐标映射
    // VR: X-右, Y-上, -Z-前  =>  ROS: X-前, Y-左, Z-上
    void map_position(float vr_x, float vr_y, float vr_z, 
                      double& ros_x, double& ros_y, double& ros_z)
    {
        ros_x = -vr_z;   // VR的-Z(前) -> ROS的X(前)
        ros_y = -vr_x;   // VR的-X(左) -> ROS的Y(左)
        ros_z = vr_y;    // VR的Y(上)  -> ROS的Z(上)
    }
    
    // VR -> ROS 四元数坐标映射 (同样的轴映射规则)
    tf2::Quaternion map_quaternion(float vr_qx, float vr_qy, float vr_qz, float vr_qw)
    {
        // 四元数的虚部代表旋转轴方向，需要同样的坐标变换
        tf2::Quaternion q(-vr_qz, -vr_qx, vr_qy, vr_qw);
        q.normalize();
        return q;
    }

    bool init_udp()
    {
#ifdef _WIN32
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            return false;
        }
#endif
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            return false;
        }

        int reuse = 1;
        setsockopt(udp_socket_, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(udp_port_);
        addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(udp_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            return false;
        }
        return true;
    }

    void close_udp()
    {
        if (udp_socket_ >= 0) {
#ifdef _WIN32
            closesocket(udp_socket_);
            WSACleanup();
#else
            close(udp_socket_);
#endif
            udp_socket_ = -1;
        }
    }

    void receive_loop()
    {
        ControllerDataPacket packet;
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        
        while (running_ && rclcpp::ok()) {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(udp_socket_, &readfds);
            
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 100000; // 100ms timeout

            int ret = select(udp_socket_ + 1, &readfds, NULL, NULL, &tv);
            
            if (ret > 0) {
                int len = recvfrom(udp_socket_, (char*)&packet, sizeof(packet), 0, 
                                   (struct sockaddr*)&client_addr, &addr_len);
                
                if (len == sizeof(ControllerDataPacket)) {
                    process_packet(packet);
                } else if (len > 0) {
                    RCLCPP_WARN(this->get_logger(), 
                        "Received packet size %d, expected %zu", len, sizeof(ControllerDataPacket));
                }
            }
        }
    }

    void process_packet(const ControllerDataPacket& packet)
    {
        auto now = this->now();
        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        // === Head ===
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "vr_origin";
            t.child_frame_id = "vr_head";
            
            map_position(packet.head_position[0], packet.head_position[1], packet.head_position[2],
                        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
            
            tf2::Quaternion q = map_quaternion(
                packet.head_orientation[0], packet.head_orientation[1],
                packet.head_orientation[2], packet.head_orientation[3]);
            
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            transforms.push_back(t);
            
            // PoseStamped
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = now;
            msg.header.frame_id = "vr_origin";
            msg.pose.position.x = t.transform.translation.x;
            msg.pose.position.y = t.transform.translation.y;
            msg.pose.position.z = t.transform.translation.z;
            msg.pose.orientation = t.transform.rotation;
            head_pose_pub_->publish(msg);
        }

        // === Left Hand ===
        std_msgs::msg::Bool left_active_msg;
        left_active_msg.data = (packet.left_active != 0);
        left_hand_active_pub_->publish(left_active_msg);

        if (packet.left_active) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "vr_origin";
            t.child_frame_id = "vr_left_hand";
            
            map_position(packet.left_position[0], packet.left_position[1], packet.left_position[2],
                        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
            
            tf2::Quaternion q = map_quaternion(
                packet.left_orientation[0], packet.left_orientation[1],
                packet.left_orientation[2], packet.left_orientation[3]);
            
            // 应用握持补偿 (本体坐标系下的旋转，右乘)
            q = q * grip_offset_q_;
            q.normalize();
            
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            transforms.push_back(t);
            
            // PoseStamped
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = now;
            msg.header.frame_id = "vr_origin";
            msg.pose.position.x = t.transform.translation.x;
            msg.pose.position.y = t.transform.translation.y;
            msg.pose.position.z = t.transform.translation.z;
            msg.pose.orientation = t.transform.rotation;
            left_hand_pose_pub_->publish(msg);

            // Joy
            sensor_msgs::msg::Joy joy;
            joy.header.stamp = now;
            joy.axes = {packet.left_joystick[0], packet.left_joystick[1],
                       packet.left_trigger, packet.left_grip};
            joy.buttons = {
                (packet.buttons_bitmask & (1 << 2)) ? 1 : 0,   // X
                (packet.buttons_bitmask & (1 << 3)) ? 1 : 0,   // Y
                (packet.buttons_bitmask & (1 << 4)) ? 1 : 0,   // Menu
                (packet.buttons_bitmask & (1 << 10)) ? 1 : 0   // Joy Click
            };
            left_hand_joy_pub_->publish(joy);
        }

        // === Right Hand ===
        std_msgs::msg::Bool right_active_msg;
        right_active_msg.data = (packet.right_active != 0);
        right_hand_active_pub_->publish(right_active_msg);

        if (packet.right_active) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "vr_origin";
            t.child_frame_id = "vr_right_hand";
            
            map_position(packet.right_position[0], packet.right_position[1], packet.right_position[2],
                        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
            
            tf2::Quaternion q = map_quaternion(
                packet.right_orientation[0], packet.right_orientation[1],
                packet.right_orientation[2], packet.right_orientation[3]);
            
            // 应用握持补偿
            q = q * grip_offset_q_;
            q.normalize();
            
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            transforms.push_back(t);
            
            // PoseStamped
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = now;
            msg.header.frame_id = "vr_origin";
            msg.pose.position.x = t.transform.translation.x;
            msg.pose.position.y = t.transform.translation.y;
            msg.pose.position.z = t.transform.translation.z;
            msg.pose.orientation = t.transform.rotation;
            right_hand_pose_pub_->publish(msg);

            // Joy
            sensor_msgs::msg::Joy joy;
            joy.header.stamp = now;
            joy.axes = {packet.right_joystick[0], packet.right_joystick[1],
                       packet.right_trigger, packet.right_grip};
            joy.buttons = {
                (packet.buttons_bitmask & (1 << 0)) ? 1 : 0,   // A
                (packet.buttons_bitmask & (1 << 1)) ? 1 : 0,   // B
                (packet.buttons_bitmask & (1 << 5)) ? 1 : 0,   // Home
                (packet.buttons_bitmask & (1 << 11)) ? 1 : 0   // Joy Click
            };
            right_hand_joy_pub_->publish(joy);
        }

        // 批量发布所有 TF
        for (const auto& tf : transforms) {
            tf_broadcaster_->sendTransform(tf);
        }
    }

    int udp_socket_ = -1;
    int udp_port_;
    double grip_offset_deg_;
    tf2::Quaternion grip_offset_q_;
    std::thread recv_thread_;
    std::atomic<bool> running_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr head_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_hand_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr left_hand_joy_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_hand_active_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_hand_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr right_hand_joy_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_hand_active_pub_;

    // TF Broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VRBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
