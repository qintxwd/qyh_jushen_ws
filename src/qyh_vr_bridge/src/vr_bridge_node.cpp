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

/**
 * VR Bridge Node - PICO 4 VR 原始数据接收 (节点1/5)
 * 
 * 职责：
 *   - 接收PICO4 SDK的UDP数据包
 *   - 将PICO坐标系转换为ROS标准坐标系（底层对齐）
 *   - 发布原始controller位姿，不做高层语义处理
 * 
 * TF 树:
 *   vr_origin (ROS坐标系: X前 Y左 Z上)
 *       ├── vr_head
 *       ├── vr_left_controller  (原始VR手柄坐标)
 *       └── vr_right_controller (原始VR手柄坐标)
 *
 * 坐标变换说明:
 *   PICO SDK坐标系: X-右, Y-上, -Z-前 (OpenXR标准)
 *   ROS标准坐标系: X-前, Y-左, Z-上 (REP-103)
 *   
 *   底层对齐映射（固定）:
 *     ros_x = -vr_z   (PICO的-Z向前 → ROS的X向前)
 *     ros_y = -vr_x   (PICO的-X向左 → ROS的Y向左)
 *     ros_z = vr_y    (PICO的Y向上  → ROS的Z向上)
 *   
 *   四元数同样映射:
 *     ros_qx = -vr_qz, ros_qy = -vr_qx, ros_qz = vr_qy, ros_qw = vr_qw
 *
 * 参数:
 *   udp_port (int): UDP监听端口，默认9999
 *
 * 发布话题:
 *   /vr/head/pose (PoseStamped)
 *   /vr/left_controller/pose, /vr/right_controller/pose (PoseStamped)
 *   /vr/left_controller/joy, /vr/right_controller/joy (Joy)
 *   /vr/left_controller/active, /vr/right_controller/active (Bool)
 * 
 * 发布TF:
 *   vr_origin → vr_head
 *   vr_origin → vr_left_controller
 *   vr_origin → vr_right_controller
 * 
 * 注意：
 *   - 不包含握持补偿、缩放、滤波等高层处理
 *   - 这些由后续节点 (coordinate_mapper_node) 负责
 */

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
        udp_port_ = this->get_parameter("udp_port").as_int();

        // Publishers - 使用 controller 命名（原始VR数据）
        head_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vr/head/pose", 10);
        left_controller_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vr/left_controller/pose", 10);
        left_controller_pose_raw_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vr/left_controller/pose_raw", 10);
        left_controller_joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/vr/left_controller/joy", 10);
        left_controller_active_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vr/left_controller/active", 10);
        right_controller_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vr/right_controller/pose", 10);
        right_controller_pose_raw_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vr/right_controller/pose_raw", 10);
        right_controller_joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/vr/right_controller/joy", 10);
        right_controller_active_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vr/right_controller/active", 10);

        // TF Broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        RCLCPP_INFO(this->get_logger(), "=== VR Bridge Node (1/5) ===");
        RCLCPP_INFO(this->get_logger(), "Role: Raw VR data receiver");
        RCLCPP_INFO(this->get_logger(), "Coordinate mapping: PICO SDK → ROS standard (REP-103)");

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
        // Debug: log buttons bitmask changes to help map which bits belong to which controller
        if (packet.buttons_bitmask != prev_buttons_bitmask_) {
            uint32_t bm = packet.buttons_bitmask;
            std::string bits;
            for (int i = 0; i < 32; ++i) {
                if (bm & (1u << i)) {
                    if (!bits.empty()) bits += ",";
                    bits += std::to_string(i);
                }
            }
            if (bits.empty()) bits = "(none)";
            RCLCPP_INFO(this->get_logger(), "buttons_bitmask changed: 0x%08x set_bits=%s", bm, bits.c_str());
            prev_buttons_bitmask_ = bm;
        }
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        
        // 调试输出计数
        static int debug_count = 0;
        debug_count++;
        bool should_log = (debug_count % 5 == 0);  // 每50帧输出一次

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

        // === Left Controller ===
        std_msgs::msg::Bool left_active_msg;
        left_active_msg.data = (packet.left_active != 0);
        left_controller_active_pub_->publish(left_active_msg);

        if (packet.left_active) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "vr_origin";
            t.child_frame_id = "vr_left_controller";
            
            // 原始VR数据 (PICO坐标系)
            float vr_x = packet.left_position[0];
            float vr_y = packet.left_position[1];
            float vr_z = packet.left_position[2];
            
            // 底层坐标对齐：PICO SDK → ROS标准
            map_position(vr_x, vr_y, vr_z,
                        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
            
            // // 调试输出
            // if (should_log) {
            //     RCLCPP_INFO(this->get_logger(), 
            //         "[LEFT] PICO: [%.3f, %.3f, %.3f] -> ROS: [%.3f, %.3f, %.3f]",
            //         vr_x, vr_y, vr_z,
            //         t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
            // }
            
            // 四元数坐标对齐（不做额外旋转补偿）
            tf2::Quaternion q = map_quaternion(
                packet.left_orientation[0], packet.left_orientation[1],
                packet.left_orientation[2], packet.left_orientation[3]);
            
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
            left_controller_pose_pub_->publish(msg);

            // Joy
            sensor_msgs::msg::Joy joy;
            joy.header.stamp = now;
            joy.axes = {packet.left_joystick[0], packet.left_joystick[1],
                       packet.left_trigger, packet.left_grip};
            // 左手柄buttons: A/B/X/Y/Menu/JoyClick（与右手统一）
            joy.buttons = {
                (packet.buttons_bitmask & (1 << 0)) ? 1 : 0,   // A
                (packet.buttons_bitmask & (1 << 1)) ? 1 : 0,   // B
                (packet.buttons_bitmask & (1 << 2)) ? 1 : 0,   // X
                (packet.buttons_bitmask & (1 << 3)) ? 1 : 0,   // Y
                (packet.buttons_bitmask & (1 << 4)) ? 1 : 0,   // Menu
                (packet.buttons_bitmask & (1 << 10)) ? 1 : 0   // Joy Click
            };
            left_controller_joy_pub_->publish(joy);

            //发布其原始的xyz和rpy数据，不做任何变换
            geometry_msgs::msg::PoseStamped raw_msg;
            raw_msg.pose.position.x = packet.left_position[0];
            raw_msg.pose.position.y = packet.left_position[1];
            raw_msg.pose.position.z = packet.left_position[2];
            raw_msg.pose.orientation.x = packet.left_orientation[0];
            raw_msg.pose.orientation.y = packet.left_orientation[1];
            raw_msg.pose.orientation.z = packet.left_orientation[2];
            raw_msg.pose.orientation.w = packet.left_orientation[3];
            left_controller_pose_raw_pub_->publish(raw_msg);
        }

        // === Right Controller ===
        std_msgs::msg::Bool right_active_msg;
        right_active_msg.data = (packet.right_active != 0);
        right_controller_active_pub_->publish(right_active_msg);

        if (packet.right_active) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "vr_origin";
            t.child_frame_id = "vr_right_controller";
            
            // 原始VR数据 (PICO坐标系)
            float vr_x = packet.right_position[0];
            float vr_y = packet.right_position[1];
            float vr_z = packet.right_position[2];
            
            // 底层坐标对齐：PICO SDK → ROS标准
            map_position(vr_x, vr_y, vr_z,
                        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
            
            // // 调试输出
            // if (should_log) {
            //     RCLCPP_INFO(this->get_logger(), 
            //         "[RIGHT] PICO: [%.3f, %.3f, %.3f] -> ROS: [%.3f, %.3f, %.3f]",
            //         vr_x, vr_y, vr_z,
            //         t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
            // }
            
            // 四元数坐标对齐（不做额外旋转补偿）
            tf2::Quaternion q = map_quaternion(
                packet.right_orientation[0], packet.right_orientation[1],
                packet.right_orientation[2], packet.right_orientation[3]);
            
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
            right_controller_pose_pub_->publish(msg);

            // Joy
            sensor_msgs::msg::Joy joy;
            joy.header.stamp = now;
            joy.axes = {packet.right_joystick[0], packet.right_joystick[1],
                       packet.right_trigger, packet.right_grip};
            // 右手柄buttons: A/B/X/Y/Menu/JoyClick（与左手统一，X/Y/Menu/JoyClick按bit位）
            joy.buttons = {
                (packet.buttons_bitmask & (1 << 0)) ? 1 : 0,   // A
                (packet.buttons_bitmask & (1 << 1)) ? 1 : 0,   // B
                (packet.buttons_bitmask & (1 << 2)) ? 1 : 0,   // X
                (packet.buttons_bitmask & (1 << 3)) ? 1 : 0,   // Y
                (packet.buttons_bitmask & (1 << 4)) ? 1 : 0,   // Menu
                (packet.buttons_bitmask & (1 << 10)) ? 1 : 0   // Joy Click
            };
            right_controller_joy_pub_->publish(joy);

            //发布其原始的xyz和rpy数据，不做任何变换
            geometry_msgs::msg::PoseStamped raw_msg;
            raw_msg.pose.position.x = packet.right_position[0];
            raw_msg.pose.position.y = packet.right_position[1];
            raw_msg.pose.position.z = packet.right_position[2];
            raw_msg.pose.orientation.x = packet.right_orientation[0];
            raw_msg.pose.orientation.y = packet.right_orientation[1];
            raw_msg.pose.orientation.z = packet.right_orientation[2];
            raw_msg.pose.orientation.w = packet.right_orientation[3];
            right_controller_pose_raw_pub_->publish(raw_msg);
        }

        // 批量发布所有 TF
        for (const auto& tf : transforms) {
            tf_broadcaster_->sendTransform(tf);
        }
    }

    int udp_socket_ = -1;
    int udp_port_;
    std::thread recv_thread_;
    std::atomic<bool> running_;

    // previous buttons bitmask for debugging
    uint32_t prev_buttons_bitmask_ = 0;

    // Publishers - 原始controller数据
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr head_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_controller_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_controller_pose_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr left_controller_joy_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_controller_active_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_controller_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_controller_pose_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr right_controller_joy_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_controller_active_pub_;

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
