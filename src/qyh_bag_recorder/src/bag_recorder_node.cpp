/**
 * @file bag_recorder_node.cpp
 * @brief QYH Bag Recorder - 录制指定话题到 rosbag2 文件
 * 
 * 功能:
 * - 通过服务调用开始/停止录制
 * - 支持动态指定要录制的话题列表
 * - 自动按 动作名/用户名_版本号_时间戳 格式保存
 */

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rcpputils/filesystem_helper.hpp>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <map>
#include <filesystem>

#include "qyh_bag_recorder/srv/start_recording.hpp"
#include "qyh_bag_recorder/srv/stop_recording.hpp"
#include "qyh_bag_recorder/srv/get_recording_status.hpp"

namespace fs = std::filesystem;

class BagRecorderNode : public rclcpp::Node
{
public:
    BagRecorderNode() : Node("bag_recorder_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("base_path", "");
        this->declare_parameter<std::string>("robot_name", "");
        this->declare_parameter<std::string>("robot_version", "");
        
        // 获取基础路径
        std::string base_path = this->get_parameter("base_path").as_string();
        if (base_path.empty()) {
            // 默认使用 ~/qyh-robot-system/model_actions/{robot_name}/{version}/
            const char* home = std::getenv("HOME");
            if (home) {
                base_path_ = std::string(home) + "/qyh-robot-system/model_actions";
            } else {
                base_path_ = "/tmp/qyh_bag_data";
            }
        } else {
            base_path_ = base_path;
        }
        
        // 获取机器人类型和版本（从参数或环境变量）
        robot_name_ = this->get_parameter("robot_name").as_string();
        if (robot_name_.empty()) {
            const char* env_robot = std::getenv("GLOBAL_ROBOT_NAME");
            robot_name_ = env_robot ? std::string(env_robot) : "general";
        }
        
        robot_version_ = this->get_parameter("robot_version").as_string();
        if (robot_version_.empty()) {
            const char* env_version = std::getenv("GLOBAL_ROBOT_VERSION");
            robot_version_ = env_version ? std::string(env_version) : "1.0";
        }
        
        // 完整路径包含机器人类型和版本
        // 结构: model_actions/{robot_name}/{version}/{action_id}/data/bags/
        full_base_path_ = base_path_ + "/" + robot_name_ + "/" + robot_version_;
        
        RCLCPP_INFO(this->get_logger(), "Bag Recorder 初始化");
        RCLCPP_INFO(this->get_logger(), "  基础路径: %s", base_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  机器人: %s / %s", robot_name_.c_str(), robot_version_.c_str());
        RCLCPP_INFO(this->get_logger(), "  完整路径: %s", full_base_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  话题将通过服务请求动态指定");
        
        // 创建服务
        start_recording_srv_ = this->create_service<qyh_bag_recorder::srv::StartRecording>(
            "~/start_recording",
            std::bind(&BagRecorderNode::handleStartRecording, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        stop_recording_srv_ = this->create_service<qyh_bag_recorder::srv::StopRecording>(
            "~/stop_recording",
            std::bind(&BagRecorderNode::handleStopRecording, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        get_status_srv_ = this->create_service<qyh_bag_recorder::srv::GetRecordingStatus>(
            "~/get_status",
            std::bind(&BagRecorderNode::handleGetStatus, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "服务已创建:");
        RCLCPP_INFO(this->get_logger(), "  - ~/start_recording");
        RCLCPP_INFO(this->get_logger(), "  - ~/stop_recording");
        RCLCPP_INFO(this->get_logger(), "  - ~/get_status");
    }
    
    ~BagRecorderNode()
    {
        // 如果还在录制，停止录制
        if (is_recording_) {
            stopRecordingInternal();
        }
    }

private:
    /**
     * @brief 生成时间戳字符串 YYYY_MM_dd_HH_mm_ss
     */
    std::string generateTimestamp()
    {
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::tm tm_now;
        localtime_r(&time_t_now, &tm_now);
        
        std::ostringstream oss;
        oss << std::put_time(&tm_now, "%Y_%m_%d_%H_%M_%S");
        return oss.str();
    }
    
    /**
     * @brief 生成包名: qyh_用户名_版本号_YYYY_MM_dd_HH_mm_ss
     */
    std::string generateBagName(const std::string& user_name, const std::string& version)
    {
        std::ostringstream oss;
        oss << "qyh_" << user_name << "_" << version << "_" << generateTimestamp();
        return oss.str();
    }
    
    /**
     * @brief 确保目录存在
     */
    bool ensureDirectoryExists(const std::string& path)
    {
        try {
            if (!fs::exists(path)) {
                fs::create_directories(path);
                RCLCPP_INFO(this->get_logger(), "创建目录: %s", path.c_str());
            }
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "创建目录失败: %s, 错误: %s", 
                        path.c_str(), e.what());
            return false;
        }
    }
    
    /**
     * @brief 开始录制
     */
    /**
     * @brief 获取话题的消息类型
     */
    std::string getTopicType(const std::string& topic_name)
    {
        auto topic_names_and_types = this->get_topic_names_and_types();
        for (const auto& [name, types] : topic_names_and_types) {
            if (name == topic_name && !types.empty()) {
                return types[0];
            }
        }
        return "";
    }
    
    /**
     * @brief 获取当前系统中所有可用的话题
     */
    std::vector<std::string> getAvailableTopics()
    {
        std::vector<std::string> topics;
        auto topic_names_and_types = this->get_topic_names_and_types();
        for (const auto& [name, types] : topic_names_and_types) {
            topics.push_back(name);
        }
        return topics;
    }
    
    bool startRecordingInternal(const std::string& action_name,
                                const std::string& user_name,
                                const std::string& version,
                                const std::vector<std::string>& topics)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (is_recording_) {
            RCLCPP_WARN(this->get_logger(), "已经在录制中");
            return false;
        }
        
        // 必须指定要录制的话题
        if (topics.empty()) {
            RCLCPP_ERROR(this->get_logger(), "没有指定要录制的话题");
            return false;
        }
        
        current_topics_ = topics;
        
        // 创建动作数据目录: model_actions/{robot}/{version}/{action_id}/data/bags/
        std::string action_data_dir = full_base_path_ + "/" + action_name + "/data/bags";
        if (!ensureDirectoryExists(action_data_dir)) {
            return false;
        }
        
        // 生成包名和完整路径
        std::string bag_name = generateBagName(user_name, version);
        current_bag_path_ = action_data_dir + "/" + bag_name;
        current_action_name_ = action_name;
        
        RCLCPP_INFO(this->get_logger(), "开始录制:");
        RCLCPP_INFO(this->get_logger(), "  动作: %s", action_name.c_str());
        RCLCPP_INFO(this->get_logger(), "  机器人: %s / %s", robot_name_.c_str(), robot_version_.c_str());
        RCLCPP_INFO(this->get_logger(), "  用户: %s", user_name.c_str());
        RCLCPP_INFO(this->get_logger(), "  版本: %s", version.c_str());
        RCLCPP_INFO(this->get_logger(), "  路径: %s", current_bag_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  话题数: %zu", current_topics_.size());
        
        for (const auto& topic : current_topics_) {
            RCLCPP_INFO(this->get_logger(), "    - %s", topic.c_str());
        }
        
        try {
            // 创建 writer
            writer_ = std::make_unique<rosbag2_cpp::Writer>();
            
            // 配置存储选项
            rosbag2_storage::StorageOptions storage_options;
            storage_options.uri = current_bag_path_;
            storage_options.storage_id = "sqlite3";  // 使用 sqlite3 格式
            
            // 打开 bag
            writer_->open(storage_options);
            
            // 为每个话题创建订阅
            createSubscriptions();
            
            // 记录开始时间
            recording_start_time_ = this->now();
            is_recording_ = true;
            
            RCLCPP_INFO(this->get_logger(), "录制已开始");
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "启动录制失败: %s", e.what());
            writer_.reset();
            subscriptions_.clear();
            return false;
        }
    }
    
    /**
     * @brief 停止录制
     */
    bool stopRecordingInternal()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!is_recording_) {
            RCLCPP_WARN(this->get_logger(), "当前没有在录制");
            return false;
        }
        
        // 计算录制时长
        auto duration = this->now() - recording_start_time_;
        last_duration_sec_ = duration.seconds();
        
        RCLCPP_INFO(this->get_logger(), "停止录制:");
        RCLCPP_INFO(this->get_logger(), "  时长: %.2f 秒", last_duration_sec_);
        RCLCPP_INFO(this->get_logger(), "  路径: %s", current_bag_path_.c_str());
        
        // 清理订阅
        subscriptions_.clear();
        
        // 关闭 writer
        if (writer_) {
            writer_.reset();
        }
        
        last_bag_path_ = current_bag_path_;
        is_recording_ = false;
        
        RCLCPP_INFO(this->get_logger(), "录制已停止");
        return true;
    }
    
    /**
     * @brief 创建话题订阅
     */
    void createSubscriptions()
    {
        subscriptions_.clear();
        topic_types_.clear();
        
        for (const auto& topic : current_topics_) {
            // 动态获取话题类型
            std::string topic_type = getTopicType(topic);
            
            if (topic_type.empty()) {
                RCLCPP_WARN(this->get_logger(), "无法获取话题 %s 的类型，跳过", topic.c_str());
                continue;
            }
            
            topic_types_[topic] = topic_type;
            RCLCPP_INFO(this->get_logger(), "订阅话题: %s [%s]", topic.c_str(), topic_type.c_str());
            
            // 在 writer 中注册话题
            rosbag2_storage::TopicMetadata topic_metadata;
            topic_metadata.name = topic;
            topic_metadata.type = topic_type;
            topic_metadata.serialization_format = "cdr";
            writer_->create_topic(topic_metadata);
            
            // 使用通用订阅（序列化消息），动态获取消息类型
            try {
                auto subscription = this->create_generic_subscription(
                    topic,
                    topic_type,
                    rclcpp::QoS(10).best_effort().durability_volatile(),
                    [this, topic](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                        this->writeMessage(topic, msg);
                    }
                );
                subscriptions_.push_back(subscription);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "创建订阅失败 %s: %s", topic.c_str(), e.what());
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "成功订阅 %zu 个话题", subscriptions_.size());
    }
    
    /**
     * @brief 写入消息到 bag
     */
    void writeMessage(const std::string& topic, 
                      std::shared_ptr<rclcpp::SerializedMessage> msg)
    {
        if (!is_recording_ || !writer_) {
            return;
        }
        
        try {
            auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            bag_msg->topic_name = topic;
            bag_msg->time_stamp = this->now().nanoseconds();
            bag_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();
            
            // 复制序列化数据
            auto& serialized = msg->get_rcl_serialized_message();
            bag_msg->serialized_data->buffer = serialized.buffer;
            bag_msg->serialized_data->buffer_length = serialized.buffer_length;
            bag_msg->serialized_data->buffer_capacity = serialized.buffer_capacity;
            
            writer_->write(bag_msg);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "写入消息失败: %s", e.what());
        }
    }
    
    // ==================== 服务回调 ====================
    
    void handleStartRecording(
        const std::shared_ptr<qyh_bag_recorder::srv::StartRecording::Request> request,
        std::shared_ptr<qyh_bag_recorder::srv::StartRecording::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "收到开始录制请求");
        
        if (request->action_name.empty()) {
            response->success = false;
            response->message = "动作名称不能为空";
            return;
        }
        
        if (request->user_name.empty()) {
            response->success = false;
            response->message = "用户名不能为空";
            return;
        }
        
        if (request->version.empty()) {
            response->success = false;
            response->message = "版本号不能为空";
            return;
        }
        
        std::vector<std::string> topics(request->topics.begin(), request->topics.end());
        
        if (startRecordingInternal(request->action_name, request->user_name, 
                                   request->version, topics)) {
            response->success = true;
            response->message = "录制已开始";
            response->bag_path = current_bag_path_;
        } else {
            response->success = false;
            response->message = is_recording_ ? "已经在录制中" : "启动录制失败";
            response->bag_path = "";
        }
    }
    
    void handleStopRecording(
        const std::shared_ptr<qyh_bag_recorder::srv::StopRecording::Request> /*request*/,
        std::shared_ptr<qyh_bag_recorder::srv::StopRecording::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "收到停止录制请求");
        
        if (stopRecordingInternal()) {
            response->success = true;
            response->message = "录制已停止";
            response->duration_sec = last_duration_sec_;
            response->bag_path = last_bag_path_;
        } else {
            response->success = false;
            response->message = "当前没有在录制";
            response->duration_sec = 0.0;
            response->bag_path = "";
        }
    }
    
    void handleGetStatus(
        const std::shared_ptr<qyh_bag_recorder::srv::GetRecordingStatus::Request> /*request*/,
        std::shared_ptr<qyh_bag_recorder::srv::GetRecordingStatus::Response> response)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        response->is_recording = is_recording_;
        response->action_name = current_action_name_;
        response->bag_path = current_bag_path_;
        response->topics = current_topics_;
        
        if (is_recording_) {
            auto duration = this->now() - recording_start_time_;
            response->duration_sec = duration.seconds();
        } else {
            response->duration_sec = last_duration_sec_;
        }
    }

private:
    // 服务
    rclcpp::Service<qyh_bag_recorder::srv::StartRecording>::SharedPtr start_recording_srv_;
    rclcpp::Service<qyh_bag_recorder::srv::StopRecording>::SharedPtr stop_recording_srv_;
    rclcpp::Service<qyh_bag_recorder::srv::GetRecordingStatus>::SharedPtr get_status_srv_;
    
    // Writer
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    
    // 订阅
    std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
    
    // 状态
    std::mutex mutex_;
    bool is_recording_ = false;
    std::string base_path_;
    std::string full_base_path_;  // 包含 robot_name/version
    std::string robot_name_;
    std::string robot_version_;
    std::string current_bag_path_;
    std::string current_action_name_;
    std::string last_bag_path_;
    std::vector<std::string> current_topics_;
    std::map<std::string, std::string> topic_types_;  // topic_name -> topic_type
    rclcpp::Time recording_start_time_;
    double last_duration_sec_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<BagRecorderNode>();
    
    RCLCPP_INFO(node->get_logger(), "Bag Recorder 节点已启动");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
