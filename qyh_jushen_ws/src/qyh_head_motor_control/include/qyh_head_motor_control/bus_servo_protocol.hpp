#ifndef QYH_HEAD_MOTOR_CONTROL__BUS_SERVO_PROTOCOL_HPP_
#define QYH_HEAD_MOTOR_CONTROL__BUS_SERVO_PROTOCOL_HPP_

#include <cstdint>
#include <vector>
#include <string>
#include <termios.h>

namespace qyh_head_motor_control
{

/**
 * @brief CRC8-MAXIM 校验计算
 */
uint8_t crc8_maxim(const uint8_t* data, size_t len);

/**
 * @brief Bus Servo 通信协议类
 * 
 * 支持功能:
 * - 位置控制 (脉宽 0-1000)
 * - ID 设置/读取
 * - 位置限位设置
 * - 偏差设置
 * - 上电/掉电
 */
class BusServoProtocol
{
public:
    // 功能码
    static constexpr uint8_t FUNC_BUS_SERVO = 0x05;
    
    // 子命令
    static constexpr uint8_t CMD_SET_POSITION = 0x01;      // 位置控制
    static constexpr uint8_t CMD_STOP = 0x03;              // 停止
    static constexpr uint8_t CMD_READ_POSITION = 0x05;     // 读取位置
    static constexpr uint8_t CMD_ENABLE_TORQUE = 0x0B;     // 掉电(失能)
    static constexpr uint8_t CMD_DISABLE_TORQUE = 0x0C;    // 上电(使能)
    static constexpr uint8_t CMD_SET_ID = 0x10;            // 设置ID
    static constexpr uint8_t CMD_READ_ID = 0x12;           // 读取ID
    static constexpr uint8_t CMD_SET_OFFSET = 0x20;        // 设置偏差
    static constexpr uint8_t CMD_SAVE_OFFSET = 0x24;       // 保存偏差
    static constexpr uint8_t CMD_SET_ANGLE_LIMIT = 0x30;   // 设置角度限位
    static constexpr uint8_t CMD_READ_ANGLE_LIMIT = 0x32;  // 读取角度限位

    /**
     * @brief 构造函数
     * @param port 串口设备路径
     * @param baudrate 波特率
     */
    BusServoProtocol(const std::string& port, int baudrate = 1000000);
    
    /**
     * @brief 析构函数
     */
    ~BusServoProtocol();
    
    /**
     * @brief 打开串口
     * @return 是否成功
     */
    bool open();
    
    /**
     * @brief 关闭串口
     */
    void close();
    
    /**
     * @brief 检查串口是否打开
     */
    bool isOpen() const { return fd_ >= 0; }
    
    /**
     * @brief 设置舵机位置
     * @param servo_id 舵机ID
     * @param position 位置 (0-1000)
     * @param duration_ms 运动时间 (ms)
     * @return 是否成功
     */
    bool setPosition(uint8_t servo_id, uint16_t position, uint16_t duration_ms = 100);
    
    /**
     * @brief 设置多个舵机位置
     * @param ids 舵机ID列表
     * @param positions 位置列表 (0-1000)
     * @param duration_ms 运动时间 (ms)
     * @return 是否成功
     */
    bool setPositions(const std::vector<uint8_t>& ids, 
                      const std::vector<uint16_t>& positions,
                      uint16_t duration_ms = 100);
    
    /**
     * @brief 读取舵机位置
     * @param servo_id 舵机ID
     * @param position 输出位置
     * @return 是否成功
     */
    bool readPosition(uint8_t servo_id, uint16_t& position);
    
    /**
     * @brief 设置舵机ID
     * @param old_id 旧ID (254=广播)
     * @param new_id 新ID
     * @return 是否成功
     */
    bool setServoId(uint8_t old_id, uint8_t new_id);
    
    /**
     * @brief 读取舵机ID (广播方式，只能连接一个舵机)
     * @param id 输出ID
     * @return 是否成功
     */
    bool readServoId(uint8_t& id);
    
    /**
     * @brief 使能舵机扭矩
     * @param servo_id 舵机ID
     * @param enable true=上电, false=掉电
     * @return 是否成功
     */
    bool enableTorque(uint8_t servo_id, bool enable);
    
    /**
     * @brief 设置位置限位
     * @param servo_id 舵机ID
     * @param min_pos 最小位置 (0-1000)
     * @param max_pos 最大位置 (0-1000)
     * @return 是否成功
     */
    bool setAngleLimit(uint8_t servo_id, uint16_t min_pos, uint16_t max_pos);
    
    /**
     * @brief 设置偏差
     * @param servo_id 舵机ID
     * @param offset 偏差 (-100 ~ +100)
     * @return 是否成功
     */
    bool setOffset(uint8_t servo_id, int8_t offset);
    
    /**
     * @brief 保存偏差到EEPROM
     * @param servo_id 舵机ID
     * @return 是否成功
     */
    bool saveOffset(uint8_t servo_id);

private:
    /**
     * @brief 发送数据帧
     */
    bool sendFrame(uint8_t func, const std::vector<uint8_t>& data);
    
    /**
     * @brief 接收数据帧
     */
    bool recvFrame(std::vector<uint8_t>& data, int timeout_ms = 100);
    
    /**
     * @brief 构建数据帧
     */
    std::vector<uint8_t> buildFrame(uint8_t func, const std::vector<uint8_t>& data);

    std::string port_;
    int baudrate_;
    int fd_;
    struct termios old_tio_;
};

}  // namespace qyh_head_motor_control

#endif  // QYH_HEAD_MOTOR_CONTROL__BUS_SERVO_PROTOCOL_HPP_
