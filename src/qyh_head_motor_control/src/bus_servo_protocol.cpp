#include "qyh_head_motor_control/bus_servo_protocol.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <iostream>

namespace qyh_head_motor_control
{

// CRC8-MAXIM 计算 (polynomial 0x31反转=0x8C, init 0x00, RefIn=True, RefOut=True)
// 使用非查表实现，与Python调试代码保持100%一致
uint8_t crc8_maxim(const uint8_t* data, size_t len)
{
    uint8_t crc = 0x00;  // 初始值必须为 0x00 (已通过Python调试验证)
    uint8_t polynomial = 0x8C;  // 0x31 的反转值
    
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ polynomial;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;  // 不需要取反
}

BusServoProtocol::BusServoProtocol(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate), fd_(-1)
{
}

BusServoProtocol::~BusServoProtocol()
{
    close();
}

bool BusServoProtocol::open()
{
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "Failed to open serial port: " << port_ << std::endl;
        return false;
    }
    
    // 保存旧配置
    tcgetattr(fd_, &old_tio_);
    
    // 新配置
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    
    // 波特率
    speed_t speed;
    switch (baudrate_) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 500000: speed = B500000; break;
        case 576000: speed = B576000; break;
        case 921600: speed = B921600; break;
        case 1000000: speed = B1000000; break;
        case 1152000: speed = B1152000; break;
        case 1500000: speed = B1500000; break;
        case 2000000: speed = B2000000; break;
        default:
            std::cerr << "Unsupported baudrate: " << baudrate_ << std::endl;
            ::close(fd_);
            fd_ = -1;
            return false;
    }
    
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    
    // 8N1, no flow control
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;
    
    // Raw input
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    
    // Raw output
    tio.c_oflag &= ~OPOST;
    
    // Timeout settings
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;  // 100ms timeout
    
    tcflush(fd_, TCIOFLUSH);
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        std::cerr << "Failed to set serial port attributes" << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    
    return true;
}

void BusServoProtocol::close()
{
    if (fd_ >= 0) {
        tcsetattr(fd_, TCSANOW, &old_tio_);
        ::close(fd_);
        fd_ = -1;
    }
}

std::vector<uint8_t> BusServoProtocol::buildFrame(uint8_t func, const std::vector<uint8_t>& data)
{
    std::vector<uint8_t> frame;
    frame.push_back(0xAA);  // Header 1
    frame.push_back(0x55);  // Header 2
    frame.push_back(func);  // Function code
    frame.push_back(static_cast<uint8_t>(data.size()));  // Data length
    frame.insert(frame.end(), data.begin(), data.end());  // Data
    
    // CRC (从 func 开始计算)
    uint8_t crc = crc8_maxim(&frame[2], frame.size() - 2);
    frame.push_back(crc);
    
    return frame;
}

bool BusServoProtocol::sendFrame(uint8_t func, const std::vector<uint8_t>& data)
{
    if (fd_ < 0) {
        return false;
    }
    
    std::vector<uint8_t> frame = buildFrame(func, data);
    
    ssize_t written = write(fd_, frame.data(), frame.size());
    if (written != static_cast<ssize_t>(frame.size())) {
        std::cerr << "Failed to write to serial port" << std::endl;
        return false;
    }
    
    tcdrain(fd_);  // 等待发送完成
    return true;
}

bool BusServoProtocol::recvFrame(std::vector<uint8_t>& data, int timeout_ms)
{
    if (fd_ < 0) {
        return false;
    }
    
    fd_set readfds;
    struct timeval tv;
    
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);
    
    int ret = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
    if (ret <= 0) {
        return false;  // Timeout or error
    }
    
    // 读取帧头
    uint8_t header[4];
    ssize_t n = read(fd_, header, 4);
    if (n != 4 || header[0] != 0xAA || header[1] != 0x55) {
        return false;
    }
    
    uint8_t func = header[2];
    uint8_t len = header[3];
    
    // 读取数据和CRC
    std::vector<uint8_t> buffer(len + 1);
    n = read(fd_, buffer.data(), len + 1);
    if (n != static_cast<ssize_t>(len + 1)) {
        return false;
    }
    
    // 验证CRC
    std::vector<uint8_t> crc_data;
    crc_data.push_back(func);
    crc_data.push_back(len);
    crc_data.insert(crc_data.end(), buffer.begin(), buffer.begin() + len);
    uint8_t crc = crc8_maxim(crc_data.data(), crc_data.size());
    
    if (crc != buffer[len]) {
        std::cerr << "CRC mismatch" << std::endl;
        return false;
    }
    
    data.assign(buffer.begin(), buffer.begin() + len);
    return true;
}

bool BusServoProtocol::setPosition(uint8_t servo_id, uint16_t position, uint16_t duration_ms)
{
    // 限制位置范围
    if (position > 1000) {
        position = 1000;
    }
    
    std::vector<uint8_t> data;
    data.push_back(CMD_SET_POSITION);  // 子命令 0x01
    // 协议要求: 先放时间(2字节), 再放数量(1字节)
    data.push_back(static_cast<uint8_t>(duration_ms & 0xFF));         // 时间低字节
    data.push_back(static_cast<uint8_t>((duration_ms >> 8) & 0xFF));  // 时间高字节
    data.push_back(1);                  // 舵机数量
    data.push_back(servo_id);           // 舵机ID
    data.push_back(static_cast<uint8_t>(position & 0xFF));            // 位置低字节
    data.push_back(static_cast<uint8_t>((position >> 8) & 0xFF));     // 位置高字节
    
    return sendFrame(FUNC_BUS_SERVO, data);
}

bool BusServoProtocol::setPositions(const std::vector<uint8_t>& ids, 
                                     const std::vector<uint16_t>& positions,
                                     uint16_t duration_ms)
{
    if (ids.size() != positions.size() || ids.empty()) {
        return false;
    }
    
    std::vector<uint8_t> data;
    data.push_back(CMD_SET_POSITION);  // 子命令 0x01
    // 协议要求: 先放时间(2字节), 再放数量(1字节)
    data.push_back(static_cast<uint8_t>(duration_ms & 0xFF));         // 时间低字节
    data.push_back(static_cast<uint8_t>((duration_ms >> 8) & 0xFF));  // 时间高字节
    data.push_back(static_cast<uint8_t>(ids.size()));  // 舵机数量
    
    for (size_t i = 0; i < ids.size(); i++) {
        uint16_t pos = positions[i];
        if (pos > 1000) {
            pos = 1000;
        }
        data.push_back(ids[i]);
        data.push_back(static_cast<uint8_t>(pos & 0xFF));
        data.push_back(static_cast<uint8_t>((pos >> 8) & 0xFF));
    }
    
    return sendFrame(FUNC_BUS_SERVO, data);
}

bool BusServoProtocol::readPosition(uint8_t servo_id, uint16_t& position)
{
    std::vector<uint8_t> data;
    data.push_back(CMD_READ_POSITION);  // 0x05
    data.push_back(servo_id);  // 协议只需要子命令+ID，无需数量参数
    
    if (!sendFrame(FUNC_BUS_SERVO, data)) {
        return false;
    }
    
    std::vector<uint8_t> response;
    if (!recvFrame(response, 100)) {
        return false;
    }
    
    // 回包结构: [ID, SubCmd(0x05), Status, PosLo, PosHi]
    // response[0] 是 ID，response[1] 才是命令字
    if (response.size() >= 5 && response[1] == CMD_READ_POSITION) {
        // 还可以验证 response[0] == servo_id 和 response[2] == 0 (Status)
        position = response[3] | (response[4] << 8);
        return true;
    }
    
    return false;
}

bool BusServoProtocol::setServoId(uint8_t old_id, uint8_t new_id)
{
    std::vector<uint8_t> data;
    data.push_back(CMD_SET_ID);
    data.push_back(old_id);
    data.push_back(new_id);
    
    return sendFrame(FUNC_BUS_SERVO, data);
}

bool BusServoProtocol::readServoId(uint8_t& id)
{
    std::vector<uint8_t> data;
    data.push_back(CMD_READ_ID);  // 0x12
    data.push_back(0xFE);  // 广播参数（协议要求）
    
    if (!sendFrame(FUNC_BUS_SERVO, data)) {
        return false;
    }
    
    std::vector<uint8_t> response;
    if (!recvFrame(response, 100)) {
        return false;
    }
    
    // 回包结构: [0xFE, SubCmd(0x12), Status, ID]
    // response[1] 是命令字，response[3] 是实际ID
    if (response.size() >= 4 && response[1] == CMD_READ_ID) {
        id = response[3];
        return true;
    }
    
    return false;
}

bool BusServoProtocol::enableTorque(uint8_t servo_id, bool enable)
{
    std::vector<uint8_t> data;
    // 注意: 上电(使能) = CMD_DISABLE_TORQUE (0x0C), 掉电(失能) = CMD_ENABLE_TORQUE (0x0B)
    // 这个命名有点反直觉,但和原始协议一致
    data.push_back(enable ? CMD_DISABLE_TORQUE : CMD_ENABLE_TORQUE);
    data.push_back(servo_id);  // 协议只需要子命令+ID，无需数量参数
    
    return sendFrame(FUNC_BUS_SERVO, data);
}

bool BusServoProtocol::setAngleLimit(uint8_t servo_id, uint16_t min_pos, uint16_t max_pos)
{
    if (min_pos > 1000) min_pos = 1000;
    if (max_pos > 1000) max_pos = 1000;
    
    std::vector<uint8_t> data;
    data.push_back(CMD_SET_ANGLE_LIMIT);
    data.push_back(servo_id);
    data.push_back(static_cast<uint8_t>(min_pos & 0xFF));
    data.push_back(static_cast<uint8_t>((min_pos >> 8) & 0xFF));
    data.push_back(static_cast<uint8_t>(max_pos & 0xFF));
    data.push_back(static_cast<uint8_t>((max_pos >> 8) & 0xFF));
    
    return sendFrame(FUNC_BUS_SERVO, data);
}

bool BusServoProtocol::setOffset(uint8_t servo_id, int8_t offset)
{
    std::vector<uint8_t> data;
    data.push_back(CMD_SET_OFFSET);
    data.push_back(servo_id);
    data.push_back(static_cast<uint8_t>(offset));  // 有符号转无符号
    
    return sendFrame(FUNC_BUS_SERVO, data);
}

bool BusServoProtocol::saveOffset(uint8_t servo_id)
{
    std::vector<uint8_t> data;
    data.push_back(CMD_SAVE_OFFSET);
    data.push_back(servo_id);
    
    return sendFrame(FUNC_BUS_SERVO, data);
}

}  // namespace qyh_head_motor_control
