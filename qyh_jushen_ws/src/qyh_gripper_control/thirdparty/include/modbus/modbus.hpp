/*
 * libmodbus C++ API
 * Copyright © 2025
 * SPDX-License-Identifier: LGPL-2.1-or-later
 * 
 * 纯 C++ 接口 - 完全隐藏底层 C 实现
 * 用户只需包含此头文件，无需任何其他依赖
 */

#ifndef MODBUS_HPP
#define MODBUS_HPP

#include <string>
#include <vector>
#include <stdexcept>
#include <memory>
#include <cstdint>

namespace modbus {

// ============================================================================
// 常量定义
// ============================================================================

// 默认端口
constexpr int DEFAULT_TCP_PORT = 502;

// Modbus 功能码
constexpr int FC_READ_COILS = 0x01;
constexpr int FC_READ_DISCRETE_INPUTS = 0x02;
constexpr int FC_READ_HOLDING_REGISTERS = 0x03;
constexpr int FC_READ_INPUT_REGISTERS = 0x04;
constexpr int FC_WRITE_SINGLE_COIL = 0x05;
constexpr int FC_WRITE_SINGLE_REGISTER = 0x06;
constexpr int FC_WRITE_MULTIPLE_COILS = 0x0F;
constexpr int FC_WRITE_MULTIPLE_REGISTERS = 0x10;
constexpr int FC_WRITE_AND_READ_REGISTERS = 0x17;

// 最大数量限制
constexpr int MAX_READ_BITS = 2000;
constexpr int MAX_WRITE_BITS = 1968;
constexpr int MAX_READ_REGISTERS = 125;
constexpr int MAX_WRITE_REGISTERS = 123;

// RTU 串口模式
constexpr int RTU_RS232 = 0;
constexpr int RTU_RS485 = 1;

// RTU RTS 模式
constexpr int RTS_NONE = 0;
constexpr int RTS_UP = 1;
constexpr int RTS_DOWN = 2;

// ============================================================================
// 前向声明 - 隐藏实现细节
// ============================================================================

class ModbusImpl;
class MappingImpl;

/**
 * @brief Modbus 异常类
 */
class Exception : public std::runtime_error {
public:
    explicit Exception(const std::string& message);
    explicit Exception(const std::string& message, int error_code);
    
    int error_code() const noexcept;
    
private:
    int error_code_;
};

/**
 * @brief Modbus 基类 - 使用 RAII 管理资源
 * 采用 pimpl 模式隐藏实现细节
 */
class Modbus {
public:
    virtual ~Modbus();

    // 禁止拷贝
    Modbus(const Modbus&) = delete;
    Modbus& operator=(const Modbus&) = delete;

    // 允许移动
    Modbus(Modbus&& other) noexcept;
    Modbus& operator=(Modbus&& other) noexcept;

    /**
     * @brief 连接到 Modbus 设备
     */
    void connect();

    /**
     * @brief 关闭连接
     */
    void close();

    /**
     * @brief 设置从站地址
     */
    void set_slave(int slave);

    /**
     * @brief 设置调试模式
     */
    void set_debug(bool on);

    /**
     * @brief 设置响应超时（秒, 微秒）
     */
    void set_response_timeout(uint32_t sec, uint32_t usec);

    /**
     * @brief 读取线圈 (Coils - Function Code 1)
     * @param addr 起始地址
     * @param nb 数量
     * @return 线圈状态数组
     */
    std::vector<uint8_t> read_coils(int addr, int nb);

    /**
     * @brief 读取离散输入 (Discrete Inputs - Function Code 2)
     */
    std::vector<uint8_t> read_discrete_inputs(int addr, int nb);

    /**
     * @brief 读取保持寄存器 (Holding Registers - Function Code 3)
     */
    std::vector<uint16_t> read_holding_registers(int addr, int nb);

    /**
     * @brief 读取输入寄存器 (Input Registers - Function Code 4)
     */
    std::vector<uint16_t> read_input_registers(int addr, int nb);

    /**
     * @brief 写单个线圈 (Function Code 5)
     */
    void write_coil(int addr, bool status);

    /**
     * @brief 写单个寄存器 (Function Code 6)
     */
    void write_register(int addr, uint16_t value);

    /**
     * @brief 写多个线圈 (Function Code 15)
     */
    void write_coils(int addr, const std::vector<uint8_t>& src);

    /**
     * @brief 写多个寄存器 (Function Code 16)
     */
    void write_registers(int addr, const std::vector<uint16_t>& src);

    /**
     * @brief 读写寄存器 (Function Code 23)
     */
    std::vector<uint16_t> write_and_read_registers(
        int write_addr, const std::vector<uint16_t>& src,
        int read_addr, int read_nb);

protected:
    explicit Modbus(std::unique_ptr<ModbusImpl> impl);
    
    // pimpl 指针
    std::unique_ptr<ModbusImpl> impl_;
    
    friend class ModbusTCPServer;
};

/**
 * @brief Modbus TCP 客户端
 */
class ModbusTCP : public Modbus {
public:
    /**
     * @brief 创建 TCP 客户端
     * @param ip IP 地址
     * @param port 端口号（默认 502）
     */
    explicit ModbusTCP(const std::string& ip, int port = DEFAULT_TCP_PORT);
    
    virtual ~ModbusTCP() = default;
};

/**
 * @brief Modbus TCP 服务器
 */
class ModbusTCPServer {
public:
    /**
     * @brief 创建 TCP 服务器
     * @param ip 绑定的 IP 地址（默认 "0.0.0.0"）
     * @param port 端口号（默认 502）
     */
    explicit ModbusTCPServer(const std::string& ip = "0.0.0.0", int port = DEFAULT_TCP_PORT);
    
    ~ModbusTCPServer();

    // 禁止拷贝和移动
    ModbusTCPServer(const ModbusTCPServer&) = delete;
    ModbusTCPServer& operator=(const ModbusTCPServer&) = delete;

    /**
     * @brief 监听连接
     * @param nb_connection 最大连接数（默认 1）
     */
    void listen(int nb_connection = 1);

    /**
     * @brief 接受客户端连接
     * @return 返回一个连接的客户端 Modbus 对象
     */
    std::unique_ptr<Modbus> accept();

    /**
     * @brief 数据映射 - 存储 Modbus 寄存器和线圈数据
     */
    class Mapping {
    public:
        explicit Mapping(int nb_bits = 500, int nb_input_bits = 500,
                        int nb_registers = 500, int nb_input_registers = 500);
        ~Mapping();

        // 禁止拷贝
        Mapping(const Mapping&) = delete;
        Mapping& operator=(const Mapping&) = delete;

        // 访问器
        uint8_t& coil(int addr);
        uint8_t& discrete_input(int addr);
        uint16_t& holding_register(int addr);
        uint16_t& input_register(int addr);

    private:
        friend class ModbusTCPServer;
        std::unique_ptr<MappingImpl> impl_;
    };

    /**
     * @brief 接收并回复请求
     * @return 接收到的字节数，-1 表示连接关闭
     */
    int receive_and_reply(Modbus& client, Mapping& mapping);

private:
    std::unique_ptr<ModbusImpl> impl_;
};

/**
 * @brief Modbus RTU 客户端
 */
class ModbusRTU : public Modbus {
public:
    /**
     * @brief 创建 RTU 客户端
     * @param device 串口设备名称 (Windows: "COM1", Linux: "/dev/ttyUSB0")
     * @param baud 波特率
     * @param parity 校验位 ('N'=无, 'E'=偶, 'O'=奇)
     * @param data_bit 数据位 (5, 6, 7, 8)
     * @param stop_bit 停止位 (1, 2)
     */
    explicit ModbusRTU(const std::string& device, int baud,
                       char parity = 'N', int data_bit = 8, int stop_bit = 1);
    
    virtual ~ModbusRTU() = default;

    /**
     * @brief 设置串口模式
     */
    void set_serial_mode(int mode);

    /**
     * @brief 设置 RTS 模式
     */
    void set_rts(int mode);
};

/**
 * @brief 获取库版本字符串
 */
std::string version();

/**
 * @brief 获取主版本号
 */
int version_major();

/**
 * @brief 获取次版本号
 */
int version_minor();

/**
 * @brief 获取修订版本号
 */
int version_micro();

} // namespace modbus

#endif // MODBUS_HPP
