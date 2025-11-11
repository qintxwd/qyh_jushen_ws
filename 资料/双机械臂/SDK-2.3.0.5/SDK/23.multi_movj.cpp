/*******************************************************************************
 * 文件名: 23.multi_movj.cpp
 * 功能描述: 节卡双臂机器人多关节运动(multi_movj)示例程序
 * 
 * 主要功能:
 * 1. 演示双臂机器人的关节空间运动控制
 * 2. 展示相对运动(INCR)和绝对运动(ABS)两种运动模式
 * 3. 演示阻塞和非阻塞运动控制方式
 * 
 * 运动说明:
 * - INCR模式: 相对当前位置进行增量运动
 * - ABS模式: 运动到指定的绝对关节位置
 * - 阻塞模式: 等待运动完成后才返回
 * - 非阻塞模式: 发送运动指令后立即返回
 ******************************************************************************/

#include <JAKAZuRobot.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include "common.h"

// 定义圆周率常量，用于角度和弧度转换
#define JK_PI (3.141592653589793)


int main()
{
    // 创建机器人对象，用于控制节卡双臂机器人
    JAKAZuRobot robot;
    RobotStatus robotStatus;  // 机器人状态结构体
    errno_t ret;              // 函数返回值，用于错误检查
    
    // 登录机器人控制器
    // IP定义在common.h中，默认为"10.5.5.100"
    ret = robot.login_in(IP);
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");

    // 获取并打印SDK版本信息
    char version[128];
    robot.get_sdk_version(version);
    std::cout << "version: " << version << std::endl;

    // 给机器人上电
    // 机器人必须先上电才能进行后续操作
    ret = robot.power_on();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "power on");

    // 检查并清除机器人错误
    // 参数true表示需要清除错误
    check_error(robot, true);

    // 使能机器人伺服系统
    // 只有使能后机器人才能运动
    ret = robot.enable_robot();
    if (ret != ERR_SUCC)
    {
        printf("enable failed! ret = %d\n", ret);
    }

    // 检查并打印机器人状态（上电、使能状态）
    check_state(robot);

    // 获取机器人当前状态
    RobotState state;
    ret = robot.get_robot_state(&state);
    // 检查伺服是否使能成功
    if (!state.servoEnabled)
    {
        // 如果使能失败，获取并打印错误信息
        ErrorCode code;
        robot.get_last_error(&code);
        printf("Robot is error! error ocde = %2x, msg = %s\n", code.code, code.message);
        robot.power_off();  // 下电
        exit(1);            // 退出程序
    }


    // 清除可能存在的错误
    ret = robot.clear_error();
    
    // 设置碰撞检测等级
    // 等级0表示关闭碰撞检测，等级越高越灵敏
    // LEFT: 左臂，RIGHT: 右臂
    robot.set_collision_level(LEFT, 0);
    robot.set_collision_level(RIGHT, 0);
    
    // 定义关节增量运动的目标位置（正值）
    // JointValue结构体包含7个关节的角度值（弧度）
    // jstep_pos[0]是左臂的关节增量，jstep_pos[1]是右臂的关节增量
    JointValue jstep_pos[2] { { 0.2, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 }, { 0.2, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 } };
    
    // 定义关节增量运动的目标位置（一正一负）
    JointValue jstep_neg[2] { { -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1 }, { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 } };
    
    // 执行5次非阻塞的相对运动
    for (int i = 0; i < 5; i++)
    {
        // 设置运动模式为增量模式（INCR）
        // INCR表示相对于当前位置进行增量运动
        MoveMode moveop[2] = {INCR, INCR};
        
        // 设置运动速度（单位：弧度/秒）
        double vel[2] = {1, 1};
        
        // 设置运动加速度（单位：弧度/秒²）
        double acc[2] = {1, 1};
        
        // 设置到位误差容限（此处未使用）
        double tol[2] = {0, 0};
        
        // 设置运动ID（此处未使用）
        double id[2] = {0, 0};
        
        // 执行多机器人关节运动
        // 参数说明:
        // -1: 同时控制两个机器人（DUAL模式）
        // moveop: 运动模式数组
        // FALSE: 非阻塞模式，发送指令后立即返回，不等待运动完成
        // jstep_pos: 目标关节位置数组
        // vel: 速度数组
        // acc: 加速度数组
        ret = robot.robot_run_multi_movj(-1, moveop, FALSE, jstep_pos, vel, acc);   // 相对运动
    }
    
    // 等待机器人运动到位
    // 因为上面使用了非阻塞模式，所以需要手动检查是否到位
    while (1)
    {
        // 每500ms检查一次
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // 检查机器人是否到达目标位置
        if (check_inpos(robot))
        {
            break;  // 到位后跳出循环
        }
    }

    // 阻塞模式运动示例
    // 循环执行3次往复运动
    for (int i = 0; i < 3; i++)
    {
        // 设置运动模式为绝对模式（ABS）
        // ABS表示运动到指定的绝对关节位置
        MoveMode moveop[2] = {ABS, ABS};
        
        // 设置较低的速度（0.1弧度/秒）
        double vel[2] = {0.1, 0.1};
        
        // 设置较低的加速度（0.1弧度/秒²）
        double acc[2] = {0.1, 0.1};
        
        double tol[2] = {0, 0};
        double id[2] = {0, 0};
        
        // 执行阻塞运动到jstep_pos位置
        // TRUE: 阻塞模式，等待运动完成后才返回
        ret = robot.robot_run_multi_movj(-1, moveop, TRUE, jstep_pos, vel, acc);
        if (ret != ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move pos failed.\n";
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move pos ok.\n";
        }

        // 执行阻塞运动到jstep_neg位置
        // 实现往复运动效果
        ret = robot.robot_run_multi_movj(-1, moveop, TRUE, jstep_neg, vel, acc);
        if (ret == ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move neg ok.\n";
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move neg failed.\n";
        }
    }

    // 登出机器人控制器，断开连接
    robot.login_out();
    return 0;
}
