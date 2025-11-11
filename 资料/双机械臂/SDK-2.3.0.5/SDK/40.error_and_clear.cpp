/*******************************************************************************
 * 文件名: 40.error_and_clear.cpp
 * 功能描述: 节卡双臂机器人错误检测和清除示例程序
 * 
 * 主要功能:
 * 1. 演示如何检测机器人错误状态
 * 2. 展示如何获取详细的错误信息
 * 3. 说明错误清除的方法
 * 4. 演示碰撞等级的设置和查询
 * 
 * 错误处理流程:
 * 1. 检测是否有错误
 * 2. 获取错误代码和描述信息
 * 3. 清除错误
 * 4. 必要时重新上电/使能
 * 
 * 碰撞等级:
 * 0-5级，级别越高越灵敏
 * 0级表示关闭碰撞检测
 ******************************************************************************/

#include <JAKAZuRobot.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include "common.h"

#define JK_PI (3.141592653589793)
#define deg_tp_rad 1.0 / 180.0 * JK_PI

int main()
{
    // 创建机器人对象
    JAKAZuRobot robot;
    RobotStatus robotStatus;
    errno_t ret;
    
    // 登录机器人
    ret = robot.login_in(IP);
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");

    // 上电
    ret = robot.power_on();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "power on");

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 检查并清除初始错误
    check_error(robot, true);

    // 使能机器人
    ret = robot.enable_robot();
    if (ret != ERR_SUCC)
    {
        printf("enable failed! ret = %d\n", ret);
    }

    // 检查机器人状态
    check_state(robot);

    // 定义一个会导致错误的关节位置
    // 这里故意设置了一个较大的关节角度(20弧度，约1146度)
    // 这会触发运动超限错误，用于演示错误处理
    JointValue jstep_pos[2] { { 20, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 }, { 0.2, 10, 0.1, 0.1, 0.1, 0.1, 0.1 } };
    
    // 尝试执行会产生错误的运动
    for (int i = 0; i < 1; i++)
    {
        MoveMode moveop[2] = {INCR, INCR};  // 增量运动模式
        double vel[2] = {1, 1};
        double acc[2] = {1, 1};
        double tol[2] = {0, 0};
        double id[2] = {0, 0};
        
        // 这个运动指令应该会失败，因为目标位置超出了机器人的运动范围
        ret = robot.robot_run_multi_movj(-1, moveop, ABS, jstep_pos, vel, acc);   // 相对运动
        if (ret != 0)
        {
            printf("run_multi_movj error %d\n", ret);
        }
    }

    // 【获取碰撞等级】
    int level[2] = {};
    // 获取左臂碰撞等级
    ret = robot.get_collision_level(LEFT, level);
    // 获取右臂碰撞等级
    ret = robot.get_collision_level(RIGHT, level + 1);
    if (ret == ERR_SUCC)
    {
        printf("get collision level succeed, level: %d, %d\n", level[0], level[1]);
    }

    // 【设置碰撞等级】
    // 设置左臂碰撞等级为2（中等灵敏度）
    ret = robot.set_collision_level(LEFT, 2);
    if (ret == ERR_SUCC)
    {
        printf("set collision level succeed\n");
    }

    // 设置右臂碰撞等级为2
    ret = robot.set_collision_level(RIGHT, 2);
    if (ret == ERR_SUCC)
    {
        printf("set collision level succeed\n");
    }

    // 【循环监测错误状态】
    // 持续检测机器人是否有错误，并在有错误时清除
    while(1)
    {
        // 每2秒检测一次
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // 检查机器人是否处于错误状态
        // error[0]: 左臂错误状态
        // error[1]: 右臂错误状态
        int error[2] = {-1, -1};
        ret = robot.robot_is_in_error(error);
        if (ret == ERR_SUCC)
        {
            printf("robot has error %s, %s\n", error[0] ? "true" : "false", error[1] ? "true" : "false");
        }
        
        // 获取最后一次错误的详细信息
        ErrorCode code;
        ret = robot.get_last_error(&code);
        
        // 如果有错误
        if (code.code != 0)
        {
            // 打印错误代码和错误消息
            // code.code: 十六进制错误代码
            // code.message: 错误描述信息
            printf("errcode = %x, %s\n", code.code, code.message);
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // 清除错误
            // 注意：某些严重错误（如硬件故障）无法通过clear_error清除
            // 需要重新上下电或手动复位
            robot.clear_error();
        }
        else
        {
            printf("no error\n");
        }
    }

    // 登出机器人
    robot.login_out();
    return 0;
}
