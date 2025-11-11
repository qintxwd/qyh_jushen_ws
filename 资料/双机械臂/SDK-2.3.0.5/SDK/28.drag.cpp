/*******************************************************************************
 * 文件名: 28.drag.cpp
 * 功能描述: 节卡双臂机器人拖动示教模式示例程序
 * 
 * 主要功能:
 * 1. 演示如何开启和关闭拖动示教模式
 * 2. 展示单臂和双臂拖动模式的切换
 * 3. 说明拖动模式状态的查询方法
 * 
 * 拖动示教模式:
 * - 使能后，用户可以手动拖动机器人末端
 * - 机器人会记录被拖动的轨迹
 * - 常用于示教编程和位置记录
 * 
 * 安全提示:
 * - 拖动前确保工作空间安全
 * - 拖动时注意机器人的运动范围
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
    errno_t ret;
    
    // 登录机器人
    ret = robot.login_in(IP);
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");

    // 上电
    ret = robot.power_on();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "power on");

    // 检查并清除错误
    check_error(robot, true);

    // 使能机器人
    // 拖动模式需要机器人处于使能状态
    ret = robot.enable_robot();
    if (ret != ERR_SUCC)
    {
        printf("enable failed! ret = %d\n", ret);
    }

    // 检查机器人状态
    check_state(robot);

    // 【使能左臂拖动模式】
    // 参数说明:
    // - LEFT: 指定左臂
    // - 1: 使能拖动模式
    ret = robot.drag_mode_enable(LEFT, 1);
    if (ret != ERR_SUCC)
    {
        std::cout << "left robot drag mode enable failed" << std::endl;
    }

    // 定义拖动状态数组
    // is_drag[0]: 左臂拖动状态
    // is_drag[1]: 右臂拖动状态
    BOOL is_drag[2];
    
    // 循环20次，每次间隔1秒
    // 演示拖动模式的状态查询和动态切换
    for(int i = 0; i < 20; i++)
    {
        // 在第10次循环后，使能右臂拖动模式
        // 演示双臂同时拖动的场景
        if (i > 10)
        {
            ret = robot.drag_mode_enable(RIGHT, 1);
        }
        
        // 等待1秒
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 查询两个机器人的拖动模式状态
        ret = robot.is_in_drag_mode(is_drag);
        if (ret != ERR_SUCC)
        {
            std::cout << "get drag mode failed" << std::endl;
            break;
        }
        
        // 打印拖动状态
        // 1表示处于拖动模式，0表示未处于拖动模式
        // 在此期间，用户可以手动拖动机器人末端
        std::cout << "left robot is in drag: " <<  is_drag[0] << " right robot is in drag: " << is_drag[1] << std::endl;
    }
    
    // 【关闭拖动模式】
    // 关闭左臂拖动模式
    ret = robot.drag_mode_enable(LEFT, 0);
    // 关闭右臂拖动模式
    ret = robot.drag_mode_enable(RIGHT, 0);
    if (ret != ERR_SUCC)
    {
        std::cout << "left robot drag mode disable failed" << std::endl;
    }
    
    // 等待1秒后再次查询状态，验证拖动模式已关闭
    std::this_thread::sleep_for(std::chrono::seconds(1));
    robot.is_in_drag_mode(is_drag);
    std::cout << "left robot is in drag: " <<  is_drag[0] << " right robot is in drag: " << is_drag[1] << std::endl;

    // 登出机器人
    robot.login_out();
    return 0;
}
