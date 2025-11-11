/*******************************************************************************
 * 文件名: 41.collision.cpp
 * 功能描述: 节卡双臂机器人碰撞检测等级设置示例程序
 * 
 * 主要功能:
 * 1. 演示如何设置机器人碰撞检测等级
 * 2. 展示碰撞等级的查询方法
 * 3. 说明不同碰撞等级的应用场景
 * 
 * 碰撞等级说明:
 * - 0级: 关闭碰撞检测（适用于高负载、低速场景）
 * - 1级: 最低灵敏度（不易触发）
 * - 2级: 低灵敏度
 * - 3级: 中等灵敏度（推荐）
 * - 4级: 高灵敏度
 * - 5级: 最高灵敏度（易触发）
 * 
 * 注意事项:
 * - 碰撞等级过高可能导致误触发
 * - 碰撞等级过低可能降低安全性
 * - 根据实际应用场景选择合适的等级
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
    
    // 定义初始关节位置
    // 这些位置用于测试，实际使用时可根据需要修改
    JointValue start_pos[2] = { { 0, 90 * deg_tp_rad, 0, -100 * deg_tp_rad, 0, -35 * deg_tp_rad, 0},
                                 { 0, -90 * deg_tp_rad, 0, -100 * deg_tp_rad, 0, 35 * deg_tp_rad, 0} };

    // 定义笛卡尔空间增量位置（未在本示例中使用）
    CartesianPose c_pos[2] { { 0, 0, 30, 0, 0, 0 }, { 0, 0, 60, 0, 0, 0 } };
    
    // 登录机器人
    ret = robot.login_in(IP);
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");


    // 上电
    ret = robot.power_on();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "power on");
    printf("power on succeed\n");

    // 使能机器人
    ret = robot.enable_robot();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "enable robot");
    printf("enable succeed\n");


    // 【左臂碰撞等级测试】
    // 从0级到5级依次设置并验证
    for (int i = 0; i < 6; i++)
    {
        // 设置左臂碰撞等级
        ret = robot.set_collision_level(LEFT, i);
        if (ret == ERR_SUCC)
        {
            printf("set collision level succeed\n");
        }
        
        // 等待1秒，让设置生效
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 读取并验证碰撞等级设置
        int level = -1;
        ret = robot.get_collision_level(LEFT, &level);
        if (ret == ERR_SUCC)
        {
            printf("get collision level succeed, level: %d\n", level);
        }
    }

    // 【右臂碰撞等级测试】
    // 从5级到0级依次设置并验证（倒序）
    for (int i = 5; i > -1; i--)
    {
        // 设置右臂碰撞等级
        ret = robot.set_collision_level(RIGHT, i);
        if (ret == ERR_SUCC)
        {
            printf("set collision level succeed\n");
        }
        
        // 等待1秒，让设置生效
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 读取并验证碰撞等级设置
        int level = -1;
        ret = robot.get_collision_level(RIGHT, &level);
        if (ret == ERR_SUCC)
        {
            printf("get collision level succeed, level: %d\n", level);
        }
    }

    // 登出机器人
    robot.login_out();
    return 0;
}
