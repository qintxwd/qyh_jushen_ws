/*******************************************************************************
 * 文件名: 26.motion_abort.cpp
 * 功能描述: 节卡双臂机器人运动中止(motion_abort)示例程序
 * 
 * 主要功能:
 * 1. 演示如何在运动过程中中止机器人运动
 * 2. 展示运动状态检测和到位判断
 * 3. 演示非阻塞运动控制的应用场景
 * 
 * 应用场景:
 * - 紧急停止
 * - 条件触发的运动中止
 * - 轨迹切换
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
    ret = robot.enable_robot();
    if (ret != ERR_SUCC)
    {
        printf("enable failed! ret = %d\n", ret);
    }

    // 检查机器人状态
    check_state(robot);

    // 定义起始关节位置和结束关节位置
    // 两个位置之间有较大差异，用于测试运动中止功能
    JointValue start_pos[2] = { { 45 * deg_tp_rad, -10 * deg_tp_rad, 0, -100 * deg_tp_rad, 0, -35 * deg_tp_rad, 0},
                                 { 45 * deg_tp_rad, -10 * deg_tp_rad, 0, -100 * deg_tp_rad, 0, 35 * deg_tp_rad, 0} };
    // end_pos初始化为零位置
    JointValue end_pos[2] = {};

    {
        // 设置运动参数
        MoveMode moveop[2] = {ABS, ABS};  // 绝对位置模式
        double vel[2] = {.5, .5};          // 速度：0.5弧度/秒
        double acc[2] = {1, 1};            // 加速度：1弧度/秒²
        
        // 【第一次运动】：非阻塞模式运动到起始位置
        // FALSE表示非阻塞，函数立即返回，不等待运动完成
        ret = robot.robot_run_multi_movj(-1, moveop, FALSE, start_pos, vel, acc);    // 阻塞等待最多20s
        if (ret != ERR_SUCC)
        {
            printf("Error happends when move to joint_pos. %2x\n", ret);
        }
        else
        {
            // 循环监测运动状态
            while(1)
            {
                // 等待2秒
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                
                // 检查机器人是否到达目标位置
                // inpos[0]表示左臂是否到位，inpos[1]表示右臂是否到位
                int inpos[2] = {};
                robot.robot_is_inpos(inpos);
                printf("robot is inpos %d, %d\n", inpos[0], inpos[1]);
                
                // 如果两个机器人都到位了，退出循环
                if (inpos[0] && inpos[1])
                {
                    printf("robot is inpos\n");
                    break;
                }
                
                // 【运动中止演示】
                // 如果有任意一个机器人未到位，执行运动中止
                // 这里演示的是在运动过程中主动中止运动
                if (!inpos[0] || !inpos[1])
                {
                    // motion_abort()会立即停止机器人运动
                    // 机器人会在当前位置停止，而不是继续运动到目标位置
                    robot.motion_abort();
                }
                
                // 再次检查是否到位
                if (inpos[0] && inpos[1])
                {
                    printf("robot is inpos\n");
                    break;
                }
            }
        }

        // 【第二次运动】：运动到零位置
        // 同样使用非阻塞模式
        ret = robot.robot_run_multi_movj(-1, moveop, FALSE, end_pos, vel, acc);
        
        // 等待运动完成
        while(1)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // 检查是否到位
            int inpos[2] = {};
            robot.robot_is_inpos(inpos);
            printf("robot is inpos %d, %d\n", inpos[0], inpos[1]);
            
            // 两个机器人都到位后退出
            if (inpos[0] && inpos[1])
            {
                printf("robot is inpos\n");
                break;
            }
        }
    }

    // 登出机器人
    robot.login_out();
    return 0;
}
