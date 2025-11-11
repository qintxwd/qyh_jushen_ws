/*******************************************************************************
 * 文件名: 27.payload.cpp
 * 功能描述: 节卡双臂机器人末端负载设置示例程序
 * 
 * 主要功能:
 * 1. 演示如何设置机器人末端负载参数
 * 2. 展示如何获取当前的负载配置
 * 3. 说明负载参数对机器人性能的影响
 * 
 * 负载参数说明:
 * - mass: 负载质量(kg)
 * - centroid: 负载质心位置(x,y,z)，相对于工具坐标系(mm)
 * 
 * 重要性:
 * 正确设置负载参数可以提高机器人运动精度和安全性
 * 负载参数影响动力学计算和碰撞检测
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
    
    // 登录机器人（无需上电和使能，负载参数可以随时设置）
    errno_t ret = robot.login_in(IP);

    // 定义左臂和右臂的负载参数
    // PayLoad结构体包含：
    // - mass: 负载质量(kg)
    // - centroid: 质心位置，包含x,y,z坐标(mm)
    // 左臂负载: 质量5kg，质心位置(60, -60, 20)mm
    PayLoad left = {5, 60, -60, 20};
    // 右臂负载: 无负载（质量0kg，质心在原点）
    PayLoad right = {0,0,0,0};
    
    // 设置左臂的末端负载
    ret = robot.robot_set_tool_payload(LEFT, &left);
    if (ret != ERR_SUCC)
    {
        printf("set left payload failed\n");
    } 
    
    // 设置右臂的末端负载
    ret = robot.robot_set_tool_payload(RIGHT, &right);
    if (ret != ERR_SUCC)
    {
        printf("set right payload failed\n");
    }
    
    // 等待设置生效
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 读取并验证负载设置
    PayLoad res[2];
    ret = robot.robot_get_tool_payload(res);
    if (ret != ERR_SUCC)
    {
        printf("get right payload failed\n");
    }
    
    // 打印当前的负载配置
    // 验证设置是否成功
    printf("left: %f %f %f , Mass = %f\n", res[0].centroid.x, res[0].centroid.y, res[0].centroid.z, res[0].mass);
    printf("right: %f %f %f Mass = %f\n", res[1].centroid.x, res[1].centroid.y, res[1].centroid.z, res[1].mass);

    // 登出机器人
    robot.login_out();
    return 0;
}
