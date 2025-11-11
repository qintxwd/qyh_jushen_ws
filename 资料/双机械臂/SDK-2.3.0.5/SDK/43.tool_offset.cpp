/*******************************************************************************
 * 文件名: 43.tool_offset.cpp
 * 功能描述: 节卡双臂机器人工具坐标偏置设置示例程序
 * 
 * 主要功能:
 * 1. 演示如何设置机器人末端工具坐标系偏置
 * 2. 展示工具偏置参数的读取和验证
 * 3. 说明工具坐标系对运动控制的影响
 * 
 * 工具坐标系偏置:
 * - 位置偏置(x,y,z): 工具中心点相对于法兰中心的位移(mm)
 * - 姿态偏置(rx,ry,rz): 工具坐标系相对于法兰坐标系的旋转(弧度)
 * 
 * 应用场景:
 * - 安装不同的末端执行器(夹爪、焊枪、喷枪等)
 * - 需要以工具中心点为基准进行运动控制
 * - TCP(Tool Center Point)标定
 * 
 * 重要性:
 * 正确设置工具偏置可以提高末端定位精度
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

    // 【定义工具坐标偏置参数】
    // CartesianPose结构包含位置和姿态信息
    // 左臂工具偏置: 位置(7.1, 8.1, 9)mm, 姿态(10, 11, 12)弧度
    // 右臂工具偏置: 位置(1, 2, 3)mm, 姿态(4.4, 5.4, 6)弧度
    CartesianPose tool_offsets[2] = {  {7.1,8.1,9,10,11,12} , {1,2,3,4.4,5.4,6}};
    
    // 登录机器人
    ret = robot.login_in(IP);
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");


    // 【设置左臂工具偏置】
    ret = robot.robot_set_tool_offset(LEFT, tool_offsets[0]);
    if (ret == ERR_SUCC)
    {
        printf("Set tool succeed!\n");
    }
    else
    {
        printf("Set tool failed ret = !\n", ret);
    }
    
    // 【设置右臂工具偏置】
    ret = robot.robot_set_tool_offset(RIGHT, tool_offsets[1]);
    if (ret == ERR_SUCC)
    {
        printf("Set tool succeed!\n");
    }
    else
    {
        printf("Set tool failed! ret = \n", ret);
    }

    // 等待设置生效
    std::this_thread::sleep_for(std::chrono::milliseconds(20));  

    // 【读取并验证工具偏置设置】
    CartesianPose cur_offset[2];
    // 获取左臂工具偏置
    ret = robot.robot_get_tool_offset(LEFT, cur_offset);
    // 获取右臂工具偏置
    ret = robot.robot_get_tool_offset(RIGHT, cur_offset + 1);

    // 打印当前工具偏置配置
    // 验证设置是否成功
    printf("left tool_offsets = %lf, %lf, %lf, %lf, %lf, %lf\n", cur_offset[0].tran.x, cur_offset[0].tran.y, cur_offset[0].tran.z, cur_offset[0].rpy.rx, cur_offset[0].rpy.ry, cur_offset[0].rpy.rz);
    printf("right tool_offsets = %lf, %lf, %lf, %lf, %lf, %lf\n", cur_offset[1].tran.x, cur_offset[1].tran.y, cur_offset[1].tran.z, cur_offset[1].rpy.rx, cur_offset[1].rpy.ry, cur_offset[1].rpy.rz);

    // 【重置工具偏置为零】
    // 定义零偏置（所有参数为0）
    CartesianPose tool_offsets2[2] = { };
    
    ret = robot.login_in(IP);
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");


    // 设置左臂工具偏置为零
    ret = robot.robot_set_tool_offset(LEFT, tool_offsets2[0]);
    if (ret == ERR_SUCC)
    {
        printf("Set tool succeed!\n");
    }
    
    // 设置右臂工具偏置为零
    ret = robot.robot_set_tool_offset(RIGHT, tool_offsets2[1]);
    if (ret == ERR_SUCC)
    {
        printf("Set tool succeed!\n");
    }


    // 【再次读取工具偏置】
    // 验证重置是否成功
    ret = robot.robot_get_tool_offset(LEFT, cur_offset);
    ret = robot.robot_get_tool_offset(RIGHT, cur_offset + 1);

    // 打印重置后的工具偏置（应该全为0）
    printf("left tool_offsets = %lf, %lf, %lf, %lf, %lf, %lf\n", cur_offset[0].tran.x, cur_offset[0].tran.y, cur_offset[0].tran.z, cur_offset[0].rpy.rx, cur_offset[0].rpy.ry, cur_offset[0].rpy.rz);
    printf("right tool_offsets = %lf, %lf, %lf, %lf, %lf, %lf\n", cur_offset[1].tran.x, cur_offset[1].tran.y, cur_offset[1].tran.z, cur_offset[1].rpy.rx, cur_offset[1].rpy.ry, cur_offset[1].rpy.rz);

    // 登出机器人
    robot.login_out();
    return 0;
}
