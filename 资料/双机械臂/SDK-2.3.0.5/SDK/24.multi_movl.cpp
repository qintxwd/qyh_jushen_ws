/*******************************************************************************
 * 文件名: 24.multi_movl.cpp
 * 功能描述: 节卡双臂机器人多笛卡尔空间直线运动(multi_movl)示例程序
 * 
 * 主要功能:
 * 1. 演示双臂机器人的笛卡尔空间直线运动控制
 * 2. 展示如何获取机器人DH参数
 * 3. 演示关节空间与笛卡尔空间的运动配合
 * 4. 展示相对运动(INCR)和绝对运动(ABS)模式
 * 
 * 运动说明:
 * - movl: 末端执行器在笛卡尔空间沿直线运动
 * - 先用movj运动到初始关节位置
 * - 然后用movl进行笛卡尔空间的增量运动
 ******************************************************************************/

#include <JAKAZuRobot.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include "common.h"

// 定义圆周率常量
#define JK_PI (3.141592653589793)
// 定义角度转弧度的转换系数
#define deg_tp_rad 1.0 / 180.0 * JK_PI

int main()
{
    // 创建机器人对象
    JAKAZuRobot robot;
    RobotStatus robotStatus;
    errno_t ret;
    
    // 登录机器人控制器
    ret = robot.login_in(IP);
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");

    // 给机器人上电
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

    // 定义初始关节位置（弧度）
    // 左臂: 关节1=0°, 关节2=100°, 关节3=0°, 关节4=-45°, 关节5=0°, 关节6=-35°, 关节7=0°
    // 右臂: 关节1=0°, 关节2=100°, 关节3=0°, 关节4=-45°, 关节5=0°, 关节6=35°, 关节7=0°
    JointValue start_pos[2] = { { 0, 100 * deg_tp_rad, 0, -45 * deg_tp_rad, 0, -35 * deg_tp_rad, 0},
                                 { 0, 100 * deg_tp_rad, 0, -45 * deg_tp_rad, 0, 35 * deg_tp_rad, 0} };

    // 定义笛卡尔空间的增量位置（正值）
    // CartesianPose包含位置(x,y,z)和姿态(rx,ry,rz)
    // 位置单位：毫米，姿态单位：弧度
    // 左臂增量: x+10mm, y+20mm, z+30mm
    // 右臂增量: x+20mm, y+40mm, z+60mm
    CartesianPose c_pos[2] { { 10, 20, 30, 0, 0, 0 }, { 20, 40, 60, 0, 0, 0 } };
    
    // 定义笛卡尔空间的增量位置（负值）
    // 用于实现往复运动
    CartesianPose c_neg[2] { { -10, -20, -30, 0, 0, 0 }, { -20, -40, -60, 0, 0, 0 } };
    
    // 获取机器人DH参数
    // DH参数(Denavit-Hartenberg)描述了机器人各关节之间的几何关系
    // 包括: alpha(扭转角), d(连杆偏距), a(连杆长度), joint_homeoff(关节零点偏移)
    DHParam dh[2] = {};
    ret = robot.robot_get_multi_robot_dh(dh);
    
    // 打印左右两臂的DH参数
    for (int i = 0; i < 2; i++)
    {
        printf("Robot[%d] : Alpha, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", i, dh[i].alpha[0], dh[i].alpha[1], dh[i].alpha[2], dh[i].alpha[3], dh[i].alpha[4], dh[i].alpha[5], dh[i].alpha[6]);
        printf("Robot[%d] : d, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", i, dh[i].d[0], dh[i].d[1], dh[i].d[2], dh[i].d[3], dh[i].d[4], dh[i].d[5], dh[i].d[6]);
        printf("Robot[%d] : a, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", i, dh[i].a[0], dh[i].a[1], dh[i].a[2], dh[i].a[3], dh[i].a[4], dh[i].a[5], dh[i].a[6]);
        printf("Robot[%d] : joint_homeoff, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", i, dh[i].joint_homeoff[0], dh[i].joint_homeoff[1], dh[i].joint_homeoff[2], dh[i].joint_homeoff[3], dh[i].joint_homeoff[4], dh[i].joint_homeoff[5], dh[i].joint_homeoff[6]);
    }

    {
        // 先用关节运动移动到初始位置
        // 这样可以确保后续的笛卡尔运动有一个已知的起始状态
        MoveMode moveop[2] = {ABS, ABS};  // 绝对位置模式
        double vel[2] = {.5, .5};          // 速度：0.5弧度/秒
        double acc[2] = {1, 1};            // 加速度：1弧度/秒²
        
        // 执行关节运动，TRUE表示阻塞等待运动完成（最多20秒）
        ret = robot.robot_run_multi_movj(-1, moveop, TRUE, start_pos, vel, acc);    // 阻塞等待最多20s
        if (ret != ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            printf("Error happends when move to joint_pos. %2x\n", ret);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "joint_move pos ok.\n";
        }
    }

    // 阻塞模式下的笛卡尔直线运动
    // 循环执行3次往复运动
    for (int i = 0; i < 3; i++)
    {
        // 增量运动模式
        MoveMode moveop[2] = {INCR, INCR};
        
        // 设置笛卡尔空间的运动速度（单位：毫米/秒）
        // 左臂：100mm/s，右臂：20mm/s
        double vel[2] = {100, 20};
        
        // 设置笛卡尔空间的运动加速度（单位：毫米/秒²）
        double acc[2] = {200, 20};
        
        // 执行笛卡尔空间直线运动（增量）
        // movl保证末端执行器在笛卡尔空间走直线
        ret = robot.robot_run_multi_movl(-1, moveop, TRUE, c_pos, vel, acc);
        if (ret != ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "linear move pos failed.\n";
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "linear move pos ok.\n";
        }

        // 执行反向的笛卡尔空间直线运动
        // 实现往复运动效果
        ret = robot.robot_run_multi_movl(-1, moveop, TRUE, c_neg, vel, acc);
        if (ret == ERR_SUCC)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "linear move neg ok.\n";
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::cout << "linear move neg failed.\n";
        }
    }

    // 登出机器人控制器
    robot.login_out();
    return 0;
}
