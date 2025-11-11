/*******************************************************************************
 * 文件名: 25.kine.cpp
 * 功能描述: 节卡双臂机器人运动学正解和逆解示例程序
 * 
 * 主要功能:
 * 1. 演示正运动学(Forward Kinematics): 从关节角度计算末端位姿
 * 2. 演示逆运动学(Inverse Kinematics): 从末端位姿计算关节角度
 * 3. 展示运动学计算在实际运动控制中的应用
 * 
 * 运动学说明:
 * - 正运动学: kine_forward() - 已知关节角度，求末端位姿
 * - 逆运动学: kine_inverse() - 已知末端位姿，求关节角度
 * - 逆运动学可能有多组解，需要提供参考关节位置来选择最优解
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

    // 定义两个测试用的关节位置
    // 左臂: 所有关节90度（除了关节4和5为-90度和90度）
    // 右臂: 各关节角度不同，用于测试不同的机器人姿态
    JointValue start_pos[2] = { { 90 * deg_tp_rad, 90 * deg_tp_rad, 90 * deg_tp_rad, -90 * deg_tp_rad, 90 * deg_tp_rad, 90 * deg_tp_rad, 90 * deg_tp_rad},
                                 { 90 * deg_tp_rad, -45 * deg_tp_rad, 0, -100 * deg_tp_rad, 0, -35 * deg_tp_rad, 90 * deg_tp_rad} };    
    
    // 定义笛卡尔位姿数组，用于存储正运动学计算结果
    CartesianPose pos[2];
    
    // 【正运动学计算】
    // 根据给定的关节角度，计算末端执行器的笛卡尔位姿
    // LEFT: 计算左臂的末端位姿
    robot.kine_forward(LEFT, &start_pos[0], &pos[0]);
    // RIGHT: 计算右臂的末端位姿
    robot.kine_forward(RIGHT, &start_pos[1], &pos[1]);
    
    // 打印正运动学计算结果
    // 位置(x,y,z)单位：毫米，姿态(rx,ry,rz)单位：弧度
    printf("left pos = %lf, %lf, %lf, %lf, %lf, %lf\n", pos[0].tran.x, pos[0].tran.y, pos[0].tran.z, pos[0].rpy.rx, pos[0].rpy.ry, pos[0].rpy.rz);
    printf("right pos = %lf, %lf, %lf, %lf, %lf, %lf\n", pos[1].tran.x, pos[1].tran.y, pos[1].tran.z, pos[1].rpy.rx, pos[1].rpy.ry, pos[1].rpy.rz);

    // 设置运动参数
    MoveMode moveop[2] = {ABS, ABS};  // 绝对位置模式
    double vel[2] = {1, 1};            // 速度：1弧度/秒
    double acc[2] = {1, 1};            // 加速度：1弧度/秒²
    double tol[2] = {0, 0};
    double id[2] = {0, 0};
    
    // 运动到起始关节位置
    // 这一步是为了让机器人实际移动到我们计算的位置
    ret = robot.robot_run_multi_movj(-1, moveop, TRUE, start_pos, vel, acc);

    // 定义用于存储逆运动学结果的关节位置
    JointValue end_pos[2];
    
    // 修改目标位姿：在xyz方向各增加20mm
    // 这样可以测试逆运动学能否计算出到达新位置的关节角度
    pos[0].tran.x += 20;
    pos[0].tran.y += 20;
    pos[0].tran.z += 20;
    pos[1].tran.x += 20;
    pos[1].tran.y += 20;
    pos[1].tran.z += 20;
    
    // 【逆运动学计算】
    // 根据期望的末端位姿，计算需要的关节角度
    // 参数说明:
    // - start_pos: 参考关节位置，用于选择多组解中最接近的一组
    // - pos: 期望的末端笛卡尔位姿
    // - end_pos: 输出计算得到的关节角度
    robot.kine_inverse(LEFT, &start_pos[0], &pos[0], &end_pos[0]);
    robot.kine_inverse(RIGHT, &start_pos[1], &pos[1], &end_pos[1]);

    // 运动到逆运动学计算出的关节位置
    // 验证逆运动学计算是否正确
    ret = robot.robot_run_multi_movj(-1, moveop, TRUE, end_pos, vel, acc);

    // 打印逆运动学计算得到的关节角度
    printf("left end pos = %lf, %lf, %lf, %lf, %lf, %lf\n", end_pos[0].jVal[0], end_pos[0].jVal[1], end_pos[0].jVal[2], end_pos[0].jVal[3], end_pos[0].jVal[4], end_pos[0].jVal[5], end_pos[0].jVal[6]);
    printf("right end pos = %lf, %lf, %lf, %lf, %lf, %lf\n", end_pos[1].jVal[0], end_pos[1].jVal[1], end_pos[1].jVal[2], end_pos[1].jVal[3], end_pos[1].jVal[4], end_pos[1].jVal[5], end_pos[0].jVal[6]);

    // 再次进行正运动学计算，验证结果
    // 用逆运动学得到的关节角度计算末端位姿
    // 理论上应该得到我们之前设定的目标位姿（误差在允许范围内）
    robot.kine_forward(LEFT, &end_pos[0], &pos[0]);
    robot.kine_forward(RIGHT, &end_pos[1], &pos[1]);
    
    // 打印验证结果
    // 对比这个位姿和之前设定的目标位姿，确认运动学计算的准确性
    printf("left pos = %lf, %lf, %lf, %lf, %lf, %lf\n", pos[0].tran.x, pos[0].tran.y, pos[0].tran.z, pos[0].rpy.rx, pos[0].rpy.ry, pos[0].rpy.rz);
    printf("right pos = %lf, %lf, %lf, %lf, %lf, %lf\n", pos[1].tran.x, pos[1].tran.y, pos[1].tran.z, pos[1].rpy.rx, pos[1].rpy.ry, pos[1].rpy.rz);

    // 登出机器人
    robot.login_out();
    return 0;
}
