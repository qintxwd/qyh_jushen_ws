/*******************************************************************************
 * 文件名: 42.enable.cpp
 * 功能描述: 节卡双臂机器人使能/下使能循环测试程序
 * 
 * 主要功能:
 * 1. 演示机器人的使能和下使能操作
 * 2. 测试频繁上下使能的稳定性
 * 3. 统计使能操作的平均耗时
 * 4. 展示使能失败的处理方法
 * 
 * 使能说明:
 * - 使能(enable): 激活机器人伺服系统，允许运动
 * - 下使能(disable): 关闭伺服系统，机器人无法运动
 * 
 * 重要提示:
 * - 下使能后1秒内不应再次上使能，否则可能损坏硬件
 * - 建议下使能后等待至少3秒再上使能
 * - 频繁上下使能可能影响硬件寿命
 ******************************************************************************/

#include <JAKAZuRobot.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>
#include <numeric>
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
    printf("power on succeed\n");

    // 关闭碰撞检测，避免在测试过程中误触发碰撞保护
    robot.set_collision_level(0, 0);
    robot.set_collision_level(1, 0);

    // 第一次使能机器人
    ret = robot.enable_robot();
    if (ret == ERR_SUCC)
    {
        printf("enable succeed!\n");
    }

    // 定义两个测试用的关节位置
    // 用于在上下使能之间进行运动，验证使能状态
    JointValue q[2] = {{1.57 ,0 ,0 ,0 ,0 ,0}, {1.57 ,0 ,0 ,0 ,0 ,0}};
    JointValue q2[2] = {{1.57 ,-1 ,0 ,0 ,0 ,0}, {1.57 ,-1 ,0 ,0 ,0 ,0}};

    // 设置运动参数
    MoveMode moveop[2] = {ABS, ABS};
    double vel[2] = {1, 1};
    double acc[2] = {1, 1};
    double tol[2] = {0, 0};
    double id[2] = {0, 0};
    
    // 先运动到第一个位置
    ret = robot.robot_run_multi_movj(DUAL, moveop, TRUE, q, vel, acc);

    // 用于记录每次使能操作的耗时
    std::vector<double> time;
    // 记录使能失败的次数
    int count = 0;
    
    // 【循环测试上下使能】
    // 执行50次上下使能循环
    for (int i = 0; i < 50; i++)
    {
        // 检查是否有错误
        int error[2] = {0, 0};
        robot.robot_is_in_error(error);
        if (error[0] || error[1])
        {
            printf("Robot seems to have error!\n"); // 部分伺服错误是无法通过clear_error清除的
            
            // 尝试清除错误
            robot.clear_error();
            
            // 下电
            robot.power_off();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));   
            
            // 重新上电
            robot.power_on();
        }
        else
        {
            printf("No error! got on test!\n");
        }
        
        printf("Try once\n");
        
        // 记录使能开始时间
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        
        // 尝试使能机器人
        ret = robot.enable_robot();
        if (ret == ERR_SUCC)
        {
            printf("enable succeed!\n");
            
            // 使能成功后，交替运动到两个位置
            // 验证使能状态正常
            if (i % 2)
                ret = robot.robot_run_multi_movj(DUAL, moveop, TRUE, q2, vel, acc);
            else
                ret = robot.robot_run_multi_movj(DUAL, moveop, TRUE, q, vel, acc);
        }
        else
        {
            // 使能失败
            printf("ERROR = %d\n", ret);
            count++;
            i--;  // 重试当前次数
        }
        
        // 记录使能结束时间
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        
        // 计算本次使能耗时
        std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
        std::cout << "Enable time: " << duration.count() << " seconds\n";
        time.push_back(duration.count());
        
        printf("Try Disable\n");
        
        // 下使能
        ret = robot.disable_robot();
        
        // 【重要】下使能后，1s内不允许再次上使能，下使能立马再上使能，易造成硬件损坏
        // 这里等待3秒以确保安全
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));   
    }

    // 计算并输出平均使能耗时
    double total2 = accumulate(time.begin(), time.end(), 0.0);
    printf("average consuming %lf\n", total2 / time.size());
    
    // 输出失败次数统计
    printf("failed times = %d\n", count);
    
    // 登出机器人
    robot.login_out();
    return 0;
}
