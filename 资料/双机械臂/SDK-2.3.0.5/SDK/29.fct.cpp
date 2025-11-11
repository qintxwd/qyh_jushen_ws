#include <JAKAZuRobot.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include "common.h"

#define JK_PI (3.141592653589793)
#define deg_tp_rad 1.0 / 180.0 * JK_PI

#define LEFT_SENSOR_ID 1
#define RIGHT_SENSOR_ID 2

void set_deadzone(JAKAZuRobot& robot, double *deadzone, double *deadzone_right)
{
    errno_t ret = robot.robot_set_ftsensor_deadzone(LEFT, LEFT_SENSOR_ID, deadzone);
    if (ret != ERR_SUCC)
    {
        std::cout << "set deadzone failed" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ret = robot.robot_get_ftsensor_deadzone(LEFT, LEFT_SENSOR_ID, deadzone);
    if (ret != ERR_SUCC)
    {
        std::cout << "get deadzone failed" << std::endl;
    }
    else
    {
        printf("deadzone: %f %f %f %f %f %f\n", deadzone[0], deadzone[1], deadzone[2], deadzone[3], deadzone[4], deadzone[5]);
    }
    
    ret = robot.robot_set_ftsensor_deadzone(RIGHT, 2, deadzone_right);
    if (ret != ERR_SUCC)
    {
        std::cout << "set deadzone failed" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ret = robot.robot_get_ftsensor_deadzone(RIGHT, 2, deadzone_right);
    if (ret != ERR_SUCC)
    {
        std::cout << "get deadzone failed" << std::endl;
    }
    else
    {
        printf("deadzone: %f %f %f %f %f %f\n", deadzone_right[0], deadzone_right[1], deadzone_right[2], deadzone_right[3], deadzone_right[4], deadzone_right[5]);
    }
}

void set_frame(JAKAZuRobot &robot, int *frame)
{
    errno_t ret = robot.robot_set_cst_ftframe(LEFT, frame[0]);
    ret = robot.robot_set_cst_ftframe(RIGHT, frame[1]);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    int res[2] = {-1, -1};
    ret = robot.robot_get_cst_ftframe(LEFT, res);
    ret = robot.robot_get_cst_ftframe(RIGHT, res+1);
    if (ret != ERR_SUCC)
    {
        std::cout << "get ftframe failed" << std::endl;
    }
    else
    {
        printf("ftframe: %d %d\n", res[0], res[1]);
    }
}

int set_filter(JAKAZuRobot &robot, double* target)
{
    errno_t ret = robot.robot_set_ftsensor_filter(LEFT, LEFT_SENSOR_ID, target[0]);
    ret = robot.robot_set_ftsensor_filter(RIGHT, RIGHT_SENSOR_ID, target [1]);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));    
    double filter[2] = {};
    ret = robot.robot_get_ftsensor_filter(LEFT, 1, filter);
    ret += robot.robot_get_ftsensor_filter(RIGHT, 2, filter+1);
    if (ret != 0)
    {
        std::cout << "get ftfilter failed" << std::endl;
    }
    else
    {
        printf("ftfilter: %f %f\n", filter[0], filter[1]);
    }
}

int set_payload(JAKAZuRobot &robot, PayLoad *payload)
{
    error_t ret = robot.robot_set_ftsensor_payload(LEFT, LEFT_SENSOR_ID, payload[0]);
    ret = robot.robot_set_ftsensor_payload(RIGHT, RIGHT_SENSOR_ID, payload[1]);
    if (ret != 0)
    {
        std::cout << "set payload failed" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    PayLoad res[2];
    ret = robot.robot_get_ftsensor_payload(LEFT, 1, res);
    ret = robot.robot_get_ftsensor_payload(RIGHT, 2, res+1);
    if (ret != 0)
    {
        std::cout << "get payload failed" << std::endl;
    }
    else
    {
         printf("left payload: %f %f %f %f\n", res[0].centroid.x, res[0].centroid.y, res[0].centroid.z, res[0].mass);
         printf("right payload: %f %f %f %f\n", res[1].centroid.x, res[1].centroid.y, res[1].centroid.z, res[1].mass);
    }
}

void check_fct_state(JAKAZuRobot &robot)
{
    int stat[2] = {-1, -1};
    robot.robot_get_force_control_stat(LEFT, stat);
    robot.robot_get_force_control_stat(RIGHT, stat + 1);
    printf("Got for state: %d %d\n", stat[0], stat[1]);
}

int main()
{
    JAKAZuRobot robot;
    errno_t ret;
    ret = robot.login_in(IP);
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "login");

    ret = robot.power_on();
    ASSERT_TRUE_OR_EXIT(ret == ERR_SUCC, "power on");

    check_error(robot, true);

    ret = robot.enable_robot();
    if (ret != ERR_SUCC)
    {
        printf("enable failed! ret = %d\n", ret);
    }

    check_state(robot);


    // 死区
    double left_deadzone[6] = {0.1, 0.2, 0.3 , 0.4, 0.5, 0.6};
    double right_deadzone[6] = {0.7, 0.8, 0.9, 0.1, 0.2, 0.3};
    set_deadzone(robot, left_deadzone, right_deadzone);
    double reset_deadzone[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    set_deadzone(robot, reset_deadzone, reset_deadzone);

    // 坐标系
    int frame[2] = {1, 1};
    set_frame(robot, frame);

    // 滤波器
    double target[2] = {200.0, 200.0};
    set_filter(robot, target);

    // 负载
    PayLoad payload2[2] = {{5,{60,-60,30}},{0,{0,0,0}}};
    set_payload(robot, payload2);

    // 力控参数
    ret = robot.robot_set_cst_ftconfig(LEFT, 0, 0, 10, 1, 1);
    ret = robot.robot_set_cst_ftconfig(LEFT, 1, 0, 10, 1, 2);
    ret = robot.robot_set_cst_ftconfig(LEFT, 2, 1, 10, 1, 3);
    ret = robot.robot_set_cst_ftconfig(LEFT, 3, 0, 10, 1, 0.1);
    ret = robot.robot_set_cst_ftconfig(LEFT, 4, 0, 10, 1, 0.1);
    ret = robot.robot_set_cst_ftconfig(LEFT, 5, 0, 10, 1, 0.1);

    ret = robot.robot_set_cst_ftconfig(RIGHT, 0, 0, 10, 1, 3);
    ret = robot.robot_set_cst_ftconfig(RIGHT, 1, 0, 20, 2, 4);
    ret = robot.robot_set_cst_ftconfig(RIGHT, 2, 1, 30, 3, 5);
    ret = robot.robot_set_cst_ftconfig(RIGHT, 3, 0, 40, 4, 0.2);
    ret = robot.robot_set_cst_ftconfig(RIGHT, 4, 0, 50, 5, 0.3);
    ret = robot.robot_set_cst_ftconfig(RIGHT, 5, 0, 60, 6, 0.4);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    RobotAdmitCtrl ftconfig[2];
    ret = robot.robot_get_cst_ftconfig(LEFT, ftconfig);
    ret = robot.robot_get_cst_ftconfig(RIGHT, ftconfig + 1);
    if (ret != 0) 
    {
        printf("robot_get_cst_ftconfig failed!\n");
    } 
    else 
    {
        printf("robot_get_cst_ftconfig success!\n");
        for (int i = 0; i < 6; i++)
        {
            printf("ftconfig[%d] = enable %d b %f k %f t %f\n", i, ftconfig[0].admit_ctrl[i].opt, ftconfig[0].admit_ctrl[i].ft_user, ftconfig[0].admit_ctrl[i].ft_rebound, ftconfig[0].admit_ctrl[i].ft_constant);
        }
        for (int i = 0; i < 6; i++)
        {
            printf("ftconfig[%d] = enable %d b %f k %f t %f\n", i, ftconfig[1].admit_ctrl[i].opt, ftconfig[1].admit_ctrl[i].ft_user, ftconfig[1].admit_ctrl[i].ft_rebound, ftconfig[1].admit_ctrl[i].ft_constant);
        }
    }

    // 校零
    robot.robot_zero_ftsensor(LEFT, LEFT_SENSOR_ID);    // 内部阻塞1s
    robot.robot_zero_ftsensor(RIGHT, RIGHT_SENSOR_ID);


    robot.robot_enable_force_control(LEFT);
    robot.robot_enable_force_control(RIGHT);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    for (int i = 0; i < 100; i++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        check_fct_state(robot);

        int status[2] = {-3, -3};
        int errorcode[2] = {-1, -1};
        double ft_original[6] = {};
        double ft_actual[6] = {};
        ret = robot.robot_get_ftsensor_stat(LEFT,  1, status, errorcode, ft_original, ft_actual);
        if (ret != 0)
        {
            printf("robot_get_ftsensor_stat failed!\n");
        }
        else
        {
            printf("LEFT ft_original: %f %f %f %f %f %f\n", ft_original[0], ft_original[1], ft_original[2], ft_original[3], ft_original[4], ft_original[5]);
            printf("LEFT ft_actual: %f %f %f %f %f %f\n", ft_actual[0], ft_actual[1], ft_actual[2], ft_actual[3], ft_actual[4], ft_actual[5]);
        }
        ret = robot.robot_get_ftsensor_stat(RIGHT,  2, status + 1, errorcode + 1, ft_original, ft_actual);
        if (ret != 0)
        {
            printf("robot_get_ftsensor_stat failed!\n");
        }
        else
        {
            printf("RIGHT ft_original: %f %f %f %f %f %f\n", ft_original[0], ft_original[1], ft_original[2], ft_original[3], ft_original[4], ft_original[5]);
            printf("RIGHT ft_actual: %f %f %f %f %f %f\n", ft_actual[0], ft_actual[1], ft_actual[2], ft_actual[3], ft_actual[4], ft_actual[5]);
        }
        printf("status: %d %d\n", status[0], status[1]);
        printf("errorcode: %d %d\n", errorcode[0], errorcode[1]);
    }

    robot.robot_disable_force_control(LEFT);
    robot.robot_disable_force_control(RIGHT);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    int stat[2] = {};
    robot.robot_get_force_control_stat(LEFT, stat);
    robot.robot_get_force_control_stat(RIGHT, stat + 1);
    printf("Got for state: %d %d\n", stat[0], stat[1]);

    robot.login_out();
    return 0;
}
