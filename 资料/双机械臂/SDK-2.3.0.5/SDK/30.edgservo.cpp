#include "JAKAZuRobot.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#include "timespec.h"
#define rad2deg(x) ((x)*180.0/M_PI)
#define deg2rad(x) ((x)*M_PI/180.0)
int64_t edg_timespec2ns(timespec t)
{
    return t.tv_sec * 1000000000 + t.tv_nsec;
}
void edg_sync(timespec reftime_, int64_t *sys_time_offset)
{
    auto reftime = edg_timespec2ns(reftime_);
    static int64_t integral = 0;
    int64_t cycletime = 1000000;
    int64_t delta = (reftime - 0) % cycletime;
    if (delta > (cycletime / 2))
    {
        delta = delta - cycletime;
    }
    if (delta > 0)
    {
        integral++;
    }
    if (delta < 0)
    {
        integral--;
    }
    if (sys_time_offset)
    {
        *sys_time_offset = -(delta / 100) - (integral / 20);  //类似PI调节
    }
}

int servoj_test(JAKAZuRobot &robot)
{
    JointValue jpos[2];
    memset(&jpos,0,sizeof(jpos));
    
    double v[] = {deg2rad(30),deg2rad(30)};
    double a[] = {deg2rad(150),deg2rad(150)};
    MoveMode mode[]={MoveMode::ABS,MoveMode::ABS};
    robot.robot_run_multi_movj(DUAL,mode,true,jpos,v,a);
    robot.servo_move_use_none_filter();
    // robot.servo_move_use_joint_LPF(125.0);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    JointValue jpos_cmd;
    memset(&jpos_cmd,0,sizeof(jpos_cmd));
    JointValue jpos_cmd2;
    memset(&jpos_cmd2,0,sizeof(jpos_cmd2));

    bool rob1_change_to_servo = false;
    // std::thread t([&](){
    //     JointValue jpos[2];
    //     memset(&jpos,0,sizeof(jpos));
    //     double v[] = {deg2rad(30),deg2rad(30)};
    //     double a[] = {deg2rad(150),deg2rad(150)};
    //     MoveMode mode[]={MoveMode::ABS,MoveMode::ABS};
    //     for(int i = 0;i<1;i++)
    //     {
    //         jpos[1].jVal[0] = deg2rad(30);
    //         robot.robot_run_multi_movj(RIGHT,mode,true,jpos,v,a);
    //         std::this_thread::sleep_for(std::chrono::seconds(1));
    //         jpos[1].jVal[0] = deg2rad(-30);
    //         robot.robot_run_multi_movj(RIGHT,mode,true,jpos,v,a);
    //         std::this_thread::sleep_for(std::chrono::seconds(1));
    //     }
    //     jpos[1].jVal[0] = deg2rad(0);
    //     robot.robot_run_multi_movj(RIGHT,mode,true,jpos,v,a);
    //     robot.servo_move_enable(1, 1);
    //     rob1_change_to_servo = true;

    // });
    sched_param sch;
    sch.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);
    robot.servo_move_enable(1, 0);
    timespec next;
    clock_gettime(CLOCK_REALTIME, &next);
    
    int rob2_start_k = 0;
    for(int i = 0;;i++)
    {
        timespec max_wkup_time;
        robot.edg_recv(&next);
        timespec cur_time;
        timespec cc;
        clock_gettime(CLOCK_REALTIME, &cc);

        JointValue jpos[2];
        CartesianPose cpos[2];
        int64_t recv_time = 0;
        robot.edg_get_stat(0, &jpos[0], &cpos[0]);
        robot.edg_get_stat(1, &jpos[1], &cpos[1]);
        unsigned long int details[3];
        robot.edg_stat_details(details);
    
        double t = (i - 0)/10000.0;
        double kk = 3;
        jpos_cmd.jVal[0] = sin(kk*t)*30;
        jpos_cmd.jVal[1] = -cos(kk*t)*20 + 20;
        jpos_cmd.jVal[3] = -cos(kk*t)*10 + 10;
        robot.edg_servo_j(0,&jpos_cmd,MoveMode::ABS);
        if(rob1_change_to_servo == true && rob2_start_k == 0)
        {
            rob2_start_k = i;
        }

        if(rob2_start_k && rob1_change_to_servo == true)
        {
            double tt = (i - rob2_start_k)/10000.0;
            double kkk = 5;
            // printf("set robot2\n");
            jpos_cmd2.jVal[0] = sin(kkk*tt)*30;
            jpos_cmd2.jVal[1] = -cos(kkk*tt)*20 + 20;
            jpos_cmd2.jVal[3] = -cos(kkk*tt)*10 + 10;
        }
        robot.edg_servo_j(1, &jpos_cmd2, MoveMode::ABS);
        robot.edg_send();
        
#if 1
        printf("%ld %ld %ld %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %ld %ld %ld\n", 
        //std::chrono::duration_cast<std::chrono::nanoseconds>(cur.time_since_epoch()).count(),
        edg_timespec2ns(cc),
        edg_timespec2ns(cur_time),
        edg_timespec2ns(max_wkup_time),
        jpos[0].jVal[0], jpos[0].jVal[1], jpos[0].jVal[3],
        jpos[1].jVal[0], jpos[1].jVal[1], jpos[1].jVal[3],
        jpos_cmd.jVal[0], jpos_cmd.jVal[1], jpos_cmd.jVal[3],
        details[0], details[1], details[2]
        );


        
#endif
        //等待下一个周期
        timespec dt;
        dt.tv_nsec = 1000000;
        dt.tv_sec = 0;
        next = timespec_add(next, dt);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
    }
}

void servop_test(JAKAZuRobot &robot)
{
    JointValue jpos[2];
    memset(&jpos,0,sizeof(jpos));
    jpos[0].jVal[0] = deg2rad(60);
    jpos[0].jVal[1] = deg2rad(30);
    jpos[0].jVal[2] = deg2rad(30);
    jpos[0].jVal[3] = deg2rad(-45);
    jpos[0].jVal[4] = deg2rad(-75);
    jpos[0].jVal[5] = deg2rad(-50);
    jpos[0].jVal[6] = deg2rad(40);
 
    double jv[2] = {deg2rad(10)};
    double ja[2] = {deg2rad(100)};
    MoveMode mode[]={MoveMode::ABS,MoveMode::ABS};
    robot.robot_run_multi_movj(LEFT,mode,true,jpos,jv,ja);

    double v[] = {10,10};
    double a[] = {100,100};
    CartesianPose cpos[2];
    robot.kine_forward(0,&jpos[0],&cpos[0]);
    robot.kine_inverse(0,&jpos[0],&cpos[0],&jpos[1]);
    cpos[0].tran.z += 10;
    robot.robot_run_multi_movl(LEFT,mode,true,cpos,v,a);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    memcpy(&cpos[1],&cpos[0],sizeof(cpos[0]));
    // cpos[1].rpy.rx = rad2deg(cpos[0].rpy.rx);
    // cpos[1].rpy.ry = rad2deg(cpos[0].rpy.ry);
    // cpos[1].rpy.rz = rad2deg(cpos[0].rpy.rz);
    sched_param sch;
    sch.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch);
    robot.servo_move_enable(1, 0);
    timespec next;
    clock_gettime(CLOCK_REALTIME, &next);
    
    for(int i=0;;i++)
    {
        robot.edg_recv(&next);

        JointValue actjpos;
        CartesianPose actcpos;
        robot.edg_get_stat(0, &actjpos, &actcpos);
        double coefficient = sin(i/1000.0);
        double z_step = 100 * coefficient;
        double x_step = 20 * coefficient;  
        cpos[1].tran.z = cpos[0].tran.z + z_step;
        cpos[1].tran.x = cpos[0].tran.x + x_step;
        double ori_step = deg2rad(10) * coefficient;
        cpos[1].rpy.rx = cpos[0].rpy.rx + ori_step;
        cpos[1].rpy.ry = cpos[0].rpy.ry + ori_step;
        cpos[1].rpy.rz = cpos[0].rpy.rz + ori_step;
        robot.edg_servo_p(0,&cpos[1],MoveMode::ABS);
        robot.edg_send();
        printf("%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", actcpos.tran.x, actcpos.tran.y, actcpos.tran.z, actcpos.rpy.rx, actcpos.rpy.ry, actcpos.rpy.rz);


        //等待下一个周期
        timespec dt;
        dt.tv_nsec = 1000000;
        dt.tv_sec = 0;
        next = timespec_add(next, dt);
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
    }
}

int main()
{
    //
    JAKAZuRobot robot;

    robot.login_in("192.168.2.200");
    // robot.login_in("192.168.2.9");

    robot.servo_move_enable(0, -1); //关闭所有机器人的伺服模式
    robot.servo_move_use_none_filter();
    robot.motion_abort();
    robot.power_on();
    robot.enable_robot();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    servoj_test(robot);
    // servop_test(robot);
    
    robot.login_out();
    return 0;
}