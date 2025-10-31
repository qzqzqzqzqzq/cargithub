#include <iostream>
#include <thread>

#include "control.h"
#include "message.h"
#include "hardware.h"
#include "globals.h"
#include "pid.h"
#include "servo.h"
#include "motor.h"

void control_loop()
{

        if (receive_message() == "Start")
        {
            motor_speed(MID_SPEED);
            //第一次发车
        }
        else if (receive_message() == "Stop")
        {
            StopAndAnnounce();
            //这个函数让小车停车并且播放录制好的音频
            //注意要让这个函数阻隔在这里10秒，以免反复执行这个函数，或者以其他方式实现也行
        }
        else if (receive_message() == "ReStart")
        {
            motor_speed(MID_SPEED);
            //重新从斑马线发车
        }
        else if (receive_message() == "ReStop")
        {
            motor_break();
            //最终停车
        }
        //每循环一次控制一次舵机方向，建议加个延时避免循环过快
        controlServoWithPID();

}
void control_loop_timer()
{
    
    while (!allfinishflag.load())
    {
        control_loop();
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 延时 100ms
    }
}

//>>>>>>>>>>循迹<<<<<<<<<<<<

/**
*
* 功能：
*  - 从全局变量 lane_error 中获取当前偏差值
*  - 使用 PID 算法计算目标舵机角度
*  - 调用servo_control(int target_angle, int base_angle) 实现舵机转动
*/
void controlServoWithPID()
{
    // 1. 读取偏差（线程安全）
    double error = lane_error.load();
    // 2. 计算PID，将结果作为角度变化值
    double delta_angle = pid_calculate(error);
    double angle = +g_middle_angle - delta_angle;
        // 4. 输出控制信号到舵机
        servo_control(angle);

}

//>>>>>>>>>发现斑马线停车并播报<<<<<<<<
void StopAndAnnounce()
{
    motor_break();
    playRecordedVoice();
    while (receive_message() == "ReStart");//防止播报的太快
    //这个函数让小车停车并且播放录制好的音频
    //注意要让这个函数阻隔在这里10秒，以免反复执行这个函数，或者以其他方式实现也行
}
