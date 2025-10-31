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
            //��һ�η���
        }
        else if (receive_message() == "Stop")
        {
            StopAndAnnounce();
            //���������С��ͣ�����Ҳ���¼�ƺõ���Ƶ
            //ע��Ҫ������������������10�룬���ⷴ��ִ�����������������������ʽʵ��Ҳ��
        }
        else if (receive_message() == "ReStart")
        {
            motor_speed(MID_SPEED);
            //���´Ӱ����߷���
        }
        else if (receive_message() == "ReStop")
        {
            motor_break();
            //����ͣ��
        }
        //ÿѭ��һ�ο���һ�ζ�����򣬽���Ӹ���ʱ����ѭ������
        controlServoWithPID();

}
void control_loop_timer()
{
    
    while (!allfinishflag.load())
    {
        control_loop();
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // ��ʱ 100ms
    }
}

//>>>>>>>>>>ѭ��<<<<<<<<<<<<

/**
*
* ���ܣ�
*  - ��ȫ�ֱ��� lane_error �л�ȡ��ǰƫ��ֵ
*  - ʹ�� PID �㷨����Ŀ�����Ƕ�
*  - ����servo_control(int target_angle, int base_angle) ʵ�ֶ��ת��
*/
void controlServoWithPID()
{
    // 1. ��ȡƫ��̰߳�ȫ��
    double error = lane_error.load();
    // 2. ����PID���������Ϊ�Ƕȱ仯ֵ
    double delta_angle = pid_calculate(error);
    double angle = +g_middle_angle - delta_angle;
        // 4. ��������źŵ����
        servo_control(angle);

}

//>>>>>>>>>���ְ�����ͣ��������<<<<<<<<
void StopAndAnnounce()
{
    motor_break();
    playRecordedVoice();
    while (receive_message() == "ReStart");//��ֹ������̫��
    //���������С��ͣ�����Ҳ���¼�ƺõ���Ƶ
    //ע��Ҫ������������������10�룬���ⷴ��ִ�����������������������ʽʵ��Ҳ��
}
