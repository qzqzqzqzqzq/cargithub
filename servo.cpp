#include "servo.h"
#include <iostream>
#include <pigpio.h>

// === 储存初始化数据的变量 ===
int g_middle_angle = 105;
int g_left_angle_limit = 90;
int g_right_angle_limit = 120;

// === 工具函数: 角度 → 脉宽 ===
//  0-180度 →（500–2500 µs）
int angleToPulse(int angle) {
    return 500 + (angle * 2000 / 180);
}

// === 舵机初始化函数 ===
void servo_init(int middle_angle, int left_angle_limit, int right_angle_limit) {
    // 保存参数到全局变量
    g_middle_angle = middle_angle;
    g_left_angle_limit = left_angle_limit;
    g_right_angle_limit = right_angle_limit;

    // GPIO12 对应舵机控制信号输出
    int pulse = angleToPulse(g_middle_angle);
    gpioServo(12, pulse);

    std::cout << "[SERVO] Servo is initialized:" << std::endl;
    std::cout << "  MiddleAngle = " << g_middle_angle << "°" << std::endl;
    std::cout << "  LeftAngle = " << g_left_angle_limit << "°" << std::endl;
    std::cout << "  RightAngle = " << g_right_angle_limit << "°" << std::endl;
}

// === 舵机控制函数 ===
//用法：servo_control(角度);yi一般认为105度是中间，120右，90度左（10月25日；错了应该是120左，90度右，所以只改了controlServoWithPID，这里先不动了）
void servo_control(int angle) {
    // 限幅防止越界
    if (angle < g_left_angle_limit) {
        angle = g_left_angle_limit;
    }
    else if (angle > g_right_angle_limit) {
        angle = g_right_angle_limit;
    }
    // 转换为 PWM 脉宽并输出
    int pulse = angleToPulse(angle);
    gpioServo(12, pulse);
}