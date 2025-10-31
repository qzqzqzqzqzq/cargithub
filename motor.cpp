#include "motor.h"
#include <iostream>
#include <thread>
#include <chrono>
#include "pca9685_driver.h"

//初始化motor
void motor_init() {
    pca_set_servo_pulse(MOTOR_PCA9685_CANNEL, 1500);
    std::cout << "[INFO] motor_init(): Please power on the ESC now. You have 2 seconds." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "[INFO] motor_init(): 2 seconds remaining..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "[INFO] motor_init(): 1 second remaining..." << std::endl;
    std::cout << "[INFO] motor_init() Beginning to unlock ESC..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    std::cout << "[INFO] motor_init(): ESC unlock sequence finished." << std::endl;
}
//用法：motor_speed(速度);,理论上速度范围为1500到2000，但是最好用已经定义的宏定义MAX_SPEED、MID_SPEED、STOP_SPEED
//！！！特别注意停止信号不是0，是1500，也就是宏定义STOP_SPEED
void motor_speed(int spd) {
    // 限制输入范围
    if (spd < 1000) spd = 1000;
    if (spd > 2000) spd = 2000;

    // 输出 PWM 脉宽信号
    pca_set_servo_pulse(MOTOR_PCA9685_CANNEL, spd);

}
//刹车函数
void motor_break() {
    motor_speed(1400);
    std::cout << "[INFO] motor_break(): The car is braking..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "[INFO] motor_break(): Returning to ESC midpoint (1500)..." << std::endl;
    motor_speed(1500);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "[INFO] motor_break(): Braking sequence completed..." << std::endl;
}