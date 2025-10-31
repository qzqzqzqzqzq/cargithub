#pragma once
#define MAX_SPEED 1600  //实际数值需根据小车电机调试修改
#define MID_SPEED 1590   
#define STOP_SPEED 1500

#define MOTOR_PCA9685_CANNEL 0 //motor信号在PCA9685上输出的通道

void motor_init();
void motor_speed(int spd);
void motor_break();