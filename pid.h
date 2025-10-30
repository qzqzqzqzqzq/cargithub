#pragma once
// 初始化PID参数
void pid_init(double kp, double ki, double kd, double limit);

// 根据当前误差计算PID输出
double pid_calculate(double error);

// 清除PID累计状态（保留KP\KI\KD）
void pid_clear(void);

//重新设置PID参数
void pid_set(double kp, double ki, double kd, double limit);