#include "pid.h"

// ==== PID 内部状态 ====
static double kp_ = 0.0;
static double ki_ = 0.0;
static double kd_ = 0.0;

static double prev_error_ = 0.0;
static double integral_ = 0.0;
static double limit_ = 0.0;

// ==== 初始化 ====
void pid_init(double kp, double ki, double kd, double limit) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    limit_ = limit;
    prev_error_ = 0.0;
    integral_ = 0.0;
}
// ==== 计算 PID 输出 ====
double pid_calculate(double error) {
    integral_ += error;
    //积分限幅
    if (integral_ > limit_) integral_ = limit_;
    else if (integral_ < -limit_) integral_ = -limit_;

    double derivative = error - prev_error_;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    //结果限幅
    if (output > limit_) output = limit_;
    else if (output < -limit_) output = -limit_;

    prev_error_ = error;

    return output;
}
// ==== 清空状态 ====
void pid_clear(void) {
    prev_error_ = 0.0;
    integral_ = 0.0;
}
void pid_set(double kp, double ki, double kd, double limit) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    limit_ = limit;
}