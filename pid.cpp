#include "pid.h"

// ==== PID �ڲ�״̬ ====
static double kp_ = 0.0;
static double ki_ = 0.0;
static double kd_ = 0.0;

static double prev_error_ = 0.0;
static double integral_ = 0.0;
static double limit_ = 0.0;

// ==== ��ʼ�� ====
void pid_init(double kp, double ki, double kd, double limit) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    limit_ = limit;
    prev_error_ = 0.0;
    integral_ = 0.0;
}
// ==== ���� PID ��� ====
double pid_calculate(double error) {
    integral_ += error;
    //�����޷�
    if (integral_ > limit_) integral_ = limit_;
    else if (integral_ < -limit_) integral_ = -limit_;

    double derivative = error - prev_error_;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    //����޷�
    if (output > limit_) output = limit_;
    else if (output < -limit_) output = -limit_;

    prev_error_ = error;

    return output;
}
// ==== ���״̬ ====
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