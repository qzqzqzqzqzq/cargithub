#pragma once
// ��ʼ��PID����
void pid_init(double kp, double ki, double kd, double limit);

// ���ݵ�ǰ������PID���
double pid_calculate(double error);

// ���PID�ۼ�״̬������KP\KI\KD��
void pid_clear(void);

//��������PID����
void pid_set(double kp, double ki, double kd, double limit);