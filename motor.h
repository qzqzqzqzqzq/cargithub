#pragma once
#define MAX_SPEED 1600  //ʵ����ֵ�����С����������޸�
#define MID_SPEED 1590   
#define STOP_SPEED 1500

#define MOTOR_PCA9685_CANNEL 0 //motor�ź���PCA9685�������ͨ��

void motor_init();
void motor_speed(int spd);