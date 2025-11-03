#pragma once
extern int g_middle_angle;
extern int g_left_angle_limit;
extern int g_right_angle_limit;

int angleToPulse(int angle);

void servo_init(int middle_angle, int left_angle_limit, int right_angle_limit);
void servo_control(int angle);

