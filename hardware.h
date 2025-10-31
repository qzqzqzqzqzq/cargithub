#pragma once


#define VOICE_FILE_PATH "audio/start.wav"//TODO:占位值。实际路径根据具体情况修改

//void motor_speed(int spd);	为了便于管理文件，这个函数已经移动到motor.cpp
//void servo_control(int target_angle, int base_angle);	为了便于管理文件，这个函数已经移动到servo.cpp

void playRecordedVoice();