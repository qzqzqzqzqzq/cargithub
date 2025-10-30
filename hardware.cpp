#include <iostream>

#include "hardware.h"

/*
* ========为了便于管理文件，这个函数已经移动到motor.cpp=========
void motor_speed(int spd)
{
    // TODO: 根据硬件接口实现PWM输出
}
 */


/*
========为了便于管理文件，这个函数已经移动到servo.cpp=========

void servo_control(int target_angle, int base_angle)
{
    // TODO: 根据 target_angle 和 base_angle 计算舵机的实际角度
    // TODO: 输出对应的 PWM 信号，驱动舵机转动
}
*/

/**
 * @brief 播放录制好的语音（路径固定）
 *
 * 功能需求：
 *  - 播放预先定义好的语音文件
 *  - 路径由宏 VOICE_FILE_PATH 定义，定义在hardware.h中
 *
 * 注意事项：
 *  - 本函数属于硬件层接口，负责与音频播放设备交互
 *  - 上层调用时不需要关心文件路径
 *
 * @return void
 */

void playRecordedVoice() {
    std::string file_path = VOICE_FILE_PATH;

    // 用 aplay 播放，屏蔽输出信息
    std::string command = "aplay " + file_path + " >/dev/null 2>&1";

    int ret = std::system(command.c_str());
    if (ret != 0) {
        // 如果失败，可以打印日志（这里简单处理）
        std::cerr << "音频播放失败: " << file_path << std::endl;
    }
}