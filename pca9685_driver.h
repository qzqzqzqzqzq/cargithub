#pragma once

#include <cstdint> // for uint8_t

/**
 * @brief 初始化 PCA9685 驱动
 * 必须在调用任何其他函数之前调用此函数。
 * @param bus_path I2C 总线的路径 (例如 "/dev/i2c-3")
 * @param address I2C 设备的地址 (默认为 0x40)
 * @return true 成功, false 失败
 */
bool pca_init(const char* bus_path = "/dev/i2c-3", uint8_t address = 0x40);

/**
 * @brief 关闭与 PCA9685 的连接
 * 程序退出时调用。
 */
void pca_close();

/**
 * @brief 设置 PWM 频率 (例如 50Hz)
 * 这是调用 pca_set_servo_pulse 之前的【必需】步骤。
 * @param freq 频率 (Hz)
 */
void pca_set_pwm_freq(float freq);

/**
 * @brief (您要求的) 设置舵机脉冲
 * @param channel 通道 (0-15)
 * @param pulse_us 脉冲宽度 (单位: 微秒, 1000-2000)
 */
void pca_set_servo_pulse(int channel, int pulse_us);

/**
 * @brief (高级) 底层 PWM 设置函数
 * @param channel 通道 (0-15)
 * @param on_ticks 脉冲开始 tick (0-4095)
 * @param off_ticks 脉冲结束 tick (0-4095)
 */
void pca_set_pwm(int channel, int on_ticks, int off_ticks);


