#pragma once

#include <cstdint> // for uint8_t

/**
 * @brief ��ʼ�� PCA9685 ����
 * �����ڵ����κ���������֮ǰ���ô˺�����
 * @param bus_path I2C ���ߵ�·�� (���� "/dev/i2c-3")
 * @param address I2C �豸�ĵ�ַ (Ĭ��Ϊ 0x40)
 * @return true �ɹ�, false ʧ��
 */
bool pca_init(const char* bus_path = "/dev/i2c-3", uint8_t address = 0x40);

/**
 * @brief �ر��� PCA9685 ������
 * �����˳�ʱ���á�
 */
void pca_close();

/**
 * @brief ���� PWM Ƶ�� (���� 50Hz)
 * ���ǵ��� pca_set_servo_pulse ֮ǰ�ġ����衿���衣
 * @param freq Ƶ�� (Hz)
 */
void pca_set_pwm_freq(float freq);

/**
 * @brief (��Ҫ���) ���ö������
 * @param channel ͨ�� (0-15)
 * @param pulse_us ������ (��λ: ΢��, 1000-2000)
 */
void pca_set_servo_pulse(int channel, int pulse_us);

/**
 * @brief (�߼�) �ײ� PWM ���ú���
 * @param channel ͨ�� (0-15)
 * @param on_ticks ���忪ʼ tick (0-4095)
 * @param off_ticks ������� tick (0-4095)
 */
void pca_set_pwm(int channel, int on_ticks, int off_ticks);


