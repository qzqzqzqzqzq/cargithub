#include <iostream>

#include "hardware.h"

/*
* ========Ϊ�˱��ڹ����ļ�����������Ѿ��ƶ���motor.cpp=========
void motor_speed(int spd)
{
    // TODO: ����Ӳ���ӿ�ʵ��PWM���
}
 */


/*
========Ϊ�˱��ڹ����ļ�����������Ѿ��ƶ���servo.cpp=========

void servo_control(int target_angle, int base_angle)
{
    // TODO: ���� target_angle �� base_angle ��������ʵ�ʽǶ�
    // TODO: �����Ӧ�� PWM �źţ��������ת��
}
*/

/**
 * @brief ����¼�ƺõ�������·���̶���
 *
 * ��������
 *  - ����Ԥ�ȶ���õ������ļ�
 *  - ·���ɺ� VOICE_FILE_PATH ���壬������hardware.h��
 *
 * ע�����
 *  - ����������Ӳ����ӿڣ���������Ƶ�����豸����
 *  - �ϲ����ʱ����Ҫ�����ļ�·��
 *
 * @return void
 */

void playRecordedVoice() {
    std::string file_path = VOICE_FILE_PATH;

    // �� aplay ���ţ����������Ϣ
    std::string command = "aplay " + file_path + " >/dev/null 2>&1";

    int ret = std::system(command.c_str());
    if (ret != 0) {
        // ���ʧ�ܣ����Դ�ӡ��־������򵥴���
        std::cerr << "��Ƶ����ʧ��: " << file_path << std::endl;
    }
}