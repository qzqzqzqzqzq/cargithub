#include <iostream>
#include <string>
#include <thread>
#include <mutex>

std::string global_message;   // ȫ���ַ�������������̼߳䴫�ݵ���Ϣ
std::mutex msg_mutex;         // ����������֤���̶߳�д��Ϣʱ�����ͻ

/**
 * @brief ��ȫ����Ϣ����д�����ݣ��̰߳�ȫ��
 *
 * ���øú���ʱ�����Զ�������ȷ��ֻ��һ���߳���ͬʱ�޸���Ϣ��
 * ���������˶���߳�ͬʱд��ʱ���֡����ݻ��ҡ��򡰶�ʧ���������
 *
 * @param msg Ҫ���ݵ��ַ�����Ϣ
 */
void send_message(const std::string& msg) {
    std::lock_guard<std::mutex> lock(msg_mutex);
    global_message = msg;  // �޸�ȫ�ֱ���
}

/**
 * @brief ��ȫ����Ϣ�����ж�ȡ���ݣ��̰߳�ȫ��
 *
 * ���øú���ʱ�����Զ�������ȷ����ȡʱ����������һ���߳�����д��������
 * ���صĽ����һ���ַ�����������˶�������ͺ�ȫ�ֱ��������ˡ�
 *
 * @return std::string ��ǰ�洢����Ϣ
 */
std::string receive_message() {
    std::lock_guard<std::mutex> lock(msg_mutex);
    return global_message;  // ����һ�ݸ���
}
