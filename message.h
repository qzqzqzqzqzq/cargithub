#pragma once
#include <string>
extern std::string global_message;   // ȫ���ַ�������������̼߳䴫�ݵ���Ϣ

void send_message(const std::string& msg);
std::string receive_message();