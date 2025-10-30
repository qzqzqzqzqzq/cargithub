#pragma once
#include <string>
extern std::string global_message;   // 全局字符串，用来存放线程间传递的消息

void send_message(const std::string& msg);
std::string receive_message();