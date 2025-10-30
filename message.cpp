#include <iostream>
#include <string>
#include <thread>
#include <mutex>

std::string global_message;   // 全局字符串，用来存放线程间传递的消息
std::mutex msg_mutex;         // 互斥锁，保证多线程读写消息时不会冲突

/**
 * @brief 向全局消息变量写入内容（线程安全）
 *
 * 调用该函数时，会自动加锁，确保只有一个线程能同时修改消息。
 * 这样避免了多个线程同时写入时出现“数据混乱”或“丢失”的情况。
 *
 * @param msg 要传递的字符串消息
 */
void send_message(const std::string& msg) {
    std::lock_guard<std::mutex> lock(msg_mutex);
    global_message = msg;  // 修改全局变量
}

/**
 * @brief 从全局消息变量中读取内容（线程安全）
 *
 * 调用该函数时，会自动加锁，确保读取时不会遇到另一个线程正在写入的情况。
 * 返回的结果是一个字符串副本，因此读出来后就和全局变量解耦了。
 *
 * @return std::string 当前存储的消息
 */
std::string receive_message() {
    std::lock_guard<std::mutex> lock(msg_mutex);
    return global_message;  // 返回一份副本
}
