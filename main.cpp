#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include <cmath>

#include "gpio.h"
#include "motor.h"
#include "servo.h"
#include "pid.h"
#include "pca9685_driver.h"

#include "timer.h"
#include "message.h"

#include "control.h"
#include "vision.h"

#include "globals.h"

#include <csignal>

// --- 新增部分 ---
#include "httplib.h"
#include "ts_queue.h"  // 线程安全队列头文件

ThreadSafeQueue<cv::Mat> g_frame_queue; // 全局帧队列
httplib::Server svr; // 全局 HTTP 服务器对象
// ------------------

// --- 推流线程函数 ---
// --- 推流线程函数 ---
void streamer_loop() {
    svr.Get("/stream", [&](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Cache-Control", "no-cache");
        res.set_header("Connection", "keep-alive");
        res.set_header("Content-Type", "multipart/x-mixed-replace; boundary=boundarydonotcross");

        res.set_content_provider(
            "multipart/x-mixed-replace; boundary=boundarydonotcross",
            [&](uint64_t offset, httplib::DataSink& sink) {
                while (!allfinishflag.load()) {
                    cv::Mat frame = g_frame_queue.wait_and_pop();
                    if (allfinishflag.load()) {
                        sink.done();
                        return false;
                    }
                    if (frame.empty()) continue;

                    std::vector<uchar> buf;
                    std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, 50 };
                    cv::imencode(".jpg", frame, buf, params);

                    std::string header = "--boundarydonotcross\r\n";
                    header += "Content-Type: image/jpeg\r\n";
                    header += "Content-Length: " + std::to_string(buf.size()) + "\r\n\r\n";

                    if (!sink.write(header.c_str(), header.length())) break;
                    if (!sink.write(reinterpret_cast<const char*>(buf.data()), buf.size())) break;
                    if (!sink.write("\r\n", 2)) break;
                }
                sink.done();
                return !allfinishflag.load();
            }
        );
        });

    std::cout << "[INFO] Streamer starting at http://<YOUR_PI_IP>:9000/stream" << std::endl;

    if (svr.listen("0.0.0.0", 9000)) {
        if (allfinishflag.load()) {
            std::cout << "[WARN] svr.listen returned true after stop flag was set." << std::endl;
        }
    }
    else {
        std::cout << "[INFO] svr.listen exited." << std::endl;
    }

    std::cout << "[INFO] Streamer stopped." << std::endl;
}
// ------------------
//Ctrl+C信号处理函数
void signal_handler(int signum) {
    allfinishflag.store(true);
}
// ------------------
int main() {
    double kp, ki, kd;
    std::cout << "Please enter PID parameters (Kp Ki Kd): ";
    std::cin >> kp >> ki >> kd;
    //初始化
    gpio_init();
    signal(SIGINT, signal_handler);    // 注册信号处理函数：捕获 Ctrl+C
    if (!pca_init()) {
        std::cerr << "Failed to initialize PCA9685 driver!" << std::endl;
        return 1;
    }
    pca_set_pwm_freq(50.0);
    motor_init();
    servo_init(105, 90, 120);
    pid_init(kp, ki, kd, 15);
    //测试
    VisionTaskState = State::ToBlueCone;
    send_message("Start");
    //初始化帧率检测进程
    std::thread thread3(timedelayIT);
    std::thread t_vision(vision_loop);    //视觉处理线程
    std::thread t_control(control_loop_timer);                    //电机，舵机控制线程

    std::thread t_streamer(streamer_loop);
    // --- 等待退出信号 ---
    while (!allfinishflag.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // --- 开始关闭过程 ---
    std::cout << "[INFO] Shutdown signal received. Stopping server..." << std::endl;
    if (svr.is_running()) {
        svr.stop();
        std::cout << "[INFO] HTTP server stop requested." << std::endl;
    }
    else {
        std::cout << "[INFO] HTTP server was not running." << std::endl;
    }
    // ---------------------

    // --- 回收线程 ---
    std::cout << "[INFO] Joining threads..." << std::endl;
    t_vision.join();
    std::cout << "[INFO] Vision thread joined." << std::endl;
    t_control.join();
    std::cout << "[INFO] Control thread joined." << std::endl;
    thread3.join();
    std::cout << "[INFO] Timer thread joined." << std::endl;
    t_streamer.join();
    std::cout << "[INFO] Streamer thread joined." << std::endl;
    // ------------------

    std::cout << "[INFO] All threads joined. Releasing resources..." << std::endl;
    pca_close();
    gpio_release();
    SaveResultsToCSV("lane_result.csv");
    std::cout << "[INFO] Program finished." << std::endl;
    return 0;
}