#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include <cmath>

#include "gpio.h"
#include "motor.h"
#include "servo.h"
#include "pid.h"

#include "timer.h"
#include "message.h"

#include "control.h"
#include "vision.h"

#include "globals.h"

#include <csignal>

// --- �������� ---
#include "httplib.h"
#include "ts_queue.h"  // �̰߳�ȫ����ͷ�ļ�

ThreadSafeQueue<cv::Mat> g_frame_queue; // ȫ��֡����
httplib::Server svr; // ȫ�� HTTP ����������
// ------------------

// --- �����̺߳��� ---
// --- �����̺߳��� ---
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
//Ctrl+C�źŴ�����
void signal_handler(int signum) {
    allfinishflag.store(true);
}
// ------------------
int main() {
    signal(SIGINT, signal_handler);    // ע���źŴ����������� Ctrl+C
    double kp, ki, kd;
    std::cout << "������ PID ���� (Kp Ki Kd): ";
    std::cin >> kp >> ki >> kd;
    //��ʼ��
    motor_init();
    servo_init(105, 90, 120);
    pid_init(kp, ki, kd, 15);
    //����
    VisionTaskState = State::ToBlueCone;
    send_message("Start");
    //��ʼ��֡�ʼ�����
    std::thread thread3(timedelayIT);
    std::thread t_vision(vision_loop);    //�Ӿ������߳�
    std::thread t_control(control_loop_timer);                    //�������������߳�

    std::thread t_streamer(streamer_loop);
    // --- �ȴ��˳��ź� ---
    while (!allfinishflag.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // --- ��ʼ�رչ��� ---
    std::cout << "[INFO] Shutdown signal received. Stopping server..." << std::endl;
    if (svr.is_running()) {
        svr.stop();
        std::cout << "[INFO] HTTP server stop requested." << std::endl;
    }
    else {
        std::cout << "[INFO] HTTP server was not running." << std::endl;
    }
    // ---------------------

    // --- �����߳� ---
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
    SaveResultsToCSV("lane_result.csv");
    std::cout << "[INFO] Program finished." << std::endl;
    return 0;
}