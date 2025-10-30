#include <iostream>
#include "timer.h"
#include <thread>
#include <atomic>

#include "globals.h"

std::atomic<int> IMfr = 0;

void TimerInterrupt()//定时器中断
{
    std::cout << "frame rate = " << IMfr.load() << '\n';
    // cout << "图像帧率为：" << IMfr << '\n';
    IMfr.store(0);
}

int timedelayIT(void)//定时器
{
    while (!allfinishflag.load())
    {
        TimerInterrupt();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 延时 1000 毫秒
    }
    return 0;
}

