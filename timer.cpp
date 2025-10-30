#include <iostream>
#include "timer.h"
#include <thread>
#include <atomic>

#include "globals.h"

std::atomic<int> IMfr = 0;

void TimerInterrupt()//��ʱ���ж�
{
    std::cout << "frame rate = " << IMfr.load() << '\n';
    // cout << "ͼ��֡��Ϊ��" << IMfr << '\n';
    IMfr.store(0);
}

int timedelayIT(void)//��ʱ��
{
    while (!allfinishflag.load())
    {
        TimerInterrupt();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // ��ʱ 1000 ����
    }
    return 0;
}

