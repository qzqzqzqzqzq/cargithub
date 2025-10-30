#pragma once
#include <atomic>

void TimerInterrupt();
int timedelayIT(void);

extern std::atomic<int> IMfr;