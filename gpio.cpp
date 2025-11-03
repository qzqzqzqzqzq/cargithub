#include "gpio.h"
#include <iostream>
#include <pigpio.h>
#include <csignal>
#include <cstdlib>

#include "vision.h"
#include "globals.h"
//gpio初始化函数。注意调用必须在ctrl+c注册信号之前，不然注册会被覆盖
void gpio_init() {

    // 保险起见，先释放旧的
    gpioTerminate();
    if (gpioInitialise() < 0) {
        std::cerr << "gpio_init():Failed to initialize pigpio!" << std::endl;
        exit(1);
    }
    std::cout << "gpio_init():pigpio library initialized successfully!" << std::endl;
}

// 释放 GPIO 资源的函数
void gpio_release() {
    std::cout << "\ngpio_release():Releasing GPIO resources..." << std::endl;
    gpioTerminate();
    std::cout << "gpio_release():GPIO released safely." << std::endl;
}
