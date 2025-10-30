#include "pca9685_driver.h"

#include <iostream>
#include <unistd.h>     // for read, write, close, usleep
#include <fcntl.h>      // for O_RDWR
#include <sys/ioctl.h>  // for ioctl
#include <linux/i2c-dev.h> // for I2C_SLAVE
#include <linux/i2c.h>     
#include <cmath>        // for floor, round
#include <cstdio>       // for perror

// --- PCA9685 寄存器地址定义 ---
#define MODE1               0x00
#define PRE_SCALE           0xFE
#define LED0_ON_L           0x06
#define LED0_ON_H           0x07
#define LED0_OFF_L          0x08
#define LED0_OFF_H          0x09

//
// ===================================================================
//  模块的“隐藏状态”
//  'static' 关键字使这些变量仅在此 .cpp 文件中可见
// ===================================================================
//
static int     g_file = -1;       // I2C 文件描述符 (句柄)
static uint8_t g_address = 0x40;  // I2C 设备地址

//
// ===================================================================
//  内部(私有)辅助函数
// ===================================================================
//
static void _pca_write_register(uint8_t reg, uint8_t value) {
    if (g_file < 0) return; // 未初始化
    uint8_t outbuf[2];
    outbuf[0] = reg;
    outbuf[1] = value;
    struct i2c_msg msg = { .addr = g_address, .flags = 0, .len = 2, .buf = outbuf };
    struct i2c_rdwr_ioctl_data ioctl_data = { .msgs = &msg, .nmsgs = 1 };
    if (ioctl(g_file, I2C_RDWR, &ioctl_data) < 0) {
        perror("ioctl _pca_write_register failed");
    }
}

static uint8_t _pca_read_register(uint8_t reg) {
    if (g_file < 0) return 0; // 未初始化
    uint8_t outbuf[1] = { reg };
    uint8_t inbuf[1] = { 0 };
    struct i2c_msg msgs[2] = {
        {.addr = g_address, .flags = 0, .len = 1, .buf = outbuf },
        {.addr = g_address, .flags = I2C_M_RD, .len = 1, .buf = inbuf }
    };
    struct i2c_rdwr_ioctl_data ioctl_data = { .msgs = msgs, .nmsgs = 2 };
    if (ioctl(g_file, I2C_RDWR, &ioctl_data) < 0) {
        perror("ioctl _pca_read_register failed");
        return 0;
    }
    return inbuf[0];
}

//
// ===================================================================
//  公共函数 (在 .h 文件中声明)
// ===================================================================
//
bool pca_init(const char* bus_path, uint8_t address) {
    if (g_file >= 0) {
        close(g_file); // 如果已打开，先关闭
    }

    g_address = address;

    if ((g_file = open(bus_path, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        return false;
    }

    if (ioctl(g_file, I2C_SLAVE, g_address) < 0) {
        perror("Failed to acquire bus access");
        close(g_file);
        g_file = -1;
        return false;
    }

    return true;
}

void pca_close() {
    if (g_file >= 0) {
        pca_set_pwm(0, 0, 0); // (可选) 关闭所有舵机
        close(g_file);
        g_file = -1;
    }
}

void pca_set_pwm_freq(float freq) {
    if (g_file < 0) return; // 必须先 init

    float prescaleval = (25000000.0 / (4096.0 * freq)) - 1.0;
    uint8_t prescale = static_cast<uint8_t>(floor(prescaleval + 0.5));

    uint8_t old_mode = _pca_read_register(MODE1);
    _pca_write_register(MODE1, (old_mode & 0x7F) | 0x10); // Sleep
    _pca_write_register(PRE_SCALE, prescale);
    _pca_write_register(MODE1, old_mode & 0xEF); // Wake up
    usleep(5000);
    _pca_write_register(MODE1, (old_mode & 0xEF) | 0x80); // Restart
}

void pca_set_pwm(int channel, int on_ticks, int off_ticks) {
    if (g_file < 0) return; // 必须先 init

    uint8_t reg_base = LED0_ON_L + 4 * channel;
    _pca_write_register(reg_base, on_ticks & 0xFF);
    _pca_write_register(reg_base + 1, on_ticks >> 8);
    _pca_write_register(reg_base + 2, off_ticks & 0xFF);
    _pca_write_register(reg_base + 3, off_ticks >> 8);
}

void pca_set_servo_pulse(int channel, int pulse_us) {
    if (g_file < 0) return; // 必须先 init

    int off_ticks = static_cast<int>(round(pulse_us * 0.2048));
    if (off_ticks < 0) off_ticks = 0;
    if (off_ticks > 4095) off_ticks = 4095;

    pca_set_pwm(channel, 0, off_ticks);
}