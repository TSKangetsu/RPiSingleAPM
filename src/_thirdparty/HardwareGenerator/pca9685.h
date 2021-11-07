
#pragma once
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <linux/i2c-dev.h>

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LEDALL_ON_L 0xFA

#define PIN_ALL 16

inline uint16_t baseReg(int pin)
{
    return (pin >= PIN_ALL ? LEDALL_ON_L : LED0_ON_L + 4 * pin);
}

inline int pca9685Setup(const char *I2CChannel, uint8_t I2CAddress, int frequency)
{
    int fd = 0;
    if ((fd = open(I2CChannel, O_RDWR)) < 0)
        return -1;
    if (ioctl(fd, I2C_SLAVE, I2CAddress) < 0)
        return -2;

    uint8_t data[8] = {0x00};
    if (write(fd, PCA9685_MODE1, 1) < 0)
        return -3;
    if (read(fd, data, 1) < 0)
        return -4;

    uint8_t settings = data[0] & 0x7f;
    uint8_t autoInc = settings | 0x20;
    if (write(fd, PCA9685_MODE1, 1) < 0)
        return -5;
    if (write(fd, &autoInc, 1))
        return -6;

    return 0;
};

inline int pca9685PWMFreq(int fd, float frequency)
{
    frequency = (frequency > 1526 ? 1526 : (frequency < 24 ? 24 : frequency));
    int prescale = (int)(25000000.0f / (4096 * frequency) - 0.5f);

    uint8_t data[8] = {0x00};
    if (write(fd, PCA9685_MODE1, 1) < 0)
        return -1;
    if (read(fd, data, 1) < 0)
        return -1;

    uint8_t settings = data[0] & 0x7f;
    uint8_t sleep = settings | 0x10;
    uint8_t wake = settings & 0xEF;
    uint8_t restart = wake | 0x80;

    if (write(fd, PCA9685_MODE1, 1) < 0)
        return -1;
    if (write(fd, &sleep, 1))
        return -1;

    if (write(fd, (uint8_t *)PCA9685_PRESCALE, 1) < 0)
        return -1;
    if (write(fd, &prescale, 1))
        return -1;

    if (write(fd, PCA9685_MODE1, 1) < 0)
        return -1;
    if (write(fd, &wake, 1))
        return -1;

    usleep(100);

    if (write(fd, PCA9685_MODE1, 1) < 0)
        return -1;
    if (write(fd, &restart, 1))
        return -1;

    return 0;
}

inline int pca9685PWMReset(int fd)
{
    uint16_t outdata = 0x0000;
    if (write(fd, (uint8_t *)LEDALL_ON_L, 1) < 0)
        return -1;
    if (write(fd, &outdata, 1))
        return -1;

    outdata = 0x1000;
    if (write(fd, (uint8_t *)(LEDALL_ON_L + 2), 1) < 0)
        return -1;
    if (write(fd, &outdata, 1))
        return -1;

    return 0;
}

inline int pca9685PWMWrite(int fd, int pin, int on, int off)
{
    uint16_t reg = baseReg(pin);

    uint16_t outdata = on & 0x0fff;
    if (write(fd, &reg, 1) < 0)
        return -1;
    if (write(fd, &outdata, 1))
        return -1;

    reg += 2;
    outdata = off & 0x0fff;
    if (write(fd, (uint16_t *)reg, 1) < 0)
        return -1;
    if (write(fd, &outdata, 1))
        return -1;

    return 0;
}

inline int pca9685PWMWriteS(int fd, int pin, int off)
{
    uint16_t reg = baseReg(pin) + 2;

    uint16_t outdata = off & 0x0fff;
    if (write(fd, (uint16_t *)reg, 1) < 0)
        return -1;
    if (write(fd, &outdata, 1))
        return -1;

    return 0;
}

inline int pca9685PWMResetON(int fd, int pin)
{
    int reg = baseReg(pin);

    if (write(fd, &reg, 1) < 0)
        return -1;
    if (write(fd, (uint8_t *)(0 & 0x0FFF), 1))
        return -1;

    return 0;
}