
#pragma once
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <asm/ioctls.h>
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

inline int pca9685PWMFreq(int fd, float frequency)
{
    frequency = (frequency > 1526 ? 1526 : (frequency < 24 ? 24 : frequency));
    int prescale = (int)(25000000.0f / (4096 * frequency) - 0.5f);
    // Read
    uint8_t data[8] = {0x00};
    uint8_t reg = PCA9685_MODE1;
    if (write(fd, &reg, 1) < 0)
        return -1;
    if (read(fd, data, 1) < 0)
        return -1;

    uint8_t settings = data[0] & 0x7f;
    uint8_t sleep = settings | 0x10;
    uint8_t wake = settings & 0xEF;
    uint8_t restart = wake | 0x80;

    // Write
    uint8_t wdata[2] = {
        PCA9685_MODE1, sleep};
    if (write(fd, wdata, 2) < 0)
        return -1;
    // Write
    wdata[0] = PCA9685_PRESCALE;
    wdata[1] = prescale;
    if (write(fd, wdata, 2) < 0)
        return -1;
    // Write
    wdata[0] = PCA9685_MODE1;
    wdata[1] = wake;
    if (write(fd, wdata, 2) < 0)
        return -1;

    usleep(100);
    // Write
    wdata[0] = PCA9685_MODE1;
    wdata[1] = restart;
    if (write(fd, wdata, 2) < 0)
        return -1;

    return 0;
}

inline int pca9685Setup(const char *I2CChannel, uint8_t I2CAddress, int frequency)
{
    int fd = 0;
    if ((fd = open(I2CChannel, O_RDWR)) < 0)
        return -1;
    if (ioctl(fd, I2C_SLAVE, I2CAddress) < 0)
        return -2;

    if (ioctl(fd, I2C_TIMEOUT, 0x01) < 0) // set to 10ms?
        return -2;

    uint8_t data[8] = {0x00};
    // Read
    uint8_t reg = PCA9685_MODE1;
    if (write(fd, &reg, 1) < 0)
        return -3;
    if (read(fd, data, 1) < 0)
        return -4;
    // Write
    uint8_t settings = data[0] & 0x7f;
    uint8_t autoInc = settings | 0x20;
    data[0] = PCA9685_MODE1;
    data[1] = autoInc;
    if (write(fd, data, 2) < 0)
        return -6;
    //

    pca9685PWMFreq(fd, frequency);
    return fd;
};

inline int pca9685PWMReset(int fd)
{
    {
        uint8_t reset8 = 0x00;
        uint8_t wresetl[2] = {LEDALL_ON_L, reset8};
        if (write(fd, wresetl, 2) < 0)
            return -1;

        uint8_t resetRegH = LEDALL_ON_L | 0x01;
        uint8_t wresetM[2] = {resetRegH, reset8};
        if (write(fd, wresetl, 2) < 0)
            return -1;
    }

    {
        uint8_t resetRegL = LEDALL_ON_L | 0x02;
        uint8_t reset8 = 0x10;
        uint8_t wresetl[2] = {resetRegL, reset8};
        if (write(fd, wresetl, 2) < 0)
            return -1;

        uint8_t resetRegH = LEDALL_ON_L | 0x03;
        reset8 = 0x00;
        uint8_t wresetM[2] = {resetRegH, reset8};
        if (write(fd, wresetl, 2) < 0)
            return -1;
    }

    return 0;
}

inline int pca9685PWMWrite(int fd, int pin, int on, int off)
{
    {
        uint8_t reg = baseReg(pin) + 2;

        uint16_t outdata = off & 0x0fff;

        uint16_t Pusher = outdata << 8;
        uint8_t dataL = Pusher >> 8;
        uint8_t wdatal[2] = {reg, dataL};
        if (write(fd, wdatal, 2) < 0)
            return -1;

        uint8_t regS = (reg | 0x01);
        uint8_t dataM = outdata >> 8;
        uint8_t wdatam[2] = {regS, dataM};
        if (write(fd, wdatam, 2) < 0)
            return -1;
    }

    {
        uint8_t reg = baseReg(pin);

        uint16_t outdata = on & 0x0fff;

        uint8_t onRegl = reg;
        uint16_t Pusher = outdata << 8;
        uint8_t dataL = Pusher >> 8;
        uint8_t wdatal[2] = {onRegl, dataL};
        if (write(fd, wdatal, 2) < 0)
            return -1;

        uint8_t onRegh = reg | 0x01;
        uint8_t dataM = outdata >> 8;
        uint8_t wdatam[2] = {onRegh, dataM};
        if (write(fd, wdatam, 2) < 0)
            return -1;
    }

    return 0;
}

inline int pca9685PWMWriteS(int fd, int pin, int off)
{
    uint8_t reg = baseReg(pin) + 2;

    uint16_t outdata = off & 0x0fff;

    uint16_t Pusher = outdata << 8;
    uint8_t dataL = Pusher >> 8;
    uint8_t wdatal[2] = {reg, dataL};
    if (write(fd, wdatal, 2) < 0)
        return -1;

    uint8_t regS = (reg | 0x01);
    uint8_t dataM = outdata >> 8;
    uint8_t wdatam[2] = {regS, dataM};
    if (write(fd, wdatam, 2) < 0)
        return -1;

    return 0;
}

inline int pca9685PWMResetON(int fd, int pin)
{
    uint8_t reg = baseReg(pin);

    uint16_t outdata = 0 & 0x0fff;

    uint8_t onRegl = reg;
    uint16_t Pusher = outdata << 8;
    uint8_t dataL = Pusher >> 8;
    uint8_t wdatal[2] = {onRegl, dataL};
    if (write(fd, wdatal, 2) < 0)
        return -1;

    uint8_t onRegh = reg | 0x01;
    uint8_t dataM = outdata >> 8;
    uint8_t wdatam[2] = {onRegh, dataM};
    if (write(fd, wdatam, 2) < 0)
        return -1;

    return 0;
}