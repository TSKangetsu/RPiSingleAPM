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
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <iostream>

#define ADS1115_MV_6P144 0.187500
#define ADS1115_MV_4P096 0.125000
#define ADS1115_MV_2P048 0.062500 // default
#define ADS1115_MV_1P024 0.031250
#define ADS1115_MV_0P512 0.015625
#define ADS1115_MV_0P256 0.007813
#define ADS1115_MV_0P256B 0.007813
#define ADS1115_MV_0P256C 0.007813

class ADS111x
{
public:
    enum SL_RangeFSR
    {
        V6_144 = 0b000, // FSR = ±6.144V(1)
        V4_096 = 0b001, // FSR = ±4.096V(1)
        V2_048 = 0b010, // FSR = ±2.048V（默认）
        V1_024 = 0b011, // FSR = ±1.024V
        V0_512 = 0b100, // FSR = ±0.512V
        V0_256 = 0b101, // FSR = ±0.256V
        // V0_256 = 0b110, // FSR = ±0.256V
        // V0_256 = 0b111, // FSR = ±0.256V
    };

    enum SL_DataRateSPS
    {
        SPS_8 = 0b000,   // FSR = ±6.144V(1)
        SPS_16 = 0b001,  // FSR = ±4.096V(1)
        SPS_32 = 0b010,  // FSR = ±2.048V（默认）
        SPS_64 = 0b011,  // FSR = ±1.024V
        SPS_128 = 0b100, // FSR = ±0.512V
        SPS_250 = 0b101, // FSR = ±0.256V
        SPS_475 = 0b110, // FSR = ±0.256V
        SPS_860 = 0b111, // FSR = ±0.256V
    };

    struct ADS111xConfig
    {
        unsigned Pin = 0b100;                              // 4 - 7
        SL_RangeFSR Range = SL_RangeFSR::V4_096;           // 0 - 7
        SL_DataRateSPS DataRate = SL_DataRateSPS::SPS_860; // 0 - 7
    };

    ADS111x(const char *i2cChannel, uint8_t i2caddr = 0x49, uint8_t timeout = 0x01)
    {
        fd = open(i2cChannel, O_RDWR);
        fd < 0 ? throw -1 : NULL;
        //
        int err = -1;
        err = ioctl(fd, I2C_SLAVE, i2caddr);
        err = ioctl(fd, I2C_TIMEOUT, timeout);
        err < 0 ? throw -2 : NULL;
    }

    int ADS111xReadmV(ADS111xConfig config)
    {
        ADSconfigM = 0b10000000;
        ADSconfigL = 0b00000011;
        //
        ADSconfigM |= (config.Pin << 4) |
                      (config.Range << 1);
        ADSconfigL |= (config.DataRate << 5);

        uint8_t regconfig[] = {0x01, ADSconfigM, ADSconfigL};
        write(fd, &regconfig, 3);
        //

        // TODO: sps fix
        switch (config.DataRate)
        {
        case SPS_8:
            usleep(1.f / 8.f * 1000000.f * 2);
            break;
        case SPS_16:
            usleep(1.f / 16.f * 1000000.f * 2);
            break;
        case SPS_32:
            usleep(1.f / 32.f * 1000000.f * 2);
            break;
        case SPS_64:
            usleep(1.f / 64.f * 1000000.f * 2);
            break;
        case SPS_128:
            usleep(1.f / 128.f * 1000000.f * 2);
            break;
        case SPS_250:
            usleep(1.f / 250.f * 1000000.f * 2);
            break;
        case SPS_475:
            usleep(1.f / 475.f * 1000000.f * 2);
            break;
        case SPS_860:
            usleep(1.f / 860.f * 1000000.f * 2);
            break;
        }
        //
        uint8_t regconvert[] = {0x00};
        write(fd, &regconvert, 1);
        //
        uint8_t regdata[2];
        read(fd, &regdata, 2);
        uint16_t rawdata = (regdata[0] << 8 | regdata[1]);
        //
        switch (config.Range)
        {
        case V6_144:
            return rawdata * ADS1115_MV_6P144;
        case V4_096:
            return rawdata * ADS1115_MV_4P096;
        case V2_048:
            return rawdata * ADS1115_MV_2P048;
        case V1_024:
            return rawdata * ADS1115_MV_1P024;
        case V0_512:
            return rawdata * ADS1115_MV_0P512;
        case V0_256:
            return rawdata * ADS1115_MV_0P256;
        default:
            return -1;
        }
    }

private:
    int fd = -1;
    uint8_t ADSconfigM;
    uint8_t ADSconfigL;
};