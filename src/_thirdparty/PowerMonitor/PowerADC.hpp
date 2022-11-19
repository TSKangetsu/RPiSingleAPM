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

#define DEV_INA226_ID 0x5449
#define DEV_INA226_DIEID 0x2260

enum
{
    INA226_REG_CONFIGURATION = 0x00,
    INA226_REG_SHUNT_VOLTAGE = 0x01,
    INA226_REG_BUS_VOLTAGE = 0x02,
    INA226_REG_POWER = 0x03,
    INA226_REG_CURRENT = 0x04,
    INA226_REG_CALIBRATION = 0x05,
    INA226_REG_MASK_ENABLE = 0x06,
    INA226_REG_ALERT_LIMIT = 0x07,
    INA226_REG_MANUFACTURER = 0xFE,
    INA226_REG_DIE_ID = 0xFF,
};

#define INA226_RESET 0x8000
#define INA226_MASK_ENABLE_CVRF 0x0008

enum
{
    INA226_BIT_SHUNT = 0,
    INA226_BIT_BUS = 1,
    INA226_BIT_MODE = 2,
};

#define INA226_MODE_SHUNT 1
#define INA226_MODE_BUS 2
#define INA226_MODE_TRIGGERED 0
#define INA226_MODE_CONTINUOUS 4

enum
{
    INA226_MODE_OFF = 0,
    INA226_MODE_SHUNT_TRIGGERED = 1,
    INA226_MODE_BUS_TRIGGERED = 2,
    INA226_MODE_SHUNT_BUS_TRIGGERED = 3,
    INA226_MODE_OFF2 = 4,
    INA226_MODE_SHUNT_CONTINUOUS = 5,
    INA226_MODE_BUS_CONTINUOUS = 6,
    INA226_MODE_SHUNT_BUS_CONTINUOUS = 7,
};

enum
{
    INA226_TIME_01MS = 0, /* 140us */
    INA226_TIME_02MS = 1, /* 204us */
    INA226_TIME_03MS = 2, /* 332us */
    INA226_TIME_05MS = 3, /* 588us */
    INA226_TIME_1MS = 4,  /* 1.1ms */
    INA226_TIME_2MS = 5,  /* 2.115ms */
    INA226_TIME_4MS = 6,  /* 4.156ms */
    INA226_TIME_8MS = 7,  /* 8.244ms */
};

enum
{
    INA226_AVERAGES_1 = 0,
    INA226_AVERAGES_4 = 1,
    INA226_AVERAGES_16 = 2,
    INA226_AVERAGES_64 = 3,
    INA226_AVERAGES_128 = 4,
    INA226_AVERAGES_256 = 5,
    INA226_AVERAGES_512 = 6,
    INA226_AVERAGES_1024 = 7,
};

struct ADCConfig
{
    float MAXCurrent;
    float ROfShunt;
    //
    uint8_t ADCBus;
    uint8_t Shunttime;
    uint8_t Averages;
    uint8_t Mode;
};

struct ADCData
{
    float shunt_voltage = 0;
    float voltage = 0;
    float current = 0;
    float power = 0;
};

class PowerADC
{
public:
    inline PowerADC(const char *I2CChannel, uint8_t I2CAddress, ADCConfig config);
    inline ADCData ADCGetData();
    inline ~PowerADC() { close(ADCFD); };

private:
    uint8_t tmpbuffer[10] = {0x00};
    float current_lsb;
    int ADCFD;
};

PowerADC::PowerADC(const char *I2CChannel, uint8_t I2CAddress, ADCConfig config)
{
    if ((ADCFD = open(I2CChannel, O_RDWR)) < 0)
        throw -1;
    if (ioctl(ADCFD, I2C_SLAVE, I2CAddress) < 0)
        throw -1;
    if (ioctl(ADCFD, I2C_TIMEOUT, 0x01) < 0) // set to 10ms?
        throw -1;
    // Check INA226's ID
    {
        uint8_t WHOAMI = 0xFE;
        write(ADCFD, &WHOAMI, 1);
        read(ADCFD, tmpbuffer, 2);
        int whoAMI = tmpbuffer[0] << 8 | tmpbuffer[1];
        if (whoAMI != DEV_INA226_ID)
            throw -2;
    }

    {
        uint8_t WHOAMI = 0xFF;
        write(ADCFD, &WHOAMI, 1);
        read(ADCFD, tmpbuffer, 2);
        int whoAMI = tmpbuffer[0] << 8 | tmpbuffer[1];
        if (whoAMI != DEV_INA226_DIEID)
            throw -2;
    }
    // reset
    {
        uint16_t configs = INA226_RESET;
        uint8_t config_reg_revL = ((uint16_t)(configs << 8)) >> 8;
        uint8_t config_reg_revM = (uint8_t)(configs >> 8);
        uint8_t configsw[3] = {INA226_REG_CONFIGURATION, config_reg_revM, config_reg_revL};
        if (write(ADCFD, &configsw, 3) < 0)
            throw -3;
    }
    // turn off
    {
        uint16_t configs = INA226_MODE_OFF;
        uint8_t config_reg_revL = ((uint16_t)(configs << 8)) >> 8;
        uint8_t config_reg_revM = (uint8_t)(configs >> 8);
        uint8_t configsw[3] = {INA226_REG_CONFIGURATION, config_reg_revM, config_reg_revL};
        if (write(ADCFD, &configsw, 3) < 0)
            throw -3;
    }
    // Calibration apply
    {
        current_lsb = config.MAXCurrent / (1 << 15);
        float calib = 0.00512 / (current_lsb * config.ROfShunt);
        uint16_t calib_reg = (uint16_t)floorf(calib);
        current_lsb = 0.00512 / (config.ROfShunt * calib_reg);
        uint8_t calib_reg_revL = ((uint16_t)(calib_reg << 8)) >> 8;
        uint8_t calib_reg_revM = (uint8_t)(calib_reg >> 8);
        uint8_t caliw[3] = {INA226_REG_CALIBRATION, calib_reg_revM, calib_reg_revL};
        if (write(ADCFD, &caliw, 3) < 0)
            throw -3;
    }
    // config apply
    {
        uint16_t configs = (config.Averages << 9) | (config.ADCBus << 6) | (config.Shunttime << 3) | config.Mode;
        uint8_t config_reg_revL = ((uint16_t)(configs << 8)) >> 8;
        uint8_t config_reg_revM = (uint8_t)(configs >> 8);
        uint8_t configsw[3] = {INA226_REG_CONFIGURATION, config_reg_revM, config_reg_revL};
        if (write(ADCFD, &configsw, 3) < 0)
            throw -3;
    }
}

ADCData PowerADC::ADCGetData()
{
    ADCData data;
    {
        uint8_t wdata = INA226_REG_BUS_VOLTAGE;
        write(ADCFD, &wdata, 1);
        read(ADCFD, tmpbuffer, 2);
        data.voltage = (float)(tmpbuffer[0] << 8 | tmpbuffer[1]) * 1.25e-3;
    }

    {
        uint8_t wdata = INA226_REG_CURRENT;
        write(ADCFD, &wdata, 1);
        read(ADCFD, tmpbuffer, 2);
        data.current = (float)(tmpbuffer[0] << 8 | tmpbuffer[1]) * 1000.0 * current_lsb;
    }

    {
        uint8_t wdata = INA226_REG_POWER;
        write(ADCFD, &wdata, 1);
        read(ADCFD, tmpbuffer, 2);
        data.power = (float)(tmpbuffer[0] << 8 | tmpbuffer[1]) * 25000.0 * current_lsb;
    }

    {
        uint8_t wdata = INA226_REG_SHUNT_VOLTAGE;
        write(ADCFD, &wdata, 1);
        read(ADCFD, tmpbuffer, 2);
        data.shunt_voltage = (float)(tmpbuffer[0] << 8 | tmpbuffer[1]) * 2.5e-3;
    }
    return data;
}