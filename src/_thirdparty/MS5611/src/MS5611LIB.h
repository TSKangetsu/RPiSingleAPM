#pragma once
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <wiringPi.h>
#define OSR_4096 10000
#define CMD_PROM_READ 0xA0
#define MS5611_ADDRESS 0x77
#define CONV_D1_4096 0x48
#define CONV_D2_4096 0x58
#define beta 0.96

class MS5611
{
public:
	inline bool MS5611Init()
	{
		if ((MS5611FD = open("/dev/i2c-1", O_RDWR)) < 0)
		{
			std::cout << "Failed to open the bus.\n";
			return false;
		}
		if (ioctl(MS5611FD, I2C_SLAVE, MS5611_ADDRESS) < 0)
		{
			std::cout << "Failed to acquire bus access and/or talk to slave.\n";
			return false;
		}
		if (write(MS5611FD, &RESET, 1) != 1)
		{
			std::cout << "write reg 8 bit Failed to write to the i2c bus.\n";
			return false;
		}
		usleep(10000);
		MS5611PROMSettle();
		return true;
	}

	inline void LocalPressureSetter(double SeaLevelPressure, int TEMPSKIPS)
	{
		LocalPressure = SeaLevelPressure;
		clockTimer = TEMPSKIPS;
		TEMPSKIP = TEMPSKIPS;
	}

	inline void MS5611PreReader(double result[2])
	{
		D1 = MS5611CONVReader(MS5611FD, CONV_D1_4096);
		D2 = MS5611CONVReader(MS5611FD, CONV_D2_4096);
		//cac
		dT = D2 - (uint32_t)C[5] * pow(2, 8);
		TEMP = (2000 + (dT * (int64_t)C[5] / pow(2, 23)));
		OFF = (int64_t)C[2] * pow(2, 16) + (dT * C[4]) / pow(2, 7);
		SENS = (int32_t)C[1] * pow(2, 15) + dT * C[3] / pow(2, 8);
		if (TEMP < 2000)
		{
			int32_t T1 = 0;
			int64_t OFF1 = 0;
			int64_t SENS1 = 0;
			T1 = pow((double)dT, 2) / 2147483648;
			OFF1 = 5 * pow(((double)TEMP - 2000), 2) / 2;
			SENS1 = 5 * pow(((double)TEMP - 2000), 2) / 4;
			if (TEMP < -1500)
			{
				OFF1 = OFF1 + 7 * pow(((double)TEMP + 1500), 2);
				SENS1 = SENS1 + 11 * pow(((double)TEMP + 1500), 2) / 2;
			}
			TEMP -= T1;
			OFF -= OFF1;
			SENS -= SENS1;
		}
		P = ((((int64_t)D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15));
		Pressure = (double)P / (double)100;
		Altitude = 44330.0f * (1.0f - pow((double)Pressure / (double)LocalPressure, 0.1902949f));
		result[0] = Pressure;
		result[1] = Altitude;
		//cac-=
	}

	inline void MS5611FastReader(double result[2])
	{
		long ret = 0;
		uint8_t D[] = {0, 0, 0};
		int h;
		char zero = 0x0;
		char output;
		if (clockTimer == TEMPSKIP)
		{
			clockTimer = 0;
			output = 0x58;
			write(MS5611FD, &output, 1);
			usleep(9800);
			write(MS5611FD, &zero, 1);
			read(MS5611FD, &D, 3);
			D2 = D[0] * (unsigned long)65536 + D[1] * (unsigned long)256 + D[2];
			dT = D2 - (uint32_t)C[5] * pow(2, 8);
			TEMP = (2000 + (dT * (int64_t)C[5] / pow(2, 23)));
			OFF = (int64_t)C[2] * pow(2, 16) + (dT * C[4]) / pow(2, 7);
			SENS = (int32_t)C[1] * pow(2, 15) + dT * C[3] / pow(2, 8);
			if (TEMP < 2000)
			{
				int32_t T1 = 0;
				int64_t OFF1 = 0;
				int64_t SENS1 = 0;
				T1 = pow((double)dT, 2) / 2147483648;
				OFF1 = 5 * pow(((double)TEMP - 2000), 2) / 2;
				SENS1 = 5 * pow(((double)TEMP - 2000), 2) / 4;
				if (TEMP < -1500)
				{
					OFF1 = OFF1 + 7 * pow(((double)TEMP + 1500), 2);
					SENS1 = SENS1 + 11 * pow(((double)TEMP + 1500), 2) / 2;
				}
				TEMP -= T1;
				OFF -= OFF1;
				SENS -= SENS1;
			}
		}
		else
		{
			output = 0x48;
			write(MS5611FD, &output, 1);
			usleep(9800);
			write(MS5611FD, &zero, 1);
			read(MS5611FD, &D, 3);
			D1 = D[0] * (unsigned long)65536 + D[1] * (unsigned long)256 + D[2];
			P = ((((int64_t)D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15));
			Pressure = (double)P / (double)100;
			PresureAvaTotal -= PresureAvaData[PresureClock];
			PresureAvaData[PresureClock] = Pressure;
			PresureAvaTotal += PresureAvaData[PresureClock];
			PresureClock++;
			if (PresureClock == TEMPSKIP)
				PresureClock = 0;
			result[0] = PresureAvaTotal / TEMPSKIP;
			result[1] = Altitude;
			clockTimer++;
		}
	}

private:
	int MS5611FD;
	char RESET = 0x1E;
	uint16_t C[7];
	uint32_t D1;
	uint32_t D2;
	//cac tmp
	int64_t dT;
	int32_t TEMP;
	int64_t OFF;
	int64_t SENS;
	int32_t P;
	//cac tmp-=
	double LocalPressure = 1023;
	double result[2];
	double Altitude;
	double Pressure;
	//cac 100hz
	int clockTimer = 8;
	int TEMPSKIP = 8;
	int PresureClock = 0;
	float PresureAvaData[50];
	float PresureAvaTotal;

	inline void MS5611PROMSettle()
	{
		for (int i = 0; i < 7; i++)
		{
			C[i] = MS5611PROMReader(MS5611FD, CMD_PROM_READ + (i * 2));
			usleep(1000);
		}
	}

	unsigned int MS5611PROMReader(int DA, char PROM_CMD)
	{
		uint16_t ret = 0;
		uint8_t r8b[] = {0, 0};
		if (write(DA, &PROM_CMD, 1) != 1)
		{
			std::cout << "read set reg Failed to write to the i2c bus.\n";
		}
		if (read(DA, r8b, 2) != 2)
		{
			std::cout << "Failed to read from the i2c bus.\n";
		}
		ret = r8b[0] * 256 + r8b[1];
		return ret;
	}

	long MS5611CONVReader(int DA, char CONV_CMD)
	{
		long ret = 0;
		uint8_t D[] = {0, 0, 0};
		int h;
		char zero = 0x0;
		if (write(DA, &CONV_CMD, 1) != 1)
		{
			std::cout << "write reg 8 bit Failed to write to the i2c bus.\n";
		}
		usleep(OSR_4096);
		if (write(DA, &zero, 1) != 1)
		{
			std::cout << "write reset 8 bit Failed to write to the i2c bus.\n";
		}
		h = read(DA, &D, 3);
		if (h != 3)
		{
			std::cout << "Failed to read from the i2c bus %d.\n";
		}
		ret = D[0] * (unsigned long)65536 + D[1] * (unsigned long)256 + D[2];
		return ret;
	}
};