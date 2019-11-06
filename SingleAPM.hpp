#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <unistd.h>
#include <ctime>
#include <thread>
#include <cstdio>
#include "_thirdparty/pca9685.h"
#include "_thirdparty/pid.hpp"
//Setup
static int PCA9658_fd;
static int RCReader_fd;
static int PWM_Freq = 490;
static int PCA9685_PinBase = 65;
static int PCA9685_Address = 0x40;
//_uORB_Output_Pin
int _flag_A1_Pin = 0;
int _flag_A2_Pin = 1;
int _flag_B1_Pin = 2;
int _flag_B2_Pin = 3;

//Base_Flags
bool _flag_ForceFailed_Safe;
int _flag_Lazy_Throttle = 2300;
int _flag_Lock_Throttle = 2200;
int _flag_Middle_Yall = 1000;
int _flag_Middle_Roll = 1000;
int _flag_Middle_Pitch = 1000;
//REC_Reading_Yall_Pitch_Yoll_Throttle_Level
int data[36];
int _uORB_RC_Roll;
int _uORB_RC_Pitch;
int _uORB_RC_Throttle;
int _uORB_RC_Yall;

//AttitudeUpdate_Data
unsigned int _uORB_True_Roll[2];
unsigned int _uORB_True_Pitch[2];
unsigned int _uORB_True_Yall[2];
float _Tmp_Prenset_A1;
float _Tmp_Prenset_A2;
float _Tmp_Prenset_B1;
float _Tmp_Prenset_B2;

//MotorOutput_finally
int _uORB_A1_Speed;
int _uORB_A2_Speed;
int _uORB_B1_Speed;
int _uORB_B2_Speed;

class Manaul_Mode
{
public:
	Manaul_Mode()
	{
		RCReader_fd = serialOpen("/dev/ttyS0", 115200);
		if (RCReader_fd < 0)
		{
			std::cout << "[Controller] Serial open failed; \n";
		}
		PCA9658_fd = pca9685Setup(PCA9685_PinBase, PCA9685_Address, PWM_Freq);
		if (PCA9658_fd < 0)
		{
			std::cout << "[PCA9685] pca9685setup failed; \n";
		}
	}

	inline void ControlRead()
	{
		if (serialDataAvail(RCReader_fd) > 0)
		{
			for (int i = 0; i <= 34; i++)
			{
				data[i] = serialGetchar(RCReader_fd);
			}
		}
		_uORB_RC_Roll = data[1] * 255 + data[2];
		_uORB_RC_Pitch = data[3] * 255 + data[4];
		_uORB_RC_Throttle = data[5] * 255 + data[6];
		_uORB_RC_Yall = data[7] * 255 + data[8];
	}

	inline void AttitudeUpdate()
	{
		if (_uORB_RC_Roll - _flag_Middle_Roll > 0)
		{
			_uORB_True_Roll[0] = 0;
			_uORB_True_Roll[1] = ((float)_uORB_RC_Roll - (float)_flag_Middle_Roll) / (float)650 * (float)300;
		}
		else if (_uORB_RC_Roll - _flag_Middle_Roll <= 0)
		{
			_uORB_True_Roll[1] = 0;
			_uORB_True_Roll[0] = (-(float)_uORB_RC_Roll + (float)_flag_Middle_Roll) / (float)650 * (float)300;
		}


		if (_uORB_RC_Pitch - _flag_Middle_Pitch > 0)
		{
			_uORB_True_Pitch[0] = 0;
			_uORB_True_Pitch[1] = ((float)_uORB_RC_Pitch - (float)_flag_Middle_Pitch) / (float)650 * (float)300;
		}
		else if (_uORB_RC_Pitch - _flag_Middle_Pitch <= 0)
		{
			_uORB_True_Pitch[1] = 0;
			_uORB_True_Pitch[0] = (-(float)_uORB_RC_Pitch + (float)_flag_Middle_Pitch) / (float)650 * (float)300;
		}


		if (_uORB_RC_Yall - _flag_Middle_Yall > 0)
		{
			_uORB_True_Yall[0] = 0;
			_uORB_True_Yall[1] = ((float)_uORB_RC_Yall - (float)_flag_Middle_Yall) / (float)650 * (float)300;
		}
		else if (_uORB_RC_Yall - _flag_Middle_Yall <= 0)
		{
			_uORB_True_Yall[1] = 0;
			_uORB_True_Yall[0] = (-(float)_uORB_RC_Yall + (float)_flag_Middle_Yall) / (float)650 * (float)300;
		}

		_uORB_B1_Speed = _uORB_RC_Throttle - _uORB_True_Pitch[0] - _uORB_True_Roll[0] - _uORB_True_Yall[0];
		_uORB_A1_Speed = _uORB_RC_Throttle - _uORB_True_Pitch[1] - _uORB_True_Roll[0] - _uORB_True_Yall[1];
		_uORB_A2_Speed = _uORB_RC_Throttle - _uORB_True_Pitch[1] - _uORB_True_Roll[1] - _uORB_True_Yall[0];
		_uORB_B2_Speed = _uORB_RC_Throttle - _uORB_True_Pitch[0] - _uORB_True_Roll[1] - _uORB_True_Yall[1];
		_Tmp_Prenset_A1 = ((float)_uORB_A1_Speed - (float)300) / (float)1400;
		_Tmp_Prenset_A2 = ((float)_uORB_A2_Speed - (float)300) / (float)1400;
		_Tmp_Prenset_B1 = ((float)_uORB_B1_Speed - (float)300) / (float)1400;
		_Tmp_Prenset_B2 = ((float)_uORB_B2_Speed - (float)300) / (float)1400;
		_uORB_A1_Speed = 700 * _Tmp_Prenset_A1 + 2350;
		_uORB_A2_Speed = 700 * _Tmp_Prenset_A2 + 2350;
		_uORB_B1_Speed = 700 * _Tmp_Prenset_B1 + 2350;
		_uORB_B2_Speed = 700 * _Tmp_Prenset_B2 + 2350;
	}

	inline void MotorUpdate()
	{
		if (_flag_ForceFailed_Safe)
		{
			pca9685PWMWrite(PCA9658_fd, _flag_A1_Pin, 0, _flag_Lock_Throttle);
			pca9685PWMWrite(PCA9658_fd, _flag_A2_Pin, 0, _flag_Lock_Throttle);
			pca9685PWMWrite(PCA9658_fd, _flag_B1_Pin, 0, _flag_Lock_Throttle);
			pca9685PWMWrite(PCA9658_fd, _flag_B2_Pin, 0, _flag_Lock_Throttle);
		}
		if (!_flag_ForceFailed_Safe)
		{
			pca9685PWMWrite(PCA9658_fd, _flag_A1_Pin, 0,
				_uORB_A1_Speed);
			pca9685PWMWrite(PCA9658_fd, _flag_A2_Pin, 0,
				_uORB_A2_Speed);
			pca9685PWMWrite(PCA9658_fd, _flag_B1_Pin, 0,
				_uORB_B1_Speed);
			pca9685PWMWrite(PCA9658_fd, _flag_B2_Pin, 0,
				_uORB_B2_Speed);
		}
	}
};

class Stablize_Mode
{
public:
	const int MPU9250_ADDR = 0x68;
	int MPU9250_fd;

	int _uORB_MPU9250_Buf[14];
	int buf;

	int _uORB_MPU9250_A_X;
	int _uORB_MPU9250_A_Y;
	int _uORB_MPU9250_A_Z;
	int _uORB_MPU9250_M_X;
	int _uORB_MPU9250_M_Y;
	int _uORB_MPU9250_M_Z;
	int _uORB_MPU9250_X;
	int _uORB_MPU9250_Y;
	int _uORB_MPU9250_Z;
	Stablize_Mode()
	{
		MPU9250_fd = wiringPiI2CSetup(MPU9250_ADDR);
		if (MPU9250_fd < 0)
		{
			std::cout << "[Sensors] MPU9250 startUP failed; \n";
		}
		else
		{
			wiringPiI2CWriteReg16(MPU9250_fd, 0x37, 0x02);
			wiringPiI2CWriteReg16(MPU9250_fd, 29, 0x06);
			wiringPiI2CWriteReg16(MPU9250_fd, 28, 0x08);
			wiringPiI2CWriteReg16(MPU9250_fd, 27, 0x10);
			wiringPiI2CWriteReg16(MPU9250_fd, 26, 0x06);
			wiringPiI2CWriteReg16(0x0c, 0x37, 0x02);
		}

		RCReader_fd = serialOpen("/dev/ttyS0", 115200);
		if (RCReader_fd < 0)
		{
			std::cout << "[Controller] Serial open failed; \n";
		}
		PCA9658_fd = pca9685Setup(PCA9685_PinBase, PCA9685_Address, PWM_Freq);
		if (PCA9658_fd < 0)
		{
			std::cout << "[PCA9685] pca9685setup failed; \n";
		}
	}

	inline void SensorsRead()
	{
		buf = wiringPiI2CReadReg16(MPU9250_fd, 0x3B);
		std::cout << buf;

		//int16_t ax = -(_uORB_MPU9250_Buf[0] << 8 | _uORB_MPU9250_Buf[1]);
		//int16_t ay = -(_uORB_MPU9250_Buf[2] << 8 | _uORB_MPU9250_Buf[3]);
		//int16_t az = _uORB_MPU9250_Buf[4] << 8 | _uORB_MPU9250_Buf[5];
		//std::cout << ax << " ";
		//std::cout << ay << " ";
		//std::cout << az << "\r";

		//int16_t gx = -(_uORB_MPU9250_Buf[8] << 8 | _uORB_MPU9250_Buf[9]);
		//int16_t gy = -(_uORB_MPU9250_Buf[10] << 8 | _uORB_MPU9250_Buf[11]);
		//int16_t gz = _uORB_MPU9250_Buf[12] << 8 | _uORB_MPU9250_Buf[13];
	}

	inline void ControlRead()
	{
		if (serialDataAvail(RCReader_fd) > 0)
		{
			for (int i = 0; i <= 34; i++)
			{
				data[i] = serialGetchar(RCReader_fd);
			}
		}
		_uORB_RC_Roll = data[1] * 255 + data[2];
		_uORB_RC_Pitch = data[3] * 255 + data[4];
		_uORB_RC_Throttle = data[5] * 255 + data[6];
		_uORB_RC_Yall = data[7] * 255 + data[8];
	}

	inline void AttitudeUpdate()
	{
		if (_uORB_RC_Roll - _flag_Middle_Roll > 0)
		{
			_uORB_True_Roll[0] = 0;
			_uORB_True_Roll[1] = ((float)_uORB_RC_Roll - (float)_flag_Middle_Roll) / (float)650 * (float)300;
		}
		else if (_uORB_RC_Roll - _flag_Middle_Roll <= 0)
		{
			_uORB_True_Roll[1] = 0;
			_uORB_True_Roll[0] = (-(float)_uORB_RC_Roll + (float)_flag_Middle_Roll) / (float)650 * (float)300;
		}


		if (_uORB_RC_Pitch - _flag_Middle_Pitch > 0)
		{
			_uORB_True_Pitch[0] = 0;
			_uORB_True_Pitch[1] = ((float)_uORB_RC_Pitch - (float)_flag_Middle_Pitch) / (float)650 * (float)300;
		}
		else if (_uORB_RC_Pitch - _flag_Middle_Pitch <= 0)
		{
			_uORB_True_Pitch[1] = 0;
			_uORB_True_Pitch[0] = (-(float)_uORB_RC_Pitch + (float)_flag_Middle_Pitch) / (float)650 * (float)300;
		}


		if (_uORB_RC_Yall - _flag_Middle_Yall > 0)
		{
			_uORB_True_Yall[0] = 0;
			_uORB_True_Yall[1] = ((float)_uORB_RC_Yall - (float)_flag_Middle_Yall) / (float)650 * (float)300;
		}
		else if (_uORB_RC_Yall - _flag_Middle_Yall <= 0)
		{
			_uORB_True_Yall[1] = 0;
			_uORB_True_Yall[0] = (-(float)_uORB_RC_Yall + (float)_flag_Middle_Yall) / (float)650 * (float)300;
		}

		_uORB_B1_Speed = _uORB_RC_Throttle - _uORB_True_Pitch[0] - _uORB_True_Roll[0] - _uORB_True_Yall[0];
		_uORB_A1_Speed = _uORB_RC_Throttle - _uORB_True_Pitch[1] - _uORB_True_Roll[0] - _uORB_True_Yall[1];
		_uORB_A2_Speed = _uORB_RC_Throttle - _uORB_True_Pitch[1] - _uORB_True_Roll[1] - _uORB_True_Yall[0];
		_uORB_B2_Speed = _uORB_RC_Throttle - _uORB_True_Pitch[0] - _uORB_True_Roll[1] - _uORB_True_Yall[1];
		_Tmp_Prenset_A1 = ((float)_uORB_A1_Speed - (float)300) / (float)1400;
		_Tmp_Prenset_A2 = ((float)_uORB_A2_Speed - (float)300) / (float)1400;
		_Tmp_Prenset_B1 = ((float)_uORB_B1_Speed - (float)300) / (float)1400;
		_Tmp_Prenset_B2 = ((float)_uORB_B2_Speed - (float)300) / (float)1400;
		_uORB_A1_Speed = 700 * _Tmp_Prenset_A1 + 2350;
		_uORB_A2_Speed = 700 * _Tmp_Prenset_A2 + 2350;
		_uORB_B1_Speed = 700 * _Tmp_Prenset_B1 + 2350;
		_uORB_B2_Speed = 700 * _Tmp_Prenset_B2 + 2350;
	}

	inline void MotorUpdate()
	{
		if (_flag_ForceFailed_Safe)
		{
			pca9685PWMWrite(PCA9658_fd, _flag_A1_Pin, 0, _flag_Lock_Throttle);
			pca9685PWMWrite(PCA9658_fd, _flag_A2_Pin, 0, _flag_Lock_Throttle);
			pca9685PWMWrite(PCA9658_fd, _flag_B1_Pin, 0, _flag_Lock_Throttle);
			pca9685PWMWrite(PCA9658_fd, _flag_B2_Pin, 0, _flag_Lock_Throttle);
		}
		if (!_flag_ForceFailed_Safe)
		{
			pca9685PWMWrite(PCA9658_fd, _flag_A1_Pin, 0,
				_uORB_A1_Speed);
			pca9685PWMWrite(PCA9658_fd, _flag_A2_Pin, 0,
				_uORB_A2_Speed);
			pca9685PWMWrite(PCA9658_fd, _flag_B1_Pin, 0,
				_uORB_B1_Speed);
			pca9685PWMWrite(PCA9658_fd, _flag_B2_Pin, 0,
				_uORB_B2_Speed);
		}
	}
};