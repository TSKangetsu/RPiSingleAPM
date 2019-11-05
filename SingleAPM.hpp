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
static int RECReader_fd;
static int PWM_Freq = 490;
static int PCA9685_PinBase = 65;
static int PCA9685_Address = 0x40;
//Base_Flags
bool _uORB_Start;
bool _uORB_Stop;
bool _uORB_ForceFailed_Safe;
//_uORB_Output_Pin
int _flag_A1_Pin = 0;
int _flag_A2_Pin = 1;
int _flag_B1_Pin = 2;
int _flag_B2_Pin = 3;
//Throttle_Flag
/*
	Throtlle define
	Max is 3000 , min is 2200 , startup throttle is 2300;
*/
int _flag_Lazy_Throttle = 2300;
int _flag_Lock_Throttle = 2200;
int _flag_Middle_Yall = 1000;
int _flag_Middle_Roll = 1000;
int _flag_Middle_Pitch = 1000;
//MotorOutput_finally
int _uORB_A1_Speed;
int _uORB_A2_Speed;
int _uORB_B1_Speed;
int _uORB_B2_Speed;
//REC_Reading_Yall_Pitch_Yoll_Throttle_Level
int data[36];
int _uORB_REC_roll;
int _uORB_REC_pitch;
int _uORB_REC_throttle;
int _uORB_REC_yall;
float _Tmp_Prenset_A1;
float _Tmp_Prenset_A2;
float _Tmp_Prenset_B1;
float _Tmp_Prenset_B2;
//SensorsRead
int _uORB_MPU9250_A_X;
int _uORB_MPU9250_A_Y;
int _uORB_MPU9250_A_Z;
int _uORB_MPU9250_M_X;
int _uORB_MPU9250_M_Y;
int _uORB_MPU9250_M_Z;
int _uORB_MPU9250_X;
int _uORB_MPU9250_Y;
int _uORB_MPU9250_Z;

//UNTest_FUNC------------------------------------------------//
unsigned int _UNTest_Roll[2];
unsigned int _UNTest_Pitch[2];
unsigned int _UNTest_Throttle[2];
unsigned int _UNTest_Yall[2];
//UNTest_FUNC------------------------------------------------//
//Manaull Mode
class Manaul_Mode
{
public:
	Manaul_Mode()
	{
		RECReader_fd = serialOpen("/dev/ttyS0", 115200);
		if (RECReader_fd < 0)
		{
			std::cout << "[Controller] Serial open failed; \n";
		}
		PCA9658_fd = pca9685Setup(PCA9685_PinBase, PCA9685_Address, PWM_Freq);
		if (PCA9658_fd < 0)
		{
			std::cout << "[PCA9685] pca9685setup failed";
		}
	}

	inline void ControlRead()
	{
		if (serialDataAvail(RECReader_fd) > 0)
		{
			for (int i = 0; i <= 34; i++)
			{
				data[i] = serialGetchar(RECReader_fd);
			}
		}
		_uORB_REC_roll = data[1] * 255 + data[2];
		_uORB_REC_pitch = data[3] * 255 + data[4];
		_uORB_REC_throttle = data[5] * 255 + data[6];
		_uORB_REC_yall = data[7] * 255 + data[8];
	}

	inline void AttitudeUpdate()
	{
		_uORB_B1_Speed = _uORB_REC_throttle + (_flag_Middle_Yall - _uORB_REC_pitch) / 4 + (_flag_Middle_Roll - _uORB_REC_roll) / 4 - (_flag_Middle_Yall - _uORB_REC_yall) / 4;
		_uORB_A1_Speed = _uORB_REC_throttle - (_flag_Middle_Yall - _uORB_REC_pitch) / 4 + (_flag_Middle_Roll - _uORB_REC_roll) / 4 + (_flag_Middle_Yall - _uORB_REC_yall) / 4;
		_uORB_A2_Speed = _uORB_REC_throttle - (_flag_Middle_Yall - _uORB_REC_pitch) / 4 - (_flag_Middle_Roll - _uORB_REC_roll) / 4 + (_flag_Middle_Yall - _uORB_REC_yall) / 4;
		_uORB_B2_Speed = _uORB_REC_throttle + (_flag_Middle_Yall - _uORB_REC_pitch) / 4 - (_flag_Middle_Roll - _uORB_REC_roll) / 4 - (_flag_Middle_Yall - _uORB_REC_yall) / 4;
		_Tmp_Prenset_A1 = ((float)_uORB_A1_Speed - (float)300) / (float)1400;
		_Tmp_Prenset_A2 = ((float)_uORB_A2_Speed - (float)300) / (float)1400;
		_Tmp_Prenset_B1 = ((float)_uORB_B1_Speed - (float)300) / (float)1400;
		_Tmp_Prenset_B2 = ((float)_uORB_B2_Speed - (float)300) / (float)1400;
		_uORB_A1_Speed = 700 * _Tmp_Prenset_A1 + 2300;
		_uORB_A2_Speed = 700 * _Tmp_Prenset_A2 + 2300;
		_uORB_B1_Speed = 700 * _Tmp_Prenset_B1 + 2300;
		_uORB_B2_Speed = 700 * _Tmp_Prenset_B2 + 2300;
	}

	//UNTest_FUNC------------------------------------------------//
	inline void AttitudeUpdate_Test()
	{
		if (_uORB_REC_roll - _flag_Middle_Roll > 0)
		{
			_UNTest_Roll[0] = 0;
			_UNTest_Roll[1] = ((float)_uORB_REC_roll - (float)_flag_Middle_Roll) / (float)650 * (float)300;
		}
		else if (_uORB_REC_roll - _flag_Middle_Roll <= 0)
		{
			_UNTest_Roll[1] = 0;
			_UNTest_Roll[0] = (-(float)_uORB_REC_roll + (float)_flag_Middle_Roll) / (float)650 * (float)300;
		}


		if (_uORB_REC_pitch - _flag_Middle_Pitch > 0)
		{
			_UNTest_Pitch[0] = 0;
			_UNTest_Pitch[1] = ((float)_uORB_REC_pitch - (float)_flag_Middle_Pitch) / (float)650 * (float)300;
		}
		else if (_uORB_REC_pitch - _flag_Middle_Pitch <= 0)
		{
			_UNTest_Pitch[1] = 0;
			_UNTest_Pitch[0] = (-(float)_uORB_REC_pitch + (float)_flag_Middle_Pitch) / (float)650 * (float)300;
		}


		if (_uORB_REC_yall - _flag_Middle_Yall > 0)
		{
			_UNTest_Yall[0] = 0;
			_UNTest_Yall[1] = ((float)_uORB_REC_yall - (float)_flag_Middle_Yall) / (float)650 * (float)300;
		}
		else if (_uORB_REC_yall - _flag_Middle_Yall <= 0)
		{
			_UNTest_Yall[1] = 0;
			_UNTest_Yall[0] = (-(float)_uORB_REC_yall + (float)_flag_Middle_Yall) / (float)650 * (float)300;
		}

		_uORB_B1_Speed = _uORB_REC_throttle - _UNTest_Pitch[0] - _UNTest_Roll[0] - _UNTest_Yall[0];
		_uORB_A1_Speed = _uORB_REC_throttle - _UNTest_Pitch[1] - _UNTest_Roll[0] - _UNTest_Yall[1];
		_uORB_A2_Speed = _uORB_REC_throttle - _UNTest_Pitch[1] - _UNTest_Roll[1] - _UNTest_Yall[0];
		_uORB_B2_Speed = _uORB_REC_throttle - _UNTest_Pitch[0] - _UNTest_Roll[1] - _UNTest_Yall[1];
		_Tmp_Prenset_A1 = ((float)_uORB_A1_Speed - (float)300) / (float)1400;
		_Tmp_Prenset_A2 = ((float)_uORB_A2_Speed - (float)300) / (float)1400;
		_Tmp_Prenset_B1 = ((float)_uORB_B1_Speed - (float)300) / (float)1400;
		_Tmp_Prenset_B2 = ((float)_uORB_B2_Speed - (float)300) / (float)1400;
		_uORB_A1_Speed = 700 * _Tmp_Prenset_A1 + 2300;
		_uORB_A2_Speed = 700 * _Tmp_Prenset_A2 + 2300;
		_uORB_B1_Speed = 700 * _Tmp_Prenset_B1 + 2300;
		_uORB_B2_Speed = 700 * _Tmp_Prenset_B2 + 2300;

	}
	//UNTest_FUNC------------------------------------------------//

	inline void MotorUpdate()
	{
		if (_uORB_ForceFailed_Safe)
		{
			pca9685PWMWrite(PCA9658_fd, _flag_A1_Pin, 0, _flag_Lock_Throttle);
			pca9685PWMWrite(PCA9658_fd, _flag_A2_Pin, 0, _flag_Lock_Throttle);
			pca9685PWMWrite(PCA9658_fd, _flag_B1_Pin, 0, _flag_Lock_Throttle);
			pca9685PWMWrite(PCA9658_fd, _flag_B2_Pin, 0, _flag_Lock_Throttle);
		}
		if (!_uORB_ForceFailed_Safe)
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
	Stablize_Mode()
	{
		PCA9658_fd = pca9685Setup(PCA9685_PinBase, PCA9685_Address, PWM_Freq);
	}

	inline void SensorsRead()
	{

	}

	inline void ControlRead()
	{
		if (serialDataAvail(RECReader_fd) > 0)
		{
			for (int i = 0; i <= 34; i++)
			{
				data[i] = serialGetchar(RECReader_fd);
			}
		}
		_uORB_REC_roll = data[1] * 255 + data[2];
		_uORB_REC_pitch = data[3] * 255 + data[4];
		_uORB_REC_throttle = data[5] * 255 + data[6];
		_uORB_REC_yall = data[7] * 255 + data[8];
	}

	inline void AttitudeUpdate()
	{
		_uORB_B1_Speed = _uORB_REC_throttle + (_flag_Middle_Yall - _uORB_REC_pitch) / 4 + (_flag_Middle_Roll - _uORB_REC_roll) / 4 - (_flag_Middle_Yall - _uORB_REC_yall) / 4;
		_uORB_A1_Speed = _uORB_REC_throttle - (_flag_Middle_Yall - _uORB_REC_pitch) / 4 + (_flag_Middle_Roll - _uORB_REC_roll) / 4 + (_flag_Middle_Yall - _uORB_REC_yall) / 4;
		_uORB_A2_Speed = _uORB_REC_throttle - (_flag_Middle_Yall - _uORB_REC_pitch) / 4 - (_flag_Middle_Roll - _uORB_REC_roll) / 4 + (_flag_Middle_Yall - _uORB_REC_yall) / 4;
		_uORB_B2_Speed = _uORB_REC_throttle + (_flag_Middle_Yall - _uORB_REC_pitch) / 4 - (_flag_Middle_Roll - _uORB_REC_roll) / 4 - (_flag_Middle_Yall - _uORB_REC_yall) / 4;
		_Tmp_Prenset_A1 = ((float)_uORB_A1_Speed - (float)300) / (float)1400;
		_Tmp_Prenset_A2 = ((float)_uORB_A2_Speed - (float)300) / (float)1400;
		_Tmp_Prenset_B1 = ((float)_uORB_B1_Speed - (float)300) / (float)1400;
		_Tmp_Prenset_B2 = ((float)_uORB_B2_Speed - (float)300) / (float)1400;
		_uORB_A1_Speed = 700 * _Tmp_Prenset_A1 + 2300;
		_uORB_A2_Speed = 700 * _Tmp_Prenset_A2 + 2300;
		_uORB_B1_Speed = 700 * _Tmp_Prenset_B1 + 2300;
		_uORB_B2_Speed = 700 * _Tmp_Prenset_B2 + 2300;
	}

	inline void MotorUpdate()
	{
		if (_uORB_ForceFailed_Safe)
		{
			pca9685PWMWrite(PCA9658_fd, _flag_A1_Pin, 0, _flag_Lock_Throttle);
			pca9685PWMWrite(PCA9658_fd, _flag_A2_Pin, 0, _flag_Lock_Throttle);
			pca9685PWMWrite(PCA9658_fd, _flag_B1_Pin, 0, _flag_Lock_Throttle);
			pca9685PWMWrite(PCA9658_fd, _flag_B2_Pin, 0, _flag_Lock_Throttle);
		}
		if (!_uORB_ForceFailed_Safe)
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