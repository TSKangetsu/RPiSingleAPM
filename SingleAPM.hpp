#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <unistd.h>
#include <ctime>
#include <cstdio>
#include "_thirdparty/pca9685.h"
#include "_thirdparty/pid.h"
//Setup
static int PCA9658_fd;
static int PWM_Freq = 490;
static int PCA9685_PinBase = 65;
static int PCA9685_Address = 0x40;
//Base_Flags
bool _uORB_Start;
bool _uORB_Stop;
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
int _flag_Middle_Yall = 2650;
int _flag_Middle_Yoll = 2650;
int _flag_Middle_Pitch = 2650;
//MotorOutput_finally
int _uORB_A1_Speed;
int _uORB_A2_Speed;
int _uORB_B1_Speed;
int _uORB_B2_Speed;
//REC_Reading_Yall_Pitch_Yoll_Throttle_Level, Max = 800
int _uORB_REC_Yall_Level;
int _uORB_REC_Pitch_Level;
int _uORB_REC_Yoll_Level;
int _uORB_REC_Throttle_Level;
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

//Manaull Mode
class Manaul_Mode
{
public:
	Manaul_Mode()
	{
		PCA9658_fd = pca9685Setup(PCA9685_PinBase, PCA9685_Address, PWM_Freq);
	}

	inline void ControlRead()
	{
		_uORB_A1_Speed = _uORB_REC_Throttle_Level
			+ (_uORB_REC_Yall_Level - _flag_Middle_Yall) / 2
			+ (_flag_Middle_Yoll - _uORB_REC_Yoll_Level) / 2
			+ (_flag_Middle_Pitch - _uORB_REC_Pitch_Level) / 2;
		_uORB_A2_Speed = _uORB_REC_Throttle_Level
			+ (_flag_Middle_Yall - _uORB_REC_Yall_Level) / 2
			+ (_flag_Middle_Yoll - _uORB_REC_Yoll_Level) / 2
			+ (_uORB_REC_Pitch_Level - _flag_Middle_Pitch) / 2;
		_uORB_B1_Speed = _uORB_REC_Throttle_Level
			+ (_flag_Middle_Yall - _uORB_REC_Yall_Level) / 2
			+ (_uORB_REC_Yoll_Level - _flag_Middle_Yoll) / 2
			+ (_flag_Middle_Pitch - _uORB_REC_Pitch_Level) / 2;
		_uORB_B2_Speed = _uORB_REC_Throttle_Level
			+ (_uORB_REC_Yall_Level - _flag_Middle_Yall) / 2
			+ (_uORB_REC_Yoll_Level - _flag_Middle_Yoll) / 2
			+ (_uORB_REC_Pitch_Level - _flag_Middle_Pitch) / 2;
	}

	inline void MotorUpdate()
	{
		if (_uORB_Start && !_uORB_Stop)
		{
			pca9685PWMWrite(PCA9658_fd, _flag_A1_Pin, _flag_Lazy_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _flag_A2_Pin, _flag_Lazy_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _flag_B1_Pin, _flag_Lazy_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _flag_B2_Pin, _flag_Lazy_Throttle, 0);
		}
		if (!_uORB_Start && _uORB_Stop)
		{
			pca9685PWMWrite(PCA9658_fd, _flag_A1_Pin, _flag_Lock_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _flag_A2_Pin, _flag_Lock_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _flag_B1_Pin, _flag_Lock_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _flag_B2_Pin, _flag_Lock_Throttle, 0);
		}
		else
		{
			pca9685PWMWrite(PCA9658_fd, _flag_A1_Pin,
				_uORB_A1_Speed,
				0);
			pca9685PWMWrite(PCA9658_fd, _flag_A2_Pin,
				_uORB_A2_Speed,
				0);
			pca9685PWMWrite(PCA9658_fd, _flag_B1_Pin,
				_uORB_B1_Speed,
				0);
			pca9685PWMWrite(PCA9658_fd, _flag_B2_Pin,
				_uORB_B2_Speed,
				0);
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

	}
	inline void MotorUpdate()
	{
		if (_uORB_Start && !_uORB_Stop)
		{
			pca9685PWMWrite(PCA9658_fd, _flag_A1_Pin, _flag_Lazy_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _flag_A2_Pin, _flag_Lazy_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _flag_B1_Pin, _flag_Lazy_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _flag_B2_Pin, _flag_Lazy_Throttle, 0);
		}
		if (!_uORB_Start && _uORB_Stop)
		{
			pca9685PWMWrite(PCA9658_fd, _flag_A1_Pin, _flag_Lock_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _flag_A2_Pin, _flag_Lock_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _flag_B1_Pin, _flag_Lock_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _flag_B2_Pin, _flag_Lock_Throttle, 0);
		}
		else
		{
			pca9685PWMWrite(PCA9658_fd, _flag_A1_Pin,
				_uORB_A1_Speed,
				0);
			pca9685PWMWrite(PCA9658_fd, _flag_A2_Pin,
				_uORB_A2_Speed,
				0);
			pca9685PWMWrite(PCA9658_fd, _flag_B1_Pin,
				_uORB_B1_Speed,
				0);
			pca9685PWMWrite(PCA9658_fd, _flag_B2_Pin,
				_uORB_B2_Speed,
				0);
		}
	}
};