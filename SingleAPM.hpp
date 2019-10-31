#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <unistd.h>
#include <ctime>
#include <cstdio>
#include "pca9685.h"
//Setup
static int PCA9658_fd;
static int PWM_Freq = 490;
static int PCA9685_PinBase = 65;
static int PCA9685_Address = 0x40;
//Base_Flags
bool _uORB_Start;
bool _uORB_Stop;
//_uORB_Output_Pin
int _uORB_A1_Pin = 0;
int _uORB_A2_Pin = 1;
int _uORB_B1_Pin = 2;
int _uORB_B2_Pin = 3;
//Throttle_Flag
int _uORB_Lazy_Throttle = 2000;
int _uORB_Base_Throttle = 2000;
int _uORB_Lock_Throttle = 2300;
//Flying Throttle , speed MAX = 800; speed MIN = 2100;
int _uORB_A1_Speed;
int _uORB_A2_Speed;
int _uORB_B1_Speed;
int _uORB_B2_Speed;
//REC_Reading_Yall_Pitch_Yoll_Throttle_Level
int _uORB_REC_Yall_Level;
int _uORB_REC_Pitch_Level;
int _uORB_REC_Yoll_Level;
int _uORB_REC_Throttle_Level;
//Manaull Mode
class Manaul_Mode
{

	Manaul_Mode()
	{
		PCA9658_fd = pca9685Setup(PCA9685_PinBase, PCA9685_Address, PWM_Freq);
	}
	inline void MotorUpdate()
	{
		if (_uORB_Start && !_uORB_Stop)
		{
			pca9685PWMWrite(PCA9658_fd, _uORB_A1_Pin, _uORB_Lazy_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _uORB_A2_Pin, _uORB_Lazy_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _uORB_B1_Pin, _uORB_Lazy_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _uORB_B2_Pin, _uORB_Lazy_Throttle, 0);
		}
		if (!_uORB_Start && _uORB_Stop)
		{
			pca9685PWMWrite(PCA9658_fd, _uORB_A1_Pin, _uORB_Lock_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _uORB_A2_Pin, _uORB_Lock_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _uORB_B1_Pin, _uORB_Lock_Throttle, 0);
			pca9685PWMWrite(PCA9658_fd, _uORB_B2_Pin, _uORB_Lock_Throttle, 0);
		}
		else
		{
			pca9685PWMWrite(PCA9658_fd, _uORB_A1_Pin,
				_uORB_A1_Speed,
				0);
			pca9685PWMWrite(PCA9658_fd, _uORB_A2_Pin,
				_uORB_A2_Speed,
				0);
			pca9685PWMWrite(PCA9658_fd, _uORB_B1_Pin,
				_uORB_B1_Speed,
				0);
			pca9685PWMWrite(PCA9658_fd, _uORB_B2_Pin,
				_uORB_B2_Speed,
				0);
		}
	}
};