#include <iostream>
#include <math.h>
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

class Stablize_Mode
{
public:
	const int MPU9250_ADDR = 0x68;
	int MPU9250_fd;
	int _Tmp_MPU9250_Buffer[14];

	long _uORB_MPU9250_A_X;
	long _uORB_MPU9250_A_Y;
	long _uORB_MPU9250_A_Z;
	long _Tmp_IMU_Accel_Vector;

	long _uORB_MPU9250_G_X;
	long _uORB_MPU9250_G_Y;
	long _uORB_MPU9250_G_Z;

	long _uORB_MPU9250_G_X_Cali;
	long _uORB_MPU9250_G_Y_Cali;
	long _uORB_MPU9250_G_Z_Cali;

	double _uORB_Angel_Pitch;
	double _uORB_Angel__Roll;

	double _uORB_Gryo_Pitch;
	double _uORB_Gryo__Roll;

	double _uORB_Real_Pitch;
	double _uORB_Real_Roll;

	Stablize_Mode()
	{
		MPU9250_fd = wiringPiI2CSetup(MPU9250_ADDR);
		if (MPU9250_fd < 0)
		{
			std::cout << "[Sensors] MPU9250 startUP failed; \n";
		}
		else
		{
			wiringPiI2CWriteReg8(MPU9250_fd, 107, 0x00); //reset
			wiringPiI2CWriteReg8(MPU9250_fd, 28, 0x10); //Accel
			wiringPiI2CWriteReg8(MPU9250_fd, 27, 0x08); // Gryo
			wiringPiI2CWriteReg8(MPU9250_fd, 26, 0x03); //config
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

	inline void SensorsGryoCalibration()
	{
		std::cout << "[Sensors] Gyro Calibration ......" << "\n";
		for (int cali_count = 0; cali_count < 2000; cali_count++)
		{
			SensorsDataRead();
			_uORB_MPU9250_G_X_Cali += _uORB_MPU9250_G_X;
			_uORB_MPU9250_G_Y_Cali += _uORB_MPU9250_G_Y;
			_uORB_MPU9250_G_Z_Cali += _uORB_MPU9250_G_Z;
			usleep(3);
		}
		_uORB_MPU9250_G_X_Cali = _uORB_MPU9250_G_X_Cali / 2000;
		_uORB_MPU9250_G_Y_Cali = _uORB_MPU9250_G_Y_Cali / 2000;
		_uORB_MPU9250_G_Z_Cali = _uORB_MPU9250_G_Z_Cali / 2000;
	}

	inline void SensorsParse()
	{
		SensorsDataRead();
		_uORB_MPU9250_G_X -= _uORB_MPU9250_G_X_Cali;
		_uORB_MPU9250_G_Y -= _uORB_MPU9250_G_Y_Cali;
		_uORB_MPU9250_G_Z -= _uORB_MPU9250_G_Z_Cali;

		_Tmp_IMU_Accel_Vector = sqrt((_uORB_MPU9250_A_X * _uORB_MPU9250_A_X) + (_uORB_MPU9250_A_Y * _uORB_MPU9250_A_Y) + (_uORB_MPU9250_A_Z * _uORB_MPU9250_A_Z));
		_uORB_Angel__Roll = asin((float)_uORB_MPU9250_A_X / _Tmp_IMU_Accel_Vector) * 57.296;
		_uORB_Angel_Pitch = asin((float)_uORB_MPU9250_A_Y / _Tmp_IMU_Accel_Vector) * -57.296;
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

private:
	inline void SensorsDataRead()
	{
		_Tmp_MPU9250_Buffer[0] = wiringPiI2CReadReg8(MPU9250_fd, 0x3B);
		_Tmp_MPU9250_Buffer[1] = wiringPiI2CReadReg8(MPU9250_fd, 0x3C);
		_uORB_MPU9250_A_X = (_Tmp_MPU9250_Buffer[0] << 8 | _Tmp_MPU9250_Buffer[1]);
		_Tmp_MPU9250_Buffer[2] = wiringPiI2CReadReg8(MPU9250_fd, 0x3D);
		_Tmp_MPU9250_Buffer[3] = wiringPiI2CReadReg8(MPU9250_fd, 0x3E);
		_uORB_MPU9250_A_Y = (_Tmp_MPU9250_Buffer[2] << 8 | _Tmp_MPU9250_Buffer[3]);
		_Tmp_MPU9250_Buffer[4] = wiringPiI2CReadReg8(MPU9250_fd, 0x3F);
		_Tmp_MPU9250_Buffer[5] = wiringPiI2CReadReg8(MPU9250_fd, 0x40);
		_uORB_MPU9250_A_Z = (_Tmp_MPU9250_Buffer[4] << 8 | _Tmp_MPU9250_Buffer[5]);

		_Tmp_MPU9250_Buffer[6] = wiringPiI2CReadReg8(MPU9250_fd, 0x43);
		_Tmp_MPU9250_Buffer[7] = wiringPiI2CReadReg8(MPU9250_fd, 0x44);
		_uORB_MPU9250_G_X = (_Tmp_MPU9250_Buffer[6] << 8 | _Tmp_MPU9250_Buffer[7]);
		_Tmp_MPU9250_Buffer[8] = wiringPiI2CReadReg8(MPU9250_fd, 0x45);
		_Tmp_MPU9250_Buffer[9] = wiringPiI2CReadReg8(MPU9250_fd, 0x46);
		_uORB_MPU9250_G_Y = (_Tmp_MPU9250_Buffer[8] << 8 | _Tmp_MPU9250_Buffer[9]);
		_Tmp_MPU9250_Buffer[10] = wiringPiI2CReadReg8(MPU9250_fd, 0x47);
		_Tmp_MPU9250_Buffer[11] = wiringPiI2CReadReg8(MPU9250_fd, 0x48);
		_uORB_MPU9250_G_Z = (_Tmp_MPU9250_Buffer[10] << 8 | _Tmp_MPU9250_Buffer[11]);
	}
};