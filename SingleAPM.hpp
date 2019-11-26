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

//Base_Motor_Flags
bool _flag_ForceFailed_Safe = true;
int _flag_Lazy_Throttle = 2300;
int _flag_Lock_Throttle = 2200;

//RC_Base_Flags
int _flag_RC_Middle__Roll = 1000;
int _flag_RC_Middle_Pitch = 1000;
int _flag_RC_Middle__Yall = 1000;
int _flag_RC_Max__Roll = 0;
int _flag_RC_Max_Pitch = 0;
int _flag_RC_Max_Throttle = 0;
int _flag_RC_Max__Yall = 0;
int _flag_RC_Min__Roll = 0;
int _flag_RC_Min_Pitch = 0;
int _flag_RC_Min_Throttle = 0;
int _flag_RC_Min__Yall = 0;

//REC_Reading_Yall_Pitch_Yoll_Throttle_Level
int data[36];
int _uORB_RC__Roll;
int _uORB_RC_Pitch;
int _uORB_RC_Throttle;
int _uORB_RC__Yall;

//MotorOutput_finally
int _uORB_A1_Speed;
int _uORB_A2_Speed;
int _uORB_B1_Speed;
int _uORB_B2_Speed;
float _uORB_Leveling__Roll;
float _uORB_Leveling_Pitch;
float _uORB_Leveling__Yall;

//PID Args
float _flag_PID_P_Gain = 0.5;
float _flag_PID_I_Gain = 0;
float _flag_PID_D_Gain = 0;
float _uORB_PID_D_Last_Value__Roll = 0;
float _uORB_PID_D_Last_Value_Pitch = 0;
float _uORB_PID_D_Last_Value__Yall = 0;
float _uORB_PID_I_Last_Value__Roll = 0;
float _uORB_PID_I_Last_Value_Pitch = 0;
float _uORB_PID_I_Last_Value__Yall = 0;
float _flag_PID_I_Max__Value = 400;
float _flag_PID_Level_Max = 400;

class Stablize_Mode
{
public:
	const int MPU9250_ADDR = 0x68;

	int MPU9250_fd;
	int _Tmp_MPU9250_Buffer[14];

	bool _flag_first_StartUp = true;

	long _Tmp_IMU_Accel_Vector;

	long _uORB_MPU9250_A_X;
	long _uORB_MPU9250_A_Y;
	long _uORB_MPU9250_A_Z;
	unsigned long _Tmp_MPU9250_A_X;
	unsigned long _Tmp_MPU9250_A_Y;
	unsigned long _Tmp_MPU9250_A_Z;
	long _uORB_MPU9250_A_X_Cali = 0;
	long _uORB_MPU9250_A_Y_Cali = 0;

	long _uORB_MPU9250_G_X;
	long _uORB_MPU9250_G_Y;
	long _uORB_MPU9250_G_Z;
	unsigned long _Tmp_MPU9250_G_X;
	unsigned long _Tmp_MPU9250_G_Y;
	unsigned long _Tmp_MPU9250_G_Z;
	long _uORB_MPU9250_G_X_Cali = 0;
	long _uORB_MPU9250_G_Y_Cali = 0;
	long _uORB_MPU9250_G_Z_Cali = 0;

	float _uORB_Accel__Roll;
	float _uORB_Accel_Pitch;
	float _uORB_Gryo__Roll;
	float _uORB_Gryo_Pitch;
	float _uORB_Gryo__Yall;
	float _uORB_Real_Pitch;
	float _uORB_Real__Roll;

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
			wiringPiI2CWriteReg8(MPU9250_fd, 28, 0x10);  //Accel
			wiringPiI2CWriteReg8(MPU9250_fd, 27, 0x08);  //Gryo
			wiringPiI2CWriteReg8(MPU9250_fd, 26, 0x03);  //config
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
		std::cout << "Gryo_X_Caili :" << _uORB_MPU9250_G_X_Cali << "\n";
		std::cout << "Gryo_Y_Caili :" << _uORB_MPU9250_G_Y_Cali << "\n";
		std::cout << "Gryo_Z_Caili :" << _uORB_MPU9250_G_Z_Cali << "\n";

		std::cout << "[Sensors] Accel Calibration ......" << "\n";
		for (int cali_count = 0; cali_count < 2000; cali_count++)
		{
			SensorsDataRead();
			_uORB_MPU9250_A_X_Cali += _uORB_MPU9250_A_X;
			_uORB_MPU9250_A_Y_Cali += _uORB_MPU9250_A_Y;
			usleep(3);
		}
		_uORB_MPU9250_A_X_Cali = _uORB_MPU9250_A_X_Cali / 2000;
		_uORB_MPU9250_A_Y_Cali = _uORB_MPU9250_A_Y_Cali / 2000;
		std::cout << "Accel_X_Caili :" << _uORB_MPU9250_A_X_Cali << "\n";
		std::cout << "Accel_Y_Caili :" << _uORB_MPU9250_A_Y_Cali << "\n";
	}

	inline void SensorsParse()
	{
		SensorsDataRead();
		//Cail----------------------------------------------------------------------//
		//Acel_cali
		_uORB_MPU9250_A_X -= _uORB_MPU9250_A_X_Cali;
		_uORB_MPU9250_A_Y -= _uORB_MPU9250_A_Y_Cali;
		//Gryo_cail
		_uORB_MPU9250_G_X -= _uORB_MPU9250_G_X_Cali;
		_uORB_MPU9250_G_Y -= _uORB_MPU9250_G_Y_Cali;
		_uORB_MPU9250_G_Z -= _uORB_MPU9250_G_Z_Cali;
		//Gryo----------------------------------------------------------------------//
		_uORB_Gryo__Roll = (_uORB_Gryo__Roll * 0.7) + ((_uORB_MPU9250_G_X / 65.5) * 0.3);
		_uORB_Gryo_Pitch = (_uORB_Gryo_Pitch * 0.7) + ((_uORB_MPU9250_G_Y / 65.5) * 0.3);
		_uORB_Gryo__Yall = (_uORB_Gryo__Yall * 0.7) + ((_uORB_MPU9250_G_Z / 65.5) * 0.3);
		//ACCEL---------------------------------------------------------------------//
		_Tmp_IMU_Accel_Vector = sqrt((_uORB_MPU9250_A_X * _uORB_MPU9250_A_X) + (_uORB_MPU9250_A_Y * _uORB_MPU9250_A_Y) + (_uORB_MPU9250_A_Z * _uORB_MPU9250_A_Z));
		if (abs(_uORB_MPU9250_A_X) < _Tmp_IMU_Accel_Vector)
			_uORB_Accel_Pitch = asin((float)_uORB_MPU9250_A_X / _Tmp_IMU_Accel_Vector) * 57.296;
		if (abs(_uORB_MPU9250_A_Y) < _Tmp_IMU_Accel_Vector)
			_uORB_Accel__Roll = asin((float)_uORB_MPU9250_A_Y / _Tmp_IMU_Accel_Vector) * -57.296;
		//Gryo_MIX_ACCEL------------------------------------------------------------//
		_uORB_Real_Pitch += _uORB_MPU9250_G_Y * 0.0000611;
		_uORB_Real__Roll += _uORB_MPU9250_G_X * 0.0000611;
		_uORB_Real_Pitch -= _uORB_Real__Roll * sin(_uORB_MPU9250_G_Z * 0.000001066);
		_uORB_Real__Roll += _uORB_Real_Pitch * sin(_uORB_MPU9250_G_Z * 0.000001066);
		if (!_flag_first_StartUp)
		{
			_uORB_Real_Pitch = _uORB_Real_Pitch * 0.9 + _uORB_Accel_Pitch * 0.1;
			_uORB_Real__Roll = _uORB_Real__Roll * 0.9 + _uORB_Accel__Roll * 0.1;
		}
		else
		{
			_uORB_Real_Pitch = _uORB_Accel_Pitch;
			_uORB_Real__Roll = _uORB_Accel__Roll;
			_flag_first_StartUp = false;
		}
	}

	inline void ControlCalibration()
	{
		std::cout << "[Controller] ControlCalibraion start ......";
		for (int cali_count = 0; cali_count < 2000; cali_count++)
		{
			ControlRead();
			if (_uORB_RC__Roll > _flag_RC_Max__Roll)
				_flag_RC_Max__Roll = _uORB_RC__Roll;
			if (_uORB_RC_Pitch > _flag_RC_Max_Pitch)
				_flag_RC_Max_Pitch = _uORB_RC_Pitch;
			if (_uORB_RC_Throttle > _flag_RC_Max_Throttle)
				_flag_RC_Max_Throttle = _uORB_RC_Throttle;
			if (_uORB_RC__Yall > _flag_RC_Max__Yall)
				_flag_RC_Max__Yall = _uORB_RC__Yall;

			if (_uORB_RC__Roll > _flag_RC_Min__Roll)
				_flag_RC_Min__Roll = _uORB_RC__Roll;
			if (_uORB_RC_Pitch > _flag_RC_Min_Pitch)
				_flag_RC_Min_Pitch = _uORB_RC_Pitch;
			if (_uORB_RC_Throttle > _flag_RC_Min_Throttle)
				_flag_RC_Min_Throttle = _uORB_RC_Throttle;
			if (_uORB_RC__Yall > _flag_RC_Min__Yall)
				_flag_RC_Min__Yall = _uORB_RC__Yall;
		}
		std::cout << "[Controller] Calibration will finshed"
			<< " Please Check the tick middle and throttle down and pass enter" << "\n";
		std::cin;
		_flag_RC_Middle_Pitch = _uORB_RC_Pitch;
		_flag_RC_Middle__Roll = _uORB_RC__Roll;
		_flag_RC_Middle__Yall = _uORB_RC__Yall;
		std::cout << "[Controler] Controller calitbration comfirm:"
			<< "Max__Roll    = " << _flag_RC_Max__Roll << "\n"
			<< "Max_Pitch    = " << _flag_RC_Max_Pitch << "\n"
			<< "Max_Throttle = " << _flag_RC_Max_Throttle << "\n"
			<< "Max__Yall    = " << _flag_RC_Max__Yall << "\n\n"

			<< "Middle__Roll = " << _flag_RC_Middle__Roll << "\n"
			<< "Middle_Pitch = " << _flag_RC_Middle_Pitch << "\n"
			<< "Middle__Yall = " << _flag_RC_Middle__Yall << "\n\n"

			<< "Min__Roll    = " << _flag_RC_Min_Pitch << "\n"
			<< "Min_Pitch    = " << _flag_RC_Min_Pitch << "\n"
			<< "Min_Throttle = " << _flag_RC_Min_Pitch << "\n"
			<< "Min__Yall    = " << _flag_RC_Min_Pitch << "\n";
		std::cout << "-----------calibration_over----------------";
	}

	inline void ControlParse()
	{
		ControlRead();

		if (_uORB_RC__Roll < _flag_RC_Middle__Roll + 20 && _uORB_RC__Roll < _flag_RC_Middle__Roll - 20)
			_uORB_RC__Roll = 0;
		else
			_uORB_RC__Roll -= _flag_RC_Middle__Roll;

		if (_uORB_RC_Pitch < _flag_RC_Middle_Pitch + 20 && _uORB_RC_Pitch < _flag_RC_Middle_Pitch - 20)
			_uORB_RC_Pitch = 0;
		else
			_uORB_RC_Pitch -= _flag_RC_Middle_Pitch;

		if (_uORB_RC__Yall < _flag_RC_Middle__Yall + 20 && _uORB_RC__Yall < _flag_RC_Middle__Yall - 20)
			_uORB_RC__Yall = 0;
		else
			_uORB_RC__Yall -= _flag_RC_Middle__Yall;
	}

	inline void AttitudeUpdate()
	{
		//Roll PID Mix
		PID_Caculate(_uORB_Gryo__Roll -= _uORB_Real__Roll * 15 + _uORB_RC__Roll,
			_uORB_Leveling__Roll, _uORB_PID_I_Last_Value__Roll, _uORB_PID_D_Last_Value__Roll);
		if (_uORB_Leveling__Roll > _flag_PID_Level_Max)
			_uORB_Leveling__Roll = _flag_PID_Level_Max;
		if (_uORB_Leveling__Roll < _flag_PID_Level_Max * -1)
			_uORB_Leveling__Roll = _flag_PID_Level_Max * -1;

		//Pitch PID Mix
		PID_Caculate(_uORB_Gryo_Pitch -= _uORB_Real_Pitch * 15 + _uORB_RC_Pitch,
			_uORB_Leveling_Pitch, _uORB_PID_I_Last_Value_Pitch, _uORB_PID_D_Last_Value_Pitch);
		if (_uORB_Leveling_Pitch > _flag_PID_Level_Max)
			_uORB_Leveling_Pitch = _flag_PID_Level_Max;
		if (_uORB_Leveling_Pitch < _flag_PID_Level_Max * -1)
			_uORB_Leveling_Pitch = _flag_PID_Level_Max * -1;

		//Yall PID Mix
		PID_Caculate(_uORB_Gryo__Yall + _uORB_RC__Yall,
			_uORB_Leveling__Yall, _uORB_PID_I_Last_Value__Yall, _uORB_PID_D_Last_Value__Yall);
		if (_uORB_Leveling__Yall > _flag_PID_Level_Max)
			_uORB_Leveling__Yall = _flag_PID_Level_Max;
		if (_uORB_Leveling__Yall < _flag_PID_Level_Max * -1)
			_uORB_Leveling__Yall = _flag_PID_Level_Max * -1;

		_uORB_A1_Speed = _uORB_RC_Throttle - _uORB_Leveling__Roll - _uORB_Leveling_Pitch - _uORB_Leveling__Yall;
		_uORB_A2_Speed = _uORB_RC_Throttle + _uORB_Leveling__Roll - _uORB_Leveling_Pitch + _uORB_Leveling__Yall;
		_uORB_B1_Speed = _uORB_RC_Throttle - _uORB_Leveling__Roll + _uORB_Leveling_Pitch + _uORB_Leveling__Yall;
		_uORB_B2_Speed = _uORB_RC_Throttle + _uORB_Leveling__Roll + _uORB_Leveling_Pitch - _uORB_Leveling__Yall;
	}

	inline void MotorUpdate()
	{
		_uORB_A1_Speed = (700 * (((float)_uORB_A1_Speed - (float)300) / (float)1400)) + 2350;
		_uORB_A1_Speed = (700 * (((float)_uORB_A1_Speed - (float)300) / (float)1400)) + 2350;
		_uORB_A1_Speed = (700 * (((float)_uORB_A1_Speed - (float)300) / (float)1400)) + 2350;
		_uORB_A1_Speed = (700 * (((float)_uORB_A1_Speed - (float)300) / (float)1400)) + 2350;

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
	inline void PID_Caculate(float inputData, float& outputData, float& last_I_Data, float& last_D_Data)
	{
		//P caculate
		outputData = _flag_PID_P_Gain * inputData;
		//D caculate
		outputData += _flag_PID_D_Gain * (inputData - last_D_Data);
		last_D_Data = inputData;
		//I caculate
		last_I_Data += inputData * _flag_PID_I_Gain;
		if (last_I_Data > _flag_PID_I_Max__Value)
			last_I_Data = _flag_PID_I_Max__Value;
		if (last_I_Data < _flag_PID_I_Max__Value * -1)
			last_I_Data = _flag_PID_I_Max__Value * -1;
		//P_I_D Mix OUTPUT
		outputData += last_I_Data;
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
		_uORB_RC__Roll = data[1] * 255 + data[2];
		_uORB_RC_Pitch = data[3] * 255 + data[4];
		_uORB_RC_Throttle = data[5] * 255 + data[6];
		_uORB_RC__Yall = data[7] * 255 + data[8];
		if (data[9] + data[10] > 1500)
		{
			_flag_ForceFailed_Safe = false;
		}
	}

	inline void SensorsDataRead()
	{
		_Tmp_MPU9250_Buffer[0] = wiringPiI2CReadReg8(MPU9250_fd, 0x3B);
		_Tmp_MPU9250_Buffer[1] = wiringPiI2CReadReg8(MPU9250_fd, 0x3C);
		_Tmp_MPU9250_A_X = (_Tmp_MPU9250_Buffer[0] << 8 | _Tmp_MPU9250_Buffer[1]);
		_uORB_MPU9250_A_X = (short)_Tmp_MPU9250_A_X;
		_Tmp_MPU9250_Buffer[2] = wiringPiI2CReadReg8(MPU9250_fd, 0x3D);
		_Tmp_MPU9250_Buffer[3] = wiringPiI2CReadReg8(MPU9250_fd, 0x3E);
		_Tmp_MPU9250_A_Y = (_Tmp_MPU9250_Buffer[2] << 8 | _Tmp_MPU9250_Buffer[3]);
		_uORB_MPU9250_A_Y = (short)_Tmp_MPU9250_A_Y;
		_Tmp_MPU9250_Buffer[4] = wiringPiI2CReadReg8(MPU9250_fd, 0x3F);
		_Tmp_MPU9250_Buffer[5] = wiringPiI2CReadReg8(MPU9250_fd, 0x40);
		_Tmp_MPU9250_A_Z = (_Tmp_MPU9250_Buffer[4] << 8 | _Tmp_MPU9250_Buffer[5]);
		_uORB_MPU9250_A_Z = (short)_Tmp_MPU9250_A_Z;

		_Tmp_MPU9250_Buffer[6] = wiringPiI2CReadReg8(MPU9250_fd, 0x43);
		_Tmp_MPU9250_Buffer[7] = wiringPiI2CReadReg8(MPU9250_fd, 0x44);
		_Tmp_MPU9250_G_X = (_Tmp_MPU9250_Buffer[6] << 8 | _Tmp_MPU9250_Buffer[7]);
		_uORB_MPU9250_G_X = (short)_Tmp_MPU9250_G_X;
		_Tmp_MPU9250_Buffer[8] = wiringPiI2CReadReg8(MPU9250_fd, 0x45);
		_Tmp_MPU9250_Buffer[9] = wiringPiI2CReadReg8(MPU9250_fd, 0x46);
		_Tmp_MPU9250_G_Y = (_Tmp_MPU9250_Buffer[8] << 8 | _Tmp_MPU9250_Buffer[9]);
		_uORB_MPU9250_G_Y = (short)_Tmp_MPU9250_G_Y;
		_Tmp_MPU9250_Buffer[10] = wiringPiI2CReadReg8(MPU9250_fd, 0x47);
		_Tmp_MPU9250_Buffer[11] = wiringPiI2CReadReg8(MPU9250_fd, 0x48);
		_Tmp_MPU9250_G_Z = (_Tmp_MPU9250_Buffer[10] << 8 | _Tmp_MPU9250_Buffer[11]);
		_uORB_MPU9250_G_Z = (short)_Tmp_MPU9250_G_Z;
	}
};