#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#ifdef SPI_MPU9250
#include <wiringPiSPI.h>
#endif

#include <nlohmann/json.hpp>

#include <math.h>
#include <thread>
#include <string>
#include <fstream>
#include <unistd.h>
#include <iostream>

#include "_thirdparty/pca9685.h"

class RPiSingelAPM
{
public:
	bool _flag_ForceFailed_Safe;
	bool _flag_Error;

	int MPU9250_fd;
	int PCA9658_fd;
	int RCReader_fd;
	int Status_Code[20];
	const int PWM_Freq = 400;

	const int PCA9685_PinBase = 65;
	const int PCA9685_Address = 0x40;

	int Update_Freqeuncy;
	int Update_Freq_Time;
	int Attitude_loopTime;

	const int _flag_Lazy_Throttle = 2300;
	const int _flag_Lock_Throttle = 2200;

	//========================RC_Controller=================//
	int data[36];
	int _uORB_RC__Safe = 0;
	int _uORB_RC__Roll = 0;
	int _uORB_RC_Pitch = 0;
	int _uORB_RC_Throttle = 0;
	int _uORB_RC__Yall = 0;
	int _uORB_RC_Out__Roll = 0;
	int _uORB_RC_Out_Pitch = 0;
	int _uORB_RC_Out__Yall = 0;
	int _flag_RC_Middle__Roll = 1000;
	int _flag_RC_Middle_Pitch = 1000;
	int _flag_RC_Middle__Yall = 1000;

	int _flag_RC_Max__Roll = 1000;
	int _flag_RC_Max_Pitch = 1000;
	int _flag_RC_Max_Throttle = 1000;
	int _flag_RC_Max__Yall = 1000;

	int _flag_RC_Min__Roll = 1000;
	int _flag_RC_Min_Pitch = 1000;
	int _flag_RC_Min_Throttle = 1000;
	int _flag_RC_Min__Yall = 1000;
	//========================RC_Controller=================//

	//========================ESCController==========//
	int _flag_A1_Pin = 0;
	int _flag_A2_Pin = 1;
	int _flag_B1_Pin = 2;
	int _flag_B2_Pin = 3;
	int _uORB_A1_Speed;
	int _uORB_A2_Speed;
	int _uORB_B1_Speed;
	int _uORB_B2_Speed;
	float _uORB_Leveling__Roll;
	float _uORB_Leveling_Pitch;
	float _uORB_Leveling__Yall;
	//========================ESCController==========//

	//==========================sensors=======//
	int MPU9250_SPI_Channel = 1;
	const int MPU9250_ADDR = 0x68;
	float _flag_MPU9250_LSB = 65.5;
	int MPU9250_SPI_Freq = 1000000;

	int _Tmp_MPU9250_Buffer[14];
	unsigned char _Tmp_MPU9250_SPI_Config[5];
	unsigned char _Tmp_MPU9250_SPI_Buffer[28];

	bool _flag_first_StartUp = true;
	long _Tmp_IMU_Accel_Calibration[20];
	long _Tmp_IMU_Accel_Vector;
	long _uORB_MPU9250_A_X;
	long _uORB_MPU9250_A_Y;
	long _uORB_MPU9250_A_Z;
	unsigned long _Tmp_MPU9250_A_X;
	unsigned long _Tmp_MPU9250_A_Y;
	unsigned long _Tmp_MPU9250_A_Z;
	long _Flag_MPU9250_A_X_Cali;
	long _Flag_MPU9250_A_Y_Cali;

	long _uORB_MPU9250_G_X;
	long _uORB_MPU9250_G_Y;
	long _uORB_MPU9250_G_Z;
	unsigned long _Tmp_MPU9250_G_X;
	unsigned long _Tmp_MPU9250_G_Y;
	unsigned long _Tmp_MPU9250_G_Z;
	long _Flag_MPU9250_G_X_Cali;
	long _Flag_MPU9250_G_Y_Cali;
	long _Flag_MPU9250_G_Z_Cali;

	float _uORB_Accel__Roll;
	float _uORB_Accel_Pitch;
	float _uORB_Gryo__Roll;
	float _uORB_Gryo_Pitch;
	float _uORB_Gryo__Yall;
	float _uORB_Real_Pitch;
	float _uORB_Real__Roll;
	//==========================sensors=======//

	//==========================PID Args==============================================//
	float _flag_PID_P__Roll_Gain;
	float _flag_PID_P_Pitch_Gain;
	float _flag_PID_P__Yall_Gain;

	float _flag_PID_I__Roll_Gain;
	float _flag_PID_I_Pitch_Gain;
	float _flag_PID_I__Yall_Gain;
	float _flag_PID_I__Roll_Max__Value;
	float _flag_PID_I_Pitch_Max__Value;
	float _flag_PID_I__Yall_Max__Value;

	float _flag_PID_D__Roll_Gain;
	float _flag_PID_D_Pitch_Gain;
	float _flag_PID_D__Yall_Gain;

	float _flag_PID_Level_Max;
	//PID Tmp
	float _uORB_PID_D_Last_Value__Roll = 0;
	float _uORB_PID_D_Last_Value_Pitch = 0;
	float _uORB_PID_D_Last_Value__Yall = 0;

	float _uORB_PID_I_Last_Value__Roll = 0;
	float _uORB_PID_I_Last_Value_Pitch = 0;
	float _uORB_PID_I_Last_Value__Yall = 0;

	float _uORB_PID__Roll_Input = 0;
	float _uORB_PID_Pitch_Input = 0;
	//==========================PID Args==============================================//

	RPiSingelAPM()
	{
		_flag_ForceFailed_Safe = true;
		Status_Code[0] = 0;
		//=======SYS_Setup=================//
		if (wiringPiSetupSys() < 0)
			Status_Code[1] = -1;
		else
		{
			Status_Code[1] = 0;
			piHiPri(99);
		}
		//=======MPU9259_Setup============//
#ifdef I2C_MPU9250
		MPU9250_fd = wiringPiI2CSetup(MPU9250_ADDR);
		if (MPU9250_fd < 0)
			Status_Code[2] = -1;
		else
		{
			wiringPiI2CWriteReg8(MPU9250_fd, 107, 0x00); //reset
			wiringPiI2CWriteReg8(MPU9250_fd, 28, 0x08);  //Accel
			wiringPiI2CWriteReg8(MPU9250_fd, 27, 0x08);  //Gryo
			wiringPiI2CWriteReg8(MPU9250_fd, 26, 0x03);  //config
			Status_Code[2] = 0;
		}
#endif
#ifdef SPI_MPU9250
		MPU9250_fd = wiringPiSPISetup(MPU9250_SPI_Channel, MPU9250_SPI_Freq);
		if (MPU9250_fd < 0)
			Status_Code[2] = -1;
		else
		{
			_Tmp_MPU9250_SPI_Config[0] = 0x6b;
			_Tmp_MPU9250_SPI_Config[1] = 0x00;
			wiringPiSPIDataRW(1, _Tmp_MPU9250_SPI_Config, 2); //reset
			sleep(1);
			_Tmp_MPU9250_SPI_Config[0] = 0x1c;
			_Tmp_MPU9250_SPI_Config[1] = 0x08;
			wiringPiSPIDataRW(1, _Tmp_MPU9250_SPI_Config, 2); // Accel
			sleep(1);
			_Tmp_MPU9250_SPI_Config[0] = 0x1b;
			_Tmp_MPU9250_SPI_Config[1] = 0x08;
			wiringPiSPIDataRW(1, _Tmp_MPU9250_SPI_Config, 2); // Gryo
			sleep(1);
			_Tmp_MPU9250_SPI_Config[0] = 0x1a;
			_Tmp_MPU9250_SPI_Config[1] = 0x03;
			wiringPiSPIDataRW(1, _Tmp_MPU9250_SPI_Config, 2);//config
			sleep(1);
			Status_Code[2] = 0;
		}
#endif
		//=======RC__Setup=================//
		RCReader_fd = serialOpen("/dev/ttyS0", 115200);
		if (RCReader_fd < 0)
			Status_Code[3] = -1;
		else
			Status_Code[3] = 0;
		//=======PWM_Setup=================//
		PCA9658_fd = pca9685Setup(PCA9685_PinBase, PCA9685_Address, PWM_Freq);
		if (PCA9658_fd < 0)
			Status_Code[4] = -1;
		else
			Status_Code[4] = 0;
		//=======Config_Parse================//
		ConfigReader();
		Update_Freq_Time = (float)1 / Update_Freqeuncy * 1000000;
		Status_Code[5] = Update_Freq_Time;
		//=======run Gryo calibration========//
		_Flag_MPU9250_G_X_Cali = 0;
		_Flag_MPU9250_G_Y_Cali = 0;
		_Flag_MPU9250_G_Z_Cali = 0;
		for (int cali_count = 0; cali_count < 2000; cali_count++)
		{
			SensorsDataRead();
			_Flag_MPU9250_G_X_Cali += _uORB_MPU9250_G_X;
			_Flag_MPU9250_G_Y_Cali += _uORB_MPU9250_G_Y;
			_Flag_MPU9250_G_Z_Cali += _uORB_MPU9250_G_Z;
			usleep(3);
		}
		_Flag_MPU9250_G_X_Cali = _Flag_MPU9250_G_X_Cali / 2000;
		_Flag_MPU9250_G_Y_Cali = _Flag_MPU9250_G_Y_Cali / 2000;
		_Flag_MPU9250_G_Z_Cali = _Flag_MPU9250_G_Z_Cali / 2000;
		Status_Code[6] = 0;
	}

	inline void SensorsParse()
	{
		SensorsDataRead();
		//Cail----------------------------------------------------------------------//
		//Acel_cali
		_uORB_MPU9250_A_X -= _Flag_MPU9250_A_X_Cali;
		_uORB_MPU9250_A_Y -= _Flag_MPU9250_A_Y_Cali;
		//Gryo_cail
		_uORB_MPU9250_G_X -= _Flag_MPU9250_G_X_Cali;
		_uORB_MPU9250_G_Y -= _Flag_MPU9250_G_Y_Cali;
		_uORB_MPU9250_G_Z -= _Flag_MPU9250_G_Z_Cali;
		//Gryo----------------------------------------------------------------------//
		_uORB_Gryo__Roll = (_uORB_Gryo__Roll * 0.7) + ((_uORB_MPU9250_G_Y / _flag_MPU9250_LSB) * 0.3);
		_uORB_Gryo_Pitch = (_uORB_Gryo_Pitch * 0.7) + ((_uORB_MPU9250_G_X / _flag_MPU9250_LSB) * 0.3);
		_uORB_Gryo__Yall = (_uORB_Gryo__Yall * 0.7) + ((_uORB_MPU9250_G_Z / _flag_MPU9250_LSB) * 0.3);
		//ACCEL---------------------------------------------------------------------//
		_Tmp_IMU_Accel_Vector = sqrt((_uORB_MPU9250_A_X * _uORB_MPU9250_A_X) + (_uORB_MPU9250_A_Y * _uORB_MPU9250_A_Y) + (_uORB_MPU9250_A_Z * _uORB_MPU9250_A_Z));
		if (abs(_uORB_MPU9250_A_X) < _Tmp_IMU_Accel_Vector)
			_uORB_Accel__Roll = asin((float)_uORB_MPU9250_A_X / _Tmp_IMU_Accel_Vector) * -57.296;
		if (abs(_uORB_MPU9250_A_Y) < _Tmp_IMU_Accel_Vector)
			_uORB_Accel_Pitch = asin((float)_uORB_MPU9250_A_Y / _Tmp_IMU_Accel_Vector) * 57.296;
		//Gryo_MIX_ACCEL------------------------------------------------------------//

		_uORB_Real_Pitch += _uORB_MPU9250_G_X / Update_Freqeuncy / _flag_MPU9250_LSB;
		_uORB_Real__Roll += _uORB_MPU9250_G_Y / Update_Freqeuncy / _flag_MPU9250_LSB;
		_uORB_Real_Pitch -= _uORB_Real__Roll * sin((_uORB_MPU9250_G_Z / Update_Freqeuncy / _flag_MPU9250_LSB) * (3.14 / 180));
		_uORB_Real__Roll += _uORB_Real_Pitch * sin((_uORB_MPU9250_G_Z / Update_Freqeuncy / _flag_MPU9250_LSB) * (3.14 / 180));
		if (!_flag_first_StartUp)
		{
			_uORB_Real_Pitch = _uORB_Real_Pitch * 0.9995 + _uORB_Accel_Pitch * 0.0005;
			_uORB_Real__Roll = _uORB_Real__Roll * 0.9995 + _uORB_Accel__Roll * 0.0005;
		}
		else
		{
			_uORB_Real_Pitch = _uORB_Accel_Pitch;
			_uORB_Real__Roll = _uORB_Accel__Roll;
			_flag_first_StartUp = false;
		}
	}

	inline void ControlParse()
	{
		ControlRead();
		if (_uORB_RC__Roll < _flag_RC_Middle__Roll + 10 && _uORB_RC__Roll > _flag_RC_Middle__Roll - 10)
			_uORB_RC_Out__Roll = 0;
		else
			_uORB_RC_Out__Roll = (_uORB_RC__Roll - _flag_RC_Middle__Roll) / 4;

		if (_uORB_RC_Pitch < _flag_RC_Middle_Pitch + 10 && _uORB_RC_Pitch > _flag_RC_Middle_Pitch - 10)
			_uORB_RC_Out_Pitch = 0;
		else
			_uORB_RC_Out_Pitch = (_uORB_RC_Pitch - _flag_RC_Middle_Pitch) / 4;

		if (_uORB_RC__Yall < _flag_RC_Middle__Yall + 10 && _uORB_RC__Yall > _flag_RC_Middle__Yall - 10)
			_uORB_RC_Out__Yall = 0;
		else
			_uORB_RC_Out__Yall = (_uORB_RC__Yall - _flag_RC_Middle__Yall) / 4;
	}

	inline void AttitudeUpdate()
	{
		//Roll PID Mix
		_uORB_PID__Roll_Input = _uORB_Gryo__Roll + _uORB_Real__Roll * 10 - _uORB_RC_Out__Roll;
		PID_Caculate(_uORB_PID__Roll_Input, _uORB_Leveling__Roll,
			_uORB_PID_I_Last_Value__Roll, _uORB_PID_D_Last_Value__Roll,
			_flag_PID_P__Roll_Gain, _flag_PID_I__Roll_Gain, _flag_PID_D__Roll_Gain, _flag_PID_I__Roll_Max__Value);
		if (_uORB_Leveling__Roll > _flag_PID_Level_Max)
			_uORB_Leveling__Roll = _flag_PID_Level_Max;
		if (_uORB_Leveling__Roll < _flag_PID_Level_Max * -1)
			_uORB_Leveling__Roll = _flag_PID_Level_Max * -1;

		//Pitch PID Mix
		_uORB_PID_Pitch_Input = _uORB_Gryo_Pitch + _uORB_Real_Pitch * 10 - _uORB_RC_Out_Pitch;
		PID_Caculate(_uORB_PID_Pitch_Input, _uORB_Leveling_Pitch,
			_uORB_PID_I_Last_Value_Pitch, _uORB_PID_D_Last_Value_Pitch,
			_flag_PID_P_Pitch_Gain, _flag_PID_I_Pitch_Gain, _flag_PID_D_Pitch_Gain, _flag_PID_I_Pitch_Max__Value);
		if (_uORB_Leveling_Pitch > _flag_PID_Level_Max)
			_uORB_Leveling_Pitch = _flag_PID_Level_Max;
		if (_uORB_Leveling_Pitch < _flag_PID_Level_Max * -1)
			_uORB_Leveling_Pitch = _flag_PID_Level_Max * -1;

		//Yall PID Mix
		PID_Caculate(_uORB_Gryo__Yall - _uORB_RC_Out__Yall, _uORB_Leveling__Yall,
			_uORB_PID_I_Last_Value__Yall, _uORB_PID_D_Last_Value__Yall,
			_flag_PID_P__Yall_Gain, _flag_PID_I__Yall_Gain, _flag_PID_D__Yall_Gain, _flag_PID_I__Yall_Max__Value);
		if (_uORB_Leveling__Yall > _flag_PID_Level_Max)
			_uORB_Leveling__Yall = _flag_PID_Level_Max;
		if (_uORB_Leveling__Yall < _flag_PID_Level_Max * -1)
			_uORB_Leveling__Yall = _flag_PID_Level_Max * -1;

		_uORB_B1_Speed = _uORB_RC_Throttle - _uORB_Leveling__Roll + _uORB_Leveling_Pitch + _uORB_Leveling__Yall;
		_uORB_A1_Speed = _uORB_RC_Throttle - _uORB_Leveling__Roll - _uORB_Leveling_Pitch - _uORB_Leveling__Yall;
		_uORB_A2_Speed = _uORB_RC_Throttle + _uORB_Leveling__Roll - _uORB_Leveling_Pitch + _uORB_Leveling__Yall;
		_uORB_B2_Speed = _uORB_RC_Throttle + _uORB_Leveling__Roll + _uORB_Leveling_Pitch - _uORB_Leveling__Yall;
	}

	inline void SaftyChecking()
	{
		//=====================AttitudeUpdate_Time_checkout=========================//
		if (Attitude_loopTime > Update_Freq_Time)
		{
			_flag_Error = true; 
			Status_Code[10] = -1;
		}
		//=====================RC_input_checkout====================================//
		if (!(_uORB_RC__Roll < _flag_RC_Max__Roll + 20 && _uORB_RC__Roll > _flag_RC_Min__Roll - 20))
		{
			_flag_Error = true; 
			Status_Code[11] = -1;
		}
		else
		{
			Status_Code[11] = 0;
		}
			
		if (!(_uORB_RC_Pitch < _flag_RC_Max_Pitch + 20 && _uORB_RC_Pitch > _flag_RC_Min_Pitch - 20))
		{
			_flag_Error = true; 
			Status_Code[12] = -1;
		}
		else
		{
			Status_Code[12] = 0;
		}
		if (!(_uORB_RC_Throttle < _flag_RC_Max_Throttle + 20 && _uORB_RC_Throttle > _flag_RC_Min_Throttle - 20))
		{
			_flag_Error = true; 
			Status_Code[13] = -1;
		}
		else
		{
			Status_Code[13] = 0;
		}
		if (!(_uORB_RC__Yall < _flag_RC_Max__Yall + 20 && _uORB_RC__Yall > _flag_RC_Min__Yall - 20))
		{
			_flag_Error = true; 
			Status_Code[14] = -1;
		}
		else
		{
			Status_Code[14] = 0;
		}
			
		//=====================Attitude_checkout====================================//
		if (_uORB_Real_Pitch > 70.0 || _uORB_Real_Pitch < -70.0)
		{
			_flag_Error = true; 
			Status_Code[15] = -1;
		}
		else 
		{
			Status_Code[15] = 0;
		}
		if (_uORB_Real__Roll > 70.0 || _uORB_Real__Roll < -70.0)
		{
			_flag_Error = true; 
			Status_Code[16] = -1;
		}
		else
		{
			Status_Code[16] = 0;
		}
			
		if (_uORB_Accel_Pitch > 90.0 || _uORB_Accel_Pitch < -90.0)
		{
			_flag_Error = true; 
			Status_Code[17] = -1;
		}
		else
		{
			Status_Code[17] = 0;
		}
		if (_uORB_Accel__Roll > 90.0 || _uORB_Accel__Roll < -90.0)
		{
			_flag_Error = true; 
			Status_Code[18] = -1;
		}
		else
		{
			Status_Code[18] = 0;
		}
		//=====================Status_checkout======================================//
		if (_flag_ForceFailed_Safe == true)
		{
			Status_Code[0] = 0;
		}
		else if(_flag_Error == true)
		{
			for (int i = 0; i < 20; i++)
			{
				std::cout << Status_Code[i] << " ";
			}
			std::cout << "\n";
		}
	}

	inline void ESCUpdate()
	{
		_uORB_A1_Speed = (700 * (((float)_uORB_A1_Speed - (float)300) / (float)1400)) + 2280;
		_uORB_A2_Speed = (700 * (((float)_uORB_A2_Speed - (float)300) / (float)1400)) + 2280;
		_uORB_B1_Speed = (700 * (((float)_uORB_B1_Speed - (float)300) / (float)1400)) + 2280;
		_uORB_B2_Speed = (700 * (((float)_uORB_B2_Speed - (float)300) / (float)1400)) + 2280;

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

	//-------------------------------------------------------------------//

	inline void Debug()
	{
		std::cout << _uORB_Real_Pitch << " ";
		std::cout << _uORB_Real__Roll << " ";
		std::cout << _uORB_Gryo_Pitch << " ";
		std::cout << _uORB_Gryo__Roll << " ";
		std::cout << _uORB_Gryo__Yall << " \r";
	}

	inline void SensorsCalibration()
	{
		int CalibrationComfirm;
		std::cout << "[Sensors] Calibration will start , input 1 to start , input -1 to skip" << "\n";
		std::cin >> CalibrationComfirm;
		if (CalibrationComfirm == -1)
		{
			return;
		}
		std::cout << "[Sensors] Accel Calibration ......" << "\n";
		for (int cali_count = 0; cali_count < 200; cali_count++)
		{
			SensorsParse();
			_Tmp_IMU_Accel_Calibration[0] += _uORB_MPU9250_A_X;
			_Tmp_IMU_Accel_Calibration[1] += _uORB_MPU9250_A_Y;
			_Tmp_IMU_Accel_Calibration[2] += _uORB_MPU9250_A_Z;
		}
		_Tmp_IMU_Accel_Calibration[0] /= 200;
		_Tmp_IMU_Accel_Calibration[1] /= 200;
		_Tmp_IMU_Accel_Calibration[2] /= 200;

		for (int cali_count = 0; cali_count < 200; cali_count++)
		{
			SensorsParse();
			_Tmp_IMU_Accel_Calibration[3] += _uORB_MPU9250_A_X;
			_Tmp_IMU_Accel_Calibration[4] += _uORB_MPU9250_A_Y;
			_Tmp_IMU_Accel_Calibration[5] += _uORB_MPU9250_A_Z;
		}
		_Tmp_IMU_Accel_Calibration[3] /= 200;
		_Tmp_IMU_Accel_Calibration[4] /= 200;
		_Tmp_IMU_Accel_Calibration[5] /= 200;		

		for (int cali_count = 0; cali_count < 200; cali_count++)
		{
			SensorsParse();
			_Tmp_IMU_Accel_Calibration[6] += _uORB_MPU9250_A_X;
			_Tmp_IMU_Accel_Calibration[7] += _uORB_MPU9250_A_Y;
			_Tmp_IMU_Accel_Calibration[8] += _uORB_MPU9250_A_Z;
		}
		_Tmp_IMU_Accel_Calibration[6] /= 200;
		_Tmp_IMU_Accel_Calibration[7] /= 200;
		_Tmp_IMU_Accel_Calibration[8] /= 200;		

		for (int cali_count = 0; cali_count < 200; cali_count++)
		{
			SensorsParse();
			_Tmp_IMU_Accel_Calibration[9] += _uORB_MPU9250_A_X;
			_Tmp_IMU_Accel_Calibration[10] += _uORB_MPU9250_A_Y;
			_Tmp_IMU_Accel_Calibration[11] += _uORB_MPU9250_A_Z;
		}
		_Tmp_IMU_Accel_Calibration[9] /= 200;
		_Tmp_IMU_Accel_Calibration[10] /= 200;
		_Tmp_IMU_Accel_Calibration[11] /= 200;		

		for (int cali_count = 0; cali_count < 200; cali_count++)
		{
			SensorsParse();
			_Tmp_IMU_Accel_Calibration[12] += _uORB_MPU9250_A_X;
			_Tmp_IMU_Accel_Calibration[13] += _uORB_MPU9250_A_Y;
			_Tmp_IMU_Accel_Calibration[14] += _uORB_MPU9250_A_Z;
		}
		_Tmp_IMU_Accel_Calibration[12] /= 200;
		_Tmp_IMU_Accel_Calibration[13] /= 200;
		_Tmp_IMU_Accel_Calibration[14] /= 200;
		std::cout << "[Sensors] Accel Calibration finsh , input -1 to retry , input 1 to write to configJSON , 0 to skip" << "\n";
		std::cin >> CalibrationComfirm;
		if (CalibrationComfirm == -1)
		{
			SensorsCalibration();
		}
		else if (CalibrationComfirm == 1)
		{
			std::ifstream config("./APMconfig.json");
			std::string content((std::istreambuf_iterator<char>(config)),
				(std::istreambuf_iterator<char>()));
			nlohmann::json Configdata = nlohmann::json::parse(content);

			Configdata["_Flag_MPU9250_A_X_Cali"] = _Flag_MPU9250_A_X_Cali;
			Configdata["_Flag_MPU9250_A_Y_Cali"] = _Flag_MPU9250_A_Y_Cali;

			std::ofstream configIN;
			configIN.open("./APMconfig.json");
			configIN.clear();
			configIN << Configdata.dump(4).c_str();
			configIN.close();

			std::cout << "[Sensors] Config write success\n";
		}
	}

	inline void ESCCalibration()
	{
		int CalibrationComfirm;
		std::cout << "[ESCStatus] ESC calibration start........." << " \n";
		std::cout << "[ESCStatus] ESC calibration start,connect ESC to power and input 1 , or input -1 to skip calibration" << " \n";
		std::cin >> CalibrationComfirm;
		if (CalibrationComfirm == -1)
		{
			std::cout << "[ESCStatus] Exiting ESC calibration ........." << " \n";
			return;
		}
		pca9685PWMWrite(PCA9658_fd, _flag_A1_Pin, 0, 3000);
		pca9685PWMWrite(PCA9658_fd, _flag_A2_Pin, 0, 3000);
		pca9685PWMWrite(PCA9658_fd, _flag_B1_Pin, 0, 3000);
		pca9685PWMWrite(PCA9658_fd, _flag_B2_Pin, 0, 3000);
		std::cout << "[ESCStatus] ESC calibration will finsh , input 1 to stop " << " \n";
		std::cin >> CalibrationComfirm;
		std::cout << "[ESCStatus] ESC calibration Finsh....." << " \n";
		pca9685PWMWrite(PCA9658_fd, _flag_A1_Pin, 0, 2200);
		pca9685PWMWrite(PCA9658_fd, _flag_A2_Pin, 0, 2200);
		pca9685PWMWrite(PCA9658_fd, _flag_B1_Pin, 0, 2200);
		pca9685PWMWrite(PCA9658_fd, _flag_B2_Pin, 0, 2200);
		sleep(3);
		std::cout << "[ESCStatus] ESC calibration over" << " \n";
	}

	inline void ControlCalibration()
	{
		int CalibrationComfirm;
		std::cout << "[Controller] ControlCalibraion start , input 1 to start , input -1 to skip \n";
		std::cin >> CalibrationComfirm;
		if (CalibrationComfirm == -1)
		{
			return;
		}
		sleep(2);
		std::cout << "[Controller] ControlCalibraion start ...... \n";
		for (int cali_count = 0; cali_count < 2000; cali_count++)
		{
			ControlRead();
			std::cout << "Roll:" << _uORB_RC__Roll << " ";
			std::cout << "Pitch:" << _uORB_RC_Pitch << " ";
			std::cout << "Throttle:" << _uORB_RC_Throttle << " ";
			std::cout << "Yall:" << _uORB_RC__Yall << " \r";

			if (_uORB_RC__Roll > _flag_RC_Max__Roll&& _uORB_RC__Roll != 0)
				_flag_RC_Max__Roll = _uORB_RC__Roll;
			if (_uORB_RC_Pitch > _flag_RC_Max_Pitch&& _uORB_RC_Pitch != 0)
				_flag_RC_Max_Pitch = _uORB_RC_Pitch;
			if (_uORB_RC_Throttle > _flag_RC_Max_Throttle&& _uORB_RC_Throttle != 0)
				_flag_RC_Max_Throttle = _uORB_RC_Throttle;
			if (_uORB_RC__Yall > _flag_RC_Max__Yall&& _uORB_RC__Yall != 0)
				_flag_RC_Max__Yall = _uORB_RC__Yall;

			if (_uORB_RC__Roll < _flag_RC_Min__Roll && _uORB_RC__Roll != 0)
				_flag_RC_Min__Roll = _uORB_RC__Roll;
			if (_uORB_RC_Pitch < _flag_RC_Min_Pitch && _uORB_RC_Pitch != 0)
				_flag_RC_Min_Pitch = _uORB_RC_Pitch;
			if (_uORB_RC_Throttle < _flag_RC_Min_Throttle && _uORB_RC_Throttle != 0)
				_flag_RC_Min_Throttle = _uORB_RC_Throttle;
			if (_uORB_RC__Yall < _flag_RC_Min__Yall && _uORB_RC__Yall != 0)
				_flag_RC_Min__Yall = _uORB_RC__Yall;
			usleep(2000);
		}
		std::cout << "\n[Controller] Calibration will finshed"
			<< " Please Check the tick middle and throttle down and pass enter" << "\n";
		std::cin >> CalibrationComfirm;
		for (int cali_count = 0; cali_count < 1000; cali_count++)
		{
			ControlRead();
			_flag_RC_Middle_Pitch = _uORB_RC_Pitch;
			_flag_RC_Middle__Roll = _uORB_RC__Roll;
			_flag_RC_Middle__Yall = _uORB_RC__Yall;
		}
		std::cout << "[Controler] Controller calitbration comfirm:\n"
			<< "Max__Roll    = " << _flag_RC_Max__Roll << "\n"
			<< "Max_Pitch    = " << _flag_RC_Max_Pitch << "\n"
			<< "Max_Throttle = " << _flag_RC_Max_Throttle << "\n"
			<< "Max__Yall    = " << _flag_RC_Max__Yall << "\n\n"

			<< "Middle__Roll = " << _flag_RC_Middle__Roll << "\n"
			<< "Middle_Pitch = " << _flag_RC_Middle_Pitch << "\n"
			<< "Middle__Yall = " << _flag_RC_Middle__Yall << "\n\n"

			<< "Min__Roll    = " << _flag_RC_Min__Roll << "\n"
			<< "Min_Pitch    = " << _flag_RC_Min_Pitch << "\n"
			<< "Min_Throttle = " << _flag_RC_Min_Throttle << "\n"
			<< "Min__Yall    = " << _flag_RC_Min__Yall << "\n";
		std::cout << "<-----------Controller_calibration_over---------------->\n";
		std::cout << "[Controller] Calibration finshed ,if you want to retry input -1 , to write to configJSON input 1 , input 0 to skip" << "\n";
		std::cin >> CalibrationComfirm;
		if (CalibrationComfirm == -1)
		{
			ControlCalibration();
		}
		else if (CalibrationComfirm == 1)
		{
			std::ifstream config("./APMconfig.json");
			std::string content((std::istreambuf_iterator<char>(config)),
				(std::istreambuf_iterator<char>()));
			nlohmann::json Configdata = nlohmann::json::parse(content);

			Configdata["_flag_RC_Max__Roll"] = _flag_RC_Max__Roll;
			Configdata["_flag_RC_Max_Pitch"] = _flag_RC_Max_Pitch;
			Configdata["_flag_RC_Max_Throttle"] = _flag_RC_Max_Throttle;
			Configdata["_flag_RC_Max__Yall"] = _flag_RC_Max__Yall;

			Configdata["_flag_RC_Middle__Roll"] = _flag_RC_Middle__Roll;
			Configdata["_flag_RC_Middle_Pitch"] = _flag_RC_Middle_Pitch;
			Configdata["_flag_RC_Middle__Yall"] = _flag_RC_Middle__Yall;

			Configdata["_flag_RC_Min__Roll"] = _flag_RC_Min__Roll;
			Configdata["_flag_RC_Min_Pitch"] = _flag_RC_Min_Pitch;
			Configdata["_flag_RC_Min_Throttle"] = _flag_RC_Min_Throttle;
			Configdata["_flag_RC_Min__Yall"] = _flag_RC_Min__Yall;

			std::ofstream configIN;
			configIN.open("./APMconfig.json");
			configIN.clear();
			configIN << Configdata.dump(4).c_str();
			configIN.close();

			std::cout << "[Controller] Config write success\n";
		}
	}

	//-------------------------------------------------------------------//

private:
	inline void PID_Caculate(float inputData, float& outputData,
		float& last_I_Data, float& last_D_Data,
		float P_Gain, float I_Gain, float D_Gain, float I_Max)
	{
		//P caculate
		outputData = P_Gain * inputData;
		//D caculate
		outputData += D_Gain * (inputData - last_D_Data);
		last_D_Data = inputData;
		//I caculate
		last_I_Data += inputData * I_Gain;
		if (last_I_Data > I_Max)
			last_I_Data = I_Max;
		if (last_I_Data < I_Max * -1)
			last_I_Data = I_Max * -1;
		//P_I_D Mix OUTPUT
		outputData += last_I_Data;
	}

	inline void ConfigReader()
	{
		std::cout << "[ConfigRead]starting to check out config file ....\n";
		std::ifstream config("./APMconfig.json");
		std::string content((std::istreambuf_iterator<char>(config)),
			(std::istreambuf_iterator<char>()));
		nlohmann::json Configdata = nlohmann::json::parse(content);
		//==========================================================Controller cofig==/
		_flag_RC_Max__Roll = Configdata["_flag_RC_Max__Roll"].get<int>();
		_flag_RC_Max_Pitch = Configdata["_flag_RC_Max_Pitch"].get<int>();
		_flag_RC_Max_Throttle = Configdata["_flag_RC_Max_Throttle"].get<int>();
		_flag_RC_Max__Yall = Configdata["_flag_RC_Max__Yall"].get<int>();

		_flag_RC_Middle__Roll = Configdata["_flag_RC_Middle__Roll"].get<int>();
		_flag_RC_Middle_Pitch = Configdata["_flag_RC_Middle_Pitch"].get<int>();
		_flag_RC_Middle__Yall = Configdata["_flag_RC_Middle__Yall"].get<int>();

		_flag_RC_Min__Roll = Configdata["_flag_RC_Min__Roll"].get<int>();
		_flag_RC_Min_Pitch = Configdata["_flag_RC_Min_Pitch"].get<int>();
		_flag_RC_Min_Throttle = Configdata["_flag_RC_Min_Throttle"].get<int>();
		_flag_RC_Min__Yall = Configdata["_flag_RC_Min__Yall"].get<int>();

		_flag_A1_Pin = Configdata["_flag_A1_Pin"].get<int>();
		_flag_A2_Pin = Configdata["_flag_A2_Pin"].get<int>();
		_flag_B1_Pin = Configdata["_flag_B1_Pin"].get<int>();
		_flag_B2_Pin = Configdata["_flag_B2_Pin"].get<int>();
		//==================================================================PID cofig==/
		_flag_PID_P__Roll_Gain = Configdata["_flag_PID_P__Roll_Gain"].get<float>();
		_flag_PID_P_Pitch_Gain = Configdata["_flag_PID_P_Pitch_Gain"].get<float>();
		_flag_PID_P__Yall_Gain = Configdata["_flag_PID_P__Yall_Gain"].get<float>();

		_flag_PID_I__Roll_Gain = Configdata["_flag_PID_I__Roll_Gain"].get<float>();
		_flag_PID_I_Pitch_Gain = Configdata["_flag_PID_I_Pitch_Gain"].get<float>();
		_flag_PID_I__Yall_Gain = Configdata["_flag_PID_I__Yall_Gain"].get<float>();
		_flag_PID_I__Roll_Max__Value = Configdata["_flag_PID_I__Roll_Max__Value"].get<float>();
		_flag_PID_I_Pitch_Max__Value = Configdata["_flag_PID_I_Pitch_Max__Value"].get<float>();
		_flag_PID_I__Yall_Max__Value = Configdata["_flag_PID_I__Yall_Max__Value"].get<float>();

		_flag_PID_D__Roll_Gain = Configdata["_flag_PID_D__Roll_Gain"].get<float>();
		_flag_PID_D_Pitch_Gain = Configdata["_flag_PID_D_Pitch_Gain"].get<float>();
		_flag_PID_D__Yall_Gain = Configdata["_flag_PID_D__Yall_Gain"].get<float>();

		_flag_PID_Level_Max = Configdata["_flag_PID_Level_Max"].get<float>();
		//==============================================================Sensors cofig==/
		_Flag_MPU9250_A_X_Cali = Configdata["_Flag_MPU9250_A_X_Cali"].get<int>();
		_Flag_MPU9250_A_Y_Cali = Configdata["_Flag_MPU9250_A_Y_Cali"].get<int>();
		//===============================================================Update cofig==/
		Update_Freqeuncy = Configdata["Update_Freqeucy"].get<int>();
		std::cout << "[ConfigRead]Config Set Success!\n";
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
		_uORB_RC__Safe = data[9] * 255 + data[10];

		if (_uORB_RC_Throttle < _flag_RC_Min_Throttle + 20 && 1400 < _uORB_RC__Safe)
		{
			if (_flag_Error == false)
			{
				_flag_ForceFailed_Safe = false;
				Status_Code[0] = 1;
			}
			else
			{
				_flag_ForceFailed_Safe = true;
			}
		}
		else if (_uORB_RC__Safe < 1400)
		{
			_flag_ForceFailed_Safe = true;
			_flag_Error = false;
			Status_Code[0] = 0;
		}
		else if (_uORB_RC_Throttle < _flag_RC_Max_Throttle + 20)
		{
			if (_flag_Error == true)
			{
				_flag_ForceFailed_Safe = true;
			}
		}

		if (_flag_ForceFailed_Safe == true)
		{
			_uORB_PID_D_Last_Value__Roll = 0;
			_uORB_PID_D_Last_Value_Pitch = 0;
			_uORB_PID_D_Last_Value__Yall = 0;
			_uORB_PID_I_Last_Value__Roll = 0;
			_uORB_PID_I_Last_Value_Pitch = 0;
			_uORB_PID_I_Last_Value__Yall = 0;
		}
	}

	inline void SensorsDataRead()
	{
#ifdef I2C_MPU9250
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
#endif
#ifdef SPI_MPU9250
		_Tmp_MPU9250_SPI_Buffer[0] = 0xBB;
		wiringPiSPIDataRW(1, _Tmp_MPU9250_SPI_Buffer, 20);
		_Tmp_MPU9250_A_X = ((int)_Tmp_MPU9250_SPI_Buffer[1] << 8 | (int)_Tmp_MPU9250_SPI_Buffer[2]);
		_uORB_MPU9250_A_X = (short)_Tmp_MPU9250_A_X;
		_Tmp_MPU9250_A_Y = ((int)_Tmp_MPU9250_SPI_Buffer[3] << 8 | (int)_Tmp_MPU9250_SPI_Buffer[4]);
		_uORB_MPU9250_A_Y = (short)_Tmp_MPU9250_A_Y;
		_Tmp_MPU9250_A_Z = ((int)_Tmp_MPU9250_SPI_Buffer[5] << 8 | (int)_Tmp_MPU9250_SPI_Buffer[6]);
		_uORB_MPU9250_A_Z = (short)_Tmp_MPU9250_A_Z;

		_Tmp_MPU9250_G_X = ((int)_Tmp_MPU9250_SPI_Buffer[9] << 8 | (int)_Tmp_MPU9250_SPI_Buffer[10]);
		_uORB_MPU9250_G_X = (short)_Tmp_MPU9250_G_X;
		_Tmp_MPU9250_G_Y = ((int)_Tmp_MPU9250_SPI_Buffer[11] << 8 | (int)_Tmp_MPU9250_SPI_Buffer[12]);
		_uORB_MPU9250_G_Y = (short)_Tmp_MPU9250_G_Y;
		_Tmp_MPU9250_G_Z = ((int)_Tmp_MPU9250_SPI_Buffer[13] << 8 | (int)_Tmp_MPU9250_SPI_Buffer[14]);
		_uORB_MPU9250_G_Z = (short)_Tmp_MPU9250_G_Z;
#endif
	}
};