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

struct APMSafeStatus
{
	bool ForceFailedSafe;
	bool SafyError;
	bool Is_SyncTimeOut;
	bool Is_RCDisconnect;
	bool Is_AngelOutLimit;
};

class RPiSingelAPM
{
public:
	int Update_Freq_Time;
	long int UpdateTimer_Start;
	long int UpdateTimer_End;
	int Attitude_loopTime;

	RPiSingelAPM()
	{
		Clocking = 0;
		Lose_Clocking = 0;
		_flag_first_StartUp = true;
		_flag_ForceFailed_Safe = true;
		//=======SYS_Setup=================//
		if (wiringPiSetupSys() < 0)
		{

		}
		else
		{
			piHiPri(99);
		}
		//=======MPU9259_Setup============//
#ifdef I2C_MPU9250
		DF.MPU9250_fd = wiringPiI2CSetup(DF.MPU9250_ADDR);
		if (DF.MPU9250_fd < 0)
		{

		}
		else
		{
			wiringPiI2CWriteReg8(DF.MPU9250_fd, 107, 0x00); //reset
			wiringPiI2CWriteReg8(DF.MPU9250_fd, 28, 0x08);  //Accel
			wiringPiI2CWriteReg8(DF.MPU9250_fd, 27, 0x08);  //Gryo
			wiringPiI2CWriteReg8(DF.MPU9250_fd, 26, 0x03);  //config
		}
#endif
#ifdef SPI_MPU9250
		DF.MPU9250_fd = wiringPiSPISetup(DF.MPU9250_SPI_Channel, DF.MPU9250_SPI_Freq);
		if (DF.MPU9250_fd < 0)
		{

		}
		else
		{
			SF._Tmp_MPU9250_SPI_Config[0] = 0x6b;
			SF._Tmp_MPU9250_SPI_Config[1] = 0x00;
			wiringPiSPIDataRW(1, SF._Tmp_MPU9250_SPI_Config, 2); //reset
			SF._Tmp_MPU9250_SPI_Config[0] = 0x1c;
			SF._Tmp_MPU9250_SPI_Config[1] = 0x08;
			wiringPiSPIDataRW(1, SF._Tmp_MPU9250_SPI_Config, 2); // Accel
			SF._Tmp_MPU9250_SPI_Config[0] = 0x1b;
			SF._Tmp_MPU9250_SPI_Config[1] = 0x08;
			wiringPiSPIDataRW(1, SF._Tmp_MPU9250_SPI_Config, 2); // Gryo
			SF._Tmp_MPU9250_SPI_Config[0] = 0x1a;
			SF._Tmp_MPU9250_SPI_Config[1] = 0x03;
			wiringPiSPIDataRW(1, SF._Tmp_MPU9250_SPI_Config, 2); //config
		}
#endif

		//=======RC__Setup=================//
		DF.RCReader_fd = serialOpen("/dev/ttyS0", 115200);
		if (DF.RCReader_fd < 0)
		{

		}
		else
		{

		}
		//=======PWM_Setup=================//
		DF.PCA9658_fd = pca9685Setup(DF.PCA9685_PinBase, DF.PCA9685_Address, DF.PWM_Freq);
		if (DF.PCA9658_fd < 0)
		{

		}
		else
		{

		}
		//=======Config_Parse================//
		ConfigReader();
		Update_Freq_Time = (float)1 / Update_Freqeuncy * 1000000;
		//=======run Gryo calibration========//
		SF._Flag_MPU9250_G_X_Cali = 0;
		SF._Flag_MPU9250_G_Y_Cali = 0;
		SF._Flag_MPU9250_G_Z_Cali = 0;
		for (int cali_count = 0; cali_count < 2000; cali_count++)
		{
			SensorsDataRead();
			SF._Flag_MPU9250_G_X_Cali += SF._uORB_MPU9250_G_X;
			SF._Flag_MPU9250_G_Y_Cali += SF._uORB_MPU9250_G_Y;
			SF._Flag_MPU9250_G_Z_Cali += SF._uORB_MPU9250_G_Z;
			usleep(3);
		}
		SF._Flag_MPU9250_G_X_Cali = SF._Flag_MPU9250_G_X_Cali / 2000;
		SF._Flag_MPU9250_G_Y_Cali = SF._Flag_MPU9250_G_Y_Cali / 2000;
		SF._Flag_MPU9250_G_Z_Cali = SF._Flag_MPU9250_G_Z_Cali / 2000;
	}

	inline void SensorsParse()
	{
		SensorsDataRead();
		//Gryo----------------------------------------------------------------------//
		SF._uORB_MPU9250_G_X -= SF._Flag_MPU9250_G_X_Cali;
		SF._uORB_MPU9250_G_Y -= SF._Flag_MPU9250_G_Y_Cali;
		SF._uORB_MPU9250_G_Z -= SF._Flag_MPU9250_G_Z_Cali;
		SF._uORB_Gryo__Roll = (SF._uORB_Gryo__Roll * 0.7) + ((SF._uORB_MPU9250_G_Y / DF._flag_MPU9250_LSB) * 0.3);
		SF._uORB_Gryo_Pitch = (SF._uORB_Gryo_Pitch * 0.7) + ((SF._uORB_MPU9250_G_X / DF._flag_MPU9250_LSB) * 0.3);
		SF._uORB_Gryo___Yaw = (SF._uORB_Gryo___Yaw * 0.7) + ((SF._uORB_MPU9250_G_Z / DF._flag_MPU9250_LSB) * 0.3);
		//ACCEL---------------------------------------------------------------------//
		SF._Tmp_IMU_Accel_Vector = sqrt((SF._uORB_MPU9250_A_X * SF._uORB_MPU9250_A_X) + (SF._uORB_MPU9250_A_Y * SF._uORB_MPU9250_A_Y) + (SF._uORB_MPU9250_A_Z * SF._uORB_MPU9250_A_Z));
		if (abs(SF._uORB_MPU9250_A_X) < SF._Tmp_IMU_Accel_Vector)
			SF._uORB_Accel__Roll = asin((float)SF._uORB_MPU9250_A_X / SF._Tmp_IMU_Accel_Vector) * -57.296;
		if (abs(SF._uORB_MPU9250_A_Y) < SF._Tmp_IMU_Accel_Vector)
			SF._uORB_Accel_Pitch = asin((float)SF._uORB_MPU9250_A_Y / SF._Tmp_IMU_Accel_Vector) * 57.296;
		SF._uORB_Accel__Roll -= SF._Flag_Accel__Roll_Cali;
		SF._uORB_Accel_Pitch -= SF._Flag_Accel_Pitch_Cali;
		//Gryo_MIX_ACCEL------------------------------------------------------------//
		SF._uORB_Real_Pitch += SF._uORB_MPU9250_G_X / Update_Freqeuncy / DF._flag_MPU9250_LSB;
		SF._uORB_Real__Roll += SF._uORB_MPU9250_G_Y / Update_Freqeuncy / DF._flag_MPU9250_LSB;
		SF._uORB_Real_Pitch -= SF._uORB_Real__Roll * sin((SF._uORB_MPU9250_G_Z / Update_Freqeuncy / DF._flag_MPU9250_LSB) * (3.14 / 180));
		SF._uORB_Real__Roll += SF._uORB_Real_Pitch * sin((SF._uORB_MPU9250_G_Z / Update_Freqeuncy / DF._flag_MPU9250_LSB) * (3.14 / 180));
		if (!_flag_first_StartUp)
		{
			SF._uORB_Real_Pitch = SF._uORB_Real_Pitch * 0.999 + SF._uORB_Accel_Pitch * 0.001;
			SF._uORB_Real__Roll = SF._uORB_Real__Roll * 0.999 + SF._uORB_Accel__Roll * 0.001;
		}
		else
		{
			SF._uORB_Real_Pitch = SF._uORB_Accel_Pitch;
			SF._uORB_Real__Roll = SF._uORB_Accel__Roll;
			_flag_first_StartUp = false;
		}
	}

	inline void ControlParse()
	{
		ControlRead();
		if (RF._uORB_RC__Roll < RF._flag_RC_Middle__Roll + 10 && RF._uORB_RC__Roll > RF._flag_RC_Middle__Roll - 10)
			RF._uORB_RC_Out__Roll = 0;
		else
			RF._uORB_RC_Out__Roll = (RF._uORB_RC__Roll - RF._flag_RC_Middle__Roll) / 3;

		if (RF._uORB_RC_Pitch < RF._flag_RC_Middle_Pitch + 10 && RF._uORB_RC_Pitch > RF._flag_RC_Middle_Pitch - 10)
			RF._uORB_RC_Out_Pitch = 0;
		else
			RF._uORB_RC_Out_Pitch = (RF._uORB_RC_Pitch - RF._flag_RC_Middle_Pitch) / 3;

		if (RF._uORB_RC___Yaw < RF._flag_RC_Middle___Yaw + 10 && RF._uORB_RC___Yaw > RF._flag_RC_Middle___Yaw - 10)
			RF._uORB_RC_Out___Yaw = 0;
		else
			RF._uORB_RC_Out___Yaw = (RF._uORB_RC___Yaw - RF._flag_RC_Middle___Yaw) / 3;
	}

	inline void ControlParse(int _Fix_Roll, int _Fix_Pitch, int _Fix_Throttle, int _Fix_Yaw, bool MixControl)
	{
		RF._uORB_RC_Out__Roll += _Fix_Roll;
		RF._uORB_RC_Out_Pitch += _Fix_Pitch;
		RF._uORB_RC_Out_Throttle += _Fix_Throttle;
		RF._uORB_RC_Out___Yaw += _Fix_Yaw;
		if (MixControl)
		{
			ControlRead();
		}
		if (RF._uORB_RC__Roll < RF._flag_RC_Middle__Roll + 10 && RF._uORB_RC__Roll > RF._flag_RC_Middle__Roll - 10)
			RF._uORB_RC_Out__Roll = 0;
		else
			RF._uORB_RC_Out__Roll = (RF._uORB_RC__Roll - RF._flag_RC_Middle__Roll) / 3;

		if (RF._uORB_RC_Pitch < RF._flag_RC_Middle_Pitch + 10 && RF._uORB_RC_Pitch > RF._flag_RC_Middle_Pitch - 10)
			RF._uORB_RC_Out_Pitch = 0;
		else
			RF._uORB_RC_Out_Pitch = (RF._uORB_RC_Pitch - RF._flag_RC_Middle_Pitch) / 3;

		if (RF._uORB_RC___Yaw < RF._flag_RC_Middle___Yaw + 10 && RF._uORB_RC___Yaw > RF._flag_RC_Middle___Yaw - 10)
			RF._uORB_RC_Out___Yaw = 0;
		else
			RF._uORB_RC_Out___Yaw = (RF._uORB_RC___Yaw - RF._flag_RC_Middle___Yaw) / 3;
	}

	inline void AttitudeUpdate()
	{
		//Roll PID Mix
		PF._uORB_PID__Roll_Input = SF._uORB_Gryo__Roll + SF._uORB_Real__Roll * 15 - RF._uORB_RC_Out__Roll;
		PID_Caculate(PF._uORB_PID__Roll_Input, PF._uORB_Leveling__Roll,
			PF._uORB_PID_I_Last_Value__Roll, PF._uORB_PID_D_Last_Value__Roll,
			PF._flag_PID_P__Roll_Gain, PF._flag_PID_I__Roll_Gain, PF._flag_PID_D__Roll_Gain, PF._flag_PID_I__Roll_Max__Value);
		if (PF._uORB_Leveling__Roll > PF._flag_PID_Level_Max)
			PF._uORB_Leveling__Roll = PF._flag_PID_Level_Max;
		if (PF._uORB_Leveling__Roll < PF._flag_PID_Level_Max * -1)
			PF._uORB_Leveling__Roll = PF._flag_PID_Level_Max * -1;

		//Pitch PID Mix
		PF._uORB_PID_Pitch_Input = SF._uORB_Gryo_Pitch + SF._uORB_Real_Pitch * 15 - RF._uORB_RC_Out_Pitch;
		PID_Caculate(PF._uORB_PID_Pitch_Input, PF._uORB_Leveling_Pitch,
			PF._uORB_PID_I_Last_Value_Pitch, PF._uORB_PID_D_Last_Value_Pitch,
			PF._flag_PID_P_Pitch_Gain, PF._flag_PID_I_Pitch_Gain, PF._flag_PID_D_Pitch_Gain, PF._flag_PID_I_Pitch_Max__Value);
		if (PF._uORB_Leveling_Pitch > PF._flag_PID_Level_Max)
			PF._uORB_Leveling_Pitch = PF._flag_PID_Level_Max;
		if (PF._uORB_Leveling_Pitch < PF._flag_PID_Level_Max * -1)
			PF._uORB_Leveling_Pitch = PF._flag_PID_Level_Max * -1;

		//Yaw PID Mix
		PID_Caculate(SF._uORB_Gryo___Yaw - RF._uORB_RC_Out___Yaw, PF._uORB_Leveling___Yaw,
			PF._uORB_PID_I_Last_Value___Yaw, PF._uORB_PID_D_Last_Value___Yaw,
			PF._flag_PID_P___Yaw_Gain, PF._flag_PID_I___Yaw_Gain, PF._flag_PID_D___Yaw_Gain, PF._flag_PID_I___Yaw_Max__Value);
		if (PF._uORB_Leveling___Yaw > PF._flag_PID_Level_Max)
			PF._uORB_Leveling___Yaw = PF._flag_PID_Level_Max;
		if (PF._uORB_Leveling___Yaw < PF._flag_PID_Level_Max * -1)
			PF._uORB_Leveling___Yaw = PF._flag_PID_Level_Max * -1;

		EF._uORB_B1_Speed = RF._uORB_RC_Throttle - PF._uORB_Leveling__Roll + PF._uORB_Leveling_Pitch + PF._uORB_Leveling___Yaw;
		EF._uORB_A1_Speed = RF._uORB_RC_Throttle - PF._uORB_Leveling__Roll - PF._uORB_Leveling_Pitch - PF._uORB_Leveling___Yaw;
		EF._uORB_A2_Speed = RF._uORB_RC_Throttle + PF._uORB_Leveling__Roll - PF._uORB_Leveling_Pitch + PF._uORB_Leveling___Yaw;
		EF._uORB_B2_Speed = RF._uORB_RC_Throttle + PF._uORB_Leveling__Roll + PF._uORB_Leveling_Pitch - PF._uORB_Leveling___Yaw;
	}

	inline bool SaftyChecking(APMSafeStatus& status)
	{
		if (RF._uORB_RC_Throttle < RF._flag_RC_Min_Throttle + 20 && RF._flag_RC_Safe_Area - 50 < RF._uORB_RC__Safe && RF._uORB_RC__Safe < RF._flag_RC_Safe_Area + 50)
		{
			if (_flag_Device_setupFailed == false)
			{
				if (_flag_Error == false)
				{
					if (_flag_StartUP_Protect == false)
					{
						_flag_ForceFailed_Safe = false;
					}
				}
				else
				{
					_flag_ForceFailed_Safe = true;
				}
			}
		}
		else if (!(RF._flag_RC_Safe_Area - 50 < RF._uORB_RC__Safe && RF._uORB_RC__Safe < RF._flag_RC_Safe_Area + 50))
		{
			_flag_StartUP_Protect = false;
			_flag_ForceFailed_Safe = true;
			_flag_Error = false;
		}
		else if (RF._uORB_RC_Throttle < RF._flag_RC_Max_Throttle + 20)
		{
			if (_flag_Error == true)
			{
				_flag_ForceFailed_Safe = true;
			}
		}
		if (RF._flag_RC_Safe_Area - 50 < RF._uORB_RC__Safe && RF._uORB_RC__Safe < RF._flag_RC_Safe_Area + 50)
		{
			if (RF._uORB_RC_Throttle > RF._flag_RC_Min_Throttle + 20)
			{

				_flag_StartUP_Protect = true;
			}
		}
		if (_flag_ForceFailed_Safe == true)
		{
			_flag_first_StartUp = true;
			PF._uORB_PID_D_Last_Value__Roll = 0;
			PF._uORB_PID_D_Last_Value_Pitch = 0;
			PF._uORB_PID_D_Last_Value___Yaw = 0;
			PF._uORB_PID_I_Last_Value__Roll = 0;
			PF._uORB_PID_I_Last_Value_Pitch = 0;
			PF._uORB_PID_I_Last_Value___Yaw = 0;
		}

		if (Attitude_loopTime > Update_Freq_Time)
		{
			std::cout << "core error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
			_flag_Error = true;
			status.Is_SyncTimeOut = true;
		}
		if (SF._uORB_Real_Pitch > 70.0 || SF._uORB_Real_Pitch < -70.0 || SF._uORB_Real__Roll > 70.0 || SF._uORB_Real__Roll < -70.0)
		{
			_flag_Error = true;
			status.Is_AngelOutLimit = true;
		}

		Clocking += Update_Freq_Time / 1000;
		if (_flag_RC_Disconnected == true)
		{
			Lose_Clocking += 1;
			if (Lose_Clocking == 500)
			{
				_flag_Error = true;
				status.Is_RCDisconnect = true;
				Lose_Clocking = 0;
			}
		}
		else if (_flag_RC_Disconnected == false)
		{
			Lose_Clocking = 0;
			status.Is_RCDisconnect = false;
		}
		status.ForceFailedSafe = _flag_ForceFailed_Safe;
		status.SafyError = _flag_Error;
	}

	inline void ESCUpdate()
	{
		EF._uORB_A1_Speed = (700 * (((float)EF._uORB_A1_Speed - (float)RF._flag_RC_Min_Throttle) / (float)(RF._flag_RC_Max_Throttle - RF._flag_RC_Min_Throttle))) + 2300;
		EF._uORB_A2_Speed = (700 * (((float)EF._uORB_A2_Speed - (float)RF._flag_RC_Min_Throttle) / (float)(RF._flag_RC_Max_Throttle - RF._flag_RC_Min_Throttle))) + 2300;
		EF._uORB_B1_Speed = (700 * (((float)EF._uORB_B1_Speed - (float)RF._flag_RC_Min_Throttle) / (float)(RF._flag_RC_Max_Throttle - RF._flag_RC_Min_Throttle))) + 2300;
		EF._uORB_B2_Speed = (700 * (((float)EF._uORB_B2_Speed - (float)RF._flag_RC_Min_Throttle) / (float)(RF._flag_RC_Max_Throttle - RF._flag_RC_Min_Throttle))) + 2300;

		if (_flag_ForceFailed_Safe)
		{
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A1_Pin, 0, EF._flag_Lock_Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A2_Pin, 0, EF._flag_Lock_Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B1_Pin, 0, EF._flag_Lock_Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B2_Pin, 0, EF._flag_Lock_Throttle);
		}
		if (!_flag_ForceFailed_Safe)
		{
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A1_Pin, 0,
				EF._uORB_A1_Speed);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A2_Pin, 0,
				EF._uORB_A2_Speed);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B1_Pin, 0,
				EF._uORB_B1_Speed);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B2_Pin, 0,
				EF._uORB_B2_Speed);
		}
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
		for (int cali_count = 0; cali_count < 2000; cali_count++)
		{
			SensorsDataRead();
			SF._Tmp_IMU_Accel_Vector = sqrt((SF._uORB_MPU9250_A_X * SF._uORB_MPU9250_A_X) + (SF._uORB_MPU9250_A_Y * SF._uORB_MPU9250_A_Y) + (SF._uORB_MPU9250_A_Z * SF._uORB_MPU9250_A_Z));
			if (abs(SF._uORB_MPU9250_A_X) < SF._Tmp_IMU_Accel_Vector)
				SF._uORB_Accel__Roll = asin((float)SF._uORB_MPU9250_A_X / SF._Tmp_IMU_Accel_Vector) * -57.296;
			if (abs(SF._uORB_MPU9250_A_Y) < SF._Tmp_IMU_Accel_Vector)
				SF._uORB_Accel_Pitch = asin((float)SF._uORB_MPU9250_A_Y / SF._Tmp_IMU_Accel_Vector) * 57.296;
			SF._Flag_Accel__Roll_Cali += SF._uORB_Accel__Roll;
			SF._Flag_Accel_Pitch_Cali += SF._uORB_Accel_Pitch;
			usleep(3);
		}
		SF._Flag_Accel__Roll_Cali = SF._Flag_Accel__Roll_Cali / 2000;
		SF._Flag_Accel_Pitch_Cali = SF._Flag_Accel_Pitch_Cali / 2000;
		std::cout << "AccelPitchCali: " << SF._Flag_Accel_Pitch_Cali << " \n";
		std::cout << "AccelRollCali: " << SF._Flag_Accel__Roll_Cali << " \n";
		std::cout << "[Sensors] Accel Calibration finsh , input -1 to retry , input 1 to write to configJSON , 0 to skip" << "\n";
		std::cin >> CalibrationComfirm;
		if (CalibrationComfirm == -1)
		{
			SensorsCalibration();
		}
		else if (CalibrationComfirm == 1)
		{
			std::ifstream config(configDir);
			std::string content((std::istreambuf_iterator<char>(config)),
				(std::istreambuf_iterator<char>()));
			nlohmann::json Configdata = nlohmann::json::parse(content);

			Configdata["_Flag_Accel__Roll_Cali"] = SF._Flag_Accel__Roll_Cali;
			Configdata["_Flag_Accel_Pitch_Cali"] = SF._Flag_Accel_Pitch_Cali;

			std::ofstream configIN;
			configIN.open(configDir);
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
		pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A1_Pin, 0, 3000);
		pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A2_Pin, 0, 3000);
		pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B1_Pin, 0, 3000);
		pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B2_Pin, 0, 3000);
		std::cout << "[ESCStatus] ESC calibration will finsh , input 1 to stop " << " \n";
		std::cin >> CalibrationComfirm;
		std::cout << "[ESCStatus] ESC calibration Finsh....." << " \n";
		pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A1_Pin, 0, 2200);
		pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A2_Pin, 0, 2200);
		pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B1_Pin, 0, 2200);
		pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B2_Pin, 0, 2200);
		sleep(3);
		std::cout << "[ESCStatus] ESC calibration over" << " \n";
	}

	inline void ControlCalibration()
	{
		bool IS_first = true;
		int CalibrationComfirm;
		std::cout << "[Controller] ControlCalibraion start , input 1 to start , input -1 to skip \n";
		std::cin >> CalibrationComfirm;
		if (CalibrationComfirm == -1)
		{
			return;
		}
		std::cout << "[Controller] ControlCalibraion start ...... \n";
		for (int cali_count = 0; cali_count < 4000; cali_count++)
		{
			ControlRead();
			std::cout << "Roll:" << RF._uORB_RC__Roll << " ";
			std::cout << "Pitch:" << RF._uORB_RC_Pitch << " ";
			std::cout << "Throttle:" << RF._uORB_RC_Throttle << " ";
			std::cout << "Yaw:" << RF._uORB_RC___Yaw << " ";
			std::cout << "SafeSwitch:" << RF._uORB_RC__Safe << " ";
			std::cout << "FuncSwitch:" << RF._uORB_RC__Func << " \r";
			if (IS_first)
			{
				if (RF._uORB_RC__Roll != 0)
				{
					RF._flag_RC_Min__Roll = RF._uORB_RC__Roll;
					RF._flag_RC_Min_Pitch = RF._uORB_RC__Roll;
					RF._flag_RC_Min_Throttle = RF._uORB_RC__Roll;
					RF._flag_RC_Min___Yaw = RF._uORB_RC__Roll;

					RF._flag_RC_Max__Roll = RF._uORB_RC__Roll;
					RF._flag_RC_Max_Pitch = RF._uORB_RC__Roll;
					RF._flag_RC_Max_Throttle = RF._uORB_RC__Roll;
					RF._flag_RC_Max___Yaw = RF._uORB_RC__Roll;

					IS_first = false;
				}
			}

			if (RF._uORB_RC__Roll > RF._flag_RC_Max__Roll&& RF._uORB_RC__Roll != 0)
				RF._flag_RC_Max__Roll = RF._uORB_RC__Roll;
			if (RF._uORB_RC_Pitch > RF._flag_RC_Max_Pitch&& RF._uORB_RC_Pitch != 0)
				RF._flag_RC_Max_Pitch = RF._uORB_RC_Pitch;
			if (RF._uORB_RC_Throttle > RF._flag_RC_Max_Throttle&& RF._uORB_RC_Throttle != 0)
				RF._flag_RC_Max_Throttle = RF._uORB_RC_Throttle;
			if (RF._uORB_RC___Yaw > RF._flag_RC_Max___Yaw&& RF._uORB_RC___Yaw != 0)
				RF._flag_RC_Max___Yaw = RF._uORB_RC___Yaw;

			if (RF._uORB_RC__Roll < RF._flag_RC_Min__Roll && RF._uORB_RC__Roll != 0)
				RF._flag_RC_Min__Roll = RF._uORB_RC__Roll;
			if (RF._uORB_RC_Pitch < RF._flag_RC_Min_Pitch && RF._uORB_RC_Pitch != 0)
				RF._flag_RC_Min_Pitch = RF._uORB_RC_Pitch;
			if (RF._uORB_RC_Throttle < RF._flag_RC_Min_Throttle && RF._uORB_RC_Throttle != 0)
				RF._flag_RC_Min_Throttle = RF._uORB_RC_Throttle;
			if (RF._uORB_RC___Yaw < RF._flag_RC_Min___Yaw && RF._uORB_RC___Yaw != 0)
				RF._flag_RC_Min___Yaw = RF._uORB_RC___Yaw;

			usleep(2000);
		}
		std::cout << "\n[Controller] Calibration will finshed"
			<< " Please Check the tick middle and throttle down and pass enter" << "\n";
		std::cin >> CalibrationComfirm;
		for (int cali_count = 0; cali_count < 1000; cali_count++)
		{
			ControlRead();
			std::cout << "Roll:" << RF._uORB_RC__Roll << " ";
			std::cout << "Pitch:" << RF._uORB_RC_Pitch << " ";
			std::cout << "Throttle:" << RF._uORB_RC_Throttle << " ";
			std::cout << "Yaw:" << RF._uORB_RC___Yaw << " ";
			std::cout << "SafeSwitch:" << RF._uORB_RC__Safe << " ";
			std::cout << "FuncSwitch:" << RF._uORB_RC__Func << " \r";

			RF._flag_RC_Middle_Pitch = RF._uORB_RC_Pitch;
			RF._flag_RC_Middle__Roll = RF._uORB_RC__Roll;
			RF._flag_RC_Middle___Yaw = RF._uORB_RC___Yaw;
			RF._flag_RC_Safe_Area = RF._uORB_RC__Safe;
			usleep(2000);
		}
		std::cout << "[Controler] Controller calitbration comfirm:\n"
			<< "Max__Roll    = " << RF._flag_RC_Max__Roll << "\n"
			<< "Max_Pitch    = " << RF._flag_RC_Max_Pitch << "\n"
			<< "Max_Throttle = " << RF._flag_RC_Max_Throttle << "\n"
			<< "Max___Yaw    = " << RF._flag_RC_Max___Yaw << "\n\n"

			<< "Middle__Roll = " << RF._flag_RC_Middle__Roll << "\n"
			<< "Middle_Pitch = " << RF._flag_RC_Middle_Pitch << "\n"
			<< "Middle___Yaw = " << RF._flag_RC_Middle___Yaw << "\n\n"

			<< "Min__Roll    = " << RF._flag_RC_Min__Roll << "\n"
			<< "Min_Pitch    = " << RF._flag_RC_Min_Pitch << "\n"
			<< "Min_Throttle = " << RF._flag_RC_Min_Throttle << "\n"
			<< "Min___Yaw    = " << RF._flag_RC_Min___Yaw << "\n"

			<< "SafeSwitch   = " << RF._flag_RC_Safe_Area << "\n";
		std::cout << "<-----------Controller_calibration_over---------------->\n";
		std::cout << "[Controller] Calibration finshed ,if you want to retry input -1 , to write to configJSON input 1 , input 0 to skip" << "\n";
		std::cin >> CalibrationComfirm;
		if (CalibrationComfirm == -1)
		{
			ControlCalibration();
		}
		else if (CalibrationComfirm == 1)
		{
			std::ifstream config(configDir);
			std::string content((std::istreambuf_iterator<char>(config)),
				(std::istreambuf_iterator<char>()));
			nlohmann::json Configdata = nlohmann::json::parse(content);

			Configdata["_flag_RC_Max__Roll"] = RF._flag_RC_Max__Roll;
			Configdata["_flag_RC_Max_Pitch"] = RF._flag_RC_Max_Pitch;
			Configdata["_flag_RC_Max_Throttle"] = RF._flag_RC_Max_Throttle;
			Configdata["_flag_RC_Max___Yaw"] = RF._flag_RC_Max___Yaw;

			Configdata["_flag_RC_Middle__Roll"] = RF._flag_RC_Middle__Roll;
			Configdata["_flag_RC_Middle_Pitch"] = RF._flag_RC_Middle_Pitch;
			Configdata["_flag_RC_Middle___Yaw"] = RF._flag_RC_Middle___Yaw;

			Configdata["_flag_RC_Min__Roll"] = RF._flag_RC_Min__Roll;
			Configdata["_flag_RC_Min_Pitch"] = RF._flag_RC_Min_Pitch;
			Configdata["_flag_RC_Min_Throttle"] = RF._flag_RC_Min_Throttle;
			Configdata["_flag_RC_Min___Yaw"] = RF._flag_RC_Min___Yaw;

			Configdata["_flag_RC_Safe_Area"] = RF._flag_RC_Safe_Area;

			std::ofstream configIN;
			configIN.open(configDir);
			configIN.clear();
			configIN << Configdata.dump(4).c_str();
			configIN.close();

			std::cout << "[Controller] Config write success\n";
		}
	}

private:
	int Update_Freqeuncy;
	long int Clocking;
	long int Lose_Clocking;
	bool _flag_Error;
	bool _flag_StartUP_Protect;
	bool _flag_first_StartUp;
	bool _flag_RC_Disconnected;
	bool _flag_ForceFailed_Safe;
	bool _flag_Device_setupFailed;
	char* configDir = "/etc/APMconfig.json";;

	struct DeviceINFO
	{
		int RCReader_fd;
		int PCA9658_fd;
		const int PWM_Freq = 400;
		const int PCA9685_PinBase = 65;
		const int PCA9685_Address = 0x40;
		int MPU9250_fd;
		int MPU9250_SPI_Channel = 1;
		const int MPU9250_ADDR = 0x68;
		float _flag_MPU9250_LSB = 65.5;
		int MPU9250_SPI_Freq = 1000000;
		int MS5611_fd;
		const int MS5611_ADDR = 0x70;
	}DF;

	struct SensorsINFO
	{
		int _Tmp_MPU9250_Buffer[14];
		unsigned char _Tmp_MPU9250_SPI_Config[5];
		unsigned char _Tmp_MPU9250_SPI_Buffer[28];

		long _uORB_MPU9250_A_X;
		long _uORB_MPU9250_A_Y;
		long _uORB_MPU9250_A_Z;
		long _uORB_MPU9250_G_X;
		long _uORB_MPU9250_G_Y;
		long _uORB_MPU9250_G_Z;

		float _uORB_Accel__Roll = 0;
		float _uORB_Accel_Pitch = 0;
		float _uORB_Gryo__Roll = 0;
		float _uORB_Gryo_Pitch = 0;
		float _uORB_Gryo___Yaw = 0;
		float _uORB_Real_Pitch = 0;
		float _uORB_Real__Roll = 0;

		unsigned long _Tmp_MPU9250_G_X;
		unsigned long _Tmp_MPU9250_G_Y;
		unsigned long _Tmp_MPU9250_G_Z;
		unsigned long _Tmp_MPU9250_A_X;
		unsigned long _Tmp_MPU9250_A_Y;
		unsigned long _Tmp_MPU9250_A_Z;

		long _Flag_MPU9250_G_X_Cali;
		long _Flag_MPU9250_G_Y_Cali;
		long _Flag_MPU9250_G_Z_Cali;
		long _Flag_Accel__Roll_Cali;
		long _Flag_Accel_Pitch_Cali;

		long _Tmp_IMU_Accel_Calibration[20];
		long _Tmp_IMU_Accel_Vector;
	}SF;

	struct PIDINFO
	{
		float _uORB_PID_D_Last_Value__Roll = 0;
		float _uORB_PID_D_Last_Value_Pitch = 0;
		float _uORB_PID_D_Last_Value___Yaw = 0;

		float _uORB_PID_I_Last_Value__Roll = 0;
		float _uORB_PID_I_Last_Value_Pitch = 0;
		float _uORB_PID_I_Last_Value___Yaw = 0;

		float _uORB_PID__Roll_Input = 0;
		float _uORB_PID_Pitch_Input = 0;

		float _uORB_Leveling__Roll;
		float _uORB_Leveling_Pitch;
		float _uORB_Leveling___Yaw;

		float _flag_PID_P__Roll_Gain;
		float _flag_PID_P_Pitch_Gain;
		float _flag_PID_P___Yaw_Gain;

		float _flag_PID_I__Roll_Gain;
		float _flag_PID_I_Pitch_Gain;
		float _flag_PID_I___Yaw_Gain;
		float _flag_PID_I__Roll_Max__Value;
		float _flag_PID_I_Pitch_Max__Value;
		float _flag_PID_I___Yaw_Max__Value;

		float _flag_PID_D__Roll_Gain;
		float _flag_PID_D_Pitch_Gain;
		float _flag_PID_D___Yaw_Gain;

		float _flag_PID_Level_Max;
	}PF;

	struct RCINFO
	{
		int _Tmp_RC_Data[36];
		int _uORB_RC__Safe = 0;
		int _uORB_RC__Func = 0;
		int _uORB_RC__Roll = 0;
		int _uORB_RC_Pitch = 0;
		int _uORB_RC_Throttle = 0;
		int _uORB_RC___Yaw = 0;

		int _uORB_RC_Out__Roll = 0;
		int _uORB_RC_Out_Pitch = 0;
		int _uORB_RC_Out_Throttle = 0;
		int _uORB_RC_Out___Yaw = 0;

		int _flag_RC_Max__Roll;
		int _flag_RC_Max_Pitch;
		int _flag_RC_Max_Throttle;
		int _flag_RC_Max___Yaw;

		int _flag_RC_Middle__Roll;
		int _flag_RC_Middle_Pitch;
		int _flag_RC_Middle___Yaw;

		int _flag_RC_Min__Roll;
		int _flag_RC_Min_Pitch;
		int _flag_RC_Min_Throttle;
		int _flag_RC_Min___Yaw;

		int _flag_RC_Safe_Area;
	}RF;

	struct ESCINFO
	{
		int _uORB_A1_Speed;
		int _uORB_A2_Speed;
		int _uORB_B1_Speed;
		int _uORB_B2_Speed;
		int _flag_A1_Pin = 0;
		int _flag_A2_Pin = 1;
		int _flag_B1_Pin = 2;
		int _flag_B2_Pin = 3;
		const int _flag_Lazy_Throttle = 2300;
		const int _flag_Lock_Throttle = 2200;
	}EF;

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
		std::ifstream config(configDir);
		std::string content((std::istreambuf_iterator<char>(config)),
			(std::istreambuf_iterator<char>()));
		nlohmann::json Configdata = nlohmann::json::parse(content);
		//==========================================================Controller cofig==/
		RF._flag_RC_Max__Roll = Configdata["_flag_RC_Max__Roll"].get<int>();
		RF._flag_RC_Max_Pitch = Configdata["_flag_RC_Max_Pitch"].get<int>();
		RF._flag_RC_Max_Throttle = Configdata["_flag_RC_Max_Throttle"].get<int>();
		RF._flag_RC_Max___Yaw = Configdata["_flag_RC_Max___Yaw"].get<int>();

		RF._flag_RC_Middle__Roll = Configdata["_flag_RC_Middle__Roll"].get<int>();
		RF._flag_RC_Middle_Pitch = Configdata["_flag_RC_Middle_Pitch"].get<int>();
		RF._flag_RC_Middle___Yaw = Configdata["_flag_RC_Middle___Yaw"].get<int>();

		RF._flag_RC_Min__Roll = Configdata["_flag_RC_Min__Roll"].get<int>();
		RF._flag_RC_Min_Pitch = Configdata["_flag_RC_Min_Pitch"].get<int>();
		RF._flag_RC_Min_Throttle = Configdata["_flag_RC_Min_Throttle"].get<int>();
		RF._flag_RC_Min___Yaw = Configdata["_flag_RC_Min___Yaw"].get<int>();

		RF._flag_RC_Safe_Area = Configdata["_flag_RC_Safe_Area"].get<int>();

		EF._flag_A1_Pin = Configdata["_flag_A1_Pin"].get<int>();
		EF._flag_A2_Pin = Configdata["_flag_A2_Pin"].get<int>();
		EF._flag_B1_Pin = Configdata["_flag_B1_Pin"].get<int>();
		EF._flag_B2_Pin = Configdata["_flag_B2_Pin"].get<int>();
		//==================================================================PID cofig==/
		PF._flag_PID_P__Roll_Gain = Configdata["_flag_PID_P__Roll_Gain"].get<float>();
		PF._flag_PID_P_Pitch_Gain = Configdata["_flag_PID_P_Pitch_Gain"].get<float>();
		PF._flag_PID_P___Yaw_Gain = Configdata["_flag_PID_P___Yaw_Gain"].get<float>();

		PF._flag_PID_I__Roll_Gain = Configdata["_flag_PID_I__Roll_Gain"].get<float>();
		PF._flag_PID_I_Pitch_Gain = Configdata["_flag_PID_I_Pitch_Gain"].get<float>();
		PF._flag_PID_I___Yaw_Gain = Configdata["_flag_PID_I___Yaw_Gain"].get<float>();
		PF._flag_PID_I__Roll_Max__Value = Configdata["_flag_PID_I__Roll_Max__Value"].get<float>();
		PF._flag_PID_I_Pitch_Max__Value = Configdata["_flag_PID_I_Pitch_Max__Value"].get<float>();
		PF._flag_PID_I___Yaw_Max__Value = Configdata["_flag_PID_I___Yaw_Max__Value"].get<float>();

		PF._flag_PID_D__Roll_Gain = Configdata["_flag_PID_D__Roll_Gain"].get<float>();
		PF._flag_PID_D_Pitch_Gain = Configdata["_flag_PID_D_Pitch_Gain"].get<float>();
		PF._flag_PID_D___Yaw_Gain = Configdata["_flag_PID_D___Yaw_Gain"].get<float>();

		PF._flag_PID_Level_Max = Configdata["_flag_PID_Level_Max"].get<float>();
		//==============================================================Sensors cofig==/
		SF._Flag_Accel__Roll_Cali = Configdata["_Flag_Accel__Roll_Cali"].get<float>();
		SF._Flag_Accel_Pitch_Cali = Configdata["_Flag_Accel_Pitch_Cali"].get<float>();
		//===============================================================Update cofig==/
		Update_Freqeuncy = Configdata["Update_Freqeucy"].get<int>();
		std::cout << "[ConfigRead]Config Set Success!\n";
	}

	inline void ControlRead()
	{
#ifdef SBUS_CONVERTER
		if (serialDataAvail(DF.RCReader_fd) > 0)
		{
			RF._Tmp_RC_Data[0] = serialGetchar(DF.RCReader_fd);
			if (RF._Tmp_RC_Data[0] == 15)
			{
				for (int i = 1; i <= 34; i++)
				{
					RF._Tmp_RC_Data[i] = serialGetchar(DF.RCReader_fd);
				}
				RF._uORB_RC__Roll = RF._Tmp_RC_Data[1] * 255 + RF._Tmp_RC_Data[2];
				RF._uORB_RC_Pitch = RF._Tmp_RC_Data[3] * 255 + RF._Tmp_RC_Data[4];
				RF._uORB_RC_Throttle = RF._Tmp_RC_Data[5] * 255 + RF._Tmp_RC_Data[6];
				RF._uORB_RC___Yaw = RF._Tmp_RC_Data[7] * 255 + RF._Tmp_RC_Data[8];
				RF._uORB_RC__Safe = RF._Tmp_RC_Data[9] * 255 + RF._Tmp_RC_Data[10];
				serialFlush(DF.RCReader_fd);
			}
			else if (RF._Tmp_RC_Data[0] != 15)
			{
				_flag_RC_Disconnected = true;
				serialFlush(DF.RCReader_fd);
			}
		}
#endif

#ifdef IBUS_Serial
		if (serialDataAvail(DF.RCReader_fd) > 0)
		{
			if (serialGetchar(DF.RCReader_fd) == 64)
			{
				for (int i = 0; i < 32; i++)
				{
					if (serialDataAvail(DF.RCReader_fd) > 0)
					{
						RF._Tmp_RC_Data[i] = serialGetchar(DF.RCReader_fd);
					}
				}
				if (RF._Tmp_RC_Data[31] == 64)
				{
					RF._uORB_RC__Roll = RF._Tmp_RC_Data[1] * 255 + RF._Tmp_RC_Data[0];
					RF._uORB_RC_Pitch = RF._Tmp_RC_Data[3] * 255 + RF._Tmp_RC_Data[2];
					RF._uORB_RC_Throttle = RF._Tmp_RC_Data[5] * 255 + RF._Tmp_RC_Data[4];
					RF._uORB_RC___Yaw = RF._Tmp_RC_Data[7] * 255 + RF._Tmp_RC_Data[6];
					RF._uORB_RC__Safe = RF._Tmp_RC_Data[9] * 255 + RF._Tmp_RC_Data[8];
					serialFlush(DF.RCReader_fd);
					_flag_RC_Disconnected = false;
				}
				else if (RF._Tmp_RC_Data[31] != 64)
				{
					_flag_RC_Disconnected = true;
					serialFlush(DF.RCReader_fd);
				}
			}
		}
		else
		{
			_flag_RC_Disconnected = true;
		}
#endif
	}

	inline void SensorsDataRead()
	{
#ifdef I2C_MPU9250
		SF._Tmp_MPU9250_Buffer[0] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x3B);
		SF._Tmp_MPU9250_Buffer[1] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x3C);
		SF._Tmp_MPU9250_A_X = (SF._Tmp_MPU9250_Buffer[0] << 8 | SF._Tmp_MPU9250_Buffer[1]);
		SF._uORB_MPU9250_A_X = (short)SF._Tmp_MPU9250_A_X;
		SF._Tmp_MPU9250_Buffer[2] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x3D);
		SF._Tmp_MPU9250_Buffer[3] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x3E);
		SF._Tmp_MPU9250_A_Y = (SF._Tmp_MPU9250_Buffer[2] << 8 | SF._Tmp_MPU9250_Buffer[3]);
		SF._uORB_MPU9250_A_Y = (short)SF._Tmp_MPU9250_A_Y;
		SF._Tmp_MPU9250_Buffer[4] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x3F);
		SF._Tmp_MPU9250_Buffer[5] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x40);
		SF._Tmp_MPU9250_A_Z = (SF._Tmp_MPU9250_Buffer[4] << 8 | SF._Tmp_MPU9250_Buffer[5]);
		SF._uORB_MPU9250_A_Z = (short)SF._Tmp_MPU9250_A_Z;

		SF._Tmp_MPU9250_Buffer[6] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x43);
		SF._Tmp_MPU9250_Buffer[7] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x44);
		SF._Tmp_MPU9250_G_X = (SF._Tmp_MPU9250_Buffer[6] << 8 | SF._Tmp_MPU9250_Buffer[7]);
		SF._uORB_MPU9250_G_X = (short)SF._Tmp_MPU9250_G_X;
		SF._Tmp_MPU9250_Buffer[8] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x45);
		SF._Tmp_MPU9250_Buffer[9] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x46);
		SF._Tmp_MPU9250_G_Y = (SF._Tmp_MPU9250_Buffer[8] << 8 | SF._Tmp_MPU9250_Buffer[9]);
		SF._uORB_MPU9250_G_Y = (short)SF._Tmp_MPU9250_G_Y;
		SF._Tmp_MPU9250_Buffer[10] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x47);
		SF._Tmp_MPU9250_Buffer[11] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x48);
		SF._Tmp_MPU9250_G_Z = (SF._Tmp_MPU9250_Buffer[10] << 8 | SF._Tmp_MPU9250_Buffer[11]);
		SF._uORB_MPU9250_G_Z = (short)SF._Tmp_MPU9250_G_Z;
#endif
#ifdef SPI_MPU9250
		//==================================Device==================//
		SF._Tmp_MPU9250_SPI_Buffer[0] = 0xBB;
		wiringPiSPIDataRW(1, SF._Tmp_MPU9250_SPI_Buffer, 20);
		SF._Tmp_MPU9250_A_X = ((int)SF._Tmp_MPU9250_SPI_Buffer[1] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[2]);
		SF._uORB_MPU9250_A_X = (short)SF._Tmp_MPU9250_A_X;
		SF._Tmp_MPU9250_A_Y = ((int)SF._Tmp_MPU9250_SPI_Buffer[3] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[4]);
		SF._uORB_MPU9250_A_Y = (short)SF._Tmp_MPU9250_A_Y;
		SF._Tmp_MPU9250_A_Z = ((int)SF._Tmp_MPU9250_SPI_Buffer[5] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[6]);
		SF._uORB_MPU9250_A_Z = (short)SF._Tmp_MPU9250_A_Z;

		SF._Tmp_MPU9250_G_X = ((int)SF._Tmp_MPU9250_SPI_Buffer[9] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[10]);
		SF._uORB_MPU9250_G_X = (short)SF._Tmp_MPU9250_G_X;
		SF._Tmp_MPU9250_G_Y = ((int)SF._Tmp_MPU9250_SPI_Buffer[11] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[12]);
		SF._uORB_MPU9250_G_Y = (short)SF._Tmp_MPU9250_G_Y;
		SF._Tmp_MPU9250_G_Z = ((int)SF._Tmp_MPU9250_SPI_Buffer[13] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[14]);
		SF._uORB_MPU9250_G_Z = (short)SF._Tmp_MPU9250_G_Z;
#endif
	}
};