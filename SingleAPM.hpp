#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <wiringPiSPI.h>
#include <math.h>
#include <thread>
#include <string>
#include <fstream>
#include <unistd.h>
#include <iostream>
#include "_thirdparty/pca9685.h"
#include "_thirdparty/Sbus/src/RPiSbus.h"

#ifdef USINGJSON
#include <nlohmann/json.hpp>
#endif

enum APMS_MPU9250Type
{
	MPU_Is_I2C,
	MPU_Is_SPI
};

enum APMS_RCType
{
	RC_Is_IBUS,
	RC_IS_SBUS
};

struct APMSafeStatus
{
	int SyncTime;
	bool ForceFailedSafe;
	bool SafyError;

	bool Is_SyncTimeOut;
	bool Is_RCDisconnect;
	bool Is_AngelOutLimit;
};

struct APMSettinngs
{
	APMS_MPU9250Type MPU9250_Type;
	APMS_RCType RC_Type;

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

	double _flag_Accel__Roll_Cali;
	double _flag_Accel_Pitch_Cali;

	int _flag_A1_Pin = 0;
	int _flag_A2_Pin = 1;
	int _flag_B1_Pin = 2;
	int _flag_B2_Pin = 3;
};

class RPiSingleAPM
{
public:
	RPiSingleAPM(APMSettinngs APMInit)
	{
		AF.RC_Lose_Clocking = 0;
		AF._flag_first_StartUp = true;
		AF._flag_ForceFailed_Safe = true;
		ConfigReader(APMInit);

		wiringPiSetupSys();
		piHiPri(99);
		DF.PCA9658_fd = pca9685Setup(DF.PCA9685_PinBase, DF.PCA9685_Address, DF.PWM_Freq);

		if (SF.MPU9250_Type == APMS_MPU9250Type::MPU_Is_I2C)
		{
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
		}
		else if (SF.MPU9250_Type == APMS_MPU9250Type::MPU_Is_SPI)
		{
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
		}

		if (RF.RC_Type == APMS_RCType::RC_Is_IBUS)
		{
			DF.RCReader_fd = serialOpen("/dev/ttyS0", 115200);
			if (DF.RCReader_fd < 0)
			{

			}
		}
		else if (RF.RC_Type == APMS_RCType::RC_IS_SBUS)
		{
			SbusInit = new Sbus("/dev/ttyS0", SbusMode::Normal);
		}

		GryoCali();
	}

	inline void SensorsParse()
	{
		AF.Update_TimerStart = micros();
		SensorsDataRead();
		//Gryo----------------------------------------------------------------------//
		SF._uORB_MPU9250_G_X -= SF._flag_MPU9250_G_X_Cali;
		SF._uORB_MPU9250_G_Y -= SF._flag_MPU9250_G_Y_Cali;
		SF._uORB_MPU9250_G_Z -= SF._flag_MPU9250_G_Z_Cali;
		SF._uORB_Gryo__Roll = (SF._uORB_Gryo__Roll * 0.7) + ((SF._uORB_MPU9250_G_Y / DF._flag_MPU9250_LSB) * 0.3);
		SF._uORB_Gryo_Pitch = (SF._uORB_Gryo_Pitch * 0.7) + ((SF._uORB_MPU9250_G_X / DF._flag_MPU9250_LSB) * 0.3);
		SF._uORB_Gryo___Yaw = (SF._uORB_Gryo___Yaw * 0.7) + ((SF._uORB_MPU9250_G_Z / DF._flag_MPU9250_LSB) * 0.3);
		//ACCEL---------------------------------------------------------------------//
		SF._Tmp_IMU_Accel_Vector = sqrt((SF._uORB_MPU9250_A_X * SF._uORB_MPU9250_A_X) + (SF._uORB_MPU9250_A_Y * SF._uORB_MPU9250_A_Y) + (SF._uORB_MPU9250_A_Z * SF._uORB_MPU9250_A_Z));
		if (abs(SF._uORB_MPU9250_A_X) < SF._Tmp_IMU_Accel_Vector)
			SF._uORB_Accel__Roll = asin((float)SF._uORB_MPU9250_A_X / SF._Tmp_IMU_Accel_Vector) * -57.296;
		if (abs(SF._uORB_MPU9250_A_Y) < SF._Tmp_IMU_Accel_Vector)
			SF._uORB_Accel_Pitch = asin((float)SF._uORB_MPU9250_A_Y / SF._Tmp_IMU_Accel_Vector) * 57.296;
		SF._uORB_Accel__Roll -= SF._flag_Accel__Roll_Cali;
		SF._uORB_Accel_Pitch -= SF._flag_Accel_Pitch_Cali;
		//Gryo_MIX_ACCEL------------------------------------------------------------//
		SF._uORB_Real_Pitch += SF._uORB_MPU9250_G_X / AF.Update_Freqeuncy / DF._flag_MPU9250_LSB;
		SF._uORB_Real__Roll += SF._uORB_MPU9250_G_Y / AF.Update_Freqeuncy / DF._flag_MPU9250_LSB;
		SF._uORB_Real_Pitch -= SF._uORB_Real__Roll * sin((SF._uORB_MPU9250_G_Z / AF.Update_Freqeuncy / DF._flag_MPU9250_LSB) * (3.14 / 180));
		SF._uORB_Real__Roll += SF._uORB_Real_Pitch * sin((SF._uORB_MPU9250_G_Z / AF.Update_Freqeuncy / DF._flag_MPU9250_LSB) * (3.14 / 180));
		if (!AF._flag_first_StartUp)
		{
			SF._uORB_Real_Pitch = SF._uORB_Real_Pitch * 0.999 + SF._uORB_Accel_Pitch * 0.001;
			SF._uORB_Real__Roll = SF._uORB_Real__Roll * 0.999 + SF._uORB_Accel__Roll * 0.001;
		}
		else
		{
			SF._uORB_Real_Pitch = SF._uORB_Accel_Pitch;
			SF._uORB_Real__Roll = SF._uORB_Accel__Roll;
			AF._flag_first_StartUp = false;
		}
	}

	inline void ControlParse(int* ChannelOut)
	{
		ControlRead();
		if (RF._uORB_RC_Channel_PWM[0] < RF._flag_RC_Mid_PWM_Value + 10 && RF._uORB_RC_Channel_PWM[0] > RF._flag_RC_Mid_PWM_Value - 10)
			RF._uORB_RC_Out__Roll = 0;
		else
			RF._uORB_RC_Out__Roll = (RF._uORB_RC_Channel_PWM[0] - RF._flag_RC_Mid_PWM_Value) / 3;
		//
		if (RF._uORB_RC_Channel_PWM[1] < RF._flag_RC_Mid_PWM_Value + 10 && RF._uORB_RC_Channel_PWM[1] > RF._flag_RC_Mid_PWM_Value - 10)
			RF._uORB_RC_Out_Pitch = 0;
		else
			RF._uORB_RC_Out_Pitch = (RF._uORB_RC_Channel_PWM[1] - RF._flag_RC_Mid_PWM_Value) / 3;
		//
		RF._uORB_RC_Out_Throttle = RF._uORB_RC_Channel_PWM[2];
		//
		if (RF._uORB_RC_Channel_PWM[3] < RF._flag_RC_Mid_PWM_Value + 10 && RF._uORB_RC_Channel_PWM[3] > RF._flag_RC_Mid_PWM_Value - 10)
			RF._uORB_RC_Out___Yaw = 0;
		else
			RF._uORB_RC_Out___Yaw = (RF._uORB_RC_Channel_PWM[3] - RF._flag_RC_Mid_PWM_Value) / 3;
		//
		RF._uORB_RC_Out___ARM = RF._uORB_RC_Channel_PWM[4];
		//
		std::copy(std::begin(RF._uORB_RC_Channel_PWM), std::end(RF._uORB_RC_Channel_PWM), ChannelOut);
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

		EF._uORB_B1_Speed = RF._uORB_RC_Out_Throttle - PF._uORB_Leveling__Roll + PF._uORB_Leveling_Pitch + PF._uORB_Leveling___Yaw;
		EF._uORB_A1_Speed = RF._uORB_RC_Out_Throttle - PF._uORB_Leveling__Roll - PF._uORB_Leveling_Pitch - PF._uORB_Leveling___Yaw;
		EF._uORB_A2_Speed = RF._uORB_RC_Out_Throttle + PF._uORB_Leveling__Roll - PF._uORB_Leveling_Pitch + PF._uORB_Leveling___Yaw;
		EF._uORB_B2_Speed = RF._uORB_RC_Out_Throttle + PF._uORB_Leveling__Roll + PF._uORB_Leveling_Pitch - PF._uORB_Leveling___Yaw;
	}

	inline bool SaftyChecking(APMSafeStatus& status)
	{
		if (RF._uORB_RC_Out___ARM < RF._flag_RC_ARM_PWM_Value + 20
			&& RF._flag_RC_ARM_PWM_Value - 50 < RF._uORB_RC_Out___ARM
			&& RF._uORB_RC_Out___ARM < RF._flag_RC_ARM_PWM_Value + 50)
		{
			if (AF._flag_Device_setupFailed == false)
			{
				if (AF._flag_Error == false)
				{
					if (AF._flag_StartUP_Protect == false)
					{
						AF._flag_ForceFailed_Safe = false;
					}
				}
				else
				{
					AF._flag_ForceFailed_Safe = true;
				}
			}
		}
		else if (!(RF._flag_RC_ARM_PWM_Value - 50 < RF._uORB_RC_Out___ARM
			&& RF._uORB_RC_Out___ARM < RF._flag_RC_ARM_PWM_Value + 50))
		{
			AF._flag_StartUP_Protect = false;
			AF._flag_ForceFailed_Safe = true;
			AF._flag_Error = false;
		}
		else if (RF._uORB_RC_Out_Throttle < RF._flag_RC_Max_PWM_Value + 20)
		{
			if (AF._flag_Error == true)
			{
				AF._flag_ForceFailed_Safe = true;
			}
		}
		if (RF._flag_RC_ARM_PWM_Value - 50 < RF._uORB_RC_Out___ARM
			&& RF._uORB_RC_Out___ARM < RF._flag_RC_ARM_PWM_Value + 50)
		{
			if (RF._uORB_RC_Out_Throttle > RF._flag_RC_Min_PWM_Value + 20)
			{

				AF._flag_StartUP_Protect = true;
			}
		}
		if (AF._flag_ForceFailed_Safe == true)
		{
			AF._flag_first_StartUp = true;
			PF._uORB_PID_D_Last_Value__Roll = 0;
			PF._uORB_PID_D_Last_Value_Pitch = 0;
			PF._uORB_PID_D_Last_Value___Yaw = 0;
			PF._uORB_PID_I_Last_Value__Roll = 0;
			PF._uORB_PID_I_Last_Value_Pitch = 0;
			PF._uORB_PID_I_Last_Value___Yaw = 0;
		}

		if (AF.Update_loopTime > AF.Update_Freq_Time)
		{
			AF._flag_Error = true;
			status.Is_SyncTimeOut = true;
		}
		if (SF._uORB_Real_Pitch > 70.0 || SF._uORB_Real_Pitch < -70.0
			|| SF._uORB_Real__Roll > 70.0 || SF._uORB_Real__Roll < -70.0)
		{
			AF._flag_Error = true;
			status.Is_AngelOutLimit = true;
		}

		if (AF._flag_RC_Disconnected == true)
		{
			AF.RC_Lose_Clocking += 1;
			if (AF.RC_Lose_Clocking == 200)
			{
				AF._flag_Error = true;
				status.Is_RCDisconnect = true;
				AF.RC_Lose_Clocking = 0;
			}
		}
		else if (AF._flag_RC_Disconnected == false)
		{
			AF.RC_Lose_Clocking = 0;
			status.Is_RCDisconnect = false;
		}

		status.ForceFailedSafe = AF._flag_ForceFailed_Safe;
		status.SafyError = AF._flag_Error;
		status.SyncTime = AF.Update_loopTime;
	}

	inline void ESCUpdate()
	{
		EF._uORB_A1_Speed = (700 * (((float)EF._uORB_A1_Speed - (float)RF._flag_RC_Min_PWM_Value) / (float)(RF._flag_RC_Max_PWM_Value - RF._flag_RC_Min_PWM_Value))) + 2300;
		EF._uORB_A2_Speed = (700 * (((float)EF._uORB_A2_Speed - (float)RF._flag_RC_Min_PWM_Value) / (float)(RF._flag_RC_Max_PWM_Value - RF._flag_RC_Min_PWM_Value))) + 2300;
		EF._uORB_B1_Speed = (700 * (((float)EF._uORB_B1_Speed - (float)RF._flag_RC_Min_PWM_Value) / (float)(RF._flag_RC_Max_PWM_Value - RF._flag_RC_Min_PWM_Value))) + 2300;
		EF._uORB_B2_Speed = (700 * (((float)EF._uORB_B2_Speed - (float)RF._flag_RC_Min_PWM_Value) / (float)(RF._flag_RC_Max_PWM_Value - RF._flag_RC_Min_PWM_Value))) + 2300;

		if (AF._flag_ForceFailed_Safe)
		{
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A1_Pin, 0, EF._Flag_Lock_Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A2_Pin, 0, EF._Flag_Lock_Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B1_Pin, 0, EF._Flag_Lock_Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B2_Pin, 0, EF._Flag_Lock_Throttle);
		}
		if (!AF._flag_ForceFailed_Safe)
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

	inline void ClockingTimer()
	{
		AF.Update_TimerEnd = micros();
		AF.Update_loopTime = AF.Update_TimerEnd - AF.Update_TimerStart;
		if (AF.Update_loopTime < 0)
			AF.Update_loopTime *= -1;
		delayMicroseconds(AF.Update_Freq_Time - AF.Update_loopTime);
	}

#ifdef USINGJSON
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
		SF._flag_Accel_Pitch_Cali = 0.f;
		SF._flag_Accel__Roll_Cali = 0.f;
		for (int cali_count = 0; cali_count < 2000; cali_count++)
		{
			SensorsDataRead();
			SF._Tmp_IMU_Accel_Vector = sqrt((SF._uORB_MPU9250_A_X * SF._uORB_MPU9250_A_X) + (SF._uORB_MPU9250_A_Y * SF._uORB_MPU9250_A_Y) + (SF._uORB_MPU9250_A_Z * SF._uORB_MPU9250_A_Z));
			if (abs(SF._uORB_MPU9250_A_X) < SF._Tmp_IMU_Accel_Vector)
				SF._uORB_Accel__Roll = asin((float)SF._uORB_MPU9250_A_X / (float)SF._Tmp_IMU_Accel_Vector) * -57.296;
			if (abs(SF._uORB_MPU9250_A_Y) < SF._Tmp_IMU_Accel_Vector)
				SF._uORB_Accel_Pitch = asin((float)SF._uORB_MPU9250_A_Y / (float)SF._Tmp_IMU_Accel_Vector) * 57.296;
			SF._flag_Accel__Roll_Cali += SF._uORB_Accel__Roll;
			SF._flag_Accel_Pitch_Cali += SF._uORB_Accel_Pitch;
			usleep(3);
		}
		SF._flag_Accel__Roll_Cali = (float)SF._flag_Accel__Roll_Cali / (float)2000;
		SF._flag_Accel_Pitch_Cali = (float)SF._flag_Accel_Pitch_Cali / (float)2000;
		std::cout << "AccelPitchCali: " << SF._flag_Accel_Pitch_Cali << " \n";
		std::cout << "AccelRollCali: " << SF._flag_Accel__Roll_Cali << " \n";
		std::cout << "[Sensors] Accel Calibration finsh , input -1 to retry , input 1 to write to configJSON , 0 to skip" << "\n";
		std::cin >> CalibrationComfirm;
		if (CalibrationComfirm == -1)
		{
			SensorsCalibration();
		}
		else if (CalibrationComfirm == 1)
		{
			std::ifstream config(DF.configDir);
			std::string content((std::istreambuf_iterator<char>(config)),
				(std::istreambuf_iterator<char>()));
			nlohmann::json Configdata = nlohmann::json::parse(content);

			Configdata["_flag_Accel__Roll_Cali"] = SF._flag_Accel__Roll_Cali;
			Configdata["_flag_Accel_Pitch_Cali"] = SF._flag_Accel_Pitch_Cali;

			std::ofstream configIN;
			configIN.open(DF.configDir);
			configIN.clear();
			configIN << Configdata.dump(4).c_str();
			configIN.close();

			std::cout << "[Sensors] Config write success\n";
		}
	}
#endif

	inline void RCCalibration()
	{
		int CalibrationComfirm;
		std::cout << "[RCStatus] RC calibration start........." << " \n";
		std::cout << "[RCStatus] RC calibration start, input 1 to start, or input -1 to skip calibration" << " \n";
		std::cin >> CalibrationComfirm;
		if (CalibrationComfirm == -1)
		{
			std::cout << "[RCStatus] Exiting RC calibration ........." << " \n";
			return;
		}
		std::cout << "Max the Throttle , and input 1" << "\n";
		std::cin >> CalibrationComfirm;
		for (size_t i = 0; i < 2000; i++)
		{
			ControlRead();
			RF._flag_RC_Max_PWM_Value = RF._uORB_RC_Channel_PWM[2];
		}

		std::cout << "Mid the Roll , and input 1" << "\n";
		std::cin >> CalibrationComfirm;
		for (size_t i = 0; i < 2000; i++)
		{
			ControlRead();
			RF._flag_RC_Mid_PWM_Value = RF._uORB_RC_Channel_PWM[0];
		}

		std::cout << "Min the Throttle , and input 1" << "\n";
		std::cin >> CalibrationComfirm;
		for (size_t i = 0; i < 2000; i++)
		{
			ControlRead();
			RF._flag_RC_Min_PWM_Value = RF._uORB_RC_Channel_PWM[2];
		}

		std::cout << "please input ARM channel , and turn to on \n";
		for (size_t i = 0; i < 2000; i++)
		{
			ControlRead();
			RF._flag_RC_ARM_PWM_Value = RF._uORB_RC_Channel_PWM[4];
		}

		std::cout << "_flag_RC_Max_PWM_Value: " << RF._flag_RC_Max_PWM_Value << " \n";
		std::cout << "_flag_RC_Mid_PWM_Value: " << RF._flag_RC_Mid_PWM_Value << " \n";
		std::cout << "_flag_RC_Min_PWM_Value: " << RF._flag_RC_Min_PWM_Value << " \n";
		std::cout << "_flag_RC_ARM_PWM_Value: " << RF._flag_RC_ARM_PWM_Value << " \n";

		std::cout << "[RCStatus] RC Calibration finsh , input -1 to retry , input 1 to write to configJSON , 0 to skip" << "\n";
		std::cin >> CalibrationComfirm;
		if (CalibrationComfirm == -1)
		{
			RCCalibration();
		}
		else if (CalibrationComfirm == 1)
		{
			std::ifstream config(DF.configDir);
			std::string content((std::istreambuf_iterator<char>(config)),
				(std::istreambuf_iterator<char>()));
			nlohmann::json Configdata = nlohmann::json::parse(content);

			Configdata["_flag_RC_Max_PWM_Value"] = RF._flag_RC_Max_PWM_Value;
			Configdata["_flag_RC_Mid_PWM_Value"] = RF._flag_RC_Mid_PWM_Value;
			Configdata["_flag_RC_Min_PWM_Value"] = RF._flag_RC_Min_PWM_Value;
			Configdata["_flag_RC_ARM_PWM_Value"] = RF._flag_RC_ARM_PWM_Value;

			std::ofstream configIN;
			configIN.open(DF.configDir);
			configIN.clear();
			configIN << Configdata.dump(4).c_str();
			configIN.close();

			std::cout << "[RCStatus] Config write success\n";
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

protected:
	Sbus* SbusInit;

	struct SafyINFO
	{
		long int RC_Lose_Clocking;
		int Update_Freqeuncy;
		int Update_Freq_Time;
		long int Update_TimerStart;
		long int Update_TimerEnd;
		unsigned int Update_loopTime;

		bool _flag_Error;
		bool _flag_StartUP_Protect;
		bool _flag_first_StartUp;
		bool _flag_RC_Disconnected;
		bool _flag_ForceFailed_Safe;
		bool _flag_Device_setupFailed;
	}AF;

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
		char configDir[20] = "/etc/APMconfig.json";
	}DF;

	struct SensorsINFO
	{
		int MPU9250_Type;
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

		long _flag_MPU9250_G_X_Cali;
		long _flag_MPU9250_G_Y_Cali;
		long _flag_MPU9250_G_Z_Cali;
		double _flag_Accel__Roll_Cali;
		double _flag_Accel_Pitch_Cali;

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
		int RC_Type;
		int _Tmp_RC_Data[36];
		int _uORB_RC_Channel_PWM[16];
		int _flag_RC_Max_PWM_Value;
		int _flag_RC_Mid_PWM_Value;
		int _flag_RC_Min_PWM_Value;
		int _flag_RC_ARM_PWM_Value;

		int _uORB_RC_Out__Roll;
		int _uORB_RC_Out_Pitch;
		int _uORB_RC_Out_Throttle;
		int _uORB_RC_Out___Yaw;
		int _uORB_RC_Out___ARM;
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
		const int _Flag_Lazy_Throttle = 2300;
		const int _Flag_Lock_Throttle = 2200;
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

	inline void ConfigReader(APMSettinngs APMInit)
	{

#ifdef USINGJSON
		std::cout << "[ConfigRead]starting to check out config file ....\n";
		std::ifstream config(DF.configDir);
		std::string content((std::istreambuf_iterator<char>(config)),
			(std::istreambuf_iterator<char>()));
		nlohmann::json Configdata = nlohmann::json::parse(content);
		//==========================================================Device Type=======/
		RF.RC_Type = Configdata["Type_RC"].get<int>();
		SF.MPU9250_Type = Configdata["Type_MPU9250"].get<int>();
		//==========================================================Controller cofig==/
		RF._flag_RC_ARM_PWM_Value = Configdata["_flag_RC_ARM_PWM_Value"].get<int>();
		RF._flag_RC_Min_PWM_Value = Configdata["_flag_RC_Min_PWM_Value"].get<int>();
		RF._flag_RC_Mid_PWM_Value = Configdata["_flag_RC_Mid_PWM_Value"].get<int>();
		RF._flag_RC_Max_PWM_Value = Configdata["_flag_RC_Max_PWM_Value"].get<int>();
		//==========================================================ESC cofig=========/
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
		SF._flag_Accel__Roll_Cali = Configdata["_flag_Accel__Roll_Cali"].get<double>();
		SF._flag_Accel_Pitch_Cali = Configdata["_flag_Accel_Pitch_Cali"].get<double>();
		//===============================================================Update cofig==/
		AF.Update_Freqeuncy = Configdata["Update_Freqeucy"].get<int>();
		AF.Update_Freq_Time = (float)1 / AF.Update_Freqeuncy * 1000000;
		std::cout << "[ConfigRead]Config Set Success!\n";
#else
		SF.MPU9250_Type = APMInit.MPU9250_Type;
		RF.RC_Type = APMInit.RC_Type;

		PF._flag_PID_P__Roll_Gain = APMInit._flag_PID_P__Roll_Gain;
		PF._flag_PID_P_Pitch_Gain = APMInit._flag_PID_P_Pitch_Gain;
		PF._flag_PID_P___Yaw_Gain = APMInit._flag_PID_P___Yaw_Gain;

		PF._flag_PID_I__Roll_Gain = APMInit._flag_PID_I__Roll_Gain;
		PF._flag_PID_I_Pitch_Gain = APMInit._flag_PID_I_Pitch_Gain;
		PF._flag_PID_I___Yaw_Gain = APMInit._flag_PID_I___Yaw_Gain;

		PF._flag_PID_I__Roll_Max__Value = APMInit._flag_PID_I__Roll_Max__Value;
		PF._flag_PID_I_Pitch_Max__Value = APMInit._flag_PID_I_Pitch_Max__Value;
		PF._flag_PID_I___Yaw_Max__Value = APMInit._flag_PID_I___Yaw_Max__Value;

		PF._flag_PID_D__Roll_Gain = APMInit._flag_PID_D__Roll_Gain;
		PF._flag_PID_D_Pitch_Gain = APMInit._flag_PID_D_Pitch_Gain;
		PF._flag_PID_D___Yaw_Gain = APMInit._flag_PID_D___Yaw_Gain;
		PF._flag_PID_Level_Max = APMInit._flag_PID_Level_Max;

		SF._flag_Accel__Roll_Cali = APMInit._flag_Accel__Roll_Cali;
		SF._flag_Accel_Pitch_Cali = APMInit._flag_Accel_Pitch_Cali;

		EF._flag_A1_Pin = APMInit._flag_A1_Pin;
		EF._flag_A2_Pin = APMInit._flag_A2_Pin;
		EF._flag_B1_Pin = APMInit._flag_B1_Pin;
		EF._flag_B2_Pin = APMInit._flag_B2_Pin;
#endif
	}

	inline void ControlRead()
	{
		if (RF.RC_Type == APMS_RCType::RC_IS_SBUS)
		{
			if (SbusInit->SbusRead(RF._Tmp_RC_Data, 0, 1) != -1)
			{
				for (size_t i = 0; i < 16; i++)
				{
					RF._uORB_RC_Channel_PWM[i] = RF._Tmp_RC_Data[i];
				}
				AF._flag_RC_Disconnected = false;
			}
			else
			{
				AF._flag_RC_Disconnected = true;
			}
		}
		else if (RF.RC_Type == APMS_RCType::RC_Is_IBUS)
		{
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
						for (size_t i = 0; i < 16; i++)
						{
							RF._uORB_RC_Channel_PWM[i] = RF._Tmp_RC_Data[i * 2 + 1] * 255 + RF._Tmp_RC_Data[i * 2];
						}
						serialFlush(DF.RCReader_fd);
						AF._flag_RC_Disconnected = false;
					}
					else if (RF._Tmp_RC_Data[31] != 64)
					{
						AF._flag_RC_Disconnected = true;
						serialFlush(DF.RCReader_fd);
					}

					RF._Tmp_RC_Data[31] = 0;
				}
			}
			else
			{
				AF._flag_RC_Disconnected = true;
			}
		}
	}

	inline void SensorsDataRead()
	{
		if (SF.MPU9250_Type == APMS_MPU9250Type::MPU_Is_I2C)
		{
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
		}
		else if (SF.MPU9250_Type == APMS_MPU9250Type::MPU_Is_SPI)
		{
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
		}
	}

	inline void GryoCali()
	{
		SF._flag_MPU9250_G_X_Cali = 0;
		SF._flag_MPU9250_G_Y_Cali = 0;
		SF._flag_MPU9250_G_Z_Cali = 0;
		for (int cali_count = 0; cali_count < 2000; cali_count++)
		{
			SensorsDataRead();
			SF._flag_MPU9250_G_X_Cali += SF._uORB_MPU9250_G_X;
			SF._flag_MPU9250_G_Y_Cali += SF._uORB_MPU9250_G_Y;
			SF._flag_MPU9250_G_Z_Cali += SF._uORB_MPU9250_G_Z;
			usleep(3);
		}
		SF._flag_MPU9250_G_X_Cali = SF._flag_MPU9250_G_X_Cali / 2000;
		SF._flag_MPU9250_G_Y_Cali = SF._flag_MPU9250_G_Y_Cali / 2000;
		SF._flag_MPU9250_G_Z_Cali = SF._flag_MPU9250_G_Z_Cali / 2000;
	}
};
