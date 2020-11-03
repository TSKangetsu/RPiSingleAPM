#pragma once
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>
#include <math.h>
#include <thread>
#include <string>
#include <fstream>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <thread>
#include <linux/i2c-dev.h>
#include "../_thirdparty/pca9685.h"
#include "../_thirdparty/Kalman.h"
#include "../_thirdparty/MS5611/src/MS5611LIB.h"
#include "../_thirdparty/RCLib/RPiIBus/RPiIBus.hpp"
#include "../_thirdparty/RCLib/RPiSBus/RPiSBus.hpp"
#include "../_thirdparty/RCLib/RPiGPS/RPiGPS.hpp"
#define MPUIsI2c 0
#define MPUIsSpi 1
#define RCIsIbus 0
#define RCIsSbus 1
#define GryoFilterType_none 0
#define GryoFilterType_pt1 1
#define GryoFilterType_Butterworth 2
#define MixFilterType_traditional 0
#define MixFilterType_Kalman 1

namespace SingleAPMAPI
{
	//if you do not use json,you must full all the APMSettings with correct data, like APMconfig.json,and disable 'add_definitions(-DUSINGJSON)' in cmakelists.txt
	struct APMSettinngs
	{
		int RC_Type;
		int MPU9250_Type;
		int IMU_Freqeuncy;
		int RXT_Freqeuncy;
		int ESC_Freqeuncy;
		int IMUFilter_Type;
		int IMUMixFilter_Type;

		float _flag_PID_P__Roll_Gain;
		float _flag_PID_P_Pitch_Gain;
		float _flag_PID_P___Yaw_Gain;
		float _flag_PID_P_Alt_Gain;
		float _flag_PID_I__Roll_Gain;
		float _flag_PID_I_Pitch_Gain;
		float _flag_PID_I___Yaw_Gain;
		float _flag_PID_I_Alt_Gain;
		float _flag_PID_I__Roll_Max__Value;
		float _flag_PID_I_Pitch_Max__Value;
		float _flag_PID_I___Yaw_Max__Value;
		float _flag_PID_D__Roll_Gain;
		float _flag_PID_D_Pitch_Gain;
		float _flag_PID_D___Yaw_Gain;
		float _flag_PID_D_Alt_Gain;
		float _flag_PID_Level_Max;
		float _flag_PID_Hover_Throttle;
		float _flag_PID_Alt_Level_Max;

		double _flag_Accel__Roll_Cali;
		double _flag_Accel_Pitch_Cali;
		double _flag_MPU9250_M_X_Scaler;
		double _flag_MPU9250_M_Y_Scaler;
		double _flag_MPU9250_M_Z_Scaler;

		int _flag_A1_Pin;
		int _flag_A2_Pin;
		int _flag_B1_Pin;
		int _flag_B2_Pin;

		int _flag_RC_ARM_PWM_Value;
		int _flag_RC_Min_PWM_Value;
		int _flag_RC_Mid_PWM_Value;
		int _flag_RC_Max_PWM_Value;

		int _flag_RCIsReserv__Roll;
		int _flag_RCIsReserv_Pitch;
		int _flag_RCIsReserv___Yaw;
	};

	enum APModeINFO
	{
		ManuallHold,
		AltHold,
		AutoStable,
		PositionHold,
	};

	class RPiSingleAPM
	{
	public:
		int RPiSingleAPMInit(APMSettinngs APMInit);

		void IMUSensorsTaskReg();

		void AltholdSensorsTaskReg();

		void ControllerTaskReg();

		void PositionTaskReg();

		void ESCUpdateTaskReg();

		void TaskThreadBlock();

		void APMCalibrator();

	protected:
		Sbus *SbusInit;
		Ibus *IbusInit;
		Kalman *Kal_Pitch;
		Kalman *Kal__Roll;
		MS5611 *MS5611S;
		GPSUart *GPSInit;

		void PID_Caculate(float inputData, float &outputData,
						  float &last_I_Data, float &last_D_Data,
						  float P_Gain, float I_Gain, float D_Gain, float I_Max);

		void PIDSoomth_Caculate(float TargetData, float inputData, float &outputData,
								float &Last_I_Data, float &Total_D_Data, float &Last_D_Data, float (&Ava_D_Data)[30],
								float P_Gain, float I_Gain, float D_Gain, float outputMax, bool StartPIDFlag);

		void PIDINC_Caculate(float TargetData, float inputData, float &outputData,
							 float &LastError, float &PrevError,
							 float P_Gain, float I_Gain, float D_Gain, float outputMax);

		void ConfigReader(APMSettinngs APMInit);

		void IMUSensorsDataRead();

		void IMUGryoFilter(long next_input_value, long &next_output_value, long *xv, long *yv, int filtertype);

		void IMUMixFilter(Kalman *kal, float next_input_value_Gryo, float next_input_value_Accel, float next_input_value_speed, float &next_output_value, int filtertype);

		void AttitudeUpdateTask();

		void SaftyCheckTaskReg();

		void DebugOutPut();

		struct SafyINFO
		{
			APModeINFO AutoPilotMode;
			long int RC_Lose_Clocking;
			bool _flag_Error;
			bool _flag_ClockingTime_Error;
			bool _flag_StartUP_Protect;
			bool _flag_MPU9250_first_StartUp;
			bool _flag_RC_Disconnected;
			bool _flag_ESC_ARMED;
			bool _flag_Device_setupFailed;

			bool _flag_MS5611_Async;
			bool _flag_GPSData_Async;

			bool _flag_IsAltHoldSet;
			bool _flag_IsAltHoldTargetSet;

			bool _flag_IsTakingOff;
			bool _flag_IsTakingAfter;
			bool _flag_IsTakingProtect;
		} AF;

		struct DeviceINFO
		{
			int PCA9658_fd = -1;
			const int PWM_Freq = 400;
			const int PCA9685_PinBase = 65;
			const int PCA9685_Address = 0x40;
			int MPU9250_fd;
			int MPU9250_SPI_Channel = 1;
			const int MPU9250_ADDR = 0x68;
			float _flag_MPU9250_LSB = 65.5;
			int MPU9250_SPI_Freq = 1000000;
			int MS5611_fd;
			const int MS5611_ADDR = 0x77;
			char RCDevice[20] = "/dev/ttyAMA0";
			char GPSDevice[20] = "/dev/ttyAMA1";
		} DF;

		struct SensorsINFO
		{
			//=========================MPU9250======//
			int MPU9250_Type;
			int MPUCompassSupport = 1;
			int IMUFilter_Type;
			int IMUMixFilter_Type;
			int _Tmp_MPU9250_Buffer[14];
			unsigned char _Tmp_MPU9250_SPI_Config[5];
			unsigned char _Tmp_MPU9250_SPI_Buffer[22];
			unsigned char _Tmp_MPU9250_SPI_Compass_Buffer[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
			long _uORB_MPU9250_A_X;
			long _uORB_MPU9250_A_Y;
			long _uORB_MPU9250_A_Z;
			long _uORB_MPU9250_G_X;
			long _uORB_MPU9250_G_Y;
			long _uORB_MPU9250_G_Z;
			float _uORB_MPU9250_M_X;
			float _uORB_MPU9250_M_Y;
			float _uORB_MPU9250_M_Z;
			long _uORB_MPU9250_G_Fixed_X;
			long _uORB_MPU9250_G_Fixed_Y;
			long _uORB_MPU9250_G_Fixed_Z;

			float _uORB_Accel__Roll = 0;
			float _uORB_Accel_Pitch = 0;
			float _uORB_Gryo__Roll = 0;
			float _uORB_Gryo_Pitch = 0;
			float _uORB_Gryo___Yaw = 0;
			float _uORB_Real_Pitch = 0;
			float _uORB_Real__Roll = 0;
			float _uORB_Real___Yaw = 0;
			float _uORB_Real__Head = 0;
			float _uORB_MAG_Heading = 0;

			float _Tmp_Real__Head;
			float _Tmp_Real__Head_Gryo;
			float _Tmp_Real__Head__Mag;
			float _Tmp_MPU9250_M_XH;
			float _Tmp_MPU9250_M_YH;
			float _Tmp_Gryo_RTSpeed__Roll;
			float _Tmp_Gryo_RTSpeed_Pitch;
			float _Tmp_Gryo_RTSpeed___Yaw;
			unsigned long _Tmp_MPU9250_G_X;
			unsigned long _Tmp_MPU9250_G_Y;
			unsigned long _Tmp_MPU9250_G_Z;
			unsigned long _Tmp_MPU9250_A_X;
			unsigned long _Tmp_MPU9250_A_Y;
			unsigned long _Tmp_MPU9250_A_Z;
			unsigned long _Tmp_MPU9250_M_X;
			unsigned long _Tmp_MPU9250_M_Y;
			unsigned long _Tmp_MPU9250_M_Z;

			long _flag_MPU9250_G_X_Cali;
			long _flag_MPU9250_G_Y_Cali;
			long _flag_MPU9250_G_Z_Cali;
			double _flag_Accel__Roll_Cali;
			double _flag_Accel_Pitch_Cali;
			double _flag_MPU9250_M_MRES;
			double _flag_MPU9250_M_X_Cali;
			double _flag_MPU9250_M_Y_Cali;
			double _flag_MPU9250_M_Z_Cali;
			double _flag_MPU9250_M_X_Scaler;
			double _flag_MPU9250_M_Y_Scaler;
			double _flag_MPU9250_M_Z_Scaler;

			long _Tmp_IMU_Accel_Calibration[20];
			long _Tmp_IMU_Accel_Vector;

			long _Tmp_Gryo_filer_Input_Quene_X[3] = {0, 0, 0};
			long _Tmp_Gryo_filer_Output_Quene_X[3] = {0, 0, 0};
			long _Tmp_Gryo_filer_Input_Quene_Y[3] = {0, 0, 0};
			long _Tmp_Gryo_filer_Output_Quene_Y[3] = {0, 0, 0};
			long _Tmp_Gryo_filer_Input_Quene_Z[3] = {0, 0, 0};
			long _Tmp_Gryo_filer_Output_Quene_Z[3] = {0, 0, 0};

			long _Tmp_Acce_filer_Input_Quene_X[5] = {0, 0, 0, 0, 0};
			long _Tmp_Acce_filer_Output_Quene_X[5] = {0, 0, 0, 0, 0};
			long _Tmp_Acce_filer_Input_Quene_Y[5] = {0, 0, 0, 0, 0};
			long _Tmp_Acce_filer_Output_Quene_Y[5] = {0, 0, 0, 0, 0};
			long _Tmp_Acce_filer_Input_Quene_Z[5] = {0, 0, 0, 0, 0};
			long _Tmp_Acce_filer_Output_Quene_Z[5] = {0, 0, 0, 0, 0};

			float _flag_Filter2x50_Gain = 4.979245121e+01;
			//=========================MS5611======//
			int _Tmp_MS5611_AvaClock = 0;
			double _Tmp_MS5611_Data[2];
			double _Tmp_MS5611_AvaTotal = 0;
			double _Tmp_MS5611_AvaData[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
			float _uORB_MS5611_Pressure = 0;
			float _uORB_MS5611_PressureFast = 0;
			float _uORB_MS5611_PressureFill = 0;
			float _uORB_MS5611_PressureDiff = 0;
			float _uORB_MS5611_PressureFinal = 0;
			float _uORB_MS5611_AltMeter = 0;
			float _flag_MS5611_LocalPressure = 1023;
			float _flag_MS5611_FilterAlpha = 0.985;
			//=========================GPS=========//
			GPSUartData _uORB_GPS_Data;
			float _uORB_GPS_Lng_Diff = 0;
			float _uORB_GPS_Lat_Diff = 0;
			int _uORB_GPS_Lat_Smooth = 0;
			int _uORB_GPS_Lng_Smooth = 0;
			float _uOBR_GPS_Lat_Smooth_Diff = 0;
			float _uOBR_GPS_Lng_Smooth_Diff = 0;
			int _uORB_GPS_Lat_Last_Data = 0;
			int _uORB_GPS_Lng_Last_Data = 0;
		} SF;

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
			//===============AltHoldPID=========//
			int _flag_PID_SOOMTH_Clock = 0;
			float _Tmp_PID_D_Alt_Var[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
			float _uORB_PID_AltInput = 0;
			float _uORB_PID_AltHold_Target = 0;
			float _flag_PID_Hover_Throttle = 1500;
			float _uORB_PID_Alt_Throttle = 0;

			float _flag_PID_P_Alt_Gain;
			float _flag_PID_I_Alt_Gain;
			float _flag_PID_D_Alt_Gain;

			float _uORB_PID_I_Last_Value_Alt = 0;
			float _uORB_PID_D_Last_Value_Alt = 0;
			float _uORB_PID_D_Toat_Value_Alt = 0;

			float _flag_PID_Alt_Level_Max;
			//==========PositionHoldPID=========//
			int _uORB_PID_GPS_Lat_Local_Diff = 0;
			int _uORB_PID_GPS_Lng_Local_Diff = 0;
			int _uORB_PID_GPS_Lat_Local_Target = 0;
			int _uORB_PID_GPS_Lng_Local_Target = 0;

			int _uORB_PID_D_GPS_Lat_Ouput;
			int _uORB_PID_D_GPS_Lng_Ouput;
			int _uORB_PID_D_GPS_Lat_LastValue = 0;
			int _uORB_PID_D_GPS_Lng_LastValue = 0;
			int _Tmp_PID_D_GPS_AvaClock = 0;
			int _Tmp_PID_D_GPS_Lat_AvaData[35] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
			int _Tmp_PID_D_GPS_Lng_AvaData[35] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

			int _flag_PID_P_GPS_Gain;
			int _flag_PID_D_GPS_Gain;

			float _uORB_PID_GPS_Lat_Ouput;
			float _uORB_PID_GPS_Lng_Ouput;
		} PF;

		struct RCINFO
		{
			int RC_Type;
			int _Tmp_RC_Data[36] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
			int _uORB_RC_Channel_PWM[16] = {1500, 1500, 1000, 1500, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
			int _flag_RC_Max_PWM_Value;
			int _flag_RC_Mid_PWM_Value;
			int _flag_RC_Min_PWM_Value;
			int _flag_RC_ARM_PWM_Value;

			int _uORB_RC_Out__Roll;
			int _uORB_RC_Out_Pitch;
			int _uORB_RC_Out_Throttle;
			int _uORB_RC_Out___Yaw;
			int _uORB_RC_Out___ARM;
			int _uORB_RC_Out_FlyMod;

			int _flag_RCIsReserv__Roll = 1;
			int _flag_RCIsReserv_Pitch = 1;
			int _flag_RCIsReserv___Yaw = 1;

			int _Tmp_UserInput__Roll;
			int _Tmp_UserInput_Pitch;
			int _Tmp_UserInput___Yaw;

			const int _flag_RC_PWM_Fixed_Min = 1000;
			const int _flag_RC_PWM_Fixed_Mid = 1500;
			const int _flag_RC_PWM_Fixed_Max = 2000;
		} RF;

		struct ESCINFO
		{
			int _uORB_A1_Speed;
			int _uORB_A2_Speed;
			int _uORB_B1_Speed;
			int _uORB_B2_Speed;
			int _Tmp_A1_Speed;
			int _Tmp_A2_Speed;
			int _Tmp_B1_Speed;
			int _Tmp_B2_Speed;
			int _flag_A1_Pin = 0;
			int _flag_A2_Pin = 1;
			int _flag_B1_Pin = 2;
			int _flag_B2_Pin = 3;
			const int _Flag_Lazy_Throttle = 2300;
			const int _Flag_Lock_Throttle = 2200;
			const int _Flag_Max__Throttle = 3000;
		} EF;

		struct TaskThread
		{
			int _Tmp_IMUThreadTimeStart;
			int _Tmp_IMUThreadTimeEnd;
			int _Tmp_IMUThreadTimeNext;
			int _Tmp_IMUThreadTimeLoop;
			int _Tmp_IMUThreadError = 0;
			int _flag_IMUThreadTimeMax;
			int _flag_IMUThreadFreq;
			std::thread *IMUTask;
			int _Tmp_RXTThreadTimeStart;
			int _Tmp_RXTThreadTimeEnd;
			int _Tmp_RXTThreadTimeNext;
			int _Tmp_RXTThreadTimeLoop;
			int _Tmp_RXTThreadError = 0;
			int _flag_RXTThreadTimeMax;
			int _flag_RXTThreadFreq;
			std::thread *RXTask;
			int _Tmp_ESCThreadTimeStart;
			int _Tmp_ESCThreadTimeEnd;
			int _Tmp_ESCThreadTimeNext;
			int _Tmp_ESCThreadTimeLoop;
			int _Tmp_ESCThreadError = 0;
			int _flag_ESCThreadTimeMax;
			int _flag_ESCThreadFreq;
			std::thread *ESCTask;
			int _Tmp_ALTThreadTimeStart;
			int _Tmp_ALTThreadTimeEnd;
			int _Tmp_ALTThreadTimeNext;
			int _Tmp_ALTThreadTimeLoop;
			int _Tmp_ALTThreadError = 0;
			std::thread *ALTTask;
			int _Tmp_GPSThreadSMooth = 0;
			int _Tmp_GPSThreadTimeStart;
			int _Tmp_GPSThreadTimeEnd;
			int _Tmp_GPSThreadTimeNext;
			int _Tmp_GPSThreadTimeLoop;
			int _Tmp_GPSThreadError = 0;
			int _flag_GPSThreadTimeMax = (float)1 / 50 * 1000000;
			std::thread *GPSTask;
		} TF;
	};
} // namespace SingleAPMAPI