#pragma once
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>
#include <math.h>
#include <thread>
#include <string>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <thread>
#include <linux/i2c-dev.h>
#include "_thirdparty/pca9685.h"
#include "_thirdparty/Kalman.h"
#include "_thirdparty/RPiMS5611LIB/src/MS5611LIB.h"
#include "_thirdparty/RaspberryPiRC/RPiIBus/RPiIBus.hpp"
#include "_thirdparty/RaspberryPiRC/RPiSBus/RPiSBus.hpp"
#include "_thirdparty/RaspberryPiRC/RPiGPS/RPiGPS.hpp"
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
	struct APMSettinngs
	{
		int RC_Type;
		int MPU9250_Type;
		int IMU_Freqeuncy;
		int RXT_Freqeuncy;
		int ESC_Freqeuncy;
		int IMUFilter_Type;
		int IMUMixFilter_Type;

		std::string __RCDevice;
		std::string __GPSDevice;

		bool _IsGPSEnable;
		bool _IsFlowEnable;
		bool _IsSonarEnable;
		bool _IsRCSafeEnable;
		bool _IsMS5611Enable;

		float _flag_PID_P_TAsix_Gain;
		float _flag_PID_P__Roll_Gain;
		float _flag_PID_P_Pitch_Gain;
		float _flag_PID_P___Yaw_Gain;
		float _flag_PID_P_Alt_Gain;
		float _flag_PID_P_GPS_Gain;
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
		float _flag_PID_D_GPS_Gain;
		float _flag_PID_Level_Max;
		float _flag_PID_Hover_Throttle;
		float _flag_PID_Alt_Level_Max;
		float _flag_PID_GPS_Level_Max;

		double _flag_Accel__Roll_Cali;
		double _flag_Accel_Pitch_Cali;
		double _flag_MPU9250_A_X_Cali;
		double _flag_MPU9250_A_Y_Cali;
		double _flag_MPU9250_A_Z_Cali;
		double _flag_MPU9250_M_X_Offset;
		double _flag_MPU9250_M_Y_Offset;
		double _flag_MPU9250_M_Z_Offset;
		double _flag_MPU9250_M_Y_Scaler;
		double _flag_MPU9250_M_Z_Scaler;
		double _flag_MPU9250_Head_Asix;

		double _flag_QMC5883L_Head_Asix;
		double _flag_QMC5883L_M_X_Offset;
		double _flag_QMC5883L_M_Y_Offset;
		double _flag_QMC5883L_M_Z_Offset;
		double _flag_QMC5883L_M_Y_Scaler;
		double _flag_QMC5883L_M_Z_Scaler;

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

		void APMCalibrator(int Type, double *data);

	protected:
		Sbus *SbusInit;
		Ibus *IbusInit;
		Kalman *Kal_Pitch;
		Kalman *Kal__Roll;
		MS5611 *MS5611S;
		GPSUart *GPSInit;
		GPSI2CCompass_QMC5883L *GPSMAGInit;

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

			bool _flag_Error;
			bool _flag_GPS_Error;
			bool _flag_ESC_ARMED;
			bool _flag_StartUP_Protect;
			bool _flag_ClockingTime_Error;

			bool _flag_Device_setupFailed;
			bool _flag_MPU9250_first_StartUp;

			bool _flag_RC_Disconnected;
			bool _flag_GPS_Disconnected;
			long int RC_Lose_Clocking;
			long int GPS_Lose_Clocking;

			bool _flag_MS5611_Async;
			bool _flag_GPSData_Async;

			bool _flag_IsAltHoldSet;
			bool _flag_IsGPSHoldSet;
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
			int MPU9250_SPI_Freq = 10000000;
			int MS5611_fd;
			const int MS5611_ADDR = 0x77;
			std::string RCDevice;
			std::string GPSDevice;

			bool _IsGPSEnable;
			bool _IsFlowEnable;
			bool _IsSonarEnable;
			bool _IsRCSafeEnable;
			bool _IsMS5611Enable;
		} DF;

		struct SensorsINFO
		{
			//=========================MPU9250======//
			int MPU9250_Type;
			int IMUFilter_Type;
			int IMUMixFilter_Type;
			int _Tmp_MPU9250_Buffer[14];
			unsigned char _flag_MPU9250_CompassConfig[39][4] = {
				{0x37, 0x30}, // INT_PIN_CFG
				{0x24, 0x5D}, // I2C_MST_CTRL
				{0x6A, 0x20}, // USER_CTRL
				{0x25, 0x0C | 0x80},
				{0x26, 0x00},
				{0x27, 0x81},
				{0x49 | 0x80, 0x00}, // Read the WHO_AM_I byte,line is 6
				{0x25, 0x0C},
				{0x26, 0x0B},
				{0x63, 0x01}, // Reset AK8963
				{0x27, 0x81},
				{0x25, 0x0C},
				{0x26, 0x0A},
				{0x63, 0x00}, // Power down magnetometer
				{0x27, 0x81},
				{0x25, 0x0C},
				{0x26, 0x0A},
				{0x63, 0x0F}, // Enter fuze mode
				{0x27, 0x81},
				{0x25, 0x0C | 0x80},
				{0x26, 0x10},
				{0x27, 0x83},
				{0x49 | 0x80}, // Read the x-, y-, and z-axis calibration values ,line is 22
				{0x25, 0x0C},
				{0x26, 0x0A},
				{0x63, 0x00}, // Power down magnetometer
				{0x27, 0x81},
				{0x25, 0x0C},
				{0x26, 0x0A},
				{0x63, 1 << 4 | 0x06}, // Set magnetometer data resolution and sample ODR
				{0x27, 0x81},
				{0x25, 0x0C | 0x80},
				{0x26, 0x0A},
				{0x27, 0x81},
			};
			unsigned char _Tmp_MPU9250_SPI_Config[10];
			unsigned char _Tmp_MPU9250_SPI_Buffer[22];
			unsigned char _Tmp_MPU9250_SPI_Compass_Buffer[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
			long _uORB_MPU9250_A_X = 0;
			long _uORB_MPU9250_A_Y = 0;
			long _uORB_MPU9250_A_Z = 0;
			long _uORB_MPU9250_G_X = 0;
			long _uORB_MPU9250_G_Y = 0;
			long _uORB_MPU9250_G_Z = 0;
			float _uORB_MPU9250_M_X = 0;
			float _uORB_MPU9250_M_Y = 0;
			float _uORB_MPU9250_M_Z = 0;
			long _uORB_MPU9250_G_Fixed_X = 0;
			long _uORB_MPU9250_G_Fixed_Y = 0;
			long _uORB_MPU9250_G_Fixed_Z = 0;

			float _uORB_Accel__Roll = 0;
			float _uORB_Accel_Pitch = 0;
			float _uORB_Gryo__Roll = 0;
			float _uORB_Gryo_Pitch = 0;
			float _uORB_Gryo___Yaw = 0;
			float _uORB_Real__Roll = 0;
			float _uORB_Real_Pitch = 0;

			float _Tmp_Real_Pitch = 0;
			float _Tmp_Real__Roll = 0;
			float _Tmp_Gryo_RTSpeed__Roll = 0;
			float _Tmp_Gryo_RTSpeed_Pitch = 0;
			float _uORB_Gryo_RTSpeed___Yaw = 0;
			unsigned long _Tmp_MPU9250_G_X = 0;
			unsigned long _Tmp_MPU9250_G_Y = 0;
			unsigned long _Tmp_MPU9250_G_Z = 0;
			unsigned long _Tmp_MPU9250_A_X = 0;
			unsigned long _Tmp_MPU9250_A_Y = 0;
			unsigned long _Tmp_MPU9250_A_Z = 0;
			unsigned long _Tmp_MPU9250_M_X = 0;
			unsigned long _Tmp_MPU9250_M_Y = 0;
			unsigned long _Tmp_MPU9250_M_Z = 0;

			float _uORB_MPU9250___Yaw = 0;
			float _uORB_MPU9250__Head = 0;
			float _Tmp_MPU9250___MAG = 0;
			float _Tmp_MPU9250__Head = 0;
			float _Tmp_MPU9250_M_XH = 0;
			float _Tmp_MPU9250_M_YH = 0;
			float _Tmp_MPU9250__Head_Gryo = 0;
			float _Tmp_MPU9250__Head__Mag = 0;

			long _flag_MPU9250_G_X_Cali;
			long _flag_MPU9250_G_Y_Cali;
			long _flag_MPU9250_G_Z_Cali;
			double _flag_MPU9250_A_X_Cali;
			double _flag_MPU9250_A_Y_Cali;
			double _flag_MPU9250_A_Z_Cali;
			double _flag_Accel__Roll_Cali;
			double _flag_Accel_Pitch_Cali;
			double _flag_MPU9250_M_MRES;
			double _flag_MPU9250_M_X_Cali;
			double _flag_MPU9250_M_Y_Cali;
			double _flag_MPU9250_M_Z_Cali;
			double _flag_MPU9250_M_X_Offset;
			double _flag_MPU9250_M_Y_Offset;
			double _flag_MPU9250_M_Z_Offset;
			double _flag_MPU9250_M_Y_Scaler;
			double _flag_MPU9250_M_Z_Scaler;
			double _flag_MPU9250_Head_Asix;

			long _Tmp_IMU_Accel_Vector;
			long _Tmp_IMU_Accel_Calibration[20];

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

			float _Tmp_MS5611_Pressure = 0;
			float _Tmp_MS5611_PressureFast = 0;
			float _Tmp_MS5611_PressureFill = 0;
			float _Tmp_MS5611_PressureDiff = 0;

			double _Tmp_MS5611_Data[2];
			double _Tmp_MS5611_AvaTotal = 0;
			double _Tmp_MS5611_AvaData[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

			float _uORB_MS5611_AltMeter = 0;
			float _uORB_MS5611_PressureFinal = 0;
			float _flag_MS5611_FilterAlpha = 0.985;
			//=========================GPS=========//
			float _Tmp_QMC5883L_M_XH = 0;
			float _Tmp_QMC5883L_M_YH = 0;
			float _Tmp_QMC5883L___MAG = 0;
			float _Tmp_QMC5883L__Head = 0;
			float _Tmp_QMC5883L__Head_Gryo = 0;
			float _Tmp_QMC5883L__Head__Mag = 0;

			long _uORB_QMC5883L_M_X = 0;
			long _uORB_QMC5883L_M_Y = 0;
			long _uORB_QMC5883L_M_Z = 0;
			float _uORB_QMC5883L__Yaw = 0;
			float _uORB_QMC5883L_Head = 0;

			double _flag_QMC5883L_Head_Asix;
			double _flag_QMC5883L_M_X_Offset;
			double _flag_QMC5883L_M_Y_Offset;
			double _flag_QMC5883L_M_Z_Offset;
			double _flag_QMC5883L_M_Y_Scaler;
			double _flag_QMC5883L_M_Z_Scaler;

			GPSUartData _uORB_GPS_Data;

			int _Tmp_GPS_Lat_Last_Data = 0;
			int _Tmp_GPS_Lng_Last_Data = 0;
			float _Tmp_GPS_Lng_Diff = 0;
			float _Tmp_GPS_Lat_Diff = 0;
			float _Tmp_GPS_Lat_Smooth_Diff = 0;
			float _Tmp_GPS_Lng_Smooth_Diff = 0;

			int _uORB_GPS_Lat_Smooth = 0;
			int _uORB_GPS_Lng_Smooth = 0;
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

			float _uORB_Leveling__Roll = 0;
			float _uORB_Leveling_Pitch = 0;
			float _uORB_Leveling___Yaw = 0;

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
			float _uORB_PID_Alt_Throttle = 0;
			float _uORB_PID_AltHold_Target = 0;

			float _uORB_PID_I_Last_Value_Alt = 0;
			float _uORB_PID_D_Last_Value_Alt = 0;
			float _uORB_PID_D_Toat_Value_Alt = 0;

			float _flag_PID_P_Alt_Gain;
			float _flag_PID_I_Alt_Gain;
			float _flag_PID_D_Alt_Gain;
			float _flag_PID_Alt_Level_Max;
			float _flag_PID_Hover_Throttle;

			float _flag_PID_P_TAsix_Gain;
			unsigned int _uORB_PID_TAsix_Ouput = 0;
			//==========PositionHoldPID=========//
			int _Tmp_PID_D_GPS_AvaClock = 0;
			int _Tmp_PID_D_GPS_Lat_AvaData[35] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
			int _Tmp_PID_D_GPS_Lng_AvaData[35] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

			int _Tmp_PID_D_GPS_Lat_Ouput = 0;
			int _Tmp_PID_D_GPS_Lng_Ouput = 0;

			int _uORB_PID_GPS_Lat_Local_Diff = 0;
			int _uORB_PID_GPS_Lng_Local_Diff = 0;
			int _uORB_PID_D_GPS_Lat_LastValue = 0;
			int _uORB_PID_D_GPS_Lng_LastValue = 0;
			int _uORB_PID_GPS_Lat_Local_Target = 0;
			int _uORB_PID_GPS_Lng_Local_Target = 0;
			float _uORB_PID_GPS_Lat_Ouput = 0;
			float _uORB_PID_GPS_Lng_Ouput = 0;
			float _uORB_PID_GPS_Pitch_Ouput = 0;
			float _uORB_PID_GPS__Roll_Ouput = 0;

			float _flag_PID_P_GPS_Gain;
			float _flag_PID_D_GPS_Gain;
			float _flag_PID_GPS_Level_Max;
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
			int _Tmp_IMUThreadTimeStart = 0;
			int _Tmp_IMUThreadTimeEnd = 0;
			int _Tmp_IMUThreadTimeNext = 0;
			int _Tmp_IMUThreadTimeLoop = 0;
			int _Tmp_IMUThreadError = 0;
			int _flag_IMUThreadTimeMax = 0;
			int _flag_IMUThreadFreq;
			int _flag_IMUErrorTimes = 0;
			std::thread *IMUTask;
			int _Tmp_RXTThreadTimeStart = 0;
			int _Tmp_RXTThreadTimeEnd = 0;
			int _Tmp_RXTThreadTimeNext = 0;
			int _Tmp_RXTThreadTimeLoop = 0;
			int _Tmp_RXTThreadError = 0;
			int _flag_RXTThreadTimeMax = 0;
			int _flag_RXTThreadFreq;
			int _flag_RXTErrorTimes = 0;
			std::thread *RXTask;
			int _Tmp_ESCThreadTimeStart = 0;
			int _Tmp_ESCThreadTimeEnd = 0;
			int _Tmp_ESCThreadTimeNext = 0;
			int _Tmp_ESCThreadTimeLoop = 0;
			int _Tmp_ESCThreadError = 0;
			int _flag_ESCThreadTimeMax = 0;
			int _flag_ESCThreadFreq;
			int _flag_ESCErrorTimes = 0;
			std::thread *ESCTask;
			int _Tmp_ALTThreadTimeStart = 0;
			int _Tmp_ALTThreadTimeEnd = 0;
			int _Tmp_ALTThreadTimeNext = 0;
			int _Tmp_ALTThreadTimeLoop = 0;
			int _Tmp_ALTThreadError = 0;
			int _flag_ALTThreadTimeMax = (float)1 / 85 * 1000000;
			int _flag_ALTErrorTimes = 0;
			std::thread *ALTTask;
			int _Tmp_GPSThreadSMooth = 0;
			int _Tmp_GPSThreadTimeStart = 0;
			int _Tmp_GPSThreadTimeEnd = 0;
			int _Tmp_GPSThreadTimeNext = 0;
			int _Tmp_GPSThreadTimeLoop = 0;
			int _Tmp_GPSThreadError = 0;
			int _flag_GPSThreadTimeMax = (float)1 / 50 * 1000000;
			int _flag_GPSErrorTimes = 0;
			std::thread *GPSTask;
			int _Tmp_MAGThreadSMooth = 0;
			int _Tmp_MAGThreadTimeStart = 0;
			int _Tmp_MAGThreadTimeEnd = 0;
			int _Tmp_MAGThreadTimeNext = 0;
			int _Tmp_MAGThreadTimeLoop = 0;
			int _Tmp_MAGThreadError = 0;
			int _flag_MAGThreadTimeMax = (float)1 / 200 * 1000000;
			int _flag_MAGErrorTimes = 0;
			std::thread *MAGTask;
		} TF;
	};
} // namespace SingleAPMAPI