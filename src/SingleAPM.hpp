#pragma once
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
#include <iomanip>
#include <math.h>
#include <thread>
#include <linux/i2c-dev.h>
#include "../_thirdparty/pca9685.h"
#include "../_thirdparty/Kalman.h"
#include "../_thirdparty/MS5611/src/MS5611LIB.h"
#include "../_thirdparty/Sbus/src/RPiSbus.h"
#include "../_thirdparty/Ibus/src/RPiIBus.h"

#ifdef USINGJSON
#include <nlohmann/json.hpp>
#endif

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

	struct UserControlInputType
	{
		bool _ESC_Self_ARMED;
		float _RawPre___Yaw;
		float _RawPre_Pitch;
		float _RawPre__Roll;
		float _Alttitude_Set;
	};

	class RPiSingleAPM
	{
	public:
		void RPiSingleAPMInit(APMSettinngs APMInit);

		void IMUSensorsTaskReg();

		void AltholdSensorsTaskReg();

		void ControlUserInput(bool EnableUserInput, UserControlInputType UserInput);

		void ControllerTaskReg();

		void ESCUpdateTaskReg();

		void DebugOutPut();

		void TaskThreadBlock();

	protected:
		Sbus *SbusInit;
		Ibus *IbusInit;
		Kalman *Kal_Pitch;
		Kalman *Kal__Roll;
		MS5611 *MS5611S;

		void AttitudeUpdateTask();

		void SaftyCheckTaskReg();

		void PID_Caculate(float inputData, float &outputData,
						  float &last_I_Data, float &last_D_Data,
						  float P_Gain, float I_Gain, float D_Gain, float I_Max);

		void ConfigReader(APMSettinngs APMInit);

		void IMUSensorsDataRead();

		void IMUGryoFilter(long next_input_value, long &next_output_value, long *xv, long *yv, int filtertype);

		void IMUMixFilter(Kalman *kal, float next_input_value_Gryo, float next_input_value_Accel, float next_input_value_speed, float &next_output_value, int filtertype);

		struct SafyINFO
		{
			long int RC_Lose_Clocking;
			bool _flag_IsLockCleanerEnable;
			bool _flag_Error;
			bool _flag_ClockingTime_Error;
			bool _flag_StartUP_Protect;
			bool _flag_MPU9250_first_StartUp;
			bool _flag_MS5611_firstStartUp;
			bool _flag_RC_Disconnected;
			bool _flag_ESC_ARMED;
			bool _flag_Device_setupFailed;
			bool _flag_UserInput_Enable;
			bool _flag_AlthHold_Enable;
		} AF;

		struct DeviceINFO
		{
			int RCReader_fd;
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
			char configDir[20] = "/etc/APMconfig.json";
		} DF;

		struct SensorsINFO
		{
			//=========================MPU9250======//
			int MPU9250_Type;
			int IMUFilter_Type;
			int IMUMixFilter_Type;
			int _Tmp_MPU9250_Buffer[14];
			unsigned char _Tmp_MPU9250_SPI_Config[5];
			unsigned char _Tmp_MPU9250_SPI_Buffer[28];

			long _uORB_MPU9250_A_X;
			long _uORB_MPU9250_A_Y;
			long _uORB_MPU9250_A_Z;
			long _uORB_MPU9250_G_X;
			long _uORB_MPU9250_G_Y;
			long _uORB_MPU9250_G_Z;
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

			float _Tmp_Gryo_RTSpeed__Roll;
			float _Tmp_Gryo_RTSpeed_Pitch;
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

			float _flag_Filter2x50_Gain = 4.840925170e+00;
			//=========================MS5611======//
			double _Tmp_MS5611_Data[2];
			double _uORB_MS5611_Pressure;
			double _uORB_MS5611_AltMeter;
			double _flag_MS5611_LocalPressure = 1023;
			double _uORB_MS5611_Last_Value_AltMeter;
			double _uORB_MS5611_ThrottleFIXUP;
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
			float _uORB_PID_D_Last_Value_Alt = 0;
			float _uORB_PID_I_Last_Value_Alt = 0;
			float _uORB_Leveling_Throttle;
			float _flag_PID_P_Alt_Gain = 5;
			float _flag_PID_I_Alt_Gain = 0.01;
			float _flag_PID_D_Alt_Gain = 4;
			float _flag_PID_Alt_Level_Max;
		} PF;

		struct RCINFO
		{
			int RC_Type;
			int _Tmp_RC_Data[36] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
			int _uORB_RC_Channel_PWM[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
			int _flag_RC_Max_PWM_Value;
			int _flag_RC_Mid_PWM_Value;
			int _flag_RC_Min_PWM_Value;
			int _flag_RC_ARM_PWM_Value;

			int _uORB_RC_Out__Roll;
			int _uORB_RC_Out_Pitch;
			int _uORB_RC_Out_Throttle;
			int _uORB_RC_Out___Yaw;
			int _uORB_RC_Out___ARM;

			int _flag_RCIsReserv__Roll = 1;
			int _flag_RCIsReserv_Pitch = 1;
			int _flag_RCIsReserv___Yaw = 1;

			int _Tmp_UserInput__Roll;
			int _Tmp_UserInput_Pitch;
			int _Tmp_UserInput___Yaw;
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
			std::thread *ALTTask;
		} TF;
	};
} // namespace SingleAPMAPI