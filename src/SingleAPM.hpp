#pragma once
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>
#include <math.h>
#include <thread>
#include <string>
#include <unistd.h>
#include <iomanip>
#include <math.h>
#include <thread>
#include <iostream>
#include <linux/i2c-dev.h>
#include "_thirdparty/pca9685.h"
#include "_thirdparty/EKFImpement.hpp"
#include "_thirdparty/RPiMS5611LIB/src/MS5611LIB.h"
#include "_thirdparty/RaspberryPiRC/RPiGPS/RPiGPS.hpp"
#include "_thirdparty/RaspberryPiRC/RPiIBus/RPiIBus.hpp"
#include "_thirdparty/RaspberryPiRC/RPiSBus/RPiSBus.hpp"
#include "_thirdparty/RaspberryPiRC/RPiFlow/RPiFlow.hpp"
#include "_thirdparty/RaspberryPiMPU/src/MPU9250/MPU9250.hpp"

#define RCIsIbus 0
#define RCIsSbus 1

#define ESCCalibration 10
#define CaliESCStart 0
#define CaliESCUserDefine 2
#define ACCELCalibration 11
#define COMPASSCalibration 12;

#define USERDEBUGINPUTSIZE 5

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
		std::string __FlowDevice;

		bool _IsGPSEnable;
		bool _IsFlowEnable;
		bool _IsRCSafeEnable;
		bool _IsMS5611Enable;

		float _flag_PID_P__Roll_Gain;
		float _flag_PID_P_Pitch_Gain;
		float _flag_PID_P___Yaw_Gain;
		float _flag_PID_P_Alt_Gain;
		float _flag_PID_P_PosX_Gain;
		float _flag_PID_P_PosY_Gain;
		float _flag_PID_P_SpeedZ_Gain;
		float _flag_PID_P_SpeedX_Gain;
		float _flag_PID_P_SpeedY_Gain;
		float _flag_PID_I__Roll_Gain;
		float _flag_PID_I_Pitch_Gain;
		float _flag_PID_I___Yaw_Gain;
		float _flag_PID_I_SpeedZ_Gain;
		float _flag_PID_I_SpeedX_Gain;
		float _flag_PID_I_SpeedY_Gain;
		float _flag_PID_I__Roll_Max__Value;
		float _flag_PID_I_Pitch_Max__Value;
		float _flag_PID_I___Yaw_Max__Value;
		float _flag_PID_D__Roll_Gain;
		float _flag_PID_D_Pitch_Gain;
		float _flag_PID_D___Yaw_Gain;
		float _flag_PID_D_SpeedZ_Gain;
		float _flag_PID_D_SpeedX_Gain;
		float _flag_PID_D_SpeedY_Gain;
		float _flag_PID_Hover_Throttle;

		float _flag_PID_Level_Max;
		float _flag_PID_Alt_Level_Max;
		float _flag_PID_Pos_Level_Max;

		float _flag_PID_Takeoff_Altitude;
		float _flag_PID_Alt_Speed_Max;
		float _flag_PID_PosMan_Speed_Max;
		float _flag_PID_Pos_Speed_Max;

		double _flag_Accel__Roll_Cali;
		double _flag_Accel_Pitch_Cali;
		double _flag_MPU9250_A_X_Cali;
		double _flag_MPU9250_A_Y_Cali;
		double _flag_MPU9250_A_Z_Cali;
		double _flag_MPU9250_A_X_Scal;
		double _flag_MPU9250_A_Y_Scal;
		double _flag_MPU9250_A_Z_Scal;
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
		float _flag_YAWOut_Reverse;

		int _flag_RC_Min_PWM_Value;
		int _flag_RC_Mid_PWM_Value;
		int _flag_RC_Max_PWM_Value;

		int _flag_RC_ARM_PWM_Value;
		int _flag_RC_ARM_PWM_Channel;
		int _flag_RC_AP_ManualHold_PWM_Value;
		int _flag_RC_AP_ManualHold_PWM_Channel;
		int _flag_RC_AP_AutoStable_PWM_Value;
		int _flag_RC_AP_AutoStable_PWM_Channel;
		int _flag_RC_AP_AltHold_PWM_Value;
		int _flag_RC_AP_AltHold_PWM_Channel;
		int _flag_RC_AP_PositionHold_PWM_Value;
		int _flag_RC_AP_PositionHold_PWM_Channel;
		int _flag_RC_AP_SpeedHold_PWM_Value;
		int _flag_RC_AP_SpeedHold_PWM_Channel;
		int _flag_RC_AP_UserAuto_PWM_Value;
		int _flag_RC_AP_UserAuto_PWM_Channel;

		int _flag_RCIsReserv__Roll;
		int _flag_RCIsReserv_Pitch;
		int _flag_RCIsReserv___Yaw;
	};

	enum APModeINFO
	{
		//AutoPilot Bank0
		ManualHold,
		//AutoPilot Bank1
		AutoStable,
		AltHold,
		//AutoPilot Bank2
		PositionHold,
		SpeedHold,
		//AutoPilot Bank3
		UserAuto
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

		int APMCalibrator(int controller, int action, int input, double *data);

		//=========APMUserControllerFunction===========//
		void APMControllerARMED();

		void APMControllerDISARM(APModeINFO APMode);

		void APMControllerServo(int pin, int on, int off);

		void APMControllerPosition(int x, int y, int z, bool resetHome);

		void APMControllerSpeed(int x, int y, int z);

	protected:
		Sbus *SbusInit;
		Ibus *IbusInit;
		MS5611 *MS5611S;
		GPSUart *GPSInit;
		RPiMPU9250 *MPUDevice;
		MSPUartFlow *FlowInit;
		int DEBUGOuputCleaner = 0;
		GPSI2CCompass_QMC5883L *GPSMAGInit;
		TotalEKF EKFDevice;

		void PID_Caculate(float inputData, float &outputData,
						  float &last_I_Data, float &last_D_Data,
						  float P_Gain, float I_Gain, float D_Gain, float I_Max);

		void PID_CaculateExtend(float inputDataP, float inputDataI, float inputDataD, float &outputData,
								float &last_I_Data, float &last_D_Data,
								float P_Gain, float I_Gain, float D_Gain, float I_Max);

		void ConfigReader(APMSettinngs APMInit);

		void AttitudeUpdateTask();

		void SaftyCheckTaskReg();

		void DebugOutPut();

		struct SafyINFO
		{
			APModeINFO AutoPilotMode;
			bool _flag_IsAutoTakeoffLock;
			bool _flag_IsAutoTakeoffRequire;

			bool _flag_Error;
			bool _flag_GPS_Error;
			bool _flag_ESC_ARMED;
			bool _flag_ESC_DISARMED_Request;
			bool _flag_StartUP_Protect;
			bool _flag_ClockingTime_Error;

			bool _flag_Device_setupFailed;
			bool _flag_MPU9250_first_StartUp;

			bool _flag_RC_Disconnected;
			bool _flag_GPS_Disconnected;
			long int RC_Lose_Clocking;
			long int GPS_Lose_Clocking;
			long int Flow_Lose_Clocking;

			bool _flag_MS5611_Async;
			bool _flag_GPSData_Async;
			bool _flag_FlowData_Async;
			bool _flag_SonarData_Async;
			bool _flag_IsGPSHoldSet;
			bool _flag_IsSonarAvalible;
			bool _flag_IsFlowAvalible;
		} AF;

		struct DeviceINFO
		{
			int PCA9658_fd = -1;
			const int PWM_Freq = 400;
			const int PCA9685_PinBase = 65;
			const int PCA9685_Address = 0x40;
			int MPU9250_SPI_Channel = 1;
			const int MPU9250_ADDR = 0x68;
			std::string RCDevice;
			std::string GPSDevice;
			std::string FlowDevice;

			bool _IsGPSEnable;
			bool _IsFlowEnable;
			bool _IsRCSafeEnable;
			bool _IsMS5611Enable;
		} DF;

		struct SensorsINFO
		{
			//=========================MPU9250======//
			int MPU9250_Type;
			int IMUFilter_Type;
			int IMUMixFilter_Type;
			MPUData _uORB_MPU_Data;
			double _flag_MPU_Accel_Cali[20];
			double _uORB_MPU_Speed_X = 0;
			double _uORB_MPU_Speed_Y = 0;
			double _uORB_MPU_Speed_Z = 0;
			double _uORB_MPU_Movement_X = 0;
			double _uORB_MPU_Movement_Y = 0;
			double _uORB_MPU_Movement_Z = 0;
			//=========================MS5611======//
			int _Tmp_MS5611_Error = 0;
			double _Tmp_MS5611_Data[10] = {1000, 1000, 1000, 0};
			double _Tmp_MS5611_Pressure = 0;
			double _Tmp_MS5611_PressureFast = 0;
			double _Tmp_MS5611_PressureFill = 0;
			double _uORB_MS5611_PressureFinal = 0;
			int _uORB_MS5611_Altitude = 0;
			int _uORB_MS5611_Last_Altitude = 0;
			double _uORB_MS5611_ClimbeRate = 0;
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
			//========================Flow=========//
			int _Tmp_Flow___Status = 0;

			int _uORB_Flow_XOutput = 0;
			int _uORB_Flow_YOutput = 0;
			int _uORB_Flow_Quality = 0;
			double _uORB_Gryo_Body_Asix_X = 0;
			double _uORB_Gryo_Body_Asix_Y = 0;
			double _uORB_Flow_Body_Asix_X = 0;
			double _uORB_Flow_Body_Asix_Y = 0;
			double _uORB_Flow_Filter_XOutput = 0;
			double _uORB_Flow_Filter_YOutput = 0;
			double _uORB_Flow_Speed_X = 0;
			double _uORB_Flow_Speed_Y = 0;
			float _uORB_Flow_XOutput_Total = 0;
			float _uORB_Flow_YOutput_Total = 0;

			int _Tmp_Flow_Altitude = 0;
			double _uORB_Flow_Altitude = 0;
			double _uORB_Flow_Altitude_Final = 0;
			double _uORB_Flow_Altitude_Last_Final = 0;
			double _uORB_Flow_ClimbeRate = 0;
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
			//Target Atitude
			float _uORB_PID_Sonar_AltInput = 0;
			float _uORB_PID_Sonar_GroundOffset = 0;
			float _uORB_PID_MS5611_AltInput = 0;
			float _uORB_PID_AltInput_Final = 0;
			float _uORB_PID_AltInput_Last_Final = 0;
			float _uORB_PID_AltHold_Target = 0;
			float _uORB_PID_PosZUserSpeed = 0;
			//Target Speed
			float _uORB_PID_InputTarget = 0;
			float _uORB_PID_Smooth_InputTarget = 0;
			float _uORB_PID_SpeedZ_Final = 0;
			//Target Output
			float _uORB_PID_Alt_Throttle = 0;
			//AltHold Gain
			double _flag_Alt_Dynamic_Beta = 0.995;
			double _flag_SpeedZ_Dynamic_Beta = 0.999;
			float _flag_PID_Alt_Speed_Max = 0;
			float _flag_PID_Takeoff_Altitude = 0;
			float _flag_PID_P_TAsix_Gain = 0;
			float _flag_PID_Alt_Level_Max;
			float _flag_PID_Hover_Throttle;
			//
			float _flag_PID_P_Alt_Gain;
			float _flag_PID_P_SpeedZ_Gain;
			float _flag_PID_I_SpeedZ_Gain;
			float _flag_PID_D_SpeedZ_Gain;
			float _uORB_PID_I_Last_Value_SpeedZ = 0;
			float _uORB_PID_D_Last_Value_SpeedZ = 0;
			//==========PositionHoldPID=========//
			float _uORB_PID_Flow_PosInput_X = 0;
			float _uORB_PID_Flow_PosInput_Y = 0;
			float _uORB_PID_Smooth_PosXTarget = 0;
			float _uORB_PID_Smooth_PosYTarget = 0;

			float _uORB_PID_PosXTarget = 0;
			float _uORB_PID_PosYTarget = 0;

			float _uORB_PID_PosXUserSpeed = 0;
			float _uORB_PID_PosYUserSpeed = 0;
			float _uORB_PID_PosXUserTarget = 0;
			float _uORB_PID_PosYUserTarget = 0;

			float _uORB_PID_PosX_Output = 0;
			float _uORB_PID_PosY_Output = 0;

			float _flag_PID_P_PosX_Gain = 0;
			float _flag_PID_P_SpeedX_Gain = 0;
			float _flag_PID_I_SpeedX_Gain = 0;
			float _flag_PID_D_SpeedX_Gain = 0;

			float _flag_PID_P_PosY_Gain = 0;
			float _flag_PID_P_SpeedY_Gain = 0;
			float _flag_PID_I_SpeedY_Gain = 0;
			float _flag_PID_D_SpeedY_Gain = 0;

			float _flag_PID_PosMan_Speed_Max = 0;
			float _flag_PID_Pos_Speed_Max = 0;
			float _flag_PID_Pos_Level_Max;

			float _uORB_PID_I_Last_Value_SpeedX = 0;
			float _uORB_PID_D_Last_Value_SpeedX = 0;
			float _uORB_PID_I_Last_Value_SpeedY = 0;
			float _uORB_PID_D_Last_Value_SpeedY = 0;
		} PF;

		struct RCINFO
		{
			int RC_Type;
			int _Tmp_RC_Data[36] = {0};
			int _uORB_RC_Channel_PWM[16] = {1500, 1500, 1500, 1500, 2000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
			int _flag_RC_Max_PWM_Value;
			int _flag_RC_Mid_PWM_Value;
			int _flag_RC_Min_PWM_Value;

			int _uORB_RC_Out__Roll;
			int _uORB_RC_Out_Pitch;
			int _uORB_RC_Out_Throttle;
			int _uORB_RC_Out___Yaw;

			int _uORB_RC_Out_AltHoldSpeed = 0;
			int _uORB_RC_Out_PosHoldSpeedX = 0;
			int _uORB_RC_Out_PosHoldSpeedY = 0;

			int _flag_RC_ARM_PWM_Value;
			int _flag_RC_ARM_PWM_Channel;
			int _flag_RC_AP_ManualHold_PWM_Value;
			int _flag_RC_AP_ManualHold_PWM_Channel;
			int _flag_RC_AP_AutoStable_PWM_Value;
			int _flag_RC_AP_AutoStable_PWM_Channel;
			int _flag_RC_AP_AltHold_PWM_Value;
			int _flag_RC_AP_AltHold_PWM_Channel;
			int _flag_RC_AP_PositionHold_PWM_Value;
			int _flag_RC_AP_PositionHold_PWM_Channel;
			int _flag_RC_AP_SpeedHold_PWM_Value;
			int _flag_RC_AP_SpeedHold_PWM_Channel;
			int _flag_RC_AP_UserAuto_PWM_Value;
			int _flag_RC_AP_UserAuto_PWM_Channel;

			int _flag_RCIsReserv__Roll = 1;
			int _flag_RCIsReserv_Pitch = 1;
			int _flag_RCIsReserv___Yaw = 1;

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
			float _flag_YAWOut_Reverse = 1.f;

			int _uORB_Servo1_On_Output = 0;
			int _uORB_Servo1_Off_Output = 0;
			int _uORB_Servo2_On_Output = 0;
			int _uORB_Servo2_Off_Output = 0;
			int _uORB_Servo3_On_Output = 0;
			int _uORB_Servo3_Off_Output = 0;
			int _uORB_Servo4_On_Output = 0;
			int _uORB_Servo4_Off_Output = 0;
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
			int _Tmp_SerThreadClock = 0;
			int _flag_SerThreadFreq = 50;
			int _flag_SerThreadTimes = 0;
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
			int _Tmp_FlowThreadSMooth = 0;
			int _Tmp_FlowThreadTimeStart = 0;
			int _Tmp_FlowThreadTimeEnd = 0;
			int _Tmp_FlowThreadTimeNext = 0;
			int _Tmp_FlowThreadTimeLoop = 0;
			int _Tmp_FlowThreadError = 0;
			int _flag_FlowThreadTimeMax = (float)1 / 28 * 1000000; //Flow is 9HZ
			int _flag_FlowErrorTimes = 0;
			std::thread *FlowTask;
			std::thread LEDSignalTask;
		} TF;
	};
} // namespace SingleAPMAPI