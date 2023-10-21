#pragma once
#include <mutex>
#include <math.h>
#include <thread>
#include <string>
#include <unistd.h>
#include <iomanip>
#include <math.h>
#include <thread>
#include <bitset>
#include <csignal>
#include <iostream>
#include <linux/i2c-dev.h>
#include "_thirdparty/EKFImpement.hpp"
#include "_thirdparty/ESCGenerator.hpp"
#include "_thirdparty/FlowController.hpp"
#include "_thirdparty/RaspberryPiBARO/src/BaroDevice.hpp"
#include "_thirdparty/RaspberryPiRC/RPiGPS/RPiGPS.hpp"
#include "_thirdparty/RaspberryPiRC/RPiIBus/RPiIBus.hpp"
#include "_thirdparty/RaspberryPiRC/RPiSBus/RPiSBus.hpp"
#include "_thirdparty/RaspberryPiRC/CRSF/CRSFUartRC.hpp"
#include "_thirdparty/RaspberryPiRC/CRSF/CRSFProtocol.hpp"
#include "_thirdparty/RaspberryPiRC/RPiFlow/RPiFlow.hpp"
#include "_thirdparty/RaspberryPiMPU/src/MPU9250/filter.h"
#include "_thirdparty/RaspberryPiMPU/src/_thirdparty/libeigen/Eigen/LU"
#include "_thirdparty/RaspberryPiMPU/src/_thirdparty/libeigen/Eigen/Dense"
#include "_thirdparty/PowerMonitor/ADS111x.hpp"
#include "_thirdparty/RaspberryPiMPU/src/MPU9250/MPU9250.hpp"
#include "_thirdparty/BlackboxEncoder/Blackbox.hpp"

#define PI 3.1415926
#define SpeedUnusableRES 1.5
#define ACC_CLIPPING_RC_CONSTANT 0.10f

#define RCIsIbus 0
#define RCIsSbus 1
#define RCIsCRSF 2

#define ESCCalibration 10
#define CaliESCStart 0
#define CaliESCUserDefine 2
#define ACCELCalibration 11
#define COMPASSCalibration 12

#define FILTERBAROLPFCUTOFF 1.f
#define FILTERTHROTTLELPFCUTOFF 4.f
#define FILTERPOSOUTLPFCUTOFF 4.f
#define FILTERMAGCUTOFF 5.f

#define ESCRANGE 800.f

#define AngleLimitTime 30000000.f
#define CalibratorLimitTime 10000000.f
#define NAVIGATION_HZ 250
#define ACCEL_UPDATE_HZ 1000
#define PID_DT_DEFAULT 250.f

#define I2CCOMPASS_ADDR 0x1e
#define I2CBARO_ADDR 0x77
#define I2CPCA_ADDR 0x70
#define I2CADS111x_ADDR 0x49

#define ADC_FRONT_GAIN 0.10138
#define ADC_VBAT_PIN 5

#define BlackBoxIInterval 32
#define BlackBoxFirmware "Cleanflight"
#define BlackBoxLogDir "/var/log/Singleflight/"

namespace SingleAPMAPI
{
	inline volatile std::sig_atomic_t SystemSignal;

	struct APMSettinngs
	{
		struct DeviceConfig
		{
			int RC_Type;
			int MPU9250_Type;
			int IMU_Freqeuncy;
			int RXT_Freqeuncy;
			int ESC_Freqeuncy;
			int BBC_Freqeuncy;
			std::string BBC_PInterval;

			std::string __RCDevice;
			std::string __GPSDevice;
			std::string __FlowDevice;
			std::string __MPUDeviceSPI;
			std::string __I2CDevice;

			bool _IsGPSEnable;
			bool _IsFlowEnable;
			bool _IsRCSafeEnable;
			bool _IsBAROEnable;
			bool _IsBlackBoxEnable;
		} DC;

		struct PIDConfig
		{
			float _flag_PID_P__Roll_Gain;
			float _flag_PID_P_Pitch_Gain;
			float _flag_PID_P___Yaw_Gain;
			float _flag_PID_P_Alt_Gain;
			float _flag_PID_P_PosX_Gain;
			float _flag_PID_P_PosY_Gain;
			float _flag_PID_P_SpeedZ_Gain;
			float _flag_PID_P_SpeedX_Gain;
			float _flag_PID_P_SpeedY_Gain;
			float _flag_PID_I_Alt_Gain;
			float _flag_PID_I_PosX_Gain;
			float _flag_PID_I_PosY_Gain;
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
			float _flag_PID_Rate_Limit;
			float _flag_PID_Alt_Level_Max;
			float _flag_PID_Pos_Level_Max;

			float _flag_PID_AngleRate__Roll_Gain;
			float _flag_PID_AngleRate_Pitch_Gain;
			float _flag_PID_AngleRate___Yaw_Gain;
			float _flag_PID_RCRate__Roll_Gain;
			float _flag_PID_RCRate_Pitch_Gain;
			float _flag_PID_RCRate___Yaw_Gain;
			float _flag_PID_RCAngle__Roll_Gain;
			float _flag_PID_RCAngle_Pitch_Gain;
			float _flag_PID_RCAngle___Yaw_Gain;

			float _flag_PID_Takeoff_Altitude;
			float _flag_PID_Alt_Speed_Max;
			float _flag_PID_Alt_Accel_Max;
			float _flag_PID_PosMan_Speed_Max;
			float _flag_PID_Pos_Speed_Max;

			float _flag_PID_TPA_Trust;
			float _flag_PID_TPA_BreakPoint;
		} PC;

		struct SensorConfig
		{
			int _flag_MPU_Flip__Roll;
			int _flag_MPU_Flip_Pitch;
			int _flag_MPU_Flip___Yaw;
			double _flag_Accel__Roll_Cali;
			double _flag_Accel_Pitch_Cali;
			double _flag_MPU9250_A_X_Cali;
			double _flag_MPU9250_A_Y_Cali;
			double _flag_MPU9250_A_Z_Cali;
			double _flag_MPU9250_A_X_Scal;
			double _flag_MPU9250_A_Y_Scal;
			double _flag_MPU9250_A_Z_Scal;
			double _flag_COMPASS_X_Offset;
			double _flag_COMPASS_X_Scaler;
			double _flag_COMPASS_Y_Offset;
			double _flag_COMPASS_Y_Scaler;
			double _flag_COMPASS_Z_Offset;
			double _flag_COMPASS_Z_Scaler;
			double _flag_COMPASS_V_Offset;
			double _flag_COMPASS_V_Scaler;
			double _flag_COMPASS_Flip__Roll;
			double _flag_COMPASS_Flip_Pitch;
			double _flag_COMPASS_Flip___Yaw;
			double _flag_COMPASS_YAW_Offset;
		} SC;

		struct OutputConfig
		{
			int _flag_A1_Pin;
			int _flag_A2_Pin;
			int _flag_B1_Pin;
			int _flag_B2_Pin;
			float _flag_YAWOut_Reverse;
			float _flag_ESC_Lazy_Per;

			int ESCPLFrequency;
			int ESCControllerType;
		} OC;

		struct RCConfig
		{
			int _flag_RC_Min_PWM_Value;
			int _flag_RC_Mid_PWM_Value;
			int _flag_RC_Max_PWM_Value;

			int _flag_RC_ARM_PWM_Value;
			int _flag_RC_ARM_PWM_Channel;
			int _flag_RC_AP_RateHold_PWM_Value;
			int _flag_RC_AP_RateHold_PWM_Channel;
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
		} RC;

		struct FilterConfig
		{
			int _flag_Filter_Gryo_Type;
			int _flag_Filter_GryoST2_Type;
			int _flag_Filter_GYaw_CutOff;
			int _flag_Filter_Gryo_CutOff;
			int _flag_Filter_GryoST2_CutOff;
			int _flag_Filter_Gryo_NotchFreq;
			int _flag_Filter_Gryo_NotchCutOff;
			int _flag_Filter_Gryo_DynamicNotchRange;
			int _flag_Filter_Gryo_DynamicNotchMinFreq;
			bool _flag_Filter_Gryo_DynamicNotchEnable;

			int _flag_Filter_Accel_Type;
			int _flag_Filter_Accel_CutOff;
			double _flag_Filter_AngleMix_Alpha;

			float _flag_Baro_Trust_Beta;
			float _flag_Accel_Trust_Beta;
			float _flag_Sonar_Trust_Beta;
			float _flag_GPSAlt_Trust_Beta;
			float _flag_AccelBias_Trust_Beta;

			float _flag_GPS_Config_Beta = 1.f;
			float _flag_Flow_Config_Beta = 1.f;
			float _flag_Braking_Speed_Gain = 2.0f;

			float _flag_Filter_RC_CutOff;
			float _flag_Filter_AngleRate_CutOff;

			float _flag_Filter_PID_I_CutOff;
			float _flag_Filter_PID_D_ST1_CutOff;
			float _flag_Filter_PID_D_ST2_CutOff;
		} FC;
	};

	enum APModeINFO
	{
		// AutoPilot Bank0
		RateHold,
		// AutoPilot Bank1
		AutoStable,
		AltHold,
		// AutoPilot Bank2
		PositionHold,
		SpeedHold,
		// AutoPilot Bank3
		UserAuto
	};

	class RPiSingleAPM
	{
	public:
		int RPiSingleAPMInit(APMSettinngs APMInit);

		void RPiSingleAPMStartUp();

		void RPiSingleAPMHotLoad(APMSettinngs APMInit) { ConfigReader(APMInit); };

		void RPiSingleAPMDeInit();

		int APMCalibrator(int controller, int action, int input, double *data);

		~RPiSingleAPM() { RPiSingleAPMDeInit(); };

		void TaskThreadBlock();

		//=========APMUserControllerFunction===========//
		void APMControllerFakeRC(int *ChannelData, bool IsError);

		void APMControllerARMED();

		void APMControllerDISARM(APModeINFO APMode);

		void APMControllerServo(int pin, int PWMInUs);

		void APMControllerPosition(int x, int y, int z, bool resetHome);

		void APMControllerSpeed(int x, int y, int z);

	protected:
		enum FailedSafeFlag
		{
			_flag_FailedSafe_RCLose = 1 << 0,
			_flag_FailedSafe_FakeRCLose = 1 << 1,
			_flag_FailedSafe_AngleLimit = 1 << 2,
			_flag_FailedSafe_MPUNoRespond = 1 << 3,
			_flag_FailedSafe_ESCNoRespond = 1 << 4,
			_flag_FailedSafe_SpeedReferenceZ = 1 << 8,
			_flag_FailedSafe_SpeedReferenceZT = 1 << 9,
			_flag_FailedSafe_SpeedReferenceX = 1 << 10,
			_flag_FailedSafe_SpeedReferenceXT = 1 << 11,
			_flag_FailedSafe_SpeedReferenceY = 1 << 12,
			_flag_FailedSafe_SpeedReferenceYT = 1 << 13,

			_flag_PreARMFailed_GyroNotStable = 1 << 0,
			_flag_PreARMFailed_AngleNotSync = 1 << 1,
			_flag_PreARMFailed_NavigationNotSync = 1 << 2,
			_flag_PreARMFailed_AccelNotStable = 1 << 3,
		};

		struct SafyINFO
		{
			APModeINFO AutoPilotMode;
			uint16_t _flag_FailedSafe_Level = 0;
			uint16_t _flag_PreARM_Check_Level = 0;
			bool _flag_Error = false;
			bool _flag_RC_Error = false;
			bool _flag_FakeRC_Error = false;
			bool _flag_GPS_Error = false;
			bool _flag_AnagleOutOfLimit = false;

			bool _flag_PreARM_Check = false;
			bool _flag_PreARM_Check_Lock = false;
			bool _flag_MPUCalibrating = false;
			bool _flag_MPUCalibratingSet = false;

			bool _flag_IsAutoTakeoffLock = false;
			bool _flag_IsAutoTakeoffRequire = false;

			bool _flag_ESC_ARMED = false;
			bool _flag_ESC_DISARMED_Request = false;
			bool _flag_StartUP_Protect = false;

			bool _flag_Device_setupFailed = false;
			bool _flag_MPU9250_first_StartUp = false;

			bool _flag_FakeRC_Deprive = false;
			bool _flag_RC_Disconnected = false;
			bool _flag_FakeRC_Disconnected = false;
			bool _flag_GPS_Disconnected = false;
			bool _flag_GPS_Recovered = false;
			long int RC_Lose_Clocking = 0;
			long int FakeRC_Lose_Clocking = 0;
			long int GPS_Lose_Clocking = 0;
			long int Flow_Lose_Clocking = 0;
			long int FakeRC_Deprive_Clocking = 0;
			long int AngleLimit_Out_Clocking = 0;

			bool _flag_BARO_Async = false;
			bool _flag_GPSData_Async = false;
			bool _flag_GPSData_AsyncB = false;
			bool _flag_FlowData_Async = false;
			bool _flag_SonarData_Async = false;
			bool _flag_IsSonarAvalible = false;
			bool _flag_IsFlowAvalible = false;
			bool _flag_IsFakeRCUpdated = false;
			bool _flag_IsNAVAvalible = false;

			bool _flag_IsNotTakeOff = false;
			bool _flag_IsNotTakeOff_Lock = false;

			bool _flag_IsINUHDisable = false;
			bool _flag_IsPositionXChange = false;
			bool _flag_IsPositionYChange = false;
			bool _flag_IsBrakingXSet = false;
			bool _flag_IsBrakingYSet = false;
			bool _flag_IsBrakingXBlock = false;
			bool _flag_IsBrakingYBlock = false;

			bool _flag_MAG_Cali_Failed = false;
		} AF;

		struct DeviceINFO
		{
			// if APM Init before started , it will be -1
			// if APM Device InitComplete , it will be 1
			// if APM Thread all complete , it will be 2
			// if APM is stop and DeInit  , it will be -2
			int APMStatus = -1;
			const int MPU9250_ADDR = 0x68;
			std::string RCDevice;
			std::string GPSDevice;
			std::string FlowDevice;
			std::string MPUDeviceSPI;
			std::string I2CDevice;

			bool _IsGPSEnable;
			bool _IsFlowEnable;
			bool _IsRCSafeEnable;
			bool _IsBAROEnable;
			bool _IsBlackBoxEnable;
			bool _IsADCEnable; // FIXME: decide by Failed or not;

			std::mutex I2CLock;
			std::unique_ptr<Sbus> SbusInit;
			std::unique_ptr<Ibus> IbusInit;
			std::unique_ptr<CRSF> CRSFInit;
			std::unique_ptr<BaroDevice> BaroDeviceD;
			std::unique_ptr<GPSUart> GPSInit;
			std::unique_ptr<ESCGenerator> ESCDevice;
			std::unique_ptr<GPSI2CCompass> CompassDevice;
			std::unique_ptr<RPiMPU9250> MPUDevice;
			std::unique_ptr<MSPUartFlow> FlowInit;
			std::unique_ptr<BlackboxEncoder> BlackBoxDevice;
			// std::unique_ptr<PowerADC> ADCDevice;
			std::unique_ptr<ADS111x> ADCDevice;
			std::ofstream BlackBoxFile;
			TotalEKF EKFDevice;

			pt1Filter_t BAROLPF;
			pt1Filter_t ThrottleLPF;
			pt1Filter_t POSOutLPF[2];
			pt1Filter_t GPSSpeedLPF[2];
			pt1Filter_t AngleRateLPF[3];
			pt1Filter_t ItermFilterPitch;
			pt1Filter_t ItermFilterRoll;
			pt1Filter_t DtermFilterPitch;
			pt1Filter_t DtermFilterRoll;
			pt1Filter_t DtermFilterPitchST2;
			pt1Filter_t DtermFilterRollST2;
			pt1Filter_t RCLPF[3];
			biquadFilter_t MAGFilter[3];
		} DF;

		struct SensorsINFO
		{
			//=========================MPU=======//
			int MPU9250_Type;
			MPUData _uORB_MPU_Data;
			int _flag_MPU_Flip__Roll;
			int _flag_MPU_Flip_Pitch;
			int _flag_MPU_Flip___Yaw;
			int _flag_Filter_Gryo_Type;
			int _flag_Filter_GryoST2_Type;
			int _flag_Filter_GYaw_CutOff;
			int _flag_Filter_Gryo_CutOff;
			int _flag_Filter_GryoST2_CutOff;
			int _flag_Filter_Gryo_NotchFreq;
			int _flag_Filter_Gryo_NotchCutOff;
			int _flag_Filter_Gryo_DynamicNotchRange;
			int _flag_Filter_Gryo_DynamicNotchMinFreq;
			bool _flag_Filter_Gryo_DynamicNotchEnable;
			int _flag_Filter_Accel_Type;
			int _flag_Filter_Accel_CutOff;
			double _flag_Filter_AngleMix_Alpha;
			double _flag_MPU_Accel_Cali[20];
			int _flag_MPUCalibratorWaitClock = 0;
			double _uORB_True_Speed_X = 0;
			double _uORB_True_Speed_Y = 0;
			double _uORB_True_Speed_Z = 0;
			double _uORB_True_Movement_X = 0;
			double _uORB_True_Movement_Y = 0;
			double _uORB_True_Movement_Z = 0;
			int _uORB_Accel_Clipped_Count = 0;
			//=========================Baro=====//
			BaroData _uORB_BARO_Data;
			float _uORB_BARO_Altitude;
			//=========================GPS=========//
			GPSUartData _uORB_GPS_Data;
			int _uORB_GPS_COR_Lat = 0;
			int _uORB_GPS_COR_Lng = 0;
			int _uORB_GPS_Hold_Lat = 0;
			int _uORB_GPS_Hold_Lng = 0;
			int _uORB_GPS_Home_Lat = 0;
			int _uORB_GPS_Home_Lng = 0;
			int _Tmp_GPS_Last_Lat = 0;
			int _Tmp_GPS_Last_Lng = 0;
			int _uORB_GPS_Real_X = 0;
			int _uORB_GPS_Real_Y = 0;
			int _uORB_GPS_Speed_X = 0;
			int _uORB_GPS_Speed_Y = 0;
			float _uORB_GPS_Speed_XF = 0;
			float _uORB_GPS_Speed_YF = 0;
			int _uORB_GPS_COR_NES = 0;
			//=========================MAG=========//
			double _uORB_MAG_Yaw = 0;
			double _uORB_MAG_StaticYaw = 0;
			double _uORB_NAV_Yaw = 0;
			int _uORB_MAG_RawX = 0;
			int _uORB_MAG_RawY = 0;
			int _uORB_MAG_RawZ = 0;
			double _uORB_MAG_Vector = 0;
			double _flag_COMPASS_YAW_Offset;
			double _flag_COMPASS_Cali[10];
			double _flag_COMPASS_Flip__Roll;
			double _flag_COMPASS_Flip_Pitch;
			double _flag_COMPASS_Flip___Yaw;
			//========================Flow=========//
			int _Tmp_Flow___Status = 0;
			int _uORB_Flow_XOutput = 0;
			int _uORB_Flow_YOutput = 0;
			int _uORB_Flow_Quality = 0;
			int _uORB_RF_Quality = 0;
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

			int _Tmp_FlowThreadTimeout = 0;
			//========================Extend=======//
			int _uORB_BAT_Scount = 0;
			float _uORB_BAT_Voltage = 0;
			float _uORB_BAT_SingleVol = 0;
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

			float _flag_PID_P__Roll_Gain = 0;
			float _flag_PID_P_Pitch_Gain = 0;
			float _flag_PID_P___Yaw_Gain = 0;

			float _flag_PID_I__Roll_Gain = 0;
			float _flag_PID_I_Pitch_Gain = 0;
			float _flag_PID_I___Yaw_Gain = 0;
			float _flag_PID_I__Roll_Max__Value = 0;
			float _flag_PID_I_Pitch_Max__Value = 0;
			float _flag_PID_I___Yaw_Max__Value = 0;
			float _uORB_PID_I_Dynamic_Gain = 1.f;

			float _flag_PID_D__Roll_Gain = 0;
			float _flag_PID_D_Pitch_Gain = 0;
			float _flag_PID_D___Yaw_Gain = 0;

			float _uORB_PID_GYaw_Output;
			float _uORB_PID_AngleRate_Pitch;
			float _uORB_PID_AngleRate__Roll;

			float _flag_PID_Level_Max = 0;
			float _flag_PID_Rate_Limit = 500.f;

			float _flag_Filter_AngleRate_CutOff = 255;
			float _flag_Filter_PID_I_CutOff = 30.f;
			float _flag_Filter_PID_D_ST1_CutOff = 100.f;
			float _flag_Filter_PID_D_ST2_CutOff = 200.f;

			float _flag_PID_RCRate__Roll_Gain = 1.5;
			float _flag_PID_RCRate_Pitch_Gain = 1.5;
			float _flag_PID_RCRate___Yaw_Gain = 1.5;
			float _flag_PID_RCAngle__Roll_Gain = 0.6;
			float _flag_PID_RCAngle_Pitch_Gain = 0.6;
			float _flag_PID_RCAngle___Yaw_Gain = 0.6;
			float _flag_PID_AngleRate__Roll_Gain = 5.f;
			float _flag_PID_AngleRate_Pitch_Gain = 5.f;
			float _flag_PID_AngleRate___Yaw_Gain = 5.f;

			float _uORB_PID_TPA_Beta = 1.f;
			float _flag_PID_TPA_Trust = 1.f;
			float _flag_PID_TPA_BreakPoint = 2000;
			//===============AltHoldPID=========//
			// Target Atitude
			float _uORB_PID_Sonar_AltInput = 0;
			int _uORB_PID_Sonar_GroundTimeOut = 0;
			bool _uORB_PID_Sonar_GroundValid = false;
			float _uORB_PID_Sonar_GroundOffset = 0;
			float _uORB_PID_BARO_AltInput = 0;
			float _uORB_PID_MoveZCorrection = 0;
			float _uORB_PID_SpeedZCorrection = 0;
			float _uORB_PID_AltInput_Last_Final = 0;
			float _uORB_PID_AltHold_Target = 0;
			float _uORB_PID_PosZUserSpeed = 0;
			// Target Speed
			float _uORB_PID_InputTarget = 0;
			float _uORB_PID_Smooth_InputTarget = 0;
			float _uORB_PID_SpeedZ_Final = 0;
			float _uORB_PID_AccelZ_Bias = 0;
			// Target Output
			float _uORB_PID_Alt_Throttle = 0;
			// AltHold Gain
			float _uORB_AccelBias_Beta = 0.02f;
			float _uORB_Baro_Dynamic_Beta = 0.35f;
			float _uORB_Sonar_Dynamic_Beta = 0.8f;
			float _uORB_GPSAlt_Dynamic_Beta = 0.15f;
			float _uORB_Accel_Dynamic_Beta = 1.f;

			float _flag_Accel_Config_Beta = 1.f;
			float _flag_Baro_Config_Beta = 0.35f;
			float _flag_Sonar_Config_Beta = 0.8f;
			//
			float _flag_PID_Alt_Speed_Max = 50;
			float _flag_PID_Alt_Accel_Max = 500;
			float _uORB_PID_Alt_Speed_Max = 50;
			float _uORB_PID_Alt_Accel_Max = 500;
			float _flag_PID_P_TAsix_Gain = 0;
			float _flag_PID_Alt_Level_Max = 400;
			float _flag_PID_Hover_Throttle = 1300;

			float _flag_PID_TakeOff_Speed_Max = 50.f;
			float _flag_PID_TakeOff_Accel_Max = 150.f;
			float _flag_PID_Takeoff_Altitude = 50.f;
			//
			float _flag_PID_P_Alt_Gain = 0;
			float _flag_PID_I_Alt_Gain = 0;
			float _flag_PID_P_SpeedZ_Gain = 0;
			float _flag_PID_I_SpeedZ_Gain = 0;
			float _flag_PID_D_SpeedZ_Gain = 0;
			float _uORB_PID_I_Last_Value_SpeedZ = 0;
			float _uORB_PID_D_Last_Value_SpeedZ = 0;
			//==========PositionHoldPID=========//
			float _uORB_PID_Flow_PosInput_X = 0;
			float _uORB_PID_Flow_PosInput_Y = 0;
			float _uORB_PID_GPS_PosInput_X = 0;
			float _uORB_PID_GPS_PosInput_Y = 0;

			float _uORB_PID_PosXTarget = 0;
			float _uORB_PID_PosYTarget = 0;
			float _uORB_PID_AccelX_Bias = 0;
			float _uORB_PID_AccelY_Bias = 0;
			float _uORB_PID_MoveXCorrection = 0;
			float _uORB_PID_SpeedXCorrection = 0;
			float _uORB_PID_MoveYCorrection = 0;
			float _uORB_PID_SpeedYCorrection = 0;

			float _uORB_PID_PosXUserSpeed = 0;
			float _uORB_PID_PosYUserSpeed = 0;
			float _uORB_PID_PosXUserTarget = 0;
			float _uORB_PID_PosYUserTarget = 0;

			float _uORB_PID_PosX_Output = 0;
			float _uORB_PID_PosY_Output = 0;

			float _flag_GPS_Dynamic_Beta = .8f;
			float _flag_Flow_Dynamic_Beta = 1.f;
			float _flag_Braking_Speed_Gain = 2.0f;

			float _flag_PID_SpeedX_Max = 0;
			float _flag_PID_AccelX_Max = 0;
			float _flag_PID_P_PosX_Gain = 0;
			float _flag_PID_I_PosX_Gain = 0;
			float _flag_PID_P_SpeedX_Gain = 0;
			float _flag_PID_I_SpeedX_Gain = 0;
			float _flag_PID_D_SpeedX_Gain = 0;

			float _flag_PID_SpeedY_Max = 0;
			float _flag_PID_AccelY_Max = 0;
			float _flag_PID_P_PosY_Gain = 0;
			float _flag_PID_I_PosY_Gain = 0;
			float _flag_PID_P_SpeedY_Gain = 0;
			float _flag_PID_I_SpeedY_Gain = 0;
			float _flag_PID_D_SpeedY_Gain = 0;

			float _flag_PID_PosMan_Speed_Max = 50;
			float _flag_PID_Pos_Speed_Max = 50;
			float _flag_PID_Pos_Level_Max = 250;

			float _uORB_PID_I_Last_Value_SpeedX = 0;
			float _uORB_PID_D_Last_Value_SpeedX = 0;
			float _uORB_PID_I_Last_Value_SpeedY = 0;
			float _uORB_PID_D_Last_Value_SpeedY = 0;
		} PF;

		struct RCINFO
		{
			int RC_Type;
			int _Tmp_RC_Data[36] = {0};
			float _flag_Filter_RC_CutOff;
			int _uORB_RC_Channel_PWM[16] = {1500, 1500, 1500, 1500, 2000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
			int _uORB_FakeRC_Channel_PWM[16] = {1500, 1500, 1500, 1500, 2000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
			int _uORB_ModeCopy_Channel_PWM[16] = {1500, 1500, 1500, 1500, 2000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
			int _flag_RC_Max_PWM_Value = 2000;
			int _flag_RC_Mid_PWM_Value = 1500;
			int _flag_RC_Min_PWM_Value = 1000;

			int _Tmp_RC_Out__Roll = 0;
			int _Tmp_RC_Out_Pitch = 0;
			int _Tmp_RC_Out_Throttle = 0;
			int _Tmp_RC_Out___Yaw = 0;

			int _uORB_RC_Out__Roll = 0;
			int _uORB_RC_Out_Pitch = 0;
			int _uORB_RC_Out_Throttle = 0;
			int _uORB_RC_Out___Yaw = 0;

			float _uORB_RC_D__Roll_Gain = 0.f;
			float _uORB_RC_D_Pitch_Gain = 0.f;
			float _uORB_RC_D___Yaw_Gain = 0.f;

			int _uORB_RC_Out_AltHoldSpeed = 0;
			int _uORB_RC_Out_PosHoldSpeedX = 0;
			int _uORB_RC_Out_PosHoldSpeedY = 0;

			int _flag_RC_ARM_PWM_Value;
			int _flag_RC_ARM_PWM_Channel;
			int _flag_RC_AP_RateHold_PWM_Value;
			int _flag_RC_AP_RateHold_PWM_Channel;
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
			int ESCPLFrequency = 1526;
			GeneratorType ESCControllerType = GeneratorType::Hardware_ONESHOT125;

			int _uORB_A1_Speed = 0;
			int _uORB_A2_Speed = 0;
			int _uORB_B1_Speed = 0;
			int _uORB_B2_Speed = 0;
			int _Tmp_A1_Speed = 0;
			int _Tmp_A2_Speed = 0;
			int _Tmp_B1_Speed = 0;
			int _Tmp_B2_Speed = 0;
			int _flag_A1_Pin = 0;
			int _flag_A2_Pin = 1;
			int _flag_B1_Pin = 2;
			int _flag_B2_Pin = 3;
			int _Flag_Lazy_Throttle = 1050;
			float _flag_ESC_Lazy_Per = 0.06f;
			const int _Flag_Lock_Throttle = 1000;
			const int _Flag_Max__Throttle = 2000;
			double _uORB_Total_Throttle = 0.0;
			float _flag_YAWOut_Reverse = 1.f;

			int _uORB_ESC_RPY_Max = 0;
			int _uORB_ESC_RPY_Min = 0;
			int _uORB_ESC_RPY_Range = 0;

			int _uORB_Dynamic_ThrottleMin = 1000;
			int _uORB_Dynamic_ThrottleMax = 2000;

			float _uORB_ESC_MIX_Range = 0;
		} EF;

		struct TaskThread
		{
			const int _flag_Sys_CPU_Asign = 3;

			float _flag_IMUFlowFreq = 1000.f;
			float _flag_RTXFlowFreq = 250.f;
			float _flag_TELFlowFreq = 15.f;
			float _flag_ESCFlowFreq = 1000.f;
			const float _flag_ALTFlowFreq = 45.f;
			const float _flag_GPSFlowFreq = 5.f;
			const float _flag_MAGFlowFreq = 200.f;
			const float _flag_OPFFlowFreq = 28.f;
			const float _flag_EXTFlowFreq = 15.f;

			std::unique_ptr<FlowThread> IMUFlow;
			std::unique_ptr<FlowThread> RTXFlow;
			std::unique_ptr<FlowThread> TELFlow;
			std::unique_ptr<FlowThread> RXCFlow;
			std::unique_ptr<FlowThread> ESCFlow;
			std::unique_ptr<FlowThread> ALTFlow;
			std::unique_ptr<FlowThread> GPSFlow;
			std::unique_ptr<FlowThread> MAGFlow;
			std::unique_ptr<FlowThread> OPFFlow;
			std::unique_ptr<FlowThread> EXTFlow;
			std::unique_ptr<FlowThread> BBQFlow;

			int _flag_SystemStartUp_Time = 0;

			float _Tmp_IMUNavThreadDT = 0;
			float _Tmp_IMUNavThreadLast = 0;
			float _Tmp_IMUAttThreadDT = 0;
			float _Tmp_IMUAttThreadLast = 0;
			// Blackbox
			int _Tmp_BBQThreadTimeStart = 0;
			int _Tmp_BBQThreadTimeEnd = 0;
			int _Tmp_BBQThreadTimeNext = 0;
			int _Tmp_BBQThreadTimeLoop = 0;
			int _flag_BBQThreadFreq;
			std::string _flag_BBQThreadINT;
			int _flag_P_Interval = 1;
			int _flag_BBQThreadTimeMax = 0;
			bool _flag_BBQ_Task_Running = false;
			std::thread BlackBoxQTask;
			int _Tmp_BBQThreadTimeup = 0;
			int _Tmp_BBQThreadloopIteration = 0;
			bool _flag_BBW_Task_Running = false;
			std::thread BlackBoxWTask;
			std::queue<std::vector<uint8_t>> BlackBoxQeueue;
			//
			int DEBUGOuputCleaner = 0;
			bool _flag_Block_Task_Running = false;
		} TF;

	private:
		void AttitudeUpdate();

		void NavigationUpdate();

		void SaftyCheck();

		void DebugOutPut();

		void IMUSensorsTaskReg();

		void AltholdSensorsTaskReg();

		void ControllerTaskReg();

		void PositionTaskReg();

		void ExtendMonitorTaskReg();

		void ESCUpdateTaskReg();

		void BlackBoxTaskReg();

		void PID_Caculate(float inputData, float &outputData,
						  float &last_I_Data, float &last_D_Data,
						  float P_Gain, float I_Gain, float D_Gain, float I_Max);

		void PID_CaculateExtend(float inputDataP, float inputDataI, float inputDataD, float &outputData,
								float &last_I_Data, float &last_D_Data,
								float P_Gain, float I_Gain, float D_Gain, float I_Max);

		void PID_CaculateHyper(float inputDataP, float inputDataI, float inputDataD, float &outputData,
							   float &last_I_Data, float &last_D_Data,
							   float P_Gain, float I_Gain, float D_Gain, float I_Max);

		void ConfigReader(APMSettinngs APMInit);

		int GetTimestamp();
	};
} // namespace SingleAPMAPI