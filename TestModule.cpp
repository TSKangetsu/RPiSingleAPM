#include "./src/SingleAPM.hpp"
#include <fstream>
#include "Drive_Json.hpp"
using namespace SingleAPMAPI;

#define CONFIGDIR "/boot/APMConfig.json"

void configWrite(const char *configDir, const char *Target, double obj);
void configSettle(const char *configDir, const char *substr, APMSettinngs &APMInit);
void SignalCatch(int Signal);

int main(int argc, char *argv[])
{
	system("clear");
	int argvs;
	double data[10];
	APMSettinngs setting;
	while ((argvs = getopt(argc, argv, "e:E:C:r:a:hv")) != -1)
	{
		switch (argvs)
		{
		case 'v':
			std::cout << "[RPiSingleAPM] version 0.9.0 Beta , Acess By TSKangetsu\n"
					  << "	checkout : https://github.com/TSKangetsu/RPiSingleAPM \n";
			break;
		case 'r':
		{
			RPiSingleAPM APM_Settle;
			configSettle(CONFIGDIR, optarg, setting);
			APM_Settle.RPiSingleAPMInit(setting);
			// Because of PiGPIO ,if you must handle Signal, should be call after RPiSingleAPMInit()
			std::signal(SIGINT, SignalCatch);
			std::signal(SIGTERM, SignalCatch);
			//
			APM_Settle.RPiSingleAPMStartUp();
			APM_Settle.TaskThreadBlock();
		}
		break;
		case 'e':
		{
			RPiSingleAPM APM_Settle;
			configSettle(CONFIGDIR, optarg, setting);
			APM_Settle.RPiSingleAPMInit(setting);
			APM_Settle.APMCalibrator(ESCCalibration, CaliESCStart, 0, data);
		}
		break;
		case 'E':
		{
			RPiSingleAPM APM_Settle;
			configSettle(CONFIGDIR, optarg, setting);
			APM_Settle.RPiSingleAPMInit(setting);
			while (true)
			{
				int PIN, VALUE;
				std::cin >> PIN;
				std::cin >> VALUE;
				data[0] = PIN;
				APM_Settle.APMCalibrator(ESCCalibration, CaliESCUserDefine, VALUE, data);
			}
		}
		break;
		case 'C':
		{
			RPiSingleAPM APM_Settle;
			configSettle(CONFIGDIR, optarg, setting);
			APM_Settle.RPiSingleAPMInit(setting);
			APM_Settle.APMCalibrator(COMPASSCalibration, -1, -1, data);
			configWrite(CONFIGDIR, "_flag_COMPASS_Y_Scaler", data[CompassYScaler]);
			configWrite(CONFIGDIR, "_flag_COMPASS_Z_Scaler", data[CompassZScaler]);
			configWrite(CONFIGDIR, "_flag_COMPASS_X_Offset", data[CompassXOffset]);
			configWrite(CONFIGDIR, "_flag_COMPASS_Y_Offset", data[CompassYOffset]);
			configWrite(CONFIGDIR, "_flag_COMPASS_Z_Offset", data[CompassZOffset]);
		}
		break;
		case 'a':
		{
			int a;
			double tmp[50] = {0};
			RPiSingleAPM APM_Settle;
			configSettle(CONFIGDIR, optarg, setting);
			APM_Settle.RPiSingleAPMInit(setting);
			std::cout << "start calibration Nose Up and Type int and enter:"
					  << " \n";
			std::cin >> a;
			APM_Settle.APMCalibrator(ACCELCalibration, MPUAccelNoseUp, a, tmp);
			std::cout << "start calibration Nose Down and Type int and enter:"
					  << " \n";
			std::cin >> a;
			APM_Settle.APMCalibrator(ACCELCalibration, MPUAccelNoseDown, a, tmp);
			std::cout << "start calibration Nose Right Up and Type int and enter:"
					  << " \n";
			std::cin >> a;
			APM_Settle.APMCalibrator(ACCELCalibration, MPUAccelNoseRight, a, tmp);
			std::cout << "start calibration Nose Left Up and Type int and enter:"
					  << " \n";
			std::cin >> a;
			APM_Settle.APMCalibrator(ACCELCalibration, MPUAccelNoseLeft, a, tmp);
			std::cout << "start calibration Nose Top  and Type int and enter:"
					  << " \n";
			std::cin >> a;
			APM_Settle.APMCalibrator(ACCELCalibration, MPUAccelNoseTop, a, tmp);
			std::cout << "start calibration Nose Rev and Type int and enter:"
					  << " \n";
			std::cin >> a;
			APM_Settle.APMCalibrator(ACCELCalibration, MPUAccelNoseRev, a, tmp);
			APM_Settle.APMCalibrator(ACCELCalibration, MPUAccelCaliGet, a, tmp);
			for (size_t i = 0; i < 15; i++)
			{
				std::cout << tmp[i] << " \n";
			}

			configWrite(CONFIGDIR, "_flag_MPU9250_A_X_Cali", tmp[MPUAccelCaliX]);
			configWrite(CONFIGDIR, "_flag_MPU9250_A_Y_Cali", tmp[MPUAccelCaliY]);
			configWrite(CONFIGDIR, "_flag_MPU9250_A_Z_Cali", tmp[MPUAccelCaliZ]);
			configWrite(CONFIGDIR, "_flag_MPU9250_A_X_Scal", tmp[MPUAccelScalX]);
			configWrite(CONFIGDIR, "_flag_MPU9250_A_Y_Scal", tmp[MPUAccelScalY]);
			configWrite(CONFIGDIR, "_flag_MPU9250_A_Z_Scal", tmp[MPUAccelScalZ]);
		}
		break;
		//--------------------------------------------------------------------------------//
		case 'h':
		{
		}
		break;
			//--------------------------------------------------------------------------------//
		}
	}
}

void configSettle(const char *configDir, const char *substr, APMSettinngs &APMInit)
{
	std::ifstream config(configDir);
	std::string content((std::istreambuf_iterator<char>(config)),
						(std::istreambuf_iterator<char>()));
	nlohmann::json Mas = nlohmann::json::parse(content);
	nlohmann::json Configdata = Mas[substr];
	//==========================================================Device Type=======/
	APMInit.DC.RC_Type = Configdata["RC_Type"].get<int>();
	APMInit.DC.MPU9250_Type = Configdata["MPU9250_Type"].get<int>();

	APMInit.DC.__RCDevice = Configdata["__RCDevice"].get<std::string>();
	APMInit.DC.__GPSDevice = Configdata["__GPSDevice"].get<std::string>();
	APMInit.DC.__FlowDevice = Configdata["__FlowDevice"].get<std::string>();
	APMInit.DC.__MPUDeviceSPI = Configdata["__MPUDeviceSPI"].get<std::string>();
	APMInit.DC.__I2CDevice = Configdata["__I2CDevice"].get<std::string>();

	APMInit.DC._IsGPSEnable = Configdata["_IsGPSEnable"].get<bool>();
	APMInit.DC._IsFlowEnable = Configdata["_IsFlowEnable"].get<bool>();
	APMInit.DC._IsBAROEnable = Configdata["_IsBAROEnable"].get<bool>();
	APMInit.DC._IsRCSafeEnable = Configdata["_IsRCSafeEnable"].get<bool>();
	APMInit.DC._IsBlackBoxEnable = Configdata["_IsBlackBoxEnable"].get<bool>();

	APMInit.DC.IMU_Freqeuncy = Configdata["IMU_Freqeuncy"].get<int>();
	APMInit.DC.RXT_Freqeuncy = Configdata["RXT_Freqeuncy"].get<int>();
	APMInit.DC.ESC_Freqeuncy = Configdata["ESC_Freqeuncy"].get<int>();
	APMInit.DC.BBC_Freqeuncy = Configdata["BBC_Freqeuncy"].get<int>();
	APMInit.DC.BBC_PInterval = Configdata["BBC_PInterval"].get<std::string>();
	//==========================================================Controller cofig==/
	APMInit.RC._flag_RC_Min_PWM_Value = Configdata["_flag_RC_Min_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_Mid_PWM_Value = Configdata["_flag_RC_Mid_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_Max_PWM_Value = Configdata["_flag_RC_Max_PWM_Value"].get<int>();
	//===========================
	APMInit.RC._flag_RC_ARM_PWM_Value = Configdata["_flag_RC_ARM_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_ARM_PWM_Channel = Configdata["_flag_RC_ARM_PWM_Channel"].get<int>();

	APMInit.RC._flag_RC_AP_RateHold_PWM_Value = Configdata["_flag_RC_AP_RateHold_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_AP_RateHold_PWM_Channel = Configdata["_flag_RC_AP_RateHold_PWM_Channel"].get<int>();

	APMInit.RC._flag_RC_AP_AutoStable_PWM_Value = Configdata["_flag_RC_AP_AutoStable_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_AP_AutoStable_PWM_Channel = Configdata["_flag_RC_AP_AutoStable_PWM_Channel"].get<int>();

	APMInit.RC._flag_RC_AP_AltHold_PWM_Value = Configdata["_flag_RC_AP_AltHold_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_AP_AltHold_PWM_Channel = Configdata["_flag_RC_AP_AltHold_PWM_Channel"].get<int>();

	APMInit.RC._flag_RC_AP_SpeedHold_PWM_Value = Configdata["_flag_RC_AP_SpeedHold_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_AP_SpeedHold_PWM_Channel = Configdata["_flag_RC_AP_SpeedHold_PWM_Channel"].get<int>();

	APMInit.RC._flag_RC_AP_PositionHold_PWM_Value = Configdata["_flag_RC_AP_PositionHold_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_AP_PositionHold_PWM_Channel = Configdata["_flag_RC_AP_PositionHold_PWM_Channel"].get<int>();

	APMInit.RC._flag_RC_AP_UserAuto_PWM_Value = Configdata["_flag_RC_AP_UserAuto_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_AP_UserAuto_PWM_Channel = Configdata["_flag_RC_AP_UserAuto_PWM_Channel"].get<int>();
	//===========================
	APMInit.RC._flag_RCIsReserv__Roll = Configdata["_flag_RCIsReserv__Roll"].get<int>();
	APMInit.RC._flag_RCIsReserv_Pitch = Configdata["_flag_RCIsReserv_Pitch"].get<int>();
	APMInit.RC._flag_RCIsReserv___Yaw = Configdata["_flag_RCIsReserv___Yaw"].get<int>();
	//==========================================================ESC cofig=========/
	APMInit.OC._flag_A1_Pin = Configdata["_flag_A1_Pin"].get<int>();
	APMInit.OC._flag_A2_Pin = Configdata["_flag_A2_Pin"].get<int>();
	APMInit.OC._flag_B1_Pin = Configdata["_flag_B1_Pin"].get<int>();
	APMInit.OC._flag_B2_Pin = Configdata["_flag_B2_Pin"].get<int>();
	APMInit.OC._flag_YAWOut_Reverse = Configdata["_flag_YAWOut_Reverse"].get<float>();
	APMInit.OC._flag_ESC_Lazy_Per = Configdata["_flag_ESC_Lazy_Per"].get<float>();
	APMInit.OC.ESCPLFrequency = Configdata["ESCPLFrequency"].get<int>();
	APMInit.OC.ESCControllerType = Configdata["ESCControllerType"].get<int>();
	//==================================================================PID cofig==/
	APMInit.PC._flag_PID_P__Roll_Gain = Configdata["_flag_PID_P__Roll_Gain"].get<float>();
	APMInit.PC._flag_PID_P_Pitch_Gain = Configdata["_flag_PID_P_Pitch_Gain"].get<float>();
	APMInit.PC._flag_PID_P___Yaw_Gain = Configdata["_flag_PID_P___Yaw_Gain"].get<float>();
	APMInit.PC._flag_PID_P_Alt_Gain = Configdata["_flag_PID_P_Alt_Gain"].get<float>();
	APMInit.PC._flag_PID_P_PosX_Gain = Configdata["_flag_PID_P_PosX_Gain"].get<float>();
	APMInit.PC._flag_PID_P_PosY_Gain = Configdata["_flag_PID_P_PosY_Gain"].get<float>();
	APMInit.PC._flag_PID_P_SpeedZ_Gain = Configdata["_flag_PID_P_SpeedZ_Gain"].get<float>();
	APMInit.PC._flag_PID_P_SpeedX_Gain = Configdata["_flag_PID_P_SpeedX_Gain"].get<float>();
	APMInit.PC._flag_PID_P_SpeedY_Gain = Configdata["_flag_PID_P_SpeedY_Gain"].get<float>();

	APMInit.PC._flag_PID_I__Roll_Gain = Configdata["_flag_PID_I__Roll_Gain"].get<float>();
	APMInit.PC._flag_PID_I_Pitch_Gain = Configdata["_flag_PID_I_Pitch_Gain"].get<float>();
	APMInit.PC._flag_PID_I___Yaw_Gain = Configdata["_flag_PID_I___Yaw_Gain"].get<float>();
	APMInit.PC._flag_PID_I_Alt_Gain = Configdata["_flag_PID_I_Alt_Gain"].get<float>();
	APMInit.PC._flag_PID_I_PosX_Gain = Configdata["_flag_PID_I_PosX_Gain"].get<float>();
	APMInit.PC._flag_PID_I_PosY_Gain = Configdata["_flag_PID_I_PosY_Gain"].get<float>();
	APMInit.PC._flag_PID_I_SpeedZ_Gain = Configdata["_flag_PID_I_SpeedZ_Gain"].get<float>();
	APMInit.PC._flag_PID_I_SpeedX_Gain = Configdata["_flag_PID_I_SpeedX_Gain"].get<float>();
	APMInit.PC._flag_PID_I_SpeedY_Gain = Configdata["_flag_PID_I_SpeedY_Gain"].get<float>();
	APMInit.PC._flag_PID_I__Roll_Max__Value = Configdata["_flag_PID_I__Roll_Max__Value"].get<float>();
	APMInit.PC._flag_PID_I_Pitch_Max__Value = Configdata["_flag_PID_I_Pitch_Max__Value"].get<float>();
	APMInit.PC._flag_PID_I___Yaw_Max__Value = Configdata["_flag_PID_I___Yaw_Max__Value"].get<float>();

	APMInit.PC._flag_PID_D__Roll_Gain = Configdata["_flag_PID_D__Roll_Gain"].get<float>();
	APMInit.PC._flag_PID_D_Pitch_Gain = Configdata["_flag_PID_D_Pitch_Gain"].get<float>();
	APMInit.PC._flag_PID_D___Yaw_Gain = Configdata["_flag_PID_D___Yaw_Gain"].get<float>();
	APMInit.PC._flag_PID_D_SpeedZ_Gain = Configdata["_flag_PID_D_SpeedZ_Gain"].get<float>();
	APMInit.PC._flag_PID_D_SpeedX_Gain = Configdata["_flag_PID_D_SpeedX_Gain"].get<float>();
	APMInit.PC._flag_PID_D_SpeedY_Gain = Configdata["_flag_PID_D_SpeedY_Gain"].get<float>();

	APMInit.PC._flag_PID_Hover_Throttle = Configdata["_flag_PID_Hover_Throttle"].get<float>();
	APMInit.PC._flag_PID_Level_Max = Configdata["_flag_PID_Level_Max"].get<float>();
	APMInit.PC._flag_PID_Rate_Limit = Configdata["_flag_PID_Rate_Limit"].get<float>();
	APMInit.PC._flag_PID_Alt_Level_Max = Configdata["_flag_PID_Alt_Level_Max"].get<float>();
	APMInit.PC._flag_PID_Pos_Level_Max = Configdata["_flag_PID_Pos_Level_Max"].get<float>();

	APMInit.PC._flag_PID_Takeoff_Altitude = Configdata["_flag_PID_Takeoff_Altitude"].get<float>();
	APMInit.PC._flag_PID_Alt_Speed_Max = Configdata["_flag_PID_Alt_Speed_Max"].get<float>();
	APMInit.PC._flag_PID_Alt_Accel_Max = Configdata["_flag_PID_Alt_Accel_Max"].get<float>();
	APMInit.PC._flag_PID_PosMan_Speed_Max = Configdata["_flag_PID_PosMan_Speed_Max"].get<float>();
	APMInit.PC._flag_PID_Pos_Accel_Max = Configdata["_flag_PID_Pos_Accel_Max"].get<float>();
	APMInit.PC._flag_PID_Pos_Speed_Max = Configdata["_flag_PID_Pos_Speed_Max"].get<float>();

	APMInit.PC._flag_PID_AngleRate__Roll_Gain = Configdata["_flag_PID_AngleRate__Roll_Gain"].get<float>();
	APMInit.PC._flag_PID_AngleRate_Pitch_Gain = Configdata["_flag_PID_AngleRate_Pitch_Gain"].get<float>();
	APMInit.PC._flag_PID_AngleRate___Yaw_Gain = Configdata["_flag_PID_AngleRate___Yaw_Gain"].get<float>();
	APMInit.PC._flag_PID_RCRate__Roll_Gain = Configdata["_flag_PID_RCRate__Roll_Gain"].get<float>();
	APMInit.PC._flag_PID_RCRate_Pitch_Gain = Configdata["_flag_PID_RCRate_Pitch_Gain"].get<float>();
	APMInit.PC._flag_PID_RCRate___Yaw_Gain = Configdata["_flag_PID_RCRate___Yaw_Gain"].get<float>();
	APMInit.PC._flag_PID_RCAngle__Roll_Gain = Configdata["_flag_PID_RCAngle__Roll_Gain"].get<float>();
	APMInit.PC._flag_PID_RCAngle_Pitch_Gain = Configdata["_flag_PID_RCAngle_Pitch_Gain"].get<float>();
	APMInit.PC._flag_PID_RCAngle___Yaw_Gain = Configdata["_flag_PID_RCAngle___Yaw_Gain"].get<float>();

	APMInit.PC._flag_PID_TPA_Trust = Configdata["_flag_PID_TPA_Trust"].get<float>();
	APMInit.PC._flag_PID_TPA_BreakPoint = Configdata["_flag_PID_TPA_BreakPoint"].get<float>();
	//==============================================================Sensors cofig==/
	APMInit.SC._flag_MPU9250_A_X_Cali = Configdata["_flag_MPU9250_A_X_Cali"].get<double>();
	APMInit.SC._flag_MPU9250_A_Y_Cali = Configdata["_flag_MPU9250_A_Y_Cali"].get<double>();
	APMInit.SC._flag_MPU9250_A_Z_Cali = Configdata["_flag_MPU9250_A_Z_Cali"].get<double>();
	APMInit.SC._flag_MPU9250_A_X_Scal = Configdata["_flag_MPU9250_A_X_Scal"].get<double>();
	APMInit.SC._flag_MPU9250_A_Y_Scal = Configdata["_flag_MPU9250_A_Y_Scal"].get<double>();
	APMInit.SC._flag_MPU9250_A_Z_Scal = Configdata["_flag_MPU9250_A_Z_Scal"].get<double>();
	APMInit.SC._flag_Accel__Roll_Cali = Configdata["_flag_Accel__Roll_Cali"].get<double>();
	APMInit.SC._flag_Accel_Pitch_Cali = Configdata["_flag_Accel_Pitch_Cali"].get<double>();

	APMInit.SC._flag_COMPASS_X_Offset = Configdata["_flag_COMPASS_X_Offset"].get<double>();
	APMInit.SC._flag_COMPASS_X_Scaler = Configdata["_flag_COMPASS_X_Scaler"].get<double>();
	APMInit.SC._flag_COMPASS_Y_Offset = Configdata["_flag_COMPASS_Y_Offset"].get<double>();
	APMInit.SC._flag_COMPASS_Y_Scaler = Configdata["_flag_COMPASS_Y_Scaler"].get<double>();
	APMInit.SC._flag_COMPASS_Z_Offset = Configdata["_flag_COMPASS_Z_Offset"].get<double>();
	APMInit.SC._flag_COMPASS_Z_Scaler = Configdata["_flag_COMPASS_Z_Scaler"].get<double>();
	//==============================================================Filter config==/
	APMInit.FC._flag_Filter_Gryo_Type = Configdata["_flag_Filter_Gryo_Type"].get<double>();
	APMInit.FC._flag_Filter_GryoST2_Type = Configdata["_flag_Filter_GryoST2_Type"].get<double>();
	APMInit.FC._flag_Filter_GYaw_CutOff = Configdata["_flag_Filter_GYaw_CutOff"].get<double>();
	APMInit.FC._flag_Filter_Gryo_CutOff = Configdata["_flag_Filter_Gryo_CutOff"].get<double>();
	APMInit.FC._flag_Filter_GryoST2_CutOff = Configdata["_flag_Filter_GryoST2_CutOff"].get<double>();
	APMInit.FC._flag_Filter_Gryo_NotchFreq = Configdata["_flag_Filter_Gryo_NotchFreq"].get<double>();
	APMInit.FC._flag_Filter_Gryo_NotchCutOff = Configdata["_flag_Filter_Gryo_NotchCutOff"].get<double>();
	APMInit.FC._flag_Filter_Gryo_DynamicNotchRange = Configdata["_flag_Filter_Gryo_DynamicNotchRange"].get<double>();
	APMInit.FC._flag_Filter_Gryo_DynamicNotchEnable = Configdata["_flag_Filter_Gryo_DynamicNotchEnable"].get<bool>();
	APMInit.FC._flag_Filter_Gryo_DynamicNotchMinFreq = Configdata["_flag_Filter_Gryo_DynamicNotchMinFreq"].get<double>();
	APMInit.FC._flag_Filter_Accel_Type = Configdata["_flag_Filter_Accel_Type"].get<double>();
	APMInit.FC._flag_Filter_Accel_CutOff = Configdata["_flag_Filter_Accel_CutOff"].get<double>();
	APMInit.FC._flag_Filter_AngleMix_Alpha = Configdata["_flag_Filter_AngleMix_Alpha"].get<double>();

	APMInit.FC._flag_Baro_Trust_Beta = Configdata["_flag_Baro_Trust_Beta"].get<double>();
	APMInit.FC._flag_Accel_Trust_Beta = Configdata["_flag_Accel_Trust_Beta"].get<double>();
	APMInit.FC._flag_Sonar_Trust_Beta = Configdata["_flag_Sonar_Trust_Beta"].get<double>();
	APMInit.FC._flag_GPSAlt_Trust_Beta = Configdata["_flag_GPSAlt_Trust_Beta"].get<double>();
	APMInit.FC._flag_AccelBias_Trust_Beta = Configdata["_flag_AccelBias_Trust_Beta"].get<double>();

	APMInit.FC._flag_Filter_RC_CutOff = Configdata["_flag_Filter_RC_CutOff"].get<double>();
	APMInit.FC._flag_Filter_AngleRate_CutOff = Configdata["_flag_Filter_AngleRate_CutOff"].get<double>();

	APMInit.FC._flag_Filter_PID_I_CutOff = Configdata["_flag_Filter_PID_I_CutOff"].get<double>();
	APMInit.FC._flag_Filter_PID_D_ST1_CutOff = Configdata["_flag_Filter_PID_D_ST1_CutOff"].get<double>();
	APMInit.FC._flag_Filter_PID_D_ST2_CutOff = Configdata["_flag_Filter_PID_D_ST2_CutOff"].get<double>();

	APMInit.FC._flag_GPS_Config_Beta = Configdata["_flag_GPS_Config_Beta"].get<double>();
	APMInit.FC._flag_Flow_Config_Beta = Configdata["_flag_Flow_Config_Beta"].get<double>();
	APMInit.FC._flag_Braking_Speed_Gain = Configdata["_flag_Braking_Speed_Gain"].get<double>();
	APMInit.FC._flag_Braking_AccelMax_Gain = Configdata["_flag_Braking_AccelMax_Gain"].get<double>();
}

void configWrite(const char *configDir, const char *Target, double obj)
{
	std::ifstream config(configDir);
	std::string content((std::istreambuf_iterator<char>(config)),
						(std::istreambuf_iterator<char>()));
	nlohmann::json Configdata = nlohmann::json::parse(content);
	Configdata[Target] = obj;
	std::ofstream configs;
	configs.open(configDir);
	configs << std::setw(4) << Configdata << std::endl;
	configs.close();
}

void SignalCatch(int Signal)
{
	SingleAPMAPI::SystemSignal = Signal;
};