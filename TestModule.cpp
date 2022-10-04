#include "./src/SingleAPM.hpp"
#include <fstream>
#include "Drive_Json.hpp"
using namespace SingleAPMAPI;

#ifndef GIT_COMMIT_HASH
#define GIT_COMMIT_HASH "0000000" // 0000000 means uninitialized
#endif

#define CONFIGDIR "/boot/APMConfig.json"

void configWrite(const char *configDir, const char *substr, const char *Target, double obj);
void configSettle(const char *configDir, const char *substr, APMSettinngs &APMInit);
void SignalCatch(int Signal);

int main(int argc, char *argv[])
{
	int argvs;
	double data[20] = {0};
	APMSettinngs setting;
	while ((argvs = getopt(argc, argv, "e:E:C:r:a:jh")) != -1)
	{
		switch (argvs)
		{
		case 'h':
		{
			std::cout << "[RPiSingleAPM] version 0.9.0 Beta, build: "
					  << GIT_COMMIT_HASH
					  << " , Acess By TSKangetsu\n"
					  << "	checkout : https://github.com/TSKangetsu/RPiSingleAPM \n";
		}
		break;
		case 'r':
		{
			// system("clear");
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
			std::signal(SIGINT, SignalCatch);
			std::cout << "start Calibrating COMPASS , Input Any to stop ...";
			std::cout.flush();
			APM_Settle.APMCalibrator(COMPASSCalibration, -1, -1, data);
			std::cout << " Done\n";
			//
			configWrite(CONFIGDIR, optarg, "_flag_COMPASS_X_Offset", (int)data[CompassXOffset]);
			configWrite(CONFIGDIR, optarg, "_flag_COMPASS_X_Scaler", (int)data[CompassXScaler]);
			configWrite(CONFIGDIR, optarg, "_flag_COMPASS_Y_Offset", (int)data[CompassYOffset]);
			configWrite(CONFIGDIR, optarg, "_flag_COMPASS_Y_Scaler", (int)data[CompassYScaler]);
			configWrite(CONFIGDIR, optarg, "_flag_COMPASS_Z_Offset", (int)data[CompassZOffset]);
			configWrite(CONFIGDIR, optarg, "_flag_COMPASS_Z_Scaler", (int)data[CompassZScaler]);
			configWrite(CONFIGDIR, optarg, "_flag_COMPASS_V_Offset", (int)data[CompassVOffset]);
			configWrite(CONFIGDIR, optarg, "_flag_COMPASS_V_Scaler", (int)data[CompassVScaler]);
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

			configWrite(CONFIGDIR, optarg, "_flag_MPU9250_A_X_Cali", tmp[MPUAccelCaliX]);
			configWrite(CONFIGDIR, optarg, "_flag_MPU9250_A_Y_Cali", tmp[MPUAccelCaliY]);
			configWrite(CONFIGDIR, optarg, "_flag_MPU9250_A_Z_Cali", tmp[MPUAccelCaliZ]);
			configWrite(CONFIGDIR, optarg, "_flag_MPU9250_A_X_Scal", tmp[MPUAccelScalX]);
			configWrite(CONFIGDIR, optarg, "_flag_MPU9250_A_Y_Scal", tmp[MPUAccelScalY]);
			configWrite(CONFIGDIR, optarg, "_flag_MPU9250_A_Z_Scal", tmp[MPUAccelScalZ]);
		}
		break;
		case 'j':
		{
			std::ifstream config(CONFIGDIR);
			std::string content((std::istreambuf_iterator<char>(config)),
								(std::istreambuf_iterator<char>()));
			nlohmann::json Mas = nlohmann::json::parse(content);

			std::ofstream configs;
			configs.open(CONFIGDIR);
			configs << std::setw(4) << Mas << std::endl;
			configs.close();
		}
		break;
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
	nlohmann::json DC = Configdata["Device"];
	APMInit.DC.RC_Type = DC["RC_Type"].get<int>();
	APMInit.DC.MPU9250_Type = DC["MPU9250_Type"].get<int>();

	APMInit.DC.__RCDevice = DC["__RCDevice"].get<std::string>();
	APMInit.DC.__GPSDevice = DC["__GPSDevice"].get<std::string>();
	APMInit.DC.__FlowDevice = DC["__FlowDevice"].get<std::string>();
	APMInit.DC.__MPUDeviceSPI = DC["__MPUDeviceSPI"].get<std::string>();
	APMInit.DC.__I2CDevice = DC["__I2CDevice"].get<std::string>();

	APMInit.DC._IsGPSEnable = DC["_IsGPSEnable"].get<bool>();
	APMInit.DC._IsFlowEnable = DC["_IsFlowEnable"].get<bool>();
	APMInit.DC._IsBAROEnable = DC["_IsBAROEnable"].get<bool>();
	APMInit.DC._IsRCSafeEnable = DC["_IsRCSafeEnable"].get<bool>();
	APMInit.DC._IsBlackBoxEnable = DC["_IsBlackBoxEnable"].get<bool>();

	APMInit.DC.IMU_Freqeuncy = DC["IMU_Freqeuncy"].get<int>();
	APMInit.DC.RXT_Freqeuncy = DC["RXT_Freqeuncy"].get<int>();
	APMInit.DC.ESC_Freqeuncy = DC["ESC_Freqeuncy"].get<int>();
	APMInit.DC.BBC_Freqeuncy = DC["BBC_Freqeuncy"].get<int>();
	APMInit.DC.BBC_PInterval = DC["BBC_PInterval"].get<std::string>();
	//==========================================================Controller cofig==/
	nlohmann::json RC = Configdata["Recvier"];
	APMInit.RC._flag_RC_Min_PWM_Value = RC["_flag_RC_Min_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_Mid_PWM_Value = RC["_flag_RC_Mid_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_Max_PWM_Value = RC["_flag_RC_Max_PWM_Value"].get<int>();
	//===========================
	APMInit.RC._flag_RC_ARM_PWM_Value = RC["_flag_RC_ARM_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_ARM_PWM_Channel = RC["_flag_RC_ARM_PWM_Channel"].get<int>();

	APMInit.RC._flag_RC_AP_RateHold_PWM_Value = RC["_flag_RC_AP_RateHold_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_AP_RateHold_PWM_Channel = RC["_flag_RC_AP_RateHold_PWM_Channel"].get<int>();

	APMInit.RC._flag_RC_AP_AutoStable_PWM_Value = RC["_flag_RC_AP_AutoStable_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_AP_AutoStable_PWM_Channel = RC["_flag_RC_AP_AutoStable_PWM_Channel"].get<int>();

	APMInit.RC._flag_RC_AP_AltHold_PWM_Value = RC["_flag_RC_AP_AltHold_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_AP_AltHold_PWM_Channel = RC["_flag_RC_AP_AltHold_PWM_Channel"].get<int>();

	APMInit.RC._flag_RC_AP_SpeedHold_PWM_Value = RC["_flag_RC_AP_SpeedHold_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_AP_SpeedHold_PWM_Channel = RC["_flag_RC_AP_SpeedHold_PWM_Channel"].get<int>();

	APMInit.RC._flag_RC_AP_PositionHold_PWM_Value = RC["_flag_RC_AP_PositionHold_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_AP_PositionHold_PWM_Channel = RC["_flag_RC_AP_PositionHold_PWM_Channel"].get<int>();

	APMInit.RC._flag_RC_AP_UserAuto_PWM_Value = RC["_flag_RC_AP_UserAuto_PWM_Value"].get<int>();
	APMInit.RC._flag_RC_AP_UserAuto_PWM_Channel = RC["_flag_RC_AP_UserAuto_PWM_Channel"].get<int>();
	//===========================
	APMInit.RC._flag_RCIsReserv__Roll = RC["_flag_RCIsReserv__Roll"].get<int>();
	APMInit.RC._flag_RCIsReserv_Pitch = RC["_flag_RCIsReserv_Pitch"].get<int>();
	APMInit.RC._flag_RCIsReserv___Yaw = RC["_flag_RCIsReserv___Yaw"].get<int>();
	//==========================================================ESC cofig=========/
	nlohmann::json OC = Configdata["PID"]["Output"];
	APMInit.OC._flag_A1_Pin = OC["_flag_A1_Pin"].get<int>();
	APMInit.OC._flag_A2_Pin = OC["_flag_A2_Pin"].get<int>();
	APMInit.OC._flag_B1_Pin = OC["_flag_B1_Pin"].get<int>();
	APMInit.OC._flag_B2_Pin = OC["_flag_B2_Pin"].get<int>();
	APMInit.OC._flag_YAWOut_Reverse = OC["_flag_YAWOut_Reverse"].get<float>();
	APMInit.OC._flag_ESC_Lazy_Per = OC["_flag_ESC_Lazy_Per"].get<float>();
	APMInit.OC.ESCPLFrequency = OC["ESCPLFrequency"].get<int>();
	APMInit.OC.ESCControllerType = OC["ESCControllerType"].get<int>();
	//==================================================================PID cofig==/
	nlohmann::json PAC = Configdata["PID"]["AttitudePID"];
	APMInit.PC._flag_PID_P__Roll_Gain = PAC["_flag_PID_P__Roll_Gain"].get<float>();
	APMInit.PC._flag_PID_P_Pitch_Gain = PAC["_flag_PID_P_Pitch_Gain"].get<float>();
	APMInit.PC._flag_PID_P___Yaw_Gain = PAC["_flag_PID_P___Yaw_Gain"].get<float>();
	APMInit.PC._flag_PID_I__Roll_Gain = PAC["_flag_PID_I__Roll_Gain"].get<float>();
	APMInit.PC._flag_PID_I_Pitch_Gain = PAC["_flag_PID_I_Pitch_Gain"].get<float>();
	APMInit.PC._flag_PID_I___Yaw_Gain = PAC["_flag_PID_I___Yaw_Gain"].get<float>();
	APMInit.PC._flag_PID_I__Roll_Max__Value = PAC["_flag_PID_I__Roll_Max__Value"].get<float>();
	APMInit.PC._flag_PID_I_Pitch_Max__Value = PAC["_flag_PID_I_Pitch_Max__Value"].get<float>();
	APMInit.PC._flag_PID_I___Yaw_Max__Value = PAC["_flag_PID_I___Yaw_Max__Value"].get<float>();
	APMInit.PC._flag_PID_D__Roll_Gain = PAC["_flag_PID_D__Roll_Gain"].get<float>();
	APMInit.PC._flag_PID_D_Pitch_Gain = PAC["_flag_PID_D_Pitch_Gain"].get<float>();
	APMInit.PC._flag_PID_D___Yaw_Gain = PAC["_flag_PID_D___Yaw_Gain"].get<float>();
	APMInit.PC._flag_PID_Level_Max = PAC["_flag_PID_Level_Max"].get<float>();
	APMInit.PC._flag_PID_Rate_Limit = PAC["_flag_PID_Rate_Limit"].get<float>();
	APMInit.PC._flag_PID_AngleRate__Roll_Gain = PAC["_flag_PID_AngleRate__Roll_Gain"].get<float>();
	APMInit.PC._flag_PID_AngleRate_Pitch_Gain = PAC["_flag_PID_AngleRate_Pitch_Gain"].get<float>();
	APMInit.PC._flag_PID_AngleRate___Yaw_Gain = PAC["_flag_PID_AngleRate___Yaw_Gain"].get<float>();
	APMInit.PC._flag_PID_RCRate__Roll_Gain = PAC["_flag_PID_RCRate__Roll_Gain"].get<float>();
	APMInit.PC._flag_PID_RCRate_Pitch_Gain = PAC["_flag_PID_RCRate_Pitch_Gain"].get<float>();
	APMInit.PC._flag_PID_RCRate___Yaw_Gain = PAC["_flag_PID_RCRate___Yaw_Gain"].get<float>();
	APMInit.PC._flag_PID_RCAngle__Roll_Gain = PAC["_flag_PID_RCAngle__Roll_Gain"].get<float>();
	APMInit.PC._flag_PID_RCAngle_Pitch_Gain = PAC["_flag_PID_RCAngle_Pitch_Gain"].get<float>();
	APMInit.PC._flag_PID_RCAngle___Yaw_Gain = PAC["_flag_PID_RCAngle___Yaw_Gain"].get<float>();
	APMInit.PC._flag_PID_TPA_Trust = PAC["_flag_PID_TPA_Trust"].get<float>();
	APMInit.PC._flag_PID_TPA_BreakPoint = PAC["_flag_PID_TPA_BreakPoint"].get<float>();
	nlohmann::json PLC = Configdata["PID"]["AltitudePID"];
	APMInit.PC._flag_PID_P_Alt_Gain = PLC["_flag_PID_P_Alt_Gain"].get<float>();
	APMInit.PC._flag_PID_I_Alt_Gain = PLC["_flag_PID_I_Alt_Gain"].get<float>();
	APMInit.PC._flag_PID_P_SpeedZ_Gain = PLC["_flag_PID_P_SpeedZ_Gain"].get<float>();
	APMInit.PC._flag_PID_I_SpeedZ_Gain = PLC["_flag_PID_I_SpeedZ_Gain"].get<float>();
	APMInit.PC._flag_PID_D_SpeedZ_Gain = PLC["_flag_PID_D_SpeedZ_Gain"].get<float>();
	APMInit.PC._flag_PID_Hover_Throttle = PLC["_flag_PID_Hover_Throttle"].get<float>();
	APMInit.PC._flag_PID_Alt_Level_Max = PLC["_flag_PID_Alt_Level_Max"].get<float>();
	APMInit.PC._flag_PID_Alt_Speed_Max = PLC["_flag_PID_Alt_Speed_Max"].get<float>();
	APMInit.PC._flag_PID_Alt_Accel_Max = PLC["_flag_PID_Alt_Accel_Max"].get<float>();
	APMInit.PC._flag_PID_Takeoff_Altitude = PLC["_flag_PID_Takeoff_Altitude"].get<float>();
	nlohmann::json PNC = Configdata["PID"]["NAVPID"];
	APMInit.PC._flag_PID_P_PosX_Gain = PNC["_flag_PID_P_PosX_Gain"].get<float>();
	APMInit.PC._flag_PID_P_PosY_Gain = PNC["_flag_PID_P_PosY_Gain"].get<float>();
	APMInit.PC._flag_PID_I_PosX_Gain = PNC["_flag_PID_I_PosX_Gain"].get<float>();
	APMInit.PC._flag_PID_I_PosY_Gain = PNC["_flag_PID_I_PosY_Gain"].get<float>();
	APMInit.PC._flag_PID_P_SpeedX_Gain = PNC["_flag_PID_P_SpeedX_Gain"].get<float>();
	APMInit.PC._flag_PID_P_SpeedY_Gain = PNC["_flag_PID_P_SpeedY_Gain"].get<float>();
	APMInit.PC._flag_PID_I_SpeedX_Gain = PNC["_flag_PID_I_SpeedX_Gain"].get<float>();
	APMInit.PC._flag_PID_I_SpeedY_Gain = PNC["_flag_PID_I_SpeedY_Gain"].get<float>();
	APMInit.PC._flag_PID_D_SpeedX_Gain = PNC["_flag_PID_D_SpeedX_Gain"].get<float>();
	APMInit.PC._flag_PID_D_SpeedY_Gain = PNC["_flag_PID_D_SpeedY_Gain"].get<float>();
	APMInit.PC._flag_PID_Pos_Level_Max = PNC["_flag_PID_Pos_Level_Max"].get<float>();
	APMInit.PC._flag_PID_PosMan_Speed_Max = PNC["_flag_PID_PosMan_Speed_Max"].get<float>();
	APMInit.PC._flag_PID_Pos_Accel_Max = PNC["_flag_PID_Pos_Accel_Max"].get<float>();
	APMInit.PC._flag_PID_Pos_Speed_Max = PNC["_flag_PID_Pos_Speed_Max"].get<float>();
	//==============================================================Sensors cofig==/
	nlohmann::json SCS = Configdata["Sensor"];
	APMInit.SC._flag_MPU_Flip__Roll = SCS["_flag_MPU_Flip__Roll"].get<int>();
	APMInit.SC._flag_MPU_Flip_Pitch = SCS["_flag_MPU_Flip_Pitch"].get<int>();
	APMInit.SC._flag_MPU_Flip___Yaw = SCS["_flag_MPU_Flip___Yaw"].get<int>();

	APMInit.SC._flag_MPU9250_A_X_Cali = SCS["_flag_MPU9250_A_X_Cali"].get<double>();
	APMInit.SC._flag_MPU9250_A_Y_Cali = SCS["_flag_MPU9250_A_Y_Cali"].get<double>();
	APMInit.SC._flag_MPU9250_A_Z_Cali = SCS["_flag_MPU9250_A_Z_Cali"].get<double>();
	APMInit.SC._flag_MPU9250_A_X_Scal = SCS["_flag_MPU9250_A_X_Scal"].get<double>();
	APMInit.SC._flag_MPU9250_A_Y_Scal = SCS["_flag_MPU9250_A_Y_Scal"].get<double>();
	APMInit.SC._flag_MPU9250_A_Z_Scal = SCS["_flag_MPU9250_A_Z_Scal"].get<double>();
	APMInit.SC._flag_Accel__Roll_Cali = SCS["_flag_Accel__Roll_Cali"].get<double>();
	APMInit.SC._flag_Accel_Pitch_Cali = SCS["_flag_Accel_Pitch_Cali"].get<double>();

	APMInit.SC._flag_COMPASS_X_Offset = SCS["_flag_COMPASS_X_Offset"].get<double>();
	APMInit.SC._flag_COMPASS_X_Scaler = SCS["_flag_COMPASS_X_Scaler"].get<double>();
	APMInit.SC._flag_COMPASS_Y_Offset = SCS["_flag_COMPASS_Y_Offset"].get<double>();
	APMInit.SC._flag_COMPASS_Y_Scaler = SCS["_flag_COMPASS_Y_Scaler"].get<double>();
	APMInit.SC._flag_COMPASS_Z_Offset = SCS["_flag_COMPASS_Z_Offset"].get<double>();
	APMInit.SC._flag_COMPASS_Z_Scaler = SCS["_flag_COMPASS_Z_Scaler"].get<double>();
	APMInit.SC._flag_COMPASS_V_Offset = SCS["_flag_COMPASS_V_Offset"].get<double>();
	APMInit.SC._flag_COMPASS_V_Scaler = SCS["_flag_COMPASS_V_Scaler"].get<double>();
	APMInit.SC._flag_COMPASS_Flip__Roll = SCS["_flag_COMPASS_Flip__Roll"].get<double>();
	APMInit.SC._flag_COMPASS_Flip_Pitch = SCS["_flag_COMPASS_Flip_Pitch"].get<double>();
	APMInit.SC._flag_COMPASS_Flip___Yaw = SCS["_flag_COMPASS_Flip___Yaw"].get<double>();
	APMInit.SC._flag_COMPASS_YAW_Offset = SCS["_flag_COMPASS_YAW_Offset"].get<double>();
	//==============================================================Filter config==/
	nlohmann::json SCF = Configdata["Filter"];
	APMInit.FC._flag_Filter_Gryo_Type = SCF["_flag_Filter_Gryo_Type"].get<double>();
	APMInit.FC._flag_Filter_GryoST2_Type = SCF["_flag_Filter_GryoST2_Type"].get<double>();
	APMInit.FC._flag_Filter_GYaw_CutOff = SCF["_flag_Filter_GYaw_CutOff"].get<double>();
	APMInit.FC._flag_Filter_Gryo_CutOff = SCF["_flag_Filter_Gryo_CutOff"].get<double>();
	APMInit.FC._flag_Filter_GryoST2_CutOff = SCF["_flag_Filter_GryoST2_CutOff"].get<double>();
	APMInit.FC._flag_Filter_Gryo_NotchFreq = SCF["_flag_Filter_Gryo_NotchFreq"].get<double>();
	APMInit.FC._flag_Filter_Gryo_NotchCutOff = SCF["_flag_Filter_Gryo_NotchCutOff"].get<double>();
	APMInit.FC._flag_Filter_Gryo_DynamicNotchRange = SCF["_flag_Filter_Gryo_DynamicNotchRange"].get<double>();
	APMInit.FC._flag_Filter_Gryo_DynamicNotchEnable = SCF["_flag_Filter_Gryo_DynamicNotchEnable"].get<bool>();
	APMInit.FC._flag_Filter_Gryo_DynamicNotchMinFreq = SCF["_flag_Filter_Gryo_DynamicNotchMinFreq"].get<double>();
	APMInit.FC._flag_Filter_Accel_Type = SCF["_flag_Filter_Accel_Type"].get<double>();
	APMInit.FC._flag_Filter_Accel_CutOff = SCF["_flag_Filter_Accel_CutOff"].get<double>();
	APMInit.FC._flag_Filter_AngleMix_Alpha = SCF["_flag_Filter_AngleMix_Alpha"].get<double>();

	APMInit.FC._flag_Baro_Trust_Beta = SCF["_flag_Baro_Trust_Beta"].get<double>();
	APMInit.FC._flag_Accel_Trust_Beta = SCF["_flag_Accel_Trust_Beta"].get<double>();
	APMInit.FC._flag_Sonar_Trust_Beta = SCF["_flag_Sonar_Trust_Beta"].get<double>();
	APMInit.FC._flag_GPSAlt_Trust_Beta = SCF["_flag_GPSAlt_Trust_Beta"].get<double>();
	APMInit.FC._flag_AccelBias_Trust_Beta = SCF["_flag_AccelBias_Trust_Beta"].get<double>();

	APMInit.FC._flag_Filter_RC_CutOff = SCF["_flag_Filter_RC_CutOff"].get<double>();
	APMInit.FC._flag_Filter_AngleRate_CutOff = SCF["_flag_Filter_AngleRate_CutOff"].get<double>();

	APMInit.FC._flag_Filter_PID_I_CutOff = SCF["_flag_Filter_PID_I_CutOff"].get<double>();
	APMInit.FC._flag_Filter_PID_D_ST1_CutOff = SCF["_flag_Filter_PID_D_ST1_CutOff"].get<double>();
	APMInit.FC._flag_Filter_PID_D_ST2_CutOff = SCF["_flag_Filter_PID_D_ST2_CutOff"].get<double>();

	APMInit.FC._flag_GPS_Config_Beta = SCF["_flag_GPS_Config_Beta"].get<double>();
	APMInit.FC._flag_Flow_Config_Beta = SCF["_flag_Flow_Config_Beta"].get<double>();
	APMInit.FC._flag_Braking_Speed_Gain = SCF["_flag_Braking_Speed_Gain"].get<double>();
	APMInit.FC._flag_Braking_AccelMax_Gain = SCF["_flag_Braking_AccelMax_Gain"].get<double>();
}

void configWrite(const char *configDir, const char *substr, const char *Target, double obj)
{
	std::ifstream config(configDir);
	std::string content((std::istreambuf_iterator<char>(config)),
						(std::istreambuf_iterator<char>()));
	nlohmann::json Configdata = nlohmann::json::parse(content);
	nlohmann::json subdata = Configdata[substr];
	subdata[Target] = obj;
	Configdata[substr] = subdata;
	std::ofstream configs;
	configs.open(configDir);
	configs << std::setw(4) << Configdata << std::endl;
	configs.close();
}

void SignalCatch(int Signal)
{
	SingleAPMAPI::SystemSignal = Signal;
};