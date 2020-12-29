#include "./src/SingleAPM.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
using namespace SingleAPMAPI;

void configWrite(const char *configDir, const char *Target, double obj);
void configSettle(const char *configDir, APMSettinngs &APMInit);

int main(int argc, char *argv[])
{
	system("clear");
	int argvs;
	double data[10];
	APMSettinngs setting;
	while ((argvs = getopt(argc, argv, "vteCrha")) != -1)
	{
		switch (argvs)
		{
		case 'v':
			std::cout << "[RPiSingleAPM] version 1.0.f Beta , Acess By TSKangetsu\n"
					  << "	checkout : https://github.com/TSKangetsu/RPiSingleAPM \n";
			break;
		case 'r':
		{
			RPiSingleAPM APM_Settle;
			configSettle("/etc/APMconfig.json", setting);
			APM_Settle.RPiSingleAPMInit(setting);
			APM_Settle.IMUSensorsTaskReg();
			APM_Settle.ControllerTaskReg();
			APM_Settle.AltholdSensorsTaskReg();
			APM_Settle.PositionTaskReg();
			APM_Settle.ESCUpdateTaskReg();
			APM_Settle.TaskThreadBlock();
		}
		break;
		case 'e':
		{
			RPiSingleAPM APM_Settle;
			configSettle("/etc/APMconfig.json", setting);
			APM_Settle.RPiSingleAPMInit(setting);
			APM_Settle.APMCalibrator(ESCCalibration, CaliESCStart, 0, data);
		}
		break;
		case 'C':
		{
			// RPiSingleAPM APM_Settle;
			// configSettle("/etc/APMconfig.json", setting);
			// APM_Settle.RPiSingleAPMInit(setting);
			// APM_Settle.APMCalibrator(1, data);
			// configWrite("/etc/APMconfig.json", "_flag_QMC5883L_M_Y_Scaler", data[0]);
			// configWrite("/etc/APMconfig.json", "_flag_QMC5883L_M_Z_Scaler", data[1]);
			// configWrite("/etc/APMconfig.json", "_flag_QMC5883L_M_X_Offset", data[2]);
			// configWrite("/etc/APMconfig.json", "_flag_QMC5883L_M_Y_Offset", data[3]);
			// configWrite("/etc/APMconfig.json", "_flag_QMC5883L_M_Z_Offset", data[4]);
			// configWrite("/etc/APMconfig.json", "_flag_MPU9250_M_Y_Scaler", data[5]);
			// configWrite("/etc/APMconfig.json", "_flag_MPU9250_M_Z_Scaler", data[6]);
			// configWrite("/etc/APMconfig.json", "_flag_MPU9250_M_X_Offset", data[7]);
			// configWrite("/etc/APMconfig.json", "_flag_MPU9250_M_Y_Offset", data[8]);
			// configWrite("/etc/APMconfig.json", "_flag_MPU9250_M_Z_Offset", data[9]);
		}
		break;
		case 'a':
		{
			RPiSingleAPM APM_Settle;
			configSettle("/etc/APMconfig.json", setting);
			APM_Settle.RPiSingleAPMInit(setting);
			APM_Settle.APMCalibrator(ACCELCalibration, CaliACCELHeadNormal, 0, data);
			configWrite("/etc/APMconfig.json", "_flag_MPU9250_A_X_Cali", data[0]);
			configWrite("/etc/APMconfig.json", "_flag_MPU9250_A_Y_Cali", data[1]);
			configWrite("/etc/APMconfig.json", "_flag_MPU9250_A_Z_Cali", data[2]);
		}
		break;
		//--------------------------------------------------------------------------------//
		case 'h':
		{
			std::cout << "using ArgeMent: \n"
					  << " -r : starting AirController \n"
					  << " -c : starting CalibrationESC \n";
		}
		break;
			//--------------------------------------------------------------------------------//
		}
	}
}

void configSettle(const char *configDir, APMSettinngs &APMInit)
{
	std::ifstream config(configDir);
	std::string content((std::istreambuf_iterator<char>(config)),
						(std::istreambuf_iterator<char>()));
	nlohmann::json Configdata = nlohmann::json::parse(content);
	//==========================================================Device Type=======/
	APMInit.RC_Type = Configdata["Type_RC"].get<int>();
	APMInit.MPU9250_Type = Configdata["Type_MPU9250"].get<int>();
	APMInit.IMUFilter_Type = Configdata["Type_IMUFilter"].get<int>();
	APMInit.IMUMixFilter_Type = Configdata["Type_IMUMixFilter"].get<int>();

	APMInit.__RCDevice = Configdata["__RCDevice"].get<std::string>(),
	APMInit.__GPSDevice = Configdata["__GPSDevice"].get<std::string>(),
	APMInit.__FlowDevice = Configdata["__FlowDevice"].get<std::string>(),

	APMInit._IsGPSEnable = Configdata["_IsGPSEnable"].get<int>();
	APMInit._IsFlowEnable = Configdata["_IsFlowEnable"].get<int>();
	APMInit._IsMS5611Enable = Configdata["_IsMS5611Enable"].get<int>();
	APMInit._IsRCSafeEnable = Configdata["_IsRCSafeEnable"].get<int>();
	//==========================================================Controller cofig==/
	APMInit._flag_RC_ARM_PWM_Value = Configdata["_flag_RC_ARM_PWM_Value"].get<int>();
	APMInit._flag_RC_Min_PWM_Value = Configdata["_flag_RC_Min_PWM_Value"].get<int>();
	APMInit._flag_RC_Mid_PWM_Value = Configdata["_flag_RC_Mid_PWM_Value"].get<int>();
	APMInit._flag_RC_Max_PWM_Value = Configdata["_flag_RC_Max_PWM_Value"].get<int>();

	APMInit._flag_RCIsReserv__Roll = Configdata["_flag_RCIsReserv__Roll"].get<int>();
	APMInit._flag_RCIsReserv_Pitch = Configdata["_flag_RCIsReserv_Pitch"].get<int>();
	APMInit._flag_RCIsReserv___Yaw = Configdata["_flag_RCIsReserv___Yaw"].get<int>();
	//==========================================================ESC cofig=========/
	APMInit._flag_A1_Pin = Configdata["_flag_A1_Pin"].get<int>();
	APMInit._flag_A2_Pin = Configdata["_flag_A2_Pin"].get<int>();
	APMInit._flag_B1_Pin = Configdata["_flag_B1_Pin"].get<int>();
	APMInit._flag_B2_Pin = Configdata["_flag_B2_Pin"].get<int>();
	//==================================================================PID cofig==/
	APMInit._flag_PID_P_TAsix_Gain = Configdata["_flag_PID_P_TAsix_Gain"].get<float>();
	APMInit._flag_PID_P__Roll_Gain = Configdata["_flag_PID_P__Roll_Gain"].get<float>();
	APMInit._flag_PID_P_Pitch_Gain = Configdata["_flag_PID_P_Pitch_Gain"].get<float>();
	APMInit._flag_PID_P___Yaw_Gain = Configdata["_flag_PID_P___Yaw_Gain"].get<float>();
	APMInit._flag_PID_P_Alt_Gain = Configdata["_flag_PID_P_Alt_Gain"].get<float>();
	APMInit._flag_PID_P_GPS_Gain = Configdata["_flag_PID_P_GPS_Gain"].get<float>();

	APMInit._flag_PID_I__Roll_Gain = Configdata["_flag_PID_I__Roll_Gain"].get<float>();
	APMInit._flag_PID_I_Pitch_Gain = Configdata["_flag_PID_I_Pitch_Gain"].get<float>();
	APMInit._flag_PID_I___Yaw_Gain = Configdata["_flag_PID_I___Yaw_Gain"].get<float>();
	APMInit._flag_PID_I_Alt_Gain = Configdata["_flag_PID_I_Alt_Gain"].get<float>();
	APMInit._flag_PID_I__Roll_Max__Value = Configdata["_flag_PID_I__Roll_Max__Value"].get<float>();
	APMInit._flag_PID_I_Pitch_Max__Value = Configdata["_flag_PID_I_Pitch_Max__Value"].get<float>();
	APMInit._flag_PID_I___Yaw_Max__Value = Configdata["_flag_PID_I___Yaw_Max__Value"].get<float>();

	APMInit._flag_PID_D__Roll_Gain = Configdata["_flag_PID_D__Roll_Gain"].get<float>();
	APMInit._flag_PID_D_Pitch_Gain = Configdata["_flag_PID_D_Pitch_Gain"].get<float>();
	APMInit._flag_PID_D___Yaw_Gain = Configdata["_flag_PID_D___Yaw_Gain"].get<float>();
	APMInit._flag_PID_D_Alt_Gain = Configdata["_flag_PID_D_Alt_Gain"].get<float>();
	APMInit._flag_PID_D_GPS_Gain = Configdata["_flag_PID_D_GPS_Gain"].get<float>();

	APMInit._flag_PID_Hover_Throttle = Configdata["_flag_PID_Hover_Throttle"].get<float>();
	APMInit._flag_PID_Level_Max = Configdata["_flag_PID_Level_Max"].get<float>();
	APMInit._flag_PID_Alt_Level_Max = Configdata["_flag_PID_Alt_Level_Max"].get<float>();
	APMInit._flag_PID_GPS_Level_Max = Configdata["_flag_PID_GPS_Level_Max"].get<float>();
	//==============================================================Sensors cofig==/
	APMInit._flag_MPU9250_A_X_Cali = Configdata["_flag_MPU9250_A_X_Cali"].get<double>();
	APMInit._flag_MPU9250_A_Y_Cali = Configdata["_flag_MPU9250_A_Y_Cali"].get<double>();
	APMInit._flag_MPU9250_A_Z_Cali = Configdata["_flag_MPU9250_A_Z_Cali"].get<double>();
	APMInit._flag_Accel__Roll_Cali = Configdata["_flag_Accel__Roll_Cali"].get<double>();
	APMInit._flag_Accel_Pitch_Cali = Configdata["_flag_Accel_Pitch_Cali"].get<double>();

	APMInit._flag_MPU9250_Head_Asix = Configdata["_flag_MPU9250_Head_Asix"].get<double>();
	APMInit._flag_MPU9250_M_X_Offset = Configdata["_flag_MPU9250_M_X_Offset"].get<double>();
	APMInit._flag_MPU9250_M_Y_Offset = Configdata["_flag_MPU9250_M_Y_Offset"].get<double>();
	APMInit._flag_MPU9250_M_Z_Offset = Configdata["_flag_MPU9250_M_Z_Offset"].get<double>();
	APMInit._flag_MPU9250_M_Y_Scaler = Configdata["_flag_MPU9250_M_Y_Scaler"].get<double>();
	APMInit._flag_MPU9250_M_Z_Scaler = Configdata["_flag_MPU9250_M_Z_Scaler"].get<double>();

	APMInit._flag_QMC5883L_Head_Asix = Configdata["_flag_QMC5883L_Head_Asix"].get<double>();
	APMInit._flag_QMC5883L_M_X_Offset = Configdata["_flag_QMC5883L_M_X_Offset"].get<double>();
	APMInit._flag_QMC5883L_M_Y_Offset = Configdata["_flag_QMC5883L_M_Y_Offset"].get<double>();
	APMInit._flag_QMC5883L_M_Z_Offset = Configdata["_flag_QMC5883L_M_Z_Offset"].get<double>();
	APMInit._flag_QMC5883L_M_Y_Scaler = Configdata["_flag_QMC5883L_M_Y_Scaler"].get<double>();
	APMInit._flag_QMC5883L_M_Z_Scaler = Configdata["_flag_QMC5883L_M_Z_Scaler"].get<double>();
	//===============================================================Update cofig==/
	APMInit.IMU_Freqeuncy = Configdata["IMU_Freqeucy"].get<int>();
	APMInit.RXT_Freqeuncy = Configdata["RXT_Freqeucy"].get<int>();
	APMInit.ESC_Freqeuncy = Configdata["ESC_Freqeucy"].get<int>();
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