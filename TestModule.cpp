#include <fstream>
#include "src/SingleAPM.hpp"
#include "json/APMJsonConfig.hpp"
#include "json/Drive_Json.hpp"
#include "UserImpTestModule.hpp"
using namespace SingleAPMAPI;

#ifndef GIT_COMMIT_HASH
#define GIT_COMMIT_HASH "0000000" // 0000000 means uninitialized
#endif

#define CONFIGDIR "/etc/APMconfig.json"

void configWrite(const char *configDir, const char *substr, const char *Target, double obj);
void configSettle(const char *configDir, const char *substr, APMSettinngs &APMInit);
void SignalCatch(int Signal);

int main(int argc, char *argv[])
{
	int argvs;
	double data[20] = {0};
	APMSettinngs setting;
	while ((argvs = getopt(argc, argv, "e:E:C:r:s:a:jh")) != -1)
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
			char cmd[50];
			sprintf(cmd, "mkdir -p %s", BlackBoxLogDir);
			std::cout << "[RPiSingleAPM] Create log dir: " << cmd << "\n";
			system(cmd);
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
		case 's':
		{
			FlightControllMain APM_Settle;
			configSettle(CONFIGDIR, optarg, setting);
			APM_Settle.RPiSingleAPMInit(setting);
			// Because of PiGPIO ,if you must handle Signal, should be call after RPiSingleAPMInit()
			std::signal(SIGINT, SignalCatch);
			std::signal(SIGTERM, SignalCatch);
			APM_Settle.ServoControllInit();
			//
			APM_Settle.RPiSingleAPMStartUp();
			APM_Settle.TaskThreadBlock();
			//
			APM_Settle.stopAndWaitForExit();
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
	APMInit = SingleAPMAPI::readConfigFromJson(Mas[substr]);
}

void configWrite(const char *configDir, const char *substr, const char *Target, double obj)
{
	std::ifstream config(configDir);
	std::string content((std::istreambuf_iterator<char>(config)),
						(std::istreambuf_iterator<char>()));
	nlohmann::json Configdata = nlohmann::json::parse(content);
	nlohmann::json subdata = Configdata[substr]["Sensor"];
	subdata[Target] = obj;
	Configdata[substr]["Sensor"] = subdata;
	std::ofstream configs;
	configs.open(configDir);
	configs << std::setw(4) << Configdata << std::endl;
	configs.close();
}

void SignalCatch(int Signal)
{
	SingleAPMAPI::SystemSignal = Signal;
};