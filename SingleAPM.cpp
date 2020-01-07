#include "SingleAPM.hpp"
#include <iomanip>
using namespace SingleAPMAPI;

int main(int argc, char* argv[])
{
	system("clear");
	int argvs;
	int APMChannelOut[16];
	int APMChannelIn[16];
	APMSafeStatus statusOut;
	APMSettinngs setting;
	while ((argvs = getopt(argc, argv, "vcrh")) != -1)
	{
		switch (argvs)
		{
		case 'v':
			std::cout << "[RPiSingleAPM] version 1.0.f Beta , Acess By TSKangetsu\n"
				<< "	checkout : https://github.com/TSKangetsu/RPiSingleAPM \n";
			break;
		case 'c':
		{
			RPiSingleAPM APM_Settle(setting);
#ifdef USINGJSON
			APM_Settle.RCCalibration();
			APM_Settle.SensorsCalibration();
#endif
		}
		break;
		case 'r':
		{
			RPiSingleAPM APM_Settle(setting);
			std::thread AutoLevelingMain([&] {
				while (true)
				{
					APM_Settle.SensorsParse();
					APM_Settle.ControlParse(APMChannelOut ,APMChannelIn , true);
					APM_Settle.AttitudeUpdate();
					APM_Settle.SaftyChecking(statusOut);
					APM_Settle.ESCUpdate();
					APM_Settle.DebugOutPut(false);
					APM_Settle.ClockingTimer();
					std::cout << "\033[150A";
					std::cout << "\033[K";
					std::cout << "RCINFO:   " << "\n";
					for (size_t i = 0; i < 16; i++)
					{
						std::cout << "Channel " << i << ": " << APMChannelOut[i] << std::setw(10) << std::setfill(' ') << "\n";
					}
					std::cout << "\n";
					std::cout << "ForceFailedSafe  " << statusOut.ForceFailedSafe << "\n";
					std::cout << "Is_AngelOutLimit " << statusOut.Is_AngelOutLimit << "\n";
					std::cout << "Is_RCDisconnect  " << statusOut.Is_RCDisconnect << "\n";
					std::cout << "Is_RCErrorInput  " << statusOut.Is_RCErrorInput << "\n";
					std::cout << "Is_SyncTimeOut   " << statusOut.Is_SyncTimeOut << "\n";
					std::cout << "SafyError        " << statusOut.SafyError << "\n";
					std::cout << "SyncTime         " << statusOut.SyncTime << "                    \n\n";
				}
				});
			cpu_set_t cpuset;
			CPU_ZERO(&cpuset);
			CPU_SET(3, &cpuset);
			int rc = pthread_setaffinity_np(AutoLevelingMain.native_handle(), sizeof(cpu_set_t), &cpuset);
			AutoLevelingMain.join();
		}
		break;
		//--------------------------------------------------------------------------------//
		case 'h':
		{
			std::cout << "using ArgeMent: \n"
				<< " -c : starting calibration \n"
				<< " -r : starting AirController \n";
		}
		break;
		//--------------------------------------------------------------------------------//
		}
	}

}