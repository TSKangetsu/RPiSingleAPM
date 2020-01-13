#include "SingleAPM.hpp"
#include <iomanip>
using namespace SingleAPMAPI;

int main(int argc, char *argv[])
{
	system("clear");
	int argvs;
	int ALTDATA[3];
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
			APM_Settle.ESCCalibration();
#endif
		}
		break;
		case 'r':
		{
			RPiSingleAPM APM_Settle(setting);
			std::thread AltHoldModeMain([&] {
				while (true)
				{
					APM_Settle.AltHoldTransRead(ALTDATA, true);
				}
			});
			std::thread AutoLevelingMain([&] {
				while (true)
				{
					APM_Settle.SensorsParse();
					APM_Settle.ControlParse(APMChannelOut, APMChannelIn, true);
					APM_Settle.AttitudeUpdate();
					APM_Settle.SaftyChecking(statusOut);
					APM_Settle.ESCUpdate();
					APM_Settle.DebugOutPut(true);
					APM_Settle.ClockingTimer();
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