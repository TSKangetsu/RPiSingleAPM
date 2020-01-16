#include "SingleAPM.hpp"
using namespace SingleAPMAPI;

int main(int argc, char* argv[])
{
	system("clear");
	int argvs;
	APMSettinngs setting;
	while ((argvs = getopt(argc, argv, "vtcrh")) != -1)
	{
		switch (argvs)
		{
		case 'v':
			std::cout << "[RPiSingleAPM] version 1.0.f Beta , Acess By TSKangetsu\n"
				<< "	checkout : https://github.com/TSKangetsu/RPiSingleAPM \n";
			break;
		case 't':
		{
			RPiSingleAPM APM_Settle;
			APM_Settle.RPiSingleAPMInit(setting);
			std::thread GryoUpdate([&] {
				int timers;
				int timere;
				while (true)
				{
					APM_Settle.IMUSensorsParse();
					APM_Settle.SaftyChecking();
					APM_Settle.ClockingTimer();
				}
				});
			std::thread ESCUpdate([&] {
				while (true)
				{
					APM_Settle.ESCUpdate();
					usleep(6000);
				}
				});				
			std::thread ControlUpdate([&] {
				while (true)
				{
					APM_Settle.ControlParse();
					APM_Settle.AttitudeUpdate();
					usleep(4000);
				}
				});
			std::thread OutputUpdate([&] {
				while (true)
				{
					APM_Settle.DebugOutPut();
					usleep(8000);
				}
				});
			std::thread AltHoldUpdate([&] {
				while (true)
				{
					APM_Settle.AltholdSensorsParse();
					usleep(4000);
				}
				});
			cpu_set_t cpuset;
			CPU_ZERO(&cpuset);
			CPU_SET(3, &cpuset);
			int rc = pthread_setaffinity_np(GryoUpdate.native_handle(), sizeof(cpu_set_t), &cpuset);
			int rc2 = pthread_setaffinity_np(ESCUpdate.native_handle(), sizeof(cpu_set_t), &cpuset);
			int rc3 = pthread_setaffinity_np(OutputUpdate.native_handle(), sizeof(cpu_set_t), &cpuset);
			int rc4 = pthread_setaffinity_np(AltHoldUpdate.native_handle(), sizeof(cpu_set_t), &cpuset);
			int rc5 = pthread_setaffinity_np(ControlUpdate.native_handle(), sizeof(cpu_set_t), &cpuset);
			GryoUpdate.join();
		}
		break;
		case 'r':
		{
			RPiSingleAPM APM_Settle;
			APM_Settle.RPiSingleAPMInit(setting);
			std::thread AltHoldModeMain([&] {
				while (true)
				{
					APM_Settle.AltholdSensorsParse();
				}
				});
			std::thread AutoLevelingMain([&] {
				while (true)
				{
					APM_Settle.IMUSensorsParse();
					APM_Settle.ControlParse();
					APM_Settle.AttitudeUpdate();
					APM_Settle.SaftyChecking();
					APM_Settle.ESCUpdate();
					APM_Settle.DebugOutPut();
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