#include "./src/SingleAPM.hpp"
using namespace SingleAPMAPI;

int main(int argc, char* argv[])
{
	system("clear");
	int argvs;
	APMSettinngs setting;
	UserControlInputType UserInput;
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
		}
		break;
		case 'r':
		{
			RPiSingleAPM APM_Settle;
			UserInput._RawPre__Roll = 1;
			UserInput._RawPre_Pitch = -1;
			UserInput._RawPre___Yaw = 0;
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
					APM_Settle.ControlUserInput(false, UserInput);
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