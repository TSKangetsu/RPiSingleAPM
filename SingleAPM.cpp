#include "SingleAPM.hpp"

int main(int argc, char* argv[])
{
	int argvs;
	APMSafeStatus statusOut;
	APMSettinngs setting;
	setting.MPU9250_Type = APMS_MPU9250Type::MPU_Is_SPI;
	setting.RC_Type = APMS_RCType::RC_IS_SBUS;
	while ((argvs = getopt(argc, argv, "vrh")) != -1)
	{
		switch (argvs)
		{
		case 'v':
			std::cout << "[RPiSingleAPM] version 1.0.f Beta , Acess By TSKangetsu\n"
				<< "	checkout : https://github.com/TSKangetsu/RPiSingleAPM";
			break;
		case 'r':
		{
			RPiSingleAPM APM_Settle(setting);
			std::thread AutoLevelingMain([&] {
				while (true)
				{
					APM_Settle.SensorsParse();
					APM_Settle.ControlParse();
					APM_Settle.AttitudeUpdate();
					APM_Settle.SaftyChecking(statusOut);
					APM_Settle.ESCUpdate();
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