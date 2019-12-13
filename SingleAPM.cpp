#include "SingleAPM.hpp"

int main(int argc, char* argv[])
{
	int argvs;
	APMSafeStatus statusOut;
	RPiSingelAPM APM_Settle;
	while ((argvs = getopt(argc, argv, "vcerh")) != -1)
	{
		switch (argvs)
		{
		case 'v':
			std::cout << "[RPiSingleAPM] version 1.0.f Beta , Acess By TSKangetsu\n"
				<< "	checkout : https://github.com/TSKangetsu/RPiSingleAPM";
			break;
			//--------------------------------------------------------------------------------//
		case 'c':
		{
			APM_Settle.ControlCalibration();
			APM_Settle.SensorsCalibration();
		}
		break;
		case 'e':
		{
			APM_Settle.ESCCalibration();
		}
		break;
		//--------------------------------------------------------------------------------//
		case 'r':
		{
			std::thread AutoLevelingMain([&] {
				while (true)
				{
					APM_Settle.UpdateTimer_Start = micros();
					APM_Settle.SensorsParse();
					APM_Settle.ControlParse();
					APM_Settle.AttitudeUpdate();
					APM_Settle.SaftyChecking(statusOut);
					APM_Settle.ESCUpdate();
					APM_Settle.UpdateTimer_End = micros();
					APM_Settle.Attitude_loopTime = APM_Settle.UpdateTimer_End - APM_Settle.UpdateTimer_Start;
					std::cout << APM_Settle.Attitude_loopTime << " \n";
					delayMicroseconds(APM_Settle.Update_Freq_Time - APM_Settle.Attitude_loopTime);
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