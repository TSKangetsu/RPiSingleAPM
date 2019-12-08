#include "SingleAPM.hpp"

int main(int argc, char* argv[])
{
	int argvs;
	RPiSingelAPM APM_Settle;
	while ((argvs = getopt(argc, argv, "vcrh")) != -1)
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
			APM_Settle.ESCCalibration();
			APM_Settle.ControlCalibration();
			APM_Settle.SensorsCalibration();
		}
		break;
		//--------------------------------------------------------------------------------//
		case 'r':
		{	
			std::thread controllerUORB([&] {
				while (true)
				{
					APM_Settle.ControlParse();
					delayMicroseconds(2000);
				}
				});
			std::thread AutoLevelingMain([&] {
				long int timer;
				long int timer_end;
				while (true)
				{
					timer = micros();
					APM_Settle.SensorsParse();
					APM_Settle.AttitudeUpdate();
					APM_Settle.SaftyChecking();
					APM_Settle.ESCUpdate();
					APM_Settle.Debug();
					timer_end = micros();
					APM_Settle.Attitude_loopTime = timer_end - timer;
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