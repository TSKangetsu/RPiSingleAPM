#include "SingleAPM.hpp"

int main(int argc, char* argv[])
{
	int argvs;
	Stablize_Mode test;
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
			test.ESCCalibration();
			test.ControlCalibration();
			test.SensorsCalibration();
		}
		break;
		//--------------------------------------------------------------------------------//
		case 'r':
		{	
			std::thread controllerUORB([&] {
				while (true)
				{
					test.ControlParse();
				}
				});
			std::thread AutoLevelingMain([&] {
				long int timer;
				long int timer_end;
				while (true)
				{
					timer = micros();
					test.SensorsParse();
					test.AttitudeUpdate();
					test.ESCUpdate();
					timer_end = micros();
					test.DebugOuput();
					loopTime = timer_end - timer;
					delayMicroseconds(Update_Freq_Time - loopTime);
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