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
			test.ConfigReader();
			std::thread controllerUORB([&] {
				long int controller_timer;
				long int controller_timer_end;
				unsigned long int controller_loopTime;
				while (true)
				{
					controller_timer = micros();
					test.ControlParse();
					controller_timer_end = micros();
					controller_loopTime = controller_timer_end - controller_timer;
					usleep(4000 - controller_loopTime);
				}
				});
			std::thread AutoLevelingMain([&] {
				long int timer;
				long int timer_end;
				long int loopTime;
				while (true)
				{
					timer = micros();
					test.SensorsParse();
					test.AttitudeUpdate();
					test.ESCUpdate();
					timer_end = micros();
					loopTime = timer_end - timer;
					if (loopTime > 4000)
					{
						std::cout << "[WARING!!!!] Frequency Sync error , Over 4ms !!!!! Dangours !!! Gryo Angle error !!!!";
						_flag_ForceFailed_Safe = true;
					}
					test.DebugOuput();
					delayMicroseconds(4000 - loopTime);
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