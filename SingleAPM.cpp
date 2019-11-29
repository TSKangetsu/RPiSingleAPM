#include "SingleAPM.hpp"

int main()
{
	long int timer;
	long int timer_end;
	Stablize_Mode test;
	test.ESCCalibration();
	test.ControlCalibration();
	test.SensorsCalibration();
	std::thread controllerUORB([&] {
		while (true)
		{
			test.ControlParse();
			usleep(4000);
		}
		});
	std::thread AutoLevelingMain([&] {
		while (true)
		{
			timer = micros();
			test.SensorsParse();
			test.AttitudeUpdate();
			test.ESCUpdate();
			//---------test-----------//
			std::cout << test._uORB_Real_Pitch << " ";
			std::cout << test._uORB_Real__Roll << " ";
			std::cout << _uORB_RC__Safe << " ";
			std::cout << _uORB_B1_Speed << " ";
			std::cout << _uORB_A1_Speed << " ";
			std::cout << _uORB_A2_Speed << " ";
			std::cout << _uORB_B2_Speed << " ";
			//----------test----------//
			timer_end = micros();
			if ((timer_end - timer) > 4000)
			{
				std::cout << "[WARING!!!!] Frequency Sync error , Over 4ms !!!!! Dangours !!! Gryo Angle error !!!!";
				_flag_ForceFailed_Safe = true;
			}
			std::cout << "-->>timer : " << timer_end - timer << "___\r";
			usleep(3990 - (timer_end - timer));
		}
		});
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(3, &cpuset);
	int rc = pthread_setaffinity_np(AutoLevelingMain.native_handle(), sizeof(cpu_set_t), &cpuset);
	AutoLevelingMain.join();
}