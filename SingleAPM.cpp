#include "SingleAPM.hpp"

int main()
{
	long int timer;
	long int timer_end;
	Stablize_Mode test;
	test.ControlCalibration();
	test.SensorsGryoCalibration();
	while (true)
	{
		timer = micros();
		test.SensorsParse();
		test.ControlParse();
		test.AttitudeUpdate();
		test.MotorUpdate();
		//---------test-----------//
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
		std::cout << "timer : " << timer_end - timer << "\n";
		usleep(3990 - (timer_end - timer));
	}
}