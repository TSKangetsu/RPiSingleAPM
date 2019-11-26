#include "SingleAPM.hpp"

int main()
{
	long int timer;
	long int timer_end;
	Stablize_Mode test;
	test.SensorsGryoCalibration();
	while (true)
	{
		timer = clock();
		test.SensorsParse();
		test.AttitudeUpdate();
		std::cout << test._uORB_MPU9250_A_X << "___";
		std::cout << test._uORB_MPU9250_A_Y << "___";
		std::cout << _uORB_Leveling_Pitch << "___";
		std::cout << _uORB_Leveling_Pitch << "___";
		timer_end = clock();
		std::cout << "timer: " << timer_end - timer << "\r";
		usleep(4000 - (timer_end - timer));
	}
}