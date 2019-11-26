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
		std::cout << test._uORB_Real_Pitch << "___";
		std::cout << test._uORB_Real__Roll << "___";
		std::cout << _uORB_Leveling_Pitch << "___";
		std::cout << _uORB_Leveling__Roll << "___";
		timer_end = clock();
		std::cout << "timer: " << timer_end - timer << "\n";
		usleep(4000 - (timer_end - timer));
	}
}