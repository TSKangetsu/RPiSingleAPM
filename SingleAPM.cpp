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
		std::cout << _uORB_Leveling_Pitch << "___";
		std::cout << _uORB_Leveling__Roll << "___\n";
		timer_end = clock();
		usleep(4000 - (timer_end - timer));
	}
}