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
		timer_end = clock();
		usleep(4000 - (timer_end - timer));
	}
}