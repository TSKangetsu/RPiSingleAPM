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
		test.ControlRead();
		test.AttitudeUpdate();
		test.MotorUpdate();
		timer_end = clock();
		std::cout << "timer: " << timer_end - timer << "\n";
		usleep(4000 - (timer_end - timer));
	}
}