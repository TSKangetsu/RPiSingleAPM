#include "SingleAPM.hpp"

int main()
{
	long int timer;
	long int timer_end;
	Stablize_Mode test;
	test.SensorsGryoCalibration();
	while (true)
	{
		test.SensorsParse();
		if (test._uORB_MPU9250_G_X > 50000)
		{
			std::cout << test._uORB_MPU9250_G_X << " ____";
			std::cout << test._uORB_MPU9250_G_Y << " ____";
			std::cout << test._uORB_MPU9250_G_Z << " ____\n";
		}
	}
}