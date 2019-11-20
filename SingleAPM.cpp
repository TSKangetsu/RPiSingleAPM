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
		std::cout << test._uORB_Accel_Pitch << " ";
		std::cout << test._uORB_Accel__Roll << "____";
		std::cout << test._uORB_Real_Pitch << " ";
		std::cout << test._uORB_Real__Roll << "____";
		timer_end = clock();
		std::cout << timer_end - timer  << "  --\r";
	}
}