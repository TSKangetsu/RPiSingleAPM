#include "SingleAPM.hpp"

int main()
{
	long int timer;
	Stablize_Mode test;
	while (true)
	{
		test.SensorsParse();
		std::cout << test._uORB_Angel_Pitch << " ";
		std::cout << test._uORB_Angel__Roll << " ->\r";
	}
}