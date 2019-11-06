#include "SingleAPM.hpp"

int main()
{
	Stablize_Mode test;
	while (true)
	{
		test.SensorsRead();
		usleep(5000);
	}
}