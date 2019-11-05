#include "SingleAPM.hpp"

int main()
{
	clock_t end;
	clock_t start;
	bool fuckdebug;
	Manaul_Mode test;
	std::thread ConRead([&]
		{
			while (true)
			{
				test.ControlRead();
				usleep(5000);
			}
		});
	while (true)
	{
		test.AttitudeUpdate_Test();
		test.MotorUpdate();
		usleep(5000);
	}
}