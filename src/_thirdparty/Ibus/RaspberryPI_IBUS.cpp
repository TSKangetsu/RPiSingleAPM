#include "src/RPiIBus.h"
#include <unistd.h>
#include <wiringPi.h>
int main()
{
	int ChannelsData[14];
	int sbusData[32];
	int lose;
	long int time;
	long int timee;

	Ibus Ibus("/dev/ttyS0");
	while (true)
	{
		time = micros();
		lose = Ibus.IbusRead(ChannelsData, 4000, 2);
		if (lose != -1)
		{
			for (size_t i = 0; i < 14; i++)
			{
				std::cout << ChannelsData[i] << " ";
			}
			timee = micros();
			std::cout << "T: " << timee - time;
			std::cout << " \n";
		}
	}
}
