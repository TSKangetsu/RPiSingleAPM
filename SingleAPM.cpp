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
 		std::cout << _uORB_REC_roll << " ";
		std::cout << _uORB_REC_pitch << " ";
		std::cout << _uORB_REC_yall << " ";
		std::cout << _uORB_REC_throttle << " ------>";

		std::cout << _uORB_B1_Speed << " ";
		std::cout << _uORB_A1_Speed << " ";
		std::cout << _uORB_A2_Speed << " ";
		std::cout << _uORB_B2_Speed << "----->";

		std::cout << _UNTest_Roll[0] << " ";
		std::cout << _UNTest_Roll[1] << "->";
		std::cout << _UNTest_Pitch[0] << " ";
		std::cout << _UNTest_Pitch[1] << "->";
		std::cout << _UNTest_Yall[0] << " ";
		std::cout << _UNTest_Yall[1] << "------------->\r";
		test.MotorUpdate();
		usleep(5000);
	}
}