#include "SingleAPM.hpp"

int main()
{
	std::cout << (float)14 / (float)18;
	std::cout << "Test\n";
	Manaul_Mode test;
	std::thread ConRead([&]
		{
			std::cout << "Test Controller\n";
			while (true)
			{
				test.ControlRead();
				usleep(10000);
			}
		});
	while (true)
	{
		test.AttitudeUpdate();
		std::cout << _uORB_B1_Speed << " ";
		std::cout << _uORB_A1_Speed << " ";
		std::cout << _uORB_A2_Speed << " ";
		std::cout << _uORB_B2_Speed << "---->";

		std::cout << _Tmp_Prenset_B1 << " ";
		std::cout << _Tmp_Prenset_A1 << " ";
		std::cout << _Tmp_Prenset_A1 << " ";
		std::cout << _Tmp_Prenset_B2 << "---->";

		std::cout << _uORB_REC_roll << " ";
		std::cout << _uORB_REC_pitch << " ";
		std::cout << _uORB_REC_throttle << " ";
		std::cout << _uORB_REC_yall << "\r";

		test.MotorUpdate();
		usleep(10000);
	}
}