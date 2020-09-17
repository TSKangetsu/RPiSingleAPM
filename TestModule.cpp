#include "./src/SingleAPM.hpp"
using namespace SingleAPMAPI;

int main(int argc, char *argv[])
{
	system("clear");
	int argvs;
	APMSettinngs setting;
	UserControlInputType UserInput;
	while ((argvs = getopt(argc, argv, "vtcrh")) != -1)
	{
		switch (argvs)
		{
		case 'v':
			std::cout << "[RPiSingleAPM] version 1.0.f Beta , Acess By TSKangetsu\n"
					  << "	checkout : https://github.com/TSKangetsu/RPiSingleAPM \n";
			break;
		case 'r':
		{
			RPiSingleAPM APM_Settle;
			UserInput._RawPre__Roll = 1;
			UserInput._RawPre_Pitch = -1;
			UserInput._RawPre___Yaw = 0;
			APM_Settle.RPiSingleAPMInit(setting);
			APM_Settle.IMUSensorsTaskReg();
			APM_Settle.ControllerTaskReg();
			APM_Settle.ESCUpdateTaskReg();
			APM_Settle.AltholdSensorsTaskReg();
			APM_Settle.TaskThreadBlock();
		}
		break;
		//--------------------------------------------------------------------------------//
		case 'h':
		{
			std::cout << "using ArgeMent: \n"
					  << " -c : starting calibration \n"
					  << " -r : starting AirController \n";
		}
		break;
			//--------------------------------------------------------------------------------//
		}
	}
}