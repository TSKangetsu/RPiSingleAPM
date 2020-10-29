#include "./src/SingleAPM.hpp"
using namespace SingleAPMAPI;

int main(int argc, char *argv[])
{
	system("clear");
	int argvs;
	APMSettinngs setting;
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
			APM_Settle.RPiSingleAPMInit(setting);
			APM_Settle.IMUSensorsTaskReg();
			APM_Settle.ControllerTaskReg();
			APM_Settle.PositionTaskReg();
			APM_Settle.ESCUpdateTaskReg();
			APM_Settle.AltholdSensorsTaskReg();
			APM_Settle.TaskThreadBlock();
		}
		break;
		case 'c':
		{
			RPiSingleAPM APM_Settle;
			APM_Settle.RPiSingleAPMInit(setting);
			APM_Settle.APMCalibrator();
		}
		break;
		//--------------------------------------------------------------------------------//
		case 'h':
		{
			std::cout << "using ArgeMent: \n"
					  << " -r : starting AirController \n"
					  << " -c : starting CalibrationESC \n";
		}
		break;
			//--------------------------------------------------------------------------------//
		}
	}
}