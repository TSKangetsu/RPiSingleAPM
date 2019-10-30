#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <pca9685.h>

//_uORB_Sensors
int8_t _uORB_Sensors_X;
int8_t _uORB_Sensors_Y;
int8_t _uORB_SSensors_Z;
int8_t _uORB_SSensors_A_X;
int8_t _uORB_SSensors_A_Y;
int8_t _uORB_SSensors_A_Z;
//REC_Reading_Yall_Pitch_Yoll_Throttle_Level
int8_t _uORB_REC_Yall_Level;
int8_t _uORB_REC_Pitch_Level;
int8_t _uORB_REC_Yoll_Level;
int8_t _uORB_REC_Throttle_Level;
//_uORB_Yall_Pitch_Yoll_Throttle_True_Level
int8_t _uORB_Yall_Level;
int8_t _uORB_Pitch_Level;
int8_t _uORB_Yoll_Level;
int8_t _uORB_Throttle_Level;
//_uORB_PWM_Level
int8_t _uORB_PWM_Level_A1;
int8_t _uORB_PWM_Level_A2;
int8_t _uORB_PWM_Level_B1;
int8_t _uORB_PWM_Level_B2;
//Flags
int PCA9658_fd;
int PCA9685_PinBase = 65;
int PCA9685_Address = 0x40;
int PWM_Freq = 490;
bool All_startUp;
bool All_Stop;

//-----------------------------//
class REC_PWM_Read
{

};

//Manaull Mode
class Manaul_Mode
{

	Manaul_Mode()
	{
		PCA9658_fd = pca9685Setup(PCA9685_PinBase, PCA9685_Address, PWM_Freq);
	}

	inline void Readconctroller()
	{

	}

	inline void MotorUpdate()
	{
		if (All_startUp && !All_Stop)
		{
			//pca9685PWMWrite(PCA9658_fd , 0 , );
		}
		if (All_Stop && !All_startUp)
		{

		}
		else
		{

		}
	}
};
//Manaull Mode