#include "SingleAPM.hpp"

void SingleAPMAPI::RPiSingleAPM::RPiSingleAPMInit(APMSettinngs APMInit)
{
	wiringPiSetupSys();
	piHiPri(99);

	AF.RC_Lose_Clocking = 0;
	AF._flag_MPU9250_first_StartUp = true;
	AF._flag_MS5611_firstStartUp = true;
	AF._flag_ESC_ARMED = true;
	AF._flag_ClockingTime_Error = false;
	AF._flag_AlthHold_Enable = false;
	ConfigReader(APMInit);

	if (DF.PCA9658_fd == -1)
		DF.PCA9658_fd = pca9685Setup(DF.PCA9685_PinBase, DF.PCA9685_Address, DF.PWM_Freq);

	if (SF.MPU9250_Type == MPUIsI2c)
	{
		DF.MPU9250_fd = wiringPiI2CSetup(DF.MPU9250_ADDR);
		wiringPiI2CWriteReg8(DF.MPU9250_fd, 107, 0x00); //reset
		wiringPiI2CWriteReg8(DF.MPU9250_fd, 28, 0x08);	//Accel
		wiringPiI2CWriteReg8(DF.MPU9250_fd, 27, 0x08);	//Gryo
		wiringPiI2CWriteReg8(DF.MPU9250_fd, 26, 0x03);	//config
	}
	else if (SF.MPU9250_Type == MPUIsSpi)
	{
		DF.MPU9250_fd = wiringPiSPISetup(DF.MPU9250_SPI_Channel, DF.MPU9250_SPI_Freq);
		SF._Tmp_MPU9250_SPI_Config[0] = 0x6b;
		SF._Tmp_MPU9250_SPI_Config[1] = 0x00;
		wiringPiSPIDataRW(1, SF._Tmp_MPU9250_SPI_Config, 2); //reset
		SF._Tmp_MPU9250_SPI_Config[0] = 0x1c;
		SF._Tmp_MPU9250_SPI_Config[1] = 0x08;
		wiringPiSPIDataRW(1, SF._Tmp_MPU9250_SPI_Config, 2); // Accel
		SF._Tmp_MPU9250_SPI_Config[0] = 0x1b;
		SF._Tmp_MPU9250_SPI_Config[1] = 0x08;
		wiringPiSPIDataRW(1, SF._Tmp_MPU9250_SPI_Config, 2); // Gryo
		SF._Tmp_MPU9250_SPI_Config[0] = 0x1a;
		SF._Tmp_MPU9250_SPI_Config[1] = 0x03;
		wiringPiSPIDataRW(1, SF._Tmp_MPU9250_SPI_Config, 2); //config
	}

	if (SF.IMUMixFilter_Type == MixFilterType_Kalman)
	{
		Kal_Pitch = new Kalman();
		Kal__Roll = new Kalman();
	}

	if (RF.RC_Type == RCIsIbus)
	{
		IbusInit = new Ibus("/dev/ttyAMA0");
	}
	else if (RF.RC_Type == RCIsSbus)
	{
		SbusInit = new Sbus("/dev/ttyAMA0", SbusMode::Normal);
	}

	MS5611S = new MS5611();
	MS5611S->MS5611Init();
	MS5611S->LocalPressureSetter(SF._flag_MS5611_LocalPressure);

	//GryoCali()
	{
		SF._flag_MPU9250_G_X_Cali = 0;
		SF._flag_MPU9250_G_Y_Cali = 0;
		SF._flag_MPU9250_G_Z_Cali = 0;
		for (int cali_count = 0; cali_count < 2000; cali_count++)
		{
			IMUSensorsDataRead();
			SF._flag_MPU9250_G_X_Cali += SF._uORB_MPU9250_G_X;
			SF._flag_MPU9250_G_Y_Cali += SF._uORB_MPU9250_G_Y;
			SF._flag_MPU9250_G_Z_Cali += SF._uORB_MPU9250_G_Z;
			usleep(3);
		}
		SF._flag_MPU9250_G_X_Cali = SF._flag_MPU9250_G_X_Cali / 2000;
		SF._flag_MPU9250_G_Y_Cali = SF._flag_MPU9250_G_Y_Cali / 2000;
		SF._flag_MPU9250_G_Z_Cali = SF._flag_MPU9250_G_Z_Cali / 2000;
	}
}

void SingleAPMAPI::RPiSingleAPM::IMUSensorsTaskReg()
{
	TF.IMUTask = new std::thread([&] {
		while (true)
		{
			TF._Tmp_IMUThreadTimeStart = micros();
			TF._Tmp_IMUThreadTimeNext = TF._Tmp_IMUThreadTimeStart - TF._Tmp_IMUThreadTimeEnd;

			IMUSensorsDataRead();
			//Gryo----------------------------------------------------------------------//
			SF._uORB_MPU9250_G_X -= SF._flag_MPU9250_G_X_Cali;
			SF._uORB_MPU9250_G_Y -= SF._flag_MPU9250_G_Y_Cali;
			SF._uORB_MPU9250_G_Z -= SF._flag_MPU9250_G_Z_Cali;
			IMUGryoFilter(SF._uORB_MPU9250_G_X, SF._uORB_MPU9250_G_Fixed_X, SF._Tmp_Gryo_filer_Input_Quene_X, SF._Tmp_Gryo_filer_Output_Quene_X, SF.IMUFilter_Type);
			IMUGryoFilter(SF._uORB_MPU9250_G_Y, SF._uORB_MPU9250_G_Fixed_Y, SF._Tmp_Gryo_filer_Input_Quene_Y, SF._Tmp_Gryo_filer_Output_Quene_Y, SF.IMUFilter_Type);
			IMUGryoFilter(SF._uORB_MPU9250_G_Z, SF._uORB_MPU9250_G_Fixed_Z, SF._Tmp_Gryo_filer_Input_Quene_Z, SF._Tmp_Gryo_filer_Output_Quene_Z, SF.IMUFilter_Type);
			SF._Tmp_Gryo_RTSpeed_Pitch = (SF._uORB_MPU9250_G_Fixed_X / DF._flag_MPU9250_LSB);
			SF._Tmp_Gryo_RTSpeed__Roll = (SF._uORB_MPU9250_G_Fixed_Y / DF._flag_MPU9250_LSB);
			SF._uORB_Real_Pitch += SF._Tmp_Gryo_RTSpeed_Pitch / TF._flag_IMUThreadFreq;
			SF._uORB_Real__Roll += SF._Tmp_Gryo_RTSpeed__Roll / TF._flag_IMUThreadFreq;
			//GryoTrue------------------------------------------------------------------//
			SF._uORB_Gryo__Roll = (SF._uORB_Gryo__Roll * 0.7) + ((SF._uORB_MPU9250_G_Fixed_Y / DF._flag_MPU9250_LSB) * 0.3);
			SF._uORB_Gryo_Pitch = (SF._uORB_Gryo_Pitch * 0.7) + ((SF._uORB_MPU9250_G_Fixed_X / DF._flag_MPU9250_LSB) * 0.3);
			SF._uORB_Gryo___Yaw = (SF._uORB_Gryo___Yaw * 0.7) + ((SF._uORB_MPU9250_G_Fixed_Z / DF._flag_MPU9250_LSB) * 0.3);
			//ACCEL---------------------------------------------------------------------//
			SF._Tmp_IMU_Accel_Vector = sqrt((SF._uORB_MPU9250_A_X * SF._uORB_MPU9250_A_X) + (SF._uORB_MPU9250_A_Y * SF._uORB_MPU9250_A_Y) + (SF._uORB_MPU9250_A_Z * SF._uORB_MPU9250_A_Z));
			if (abs(SF._uORB_MPU9250_A_X) < SF._Tmp_IMU_Accel_Vector)
				SF._uORB_Accel__Roll = asin((float)SF._uORB_MPU9250_A_X / SF._Tmp_IMU_Accel_Vector) * -57.296;
			if (abs(SF._uORB_MPU9250_A_Y) < SF._Tmp_IMU_Accel_Vector)
				SF._uORB_Accel_Pitch = asin((float)SF._uORB_MPU9250_A_Y / SF._Tmp_IMU_Accel_Vector) * 57.296;
			SF._uORB_Accel__Roll -= SF._flag_Accel__Roll_Cali;
			SF._uORB_Accel_Pitch -= SF._flag_Accel_Pitch_Cali;
			//Gryo_MIX_ACCEL------------------------------------------------------------//
			if (!AF._flag_MPU9250_first_StartUp)
			{
				IMUMixFilter(Kal_Pitch, SF._uORB_Real_Pitch, SF._uORB_Accel_Pitch, SF._Tmp_Gryo_RTSpeed_Pitch, SF._uORB_Real_Pitch, SF.IMUMixFilter_Type);
				IMUMixFilter(Kal__Roll, SF._uORB_Real__Roll, SF._uORB_Accel__Roll, SF._Tmp_Gryo_RTSpeed__Roll, SF._uORB_Real__Roll, SF.IMUMixFilter_Type);
			}
			else
			{
				if (SF.IMUMixFilter_Type == MixFilterType_Kalman)
				{
					Kal_Pitch->setAngle(SF._uORB_Accel_Pitch);
					Kal__Roll->setAngle(SF._uORB_Accel__Roll);
				}
				SF._uORB_Real_Pitch = SF._uORB_Accel_Pitch;
				SF._uORB_Real__Roll = SF._uORB_Accel__Roll;
				AF._flag_MPU9250_first_StartUp = false;
			}
			SF._uORB_Real_Pitch -= SF._uORB_Real__Roll * sin((SF._uORB_MPU9250_G_Fixed_Z / TF._flag_IMUThreadFreq / DF._flag_MPU9250_LSB) * (3.14 / 180));
			SF._uORB_Real__Roll += SF._uORB_Real_Pitch * sin((SF._uORB_MPU9250_G_Fixed_Z / TF._flag_IMUThreadFreq / DF._flag_MPU9250_LSB) * (3.14 / 180));

			TF._Tmp_IMUThreadTimeEnd = micros();
			TF._Tmp_IMUThreadTimeLoop = TF._Tmp_IMUThreadTimeEnd - TF._Tmp_IMUThreadTimeStart;
			if (TF._Tmp_IMUThreadTimeLoop + TF._Tmp_IMUThreadTimeNext > TF._flag_IMUThreadTimeMax | TF._Tmp_IMUThreadTimeNext < 0)
			{
				usleep(50);
				AF._flag_ClockingTime_Error = true;
			}
			else
			{
				usleep(TF._flag_IMUThreadTimeMax - TF._Tmp_IMUThreadTimeLoop - TF._Tmp_IMUThreadTimeNext);
			}
			if (TF._Tmp_IMUThreadTimeLoop + TF._Tmp_IMUThreadTimeNext > TF._Tmp_IMUThreadError)
			{
				TF._Tmp_IMUThreadError = TF._Tmp_IMUThreadTimeLoop;
			}
			TF._Tmp_IMUThreadTimeEnd = micros();

			AttitudeUpdateTask();
		}
	});
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(3, &cpuset);
	int rc = pthread_setaffinity_np(TF.IMUTask->native_handle(), sizeof(cpu_set_t), &cpuset);
}

void SingleAPMAPI::RPiSingleAPM::AltholdSensorsTaskReg()
{
	TF.ALTTask = new std::thread([&] {
		while (true)
		{
			TF._Tmp_ALTThreadTimeStart = micros();
			TF._Tmp_ALTThreadTimeNext = TF._Tmp_ALTThreadTimeStart - TF._Tmp_ALTThreadTimeEnd;

			SF._uORB_MS5611_Last_Value_AltMeter = SF._uORB_MS5611_AltMeter;
			MS5611S->MS5611PreReader(SF._Tmp_MS5611_Data);
			SF._uORB_MS5611_Pressure = SF._Tmp_MS5611_Data[0];
			SF._uORB_MS5611_AltMeter = SF._Tmp_MS5611_Data[1];
			// SF._uORB_MS5611_ClimbeRate =
			// 	((SF._uORB_MS5611_AltMeter - SF._uORB_MS5611_Last_Value_AltMeter) * ((double)TF._Tmp_ALTThreadTimeLoop / 1000000.f));
			SF._uORB_MS5611_ClimbeRate = (int)(100000 * (SF._uORB_MS5611_AltMeter - SF._uORB_MS5611_Last_Value_AltMeter) / TF._Tmp_ALTThreadTimeLoop);

			TF._Tmp_ALTThreadTimeEnd = micros();
			TF._Tmp_ALTThreadTimeLoop = TF._Tmp_ALTThreadTimeEnd - TF._Tmp_ALTThreadTimeStart;
			if (TF._Tmp_ALTThreadTimeLoop + TF._Tmp_ALTThreadTimeNext > TF._Tmp_ALTThreadError)
			{
				TF._Tmp_ALTThreadError = TF._Tmp_ALTThreadTimeLoop;
			}
		}
	});
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(3, &cpuset);
	int rc = pthread_setaffinity_np(TF.ALTTask->native_handle(), sizeof(cpu_set_t), &cpuset);
}

void SingleAPMAPI::RPiSingleAPM::ControlUserInput(bool EnableUserInput, UserControlInputType UserInput)
{
	if (UserInput._RawPre__Roll > 0)
		RF._Tmp_UserInput__Roll = (RF._flag_RC_Max_PWM_Value - RF._flag_RC_Mid_PWM_Value) * UserInput._RawPre__Roll / 2 * RF._flag_RCIsReserv_Pitch;
	else if (UserInput._RawPre__Roll <= 0)
		RF._Tmp_UserInput__Roll = (RF._flag_RC_Min_PWM_Value - RF._flag_RC_Mid_PWM_Value) * UserInput._RawPre__Roll / 2 * RF._flag_RCIsReserv_Pitch;

	if (UserInput._RawPre_Pitch > 0)
		RF._Tmp_UserInput_Pitch = (RF._flag_RC_Max_PWM_Value - RF._flag_RC_Mid_PWM_Value) * UserInput._RawPre_Pitch / 2 * RF._flag_RCIsReserv__Roll;
	else if (UserInput._RawPre_Pitch <= 0)
		RF._Tmp_UserInput_Pitch = (RF._flag_RC_Min_PWM_Value - RF._flag_RC_Mid_PWM_Value) * UserInput._RawPre_Pitch / 2 * RF._flag_RCIsReserv__Roll;

	if (UserInput._RawPre___Yaw > 0)
		RF._Tmp_UserInput___Yaw = (RF._flag_RC_Max_PWM_Value - RF._flag_RC_Mid_PWM_Value) * UserInput._RawPre___Yaw / 2 * RF._flag_RCIsReserv___Yaw;
	else if (UserInput._RawPre___Yaw <= 0)
		RF._Tmp_UserInput___Yaw = (RF._flag_RC_Min_PWM_Value - RF._flag_RC_Mid_PWM_Value) * UserInput._RawPre___Yaw / 2 * RF._flag_RCIsReserv___Yaw;
	AF._flag_UserInput_Enable = EnableUserInput;
}

void SingleAPMAPI::RPiSingleAPM::ControllerTaskReg()
{
	TF.RXTask = new std::thread([&] {
		while (true)
		{
			TF._Tmp_RXTThreadTimeStart = micros();
			TF._Tmp_RXTThreadTimeNext = TF._Tmp_RXTThreadTimeStart - TF._Tmp_RXTThreadTimeEnd;

			if (RF.RC_Type == RCIsSbus)
			{
				if (SbusInit->SbusRead(RF._Tmp_RC_Data, 0, 1) != -1)
				{
					for (size_t i = 0; i < 16; i++)
					{
						RF._uORB_RC_Channel_PWM[i] = RF._Tmp_RC_Data[i];
					}
					AF._flag_RC_Disconnected = false;
				}
				else
				{
					AF._flag_RC_Disconnected = true;
				}
			}
			else if (RF.RC_Type == RCIsIbus)
			{
				if (IbusInit->IbusRead(RF._Tmp_RC_Data, 0, 1) != -1)
				{
					for (size_t i = 0; i < 16; i++)
					{
						RF._uORB_RC_Channel_PWM[i] = RF._Tmp_RC_Data[i];
					}
					AF._flag_RC_Disconnected = false;
				}
				else
				{
					AF._flag_RC_Disconnected = true;
				}
			}

			if (!AF._flag_UserInput_Enable)
			{
				if (RF._uORB_RC_Channel_PWM[0] < RF._flag_RC_Mid_PWM_Value + 10 && RF._uORB_RC_Channel_PWM[0] > RF._flag_RC_Mid_PWM_Value - 10)
					RF._uORB_RC_Out__Roll = 0;
				else
					RF._uORB_RC_Out__Roll = (RF._uORB_RC_Channel_PWM[0] - RF._flag_RC_Mid_PWM_Value) / 2 * RF._flag_RCIsReserv__Roll;
				//
				if (RF._uORB_RC_Channel_PWM[1] < RF._flag_RC_Mid_PWM_Value + 10 && RF._uORB_RC_Channel_PWM[1] > RF._flag_RC_Mid_PWM_Value - 10)
					RF._uORB_RC_Out_Pitch = 0;
				else
					RF._uORB_RC_Out_Pitch = (RF._uORB_RC_Channel_PWM[1] - RF._flag_RC_Mid_PWM_Value) / 2 * RF._flag_RCIsReserv_Pitch;
				//
				if (RF._uORB_RC_Channel_PWM[3] < RF._flag_RC_Mid_PWM_Value + 10 && RF._uORB_RC_Channel_PWM[3] > RF._flag_RC_Mid_PWM_Value - 10)
					RF._uORB_RC_Out___Yaw = 0;
				else
					RF._uORB_RC_Out___Yaw = (RF._uORB_RC_Channel_PWM[3] - RF._flag_RC_Mid_PWM_Value) / 2 * RF._flag_RCIsReserv___Yaw;
			}
			else
			{
				RF._uORB_RC_Out_Pitch = RF._Tmp_UserInput_Pitch;
				RF._uORB_RC_Out__Roll = RF._Tmp_UserInput__Roll;
				RF._uORB_RC_Out___Yaw = RF._Tmp_UserInput___Yaw;
				AF._flag_UserInput_Enable = false;
			}
			//
			RF._uORB_RC_Out_Throttle = RF._uORB_RC_Channel_PWM[2];
			//
			RF._uORB_RC_Out___ARM = RF._uORB_RC_Channel_PWM[4];

			TF._Tmp_RXTThreadTimeEnd = micros();
			TF._Tmp_RXTThreadTimeLoop = TF._Tmp_RXTThreadTimeEnd - TF._Tmp_RXTThreadTimeStart;
			if (TF._Tmp_RXTThreadTimeLoop + TF._Tmp_RXTThreadTimeNext > TF._flag_RXTThreadTimeMax | TF._Tmp_RXTThreadTimeNext < 0)
			{
				usleep(50);
				AF._flag_ClockingTime_Error = true;
			}
			else
			{
				usleep(TF._flag_RXTThreadTimeMax - TF._Tmp_RXTThreadTimeLoop - TF._Tmp_RXTThreadTimeNext);
			}
			if (TF._Tmp_RXTThreadTimeLoop + TF._Tmp_RXTThreadTimeNext > TF._Tmp_RXTThreadError)
			{
				TF._Tmp_RXTThreadError = TF._Tmp_RXTThreadTimeLoop;
			}
			TF._Tmp_RXTThreadTimeEnd = micros();
		}
	});
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(3, &cpuset);
	int rc = pthread_setaffinity_np(TF.RXTask->native_handle(), sizeof(cpu_set_t), &cpuset);
}

void SingleAPMAPI::RPiSingleAPM::AttitudeUpdateTask()
{
	//Roll PID Mix
	PF._uORB_PID__Roll_Input = SF._uORB_Gryo__Roll + SF._uORB_Real__Roll * 15 - RF._uORB_RC_Out__Roll;
	PID_Caculate(PF._uORB_PID__Roll_Input, PF._uORB_Leveling__Roll,
				 PF._uORB_PID_I_Last_Value__Roll, PF._uORB_PID_D_Last_Value__Roll,
				 PF._flag_PID_P__Roll_Gain, PF._flag_PID_I__Roll_Gain, PF._flag_PID_D__Roll_Gain, PF._flag_PID_I__Roll_Max__Value);
	if (PF._uORB_Leveling__Roll > PF._flag_PID_Level_Max)
		PF._uORB_Leveling__Roll = PF._flag_PID_Level_Max;
	if (PF._uORB_Leveling__Roll < PF._flag_PID_Level_Max * -1)
		PF._uORB_Leveling__Roll = PF._flag_PID_Level_Max * -1;

	//Pitch PID Mix
	PF._uORB_PID_Pitch_Input = SF._uORB_Gryo_Pitch + SF._uORB_Real_Pitch * 15 - RF._uORB_RC_Out_Pitch;
	PID_Caculate(PF._uORB_PID_Pitch_Input, PF._uORB_Leveling_Pitch,
				 PF._uORB_PID_I_Last_Value_Pitch, PF._uORB_PID_D_Last_Value_Pitch,
				 PF._flag_PID_P_Pitch_Gain, PF._flag_PID_I_Pitch_Gain, PF._flag_PID_D_Pitch_Gain, PF._flag_PID_I_Pitch_Max__Value);
	if (PF._uORB_Leveling_Pitch > PF._flag_PID_Level_Max)
		PF._uORB_Leveling_Pitch = PF._flag_PID_Level_Max;
	if (PF._uORB_Leveling_Pitch < PF._flag_PID_Level_Max * -1)
		PF._uORB_Leveling_Pitch = PF._flag_PID_Level_Max * -1;

	//Yaw PID Mix
	PID_Caculate(SF._uORB_Gryo___Yaw - RF._uORB_RC_Out___Yaw, PF._uORB_Leveling___Yaw,
				 PF._uORB_PID_I_Last_Value___Yaw, PF._uORB_PID_D_Last_Value___Yaw,
				 PF._flag_PID_P___Yaw_Gain, PF._flag_PID_I___Yaw_Gain, PF._flag_PID_D___Yaw_Gain, PF._flag_PID_I___Yaw_Max__Value);
	if (PF._uORB_Leveling___Yaw > PF._flag_PID_Level_Max)
		PF._uORB_Leveling___Yaw = PF._flag_PID_Level_Max;
	if (PF._uORB_Leveling___Yaw < PF._flag_PID_Level_Max * -1)
		PF._uORB_Leveling___Yaw = PF._flag_PID_Level_Max * -1;
	if (AF._flag_AlthHold_Enable)
	{
		PID_Caculate(SF._uORB_MS5611_Last_Value_AltMeter * 500 - SF._uORB_MS5611_AltMeter * 500, PF._uORB_Leveling_Throttle,
					 PF._uORB_PID_I_Last_Value_Alt, PF._uORB_PID_D_Last_Value_Alt,
					 PF._flag_PID_P_Alt_Gain, PF._flag_PID_I_Alt_Gain, PF._flag_PID_D_Alt_Gain, PF._flag_PID_Alt_Level_Max);
	}
	else
	{
		EF._Tmp_B1_Speed = RF._uORB_RC_Out_Throttle - PF._uORB_Leveling__Roll + PF._uORB_Leveling_Pitch + PF._uORB_Leveling___Yaw;
		EF._Tmp_A1_Speed = RF._uORB_RC_Out_Throttle - PF._uORB_Leveling__Roll - PF._uORB_Leveling_Pitch - PF._uORB_Leveling___Yaw;
		EF._Tmp_A2_Speed = RF._uORB_RC_Out_Throttle + PF._uORB_Leveling__Roll - PF._uORB_Leveling_Pitch + PF._uORB_Leveling___Yaw;
		EF._Tmp_B2_Speed = RF._uORB_RC_Out_Throttle + PF._uORB_Leveling__Roll + PF._uORB_Leveling_Pitch - PF._uORB_Leveling___Yaw;
	}

	EF._uORB_A1_Speed = (700 * (((float)EF._Tmp_A1_Speed - (float)RF._flag_RC_Min_PWM_Value) / (float)(RF._flag_RC_Max_PWM_Value - RF._flag_RC_Min_PWM_Value))) + 2300;
	EF._uORB_A2_Speed = (700 * (((float)EF._Tmp_A2_Speed - (float)RF._flag_RC_Min_PWM_Value) / (float)(RF._flag_RC_Max_PWM_Value - RF._flag_RC_Min_PWM_Value))) + 2300;
	EF._uORB_B1_Speed = (700 * (((float)EF._Tmp_B1_Speed - (float)RF._flag_RC_Min_PWM_Value) / (float)(RF._flag_RC_Max_PWM_Value - RF._flag_RC_Min_PWM_Value))) + 2300;
	EF._uORB_B2_Speed = (700 * (((float)EF._Tmp_B2_Speed - (float)RF._flag_RC_Min_PWM_Value) / (float)(RF._flag_RC_Max_PWM_Value - RF._flag_RC_Min_PWM_Value))) + 2300;
}

void SingleAPMAPI::RPiSingleAPM::SaftyCheckTaskReg()
{
	//ECSLockCheck
	if (RF._uORB_RC_Out_Throttle < RF._flag_RC_Min_PWM_Value + 20 && RF._flag_RC_ARM_PWM_Value - 50 < RF._uORB_RC_Out___ARM && RF._uORB_RC_Out___ARM < RF._flag_RC_ARM_PWM_Value + 50)
	{
		AF._flag_IsLockCleanerEnable = false;
		if (AF._flag_Device_setupFailed == false)
		{
			if (AF._flag_Error == false)
			{
				if (AF._flag_StartUP_Protect == false)
				{
					AF._flag_ESC_ARMED = false;
				}
			}
			else
			{
				AF._flag_ESC_ARMED = true;
			}
		}
	}
	else if (!(RF._flag_RC_ARM_PWM_Value - 50 < RF._uORB_RC_Out___ARM && RF._uORB_RC_Out___ARM < RF._flag_RC_ARM_PWM_Value + 50))
	{
		if (RF._flag_RC_Min_PWM_Value - 50 < RF._uORB_RC_Out___ARM && RF._uORB_RC_Out___ARM < RF._flag_RC_Max_PWM_Value + 50)
		{
			AF._flag_IsLockCleanerEnable = true;
			AF._flag_StartUP_Protect = false;
			AF._flag_ESC_ARMED = true;
			AF._flag_Error = false;
			AF._flag_ClockingTime_Error = false;
		}
	}
	else if (RF._uORB_RC_Out_Throttle < RF._flag_RC_Max_PWM_Value + 20)
	{
		if (AF._flag_Error == true)
		{
			AF._flag_ESC_ARMED = true;
		}
	}
	if (RF._flag_RC_ARM_PWM_Value - 50 < RF._uORB_RC_Out___ARM && RF._uORB_RC_Out___ARM < RF._flag_RC_ARM_PWM_Value + 50)
	{
		if (RF._uORB_RC_Out_Throttle > RF._flag_RC_Min_PWM_Value + 20)
		{
			AF._flag_StartUP_Protect = true;
		}
	}
	if (AF._flag_ESC_ARMED == true)
	{
		AF._flag_MS5611_firstStartUp = true;
		AF._flag_MPU9250_first_StartUp = true;
		PF._uORB_PID_D_Last_Value__Roll = 0;
		PF._uORB_PID_D_Last_Value_Pitch = 0;
		PF._uORB_PID_D_Last_Value___Yaw = 0;
		PF._uORB_PID_I_Last_Value__Roll = 0;
		PF._uORB_PID_I_Last_Value_Pitch = 0;
		PF._uORB_PID_I_Last_Value___Yaw = 0;
	}
	//ErrorDetect
	if (SF._uORB_Real_Pitch > 70.0 || SF._uORB_Real_Pitch < -70.0 || SF._uORB_Real__Roll > 70.0 || SF._uORB_Real__Roll < -70.0)
	{
		AF._flag_Error = true;
	}
	if (RF._flag_RC_Min_PWM_Value < RF._uORB_RC_Channel_PWM[0] && RF._uORB_RC_Channel_PWM[0] > RF._flag_RC_Max_PWM_Value ||
		RF._flag_RC_Min_PWM_Value < RF._uORB_RC_Channel_PWM[1] && RF._uORB_RC_Channel_PWM[1] > RF._flag_RC_Max_PWM_Value ||
		RF._flag_RC_Min_PWM_Value < RF._uORB_RC_Channel_PWM[2] && RF._uORB_RC_Channel_PWM[2] > RF._flag_RC_Max_PWM_Value ||
		RF._flag_RC_Min_PWM_Value < RF._uORB_RC_Channel_PWM[3] && RF._uORB_RC_Channel_PWM[3] > RF._flag_RC_Max_PWM_Value)
	{
		AF._flag_RC_Disconnected = true;
	}
	//RC_LOSE_Detect
	if (AF._flag_RC_Disconnected == true)
	{
		AF.RC_Lose_Clocking += 1;
		if (AF.RC_Lose_Clocking == 200)
		{
			AF._flag_Error = true;
			AF.RC_Lose_Clocking = 0;
		}
	}
	else if (AF._flag_RC_Disconnected == false)
	{
		AF.RC_Lose_Clocking = 0;
	}
}

void SingleAPMAPI::RPiSingleAPM::ESCUpdateTaskReg()
{
	TF.ESCTask = new std::thread([&] {
		while (true)
		{
			TF._Tmp_ESCThreadTimeStart = micros();
			TF._Tmp_ESCThreadTimeNext = TF._Tmp_ESCThreadTimeStart - TF._Tmp_ESCThreadTimeEnd;

			if (AF._flag_ESC_ARMED)
			{
				pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A1_Pin, 0, EF._Flag_Lock_Throttle);
				pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A2_Pin, 0, EF._Flag_Lock_Throttle);
				pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B1_Pin, 0, EF._Flag_Lock_Throttle);
				pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B2_Pin, 0, EF._Flag_Lock_Throttle);
			}
			if (!AF._flag_ESC_ARMED)
			{
				pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A1_Pin, 0,
								EF._uORB_A1_Speed);
				pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A2_Pin, 0,
								EF._uORB_A2_Speed);
				pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B1_Pin, 0,
								EF._uORB_B1_Speed);
				pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B2_Pin, 0,
								EF._uORB_B2_Speed);
			}

			TF._Tmp_ESCThreadTimeEnd = micros();
			TF._Tmp_ESCThreadTimeLoop = TF._Tmp_ESCThreadTimeEnd - TF._Tmp_ESCThreadTimeStart;
			if (TF._Tmp_ESCThreadTimeLoop + TF._Tmp_ESCThreadTimeNext > TF._flag_ESCThreadTimeMax | TF._Tmp_ESCThreadTimeNext < 0)
			{
				usleep(50);
				AF._flag_ClockingTime_Error = true;
			}
			else
			{
				usleep(TF._flag_ESCThreadTimeMax - TF._Tmp_ESCThreadTimeLoop - TF._Tmp_ESCThreadTimeNext);
			}
			if (TF._Tmp_ESCThreadTimeLoop + TF._Tmp_ESCThreadTimeNext > TF._Tmp_ESCThreadError)
			{
				TF._Tmp_ESCThreadError = TF._Tmp_ESCThreadTimeLoop;
			}
			TF._Tmp_ESCThreadTimeEnd = micros();
		}
	});
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(3, &cpuset);
	int rc = pthread_setaffinity_np(TF.ESCTask->native_handle(), sizeof(cpu_set_t), &cpuset);
}

void SingleAPMAPI::RPiSingleAPM::DebugOutPut()
{
	std::cout << "\033[150A";
	std::cout << "\033[K";
	std::cout << "ESCSpeedOutput:"
			  << " \n";
	std::cout << " A1 " << EF._uORB_A1_Speed << "    "
			  << " A2 " << EF._uORB_A2_Speed << "                        "
			  << "\n";
	std::cout << " B1 " << EF._uORB_B1_Speed << "    "
			  << " B2 " << EF._uORB_B2_Speed << "                        "
			  << "\n\n";

	std::cout << "IMUSenorData: "
			  << " \n";
	std::cout << " GryoPitch: " << (int)SF._uORB_Gryo_Pitch << "    "
			  << " GryoRoll: " << (int)SF._uORB_Gryo__Roll << "    "
			  << " GryoYaw: " << (int)SF._uORB_Gryo___Yaw
			  << "                        "
			  << "\n";
	std::cout << " AccePitch: " << (int)SF._uORB_Accel_Pitch << "    "
			  << " AcceRoll: " << (int)SF._uORB_Accel__Roll
			  << "                        "
			  << "\n";
	std::cout << " RealPitch: " << (int)SF._uORB_Real_Pitch << "    "
			  << " RealRoll: " << (int)SF._uORB_Real__Roll
			  << "                        "
			  << "\n\n";

	std::cout << "MS5611ParseDataINFO:"
			  << "\n";
	std::cout << " Pressure:" << SF._uORB_MS5611_Pressure << "            \n";
	std::cout << " altitude:" << SF._uORB_MS5611_AltMeter << "            \n";
	std::cout << " ClimbeRate:" << SF._uORB_MS5611_ClimbeRate << "            \n";
	std::cout << " Leveling_Throttle:" << PF._uORB_Leveling_Throttle << "            \n";
	std::cout << " MS5611ValueSettle:" << SF._uORB_MS5611_Last_Value_AltMeter << "            \n";
	std::cout << "\n";

	std::cout << "RCOutPUTINFO:   "
			  << "\n";
	std::cout << " ChannelRoll  "
			  << ": " << RF._uORB_RC_Out__Roll << std::setw(10) << std::setfill(' ') << "\n";
	std::cout << " ChannelPitch "
			  << ": " << RF._uORB_RC_Out_Pitch << std::setw(10) << std::setfill(' ') << "\n";
	std::cout << " ChannelThrot "
			  << ": " << RF._uORB_RC_Out_Throttle << std::setw(10) << std::setfill(' ') << "\n";
	std::cout << " ChannelYaw   "
			  << ": " << RF._uORB_RC_Out___Yaw << std::setw(10) << std::setfill(' ') << " \n\n";

	std::cout << "ChannelINFO: "
			  << " \n";
	for (size_t i = 0; i < 16; i++)
	{
		std::cout << " " << RF._uORB_RC_Channel_PWM[i] << " ";
	}
	std::cout << " \n\n";

	std::cout << " Flag_ESC_ARMED:" << AF._flag_ESC_ARMED << "               \n";
	std::cout << " Flag_Error:" << AF._flag_Error << "           \n";
	std::cout << " Flag_IsLockEnable:" << AF._flag_IsLockCleanerEnable << "         \n";
	std::cout << " Flag_ClockingTime_Error:" << AF._flag_ClockingTime_Error << "         \n";
	std::cout << " Flag_RC_Disconnected:" << AF._flag_RC_Disconnected << "         \n";
	std::cout << " RC_Lose_Clocking:" << AF.RC_Lose_Clocking << "                        \n\n";

	std::cout << " IMULoopTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_IMUThreadTimeLoop;
	std::cout << " IMUNextTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_IMUThreadTimeNext;
	std::cout << " IMUMaxTime:  " << std::setw(7) << std::setfill(' ') << TF._Tmp_IMUThreadError << "    \n";
	std::cout << " RXTLoopTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_RXTThreadTimeLoop;
	std::cout << " RXTNextTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_RXTThreadTimeNext;
	std::cout << " RXTMaxTime:  " << std::setw(7) << std::setfill(' ') << TF._Tmp_RXTThreadError << "    \n";
	std::cout << " ESCLoopTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_ESCThreadTimeLoop;
	std::cout << " ESCNextTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_ESCThreadTimeNext;
	std::cout << " ESCMaxTime:  " << std::setw(7) << std::setfill(' ') << TF._Tmp_ESCThreadError << "    \n";
	std::cout << " ALTLoopTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_ALTThreadTimeLoop;
	std::cout << " ALTNextTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_ALTThreadTimeNext;
	std::cout << " ALTMaxTime:  " << std::setw(7) << std::setfill(' ') << TF._Tmp_ALTThreadError << "    \n";
}

void SingleAPMAPI::RPiSingleAPM::TaskThreadBlock()
{
	while (true)
	{
		DebugOutPut();
		SaftyCheckTaskReg();
		usleep(20000);
	}
}

//=-----------------------------------------------------------------------------------------==//

void SingleAPMAPI::RPiSingleAPM::PID_Caculate(float inputData, float &outputData,
											  float &last_I_Data, float &last_D_Data,
											  float P_Gain, float I_Gain, float D_Gain, float I_Max)
{
	//P caculate
	outputData = P_Gain * inputData;
	//D caculate
	outputData += D_Gain * (inputData - last_D_Data);
	last_D_Data = inputData;
	//I caculate
	last_I_Data += inputData * I_Gain;
	if (last_I_Data > I_Max)
		last_I_Data = I_Max;
	if (last_I_Data < I_Max * -1)
		last_I_Data = I_Max * -1;
	//P_I_D Mix OUTPUT
	outputData += last_I_Data;
}

void SingleAPMAPI::RPiSingleAPM::ConfigReader(APMSettinngs APMInit)
{
#ifdef USINGJSON
	std::ifstream config(DF.configDir);
	std::string content((std::istreambuf_iterator<char>(config)),
						(std::istreambuf_iterator<char>()));
	nlohmann::json Configdata = nlohmann::json::parse(content);
	//==========================================================Device Type=======/
	RF.RC_Type = Configdata["Type_RC"].get<int>();
	SF.MPU9250_Type = Configdata["Type_MPU9250"].get<int>();
	SF.IMUFilter_Type = Configdata["Type_IMUFilter"].get<int>();
	SF.IMUMixFilter_Type = Configdata["Type_IMUMixFilter"].get<int>();
	//==========================================================Controller cofig==/
	RF._flag_RC_ARM_PWM_Value = Configdata["_flag_RC_ARM_PWM_Value"].get<int>();
	RF._flag_RC_Min_PWM_Value = Configdata["_flag_RC_Min_PWM_Value"].get<int>();
	RF._flag_RC_Mid_PWM_Value = Configdata["_flag_RC_Mid_PWM_Value"].get<int>();
	RF._flag_RC_Max_PWM_Value = Configdata["_flag_RC_Max_PWM_Value"].get<int>();

	RF._flag_RCIsReserv__Roll = Configdata["_flag_RCIsReserv__Roll"].get<int>();
	RF._flag_RCIsReserv_Pitch = Configdata["_flag_RCIsReserv_Pitch"].get<int>();
	RF._flag_RCIsReserv___Yaw = Configdata["_flag_RCIsReserv___Yaw"].get<int>();
	//==========================================================ESC cofig=========/
	EF._flag_A1_Pin = Configdata["_flag_A1_Pin"].get<int>();
	EF._flag_A2_Pin = Configdata["_flag_A2_Pin"].get<int>();
	EF._flag_B1_Pin = Configdata["_flag_B1_Pin"].get<int>();
	EF._flag_B2_Pin = Configdata["_flag_B2_Pin"].get<int>();
	//==================================================================PID cofig==/
	PF._flag_PID_P__Roll_Gain = Configdata["_flag_PID_P__Roll_Gain"].get<float>();
	PF._flag_PID_P_Pitch_Gain = Configdata["_flag_PID_P_Pitch_Gain"].get<float>();
	PF._flag_PID_P___Yaw_Gain = Configdata["_flag_PID_P___Yaw_Gain"].get<float>();

	PF._flag_PID_I__Roll_Gain = Configdata["_flag_PID_I__Roll_Gain"].get<float>();
	PF._flag_PID_I_Pitch_Gain = Configdata["_flag_PID_I_Pitch_Gain"].get<float>();
	PF._flag_PID_I___Yaw_Gain = Configdata["_flag_PID_I___Yaw_Gain"].get<float>();
	PF._flag_PID_I__Roll_Max__Value = Configdata["_flag_PID_I__Roll_Max__Value"].get<float>();
	PF._flag_PID_I_Pitch_Max__Value = Configdata["_flag_PID_I_Pitch_Max__Value"].get<float>();
	PF._flag_PID_I___Yaw_Max__Value = Configdata["_flag_PID_I___Yaw_Max__Value"].get<float>();

	PF._flag_PID_D__Roll_Gain = Configdata["_flag_PID_D__Roll_Gain"].get<float>();
	PF._flag_PID_D_Pitch_Gain = Configdata["_flag_PID_D_Pitch_Gain"].get<float>();
	PF._flag_PID_D___Yaw_Gain = Configdata["_flag_PID_D___Yaw_Gain"].get<float>();

	PF._flag_PID_Level_Max = Configdata["_flag_PID_Level_Max"].get<float>();
	//==============================================================Sensors cofig==/
	SF._flag_Accel__Roll_Cali = Configdata["_flag_Accel__Roll_Cali"].get<double>();
	SF._flag_Accel_Pitch_Cali = Configdata["_flag_Accel_Pitch_Cali"].get<double>();
	//===============================================================Update cofig==/
	TF._flag_IMUThreadFreq = Configdata["IMU_Freqeucy"].get<int>();
	TF._flag_IMUThreadTimeMax = (float)1 / TF._flag_IMUThreadFreq * 1000000;
	TF._flag_RXTThreadFreq = Configdata["RXT_Freqeucy"].get<int>();
	TF._flag_RXTThreadTimeMax = (float)1 / TF._flag_RXTThreadFreq * 1000000;
	TF._flag_ESCThreadFreq = Configdata["ESC_Freqeucy"].get<int>();
	TF._flag_ESCThreadTimeMax = (float)1 / TF._flag_ESCThreadFreq * 1000000;
#else
	SF.MPU9250_Type = APMInit.MPU9250_Type;
	RF.RC_Type = APMInit.RC_Type;
	SF.IMUFilter_Type = APMInit.IMUFilter_Type;
	SF.IMUMixFilter_Type = APMInit.IMUMixFilter_Type;

	TF._flag_IMUThreadFreq = APMInit.IMU_Freqeuncy;
	TF._flag_IMUThreadTimeMax = (float)1 / TF._flag_IMUThreadFreq * 1000000;
	TF._flag_RXTThreadFreq = APMInit.RXT_Freqeuncy;
	TF._flag_RXTThreadTimeMax = (float)1 / TF._flag_RXTThreadFreq * 1000000;
	TF._flag_ESCThreadFreq = APMInit.ESC_Freqeuncy;
	TF._flag_ESCThreadTimeMax = (float)1 / TF._flag_ESCThreadFreq * 1000000;

	PF._flag_PID_P__Roll_Gain = APMInit._flag_PID_P__Roll_Gain;
	PF._flag_PID_P_Pitch_Gain = APMInit._flag_PID_P_Pitch_Gain;
	PF._flag_PID_P___Yaw_Gain = APMInit._flag_PID_P___Yaw_Gain;

	PF._flag_PID_I__Roll_Gain = APMInit._flag_PID_I__Roll_Gain;
	PF._flag_PID_I_Pitch_Gain = APMInit._flag_PID_I_Pitch_Gain;
	PF._flag_PID_I___Yaw_Gain = APMInit._flag_PID_I___Yaw_Gain;

	PF._flag_PID_I__Roll_Max__Value = APMInit._flag_PID_I__Roll_Max__Value;
	PF._flag_PID_I_Pitch_Max__Value = APMInit._flag_PID_I_Pitch_Max__Value;
	PF._flag_PID_I___Yaw_Max__Value = APMInit._flag_PID_I___Yaw_Max__Value;

	PF._flag_PID_D__Roll_Gain = APMInit._flag_PID_D__Roll_Gain;
	PF._flag_PID_D_Pitch_Gain = APMInit._flag_PID_D_Pitch_Gain;
	PF._flag_PID_D___Yaw_Gain = APMInit._flag_PID_D___Yaw_Gain;
	PF._flag_PID_Level_Max = APMInit._flag_PID_Level_Max;

	SF._flag_Accel__Roll_Cali = APMInit._flag_Accel__Roll_Cali;
	SF._flag_Accel_Pitch_Cali = APMInit._flag_Accel_Pitch_Cali;

	EF._flag_A1_Pin = APMInit._flag_A1_Pin;
	EF._flag_A2_Pin = APMInit._flag_A2_Pin;
	EF._flag_B1_Pin = APMInit._flag_B1_Pin;
	EF._flag_B2_Pin = APMInit._flag_B2_Pin;

	RF._flag_RC_Min_PWM_Value = APMInit._flag_RC_Min_PWM_Value;
	RF._flag_RC_Mid_PWM_Value = APMInit._flag_RC_Mid_PWM_Value;
	RF._flag_RC_Max_PWM_Value = APMInit._flag_RC_Max_PWM_Value;
	RF._flag_RC_ARM_PWM_Value = APMInit._flag_RC_ARM_PWM_Value;
#endif
}

void SingleAPMAPI::RPiSingleAPM::IMUSensorsDataRead()
{
	if (SF.MPU9250_Type == MPUIsI2c)
	{
		SF._Tmp_MPU9250_Buffer[0] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x3B);
		SF._Tmp_MPU9250_Buffer[1] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x3C);
		SF._Tmp_MPU9250_A_X = (SF._Tmp_MPU9250_Buffer[0] << 8 | SF._Tmp_MPU9250_Buffer[1]);
		SF._uORB_MPU9250_A_X = (short)SF._Tmp_MPU9250_A_X;
		SF._Tmp_MPU9250_Buffer[2] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x3D);
		SF._Tmp_MPU9250_Buffer[3] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x3E);
		SF._Tmp_MPU9250_A_Y = (SF._Tmp_MPU9250_Buffer[2] << 8 | SF._Tmp_MPU9250_Buffer[3]);
		SF._uORB_MPU9250_A_Y = (short)SF._Tmp_MPU9250_A_Y;
		SF._Tmp_MPU9250_Buffer[4] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x3F);
		SF._Tmp_MPU9250_Buffer[5] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x40);
		SF._Tmp_MPU9250_A_Z = (SF._Tmp_MPU9250_Buffer[4] << 8 | SF._Tmp_MPU9250_Buffer[5]);
		SF._uORB_MPU9250_A_Z = (short)SF._Tmp_MPU9250_A_Z;

		SF._Tmp_MPU9250_Buffer[6] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x43);
		SF._Tmp_MPU9250_Buffer[7] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x44);
		SF._Tmp_MPU9250_G_X = (SF._Tmp_MPU9250_Buffer[6] << 8 | SF._Tmp_MPU9250_Buffer[7]);
		SF._uORB_MPU9250_G_X = (short)SF._Tmp_MPU9250_G_X;
		SF._Tmp_MPU9250_Buffer[8] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x45);
		SF._Tmp_MPU9250_Buffer[9] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x46);
		SF._Tmp_MPU9250_G_Y = (SF._Tmp_MPU9250_Buffer[8] << 8 | SF._Tmp_MPU9250_Buffer[9]);
		SF._uORB_MPU9250_G_Y = (short)SF._Tmp_MPU9250_G_Y;
		SF._Tmp_MPU9250_Buffer[10] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x47);
		SF._Tmp_MPU9250_Buffer[11] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x48);
		SF._Tmp_MPU9250_G_Z = (SF._Tmp_MPU9250_Buffer[10] << 8 | SF._Tmp_MPU9250_Buffer[11]);
		SF._uORB_MPU9250_G_Z = (short)SF._Tmp_MPU9250_G_Z;
	}
	else if (SF.MPU9250_Type == MPUIsSpi)
	{
		SF._Tmp_MPU9250_SPI_Buffer[0] = 0xBB;
		wiringPiSPIDataRW(1, SF._Tmp_MPU9250_SPI_Buffer, 20);
		SF._Tmp_MPU9250_A_X = ((int)SF._Tmp_MPU9250_SPI_Buffer[1] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[2]);
		SF._uORB_MPU9250_A_X = (short)SF._Tmp_MPU9250_A_X;
		SF._Tmp_MPU9250_A_Y = ((int)SF._Tmp_MPU9250_SPI_Buffer[3] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[4]);
		SF._uORB_MPU9250_A_Y = (short)SF._Tmp_MPU9250_A_Y;
		SF._Tmp_MPU9250_A_Z = ((int)SF._Tmp_MPU9250_SPI_Buffer[5] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[6]);
		SF._uORB_MPU9250_A_Z = (short)SF._Tmp_MPU9250_A_Z;

		SF._Tmp_MPU9250_G_X = ((int)SF._Tmp_MPU9250_SPI_Buffer[9] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[10]);
		SF._uORB_MPU9250_G_X = (short)SF._Tmp_MPU9250_G_X;
		SF._Tmp_MPU9250_G_Y = ((int)SF._Tmp_MPU9250_SPI_Buffer[11] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[12]);
		SF._uORB_MPU9250_G_Y = (short)SF._Tmp_MPU9250_G_Y;
		SF._Tmp_MPU9250_G_Z = ((int)SF._Tmp_MPU9250_SPI_Buffer[13] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[14]);
		SF._uORB_MPU9250_G_Z = (short)SF._Tmp_MPU9250_G_Z;
	}
}

void SingleAPMAPI::RPiSingleAPM::IMUGryoFilter(long next_input_value, long &next_output_value, long *xv, long *yv, int filtertype)
{
	if (filtertype == GryoFilterType_none)
	{
		next_output_value = next_input_value;
	}
	else if (filtertype == GryoFilterType_pt1)
	{
	}
	else if (filtertype == GryoFilterType_Butterworth)
	{
		xv[0] = xv[1];
		xv[1] = xv[2];
		xv[2] = next_input_value / SF._flag_Filter2x50_Gain;
		yv[0] = yv[1];
		yv[1] = yv[2];
		yv[2] = (xv[0] + xv[2]) + 2 * xv[1] + (-0.1958157127 * yv[0]) + (0.3695273774 * yv[1]);
		next_output_value = yv[2];
	}
};

void SingleAPMAPI::RPiSingleAPM::IMUMixFilter(Kalman *kal, float next_input_value_Gryo, float next_input_value_Accel,
											  float next_input_value_speed, float &next_output_value, int filtertype)
{
	if (filtertype == MixFilterType_traditional)
	{
		next_output_value = next_input_value_Gryo * 0.9996 + next_input_value_Accel * 0.0004;
	}
	else if (filtertype == MixFilterType_Kalman)
	{
		next_output_value = kal->getAngle(next_input_value_Accel, next_input_value_speed, 1.f / (float)TF._flag_IMUThreadFreq);
	}
}
