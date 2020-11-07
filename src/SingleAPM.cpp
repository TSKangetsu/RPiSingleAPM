#include "SingleAPM.hpp"

int SingleAPMAPI::RPiSingleAPM::RPiSingleAPMInit(APMSettinngs APMInit)
{
	wiringPiSetup();
	AF.RC_Lose_Clocking = 0;
	AF.GPS_Lose_Clocking = 0;
	AF._flag_MPU9250_first_StartUp = true;
	AF._flag_ESC_ARMED = true;
	AF.AutoPilotMode = APModeINFO::AutoStable;
	ConfigReader(APMInit);

#ifdef DEBUG
	std::cout << "[RPiSingleAPM]ESCControllerIniting \n";
#endif
	if (DF.PCA9658_fd == -1)
		DF.PCA9658_fd = pca9685Setup(DF.PCA9685_PinBase, DF.PCA9685_Address, DF.PWM_Freq);
	else
		pca9685PWMReset(DF.PCA9658_fd);
	if (DF.PCA9658_fd == -1)
	{
#ifdef DEBUG
		std::cout << "[RPiSingleAPM]ESCControllerInitFailed \n";
#endif
		return -1;
	}

	if (SF.MPU9250_Type == MPUIsI2c)
	{
#ifdef DEBUG
		std::cout << "[RPiSingleAPM]MPUI2CIniting \n";
#endif
		DF.MPU9250_fd = wiringPiI2CSetup(DF.MPU9250_ADDR);
		wiringPiI2CWriteReg8(DF.MPU9250_fd, 107, 0x00); //reset
		wiringPiI2CWriteReg8(DF.MPU9250_fd, 28, 0x08);	//Accel
		wiringPiI2CWriteReg8(DF.MPU9250_fd, 27, 0x08);	//Gryo
		wiringPiI2CWriteReg8(DF.MPU9250_fd, 26, 0x03);	//config
	}
	else if (SF.MPU9250_Type == MPUIsSpi)
	{
#ifdef DEBUG
		std::cout << "[RPiSingleAPM]MPUSPIIniting \n";
#endif
		DF.MPU9250_fd = wiringPiSPISetup(DF.MPU9250_SPI_Channel, DF.MPU9250_SPI_Freq);
		SF._Tmp_MPU9250_SPI_Config[0] = 0x75;
		wiringPiSPIDataRW(DF.MPU9250_SPI_Channel, SF._Tmp_MPU9250_SPI_Config, 2); //WHOAMI
		SF._Tmp_MPU9250_SPI_Config[0] = 0x6b;
		SF._Tmp_MPU9250_SPI_Config[1] = 0x00;
		wiringPiSPIDataRW(DF.MPU9250_SPI_Channel, SF._Tmp_MPU9250_SPI_Config, 2); //reset
		SF._Tmp_MPU9250_SPI_Config[0] = 0x1c;
		SF._Tmp_MPU9250_SPI_Config[1] = 0x08;
		wiringPiSPIDataRW(DF.MPU9250_SPI_Channel, SF._Tmp_MPU9250_SPI_Config, 2); // Accel
		SF._Tmp_MPU9250_SPI_Config[0] = 0x1b;
		SF._Tmp_MPU9250_SPI_Config[1] = 0x08;
		wiringPiSPIDataRW(DF.MPU9250_SPI_Channel, SF._Tmp_MPU9250_SPI_Config, 2); // Gryo
		SF._Tmp_MPU9250_SPI_Config[0] = 0x1a;
		SF._Tmp_MPU9250_SPI_Config[1] = 0x03;
		wiringPiSPIDataRW(DF.MPU9250_SPI_Channel, SF._Tmp_MPU9250_SPI_Config, 2); //config
// MPU9250-AK8963 Slave settle
#ifdef DEBUG
		std::cout << "[RPiSingleAPM]Waiting for Compass Config ... ";
		std::cout.flush();
#endif
		{
			for (size_t Index = 0; Index < 39; Index++)
			{
				if (Index == 22)
				{
					wiringPiSPIDataRW(DF.MPU9250_SPI_Channel, SF._flag_MPU9250_CompassConfig[Index], 4);
					SF._flag_MPU9250_M_X_Cali = (float)(SF._flag_MPU9250_CompassConfig[Index][1] - 128) / 256.0f + 1.0f;
					SF._flag_MPU9250_M_Y_Cali = (float)(SF._flag_MPU9250_CompassConfig[Index][2] - 128) / 256.0f + 1.0f;
					SF._flag_MPU9250_M_Z_Cali = (float)(SF._flag_MPU9250_CompassConfig[Index][3] - 128) / 256.0f + 1.0f;
					SF._flag_MPU9250_M_MRES = 10. * 4912. / 32760.0;
				}
				else
				{
					wiringPiSPIDataRW(DF.MPU9250_SPI_Channel, SF._flag_MPU9250_CompassConfig[Index], 2);
				}
				usleep(50000);
			}
			unsigned char CompassConfigFinal[4][2] = {{0x25, 0x0C | 0x80}, {0x26, 0x03}, {0x27, 0x87}};
			for (size_t Index = 0; Index < 3; Index++)
			{
				wiringPiSPIDataRW(DF.MPU9250_SPI_Channel, CompassConfigFinal[Index], 2);
			}
		}
#ifdef DEBUG
		std::cout << "Done!\n";
#endif
	}
	if (DF.MPU9250_fd == -1)
	{
#ifdef DEBUG
		std::cout << "[RPiSingleAPM]MPU9250DeviceError \n";
#endif
		return -2;
	}

	if (SF.IMUMixFilter_Type == MixFilterType_Kalman)
	{
		Kal_Pitch = new Kalman();
		Kal__Roll = new Kalman();
	}

	if (RF.RC_Type == RCIsIbus)
	{
#ifdef DEBUG
		std::cout << "[RPiSingleAPM]Controller Ibus config comfirm\n";
#endif
		IbusInit = new Ibus(DF.RCDevice);
	}
	else if (RF.RC_Type == RCIsSbus)
	{
#ifdef DEBUG
		std::cout << "[RPiSingleAPM]Controller Sbus config comfirm\n";
#endif
		SbusInit = new Sbus(DF.RCDevice, SbusMode::Normal);
	}

	{
#ifdef DEBUG
		std::cout << "[RPiSingleAPM]Checking MS5611 ... ";
		std::cout.flush();
#endif
		MS5611S = new MS5611();
		if (!MS5611S->MS5611Init())
		{
#ifdef DEBUG
			std::cout << "[RPiSingleAPM]MS5611InitError \n";
#endif
		}
		MS5611S->LocalPressureSetter(0, 5);
		for (size_t i = 0; i < 40; i++)
		{
			MS5611S->MS5611FastReader(SF._Tmp_MS5611_Data);
			SF._uORB_MS5611_Pressure = SF._Tmp_MS5611_Data[0] * 100.f;
			SF._Tmp_MS5611_AvaTotal -= SF._Tmp_MS5611_AvaData[SF._Tmp_MS5611_AvaClock];
			SF._Tmp_MS5611_AvaData[SF._Tmp_MS5611_AvaClock] = SF._uORB_MS5611_Pressure;
			SF._Tmp_MS5611_AvaTotal += SF._Tmp_MS5611_AvaData[SF._Tmp_MS5611_AvaClock];
			SF._Tmp_MS5611_AvaClock++;
			if (SF._Tmp_MS5611_AvaClock == 20)
				SF._Tmp_MS5611_AvaClock = 0;
			SF._uORB_MS5611_PressureFast = SF._Tmp_MS5611_AvaTotal / 20.f;
			SF._uORB_MS5611_PressureFill = SF._uORB_MS5611_PressureFast;
		}
		MS5611S->LocalPressureSetter(SF._uORB_MS5611_PressureFill, 5);
#ifdef DEBUG
		std::cout << "Done! LocalPressure Is: " << SF._uORB_MS5611_PressureFill << "\n";
#endif
	}

#ifdef DEBUG
	std::cout << "[RPiSingleAPM]Waiting for GPS Data ... ";
	std::cout.flush();
#endif
	GPSInit = new GPSUart(DF.GPSDevice);
	GPSMAGInit = new GPSI2CCompass_QMC5883L();
	GPSMAGInit->GPSI2CCompass_QMC5883LInit();
	SF._uORB_GPS_Data = GPSInit->GPSParse();
#ifdef DEBUG
	std::cout << "Done \n";
#endif
#ifdef DEBUG
	std::cout << "[RPiSingleAPM]Calibrating Gryo , Dont Move!! ...";
	std::cout.flush();
#endif
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
			usleep(500);
		}
		SF._flag_MPU9250_G_X_Cali = SF._flag_MPU9250_G_X_Cali / 2000;
		SF._flag_MPU9250_G_Y_Cali = SF._flag_MPU9250_G_Y_Cali / 2000;
		SF._flag_MPU9250_G_Z_Cali = SF._flag_MPU9250_G_Z_Cali / 2000;
	}
#ifdef DEBUG
	std::cout << "Done \n";
	sleep(1);
	system("clear");
#endif
	return 0;
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
			SF._Tmp_Gryo_RTSpeed___Yaw = (SF._uORB_MPU9250_G_Fixed_Z / DF._flag_MPU9250_LSB);
			SF._uORB_Real_Pitch += SF._Tmp_Gryo_RTSpeed_Pitch / TF._flag_IMUThreadFreq;
			SF._uORB_Real__Roll += SF._Tmp_Gryo_RTSpeed__Roll / TF._flag_IMUThreadFreq;
			SF._uORB_Real___Yaw += SF._Tmp_Gryo_RTSpeed___Yaw / TF._flag_IMUThreadFreq;
			if (SF._uORB_Real___Yaw < 0)
				SF._uORB_Real___Yaw += 360;
			else if (SF._uORB_Real___Yaw >= 360)
				SF._uORB_Real___Yaw -= 360;
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
			//MAG-----------------------------------------------------------------------//
			SF._uORB_MPU9250_M_X = SF._uORB_MPU9250_M_X * SF._flag_MPU9250_M_MRES * SF._flag_MPU9250_M_X_Cali - SF._flag_MPU9250_M_X_Scaler;
			SF._uORB_MPU9250_M_Y = SF._uORB_MPU9250_M_Y * SF._flag_MPU9250_M_MRES * SF._flag_MPU9250_M_Y_Cali - SF._flag_MPU9250_M_Y_Scaler;
			SF._uORB_MPU9250_M_Z = SF._uORB_MPU9250_M_Z * SF._flag_MPU9250_M_MRES * SF._flag_MPU9250_M_Z_Cali - SF._flag_MPU9250_M_Z_Scaler;
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
			//HeadingCaculate------------------------------------------------------------//
			SF._Tmp_MPU9250_M_XH = SF._uORB_MPU9250_M_X * cos(SF._uORB_Real_Pitch * -0.0174533) +
								   SF._uORB_MPU9250_M_Y * sin(SF._uORB_Real_Pitch * 0.0174533) + sin(SF._uORB_Real__Roll * -0.0174533) -
								   SF._uORB_MPU9250_M_Z * cos(SF._uORB_Real_Pitch * 0.0174533) * sin(SF._uORB_Real__Roll * -0.0174533);
			SF._Tmp_MPU9250_M_YH = SF._uORB_MPU9250_M_Y * cos(SF._uORB_Real_Pitch * 0.0174533) +
								   SF._uORB_MPU9250_M_Z + sin(SF._uORB_Real_Pitch * 0.0174533);
			if (SF._Tmp_MPU9250_M_YH < 0)
				SF._uORB_MAG_Heading = 180 + (180 + ((atan2(SF._Tmp_MPU9250_M_YH, SF._Tmp_MPU9250_M_XH)) * (180 / 3.14)));
			else
				SF._uORB_MAG_Heading = (atan2(SF._Tmp_MPU9250_M_YH, SF._Tmp_MPU9250_M_XH)) * (180 / 3.14);
			if (SF._uORB_MAG_Heading < 0)
				SF._uORB_MAG_Heading += 360;
			else if (SF._uORB_MAG_Heading >= 360)
				SF._uORB_MAG_Heading -= 360;

			SF._Tmp_Real__Head = SF._uORB_Real___Yaw - SF._uORB_MAG_Heading;
			if (SF._Tmp_Real__Head < -180 || SF._Tmp_Real__Head > 180)
			{
				if (SF._uORB_MAG_Heading > 180)
					SF._Tmp_Real__Head__Mag = SF._uORB_MAG_Heading - 180;
				else
					SF._Tmp_Real__Head__Mag = SF._uORB_MAG_Heading + 180;
				if (SF._Tmp_Real__Head > 180)
					SF._Tmp_Real__Head_Gryo = SF._uORB_Real___Yaw - 180;
				else
					SF._Tmp_Real__Head_Gryo = SF._uORB_Real___Yaw + 180;
				SF._Tmp_Real__Head = SF._Tmp_Real__Head_Gryo - SF._Tmp_Real__Head__Mag;
			}
			SF._uORB_Real___Yaw -= SF._Tmp_Real__Head / 1200.0;
			if (SF._uORB_Real___Yaw < 0)
				SF._uORB_Real___Yaw += 360;
			else if (SF._uORB_Real___Yaw >= 360)
				SF._uORB_Real___Yaw -= 360;
			SF._uORB_Real__Head = SF._uORB_Real___Yaw + SF._flag_MPU9250_Head_Asix;
			if (SF._uORB_Real__Head < 0)
				SF._uORB_Real__Head += 360;
			else if (SF._uORB_Real__Head >= 360)
				SF._uORB_Real__Head -= 360;
			//IMU SaftyChecking---------------------------------------------------------//
			if (SF._uORB_Real_Pitch > 70.0 || SF._uORB_Real_Pitch < -70.0 || SF._uORB_Real__Roll > 70.0 || SF._uORB_Real__Roll < -70.0)
			{
				AF._flag_Error = true;
			}

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

			MS5611S->MS5611FastReader(SF._Tmp_MS5611_Data);
			SF._uORB_MS5611_Pressure = SF._Tmp_MS5611_Data[0] * 100.f;
			SF._Tmp_MS5611_AvaTotal -= SF._Tmp_MS5611_AvaData[SF._Tmp_MS5611_AvaClock];
			SF._Tmp_MS5611_AvaData[SF._Tmp_MS5611_AvaClock] = SF._uORB_MS5611_Pressure;
			SF._Tmp_MS5611_AvaTotal += SF._Tmp_MS5611_AvaData[SF._Tmp_MS5611_AvaClock];
			SF._Tmp_MS5611_AvaClock++;
			if (SF._Tmp_MS5611_AvaClock == 20)
				SF._Tmp_MS5611_AvaClock = 0;
			SF._uORB_MS5611_PressureFast = SF._Tmp_MS5611_AvaTotal / 20.f;
			SF._uORB_MS5611_PressureFill = SF._flag_MS5611_FilterAlpha * SF._uORB_MS5611_PressureFill + (1.f - SF._flag_MS5611_FilterAlpha) * SF._uORB_MS5611_PressureFast;
			SF._uORB_MS5611_PressureDiff = SF._uORB_MS5611_PressureFill - SF._uORB_MS5611_PressureFast;
			if (SF._uORB_MS5611_PressureDiff > 8)
				SF._uORB_MS5611_PressureDiff = 8;
			if (SF._uORB_MS5611_PressureDiff < -8)
				SF._uORB_MS5611_PressureDiff = -8;
			if (SF._uORB_MS5611_PressureDiff > 1 || SF._uORB_MS5611_PressureDiff < -1)
				SF._uORB_MS5611_PressureFill -= SF._uORB_MS5611_PressureDiff / 6.f;
			SF._uORB_MS5611_PressureFinal = SF._uORB_MS5611_PressureFill;
			AF._flag_MS5611_Async = true;

			TF._Tmp_ALTThreadTimeEnd = micros();
			TF._Tmp_ALTThreadTimeLoop = TF._Tmp_ALTThreadTimeEnd - TF._Tmp_ALTThreadTimeStart;
			TF._Tmp_ALTThreadTimeLoop = TF._Tmp_ALTThreadTimeEnd - TF._Tmp_ALTThreadTimeStart;
			if (TF._Tmp_ALTThreadTimeLoop + TF._Tmp_ALTThreadTimeNext > TF._flag_ALTThreadTimeMax | TF._Tmp_ALTThreadTimeNext < 0)
			{
				usleep(50);
				AF._flag_ClockingTime_Error = true;
			}
			else
			{
				usleep(TF._flag_ALTThreadTimeMax - TF._Tmp_ALTThreadTimeLoop - TF._Tmp_ALTThreadTimeNext);
			}
			if (TF._Tmp_ALTThreadTimeLoop + TF._Tmp_ALTThreadTimeNext > TF._Tmp_ALTThreadError)
			{
				TF._Tmp_ALTThreadError = TF._Tmp_ALTThreadTimeLoop;
			}
			TF._Tmp_ALTThreadTimeEnd = micros();
		}
	});
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(3, &cpuset);
	int rc = pthread_setaffinity_np(TF.ALTTask->native_handle(), sizeof(cpu_set_t), &cpuset);
}

void SingleAPMAPI::RPiSingleAPM::ControllerTaskReg()
{
	TF.RXTask = new std::thread([&] {
		while (true)
		{
			TF._Tmp_RXTThreadTimeStart = micros();
			TF._Tmp_RXTThreadTimeNext = TF._Tmp_RXTThreadTimeStart - TF._Tmp_RXTThreadTimeEnd;

			//RC Read Parse
			{
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
			}
			//RC Out Caculation
			{
				if (RF._uORB_RC_Channel_PWM[0] < RF._flag_RC_Mid_PWM_Value + 10 && RF._uORB_RC_Channel_PWM[0] > RF._flag_RC_Mid_PWM_Value - 10)
					RF._uORB_RC_Out__Roll = 0;
				else
					RF._uORB_RC_Out__Roll = (RF._uORB_RC_Channel_PWM[0] - RF._flag_RC_Mid_PWM_Value) * RF._flag_RCIsReserv__Roll;
				//
				if (RF._uORB_RC_Channel_PWM[1] < RF._flag_RC_Mid_PWM_Value + 10 && RF._uORB_RC_Channel_PWM[1] > RF._flag_RC_Mid_PWM_Value - 10)
					RF._uORB_RC_Out_Pitch = 0;
				else
					RF._uORB_RC_Out_Pitch = (RF._uORB_RC_Channel_PWM[1] - RF._flag_RC_Mid_PWM_Value) * RF._flag_RCIsReserv_Pitch;
				//
				if (RF._uORB_RC_Channel_PWM[3] < RF._flag_RC_Mid_PWM_Value + 10 && RF._uORB_RC_Channel_PWM[3] > RF._flag_RC_Mid_PWM_Value - 10)
					RF._uORB_RC_Out___Yaw = 0;
				else
					RF._uORB_RC_Out___Yaw = (RF._uORB_RC_Channel_PWM[3] - RF._flag_RC_Mid_PWM_Value) * RF._flag_RCIsReserv___Yaw;
				//
				RF._uORB_RC_Out_Throttle = RF._uORB_RC_Channel_PWM[2];
				//
				RF._uORB_RC_Out___ARM = RF._uORB_RC_Channel_PWM[4];
				//
				RF._uORB_RC_Out_FlyMod = RF._uORB_RC_Channel_PWM[5];
			}
			//RC Safty Checking
			{
				if (RF._flag_RC_Min_PWM_Value < RF._uORB_RC_Channel_PWM[0] && RF._uORB_RC_Channel_PWM[0] > RF._flag_RC_Max_PWM_Value ||
					RF._flag_RC_Min_PWM_Value < RF._uORB_RC_Channel_PWM[1] && RF._uORB_RC_Channel_PWM[1] > RF._flag_RC_Max_PWM_Value ||
					RF._flag_RC_Min_PWM_Value < RF._uORB_RC_Channel_PWM[2] && RF._uORB_RC_Channel_PWM[2] > RF._flag_RC_Max_PWM_Value ||
					RF._flag_RC_Min_PWM_Value < RF._uORB_RC_Channel_PWM[3] && RF._uORB_RC_Channel_PWM[3] > RF._flag_RC_Max_PWM_Value)
					AF._flag_RC_Disconnected = true;
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
				//RC UNLOCK Checking
				if (RF._uORB_RC_Out_Throttle < RF._flag_RC_Min_PWM_Value + 20 && RF._flag_RC_ARM_PWM_Value - 50 < RF._uORB_RC_Out___ARM && RF._uORB_RC_Out___ARM < RF._flag_RC_ARM_PWM_Value + 50)
				{
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
						AF._flag_StartUP_Protect = false;
						AF._flag_ESC_ARMED = true;
						AF._flag_Error = false;
						AF._flag_GPS_Error = false;
						AF._flag_ClockingTime_Error = false;
					}
				}
				if (RF._flag_RC_ARM_PWM_Value - 50 < RF._uORB_RC_Out___ARM && RF._uORB_RC_Out___ARM < RF._flag_RC_ARM_PWM_Value + 50)
				{
					if (RF._uORB_RC_Out_Throttle > RF._flag_RC_Min_PWM_Value + 20)
					{
						AF._flag_StartUP_Protect = true;
					}
				}
			}
			//flyMode Switch
			{
				if (RF._flag_RC_Min_PWM_Value + 50 > RF._uORB_RC_Out_FlyMod && RF._flag_RC_Min_PWM_Value - 50 < RF._uORB_RC_Out_FlyMod)
				{
					AF.AutoPilotMode = APModeINFO::AutoStable;
					AF._flag_IsAltHoldSet = false;
				}
				if (RF._flag_RC_Mid_PWM_Value + 50 > RF._uORB_RC_Out_FlyMod && RF._flag_RC_Mid_PWM_Value - 50 < RF._uORB_RC_Out_FlyMod)
				{
					AF.AutoPilotMode = APModeINFO::AltHold;
				}
				if (RF._flag_RC_Max_PWM_Value + 50 > RF._uORB_RC_Out_FlyMod && RF._flag_RC_Max_PWM_Value - 50 < RF._uORB_RC_Out_FlyMod)
				{
					AF.AutoPilotMode = APModeINFO::PositionHold;
				}
			}
			//flyMode Function
			{
				if (!AF._flag_ESC_ARMED && (AF.AutoPilotMode == APModeINFO::AltHold || AF.AutoPilotMode == APModeINFO::PositionHold))
				{
					AF._flag_IsAltHoldSet = true;
					if (AF.AutoPilotMode == APModeINFO::PositionHold)
					{
						if (RF._uORB_RC_Out__Roll == 0 && RF._uORB_RC_Out_Pitch == 0)
						{
							if (!AF._flag_GPS_Error)
							{
								AF._flag_IsGPSHoldSet = true;
							}
							else
							{
								AF._flag_IsGPSHoldSet = false;
							}
						}
						else
						{
							AF._flag_IsGPSHoldSet = false;
						}
					}
				}
			}

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

void SingleAPMAPI::RPiSingleAPM::PositionTaskReg()
{
	TF.GPSTask = new std::thread([&] {
		while (true)
		{
			TF._Tmp_GPSThreadTimeStart = micros();
			TF._Tmp_GPSThreadTimeNext = TF._Tmp_GPSThreadTimeStart - TF._Tmp_GPSThreadTimeEnd;
			TF._Tmp_GPSThreadSMooth++;
			{

				if (TF._Tmp_GPSThreadSMooth == 10)
				{
					SF._uORB_GPS_Data = GPSInit->GPSParse();
					TF._Tmp_GPSThreadSMooth = 0;
					SF._uORB_GPS_Lat_Diff = (float)(SF._uORB_GPS_Data.lat - SF._uORB_GPS_Lat_Last_Data) / 10.f;
					SF._uORB_GPS_Lng_Diff = (float)(SF._uORB_GPS_Data.lng - SF._uORB_GPS_Lng_Last_Data) / 10.f;
					SF._uORB_GPS_Lat_Smooth = SF._uORB_GPS_Lat_Last_Data;
					SF._uORB_GPS_Lng_Smooth = SF._uORB_GPS_Lng_Last_Data;
					SF._uORB_GPS_Lat_Last_Data = SF._uORB_GPS_Data.lat;
					SF._uORB_GPS_Lng_Last_Data = SF._uORB_GPS_Data.lng;
					SF._uOBR_GPS_Lat_Smooth_Diff = 0;
					SF._uOBR_GPS_Lng_Smooth_Diff = 0;
				}
				//
				if (SF._uORB_GPS_Lat_Last_Data == 0 && SF._uORB_GPS_Lng_Last_Data == 0)
				{
					SF._uORB_GPS_Lat_Last_Data = SF._uORB_GPS_Data.lat;
					SF._uORB_GPS_Lng_Last_Data = SF._uORB_GPS_Data.lng;
				}
				SF._uOBR_GPS_Lat_Smooth_Diff += SF._uORB_GPS_Lat_Diff;
				if (abs(SF._uOBR_GPS_Lat_Smooth_Diff) > 1)
				{
					SF._uORB_GPS_Lat_Smooth += (int)SF._uOBR_GPS_Lat_Smooth_Diff;
					SF._uOBR_GPS_Lat_Smooth_Diff -= (int)SF._uOBR_GPS_Lat_Smooth_Diff;
				}
				SF._uOBR_GPS_Lng_Smooth_Diff += SF._uORB_GPS_Lng_Diff;
				if (abs(SF._uOBR_GPS_Lng_Smooth_Diff) > 1)
				{
					SF._uORB_GPS_Lng_Smooth += (int)SF._uOBR_GPS_Lng_Smooth_Diff;
					SF._uOBR_GPS_Lng_Smooth_Diff -= (int)SF._uOBR_GPS_Lng_Smooth_Diff;
				}
				//
				if (SF._uORB_GPS_Data.satillitesCount < 4)
				{
					AF._flag_GPS_Disconnected = true;
				}
				else if (SF._uORB_GPS_Data.satillitesCount > 4)
				{
					AF._flag_GPS_Disconnected = false;
				}
				else
				{
					AF._flag_GPS_Disconnected = true;
				}
				if (AF._flag_GPS_Disconnected == true)
				{
					AF.GPS_Lose_Clocking += 1;
					if (AF.GPS_Lose_Clocking == 50)
					{
						AF._flag_GPS_Error = true;
						AF.GPS_Lose_Clocking = 0;
					}
				}
				else if (AF._flag_GPS_Disconnected == false)
				{
					AF.GPS_Lose_Clocking = 0;
				}
				//
				AF._flag_GPSData_Async = true;
			}
			TF._Tmp_GPSThreadTimeEnd = micros();
			TF._Tmp_GPSThreadTimeLoop = TF._Tmp_GPSThreadTimeEnd - TF._Tmp_GPSThreadTimeStart;
			if (TF._Tmp_GPSThreadTimeLoop + TF._Tmp_GPSThreadTimeNext > TF._flag_GPSThreadTimeMax | TF._Tmp_GPSThreadTimeNext < 0)
			{
				usleep(50);
				AF._flag_ClockingTime_Error = true;
			}
			else
			{
				usleep(TF._flag_GPSThreadTimeMax - TF._Tmp_GPSThreadTimeLoop - TF._Tmp_GPSThreadTimeNext);
			}
			if (TF._Tmp_GPSThreadTimeLoop + TF._Tmp_GPSThreadTimeNext > TF._Tmp_GPSThreadError)
			{
				TF._Tmp_GPSThreadError = TF._Tmp_GPSThreadTimeLoop;
			}
			TF._Tmp_GPSThreadTimeEnd = micros();
		}
	});

	cpu_set_t cpuset2;
	CPU_ZERO(&cpuset2);
	CPU_SET(3, &cpuset2);
	int rc2 = pthread_setaffinity_np(TF.GPSTask->native_handle(), sizeof(cpu_set_t), &cpuset2);

	TF.MAGTask = new std::thread([&] {
		while (true)
		{
			TF._Tmp_MAGThreadTimeStart = micros();
			TF._Tmp_MAGThreadTimeNext = TF._Tmp_MAGThreadTimeStart - TF._Tmp_MAGThreadTimeEnd;
			{
				GPSMAGInit->GPSI2CCompass_QMC5883LRead(SF._uORB_QMC5883L_M_X, SF._uORB_QMC5883L_M_Y, SF._uORB_QMC5883L_M_Z);
				SF._Tmp_QMC5883L_M_XH = SF._uORB_QMC5883L_M_X * cos(SF._uORB_Real_Pitch * -0.0174533) +
										SF._uORB_QMC5883L_M_Y * sin(SF._uORB_Real_Pitch * 0.0174533) + sin(SF._uORB_Real__Roll * -0.0174533) -
										SF._uORB_QMC5883L_M_Z * cos(SF._uORB_Real_Pitch * 0.0174533) * sin(SF._uORB_Real__Roll * -0.0174533);
				SF._Tmp_QMC5883L_M_YH = SF._uORB_QMC5883L_M_Y * cos(SF._uORB_Real_Pitch * 0.0174533) +
										SF._uORB_QMC5883L_M_Z + sin(SF._uORB_Real_Pitch * 0.0174533);
				if (SF._Tmp_QMC5883L_M_YH < 0)
					SF._uORB_QMC5883L_Head = 180 + (180 + ((atan2(SF._Tmp_QMC5883L_M_YH, SF._Tmp_QMC5883L_M_XH)) * (180 / 3.14)));
				else
					SF._uORB_QMC5883L_Head = (atan2(SF._Tmp_QMC5883L_M_YH, SF._Tmp_QMC5883L_M_XH)) * (180 / 3.14);
				if (SF._uORB_QMC5883L_Head < 0)
					SF._uORB_QMC5883L_Head += 360;
				else if (SF._uORB_QMC5883L_Head >= 360)
					SF._uORB_QMC5883L_Head -= 360;
			}
			TF._Tmp_MAGThreadTimeEnd = micros();
			TF._Tmp_MAGThreadTimeLoop = TF._Tmp_MAGThreadTimeEnd - TF._Tmp_MAGThreadTimeStart;
			if (TF._Tmp_MAGThreadTimeLoop + TF._Tmp_MAGThreadTimeNext > TF._flag_MAGThreadTimeMax | TF._Tmp_MAGThreadTimeNext < 0)
			{
				usleep(50);
				AF._flag_ClockingTime_Error = true;
			}
			else
			{
				usleep(TF._flag_MAGThreadTimeMax - TF._Tmp_MAGThreadTimeLoop - TF._Tmp_MAGThreadTimeNext);
			}
			if (TF._Tmp_MAGThreadTimeLoop + TF._Tmp_MAGThreadTimeNext > TF._Tmp_MAGThreadError)
			{
				TF._Tmp_MAGThreadError = TF._Tmp_MAGThreadTimeLoop;
			}
			TF._Tmp_MAGThreadTimeEnd = micros();
		}
	});
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(3, &cpuset);
	int rc = pthread_setaffinity_np(TF.MAGTask->native_handle(), sizeof(cpu_set_t), &cpuset);
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

void SingleAPMAPI::RPiSingleAPM::TaskThreadBlock()
{
	while (true)
	{
#ifdef DEBUG
		DebugOutPut();
#endif
		SaftyCheckTaskReg();
		usleep(20000);
	}
}

void SingleAPMAPI::RPiSingleAPM::APMCalibrator()
{
	char Comfirm[128];
	std::cout << "[WARNING! WARNING! WARNING! ]\n";
	std::cout << "YOU ARE TRY TO ENABLE CALIBRATION THE ESC\n";
	std::cout << "PLEASE REMOVE ALL THE PROPELLER\n";
	std::cout << "IF YOU STILL NEED TO ENABLE , PLEASE INPUT : YES,DO AS I SAY , AND ENTER\n";
	std::cout << "(YES,DO AS I SAY)";
	std::cin.getline(Comfirm, sizeof(Comfirm));
	if (strncmp(Comfirm, "YES,DO AS I SAY", 16) == 0)
	{
		std::cout << "\nALL ESC WILL PULL TO MAX,IF THE ESC RING, PLEASE INPUT : CHEACK , AND ENTER\n";
		pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A1_Pin, 0, EF._Flag_Max__Throttle);
		pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A2_Pin, 0, EF._Flag_Max__Throttle);
		pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B1_Pin, 0, EF._Flag_Max__Throttle);
		pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B2_Pin, 0, EF._Flag_Max__Throttle);
		std::cout << "(CHECK)";
		std::cin >> Comfirm;
		if (strncmp(Comfirm, "CHECK", 6) == 0)
		{
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A1_Pin, 0, EF._Flag_Lazy_Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A2_Pin, 0, EF._Flag_Lazy_Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B1_Pin, 0, EF._Flag_Lazy_Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B2_Pin, 0, EF._Flag_Lazy_Throttle);
			std::cout << "\nESC CALIBRATION COMPLETE\n";
		}
		else
		{
			std::cout << "\nABORT!\n";
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A1_Pin, 0, EF._Flag_Lock_Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A2_Pin, 0, EF._Flag_Lock_Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B1_Pin, 0, EF._Flag_Lock_Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B2_Pin, 0, EF._Flag_Lock_Throttle);
		}
	}
	else
	{
		std::cout << "\nABORT!\n";
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

void SingleAPMAPI::RPiSingleAPM::PIDSoomth_Caculate(float TargetData, float inputData, float &outputData,
													float &Last_I_Data, float &Total_D_Data, float &Last_D_Data, float (&Ava_D_Data)[30],
													float P_Gain, float I_Gain, float D_Gain, float outputMax, bool StartPIDFlag)
{
	float Diff = inputData - TargetData;
	outputData = Diff * PF._flag_PID_P_Alt_Gain;

	if (!StartPIDFlag)
		Last_D_Data = inputData * 10.f;
	Total_D_Data -= Ava_D_Data[PF._flag_PID_SOOMTH_Clock];
	Ava_D_Data[PF._flag_PID_SOOMTH_Clock] = inputData * 10.f - Last_D_Data;
	Total_D_Data += Ava_D_Data[PF._flag_PID_SOOMTH_Clock];
	Last_D_Data = inputData * 10.f;
	PF._flag_PID_SOOMTH_Clock++;
	PF._flag_PID_SOOMTH_Clock = PF._flag_PID_SOOMTH_Clock == 30 ? 0 : PF._flag_PID_SOOMTH_Clock;

	Last_I_Data += (I_Gain / 100.0) * Diff;
	Last_I_Data = Last_I_Data > 300 ? 300 : Last_I_Data;
	Last_I_Data = Last_I_Data < -300 ? -300 : Last_I_Data;

	outputData += Total_D_Data * D_Gain + Last_I_Data;
	outputData = outputData > outputMax ? outputMax : outputData;
	outputData = outputData < -outputMax ? -outputMax : outputData;

	outputData = StartPIDFlag ? outputData : 0;
}

void SingleAPMAPI::RPiSingleAPM::PIDINC_Caculate(float TargetData, float inputData, float &outputData,
												 float &LastError, float &PrevError,
												 float P_Gain, float I_Gain, float D_Gain, float outputMax)
{
	int32_t iError;
	iError = TargetData - inputData;
	outputData = (P_Gain * iError) - (I_Gain * LastError) + (D_Gain * PrevError);
	PrevError = LastError;
	LastError = iError;
	outputData = outputData > outputMax ? outputMax : outputData;
	outputData = outputData < -outputMax ? -outputMax : outputData;
}

void SingleAPMAPI::RPiSingleAPM::ConfigReader(APMSettinngs APMInit)
{
	//==========================================================Device Type=======/
	RF.RC_Type = APMInit.RC_Type;
	SF.MPU9250_Type = APMInit.MPU9250_Type;
	SF.IMUFilter_Type = APMInit.IMUFilter_Type;
	SF.IMUMixFilter_Type = APMInit.IMUMixFilter_Type;
	//==========================================================Controller cofig==/
	RF._flag_RC_ARM_PWM_Value = APMInit._flag_RC_ARM_PWM_Value;
	RF._flag_RC_Min_PWM_Value = APMInit._flag_RC_Min_PWM_Value;
	RF._flag_RC_Mid_PWM_Value = APMInit._flag_RC_Mid_PWM_Value;
	RF._flag_RC_Max_PWM_Value = APMInit._flag_RC_Max_PWM_Value;

	RF._flag_RCIsReserv__Roll = APMInit._flag_RCIsReserv__Roll;
	RF._flag_RCIsReserv_Pitch = APMInit._flag_RCIsReserv_Pitch;
	RF._flag_RCIsReserv___Yaw = APMInit._flag_RCIsReserv___Yaw;
	//==========================================================ESC cofig=========/
	EF._flag_A1_Pin = APMInit._flag_A1_Pin;
	EF._flag_A2_Pin = APMInit._flag_A2_Pin;
	EF._flag_B1_Pin = APMInit._flag_B1_Pin;
	EF._flag_B2_Pin = APMInit._flag_B2_Pin;
	//==================================================================PID cofig==/
	PF._flag_PID_P__Roll_Gain = APMInit._flag_PID_P__Roll_Gain;
	PF._flag_PID_P_Pitch_Gain = APMInit._flag_PID_P_Pitch_Gain;
	PF._flag_PID_P___Yaw_Gain = APMInit._flag_PID_P___Yaw_Gain;
	PF._flag_PID_P_Alt_Gain = APMInit._flag_PID_P_Alt_Gain;
	PF._flag_PID_P_GPS_Gain = APMInit._flag_PID_P_GPS_Gain;

	PF._flag_PID_I__Roll_Gain = APMInit._flag_PID_I__Roll_Gain;
	PF._flag_PID_I_Pitch_Gain = APMInit._flag_PID_I_Pitch_Gain;
	PF._flag_PID_I___Yaw_Gain = APMInit._flag_PID_I___Yaw_Gain;
	PF._flag_PID_I_Alt_Gain = APMInit._flag_PID_I_Alt_Gain;
	PF._flag_PID_I__Roll_Max__Value = APMInit._flag_PID_I__Roll_Max__Value;
	PF._flag_PID_I_Pitch_Max__Value = APMInit._flag_PID_I_Pitch_Max__Value;
	PF._flag_PID_I___Yaw_Max__Value = APMInit._flag_PID_I___Yaw_Max__Value;

	PF._flag_PID_D__Roll_Gain = APMInit._flag_PID_D__Roll_Gain;
	PF._flag_PID_D_Pitch_Gain = APMInit._flag_PID_D_Pitch_Gain;
	PF._flag_PID_D___Yaw_Gain = APMInit._flag_PID_D___Yaw_Gain;
	PF._flag_PID_D_Alt_Gain = APMInit._flag_PID_D_Alt_Gain;
	PF._flag_PID_D_GPS_Gain = APMInit._flag_PID_D_GPS_Gain;

	PF._flag_PID_Hover_Throttle = APMInit._flag_PID_Hover_Throttle;
	PF._flag_PID_Level_Max = APMInit._flag_PID_Level_Max;
	PF._flag_PID_Alt_Level_Max = APMInit._flag_PID_Alt_Level_Max;
	PF._flag_PID_GPS_Level_Max = APMInit._flag_PID_GPS_Level_Max;
	//==============================================================Sensors cofig==/
	SF._flag_Accel__Roll_Cali = APMInit._flag_Accel__Roll_Cali;
	SF._flag_Accel_Pitch_Cali = APMInit._flag_Accel_Pitch_Cali;

	SF._flag_MPU9250_M_X_Scaler = APMInit._flag_MPU9250_M_X_Scaler;
	SF._flag_MPU9250_M_Y_Scaler = APMInit._flag_MPU9250_M_Y_Scaler;
	SF._flag_MPU9250_M_Z_Scaler = APMInit._flag_MPU9250_M_Z_Scaler;

	SF._flag_MPU9250_Head_Asix = APMInit._flag_MPU9250_Head_Asix;
	//===============================================================Update cofig==/
	TF._flag_IMUThreadFreq = APMInit.IMU_Freqeuncy;
	TF._flag_IMUThreadTimeMax = (float)1 / TF._flag_IMUThreadFreq * 1000000;
	TF._flag_RXTThreadFreq = APMInit.RXT_Freqeuncy;
	TF._flag_RXTThreadTimeMax = (float)1 / TF._flag_RXTThreadFreq * 1000000;
	TF._flag_ESCThreadFreq = APMInit.ESC_Freqeuncy;
	TF._flag_ESCThreadTimeMax = (float)1 / TF._flag_ESCThreadFreq * 1000000;
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
		wiringPiSPIDataRW(DF.MPU9250_SPI_Channel, SF._Tmp_MPU9250_SPI_Buffer, 21);
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

		SF._Tmp_MPU9250_M_X = ((int)SF._Tmp_MPU9250_SPI_Buffer[16] << 8) | (int)SF._Tmp_MPU9250_SPI_Buffer[15];
		SF._uORB_MPU9250_M_X = (short)SF._Tmp_MPU9250_M_X;
		SF._Tmp_MPU9250_M_Y = ((int)SF._Tmp_MPU9250_SPI_Buffer[18] << 8) | (int)SF._Tmp_MPU9250_SPI_Buffer[17];
		SF._uORB_MPU9250_M_Y = (short)SF._Tmp_MPU9250_M_Y;
		SF._Tmp_MPU9250_M_Z = ((int)SF._Tmp_MPU9250_SPI_Buffer[20] << 8) | (int)SF._Tmp_MPU9250_SPI_Buffer[19];
		SF._uORB_MPU9250_M_Z = (short)SF._Tmp_MPU9250_M_Z;
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
		yv[2] = (xv[0] + xv[2]) + 2 * xv[1] + (-0.6413515381 * yv[0]) + (1.5610180758 * yv[1]);
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

void SingleAPMAPI::RPiSingleAPM::AttitudeUpdateTask()
{
	//PID Checking
	if (AF._flag_ESC_ARMED == true)
	{
		AF._flag_MPU9250_first_StartUp = true;
		PF._uORB_PID_D_Last_Value__Roll = 0;
		PF._uORB_PID_D_Last_Value_Pitch = 0;
		PF._uORB_PID_D_Last_Value___Yaw = 0;
		PF._uORB_PID_I_Last_Value__Roll = 0;
		PF._uORB_PID_I_Last_Value_Pitch = 0;
		PF._uORB_PID_I_Last_Value___Yaw = 0;
	}

	if (AF.AutoPilotMode == APModeINFO::AltHold ||
		AF.AutoPilotMode == APModeINFO::AutoStable ||
		AF.AutoPilotMode == APModeINFO::PositionHold)
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

		//AltHold Caculate
		{
			if (AF._flag_MS5611_Async)
			{
				if (!AF._flag_IsAltHoldSet)
				{
					PF._uORB_PID_I_Last_Value_Alt = 0;
					PF._uORB_PID_AltHold_Target = PF._uORB_PID_AltInput;
				}
				PF._uORB_PID_AltInput = SF._uORB_MS5611_PressureFinal;
				PIDSoomth_Caculate(PF._uORB_PID_AltHold_Target, PF._uORB_PID_AltInput, PF._uORB_PID_Alt_Throttle,
								   PF._uORB_PID_I_Last_Value_Alt, PF._uORB_PID_D_Toat_Value_Alt, PF._uORB_PID_D_Last_Value_Alt, PF._Tmp_PID_D_Alt_Var,
								   PF._flag_PID_P_Alt_Gain, PF._flag_PID_I_Alt_Gain, PF._flag_PID_D_Alt_Gain,
								   PF._flag_PID_Alt_Level_Max, AF._flag_IsAltHoldSet);
				AF._flag_MS5611_Async = false;
			}
		}

		//Position Caculate
		{
			if (AF._flag_GPSData_Async)
			{
				if (!AF._flag_IsGPSHoldSet)
				{
					PF._uORB_PID_GPS_Lat_Local_Target = SF._uORB_GPS_Lat_Smooth;
					PF._uORB_PID_GPS_Lng_Local_Target = SF._uORB_GPS_Lng_Smooth;
				}

				PF._uORB_PID_GPS_Lng_Local_Diff = SF._uORB_GPS_Lng_Smooth - PF._uORB_PID_GPS_Lng_Local_Target;
				PF._uORB_PID_GPS_Lat_Local_Diff = PF._uORB_PID_GPS_Lat_Local_Target - SF._uORB_GPS_Lat_Smooth;

				PF._uORB_PID_D_GPS_Lat_Ouput -= PF._Tmp_PID_D_GPS_Lat_AvaData[PF._Tmp_PID_D_GPS_AvaClock];
				PF._Tmp_PID_D_GPS_Lat_AvaData[PF._Tmp_PID_D_GPS_AvaClock] = PF._uORB_PID_GPS_Lat_Local_Diff - PF._uORB_PID_D_GPS_Lat_LastValue;
				PF._uORB_PID_D_GPS_Lat_Ouput += PF._Tmp_PID_D_GPS_Lat_AvaData[PF._Tmp_PID_D_GPS_AvaClock];

				PF._uORB_PID_D_GPS_Lng_Ouput -= PF._Tmp_PID_D_GPS_Lng_AvaData[PF._Tmp_PID_D_GPS_AvaClock];
				PF._Tmp_PID_D_GPS_Lng_AvaData[PF._Tmp_PID_D_GPS_AvaClock] = PF._uORB_PID_GPS_Lng_Local_Diff - PF._uORB_PID_D_GPS_Lng_LastValue;
				PF._uORB_PID_D_GPS_Lng_Ouput += PF._Tmp_PID_D_GPS_Lng_AvaData[PF._Tmp_PID_D_GPS_AvaClock];
				PF._Tmp_PID_D_GPS_AvaClock++;
				if (PF._Tmp_PID_D_GPS_AvaClock == 35)
				{
					PF._Tmp_PID_D_GPS_AvaClock = 0;
				}

				PF._uORB_PID_D_GPS_Lat_LastValue = PF._uORB_PID_GPS_Lat_Local_Diff;
				PF._uORB_PID_D_GPS_Lng_LastValue = PF._uORB_PID_GPS_Lng_Local_Diff;

				PF._uORB_PID_GPS_Lat_Ouput = (float)PF._uORB_PID_GPS_Lat_Local_Diff * PF._flag_PID_P_GPS_Gain + PF._uORB_PID_D_GPS_Lat_Ouput * PF._flag_PID_D_GPS_Gain;
				PF._uORB_PID_GPS_Lng_Ouput = (float)PF._uORB_PID_GPS_Lng_Local_Diff * PF._flag_PID_P_GPS_Gain + PF._uORB_PID_D_GPS_Lng_Ouput * PF._flag_PID_D_GPS_Gain;

				if (AF.AutoPilotMode == APModeINFO::PositionHold && AF._flag_IsGPSHoldSet)
				{
					PF._uORB_PID_GPS__Roll_Ouput = PF._uORB_PID_GPS_Lat_Ouput * cos(SF._uORB_Real__Head * 0.017453) + PF._uORB_PID_GPS_Lng_Ouput * cos((SF._uORB_Real__Head - 90.f) * 0.017453);
					PF._uORB_PID_GPS_Pitch_Ouput = PF._uORB_PID_GPS_Lng_Ouput * cos(SF._uORB_Real__Head * 0.017453) + PF._uORB_PID_GPS_Lat_Ouput * cos((SF._uORB_Real__Head + 90.f) * 0.017453);

					PF._uORB_PID_GPS__Roll_Ouput = PF._uORB_PID_GPS__Roll_Ouput > PF._flag_PID_GPS_Level_Max ? PF._flag_PID_GPS_Level_Max : PF._uORB_PID_GPS__Roll_Ouput;
					PF._uORB_PID_GPS__Roll_Ouput = PF._uORB_PID_GPS__Roll_Ouput < -1 * PF._flag_PID_GPS_Level_Max ? -1 * PF._flag_PID_GPS_Level_Max : PF._uORB_PID_GPS__Roll_Ouput;
					PF._uORB_PID_GPS_Pitch_Ouput = PF._uORB_PID_GPS_Pitch_Ouput > PF._flag_PID_GPS_Level_Max ? PF._flag_PID_GPS_Level_Max : PF._uORB_PID_GPS_Pitch_Ouput;
					PF._uORB_PID_GPS_Pitch_Ouput = PF._uORB_PID_GPS_Pitch_Ouput < -1 * PF._flag_PID_GPS_Level_Max ? -1 * PF._flag_PID_GPS_Level_Max : PF._uORB_PID_GPS_Pitch_Ouput;
				}
				else
				{
					PF._uORB_PID_GPS__Roll_Ouput = 0;
					PF._uORB_PID_GPS_Pitch_Ouput = 0;
				}
				AF._flag_GPSData_Async = false;
			}
		}
	}
	//ESC Caculate
	if (AF.AutoPilotMode == APModeINFO::AltHold ||
		AF.AutoPilotMode == APModeINFO::PositionHold)
	{
		EF._Tmp_B1_Speed = PF._flag_PID_Hover_Throttle + PF._uORB_PID_Alt_Throttle - PF._uORB_Leveling__Roll + PF._uORB_Leveling_Pitch + PF._uORB_Leveling___Yaw - PF._uORB_PID_GPS__Roll_Ouput + PF._uORB_PID_GPS_Pitch_Ouput;
		EF._Tmp_A1_Speed = PF._flag_PID_Hover_Throttle + PF._uORB_PID_Alt_Throttle - PF._uORB_Leveling__Roll - PF._uORB_Leveling_Pitch - PF._uORB_Leveling___Yaw - PF._uORB_PID_GPS__Roll_Ouput - PF._uORB_PID_GPS_Pitch_Ouput;
		EF._Tmp_A2_Speed = PF._flag_PID_Hover_Throttle + PF._uORB_PID_Alt_Throttle + PF._uORB_Leveling__Roll - PF._uORB_Leveling_Pitch + PF._uORB_Leveling___Yaw + PF._uORB_PID_GPS__Roll_Ouput - PF._uORB_PID_GPS_Pitch_Ouput;
		EF._Tmp_B2_Speed = PF._flag_PID_Hover_Throttle + PF._uORB_PID_Alt_Throttle + PF._uORB_Leveling__Roll + PF._uORB_Leveling_Pitch - PF._uORB_Leveling___Yaw + PF._uORB_PID_GPS__Roll_Ouput + PF._uORB_PID_GPS_Pitch_Ouput;
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

	if (EF._uORB_A1_Speed < EF._Flag_Lazy_Throttle)
		EF._uORB_A1_Speed = EF._Flag_Lazy_Throttle;
	if (EF._uORB_A2_Speed < EF._Flag_Lazy_Throttle)
		EF._uORB_A2_Speed = EF._Flag_Lazy_Throttle;
	if (EF._uORB_B1_Speed < EF._Flag_Lazy_Throttle)
		EF._uORB_B1_Speed = EF._Flag_Lazy_Throttle;
	if (EF._uORB_B2_Speed < EF._Flag_Lazy_Throttle)
		EF._uORB_B2_Speed = EF._Flag_Lazy_Throttle;

	if (EF._uORB_A1_Speed > EF._Flag_Max__Throttle)
		EF._uORB_A1_Speed = EF._Flag_Max__Throttle;
	if (EF._uORB_A2_Speed > EF._Flag_Max__Throttle)
		EF._uORB_A2_Speed = EF._Flag_Max__Throttle;
	if (EF._uORB_B1_Speed > EF._Flag_Max__Throttle)
		EF._uORB_B1_Speed = EF._Flag_Max__Throttle;
	if (EF._uORB_B2_Speed > EF._Flag_Max__Throttle)
		EF._uORB_B2_Speed = EF._Flag_Max__Throttle;
}

void SingleAPMAPI::RPiSingleAPM::SaftyCheckTaskReg()
{
	if (AF._flag_Error == true)
	{
		AF._flag_ESC_ARMED = true;
	}
}

void SingleAPMAPI::RPiSingleAPM::DebugOutPut()
{
	std::cout << "\033[200A";
	std::cout << "\033[K";
	std::cout << "FlyMode:\n";
	if (AF.AutoPilotMode == APModeINFO::AutoStable)
	{
		std::cout << " AutoStable        ";
	}
	else if (AF.AutoPilotMode == APModeINFO::AltHold)
	{
		std::cout << " AltHold           ";
	}
	else if (AF.AutoPilotMode == APModeINFO::PositionHold)
	{
		std::cout << " PositionHold      ";
	}
	std::cout << "\n";
	std::cout << "ESCSpeedOutput:"
			  << " \n";
	std::cout << " A1 " << EF._uORB_A1_Speed << "    "
			  << " A2 " << EF._uORB_A2_Speed << "                        "
			  << "\n";
	std::cout << " B1 " << EF._uORB_B1_Speed << "    "
			  << " B2 " << EF._uORB_B2_Speed << "                        "
			  << "\n";

	std::cout << "IMUSenorData: "
			  << " \n";
	std::cout << " GryoPitch:  " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Gryo_Pitch << "    "
			  << " GryoRoll:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Gryo__Roll << "    "
			  << " GryoYaw:    " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Gryo___Yaw
			  << "                        "
			  << std::endl;
	;
	std::cout << " AccePitch:  " << std::setw(7) << std::setfill(' ') << (int)(SF._uORB_Accel_Pitch) << "    "
			  << " AcceRoll:   " << std::setw(7) << std::setfill(' ') << (int)(SF._uORB_Accel__Roll) << "    "
			  << " ClimbeRate: " << std::setw(7) << std::setfill(' ') << (int)(SF._uORB_MPU9250_A_Z)
			  << "                        "
			  << std::endl;
	std::cout << " RealPitch:  " << std::setw(7) << std::setfill(' ') << (int)(SF._uORB_Real_Pitch) << "    "
			  << " RealRoll:   " << std::setw(7) << std::setfill(' ') << (int)(SF._uORB_Real__Roll) << "    "
			  << " RealYaw:    " << std::setw(7) << std::setfill(' ') << (int)(SF._uORB_Real__Head)
			  << "                        "
			  << std::endl;
	std::cout << " CompassX:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU9250_M_X << "    "
			  << " CompassY:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU9250_M_Y << "    "
			  << " CompassZ:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU9250_M_Z << "    "
			  << " CHeading:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MAG_Heading << "    "
			  << "                        "
			  << std::endl;
	std::cout << " GCompassX:  " << std::setw(7) << std::setfill(' ') << SF._uORB_QMC5883L_M_X << "    "
			  << " GCompassY:  " << std::setw(7) << std::setfill(' ') << SF._uORB_QMC5883L_M_Y << "    "
			  << " GCompassZ:  " << std::setw(7) << std::setfill(' ') << SF._uORB_QMC5883L_M_Z << "    "
			  << " GCHeading:  " << std::setw(7) << std::setfill(' ') << SF._uORB_QMC5883L_Head << "    "
			  << "                        "
			  << std::endl;

	std::cout
		<< "MS5611ParseDataINFO:"
		<< "\n";
	std::cout << " FastPressure :      " << SF._uORB_MS5611_PressureFast << "            \n";
	std::cout << " FilterPressureFast :" << SF._uORB_MS5611_PressureFinal << "            \n";
	std::cout << " AltHoldTarget:      " << PF._uORB_PID_AltHold_Target << "            \n";
	std::cout << " AltholdThrottle:    " << PF._uORB_PID_Alt_Throttle << "            \n";
	std::cout << " Althold_I_Ouput:    " << PF._uORB_PID_I_Last_Value_Alt << "            \n";

	std::cout << "GPSDataINFO:"
			  << "\n";
	std::cout << " GPSLAT:     " << std::setw(10) << std::setfill(' ') << (int)SF._uORB_GPS_Lat_Smooth
			  << " ||GPSHoldLAT: " << std::setw(10) << std::setfill(' ') << PF._uORB_PID_GPS_Lat_Local_Target
			  << " ||GPSPitchOut:" << std::setw(10) << std::setfill(' ') << PF._uORB_PID_GPS_Pitch_Ouput << "            \n";
	std::cout << " GPSLNG:     " << std::setw(10) << std::setfill(' ') << (int)SF._uORB_GPS_Lng_Smooth
			  << " ||GPSHoldLNG: " << std::setw(10) << std::setfill(' ') << PF._uORB_PID_GPS_Lng_Local_Target
			  << " ||GPSRollOut: " << std::setw(10) << std::setfill(' ') << PF._uORB_PID_GPS__Roll_Ouput << "            \n";
	std::cout << " GPSNE:      " << SF._uORB_GPS_Data.lat_North_Mode << " -> " << SF._uORB_GPS_Data.lat_East_Mode << "            \n";
	std::cout << " GPSSATCount:" << SF._uORB_GPS_Data.satillitesCount << "            \n";

	std::cout << "RCOutPUTINFO:   "
			  << "\n";
	std::cout << " ChannelRoll  "
			  << ": " << RF._uORB_RC_Out__Roll << std::setw(10) << std::setfill(' ') << "\n";
	std::cout << " ChannelPitch "
			  << ": " << RF._uORB_RC_Out_Pitch << std::setw(10) << std::setfill(' ') << "\n";
	std::cout << " ChannelThrot "
			  << ": " << RF._uORB_RC_Out_Throttle << std::setw(10) << std::setfill(' ') << "\n";
	std::cout << " ChannelYaw   "
			  << ": " << RF._uORB_RC_Out___Yaw << std::setw(10) << std::setfill(' ') << " \n";

	std::cout << "ChannelINFO: "
			  << " \n";
	for (size_t i = 0; i < 16; i++)
	{
		std::cout << " " << RF._uORB_RC_Channel_PWM[i] << " ";
	}
	std::cout << " \n\n";

	std::cout << " Flag_ESC_ARMED:" << AF._flag_ESC_ARMED << "               \n";
	std::cout << " Flag_Error:" << AF._flag_Error << "           \n";
	std::cout << " Flag_GPS_Error:" << AF._flag_GPS_Error << "           \n";
	std::cout << " Flag_ClockingTime_Error:" << AF._flag_ClockingTime_Error << "         \n";
	std::cout << " Flag_RC_Disconnected:" << AF._flag_RC_Disconnected << "         \n";
	std::cout << " Flag_GPS_Disconnected:" << AF._flag_GPS_Disconnected << "         \n";
	std::cout << " Flag_IsAltHoldSet:" << AF._flag_IsAltHoldSet << "         \n";
	std::cout << " Flag_IsGPSHoldSet:" << AF._flag_IsGPSHoldSet << "         \n";
	std::cout << " GPS_Lose_Clocking:" << AF.GPS_Lose_Clocking << "         \n";
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
	std::cout << " GPSLoopTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_GPSThreadTimeLoop;
	std::cout << " GPSNextTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_GPSThreadTimeNext;
	std::cout << " GPSMaxTime:  " << std::setw(7) << std::setfill(' ') << TF._Tmp_GPSThreadError << "    \n";
	std::cout << " MAGLoopTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_MAGThreadTimeLoop;
	std::cout << " MAGNextTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_MAGThreadTimeNext;
	std::cout << " MAGMaxTime:  " << std::setw(7) << std::setfill(' ') << TF._Tmp_MAGThreadError << "    \n"
			  << std::endl;
}