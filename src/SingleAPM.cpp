#include "SingleAPM.hpp"

int SingleAPMAPI::RPiSingleAPM::RPiSingleAPMInit(APMSettinngs APMInit)
{
	wiringPiSetupSys();
	piHiPri(99);
	{
		AF.RC_Lose_Clocking = 0;
		AF.GPS_Lose_Clocking = 0;
		AF._flag_MPU9250_first_StartUp = true;
		AF._flag_ESC_ARMED = true;
		AF.AutoPilotMode = APModeINFO::AutoStable;
	}
	ConfigReader(APMInit);
	//--------------------------------------------------------------------//
	{
#ifdef RPiDEBUGStart
		std::cout << "[RPiSingleAPM]ESCControllerIniting \n";
#endif
		if (DF.PCA9658_fd == -1)
			DF.PCA9658_fd = pca9685Setup(DF.PCA9685_PinBase, DF.PCA9685_Address, DF.PWM_Freq);
		else
			pca9685PWMReset(DF.PCA9658_fd);
		if (DF.PCA9658_fd == -1)
		{
#ifdef RPiDEBUGStart
			std::cout << "[RPiSingleAPM]ESCControllerInitFailed \n";
#endif
			return -1;
		}
	}
	//--------------------------------------------------------------------//
	{
		if (RF.RC_Type == RCIsIbus)
		{
#ifdef RPiDEBUGStart
			std::cout << "[RPiSingleAPM]Controller Ibus config comfirm\n";
#endif
			IbusInit = new Ibus(DF.RCDevice.c_str());
		}
		else if (RF.RC_Type == RCIsSbus)
		{
#ifdef RPiDEBUGStart
			std::cout << "[RPiSingleAPM]Controller Sbus config comfirm\n";
#endif
			SbusInit = new Sbus(DF.RCDevice.c_str());
		}
	}
	//--------------------------------------------------------------------//
	{
		if (DF._IsMS5611Enable)
		{
#ifdef RPiDEBUGStart
			std::cout << "[RPiSingleAPM]Checking MS5611 ... ";
			std::cout.flush();
#endif
			MS5611S = new MS5611();
			if (!MS5611S->MS5611Init())
			{
#ifdef RPiDEBUGStart
				std::cout << "[RPiSingleAPM]MS5611InitError \n";
#endif
			}
			else
			{
				MS5611S->LocalPressureSetter(0, 5);
				for (size_t i = 0; i < 40; i++)
				{
					MS5611S->MS5611FastReader(SF._Tmp_MS5611_Data);
					SF._Tmp_MS5611_Pressure = SF._Tmp_MS5611_Data[0] * 100.0;
					SF._Tmp_MS5611_AvaTotal -= SF._Tmp_MS5611_AvaData[SF._Tmp_MS5611_AvaClock];
					SF._Tmp_MS5611_AvaData[SF._Tmp_MS5611_AvaClock] = SF._Tmp_MS5611_Pressure;
					SF._Tmp_MS5611_AvaTotal += SF._Tmp_MS5611_AvaData[SF._Tmp_MS5611_AvaClock];
					SF._Tmp_MS5611_AvaClock++;
					if (SF._Tmp_MS5611_AvaClock == 20)
						SF._Tmp_MS5611_AvaClock = 0;
					SF._Tmp_MS5611_PressureFast = SF._Tmp_MS5611_AvaTotal / 20.0;
					SF._Tmp_MS5611_PressureFill = SF._Tmp_MS5611_PressureFast;
				}
				MS5611S->LocalPressureSetter(SF._Tmp_MS5611_PressureFill, 5);
#ifdef RPiDEBUGStart
				std::cout << "Done! LocalPressure Is: " << SF._Tmp_MS5611_PressureFill << "\n";
#endif
			}
		}
	}
	//--------------------------------------------------------------------//
	{
		if (DF._IsGPSEnable)
		{

#ifdef RPiDEBUGStart
			std::cout << "[RPiSingleAPM]Waiting for GPS Data ... ";
			std::cout.flush();
#endif
			GPSInit = new GPSUart(DF.GPSDevice.c_str());
			GPSMAGInit = new GPSI2CCompass_QMC5883L();
			GPSMAGInit->GPSI2CCompass_QMC5883LInit();
#ifdef RPiDEBUGStart
			std::cout << "Done \n";
#endif
		}
		//--------------------------------------------------------------------//
		if (DF._IsFlowEnable)
		{
#ifdef RPiDEBUGStart
			std::cout << "[RPiSingleAPM]Setting UP FlowSensor... ";
			std::cout.flush();
#endif
			FlowInit = new MSPUartFlow(DF.FlowDevice.c_str());
#ifdef RPiDEBUGStart
			std::cout << "Done \n";
#endif
		}
	}
	//--------------------------------------------------------------------//
	MPUDevice = new RPiMPU9250(SF.MPU9250_Type, false, 1, DF.MPU9250_ADDR,
							   TF._flag_IMUThreadFreq, SF.IMUMixFilter_Type);
	MPUDevice->MPUGryoCalibration();
	return 0;
}

void SingleAPMAPI::RPiSingleAPM::IMUSensorsTaskReg()
{
	TF.IMUTask = new std::thread([&] {
		while (true)
		{
			TF._Tmp_IMUThreadTimeStart = micros();
			TF._Tmp_IMUThreadTimeNext = TF._Tmp_IMUThreadTimeStart - TF._Tmp_IMUThreadTimeEnd;

			SF._uORB_MPU_Data = MPUDevice->MPUSensorsDataGet();

			if (AF._flag_MPU9250_first_StartUp)
			{
				MPUDevice->ResetMPUMixAngle();
				AF._flag_MPU9250_first_StartUp = false;
			}
			//IMU SaftyChecking---------------------------------------------------------//
			if (SF._uORB_MPU_Data->_uORB_Real_Pitch > 70.0 || SF._uORB_MPU_Data->_uORB_Real_Pitch < -70.0 || SF._uORB_MPU_Data->_uORB_Real__Roll > 70.0 || SF._uORB_MPU_Data->_uORB_Real__Roll < -70.0)
			{
				AF._flag_Error = true;
			}
			AttitudeUpdateTask();

			TF._Tmp_IMUThreadTimeEnd = micros();
			TF._Tmp_IMUThreadTimeLoop = TF._Tmp_IMUThreadTimeEnd - TF._Tmp_IMUThreadTimeStart;
			if (TF._Tmp_IMUThreadTimeLoop + TF._Tmp_IMUThreadTimeNext > TF._flag_IMUThreadTimeMax | TF._Tmp_IMUThreadTimeNext < 0)
			{
				usleep(50);
				TF._flag_IMUErrorTimes++;
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
		}
	});
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(3, &cpuset);
	int rc = pthread_setaffinity_np(TF.IMUTask->native_handle(), sizeof(cpu_set_t), &cpuset);
}

void SingleAPMAPI::RPiSingleAPM::AltholdSensorsTaskReg()
{
	if (DF._IsMS5611Enable)
	{
		TF.ALTTask = new std::thread([&] {
			while (true)
			{
				TF._Tmp_ALTThreadTimeStart = micros();
				TF._Tmp_ALTThreadTimeNext = TF._Tmp_ALTThreadTimeStart - TF._Tmp_ALTThreadTimeEnd;

				SF._Tmp_MS5611_Error = MS5611S->MS5611FastReader(SF._Tmp_MS5611_Data);
				if (SF._Tmp_MS5611_Error < 0)
					TF._flag_ALTErrorTimes++;
				SF._Tmp_MS5611_Pressure = SF._Tmp_MS5611_Data[0] * 100.0;
				SF._Tmp_MS5611_AvaTotal -= SF._Tmp_MS5611_AvaData[SF._Tmp_MS5611_AvaClock];
				SF._Tmp_MS5611_AvaData[SF._Tmp_MS5611_AvaClock] = SF._Tmp_MS5611_Pressure;
				SF._Tmp_MS5611_AvaTotal += SF._Tmp_MS5611_AvaData[SF._Tmp_MS5611_AvaClock];
				SF._Tmp_MS5611_AvaClock++;
				if (SF._Tmp_MS5611_AvaClock == 20)
					SF._Tmp_MS5611_AvaClock = 0;
				SF._Tmp_MS5611_PressureFast = SF._Tmp_MS5611_AvaTotal / 20.0;
				SF._Tmp_MS5611_PressureFill = SF._flag_MS5611_FilterAlpha * SF._Tmp_MS5611_PressureFill + (1.0 - SF._flag_MS5611_FilterAlpha) * SF._Tmp_MS5611_PressureFast;
				SF._Tmp_MS5611_PressureDiff = SF._Tmp_MS5611_PressureFill - SF._Tmp_MS5611_PressureFast;
				if (SF._Tmp_MS5611_PressureDiff > 8)
					SF._Tmp_MS5611_PressureDiff = 8;
				if (SF._Tmp_MS5611_PressureDiff < -8)
					SF._Tmp_MS5611_PressureDiff = -8;
				if (SF._Tmp_MS5611_PressureDiff > 1 || SF._Tmp_MS5611_PressureDiff < -1)
					SF._Tmp_MS5611_PressureFill -= SF._Tmp_MS5611_PressureDiff / 6.0;
				SF._uORB_MS5611_PressureFinal = SF._Tmp_MS5611_PressureFill;
				AF._flag_MS5611_Async = true;

				TF._Tmp_ALTThreadTimeEnd = micros();
				TF._Tmp_ALTThreadTimeLoop = TF._Tmp_ALTThreadTimeEnd - TF._Tmp_ALTThreadTimeStart;
				TF._Tmp_ALTThreadTimeLoop = TF._Tmp_ALTThreadTimeEnd - TF._Tmp_ALTThreadTimeStart;
				if (TF._Tmp_ALTThreadTimeLoop + TF._Tmp_ALTThreadTimeNext > TF._flag_ALTThreadTimeMax | TF._Tmp_ALTThreadTimeNext < 0)
				{
					usleep(50);
					TF._flag_ALTErrorTimes++;
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
				else if (RF._flag_RC_Mid_PWM_Value + 50 > RF._uORB_RC_Out_FlyMod && RF._flag_RC_Mid_PWM_Value - 50 < RF._uORB_RC_Out_FlyMod && DF._IsMS5611Enable)
				{
					AF.AutoPilotMode = APModeINFO::AltHold;
				}
				else if (RF._flag_RC_Max_PWM_Value + 50 > RF._uORB_RC_Out_FlyMod && RF._flag_RC_Max_PWM_Value - 50 < RF._uORB_RC_Out_FlyMod &&
						 DF._IsGPSEnable && DF._IsMS5611Enable)
				{
					AF.AutoPilotMode = APModeINFO::PositionHold;
				}
				else
				{
					AF.AutoPilotMode = APModeINFO::AutoStable;
				}
			}
			//flyMode Function
			{
				if (!AF._flag_ESC_ARMED && (AF.AutoPilotMode == APModeINFO::AltHold || AF.AutoPilotMode == APModeINFO::PositionHold))
				{
					if (!AF._flag_IsFlowAvalible || !DF._IsFlowEnable)
					{
						AF._flag_IsAltHoldSet = true;
					}
					else
					{
						AF._flag_IsAltHoldSet = false;
					}
					if (AF.AutoPilotMode == APModeINFO::PositionHold)
					{
						if (RF._uORB_RC_Out__Roll == 0 && RF._uORB_RC_Out_Pitch == 0)
						{
							if (!AF._flag_GPS_Error || !AF._flag_IsFlowAvalible)
							{
								AF._flag_IsGPSHoldSet = true;
								AF._flag_IsFlowHoldSet = false;
							}
							else if (AF._flag_IsFlowAvalible && DF._IsFlowEnable)
							{
								AF._flag_IsFlowHoldSet = true;
								AF._flag_IsGPSHoldSet = false;
							}
							else
							{
								AF._flag_IsFlowHoldSet = false;
								AF._flag_IsGPSHoldSet = false;
							}
						}
						else
						{
							AF._flag_IsFlowHoldSet = false;
							AF._flag_IsGPSHoldSet = false;
						}

						if (!AF._flag_IsFlowAvalible)
						{
							AF._flag_IsFlowHoldSet = false;
						}
					}
				}
			}

			TF._Tmp_RXTThreadTimeEnd = micros();
			TF._Tmp_RXTThreadTimeLoop = TF._Tmp_RXTThreadTimeEnd - TF._Tmp_RXTThreadTimeStart;
			if (TF._Tmp_RXTThreadTimeLoop + TF._Tmp_RXTThreadTimeNext > TF._flag_RXTThreadTimeMax | TF._Tmp_RXTThreadTimeNext < 0)
			{
				usleep(50);
				TF._flag_RXTErrorTimes++;
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
	if (DF._IsGPSEnable)
	{
		TF.GPSTask = new std::thread([&] {
			GPSInit->GPSReOpen();
			TF._Tmp_GPSThreadSMooth = 10;
			while (true)
			{
				TF._Tmp_GPSThreadTimeStart = micros();
				TF._Tmp_GPSThreadTimeNext = TF._Tmp_GPSThreadTimeStart - TF._Tmp_GPSThreadTimeEnd;
				{
					if (TF._Tmp_GPSThreadSMooth == 10)
					{
						SF._uORB_GPS_Data = GPSInit->GPSParse();
						TF._Tmp_GPSThreadSMooth = 0;
						if (!SF._uORB_GPS_Data.DataUnCorrect)
						{
							SF._Tmp_GPS_Lat_Diff = (float)(SF._uORB_GPS_Data.lat - SF._Tmp_GPS_Lat_Last_Data) / 10.f;
							SF._Tmp_GPS_Lng_Diff = (float)(SF._uORB_GPS_Data.lng - SF._Tmp_GPS_Lng_Last_Data) / 10.f;
							SF._uORB_GPS_Lat_Smooth = SF._Tmp_GPS_Lat_Last_Data;
							SF._uORB_GPS_Lng_Smooth = SF._Tmp_GPS_Lng_Last_Data;
							SF._Tmp_GPS_Lat_Last_Data = SF._uORB_GPS_Data.lat;
							SF._Tmp_GPS_Lng_Last_Data = SF._uORB_GPS_Data.lng;
							SF._Tmp_GPS_Lat_Smooth_Diff = 0;
							SF._Tmp_GPS_Lng_Smooth_Diff = 0;
						}
						else
						{
							SF._Tmp_GPS_Lat_Diff = 0;
							SF._Tmp_GPS_Lng_Diff = 0;
							SF._Tmp_GPS_Lat_Smooth_Diff = 0;
							SF._Tmp_GPS_Lng_Smooth_Diff = 0;
							SF._Tmp_GPS_Lat_Last_Data = SF._uORB_GPS_Lat_Smooth;
							SF._Tmp_GPS_Lng_Last_Data = SF._uORB_GPS_Lng_Smooth;
						}
					}
					//
					if (SF._Tmp_GPS_Lat_Last_Data == 0 && SF._Tmp_GPS_Lng_Last_Data == 0)
					{
						SF._Tmp_GPS_Lat_Last_Data = SF._uORB_GPS_Data.lat;
						SF._Tmp_GPS_Lng_Last_Data = SF._uORB_GPS_Data.lng;
					}
					SF._Tmp_GPS_Lat_Smooth_Diff += SF._Tmp_GPS_Lat_Diff;
					if (abs(SF._Tmp_GPS_Lat_Smooth_Diff) > 1)
					{
						SF._uORB_GPS_Lat_Smooth += (int)SF._Tmp_GPS_Lat_Smooth_Diff;
						SF._Tmp_GPS_Lat_Smooth_Diff -= (int)SF._Tmp_GPS_Lat_Smooth_Diff;
					}
					SF._Tmp_GPS_Lng_Smooth_Diff += SF._Tmp_GPS_Lng_Diff;
					if (abs(SF._Tmp_GPS_Lng_Smooth_Diff) > 1)
					{
						SF._uORB_GPS_Lng_Smooth += (int)SF._Tmp_GPS_Lng_Smooth_Diff;
						SF._Tmp_GPS_Lng_Smooth_Diff -= (int)SF._Tmp_GPS_Lng_Smooth_Diff;
					}
					//
					if (SF._uORB_GPS_Data.satillitesCount < 4 || SF._uORB_GPS_Data.DataUnCorrect)
					{
						AF._flag_GPS_Disconnected = true;
					}
					else if (SF._uORB_GPS_Data.satillitesCount > 4 || !SF._uORB_GPS_Data.DataUnCorrect)
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
				TF._Tmp_GPSThreadSMooth++;
				TF._Tmp_GPSThreadTimeEnd = micros();
				TF._Tmp_GPSThreadTimeLoop = TF._Tmp_GPSThreadTimeEnd - TF._Tmp_GPSThreadTimeStart;
				if (TF._Tmp_GPSThreadTimeLoop + TF._Tmp_GPSThreadTimeNext > TF._flag_GPSThreadTimeMax | TF._Tmp_GPSThreadTimeNext < 0)
				{
					usleep(50);
					TF._flag_GPSErrorTimes++;
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

		cpu_set_t cpuset;
		CPU_ZERO(&cpuset);
		CPU_SET(3, &cpuset);
		int rc = pthread_setaffinity_np(TF.GPSTask->native_handle(), sizeof(cpu_set_t), &cpuset);

		TF.MAGTask = new std::thread([&] {
			while (true)
			{
				TF._Tmp_MAGThreadTimeStart = micros();
				TF._Tmp_MAGThreadTimeNext = TF._Tmp_MAGThreadTimeStart - TF._Tmp_MAGThreadTimeEnd;
				{
					GPSMAGInit->GPSI2CCompass_QMC5883LRead(SF._uORB_QMC5883L_M_X, SF._uORB_QMC5883L_M_Y, SF._uORB_QMC5883L_M_Z);
					SF._uORB_QMC5883L_M_X += SF._flag_QMC5883L_M_X_Offset;
					SF._uORB_QMC5883L_M_Y += SF._flag_QMC5883L_M_Y_Offset;
					SF._uORB_QMC5883L_M_Y *= SF._flag_QMC5883L_M_Y_Scaler;
					SF._uORB_QMC5883L_M_Z += SF._flag_QMC5883L_M_Z_Offset;
					SF._uORB_QMC5883L_M_Z *= SF._flag_QMC5883L_M_Z_Scaler;
					SF._Tmp_QMC5883L_M_XH = (float)SF._uORB_QMC5883L_M_Y * cos(SF._uORB_MPU_Data->_uORB_Real_Pitch * (3.14 / 180.f)) +
											(float)SF._uORB_QMC5883L_M_X * sin(SF._uORB_MPU_Data->_uORB_Real__Roll * (3.14 / -180.f)) * sin(SF._uORB_MPU_Data->_uORB_Real_Pitch * (3.14 / 180.f)) -
											(float)SF._uORB_QMC5883L_M_Z * cos(SF._uORB_MPU_Data->_uORB_Real__Roll * (3.14 / -180.f)) * sin(SF._uORB_MPU_Data->_uORB_Real_Pitch * (3.14 / 180.f));
					SF._Tmp_QMC5883L_M_YH = (float)SF._uORB_QMC5883L_M_X * cos(SF._uORB_MPU_Data->_uORB_Real__Roll * (3.14 / -180.f)) +
											(float)SF._uORB_QMC5883L_M_Z * sin(SF._uORB_MPU_Data->_uORB_Real__Roll * (3.14 / -180.f));

					if (SF._Tmp_QMC5883L_M_YH < 0)
						SF._Tmp_QMC5883L___MAG = 180 + (180 + ((atan2(SF._Tmp_QMC5883L_M_YH, SF._Tmp_QMC5883L_M_XH)) * (180 / 3.14)));
					else
						SF._Tmp_QMC5883L___MAG = (atan2(SF._Tmp_QMC5883L_M_YH, SF._Tmp_QMC5883L_M_XH)) * (180 / 3.14);
					SF._Tmp_QMC5883L___MAG += 180;
					SF._Tmp_QMC5883L___MAG += SF._flag_QMC5883L_Head_Asix;
					if (SF._Tmp_QMC5883L___MAG < 0)
						SF._Tmp_QMC5883L___MAG += 360;
					else if (SF._Tmp_QMC5883L___MAG >= 360)
						SF._Tmp_QMC5883L___MAG -= 360;
					//--------------------------------------------------------------------//
					// SF._uORB_QMC5883L__Yaw += 200.f;
					if (SF._uORB_QMC5883L__Yaw < 0)
						SF._uORB_QMC5883L__Yaw += 360;
					else if (SF._uORB_QMC5883L__Yaw >= 360)
						SF._uORB_QMC5883L__Yaw -= 360;
					SF._Tmp_QMC5883L__Head = SF._uORB_QMC5883L__Yaw - SF._Tmp_QMC5883L___MAG;
					if (SF._Tmp_QMC5883L__Head < -180 || SF._Tmp_QMC5883L__Head > 180)
					{
						if (SF._Tmp_QMC5883L___MAG > 180)
							SF._Tmp_QMC5883L__Head__Mag = SF._Tmp_QMC5883L___MAG - 180;
						else
							SF._Tmp_QMC5883L__Head__Mag = SF._Tmp_QMC5883L___MAG + 180;
						if (SF._Tmp_QMC5883L__Head > 180)
							SF._Tmp_QMC5883L__Head_Gryo = SF._uORB_QMC5883L__Yaw - 180;
						else
							SF._Tmp_QMC5883L__Head_Gryo = SF._uORB_QMC5883L__Yaw + 180;
						SF._Tmp_QMC5883L__Head = SF._Tmp_QMC5883L__Head_Gryo - SF._Tmp_QMC5883L__Head__Mag;
					}
					SF._uORB_QMC5883L__Yaw -= SF._Tmp_QMC5883L__Head / 1200.0;
					if (SF._uORB_QMC5883L__Yaw < 0)
						SF._uORB_QMC5883L__Yaw += 360;
					else if (SF._uORB_QMC5883L__Yaw >= 360)
						SF._uORB_QMC5883L__Yaw -= 360;
					SF._uORB_QMC5883L_Head = SF._uORB_QMC5883L__Yaw;
					if (AF._flag_ESC_ARMED == true)
					{
						SF._uORB_QMC5883L__Yaw = SF._Tmp_QMC5883L___MAG;
					}
				}
				TF._Tmp_MAGThreadTimeEnd = micros();
				TF._Tmp_MAGThreadTimeLoop = TF._Tmp_MAGThreadTimeEnd - TF._Tmp_MAGThreadTimeStart;
				if (TF._Tmp_MAGThreadTimeLoop + TF._Tmp_MAGThreadTimeNext > TF._flag_MAGThreadTimeMax | TF._Tmp_MAGThreadTimeNext < 0)
				{
					usleep(50);
					TF._flag_MAGErrorTimes++;
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
		cpu_set_t cpuset2;
		CPU_ZERO(&cpuset2);
		CPU_SET(3, &cpuset2);
		int rc2 = pthread_setaffinity_np(TF.MAGTask->native_handle(), sizeof(cpu_set_t), &cpuset2);
	}

	if (DF._IsFlowEnable)
	{
		TF.FlowTask = new std::thread([&] {
			while (true)
			{
				TF._Tmp_FlowThreadTimeStart = micros();
				TF._Tmp_FlowThreadTimeNext = TF._Tmp_FlowThreadTimeStart - TF._Tmp_FlowThreadTimeEnd;
				{
					TF._Tmp_FlowThreadSMooth++;
					SF._Tmp_Flow___Status = FlowInit->MSPDataRead(SF._uORB_Flow_XOutput, SF._uORB_Flow_YOutput, SF._uORB_Flow_Altitude);

					if (SF._Tmp_Flow___Status == 1 || SF._Tmp_Flow___Status == 3)
					{
						SF._uORB_Flow_Altitude = SF._uORB_Flow_Altitude * (1.f - cos(abs(SF._uORB_MPU_Data->_uORB_Real_Pitch) * 3.14 / 180.f) *
																					 (1.f - cos(abs(SF._uORB_MPU_Data->_uORB_Real__Roll) * 3.14 / 180.f)));
						SF._uORB_Flow_Altitude_Final += (SF._uORB_Flow_Altitude - SF._uORB_Flow_Altitude_Final) * 0.2;
					}
					if (SF._Tmp_Flow___Status == 2 || SF._Tmp_Flow___Status == 3)
					{
						SF._Tmp_Flow_XOuput_Total += SF._uORB_Flow_XOutput;
						SF._Tmp_Flow_YOuput_Total += SF._uORB_Flow_YOutput;
						SF._Tmp_Flow_Filter_XOutput_Total += (SF._Tmp_Flow_XOuput_Total - SF._Tmp_Flow_Filter_XOutput_Total) * 0.2;
						SF._Tmp_Flow_Filter_YOutput_Total += (SF._Tmp_Flow_YOuput_Total - SF._Tmp_Flow_Filter_YOutput_Total) * 0.2;

						SF._Tmp_Flow_XOutput_Total_Diff = (float)(SF._Tmp_Flow_Filter_XOutput_Total - SF._Tmp_Flow_XOutput_Total_Last_Data) / 12.f;
						SF._Tmp_Flow_YOutput_Total_Diff = (float)(SF._Tmp_Flow_Filter_YOutput_Total - SF._Tmp_Flow_YOutput_Total_Last_Data) / 12.f;

						SF._Tmp_Flow_XOutput_Total_Smooth = SF._Tmp_Flow_XOutput_Total_Last_Data;
						SF._Tmp_Flow_YOutput_Total_Smooth = SF._Tmp_Flow_YOutput_Total_Last_Data;

						SF._Tmp_Flow_XOutput_Total_Last_Data = SF._Tmp_Flow_Filter_XOutput_Total;
						SF._Tmp_Flow_YOutput_Total_Last_Data = SF._Tmp_Flow_Filter_YOutput_Total;

						SF._Tmp_Flow_XOutput_Total_Smooth_Diff = 0;
						SF._Tmp_Flow_YOutput_Total_Smooth_Diff = 0;
						TF._Tmp_FlowThreadSMooth = 0;
					}
					if (TF._Tmp_FlowThreadSMooth >= 12)
					{
						TF._flag_FlowErrorTimes++;
						SF._Tmp_Flow_XOutput_Total_Diff = 0;
						SF._Tmp_Flow_YOutput_Total_Diff = 0;
						SF._Tmp_Flow_XOutput_Total_Smooth_Diff = 0;
						SF._Tmp_Flow_YOutput_Total_Smooth_Diff = 0;
						SF._Tmp_Flow_XOutput_Total_Last_Data = SF._Tmp_Flow_XOutput_Total_Smooth;
						SF._Tmp_Flow_YOutput_Total_Last_Data = SF._Tmp_Flow_YOutput_Total_Smooth;
					}

					if (SF._Tmp_Flow_XOutput_Total_Last_Data == 0 && SF._Tmp_Flow_YOutput_Total_Last_Data == 0)
					{
						SF._Tmp_Flow_XOutput_Total_Last_Data = SF._Tmp_Flow_Filter_XOutput_Total;
						SF._Tmp_Flow_YOutput_Total_Last_Data = SF._Tmp_Flow_Filter_YOutput_Total;
					}
					//
					SF._Tmp_Flow_XOutput_Total_Smooth_Diff += SF._Tmp_Flow_XOutput_Total_Diff;
					if (abs(SF._Tmp_Flow_XOutput_Total_Smooth_Diff) > 1)
					{
						SF._Tmp_Flow_XOutput_Total_Smooth += (int)SF._Tmp_Flow_XOutput_Total_Smooth_Diff;
						SF._Tmp_Flow_XOutput_Total_Smooth_Diff -= (int)SF._Tmp_Flow_XOutput_Total_Smooth_Diff;
					}
					SF._Tmp_Flow_YOutput_Total_Smooth_Diff += SF._Tmp_Flow_YOutput_Total_Diff;
					if (abs(SF._Tmp_Flow_YOutput_Total_Smooth_Diff) > 1)
					{
						SF._Tmp_Flow_YOutput_Total_Smooth += (int)SF._Tmp_Flow_YOutput_Total_Smooth_Diff;
						SF._Tmp_Flow_YOutput_Total_Smooth_Diff -= (int)SF._Tmp_Flow_YOutput_Total_Smooth_Diff;
					}
					SF._uORB_Flow_Fix_XOutput += (300.f * tan((SF._uORB_MPU_Data->_uORB_Real__Roll) * 3.14 / 180.f) - SF._uORB_Flow_Fix_XOutput) * 0.2;
					SF._uORB_Flow_Fix_YOutput += (300.f * tan((SF._uORB_MPU_Data->_uORB_Real_Pitch) * 3.14 / 180.f) - SF._uORB_Flow_Fix_YOutput) * 0.2;
					SF._uORB_Flow_XOutput_Total_Smooth = SF._Tmp_Flow_XOutput_Total_Smooth + SF._uORB_Flow_Fix_XOutput;
					SF._uORB_Flow_YOutput_Total_Smooth = SF._Tmp_Flow_YOutput_Total_Smooth - SF._uORB_Flow_Fix_YOutput;

					if (20.f < SF._uORB_Flow_Altitude_Final && SF._uORB_Flow_Altitude_Final < 1800.f)
					{
						AF._flag_IsFlowAvalible = true;
					}
					else
					{
						AF._flag_IsFlowAvalible = false;
					}

					AF._flag_FlowData_Async = true;
				}
				TF._Tmp_FlowThreadTimeEnd = micros();
				TF._Tmp_FlowThreadTimeLoop = TF._Tmp_FlowThreadTimeEnd - TF._Tmp_FlowThreadTimeStart;
				if (TF._Tmp_FlowThreadTimeLoop + TF._Tmp_FlowThreadTimeNext > TF._flag_FlowThreadTimeMax | TF._Tmp_FlowThreadTimeNext < 0)
				{
					usleep(50);
					TF._flag_FlowErrorTimes++;
					AF._flag_ClockingTime_Error = true;
				}
				else
				{
					usleep(TF._flag_FlowThreadTimeMax - TF._Tmp_FlowThreadTimeLoop - TF._Tmp_FlowThreadTimeNext);
				}
				if (TF._Tmp_FlowThreadTimeLoop + TF._Tmp_FlowThreadTimeNext > TF._Tmp_FlowThreadError)
				{
					TF._Tmp_FlowThreadError = TF._Tmp_FlowThreadTimeLoop;
				}
				TF._Tmp_FlowThreadTimeEnd = micros();
			}
		});
		cpu_set_t cpuset3;
		CPU_ZERO(&cpuset3);
		CPU_SET(3, &cpuset3);
		int rc3 = pthread_setaffinity_np(TF.FlowTask->native_handle(), sizeof(cpu_set_t), &cpuset3);
	}
	else
	{
		AF._flag_IsFlowHoldSet = false;
		AF._flag_IsFlowAvalible = false;
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
				TF._flag_ESCErrorTimes++;
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
#ifdef RPiDEBUG
		DebugOutPut();
		DEBUGOuputCleaner++;
		if (DEBUGOuputCleaner > 60)
		{
			system("clear");
			DEBUGOuputCleaner = 0;
		}
#endif
		SaftyCheckTaskReg();
		usleep(50000);
	}
}

int SingleAPMAPI::RPiSingleAPM::APMCalibrator(int controller, int action, int input, double *data)
{
	if (controller == ESCCalibration)
	{
		if (action == CaliESCStart)
		{
			pca9685PWMReset(DF.PCA9658_fd);
			sleep(5);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A1_Pin, 0, EF._Flag_Max__Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A2_Pin, 0, EF._Flag_Max__Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B1_Pin, 0, EF._Flag_Max__Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B2_Pin, 0, EF._Flag_Max__Throttle);
			sleep(5);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A1_Pin, 0, EF._Flag_Lock_Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A2_Pin, 0, EF._Flag_Lock_Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B1_Pin, 0, EF._Flag_Lock_Throttle);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B2_Pin, 0, EF._Flag_Lock_Throttle);
			return 0;
		}
		else if (action == CaliESCUserDefine)
		{
			int Ouput = (700 * (((float)input - (float)RF._flag_RC_Min_PWM_Value) / (float)(RF._flag_RC_Max_PWM_Value - RF._flag_RC_Min_PWM_Value))) + 2300;
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A1_Pin, 0, Ouput);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_A2_Pin, 0, Ouput);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B1_Pin, 0, Ouput);
			pca9685PWMWrite(DF.PCA9658_fd, EF._flag_B2_Pin, 0, Ouput);
			return 1;
		}
	}
	else if (controller == ACCELCalibration)
	{
		if (action == CaliACCELHeadNormal)
		{
		}
	}
	// else if (type == 1)
	// {
	// 	int i;
	// 	bool finish;
	// 	std::cout << "[COMPASS CALIBRATION!COMPASS CALIBRATION!COMPASS CALIBRATION!]\n";
	// 	std::cout << "  INPUT ANY NUMBER TO START COMPASS CALIBRATION\n";
	// 	std::cin >> i;
	// 	finish = true;
	// 	long compassCal[6] = {0, 0, 0, 0, 0};
	// 	long compassCalMPU[6] = {0, 0, 0, 0, 0};
	// 	std::thread compass = std::thread([&] {
	// 		double compassXOffset;
	// 		double compassYOffset;
	// 		double compassZOffset;
	// 		double compassYScaler;
	// 		double compassZScaler;
	// 		double compassXOffsetMPU;
	// 		double compassYOffsetMPU;
	// 		double compassZOffsetMPU;
	// 		double compassYScalerMPU;
	// 		double compassZScalerMPU;
	// 		while (finish)
	// 		{
	// 			GPSMAGInit->GPSI2CCompass_QMC5883LRead(SF._uORB_QMC5883L_M_X, SF._uORB_QMC5883L_M_Y, SF._uORB_QMC5883L_M_Z);
	// 			compassCal[0] = SF._uORB_QMC5883L_M_X < compassCal[0] ? SF._uORB_QMC5883L_M_X : compassCal[0];
	// 			compassCal[1] = SF._uORB_QMC5883L_M_X > compassCal[1] ? SF._uORB_QMC5883L_M_X : compassCal[1];
	// 			compassCal[2] = SF._uORB_QMC5883L_M_Y < compassCal[2] ? SF._uORB_QMC5883L_M_Y : compassCal[2];
	// 			compassCal[3] = SF._uORB_QMC5883L_M_Y > compassCal[3] ? SF._uORB_QMC5883L_M_Y : compassCal[3];
	// 			compassCal[4] = SF._uORB_QMC5883L_M_Z < compassCal[4] ? SF._uORB_QMC5883L_M_Z : compassCal[4];
	// 			compassCal[5] = SF._uORB_QMC5883L_M_Z > compassCal[5] ? SF._uORB_QMC5883L_M_Z : compassCal[5];

	// 			IMUSensorsDataRead();
	// 			compassCalMPU[0] = SF._uORB_MPU9250_M_X < compassCalMPU[0] ? SF._uORB_MPU9250_M_X : compassCalMPU[0];
	// 			compassCalMPU[1] = SF._uORB_MPU9250_M_X > compassCalMPU[1] ? SF._uORB_MPU9250_M_X : compassCalMPU[1];
	// 			compassCalMPU[2] = SF._uORB_MPU9250_M_Y < compassCalMPU[2] ? SF._uORB_MPU9250_M_Y : compassCalMPU[2];
	// 			compassCalMPU[3] = SF._uORB_MPU9250_M_Y > compassCalMPU[3] ? SF._uORB_MPU9250_M_Y : compassCalMPU[3];
	// 			compassCalMPU[4] = SF._uORB_MPU9250_M_Z < compassCalMPU[4] ? SF._uORB_MPU9250_M_Z : compassCalMPU[4];
	// 			compassCalMPU[5] = SF._uORB_MPU9250_M_Z > compassCalMPU[5] ? SF._uORB_MPU9250_M_Z : compassCalMPU[5];
	// 		}
	// 		compassYScaler = ((float)compassCal[1] - compassCal[0]) / (compassCal[3] - compassCal[2]);
	// 		compassZScaler = ((float)compassCal[1] - compassCal[0]) / (compassCal[5] - compassCal[4]);
	// 		compassXOffset = (compassCal[1] - compassCal[0]) / 2 - compassCal[1];
	// 		compassYOffset = (((float)compassCal[3] - compassCal[2]) / 2 - compassCal[3]) * compassYScaler;
	// 		compassZOffset = (((float)compassCal[5] - compassCal[4]) / 2 - compassCal[5]) * compassZScaler;

	// 		compassYScalerMPU = ((float)compassCalMPU[1] - compassCalMPU[0]) / (compassCalMPU[3] - compassCalMPU[2]);
	// 		compassZScalerMPU = ((float)compassCalMPU[1] - compassCalMPU[0]) / (compassCalMPU[5] - compassCalMPU[4]);
	// 		compassXOffsetMPU = (compassCalMPU[1] - compassCalMPU[0]) / 2 - compassCalMPU[1];
	// 		compassYOffsetMPU = (((float)compassCalMPU[3] - compassCalMPU[2]) / 2 - compassCalMPU[3]) * compassYScalerMPU;
	// 		compassZOffsetMPU = (((float)compassCalMPU[5] - compassCalMPU[4]) / 2 - compassCalMPU[5]) * compassZScalerMPU;

	// 		data[0] = compassYScaler;
	// 		data[1] = compassZScaler;
	// 		data[2] = compassXOffset;
	// 		data[3] = compassYOffset;
	// 		data[4] = compassZOffset;

	// 		data[5] = compassYScalerMPU;
	// 		data[6] = compassZScalerMPU;
	// 		data[7] = compassXOffsetMPU;
	// 		data[8] = compassYOffsetMPU;
	// 		data[9] = compassZOffsetMPU;
	// 	});
	// 	std::cout << "  INPUT ANY NUMBER TO STOP COMPASS CALIBRATION\n";
	// 	std::cin >> i;
	// 	finish = false;
	// 	compass.join();
	// 	std::cout << "CAL1:" << compassCal[0] << "\n";
	// 	std::cout << "CAL2:" << compassCal[1] << "\n";
	// 	std::cout << "CAL3:" << compassCal[2] << "\n";
	// 	std::cout << "CAL4:" << compassCal[3] << "\n";
	// 	std::cout << "CAL5:" << compassCal[4] << "\n";
	// 	std::cout << "CAL6:" << compassCal[5] << "\n";
	// 	std::cout << "compassYScaler :" << data[0] << "\n";
	// 	std::cout << "compassZScaler :" << data[1] << "\n";
	// 	std::cout << "compassXOffset :" << data[2] << "\n";
	// 	std::cout << "compassYOffset :" << data[3] << "\n";
	// 	std::cout << "compassZOffset :" << data[4] << "\n";
	// }
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

void SingleAPMAPI::RPiSingleAPM::ConfigReader(APMSettinngs APMInit)
{
	//==========================================================Device Type=======/
	RF.RC_Type = APMInit.RC_Type;
	SF.MPU9250_Type = APMInit.MPU9250_Type;
	SF.IMUFilter_Type = APMInit.IMUFilter_Type;
	SF.IMUMixFilter_Type = APMInit.IMUMixFilter_Type;

	DF.RCDevice = APMInit.__RCDevice;
	DF.GPSDevice = APMInit.__GPSDevice;
	DF.FlowDevice = APMInit.__FlowDevice;

	DF._IsGPSEnable = APMInit._IsGPSEnable;
	DF._IsFlowEnable = APMInit._IsFlowEnable;
	DF._IsRCSafeEnable = APMInit._IsRCSafeEnable;
	DF._IsMS5611Enable = APMInit._IsMS5611Enable;
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
	PF._flag_PID_P_TAsix_Gain = APMInit._flag_PID_P_TAsix_Gain;
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
	SF._flag_QMC5883L_Head_Asix = APMInit._flag_QMC5883L_Head_Asix;
	SF._flag_QMC5883L_M_X_Offset = APMInit._flag_QMC5883L_M_X_Offset;
	SF._flag_QMC5883L_M_Y_Offset = APMInit._flag_QMC5883L_M_Y_Offset;
	SF._flag_QMC5883L_M_Z_Offset = APMInit._flag_QMC5883L_M_Z_Offset;
	SF._flag_QMC5883L_M_Y_Scaler = APMInit._flag_QMC5883L_M_Y_Scaler;
	SF._flag_QMC5883L_M_Z_Scaler = APMInit._flag_QMC5883L_M_Z_Scaler;
	//===============================================================Update cofig==/
	TF._flag_IMUThreadFreq = APMInit.IMU_Freqeuncy;
	TF._flag_IMUThreadTimeMax = (float)1 / TF._flag_IMUThreadFreq * 1000000;
	TF._flag_RXTThreadFreq = APMInit.RXT_Freqeuncy;
	TF._flag_RXTThreadTimeMax = (float)1 / TF._flag_RXTThreadFreq * 1000000;
	TF._flag_ESCThreadFreq = APMInit.ESC_Freqeuncy;
	TF._flag_ESCThreadTimeMax = (float)1 / TF._flag_ESCThreadFreq * 1000000;
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

				PF._uORB_PID_GPS_Lng_Local_Diff = PF._uORB_PID_GPS_Lng_Local_Target - SF._uORB_GPS_Lng_Smooth;
				PF._uORB_PID_GPS_Lat_Local_Diff = SF._uORB_GPS_Lat_Smooth - PF._uORB_PID_GPS_Lat_Local_Target;

				PF._Tmp_PID_D_GPS_Lat_Ouput -= PF._Tmp_PID_D_GPS_Lat_AvaData[PF._Tmp_PID_D_GPS_AvaClock];
				PF._Tmp_PID_D_GPS_Lat_AvaData[PF._Tmp_PID_D_GPS_AvaClock] = PF._uORB_PID_GPS_Lat_Local_Diff - PF._uORB_PID_D_GPS_Lat_LastValue;
				PF._Tmp_PID_D_GPS_Lat_Ouput += PF._Tmp_PID_D_GPS_Lat_AvaData[PF._Tmp_PID_D_GPS_AvaClock];

				PF._Tmp_PID_D_GPS_Lng_Ouput -= PF._Tmp_PID_D_GPS_Lng_AvaData[PF._Tmp_PID_D_GPS_AvaClock];
				PF._Tmp_PID_D_GPS_Lng_AvaData[PF._Tmp_PID_D_GPS_AvaClock] = PF._uORB_PID_GPS_Lng_Local_Diff - PF._uORB_PID_D_GPS_Lng_LastValue;
				PF._Tmp_PID_D_GPS_Lng_Ouput += PF._Tmp_PID_D_GPS_Lng_AvaData[PF._Tmp_PID_D_GPS_AvaClock];
				PF._Tmp_PID_D_GPS_AvaClock++;
				if (PF._Tmp_PID_D_GPS_AvaClock == 35)
				{
					PF._Tmp_PID_D_GPS_AvaClock = 0;
				}

				PF._uORB_PID_D_GPS_Lat_LastValue = PF._uORB_PID_GPS_Lat_Local_Diff;
				PF._uORB_PID_D_GPS_Lng_LastValue = PF._uORB_PID_GPS_Lng_Local_Diff;

				PF._uORB_PID_GPS_Lat_Ouput = (float)PF._uORB_PID_GPS_Lat_Local_Diff * PF._flag_PID_P_GPS_Gain + PF._Tmp_PID_D_GPS_Lat_Ouput * PF._flag_PID_D_GPS_Gain;
				PF._uORB_PID_GPS_Lng_Ouput = (float)PF._uORB_PID_GPS_Lng_Local_Diff * PF._flag_PID_P_GPS_Gain + PF._Tmp_PID_D_GPS_Lng_Ouput * PF._flag_PID_D_GPS_Gain;

				PF._uORB_PID_GPS_Lat_Ouput = !SF._uORB_GPS_Data.lat_North_Mode ? PF._uORB_PID_GPS_Lat_Ouput * -1 : PF._uORB_PID_GPS_Lat_Ouput;
				PF._uORB_PID_GPS_Lng_Ouput = !SF._uORB_GPS_Data.lat_East_Mode ? PF._uORB_PID_GPS_Lng_Ouput * -1 : PF._uORB_PID_GPS_Lng_Ouput;

				if (AF.AutoPilotMode == APModeINFO::PositionHold && AF._flag_IsGPSHoldSet)
				{
					PF._uORB_PID_GPS__Roll_Ouput = PF._uORB_PID_GPS_Lng_Ouput * cos(SF._uORB_QMC5883L_Head * 3.14 / 180.f) + PF._uORB_PID_GPS_Lat_Ouput * cos((SF._uORB_QMC5883L_Head - 90.f) * 3.14 / 180.f);
					PF._uORB_PID_GPS_Pitch_Ouput = PF._uORB_PID_GPS_Lat_Ouput * cos(SF._uORB_QMC5883L_Head * 3.14 / 180.f) + PF._uORB_PID_GPS_Lng_Ouput * cos((SF._uORB_QMC5883L_Head + 90.f) * 3.14 / 180.f);

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

		{
			if (AF._flag_FlowData_Async)
			{
				if (!AF._flag_IsFlowHoldSet)
				{
					SF._Tmp_Flow_XOuput_Total = 0;
					SF._Tmp_Flow_YOuput_Total = 0;
					SF._Tmp_Flow_Filter_YOutput_Total = 0;
					SF._Tmp_Flow_Filter_YOutput_Total = 0;
					SF._Tmp_Flow_XOutput_Total_Smooth = 0;
					SF._Tmp_Flow_YOutput_Total_Smooth = 0;
				}

				if (AF.AutoPilotMode == APModeINFO::PositionHold && AF._flag_IsFlowHoldSet)
				{
				}
				else
				{
				}
				AF._flag_FlowData_Async = false;
			}
		}

		//Roll PID Mix
		PF._uORB_PID__Roll_Input = SF._uORB_MPU_Data->_uORB_Gryo__Roll + SF._uORB_MPU_Data->_uORB_Real__Roll * 15 -
								   RF._uORB_RC_Out__Roll - PF._uORB_PID_GPS__Roll_Ouput;
		PID_Caculate(PF._uORB_PID__Roll_Input, PF._uORB_Leveling__Roll,
					 PF._uORB_PID_I_Last_Value__Roll, PF._uORB_PID_D_Last_Value__Roll,
					 PF._flag_PID_P__Roll_Gain, PF._flag_PID_I__Roll_Gain, PF._flag_PID_D__Roll_Gain, PF._flag_PID_I__Roll_Max__Value);
		if (PF._uORB_Leveling__Roll > PF._flag_PID_Level_Max)
			PF._uORB_Leveling__Roll = PF._flag_PID_Level_Max;
		if (PF._uORB_Leveling__Roll < PF._flag_PID_Level_Max * -1)
			PF._uORB_Leveling__Roll = PF._flag_PID_Level_Max * -1;

		//Pitch PID Mix
		PF._uORB_PID_Pitch_Input = SF._uORB_MPU_Data->_uORB_Gryo_Pitch + SF._uORB_MPU_Data->_uORB_Real_Pitch * 15 -
								   RF._uORB_RC_Out_Pitch - PF._uORB_PID_GPS_Pitch_Ouput;
		PID_Caculate(PF._uORB_PID_Pitch_Input, PF._uORB_Leveling_Pitch,
					 PF._uORB_PID_I_Last_Value_Pitch, PF._uORB_PID_D_Last_Value_Pitch,
					 PF._flag_PID_P_Pitch_Gain, PF._flag_PID_I_Pitch_Gain, PF._flag_PID_D_Pitch_Gain, PF._flag_PID_I_Pitch_Max__Value);
		if (PF._uORB_Leveling_Pitch > PF._flag_PID_Level_Max)
			PF._uORB_Leveling_Pitch = PF._flag_PID_Level_Max;
		if (PF._uORB_Leveling_Pitch < PF._flag_PID_Level_Max * -1)
			PF._uORB_Leveling_Pitch = PF._flag_PID_Level_Max * -1;

		//Yaw PID Mix
		PID_Caculate(SF._uORB_MPU_Data->_uORB_Gryo___Yaw - RF._uORB_RC_Out___Yaw, PF._uORB_Leveling___Yaw,
					 PF._uORB_PID_I_Last_Value___Yaw, PF._uORB_PID_D_Last_Value___Yaw,
					 PF._flag_PID_P___Yaw_Gain, PF._flag_PID_I___Yaw_Gain, PF._flag_PID_D___Yaw_Gain, PF._flag_PID_I___Yaw_Max__Value);
		if (PF._uORB_Leveling___Yaw > PF._flag_PID_Level_Max)
			PF._uORB_Leveling___Yaw = PF._flag_PID_Level_Max;
		if (PF._uORB_Leveling___Yaw < PF._flag_PID_Level_Max * -1)
			PF._uORB_Leveling___Yaw = PF._flag_PID_Level_Max * -1;
	}
	//ESC Caculate
	if (AF.AutoPilotMode == APModeINFO::AltHold ||
		AF.AutoPilotMode == APModeINFO::PositionHold)
	{
		PF._uORB_PID_TAsix_Ouput = ((PF._flag_PID_Hover_Throttle - 1000 + PF._uORB_PID_Alt_Throttle) * (1.f - cos((abs(SF._uORB_MPU_Data->_uORB_Real_Pitch)) * 3.14 / 180.f)) +
									(PF._flag_PID_Hover_Throttle - 1000 + PF._uORB_PID_Alt_Throttle) * (1.f - cos((abs(SF._uORB_MPU_Data->_uORB_Real__Roll)) * 3.14 / 180.f))) *
								   PF._flag_PID_P_TAsix_Gain;
		PF._uORB_PID_TAsix_Ouput = PF._uORB_PID_TAsix_Ouput > 500 ? 500 : PF._uORB_PID_TAsix_Ouput;
		PF._uORB_PID_TAsix_Ouput = PF._uORB_PID_TAsix_Ouput < 0 ? 0 : PF._uORB_PID_TAsix_Ouput;
		EF._Tmp_B1_Speed = PF._flag_PID_Hover_Throttle + PF._uORB_PID_Alt_Throttle - PF._uORB_Leveling__Roll + PF._uORB_Leveling_Pitch + PF._uORB_Leveling___Yaw + PF._uORB_PID_TAsix_Ouput;
		EF._Tmp_A1_Speed = PF._flag_PID_Hover_Throttle + PF._uORB_PID_Alt_Throttle - PF._uORB_Leveling__Roll - PF._uORB_Leveling_Pitch - PF._uORB_Leveling___Yaw + PF._uORB_PID_TAsix_Ouput;
		EF._Tmp_A2_Speed = PF._flag_PID_Hover_Throttle + PF._uORB_PID_Alt_Throttle + PF._uORB_Leveling__Roll - PF._uORB_Leveling_Pitch + PF._uORB_Leveling___Yaw + PF._uORB_PID_TAsix_Ouput;
		EF._Tmp_B2_Speed = PF._flag_PID_Hover_Throttle + PF._uORB_PID_Alt_Throttle + PF._uORB_Leveling__Roll + PF._uORB_Leveling_Pitch - PF._uORB_Leveling___Yaw + PF._uORB_PID_TAsix_Ouput;
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

	EF._uORB_A1_Speed = EF._uORB_A1_Speed < EF._Flag_Lazy_Throttle ? EF._Flag_Lazy_Throttle : EF._uORB_A1_Speed;
	EF._uORB_A2_Speed = EF._uORB_A2_Speed < EF._Flag_Lazy_Throttle ? EF._Flag_Lazy_Throttle : EF._uORB_A2_Speed;
	EF._uORB_B1_Speed = EF._uORB_B1_Speed < EF._Flag_Lazy_Throttle ? EF._Flag_Lazy_Throttle : EF._uORB_B1_Speed;
	EF._uORB_B2_Speed = EF._uORB_B2_Speed < EF._Flag_Lazy_Throttle ? EF._Flag_Lazy_Throttle : EF._uORB_B2_Speed;

	EF._uORB_A1_Speed = EF._uORB_A1_Speed > EF._Flag_Max__Throttle ? EF._Flag_Max__Throttle : EF._uORB_A1_Speed;
	EF._uORB_A2_Speed = EF._uORB_A2_Speed > EF._Flag_Max__Throttle ? EF._Flag_Max__Throttle : EF._uORB_A2_Speed;
	EF._uORB_B1_Speed = EF._uORB_B1_Speed > EF._Flag_Max__Throttle ? EF._Flag_Max__Throttle : EF._uORB_B1_Speed;
	EF._uORB_B2_Speed = EF._uORB_B2_Speed > EF._Flag_Max__Throttle ? EF._Flag_Max__Throttle : EF._uORB_B2_Speed;
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
	std::cout << " GryoPitch:  " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data->_uORB_Gryo_Pitch << "    "
			  << " GryoRoll:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data->_uORB_Gryo__Roll << "    "
			  << " GryoYaw:    " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data->_uORB_Gryo___Yaw << "    "
			  //   << " GVSpeed:    " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data->_uORB_Accel_VSpeed << "    "
			  << "                        "
			  << std::endl;
	std::cout << " AccePitch:  " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data->_uORB_Accel_Pitch << "    "
			  << " AcceRoll:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data->_uORB_Accel__Roll << "    "
			  << " MPURawAX:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data->_uORB_MPU9250_A_X << "    "
			  << " MPURawAY:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data->_uORB_MPU9250_A_Y << "    "
			  << " MPURawAZ:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data->_uORB_MPU9250_A_Z << "    "
			  << "                        "
			  << std::endl;
	std::cout << " RealPitch:  " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data->_uORB_Real_Pitch << "    "
			  << " RealRoll:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data->_uORB_Real__Roll << "    "
			  << " MPURawGX:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data->_uORB_MPU9250_G_X << "    "
			  << " MPURawGY:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data->_uORB_MPU9250_G_Y << "    "
			  << " MPURawGZ:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data->_uORB_MPU9250_G_Z << "    "
			  << "                        "
			  << std::endl;
	// std::cout << " CompassX:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU9250_M_X << "    "
	// 		  << " CompassY:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU9250_M_Y << "    "
	// 		  << " CompassZ:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU9250_M_Z << "    "
	// 		  << " MPUHeading: " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU9250__Head << "    "
	// 		  << " MPUHeadRaw: " << std::setw(7) << std::setfill(' ') << (int)SF._Tmp_MPU9250___MAG << "    "
	// 		  << "                        "
	// 		  << std::endl;
	std::cout << " GPSCompassX:" << std::setw(7) << std::setfill(' ') << (int)SF._uORB_QMC5883L_M_X << "    "
			  << " GPSCompassY:" << std::setw(7) << std::setfill(' ') << (int)SF._uORB_QMC5883L_M_Y << "    "
			  << " GPSCompassZ:" << std::setw(7) << std::setfill(' ') << (int)SF._uORB_QMC5883L_M_Z << "    "
			  << " GPSHeading: " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_QMC5883L_Head << "    "
			  << " GPSHeadRaw: " << std::setw(7) << std::setfill(' ') << (int)SF._Tmp_QMC5883L___MAG << "    "
			  << "                        "
			  << std::endl;

	std::cout
		<< "MS5611ParseDataINFO:"
		<< "\n";
	std::cout << " FastPressure :      " << std::setw(7) << std::setfill(' ') << (int)SF._Tmp_MS5611_PressureFast;
	std::cout << " ||FilterPressureFast :" << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MS5611_PressureFinal;
	std::cout << " ||AltHoldTarget:      " << std::setw(7) << std::setfill(' ') << (int)PF._uORB_PID_AltHold_Target << "            \n";
	std::cout << " AltholdThrottle:    " << std::setw(7) << std::setfill(' ') << (int)PF._uORB_PID_Alt_Throttle;
	std::cout << " ||Althold_I_Ouput:    " << std::setw(7) << std::setfill(' ') << (int)PF._uORB_PID_I_Last_Value_Alt;
	std::cout << " ||AltholdAsixOuput:   " << std::setw(7) << std::setfill(' ') << (int)PF._uORB_PID_TAsix_Ouput << "            \n";

	std::cout
		<< "FlowParseDataINFO:"
		<< "\n";
	std::cout << " FlowXOut:" << std::setw(7) << std::setfill(' ') << SF._uORB_Flow_XOutput_Total_Smooth;
	std::cout << " ||FlowYOut:" << std::setw(7) << std::setfill(' ') << SF._uORB_Flow_YOutput_Total_Smooth;
	std::cout << " ||FlowAltitude:" << std::setw(7) << std::setfill(' ') << SF._uORB_Flow_Altitude_Final << "            \n";
	std::cout << " FlowXFix:" << std::setw(7) << std::setfill(' ') << SF._uORB_Flow_Fix_XOutput;
	std::cout << " ||FlowYFix:" << std::setw(7) << std::setfill(' ') << SF._uORB_Flow_Fix_YOutput << "            \n";

	std::cout << "GPSDataINFO:"
			  << "\n";
	std::cout << " GPSLAT:    " << std::setw(10) << std::setfill(' ') << (int)SF._uORB_GPS_Lat_Smooth
			  << " ||GPSRawLat:  " << std::setw(10) << std::setfill(' ') << (int)SF._uORB_GPS_Data.lat
			  << " ||GPSHoldLAT: " << std::setw(10) << std::setfill(' ') << PF._uORB_PID_GPS_Lat_Local_Target
			  << " ||GPSPitchOut:" << std::setw(10) << std::setfill(' ') << PF._uORB_PID_GPS_Pitch_Ouput << "            \n";
	std::cout << " GPSLNG:    " << std::setw(10) << std::setfill(' ') << (int)SF._uORB_GPS_Lng_Smooth
			  << " ||GPSRawLng:  " << std::setw(10) << std::setfill(' ') << (int)SF._uORB_GPS_Data.lng
			  << " ||GPSHoldLNG: " << std::setw(10) << std::setfill(' ') << PF._uORB_PID_GPS_Lng_Local_Target
			  << " ||GPSRollOut: " << std::setw(10) << std::setfill(' ') << PF._uORB_PID_GPS__Roll_Ouput << "            \n";
	std::cout << " GPSNE:      " << SF._uORB_GPS_Data.lat_North_Mode << " -> " << SF._uORB_GPS_Data.lat_East_Mode << "            \n";
	std::cout << " GPSSATCount:" << SF._uORB_GPS_Data.satillitesCount << "            \n";

	std::cout << "RCOutPUTINFO:   "
			  << "\n";
	std::cout << " ChannelRoll  "
			  << ": " << RF._uORB_RC_Out__Roll << std::setw(5) << std::setfill(' ') << " ||";
	std::cout << " ChannelPitch "
			  << ": " << RF._uORB_RC_Out_Pitch << std::setw(5) << std::setfill(' ') << " ||";
	std::cout << " ChannelThrot "
			  << ": " << RF._uORB_RC_Out_Throttle << std::setw(5) << std::setfill(' ') << " ||";
	std::cout << " ChannelYaw   "
			  << ": " << RF._uORB_RC_Out___Yaw << std::setw(5) << std::setfill(' ') << " \n";

	std::cout << "ChannelINFO: "
			  << " \n";
	for (size_t i = 0; i < 16; i++)
	{
		std::cout << " " << RF._uORB_RC_Channel_PWM[i] << " ";
	}
	std::cout << " \n\n";
	std::cout << " Flag_ESC_ARMED:         " << std::setw(3) << std::setfill(' ') << AF._flag_ESC_ARMED << " |";
	std::cout << " Flag_Error:             " << std::setw(3) << std::setfill(' ') << AF._flag_Error << " |";
	std::cout << " Flag_GPS_Error:         " << std::setw(3) << std::setfill(' ') << AF._flag_GPS_Error << "           \n";
	std::cout << " Flag_ClockingTime_Error:" << std::setw(3) << std::setfill(' ') << AF._flag_ClockingTime_Error << " |";
	std::cout << " Flag_RC_Disconnected:   " << std::setw(3) << std::setfill(' ') << AF._flag_RC_Disconnected << " |";
	std::cout << " Flag_GPS_Disconnected:  " << std::setw(3) << std::setfill(' ') << AF._flag_GPS_Disconnected << "         \n";
	std::cout << " Flag_IsAltHoldSet:      " << std::setw(3) << std::setfill(' ') << AF._flag_IsAltHoldSet << " |";
	std::cout << " Flag_IsGPSHoldSet:      " << std::setw(3) << std::setfill(' ') << AF._flag_IsGPSHoldSet << " |";
	std::cout << " Flag_IsFlowHoldSet:     " << std::setw(3) << std::setfill(' ') << AF._flag_IsFlowHoldSet << "           \n";
	std::cout << " Flag_IsFlowAvalible:    " << std::setw(3) << std::setfill(' ') << AF._flag_IsFlowAvalible << " |";
	std::cout << " GPS_Lose_Clocking:      " << std::setw(3) << std::setfill(' ') << AF.GPS_Lose_Clocking << " |";
	std::cout << " RC_Lose_Clocking:       " << std::setw(3) << std::setfill(' ') << AF.RC_Lose_Clocking << "                        \n\n";

	std::cout << " IMULoopTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_IMUThreadTimeLoop;
	std::cout << " IMUNextTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_IMUThreadTimeNext;
	std::cout << " IMUErrorTime:" << std::setw(7) << std::setfill(' ') << TF._flag_IMUErrorTimes;
	std::cout << " IMUMaxTime:  " << std::setw(7) << std::setfill(' ') << TF._Tmp_IMUThreadError << "    \n";
	std::cout << " RXTLoopTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_RXTThreadTimeLoop;
	std::cout << " RXTNextTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_RXTThreadTimeNext;
	std::cout << " RXTErrorTime:" << std::setw(7) << std::setfill(' ') << TF._flag_RXTErrorTimes;
	std::cout << " RXTMaxTime:  " << std::setw(7) << std::setfill(' ') << TF._Tmp_RXTThreadError << "    \n";
	std::cout << " ESCLoopTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_ESCThreadTimeLoop;
	std::cout << " ESCNextTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_ESCThreadTimeNext;
	std::cout << " ESCErrorTime:" << std::setw(7) << std::setfill(' ') << TF._flag_ESCErrorTimes;
	std::cout << " ESCMaxTime:  " << std::setw(7) << std::setfill(' ') << TF._Tmp_ESCThreadError << "    \n";
	std::cout << " ALTLoopTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_ALTThreadTimeLoop;
	std::cout << " ALTNextTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_ALTThreadTimeNext;
	std::cout << " ALTErrorTime:" << std::setw(7) << std::setfill(' ') << TF._flag_ALTErrorTimes;
	std::cout << " ALTMaxTime:  " << std::setw(7) << std::setfill(' ') << TF._Tmp_ALTThreadError << "    \n";
	std::cout << " GPSLoopTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_GPSThreadTimeLoop;
	std::cout << " GPSNextTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_GPSThreadTimeNext;
	std::cout << " GPSErrorTime:" << std::setw(7) << std::setfill(' ') << TF._flag_GPSErrorTimes;
	std::cout << " GPSMaxTime:  " << std::setw(7) << std::setfill(' ') << TF._Tmp_GPSThreadError << "    \n";
	std::cout << " MAGLoopTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_MAGThreadTimeLoop;
	std::cout << " MAGNextTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_MAGThreadTimeNext;
	std::cout << " MAGErrorTime:" << std::setw(7) << std::setfill(' ') << TF._flag_MAGErrorTimes;
	std::cout << " MAGMaxTime:  " << std::setw(7) << std::setfill(' ') << TF._Tmp_MAGThreadError << "    \n";
	std::cout << " FloLoopTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_FlowThreadTimeLoop;
	std::cout << " FloNextTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_FlowThreadTimeNext;
	std::cout << " FloErrorTime:" << std::setw(7) << std::setfill(' ') << TF._flag_FlowErrorTimes;
	std::cout << " FloMaxTime:  " << std::setw(7) << std::setfill(' ') << TF._Tmp_FlowThreadError << "    \n"
			  << std::endl;
	std::cout << "<===============================UserDefineMessager====================================>\n";
	std::queue<std::string> TmpBuffer = DEBUGBuffer;
	for (int i = 0; i < DEBUGBuffer.size(); i++)
	{
		std::cout << TmpBuffer.front();
		TmpBuffer.pop();
	}
}

void SingleAPMAPI::RPiSingleAPM::DebugScreenPush(std::string PushMessage)
{
	if (DEBUGBuffer.size() == USERDEBUGINPUTSIZE)
		DEBUGBuffer.pop();
	DEBUGBuffer.push(PushMessage);
}