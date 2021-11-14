#include "SingleAPM.hpp"

int SingleAPMAPI::RPiSingleAPM::RPiSingleAPMInit(APMSettinngs APMInit)
{
	{
		DF.APMStatus = -1;
		AF.RC_Lose_Clocking = 0;
		AF.GPS_Lose_Clocking = 0;
		AF.Flow_Lose_Clocking = 0;
		AF.FakeRC_Deprive_Clocking = 0;
		AF.AngleLimit_Out_Clocking = 0;
		AF._flag_FakeRC_Disconnected = false;
		AF._flag_AnagleOutOfLimit = false;
		AF._flag_ESC_ARMED = true;
		AF._flag_IsINUHDiable = true;
		AF._flag_IsFlowAvalible = false;
		AF._flag_StartUP_Protect = true;
		AF._flag_IsSonarAvalible = false;
		AF._flag_Device_setupFailed = true;
		AF._flag_MPU9250_first_StartUp = true;
		AF._flag_ESC_DISARMED_Request = false;
		AF._flag_FakeRC_Error = true;
		AF._flag_FakeRC_Deprive = true;
		AF._flag_IsFakeRCUpdated = false;
		AF._flag_IsNotTakeOff = false;
		AF._flag_FakeRC_Disconnected = false;
		AF._flag_PreARM_Check = false;
		AF._flag_PreARM_Check_Lock = false;
		AF.AutoPilotMode = APModeINFO::AutoStable;
		AF._flag_FailedSafe_Level = 0;
		AF._flag_PreARM_Check_Level = 0;
		PF._uORB_Accel_Dynamic_Beta = PF._flag_Accel_Config_Beta;

		TF._Tmp_IMUNavThreadDT = GetTimestamp();
		TF._Tmp_IMUNavThreadLast = GetTimestamp();
		TF._Tmp_IMUAttThreadDT = GetTimestamp();
		TF._Tmp_IMUAttThreadLast = GetTimestamp();
	}
	ConfigReader(APMInit);
	//--------------------------------------------------------------------//
	{
#ifdef RPiDEBUGStart
		std::cout << "[RPiSingleAPM]ESCControllerIniting \n";
#endif
		DF.ESCDevice.reset(new ESCGenerator(EF.ESCControllerType, "/dev/i2c-1", EF.ESCPLFrequency));
	}
	//--------------------------------------------------------------------//
	{
		if (RF.RC_Type == RCIsIbus)
		{
#ifdef RPiDEBUGStart
			std::cout << "[RPiSingleAPM]Controller Ibus config comfirm\n";
#endif
			DF.IbusInit.reset(new Ibus(DF.RCDevice.c_str()));
		}
		else if (RF.RC_Type == RCIsSbus)
		{
#ifdef RPiDEBUGStart
			std::cout << "[RPiSingleAPM]Controller Sbus config comfirm\n";
#endif
			DF.SbusInit.reset(new Sbus(DF.RCDevice.c_str()));
		}
	}
	//--------------------------------------------------------------------//
	{
		if (DF._IsBAROEnable)
		{
#ifdef RPiDEBUGStart
			std::cout << "[RPiSingleAPM]Baro initializating ......";
#endif
			std::cout.flush();
			DF.BaroDeviceD.reset(new BaroDevice(BaroType::MS5611, "/dev/i2c-1", 0x77));
#ifdef RPiDEBUGStart
			std::cout << "Done!\n";
#endif
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
			DF.GPSInit.reset(new GPSUart(DF.GPSDevice.c_str()));
			// DF.CompassDevice.reset(new GPSI2CCompass(COMPASS_QMC5883L));
			// DF.CompassDevice->CompassCalibration(false, SF._flag_MPU_MAG_Cali);
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
			DF.FlowInit.reset(new MSPUartFlow(DF.FlowDevice.c_str()));
#ifdef RPiDEBUGStart
			std::cout << "Done \n";
#endif
		}
	}
	//--------------------------------------------------------------------//
	{
		MPUConfig config;
		config.MPUType = SF.MPU9250_Type;
		config.MPUSPIChannel = DF.MPUDeviceSPI.c_str();
		config.MPUI2CAddress = DF.MPU9250_ADDR;
		config.MPU9250_SPI_Freq = 1000 * 1000;
		//
		config.TargetFreqency = TF._flag_IMUThreadFreq;
		config.GyroToAccelBeta = SF._flag_Filter_AngleMix_Alpha;
		config.GyroDynamicAnalyse = true;
		//
		config.GyroHardwareFilterFreq = 250;
		//
		config.GyroFilterType = SF._flag_Filter_Gryo_Type;
		config.GyroFilterTypeST2 = SF._flag_Filter_GryoST2_Type;
		config.GyroFilterCutOff = SF._flag_Filter_Gryo_CutOff;
		config.GyroFilterCutOffST2 = SF._flag_Filter_GryoST2_CutOff;
		config.GyroFilterNotchCenterFreq = SF._flag_Filter_Gryo_NotchFreq;
		config.GyroFilterNotchCutOff = SF._flag_Filter_Gryo_NotchCutOff;
		//
		config.DynamicNotchQ = SF._flag_Filter_Gryo_DynamicNotchRange;
		config.DynamicNotchEnable = SF._flag_Filter_Gryo_DynamicNotchEnable;
		config.DynamicNotchMinFreq = SF._flag_Filter_Gryo_DynamicNotchMinFreq;
		//
		config.AccTargetFreqency = ACCEL_UPDATE_HZ;
		config.AccelFilterType = SF._flag_Filter_Accel_Type;
		config.AccelFilterCutOff = SF._flag_Filter_Accel_CutOff;
		DF.MPUDevice.reset(new RPiMPU9250(config));
#ifdef RPiDEBUGStart
		std::cout << "[RPiSingleAPM]MPU Calibrating Gryo ......";
		std::cout.flush();
#endif
		DF.MPUDevice->MPUCalibration(SF._flag_MPU_Accel_Cali);
#ifdef RPiDEBUGStart
		std::cout << "Done!"
				  << "\n";
#endif
	}
	//--------------------------------------------------------------------//
	{
		pt1FilterInit(&DF.ThrottleLPF, FILTERTHROTTLELPFCUTOFF, 0.f);
		pt1FilterInit(&DF.POSOutLPF[0], FILTERPOSOUTLPFCUTOFF, 0.f);
		pt1FilterInit(&DF.POSOutLPF[1], FILTERPOSOUTLPFCUTOFF, 0.f);
		pt1FilterInit(&DF.AngleRateLPF[0], PF._flag_Filter_AngleRate_CutOff, 0.f);
		pt1FilterInit(&DF.AngleRateLPF[1], PF._flag_Filter_AngleRate_CutOff, 0.f);
		pt1FilterInit(&DF.AngleRateLPF[2], PF._flag_Filter_AngleRate_CutOff, 0.f);
		pt1FilterInit(&DF.ItermFilterPitch, PF._flag_Filter_PID_I_CutOff, 0.f);
		pt1FilterInit(&DF.ItermFilterRoll, PF._flag_Filter_PID_I_CutOff, 0.f);
		pt1FilterInit(&DF.DtermFilterPitch, PF._flag_Filter_PID_D_ST1_CutOff, 0.f);
		pt1FilterInit(&DF.DtermFilterRoll, PF._flag_Filter_PID_D_ST1_CutOff, 0.f);
		pt1FilterInit(&DF.DtermFilterPitchST2, PF._flag_Filter_PID_D_ST2_CutOff, 0.f);
		pt1FilterInit(&DF.DtermFilterRollST2, PF._flag_Filter_PID_D_ST2_CutOff, 0.f);
		pt1FilterInit(&DF.RCLPF[0], RF._flag_Filter_RC_CutOff, 0.f);
		pt1FilterInit(&DF.RCLPF[1], RF._flag_Filter_RC_CutOff, 0.f);
		pt1FilterInit(&DF.RCLPF[2], RF._flag_Filter_RC_CutOff, 0.f);
		pt1FilterInit(&DF.BAROLPF, FILTERBAROLPFCUTOFF, 0.0f);
	}
	//--------------------------------------------------------------------//
	{
		char outTime[16];
		std::time_t t = std::time(NULL);
		std::strftime(outTime, sizeof(outTime), "%Y%m%d%H%M%S", std::localtime(&t));
		std::string file = (std::string(BlackBoxLogDir, sizeof(BlackBoxLogDir) - 1) +
							std::string("Singleflight", sizeof("Singleflight") - 1) +
							std::string("-", sizeof("-") - 1) +
							std::string(outTime, sizeof(outTime) - 2) +
							std::string(".log", sizeof(".log") - 1));
		DF.BlackBoxFile.open(file.c_str(), std::ios::out | std::ios::binary);

		std::vector<BlackboxList> BlackBoxIInfo = {
			{.FrameName = "loopIteration", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
			{.FrameName = "time", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
			{.FrameName = "axisRate[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "axisRate[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "axisRate[2]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "rcCommand[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "rcCommand[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "rcCommand[2]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "rcCommand[3]", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
			{.FrameName = "attitude[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "attitude[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "attitude[2]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "gyroADC[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "gyroADC[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "gyroADC[2]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "accSmooth[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "accSmooth[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "accSmooth[2]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "motor[0]", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
			{.FrameName = "motor[1]", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
			{.FrameName = "motor[2]", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
			{.FrameName = "motor[3]", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
		};

		std::vector<BlackboxList> BlackBoxPInfo = {
			{.FrameName = "loopIteration", .FrameSigned = 0, .FramePredictor = 6, .FrameEncoder = 9},
			{.FrameName = "time", .FrameSigned = 0, .FramePredictor = 2, .FrameEncoder = 0},
			{.FrameName = "axisRate[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "axisRate[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "axisRate[2]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "rcCommand[0]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
			{.FrameName = "rcCommand[1]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
			{.FrameName = "rcCommand[2]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
			{.FrameName = "rcCommand[3]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
			{.FrameName = "attitude[0]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
			{.FrameName = "attitude[1]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
			{.FrameName = "attitude[2]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
			{.FrameName = "gyroADC[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "gyroADC[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "gyroADC[2]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
			{.FrameName = "accSmooth[0]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
			{.FrameName = "accSmooth[1]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
			{.FrameName = "accSmooth[2]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
			{.FrameName = "motor[0]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
			{.FrameName = "motor[1]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
			{.FrameName = "motor[2]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
			{.FrameName = "motor[3]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
		};

		DF.BlackBoxDevice.reset(new BlackboxEncoder({
			.IInterval = BlackBoxIInterval,
			.PInterval = BlackBoxPInterval,
			.FirmwareType = BlackBoxFirmware,
			.BlackBoxDataIInfo = BlackBoxIInfo,
			.BlackBoxDataPInfo = BlackBoxPInfo,
			.BlackBoxCustom = {
				"H Firmware revision:Singleflight 0.9.5 BETA Raspberrypi4B\n",
				"H acc_1G:" + std::to_string((int)MPU9250_ACCEL_LSB) + "\n",
				"H gyro_scale:0x3f800000\n",
				"H motorOutput:1000,2000\n",
				"H minthrottle:1000\n",
				"H maxthrottle:2000\n",
				("H looptime:" + std::to_string(TF._flag_IMUThreadTimeMax + LINUX_SYSTEM_SLEEP_DELAY) + "\n"),
			},
		}));
	}
	sleep(2);
#ifdef RPiDEBUGStart
	system("clear");
#endif
	AF._flag_Device_setupFailed = false;
	DF.APMStatus = 1;
	return 0;
}

void SingleAPMAPI::RPiSingleAPM::RPiSingleAPMStartUp()
{
	{
		IMUSensorsTaskReg();

		AltholdSensorsTaskReg();

		ControllerTaskReg();

		PositionTaskReg();

		ESCUpdateTaskReg();

		BlackBoxTaskReg();

		DF.APMStatus += 1;
	}
}

void SingleAPMAPI::RPiSingleAPM::RPiSingleAPMDeInit()
{
	//--------------------------------------------------------------------//
	TF._flag_IMU_Task_Running = false;
	TF._flag_RXT_Task_Running = false;
	TF._flag_ESC_Task_Running = false;
	TF._flag_ALT_Task_Running = false;
	TF._flag_GPS_Task_Running = false;
	TF._flag_MAG_Task_Running = false;
	TF._flag_Flow_Task_Running = false;
	TF._flag_Block_Task_Running = false;
	TF._flag_BBQ_Task_Running = false;
	TF._flag_BBW_Task_Running = false;
	if (TF.IMUTask.joinable())
		TF.IMUTask.join();
	if (TF.RXTask.joinable())
		TF.RXTask.join();
	if (TF.ESCTask.joinable())
		TF.ESCTask.join();
	if (TF.ALTTask.joinable())
		TF.ALTTask.join();
	if (TF.GPSTask.joinable())
		TF.GPSTask.join();
	if (TF.MAGTask.joinable())
		TF.MAGTask.join();
	if (TF.FlowTask.joinable())
		TF.FlowTask.join();
	if (TF.BlackBoxQTask.joinable())
		TF.BlackBoxQTask.join();
	if (TF.BlackBoxWTask.joinable())
		TF.BlackBoxWTask.join();
	//--------------------------------------------------------------------//
	usleep(5000);
	if (DF.ESCDevice.get() != nullptr)
		DF.ESCDevice->ESCClear(PCA9685_ALL_PIN);
	DF.ESCDevice.reset();
	DF.IbusInit.reset();
	DF.SbusInit.reset();
	DF.BaroDeviceD.reset();
	DF.GPSInit.reset();
	// DF.CompassDevice.reset();
	DF.FlowInit.reset();
	DF.MPUDevice.reset();
	DF.BlackBoxDevice.reset();
	//
	DF.BlackBoxFile << 'E';
	DF.BlackBoxFile << (uint8_t)0xFF;
	DF.BlackBoxFile << "End of log";
	DF.BlackBoxFile << (uint8_t)0x00;
	DF.BlackBoxFile.close();
	//--------------------------------------------------------------------//
	{
		AF.RC_Lose_Clocking = 0;
		AF.GPS_Lose_Clocking = 0;
		AF.Flow_Lose_Clocking = 0;
		AF.FakeRC_Deprive_Clocking = 0;
		AF._flag_FakeRC_Disconnected = 0;
		AF._flag_ESC_ARMED = true;
		AF._flag_IsINUHDiable = true;
		AF._flag_IsFlowAvalible = false;
		AF._flag_StartUP_Protect = true;
		AF._flag_IsSonarAvalible = false;
		AF._flag_Device_setupFailed = true;
		AF._flag_MPU9250_first_StartUp = true;
		AF._flag_ESC_DISARMED_Request = false;
		AF._flag_FakeRC_Error = true;
		AF._flag_FakeRC_Deprive = true;
		AF._flag_IsFakeRCUpdated = false;
		AF._flag_IsNotTakeOff = false;
		AF._flag_FakeRC_Disconnected = false;
		AF._flag_PreARM_Check = false;
		AF._flag_PreARM_Check_Lock = false;
		AF.AutoPilotMode = APModeINFO::AutoStable;
		AF._flag_FailedSafe_Level = 0;
		AF._flag_PreARM_Check_Level = 0;
		PF._uORB_Accel_Dynamic_Beta = PF._flag_Accel_Config_Beta;
		DF.APMStatus = -2;
	}
}

void SingleAPMAPI::RPiSingleAPM::IMUSensorsTaskReg()
{
	TF.IMUTask = std::thread(
		[&] {
			TF._flag_IMU_Task_Running = true;
			while (TF._flag_IMU_Task_Running)
			{
				TF._Tmp_IMUThreadTimeStart = GetTimestamp();
				TF._Tmp_IMUThreadTimeNext = TF._Tmp_IMUThreadTimeStart - TF._Tmp_IMUThreadTimeEnd;

				SF._uORB_MPU_Data = DF.MPUDevice->MPUSensorsDataGet();

				//============Online Catlibration======================================//
				{
					if (AF._flag_MPUCalibrating == 1)
					{
						if (AF._flag_ESC_ARMED &&
							((AF._flag_PreARM_Check_Level & FailedSafeFlag::_flag_PreARMFailed_GyroNotStable) ||
							 (AF._flag_PreARM_Check_Level & FailedSafeFlag::_flag_PreARMFailed_AccelNotStable)))
						{
							AF._flag_MPUCalibrating = DF.MPUDevice->MPUCalibrationOnline();
							AF._flag_MPUCalibratingSet = true;
						}
						SF._flag_MPUCalibratorWaitClock = 0;
					}
					else if (SF._flag_MPUCalibratorWaitClock > CalibratorLimitTime)
					{
						if (AF._flag_ESC_ARMED &&
							((AF._flag_PreARM_Check_Level & FailedSafeFlag::_flag_PreARMFailed_GyroNotStable) ||
							 (AF._flag_PreARM_Check_Level & FailedSafeFlag::_flag_PreARMFailed_AccelNotStable)))
						{
							AF._flag_MPUCalibrating = 1;
						}
						else
							AF._flag_MPUCalibrating = 0;
						AF._flag_MPUCalibratingSet = false;
						SF._flag_MPUCalibratorWaitClock = 0;
					}
					else
					{
						SF._flag_MPUCalibratorWaitClock += 1.f / (float)TF._flag_IMUThreadTimeMax * 1000000.f;
						AF._flag_MPUCalibratingSet = false;
					}
				}
				//=====================================================================//

				if (SF._uORB_MPU_Data._uORB_MPU9250_AccelCountDown == 1)
					NavigationUpdate();

				AttitudeUpdate();

				if (AF._flag_MPU9250_first_StartUp)
				{
					DF.MPUDevice->ResetMPUMixAngle();
					AF._flag_MPU9250_first_StartUp = false;
				}
				TF._Tmp_IMUThreadTimeEnd = GetTimestamp();
				TF._Tmp_IMUThreadTimeLoop = TF._Tmp_IMUThreadTimeEnd - TF._Tmp_IMUThreadTimeStart;
				if (TF._Tmp_IMUThreadTimeLoop + TF._Tmp_IMUThreadTimeNext > TF._flag_IMUThreadTimeMax | TF._Tmp_IMUThreadTimeNext < 0)
				{
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
				TF._Tmp_IMUThreadTimeEnd = GetTimestamp();
			}
		});
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(3, &cpuset);
	int rc = pthread_setaffinity_np(TF.IMUTask.native_handle(), sizeof(cpu_set_t), &cpuset);
}

void SingleAPMAPI::RPiSingleAPM::AltholdSensorsTaskReg()
{
	if (DF._IsBAROEnable)
	{
		TF.ALTTask = std::thread(
			[&] {
				TF._flag_ALT_Task_Running = true;
				while (TF._flag_ALT_Task_Running)
				{
					TF._Tmp_ALTThreadTimeStart = GetTimestamp();
					TF._Tmp_ALTThreadTimeNext = TF._Tmp_ALTThreadTimeStart - TF._Tmp_ALTThreadTimeEnd;

					SF._uORB_BARO_Data = DF.BaroDeviceD->BaroRead();
					float DT = (TF._flag_ALTThreadTimeMax / 1000000.f);
					SF._uORB_BARO_Altitude = pt1FilterApply3(&DF.BAROLPF, (SF._uORB_BARO_Data.AltitudeM * 100.f), DT);
					//
					AF._flag_BARO_Async = true;

					TF._Tmp_ALTThreadTimeEnd = GetTimestamp();
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
					TF._Tmp_ALTThreadTimeEnd = GetTimestamp();
				}
			});
		cpu_set_t cpuset;
		CPU_ZERO(&cpuset);
		CPU_SET(3, &cpuset);
		int rc = pthread_setaffinity_np(TF.ALTTask.native_handle(), sizeof(cpu_set_t), &cpuset);
	}
}

void SingleAPMAPI::RPiSingleAPM::ControllerTaskReg()
{
	TF.RXTask = std::thread(
		[&] {
			TF._flag_RXT_Task_Running = true;
			while (TF._flag_RXT_Task_Running)
			{
				TF._Tmp_RXTThreadTimeStart = GetTimestamp();
				TF._Tmp_RXTThreadTimeNext = TF._Tmp_RXTThreadTimeStart - TF._Tmp_RXTThreadTimeEnd;

				//RC Read Parse
				{
					if (RF.RC_Type == RCIsSbus)
					{
						if (DF.SbusInit->SbusRead(RF._Tmp_RC_Data, 0, 1) != -1)
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
						if (DF.IbusInit->IbusRead(RF._Tmp_RC_Data, 0, 1) != -1)
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

					if (AF._flag_IsFakeRCUpdated && !AF._flag_FakeRC_Error && AF._flag_RC_Error)
					{
						for (size_t i = 4; i < 10; i++)
						{
							RF._uORB_RC_Channel_PWM[i] = RF._uORB_FakeRC_Channel_PWM[i];
						}
					}
				}
				//RC Out Caculation
				{
					if (!AF._flag_RC_Disconnected)
					{
						float DT = (float)TF._flag_RXTThreadTimeMax / 1000000.f;
						RF._uORB_RC_Channel_PWM[0] = pt1FilterApply4(&DF.RCLPF[0], (float)RF._uORB_RC_Channel_PWM[0],
																	 RF._flag_Filter_RC_CutOff, DT);
						RF._uORB_RC_Channel_PWM[1] = pt1FilterApply4(&DF.RCLPF[1], (float)RF._uORB_RC_Channel_PWM[1],
																	 RF._flag_Filter_RC_CutOff, DT);
						RF._uORB_RC_Channel_PWM[3] = pt1FilterApply4(&DF.RCLPF[2], (float)RF._uORB_RC_Channel_PWM[3],
																	 RF._flag_Filter_RC_CutOff, DT);
					}
					if (RF._uORB_RC_Channel_PWM[0] < RF._flag_RC_Mid_PWM_Value + 10 && RF._uORB_RC_Channel_PWM[0] > RF._flag_RC_Mid_PWM_Value - 10)
						RF._Tmp_RC_Out__Roll = 0;
					else
						RF._Tmp_RC_Out__Roll = (RF._uORB_RC_Channel_PWM[0] - RF._flag_RC_Mid_PWM_Value) * RF._flag_RCIsReserv__Roll;
					//
					if (RF._uORB_RC_Channel_PWM[1] < RF._flag_RC_Mid_PWM_Value + 10 && RF._uORB_RC_Channel_PWM[1] > RF._flag_RC_Mid_PWM_Value - 10)
						RF._Tmp_RC_Out_Pitch = 0;
					else
						RF._Tmp_RC_Out_Pitch = (RF._uORB_RC_Channel_PWM[1] - RF._flag_RC_Mid_PWM_Value) * RF._flag_RCIsReserv_Pitch;
					//
					if (RF._uORB_RC_Channel_PWM[3] < RF._flag_RC_Mid_PWM_Value + 10 && RF._uORB_RC_Channel_PWM[3] > RF._flag_RC_Mid_PWM_Value - 10)
						RF._Tmp_RC_Out___Yaw = 0;
					else
						RF._Tmp_RC_Out___Yaw = (RF._uORB_RC_Channel_PWM[3] - RF._flag_RC_Mid_PWM_Value) * RF._flag_RCIsReserv___Yaw;
					//
					RF._Tmp_RC_Out_Throttle = RF._uORB_RC_Channel_PWM[2];
					//================================================//
					if (!AF._flag_FakeRC_Error)
					{
						if (RF._Tmp_RC_Out__Roll != 0 || RF._Tmp_RC_Out_Pitch != 0)
						{
							AF.FakeRC_Deprive_Clocking++;
						}
						else
						{
							if (AF.FakeRC_Deprive_Clocking > 0)
								AF.FakeRC_Deprive_Clocking--;
							if (AF.FakeRC_Deprive_Clocking == 0)
								AF._flag_FakeRC_Deprive = false;
						}

						if (AF.FakeRC_Deprive_Clocking >= 50)
						{
							AF.FakeRC_Deprive_Clocking = 50;
							AF._flag_FakeRC_Deprive = true;
						}
					}
					if (!AF._flag_FakeRC_Deprive && !AF._flag_FakeRC_Error && AF._flag_RC_Error)
					{
						if (RF._uORB_FakeRC_Channel_PWM[0] < RF._flag_RC_Mid_PWM_Value + 10 && RF._uORB_FakeRC_Channel_PWM[0] > RF._flag_RC_Mid_PWM_Value - 10)
							RF._Tmp_RC_Out__Roll = 0;
						else
							RF._Tmp_RC_Out__Roll = (RF._uORB_FakeRC_Channel_PWM[0] - RF._flag_RC_Mid_PWM_Value) * RF._flag_RCIsReserv__Roll;
						//
						if (RF._uORB_FakeRC_Channel_PWM[1] < RF._flag_RC_Mid_PWM_Value + 10 && RF._uORB_FakeRC_Channel_PWM[1] > RF._flag_RC_Mid_PWM_Value - 10)
							RF._Tmp_RC_Out_Pitch = 0;
						else
							RF._Tmp_RC_Out_Pitch = (RF._uORB_FakeRC_Channel_PWM[1] - RF._flag_RC_Mid_PWM_Value) * RF._flag_RCIsReserv_Pitch;
						//
						if (RF._uORB_FakeRC_Channel_PWM[3] < RF._flag_RC_Mid_PWM_Value + 10 && RF._uORB_FakeRC_Channel_PWM[3] > RF._flag_RC_Mid_PWM_Value - 10)
							RF._Tmp_RC_Out___Yaw = 0;
						else
							RF._Tmp_RC_Out___Yaw = (RF._uORB_FakeRC_Channel_PWM[3] - RF._flag_RC_Mid_PWM_Value) * RF._flag_RCIsReserv___Yaw;
						//
						RF._Tmp_RC_Out_Throttle = RF._uORB_FakeRC_Channel_PWM[2];
					}
				}
				//RC Safty Checking
				{
					if ((RF._flag_RC_Min_PWM_Value < RF._uORB_RC_Channel_PWM[0] && RF._uORB_RC_Channel_PWM[0] > RF._flag_RC_Max_PWM_Value) ||
						(RF._flag_RC_Min_PWM_Value < RF._uORB_RC_Channel_PWM[1] && RF._uORB_RC_Channel_PWM[1] > RF._flag_RC_Max_PWM_Value) ||
						(RF._flag_RC_Min_PWM_Value < RF._uORB_RC_Channel_PWM[2] && RF._uORB_RC_Channel_PWM[2] > RF._flag_RC_Max_PWM_Value) ||
						(RF._flag_RC_Min_PWM_Value < RF._uORB_RC_Channel_PWM[3] && RF._uORB_RC_Channel_PWM[3] > RF._flag_RC_Max_PWM_Value))
						AF._flag_RC_Disconnected = true;
					if (AF._flag_RC_Disconnected == true)
					{
						AF.RC_Lose_Clocking += 1;
						if (AF.RC_Lose_Clocking >= 200)
						{
							AF._flag_RC_Error = true;
							AF.RC_Lose_Clocking = 0;
						}
					}
					else if (AF._flag_RC_Disconnected == false)
					{
						AF._flag_RC_Error = false;
						AF.RC_Lose_Clocking = 0;
					}
					//================================================//
					if (AF._flag_IsFakeRCUpdated)
					{
						AF._flag_IsFakeRCUpdated = false;
						AF.FakeRC_Lose_Clocking = 0;
						AF._flag_FakeRC_Error = false;
					}
					else
					{
						AF.FakeRC_Lose_Clocking += 1;
					}
					//
					if (AF.FakeRC_Lose_Clocking >= 100)
					{
						AF.FakeRC_Lose_Clocking = 0;
						AF._flag_FakeRC_Error = true;
					}
					//================================================//
				}
				//RC data out
				{
					RF._uORB_RC_Out__Roll = RF._Tmp_RC_Out__Roll;
					RF._uORB_RC_Out_Pitch = RF._Tmp_RC_Out_Pitch;
					RF._uORB_RC_Out___Yaw = RF._Tmp_RC_Out___Yaw;
					RF._uORB_RC_Out_Throttle = RF._Tmp_RC_Out_Throttle;
				}
				//RC UNLOCK Checking
				{
					if (AF.AutoPilotMode == APModeINFO::AutoStable || AF.AutoPilotMode == APModeINFO::RateHold)
					{
						if (RF._uORB_RC_Out_Throttle < RF._flag_RC_Min_PWM_Value + 20 &&
							RF._flag_RC_ARM_PWM_Value - 50 < RF._uORB_RC_Channel_PWM[RF._flag_RC_ARM_PWM_Channel] &&
							RF._uORB_RC_Channel_PWM[RF._flag_RC_ARM_PWM_Channel] < RF._flag_RC_ARM_PWM_Value + 50)
						{
							APMControllerDISARM(AF.AutoPilotMode);
						}
					}
					else if (AF.AutoPilotMode == APModeINFO::AltHold || AF.AutoPilotMode == APModeINFO::PositionHold ||
							 AF.AutoPilotMode == APModeINFO::SpeedHold)
					{
						if (RF._flag_RC_Mid_PWM_Value - 100 < RF._uORB_RC_Out_Throttle && RF._uORB_RC_Out_Throttle < RF._flag_RC_Mid_PWM_Value + 100 &&
							RF._flag_RC_ARM_PWM_Value - 50 < RF._uORB_RC_Channel_PWM[RF._flag_RC_ARM_PWM_Channel] &&
							RF._uORB_RC_Channel_PWM[RF._flag_RC_ARM_PWM_Channel] < RF._flag_RC_ARM_PWM_Value + 50)
						{
							APMControllerDISARM(AF.AutoPilotMode);
						}
					}
					else if (AF.AutoPilotMode == APModeINFO::UserAuto)
					{
						if (AF._flag_ESC_DISARMED_Request &&
							RF._flag_RC_ARM_PWM_Value - 50 < RF._uORB_RC_Channel_PWM[RF._flag_RC_ARM_PWM_Channel] &&
							RF._uORB_RC_Channel_PWM[RF._flag_RC_ARM_PWM_Channel] < RF._flag_RC_ARM_PWM_Value + 50)
							APMControllerDISARM(AF.AutoPilotMode);
						else if (!AF._flag_ESC_DISARMED_Request)
							APMControllerARMED();
					}
					//-----------------------------------------------------------------------------------------------------------//
					if (!(RF._flag_RC_ARM_PWM_Value - 50 < RF._uORB_RC_Channel_PWM[RF._flag_RC_ARM_PWM_Channel] &&
						  RF._uORB_RC_Channel_PWM[RF._flag_RC_ARM_PWM_Channel] < RF._flag_RC_ARM_PWM_Value + 50))
					{
						APMControllerARMED();
					}
					if (RF._flag_RC_ARM_PWM_Value - 50 < RF._uORB_RC_Channel_PWM[RF._flag_RC_ARM_PWM_Channel] &&
						RF._uORB_RC_Channel_PWM[RF._flag_RC_ARM_PWM_Channel] < RF._flag_RC_ARM_PWM_Value + 50)
					{
						if (AF.AutoPilotMode == APModeINFO::AutoStable || AF.AutoPilotMode == APModeINFO::RateHold)
						{
							if (RF._uORB_RC_Out_Throttle > RF._flag_RC_Min_PWM_Value + 20)
							{
								AF._flag_StartUP_Protect = true;
							}
						}
						if (AF.AutoPilotMode == APModeINFO::AltHold || AF.AutoPilotMode == APModeINFO::SpeedHold ||
							AF.AutoPilotMode == APModeINFO::SpeedHold || AF.AutoPilotMode == APModeINFO::UserAuto)
						{
							if (!(RF._flag_RC_Mid_PWM_Value - 100 < RF._uORB_RC_Out_Throttle && RF._uORB_RC_Out_Throttle < RF._flag_RC_Mid_PWM_Value + 100))
							{
								AF._flag_StartUP_Protect = true;
							}
						}
					}
				}
				//flyMode Switch
				{
					if (RF._flag_RC_AP_AutoStable_PWM_Value + 50 > RF._uORB_RC_Channel_PWM[RF._flag_RC_AP_AutoStable_PWM_Channel] &&
						RF._flag_RC_AP_AutoStable_PWM_Value - 50 < RF._uORB_RC_Channel_PWM[RF._flag_RC_AP_AutoStable_PWM_Channel])
					{
						AF.AutoPilotMode = APModeINFO::AutoStable;
					}
					if (RF._flag_RC_AP_RateHold_PWM_Value + 50 > RF._uORB_RC_Channel_PWM[RF._flag_RC_AP_RateHold_PWM_Channel] &&
						RF._flag_RC_AP_RateHold_PWM_Value - 50 < RF._uORB_RC_Channel_PWM[RF._flag_RC_AP_RateHold_PWM_Channel])
					{
						AF.AutoPilotMode = APModeINFO::RateHold;
					}
					else if (RF._flag_RC_AP_AltHold_PWM_Value + 50 > RF._uORB_RC_Channel_PWM[RF._flag_RC_AP_AltHold_PWM_Channel] &&
							 RF._flag_RC_AP_AltHold_PWM_Value - 50 < RF._uORB_RC_Channel_PWM[RF._flag_RC_AP_AltHold_PWM_Channel] &&
							 (DF._IsBAROEnable || DF._IsFlowEnable))
					{
						AF.AutoPilotMode = APModeINFO::AltHold;
					}
					else if (RF._flag_RC_AP_PositionHold_PWM_Value + 50 > RF._uORB_RC_Channel_PWM[RF._flag_RC_AP_PositionHold_PWM_Channel] &&
							 RF._flag_RC_AP_PositionHold_PWM_Value - 50 < RF._uORB_RC_Channel_PWM[RF._flag_RC_AP_PositionHold_PWM_Channel])
					{
						if ((DF._IsGPSEnable && DF._IsBAROEnable) || DF._IsFlowEnable)
							AF.AutoPilotMode = APModeINFO::PositionHold;
						else
							AF.AutoPilotMode = APModeINFO::AltHold;
					}
					else if (RF._flag_RC_AP_SpeedHold_PWM_Value + 50 > RF._uORB_RC_Channel_PWM[RF._flag_RC_AP_SpeedHold_PWM_Channel] &&
							 RF._flag_RC_AP_SpeedHold_PWM_Value - 50 < RF._uORB_RC_Channel_PWM[RF._flag_RC_AP_SpeedHold_PWM_Channel])
					{
						if ((DF._IsGPSEnable && DF._IsBAROEnable) || DF._IsFlowEnable)
							AF.AutoPilotMode = APModeINFO::SpeedHold;
						else
							AF.AutoPilotMode = APModeINFO::AltHold;
					}
					else if (RF._flag_RC_AP_UserAuto_PWM_Value + 50 > RF._uORB_RC_Channel_PWM[RF._flag_RC_AP_UserAuto_PWM_Channel] &&
							 RF._flag_RC_AP_UserAuto_PWM_Value - 50 < RF._uORB_RC_Channel_PWM[RF._flag_RC_AP_UserAuto_PWM_Channel])
					{
						if ((DF._IsGPSEnable && DF._IsBAROEnable) || DF._IsFlowEnable)
							AF.AutoPilotMode = APModeINFO::UserAuto;
						else
							AF.AutoPilotMode = APModeINFO::AltHold;
					}
					else
					{
						AF.AutoPilotMode = APModeINFO::AutoStable;
					}
				}
				//stick to speed controller
				{
					if (AF.AutoPilotMode == APModeINFO::AltHold || AF.AutoPilotMode == APModeINFO::PositionHold ||
						AF.AutoPilotMode == APModeINFO::SpeedHold)
					{
						if (RF._flag_RC_Mid_PWM_Value + 100 < RF._uORB_RC_Out_Throttle && RF._uORB_RC_Out_Throttle < RF._flag_RC_Max_PWM_Value + 100)
						{
							RF._uORB_RC_Out_AltHoldSpeed = ((500.f - (RF._flag_RC_Max_PWM_Value - RF._uORB_RC_Out_Throttle)) / 500.f) * PF._flag_PID_Alt_Speed_Max * -1;
						}
						else if (RF._flag_RC_Mid_PWM_Value - 100 > RF._uORB_RC_Out_Throttle && RF._uORB_RC_Out_Throttle > RF._flag_RC_Min_PWM_Value - 100)
						{
							RF._uORB_RC_Out_AltHoldSpeed = ((500.f - (RF._uORB_RC_Out_Throttle - RF._flag_RC_Min_PWM_Value)) / 500.f) * PF._flag_PID_Alt_Speed_Max;
						}
						else
						{
							RF._uORB_RC_Out_AltHoldSpeed = 0;
						}

						if (AF.AutoPilotMode == APModeINFO::SpeedHold)
						{
							if (RF._uORB_RC_Out__Roll < 0)
							{
								RF._uORB_RC_Out_PosHoldSpeedX = ((RF._uORB_RC_Out__Roll) / 500.f) * PF._flag_PID_PosMan_Speed_Max;
							}
							else if (RF._uORB_RC_Out__Roll > 0)
							{
								RF._uORB_RC_Out_PosHoldSpeedX = ((RF._uORB_RC_Out__Roll) / 500.f) * PF._flag_PID_PosMan_Speed_Max;
							}
							else
							{
								RF._uORB_RC_Out_PosHoldSpeedX = 0;
							}

							if (RF._uORB_RC_Out_Pitch < 0)
							{
								RF._uORB_RC_Out_PosHoldSpeedY = ((RF._uORB_RC_Out_Pitch) / 500.f) * PF._flag_PID_PosMan_Speed_Max;
							}
							else if (RF._uORB_RC_Out_Pitch > 0)
							{
								RF._uORB_RC_Out_PosHoldSpeedY = ((RF._uORB_RC_Out_Pitch) / 500.f) * PF._flag_PID_PosMan_Speed_Max;
							}
							else
							{
								RF._uORB_RC_Out_PosHoldSpeedY = 0;
							}
						}
					}
					else
					{
						RF._uORB_RC_Out_PosHoldSpeedX = 0;
						RF._uORB_RC_Out_PosHoldSpeedY = 0;
						RF._uORB_RC_Out_AltHoldSpeed = 0;
					}
				}
				//
				TF._Tmp_RXTThreadTimeEnd = GetTimestamp();
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
				TF._Tmp_RXTThreadTimeEnd = GetTimestamp();
			}
		});
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(3, &cpuset);
	int rc = pthread_setaffinity_np(TF.RXTask.native_handle(), sizeof(cpu_set_t), &cpuset);
}

void SingleAPMAPI::RPiSingleAPM::PositionTaskReg()
{
	if (DF._IsGPSEnable)
	{
		TF.GPSTask = std::thread(
			[&] {
				TF._flag_GPS_Task_Running = true;
				DF.GPSInit->GPSReOpen();
				while (TF._flag_GPS_Task_Running)
				{
					TF._Tmp_GPSThreadTimeStart = GetTimestamp();
					TF._Tmp_GPSThreadTimeNext = TF._Tmp_GPSThreadTimeStart - TF._Tmp_GPSThreadTimeEnd;
					{
						SF._uORB_GPS_Data = DF.GPSInit->GPSParse();
						if (SF._uORB_GPS_Data.lat != 0 && SF._uORB_GPS_Data.lng != 0 && !SF._uORB_GPS_Data.DataUnCorrect)
						{
							SF._uORB_GPS_COR_Lat = SF._uORB_GPS_Data.lat;
							SF._uORB_GPS_COR_Lng = SF._uORB_GPS_Data.lng;
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
					TF._Tmp_GPSThreadTimeEnd = GetTimestamp();
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
					TF._Tmp_GPSThreadTimeEnd = GetTimestamp();
				}
			});

		cpu_set_t cpuset;
		CPU_ZERO(&cpuset);
		CPU_SET(3, &cpuset);
		int rc = pthread_setaffinity_np(TF.GPSTask.native_handle(), sizeof(cpu_set_t), &cpuset);

		TF.MAGTask = std::thread(
			[&] {
				TF._flag_MAG_Task_Running = true;
				while (TF._flag_MAG_Task_Running)
				{
					TF._Tmp_MAGThreadTimeStart = GetTimestamp();
					TF._Tmp_MAGThreadTimeNext = TF._Tmp_MAGThreadTimeStart - TF._Tmp_MAGThreadTimeEnd;
					{
						// DF.CompassDevice->CompassUpdate();
						// DF.CompassDevice->CompassGetFixAngle(SF._uORB_MAG_Yaw, SF._uORB_MPU_Data._uORB_Real__Roll, SF._uORB_MPU_Data._uORB_Real_Pitch);
						// DF.CompassDevice->CompassGetUnfixAngle(SF._uORB_MAG_Yaw);
						// DF.CompassDevice->CompassGetRaw(SF._uORB_MAG_RawX, SF._uORB_MAG_RawY, SF._uORB_MAG_RawZ);
					}
					TF._Tmp_MAGThreadTimeEnd = GetTimestamp();
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
					TF._Tmp_MAGThreadTimeEnd = GetTimestamp();
				}
			});
		cpu_set_t cpuset2;
		CPU_ZERO(&cpuset2);
		CPU_SET(3, &cpuset2);
		int rc2 = pthread_setaffinity_np(TF.MAGTask.native_handle(), sizeof(cpu_set_t), &cpuset2);
	}

	if (DF._IsFlowEnable)
	{
		TF.FlowTask = std::thread(
			[&] {
				TF._flag_Flow_Task_Running = true;
				while (TF._flag_Flow_Task_Running)
				{
					TF._Tmp_FlowThreadTimeStart = GetTimestamp();
					TF._Tmp_FlowThreadTimeNext = TF._Tmp_FlowThreadTimeStart - TF._Tmp_FlowThreadTimeEnd;
					{
						TF._Tmp_FlowThreadSMooth++;
						SF._Tmp_Flow___Status = DF.FlowInit->MSPDataRead(SF._uORB_Flow_XOutput, SF._uORB_Flow_YOutput, SF._Tmp_Flow_Altitude, SF._uORB_Flow_Quality);
						//
						if (SF._Tmp_Flow___Status == 1 || SF._Tmp_Flow___Status == 3)
						{
							SF._uORB_Flow_Altitude = SF._Tmp_Flow_Altitude * (1.f - cos(abs(SF._uORB_MPU_Data._uORB_Real_Pitch) * 3.14 / 180.f) *
																						(1.f - cos(abs(SF._uORB_MPU_Data._uORB_Real__Roll) * 3.14 / 180.f)));
							if (10.f < SF._Tmp_Flow_Altitude && SF._Tmp_Flow_Altitude < 1500.f)
							{
								AF.Flow_Lose_Clocking--;
								if (AF.Flow_Lose_Clocking < 0)
								{
									AF._flag_IsSonarAvalible = true;
									AF.Flow_Lose_Clocking = 0;
								}
							}
							else
							{
								SF._uORB_Flow_Altitude = SF._uORB_Flow_Altitude_Last_Final;
								AF.Flow_Lose_Clocking++;
							}
							SF._uORB_Flow_Altitude_Last_Final = SF._uORB_Flow_Altitude_Final;
							SF._uORB_Flow_Altitude_Final += ((float)SF._uORB_Flow_Altitude - SF._uORB_Flow_Altitude_Final) * 0.2;
							//
							SF._uORB_Flow_ClimbeRate = ((SF._uORB_Flow_Altitude_Final / 10.f) -
														(SF._uORB_Flow_Altitude_Last_Final / 10.f)) /
													   (35000.f / 1000000.f);
							if (AF.Flow_Lose_Clocking > 5)
							{
								AF._flag_IsSonarAvalible = false;
								SF._uORB_Flow_Altitude_Final = 0;
								SF._uORB_Flow_ClimbeRate = 0;
								AF.Flow_Lose_Clocking = 5;
							}
							AF._flag_SonarData_Async = true;
						}
						if (SF._Tmp_Flow___Status == 2 || SF._Tmp_Flow___Status == 3)
						{
							if (SF._uORB_Flow_Quality < 105.f)
								AF._flag_IsFlowAvalible = false;
							else
								AF._flag_IsFlowAvalible = true;

							SF._uORB_Flow_Body_Asix_X = SF._uORB_Gryo_Body_Asix_X * 10.f;
							SF._uORB_Flow_Body_Asix_Y = SF._uORB_Gryo_Body_Asix_Y * 10.f;
							SF._uORB_Flow_Filter_XOutput = ((float)SF._uORB_Flow_XOutput + SF._uORB_Flow_Body_Asix_X) * SF._uORB_Flow_Altitude / 100.f;
							SF._uORB_Flow_Filter_YOutput = ((float)SF._uORB_Flow_YOutput + SF._uORB_Flow_Body_Asix_Y) * SF._uORB_Flow_Altitude / 100.f;
							SF._uORB_Flow_Speed_X = (SF._uORB_Flow_Filter_XOutput / 50.f) / ((float)TF._flag_FlowThreadTimeMax * 3.f / 1000000.f);
							SF._uORB_Flow_Speed_Y = (SF._uORB_Flow_Filter_YOutput / 50.f) / ((float)TF._flag_FlowThreadTimeMax * 3.f / 1000000.f);
							SF._uORB_Flow_XOutput_Total += SF._uORB_Flow_Speed_X * ((float)TF._flag_FlowThreadTimeMax * 3.f / 1000000.f);
							SF._uORB_Flow_YOutput_Total += SF._uORB_Flow_Speed_Y * ((float)TF._flag_FlowThreadTimeMax * 3.f / 1000000.f);

							TF._Tmp_FlowThreadSMooth = 0;
							AF._flag_FlowData_Async = true;
							SF._uORB_Gryo_Body_Asix_X = 0;
							SF._uORB_Gryo_Body_Asix_Y = 0;

							if (!AF._flag_IsFlowAvalible)
							{
								SF._uORB_Flow_Speed_X = 0;
								SF._uORB_Flow_Speed_Y = 0;
								SF._uORB_Flow_XOutput_Total = 0;
								SF._uORB_Flow_YOutput_Total = 0;
							}
						}
						if (TF._Tmp_FlowThreadSMooth > 3)
						{
							TF._flag_FlowErrorTimes++;
							SF._uORB_Gryo_Body_Asix_X = 0;
							SF._uORB_Gryo_Body_Asix_Y = 0;
						}
					}
					TF._Tmp_FlowThreadTimeEnd = GetTimestamp();
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
					TF._Tmp_FlowThreadTimeEnd = GetTimestamp();
				}
			});
		cpu_set_t cpuset3;
		CPU_ZERO(&cpuset3);
		CPU_SET(3, &cpuset3);
		int rc3 = pthread_setaffinity_np(TF.FlowTask.native_handle(), sizeof(cpu_set_t), &cpuset3);
	}
}

void SingleAPMAPI::RPiSingleAPM::ESCUpdateTaskReg()
{
	TF.ESCTask = std::thread(
		[&] {
			TF._flag_ESC_Task_Running = true;
			while (TF._flag_ESC_Task_Running)
			{
				TF._Tmp_ESCThreadTimeStart = GetTimestamp();
				TF._Tmp_ESCThreadTimeNext = TF._Tmp_ESCThreadTimeStart - TF._Tmp_ESCThreadTimeEnd;

				if (AF._flag_ESC_ARMED)
				{
					DF.ESCDevice->ESCUpdate(EF._flag_A1_Pin, EF._Flag_Lock_Throttle);
					DF.ESCDevice->ESCUpdate(EF._flag_A2_Pin, EF._Flag_Lock_Throttle);
					DF.ESCDevice->ESCUpdate(EF._flag_B1_Pin, EF._Flag_Lock_Throttle);
					DF.ESCDevice->ESCUpdate(EF._flag_B2_Pin, EF._Flag_Lock_Throttle);
				}
				if (!AF._flag_ESC_ARMED)
				{
					DF.ESCDevice->ESCUpdate(EF._flag_A1_Pin, EF._uORB_A1_Speed);
					DF.ESCDevice->ESCUpdate(EF._flag_A2_Pin, EF._uORB_A2_Speed);
					DF.ESCDevice->ESCUpdate(EF._flag_B1_Pin, EF._uORB_B1_Speed);
					DF.ESCDevice->ESCUpdate(EF._flag_B2_Pin, EF._uORB_B2_Speed);
				}
				//
				TF._Tmp_ServoThreadClock++;
				if (TF._Tmp_ServoThreadClock > TF._flag_ServoThreadTimes)
				{
					TF._Tmp_ServoThreadClock = 0;
				}

				TF._Tmp_ESCThreadTimeEnd = GetTimestamp();
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
				TF._Tmp_ESCThreadTimeEnd = GetTimestamp();
			}
		});
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	CPU_SET(3, &cpuset);
	int rc = pthread_setaffinity_np(TF.ESCTask.native_handle(), sizeof(cpu_set_t), &cpuset);
}

void SingleAPMAPI::RPiSingleAPM::TaskThreadBlock()
{
	TF._flag_Block_Task_Running = true;
	while (TF._flag_Block_Task_Running)
	{
#ifdef RPiDEBUG
		DebugOutPut();
		TF.DEBUGOuputCleaner++;
		if (TF.DEBUGOuputCleaner > 60)
		{
			system("clear");
			TF.DEBUGOuputCleaner = 0;
		}
#endif
		SaftyCheck();
		if (DF.APMStatus == -2)
		{
			TF._flag_Block_Task_Running = false;
			break;
		}
		usleep(50000);
	}
}

void SingleAPMAPI::RPiSingleAPM::BlackBoxTaskReg()
{
	if (DF._IsBlackBoxEnable)
	{
		TF._Tmp_BBQThreadTimeup = GetTimestamp();
		FileInjectSTR(DF.BlackBoxFile, DF.BlackBoxDevice->FullBlackboxHeader.c_str());

		TF.BlackBoxQTask = std::thread([&] {
			TF._flag_BBQ_Task_Running = true;
			while (TF._flag_BBQ_Task_Running)
			{
				TF._Tmp_BBQThreadTimeStart = GetTimestamp();
				TF._Tmp_BBQThreadTimeNext = TF._Tmp_BBQThreadTimeStart - TF._Tmp_BBQThreadTimeEnd;
				//
				if (!AF._flag_ESC_ARMED)
				{
					TF.BlackBoxQeueue.push(DF.BlackBoxDevice->BlackboxPIPush({
						TF._Tmp_BBQThreadloopIteration,
						GetTimestamp() - TF._Tmp_BBQThreadTimeup,
						(int)PF._uORB_Leveling__Roll,
						(int)PF._uORB_Leveling_Pitch,
						(int)PF._uORB_Leveling___Yaw,
						(-1 * (RF._flag_RC_Mid_PWM_Value - RF._uORB_RC_Channel_PWM[0])), // Convert to Cleanflight output
						(RF._flag_RC_Mid_PWM_Value - RF._uORB_RC_Channel_PWM[1]),
						(RF._flag_RC_Mid_PWM_Value - RF._uORB_RC_Channel_PWM[3]),
						(RF._uORB_RC_Channel_PWM[2]),
						(int)SF._uORB_MPU_Data._uORB_Real__Roll,
						(int)SF._uORB_MPU_Data._uORB_Real_Pitch * -1, // Convert to Cleanflight output
						(int)SF._uORB_MPU_Data._uORB_Real___Yaw,
						(int)SF._uORB_MPU_Data._uORB_Gryo__Roll,
						(int)SF._uORB_MPU_Data._uORB_Gryo_Pitch * -1, // Convert to Cleanflight output
						(int)SF._uORB_MPU_Data._uORB_Gryo___Yaw,
						(int)(SF._uORB_MPU_Data._uORB_MPU9250_ADF_Y * MPU9250_ACCEL_LSB),
						(int)(SF._uORB_MPU_Data._uORB_MPU9250_ADF_X * MPU9250_ACCEL_LSB),
						(int)(SF._uORB_MPU_Data._uORB_MPU9250_ADF_Z * MPU9250_ACCEL_LSB),
						EF._uORB_B2_Speed,
						EF._uORB_A2_Speed,
						EF._uORB_B1_Speed,
						EF._uORB_A1_Speed,
					}));
					TF._Tmp_BBQThreadloopIteration++;
				}
				//
				TF._Tmp_BBQThreadTimeEnd = GetTimestamp();
				TF._Tmp_BBQThreadTimeLoop = TF._Tmp_BBQThreadTimeEnd - TF._Tmp_BBQThreadTimeStart;
				if (TF._Tmp_BBQThreadTimeLoop + TF._Tmp_BBQThreadTimeNext > TF._flag_BBQThreadTimeMax | TF._Tmp_BBQThreadTimeNext < 0)
				{
					usleep(100);
				}
				else
				{
					usleep(TF._flag_BBQThreadTimeMax - TF._Tmp_BBQThreadTimeLoop - TF._Tmp_BBQThreadTimeNext);
				}
				TF._Tmp_BBQThreadTimeEnd = GetTimestamp();
			}
		});

		TF.BlackBoxWTask = std::thread([&] {
			TF._flag_BBW_Task_Running = true;
			while (TF._flag_BBW_Task_Running)
			{
				if (!TF.BlackBoxQeueue.empty())
				{
					FileInjectQueue(DF.BlackBoxFile, TF.BlackBoxQeueue.front());
					TF.BlackBoxQeueue.pop();
				}
				else
				{
					usleep(TF._flag_BBQThreadTimeMax * 2);
				}
			}
		});
	}
}

int SingleAPMAPI::RPiSingleAPM::APMCalibrator(int controller, int action, int input, double *data)
{
	if (controller == ESCCalibration)
	{
		if (action == CaliESCStart)
		{
			sleep(5);
			DF.ESCDevice->ESCUpdate(EF._flag_A1_Pin, EF._Flag_Max__Throttle);
			DF.ESCDevice->ESCUpdate(EF._flag_A2_Pin, EF._Flag_Max__Throttle);
			DF.ESCDevice->ESCUpdate(EF._flag_B1_Pin, EF._Flag_Max__Throttle);
			DF.ESCDevice->ESCUpdate(EF._flag_B2_Pin, EF._Flag_Max__Throttle);
			sleep(15);
			DF.ESCDevice->ESCUpdate(EF._flag_A1_Pin, EF._Flag_Lock_Throttle);
			DF.ESCDevice->ESCUpdate(EF._flag_A2_Pin, EF._Flag_Lock_Throttle);
			DF.ESCDevice->ESCUpdate(EF._flag_B1_Pin, EF._Flag_Lock_Throttle);
			DF.ESCDevice->ESCUpdate(EF._flag_B2_Pin, EF._Flag_Lock_Throttle);
			return 0;
		}
		else if (action == CaliESCUserDefine)
		{
			DF.ESCDevice->ESCUpdate((int)data[0], input);
			return 1;
		}
	}
	else if (controller == ACCELCalibration)
	{
		DF.MPUDevice->MPUAccelCalibration(action, data);
	}
	else if (controller == COMPASSCalibration)
	{
		// DF.CompassDevice->CompassCalibration(true, data);
	}
	return -1;
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

void SingleAPMAPI::RPiSingleAPM::PID_CaculateExtend(float inputDataP, float inputDataI, float inputDataD, float &outputData,
													float &last_I_Data, float &last_D_Data,
													float P_Gain, float I_Gain, float D_Gain, float I_Max)
{
	//P caculate
	outputData = P_Gain * inputDataP;
	//D caculate
	outputData += D_Gain * (inputDataD - last_D_Data);
	last_D_Data = inputDataD;
	//I caculate
	last_I_Data += inputDataI * I_Gain;
	if (last_I_Data > I_Max)
		last_I_Data = I_Max;
	if (last_I_Data < I_Max * -1)
		last_I_Data = I_Max * -1;
	//P_I_D Mix OUTPUT
	outputData += last_I_Data;
}

void SingleAPMAPI::RPiSingleAPM::PID_CaculateHyper(float inputDataP, float inputDataI, float inputDataD, float &outputData,
												   float &last_I_Data, float &last_D_Data,
												   float P_Gain, float I_Gain, float D_Gain, float I_Max)
{
	//P caculate
	outputData = P_Gain * inputDataP;
	//D caculate
	outputData += D_Gain * inputDataD;
	//I caculate
	last_I_Data += inputDataI * I_Gain;
	if (last_I_Data > I_Max)
		last_I_Data = I_Max;
	if (last_I_Data < I_Max * -1)
		last_I_Data = I_Max * -1;
	//P_I_D Mix OUTPUT
	outputData += last_I_Data;
}

void SingleAPMAPI::RPiSingleAPM::ConfigReader(APMSettinngs APMInit)
{
	//==========================================================Device Type=======/
	RF.RC_Type = APMInit.DC.RC_Type;
	SF.MPU9250_Type = APMInit.DC.MPU9250_Type;

	DF.RCDevice = APMInit.DC.__RCDevice;
	DF.GPSDevice = APMInit.DC.__GPSDevice;
	DF.FlowDevice = APMInit.DC.__FlowDevice;
	DF.MPUDeviceSPI = APMInit.DC.__MPUDeviceSPI;

	DF._IsGPSEnable = APMInit.DC._IsGPSEnable;
	DF._IsFlowEnable = APMInit.DC._IsFlowEnable;
	DF._IsRCSafeEnable = APMInit.DC._IsRCSafeEnable;
	DF._IsBAROEnable = APMInit.DC._IsBAROEnable;
	DF._IsBlackBoxEnable = APMInit.DC._IsBlackBoxEnable;

	TF._flag_IMUThreadFreq = APMInit.DC.IMU_Freqeuncy;
	TF._flag_IMUThreadTimeMax = 1.f / (float)TF._flag_IMUThreadFreq * 1000000.f - LINUX_SYSTEM_SLEEP_DELAY;
	TF._flag_RXTThreadFreq = APMInit.DC.RXT_Freqeuncy;
	TF._flag_RXTThreadTimeMax = 1.f / (float)TF._flag_RXTThreadFreq * 1000000.f - LINUX_SYSTEM_SLEEP_DELAY;
	TF._flag_ESCThreadFreq = APMInit.DC.ESC_Freqeuncy;
	TF._flag_ESCThreadTimeMax = 1.f / (float)TF._flag_ESCThreadFreq * 1000000.f - LINUX_SYSTEM_SLEEP_DELAY;
	TF._flag_ServoThreadTimes = TF._flag_ESCThreadFreq / TF._flag_ServoThreadFreq;
	TF._flag_BBQThreadFreq = APMInit.DC.BBC_Freqeuncy;
	TF._flag_BBQThreadTimeMax = 1.f / (float)TF._flag_BBQThreadFreq * 1000000.f - LINUX_SYSTEM_SLEEP_DELAY;
	//==========================================================Controller config==/

	RF._flag_RC_Min_PWM_Value = APMInit.RC._flag_RC_Min_PWM_Value;
	RF._flag_RC_Mid_PWM_Value = APMInit.RC._flag_RC_Mid_PWM_Value;
	RF._flag_RC_Max_PWM_Value = APMInit.RC._flag_RC_Max_PWM_Value;

	RF._flag_RC_ARM_PWM_Value = APMInit.RC._flag_RC_ARM_PWM_Value;
	RF._flag_RC_ARM_PWM_Channel = APMInit.RC._flag_RC_ARM_PWM_Channel;
	RF._flag_RC_AP_RateHold_PWM_Value = APMInit.RC._flag_RC_AP_RateHold_PWM_Value;
	RF._flag_RC_AP_RateHold_PWM_Channel = APMInit.RC._flag_RC_AP_RateHold_PWM_Channel;
	RF._flag_RC_AP_AutoStable_PWM_Value = APMInit.RC._flag_RC_AP_AutoStable_PWM_Value;
	RF._flag_RC_AP_AutoStable_PWM_Channel = APMInit.RC._flag_RC_AP_AutoStable_PWM_Channel;
	RF._flag_RC_AP_AltHold_PWM_Value = APMInit.RC._flag_RC_AP_AltHold_PWM_Value;
	RF._flag_RC_AP_AltHold_PWM_Channel = APMInit.RC._flag_RC_AP_AltHold_PWM_Channel;
	RF._flag_RC_AP_PositionHold_PWM_Value = APMInit.RC._flag_RC_AP_PositionHold_PWM_Value;
	RF._flag_RC_AP_PositionHold_PWM_Channel = APMInit.RC._flag_RC_AP_PositionHold_PWM_Channel;
	RF._flag_RC_AP_SpeedHold_PWM_Value = APMInit.RC._flag_RC_AP_SpeedHold_PWM_Value;
	RF._flag_RC_AP_SpeedHold_PWM_Channel = APMInit.RC._flag_RC_AP_SpeedHold_PWM_Channel;
	RF._flag_RC_AP_UserAuto_PWM_Value = APMInit.RC._flag_RC_AP_UserAuto_PWM_Value;
	RF._flag_RC_AP_UserAuto_PWM_Channel = APMInit.RC._flag_RC_AP_UserAuto_PWM_Channel;

	RF._flag_RCIsReserv__Roll = APMInit.RC._flag_RCIsReserv__Roll;
	RF._flag_RCIsReserv_Pitch = APMInit.RC._flag_RCIsReserv_Pitch;
	RF._flag_RCIsReserv___Yaw = APMInit.RC._flag_RCIsReserv___Yaw;
	//==========================================================ESC config=========/
	EF._flag_A1_Pin = APMInit.OC._flag_A1_Pin;
	EF._flag_A2_Pin = APMInit.OC._flag_A2_Pin;
	EF._flag_B1_Pin = APMInit.OC._flag_B1_Pin;
	EF._flag_B2_Pin = APMInit.OC._flag_B2_Pin;
	EF._flag_YAWOut_Reverse = APMInit.OC._flag_YAWOut_Reverse;
	EF._flag_ESC_Lazy_Per = APMInit.OC._flag_ESC_Lazy_Per;
	EF._Flag_Lazy_Throttle = (EF._Flag_Lock_Throttle + ESCRANGE * EF._flag_ESC_Lazy_Per);
	EF.ESCPLFrequency = APMInit.OC.ESCPLFrequency;
	EF.ESCControllerType = (GeneratorType)APMInit.OC.ESCControllerType;
	//==================================================================PID config==/
	PF._flag_PID_P__Roll_Gain = APMInit.PC._flag_PID_P__Roll_Gain;
	PF._flag_PID_P_Pitch_Gain = APMInit.PC._flag_PID_P_Pitch_Gain;
	PF._flag_PID_P___Yaw_Gain = APMInit.PC._flag_PID_P___Yaw_Gain;
	PF._flag_PID_P_Alt_Gain = APMInit.PC._flag_PID_P_Alt_Gain;
	PF._flag_PID_P_PosX_Gain = APMInit.PC._flag_PID_P_PosX_Gain;
	PF._flag_PID_P_PosY_Gain = APMInit.PC._flag_PID_P_PosY_Gain;
	PF._flag_PID_P_SpeedZ_Gain = APMInit.PC._flag_PID_P_SpeedZ_Gain;
	PF._flag_PID_P_SpeedX_Gain = APMInit.PC._flag_PID_P_SpeedX_Gain;
	PF._flag_PID_P_SpeedY_Gain = APMInit.PC._flag_PID_P_SpeedY_Gain;

	PF._flag_PID_I__Roll_Gain = APMInit.PC._flag_PID_I__Roll_Gain;
	PF._flag_PID_I_Pitch_Gain = APMInit.PC._flag_PID_I_Pitch_Gain;
	PF._flag_PID_I___Yaw_Gain = APMInit.PC._flag_PID_I___Yaw_Gain;
	PF._flag_PID_I_Alt_Gain = APMInit.PC._flag_PID_I_Alt_Gain;
	PF._flag_PID_I_PosX_Gain = APMInit.PC._flag_PID_I_PosX_Gain;
	PF._flag_PID_I_PosY_Gain = APMInit.PC._flag_PID_I_PosY_Gain;
	PF._flag_PID_I_SpeedZ_Gain = APMInit.PC._flag_PID_I_SpeedZ_Gain;
	PF._flag_PID_I_SpeedX_Gain = APMInit.PC._flag_PID_I_SpeedX_Gain;
	PF._flag_PID_I_SpeedY_Gain = APMInit.PC._flag_PID_I_SpeedY_Gain;
	PF._flag_PID_I__Roll_Max__Value = APMInit.PC._flag_PID_I__Roll_Max__Value;
	PF._flag_PID_I_Pitch_Max__Value = APMInit.PC._flag_PID_I_Pitch_Max__Value;
	PF._flag_PID_I___Yaw_Max__Value = APMInit.PC._flag_PID_I___Yaw_Max__Value;

	PF._flag_PID_D__Roll_Gain = APMInit.PC._flag_PID_D__Roll_Gain;
	PF._flag_PID_D_Pitch_Gain = APMInit.PC._flag_PID_D_Pitch_Gain;
	PF._flag_PID_D___Yaw_Gain = APMInit.PC._flag_PID_D___Yaw_Gain;
	PF._flag_PID_D_SpeedZ_Gain = APMInit.PC._flag_PID_D_SpeedZ_Gain;
	PF._flag_PID_D_SpeedX_Gain = APMInit.PC._flag_PID_D_SpeedX_Gain;
	PF._flag_PID_D_SpeedY_Gain = APMInit.PC._flag_PID_D_SpeedY_Gain;

	PF._flag_PID_Hover_Throttle = APMInit.PC._flag_PID_Hover_Throttle;
	PF._flag_PID_Level_Max = APMInit.PC._flag_PID_Level_Max;
	PF._flag_PID_Rate_Limit = APMInit.PC._flag_PID_Rate_Limit;
	PF._flag_PID_Alt_Level_Max = APMInit.PC._flag_PID_Alt_Level_Max;
	PF._flag_PID_Pos_Level_Max = APMInit.PC._flag_PID_Pos_Level_Max;

	PF._flag_PID_Takeoff_Altitude = APMInit.PC._flag_PID_Takeoff_Altitude;
	PF._flag_PID_Alt_Speed_Max = APMInit.PC._flag_PID_Alt_Speed_Max;
	PF._flag_PID_Alt_Accel_Max = APMInit.PC._flag_PID_Alt_Accel_Max;
	PF._flag_PID_PosMan_Speed_Max = APMInit.PC._flag_PID_PosMan_Speed_Max;
	PF._flag_PID_Pos_Speed_Max = APMInit.PC._flag_PID_Pos_Speed_Max;
	PF._flag_PID_Pos_Accel_Max = APMInit.PC._flag_PID_Pos_Accel_Max;

	PF._flag_PID_AngleRate__Roll_Gain = APMInit.PC._flag_PID_AngleRate__Roll_Gain;
	PF._flag_PID_AngleRate_Pitch_Gain = APMInit.PC._flag_PID_AngleRate_Pitch_Gain;
	PF._flag_PID_AngleRate___Yaw_Gain = APMInit.PC._flag_PID_AngleRate___Yaw_Gain;

	PF._flag_PID_RCRate__Roll_Gain = APMInit.PC._flag_PID_RCRate__Roll_Gain;
	PF._flag_PID_RCRate_Pitch_Gain = APMInit.PC._flag_PID_RCRate_Pitch_Gain;
	PF._flag_PID_RCRate___Yaw_Gain = APMInit.PC._flag_PID_RCRate___Yaw_Gain;

	PF._flag_PID_RCAngle__Roll_Gain = APMInit.PC._flag_PID_RCAngle__Roll_Gain;
	PF._flag_PID_RCAngle_Pitch_Gain = APMInit.PC._flag_PID_RCAngle_Pitch_Gain;
	PF._flag_PID_RCAngle___Yaw_Gain = APMInit.PC._flag_PID_RCAngle___Yaw_Gain;

	PF._flag_PID_TPA_Trust = APMInit.PC._flag_PID_TPA_Trust;
	PF._flag_PID_TPA_BreakPoint = APMInit.PC._flag_PID_TPA_BreakPoint;
	//==============================================================Sensors config==/
	SF._flag_MPU_Accel_Cali[MPUAccelCaliX] = APMInit.SC._flag_MPU9250_A_X_Cali;
	SF._flag_MPU_Accel_Cali[MPUAccelCaliY] = APMInit.SC._flag_MPU9250_A_Y_Cali;
	SF._flag_MPU_Accel_Cali[MPUAccelCaliZ] = APMInit.SC._flag_MPU9250_A_Z_Cali;
	SF._flag_MPU_Accel_Cali[MPUAccelScalX] = APMInit.SC._flag_MPU9250_A_X_Scal;
	SF._flag_MPU_Accel_Cali[MPUAccelScalY] = APMInit.SC._flag_MPU9250_A_Y_Scal;
	SF._flag_MPU_Accel_Cali[MPUAccelScalZ] = APMInit.SC._flag_MPU9250_A_Z_Scal;
	SF._flag_MPU_Accel_Cali[MPUAccelTRIMPitch] = APMInit.SC._flag_Accel_Pitch_Cali;
	SF._flag_MPU_Accel_Cali[MPUAccelTRIM_Roll] = APMInit.SC._flag_Accel__Roll_Cali;

	SF._flag_MPU_MAG_Cali[CompassYScaler] = APMInit.SC._flag_COMPASS_Y_Scaler;
	SF._flag_MPU_MAG_Cali[CompassZScaler] = APMInit.SC._flag_COMPASS_Z_Scaler;
	SF._flag_MPU_MAG_Cali[CompassXOffset] = APMInit.SC._flag_COMPASS_X_Offset;
	SF._flag_MPU_MAG_Cali[CompassYOffset] = APMInit.SC._flag_COMPASS_Y_Offset;
	SF._flag_MPU_MAG_Cali[CompassZOffset] = APMInit.SC._flag_COMPASS_Z_Offset;
	//==============================================================Filter config==/
	SF._flag_Filter_Gryo_Type = APMInit.FC._flag_Filter_Gryo_Type;
	SF._flag_Filter_GryoST2_Type = APMInit.FC._flag_Filter_GryoST2_Type;
	SF._flag_Filter_GYaw_CutOff = APMInit.FC._flag_Filter_GYaw_CutOff;
	SF._flag_Filter_Gryo_CutOff = APMInit.FC._flag_Filter_Gryo_CutOff;
	SF._flag_Filter_GryoST2_CutOff = APMInit.FC._flag_Filter_GryoST2_CutOff;
	SF._flag_Filter_Gryo_NotchFreq = APMInit.FC._flag_Filter_Gryo_NotchFreq;
	SF._flag_Filter_Gryo_NotchCutOff = APMInit.FC._flag_Filter_Gryo_NotchCutOff;
	SF._flag_Filter_Gryo_DynamicNotchRange = APMInit.FC._flag_Filter_Gryo_DynamicNotchRange;
	SF._flag_Filter_Gryo_DynamicNotchEnable = APMInit.FC._flag_Filter_Gryo_DynamicNotchEnable;
	SF._flag_Filter_Gryo_DynamicNotchMinFreq = APMInit.FC._flag_Filter_Gryo_DynamicNotchMinFreq;
	SF._flag_Filter_Accel_Type = APMInit.FC._flag_Filter_Accel_Type;
	SF._flag_Filter_Accel_CutOff = APMInit.FC._flag_Filter_Accel_CutOff;
	SF._flag_Filter_AngleMix_Alpha = APMInit.FC._flag_Filter_AngleMix_Alpha;

	PF._flag_Baro_Config_Beta = APMInit.FC._flag_Baro_Trust_Beta;
	PF._flag_Accel_Config_Beta = APMInit.FC._flag_Accel_Trust_Beta;
	PF._flag_Sonar_Config_Beta = APMInit.FC._flag_Sonar_Trust_Beta;
	PF._uORB_GPSAlt_Dynamic_Beta = APMInit.FC._flag_GPSAlt_Trust_Beta;
	PF._uORB_AccelBias_Beta = APMInit.FC._flag_AccelBias_Trust_Beta;

	RF._flag_Filter_RC_CutOff = APMInit.FC._flag_Filter_RC_CutOff;
	PF._flag_Filter_AngleRate_CutOff = APMInit.FC._flag_Filter_AngleRate_CutOff;
	SF._flag_Filter_AngleMix_Alpha = APMInit.FC._flag_Filter_AngleMix_Alpha;

	PF._flag_Filter_PID_I_CutOff = APMInit.FC._flag_Filter_PID_I_CutOff;
	PF._flag_Filter_PID_D_ST1_CutOff = APMInit.FC._flag_Filter_PID_D_ST1_CutOff;
	PF._flag_Filter_PID_D_ST2_CutOff = APMInit.FC._flag_Filter_PID_D_ST2_CutOff;

	PF._flag_GPS_Dynamic_Beta = APMInit.FC._flag_GPS_Config_Beta;
	PF._flag_Flow_Dynamic_Beta = APMInit.FC._flag_Flow_Config_Beta;
	PF._flag_Braking_Speed_Gain = APMInit.FC._flag_Braking_Speed_Gain;
	PF._flag_Braking_AccelMax_Gain = APMInit.FC._flag_Braking_AccelMax_Gain;
}

//=-----------------------------------------------------------------------------------------==//

void SingleAPMAPI::RPiSingleAPM::AttitudeUpdate()
{
	TF._Tmp_IMUAttThreadDT = GetTimestamp() - TF._Tmp_IMUAttThreadLast;
	//PID Checking
	{
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
		//NotTakeoff Detect
		if (AF._flag_IsNotTakeOff_Lock && RF._uORB_RC_Out_Throttle < (RF._flag_RC_Min_PWM_Value + 100.f))
		{
			AF._flag_IsNotTakeOff = true;
			AF._flag_IsNotTakeOff_Lock = false;
		}
		else if (RF._uORB_RC_Out_Throttle > (RF._flag_RC_Min_PWM_Value + 100.f))
		{
			AF._flag_IsNotTakeOff = false;
		}
		if (AF._flag_IsNotTakeOff)
		{
			PF._uORB_PID_I_Last_Value_Pitch = 0;
			PF._uORB_PID_I_Last_Value__Roll = 0;
			PF._uORB_PID_I_Last_Value___Yaw = 0;
		}
	}
	//
	if (AF.AutoPilotMode == APModeINFO::AltHold ||
		AF.AutoPilotMode == APModeINFO::AutoStable ||
		AF.AutoPilotMode == APModeINFO::SpeedHold ||
		AF.AutoPilotMode == APModeINFO::PositionHold ||
		AF.AutoPilotMode == APModeINFO::UserAuto ||
		AF.AutoPilotMode == APModeINFO::RateHold)
	{
		//Leveling PID MIX
		{
			//IMU SaftyChecking---------------------------------------------------------//
			if (SF._uORB_MPU_Data._uORB_Real_Pitch > 70.0 || SF._uORB_MPU_Data._uORB_Real_Pitch < -70.0 ||
				SF._uORB_MPU_Data._uORB_Real__Roll > 70.0 || SF._uORB_MPU_Data._uORB_Real__Roll < -70.0)
			{
				if (AF.AutoPilotMode == APModeINFO::AltHold ||
					AF.AutoPilotMode == APModeINFO::AutoStable ||
					AF.AutoPilotMode == APModeINFO::SpeedHold ||
					AF.AutoPilotMode == APModeINFO::PositionHold ||
					AF.AutoPilotMode == APModeINFO::UserAuto)
				{
					AF.AngleLimit_Out_Clocking += 1.f / (float)TF._flag_IMUThreadTimeMax * 1000000.f;
					if (AF.AngleLimit_Out_Clocking > AngleLimitTime)
					{
						AF._flag_AnagleOutOfLimit = true;
						AF.AngleLimit_Out_Clocking = 0;
					}
				}
			}
			else
			{
				AF._flag_AnagleOutOfLimit = false;
				AF.AngleLimit_Out_Clocking = 0;
			}
			//--------------------------------------------------------------------------//
			if (PF._flag_Filter_AngleRate_CutOff != 0)
				PF._uORB_PID_AngleRate__Roll = pt1FilterApply4(&DF.AngleRateLPF[0], SF._uORB_MPU_Data._uORB_Real__Roll,
															   PF._flag_Filter_AngleRate_CutOff, ((float)TF._Tmp_IMUAttThreadDT / 1000000.f));
			else
				PF._uORB_PID_AngleRate__Roll = SF._uORB_MPU_Data._uORB_Real__Roll;
			//
			if (PF._flag_Filter_AngleRate_CutOff != 0)
				PF._uORB_PID_AngleRate_Pitch = pt1FilterApply4(&DF.AngleRateLPF[1], SF._uORB_MPU_Data._uORB_Real_Pitch,
															   PF._flag_Filter_AngleRate_CutOff, ((float)TF._Tmp_IMUAttThreadDT / 1000000.f));
			else
				PF._uORB_PID_AngleRate_Pitch = SF._uORB_MPU_Data._uORB_Real_Pitch;
			//
			if (SF._flag_Filter_GYaw_CutOff != 0)
				PF._uORB_PID_GYaw_Output = pt1FilterApply4(&DF.AngleRateLPF[2], SF._uORB_MPU_Data._uORB_Gryo___Yaw,
														   SF._flag_Filter_GYaw_CutOff, ((float)TF._Tmp_IMUAttThreadDT / 1000000.f));
			else
				PF._uORB_PID_GYaw_Output = SF._uORB_MPU_Data._uORB_Gryo___Yaw;
			//--------------------------------------------------------------------------//
			PF._uORB_PID__Roll_Input = 0;
			PF._uORB_PID_Pitch_Input = 0;
			//--------------------------------------------------------------------------//
			if ((AF.AutoPilotMode == APModeINFO::SpeedHold && AF._flag_IsFlowAvalible) || (AF.AutoPilotMode == APModeINFO::UserAuto && AF._flag_IsFlowAvalible))
			{
				PF._uORB_PID__Roll_Input += PF._uORB_PID_PosX_Output;
				PF._uORB_PID_Pitch_Input += PF._uORB_PID_PosY_Output;
			}
			else if (AF.AutoPilotMode == APModeINFO::PositionHold && AF._flag_IsFlowAvalible)
			{
				if (RF._uORB_RC_Out__Roll != 0 && !AF._flag_IsBrakingXSet)
					PF._uORB_PID__Roll_Input -= RF._uORB_RC_Out__Roll * PF._flag_PID_RCAngle__Roll_Gain;
				else
					PF._uORB_PID__Roll_Input += PF._uORB_PID_PosX_Output;
				if (RF._uORB_RC_Out_Pitch != 0 && !AF._flag_IsBrakingYSet)
					PF._uORB_PID_Pitch_Input -= RF._uORB_RC_Out_Pitch * PF._flag_PID_RCAngle_Pitch_Gain;
				else
					PF._uORB_PID_Pitch_Input += PF._uORB_PID_PosY_Output;
			}
			else if (AF.AutoPilotMode == APModeINFO::AutoStable ||
					 AF.AutoPilotMode == APModeINFO::AltHold)
			{
				PF._uORB_PID__Roll_Input -= RF._uORB_RC_Out__Roll * PF._flag_PID_RCAngle__Roll_Gain;
				PF._uORB_PID_Pitch_Input -= RF._uORB_RC_Out_Pitch * PF._flag_PID_RCAngle_Pitch_Gain;
			}
			else if (AF.AutoPilotMode == APModeINFO::RateHold)
			{
				PF._uORB_PID__Roll_Input -= RF._uORB_RC_Out__Roll * PF._flag_PID_RCRate__Roll_Gain;
				PF._uORB_PID_Pitch_Input -= RF._uORB_RC_Out_Pitch * PF._flag_PID_RCRate_Pitch_Gain;
			}
			//--------------------------------------------------------------------------//
			if (AF.AutoPilotMode == APModeINFO::AltHold ||
				AF.AutoPilotMode == APModeINFO::AutoStable ||
				AF.AutoPilotMode == APModeINFO::SpeedHold ||
				AF.AutoPilotMode == APModeINFO::PositionHold ||
				AF.AutoPilotMode == APModeINFO::UserAuto)
			{
				float AngleEXPO__Roll = PF._uORB_PID_AngleRate__Roll * PF._flag_PID_AngleRate__Roll_Gain;
				float AngleEXPO_Pitch = PF._uORB_PID_AngleRate_Pitch * PF._flag_PID_AngleRate_Pitch_Gain;
				PF._uORB_PID__Roll_Input += AngleEXPO__Roll;
				PF._uORB_PID_Pitch_Input += AngleEXPO_Pitch;
			}
			else if (AF.AutoPilotMode == APModeINFO::RateHold)
			{
				PF._uORB_PID__Roll_Input += 0;
				PF._uORB_PID_Pitch_Input += 0;
			}
			//--------------------------------------------------------------------------//
			SF._uORB_MPU_Data._uORB_Gryo_Pitch = SF._uORB_MPU_Data._uORB_Gryo_Pitch > PF._flag_PID_Rate_Limit ? PF._flag_PID_Rate_Limit : SF._uORB_MPU_Data._uORB_Gryo_Pitch;
			SF._uORB_MPU_Data._uORB_Gryo_Pitch = SF._uORB_MPU_Data._uORB_Gryo_Pitch < -1 * PF._flag_PID_Rate_Limit ? -1 * PF._flag_PID_Rate_Limit : SF._uORB_MPU_Data._uORB_Gryo_Pitch;
			SF._uORB_MPU_Data._uORB_Gryo__Roll = SF._uORB_MPU_Data._uORB_Gryo__Roll > PF._flag_PID_Rate_Limit ? PF._flag_PID_Rate_Limit : SF._uORB_MPU_Data._uORB_Gryo__Roll;
			SF._uORB_MPU_Data._uORB_Gryo__Roll = SF._uORB_MPU_Data._uORB_Gryo__Roll < -1 * PF._flag_PID_Rate_Limit ? -1 * PF._flag_PID_Rate_Limit : SF._uORB_MPU_Data._uORB_Gryo__Roll;
			PF._uORB_PID__Roll_Input += SF._uORB_MPU_Data._uORB_Gryo__Roll;
			PF._uORB_PID_Pitch_Input += SF._uORB_MPU_Data._uORB_Gryo_Pitch;
			//--------------------------------------------------------------------------//
			//TPA caculate
			if (EF._uORB_Total_Throttle > PF._flag_PID_TPA_BreakPoint)
			{
				PF._uORB_PID_TPA_Beta = 1.f - ((1.f - PF._flag_PID_TPA_Trust) *
											   ((EF._uORB_Total_Throttle - PF._flag_PID_TPA_BreakPoint) /
												((RF._flag_RC_Max_PWM_Value - 200.f) - PF._flag_PID_TPA_BreakPoint)));
			}
			else
				PF._uORB_PID_TPA_Beta = 1.f;
			//--------------------------------------------------------------------------//
			//Roll PID Mix
			float ROLLDInput = SF._uORB_MPU_Data._uORB_Gryo__Roll - PF._uORB_PID_D_Last_Value__Roll;
			PF._uORB_PID_D_Last_Value__Roll = SF._uORB_MPU_Data._uORB_Gryo__Roll;
			float ROLLITERM = PF._uORB_PID__Roll_Input;
			float ROLLDTERM = ROLLDInput;
			if (PF._flag_Filter_PID_I_CutOff)
				ROLLITERM = pt1FilterApply4(&DF.ItermFilterRoll, PF._uORB_PID__Roll_Input, PF._flag_Filter_PID_I_CutOff, ((float)TF._Tmp_IMUAttThreadDT / 1000000.f));
			if (PF._flag_Filter_PID_D_ST1_CutOff)
				ROLLDTERM = pt1FilterApply4(&DF.DtermFilterRoll, ROLLDInput, PF._flag_Filter_PID_D_ST1_CutOff, ((float)TF._Tmp_IMUAttThreadDT / 1000000.f));
			if (PF._flag_Filter_PID_D_ST2_CutOff)
				ROLLDTERM = pt1FilterApply4(&DF.DtermFilterRollST2, ROLLDTERM, PF._flag_Filter_PID_D_ST2_CutOff, ((float)TF._Tmp_IMUAttThreadDT / 1000000.f));
			PID_CaculateHyper((PF._uORB_PID__Roll_Input),
							  (ROLLITERM * (TF._Tmp_IMUAttThreadDT / PID_DT_DEFAULT)),
							  (ROLLDTERM / (TF._Tmp_IMUAttThreadDT / PID_DT_DEFAULT)),
							  PF._uORB_Leveling__Roll, PF._uORB_PID_I_Last_Value__Roll, PF._uORB_PID_D_Last_Value__Roll,
							  (PF._flag_PID_P__Roll_Gain * PF._uORB_PID_TPA_Beta),
							  (PF._flag_PID_I__Roll_Gain * PF._uORB_PID_I_Dynamic_Gain),
							  (PF._flag_PID_D__Roll_Gain * PF._uORB_PID_TPA_Beta),
							  PF._flag_PID_I__Roll_Max__Value);
			if (PF._uORB_Leveling__Roll > PF._flag_PID_Level_Max)
				PF._uORB_Leveling__Roll = PF._flag_PID_Level_Max;
			if (PF._uORB_Leveling__Roll < PF._flag_PID_Level_Max * -1)
				PF._uORB_Leveling__Roll = PF._flag_PID_Level_Max * -1;

			//Pitch PID Mix
			float PITCHDInput = SF._uORB_MPU_Data._uORB_Gryo_Pitch - PF._uORB_PID_D_Last_Value_Pitch;
			PF._uORB_PID_D_Last_Value_Pitch = SF._uORB_MPU_Data._uORB_Gryo_Pitch;
			float PITCHITERM = PF._uORB_PID_Pitch_Input;
			float PITCHDTERM = PITCHDInput;
			if (PF._flag_Filter_PID_I_CutOff)
				PITCHITERM = pt1FilterApply4(&DF.ItermFilterPitch, PF._uORB_PID_Pitch_Input, PF._flag_Filter_PID_I_CutOff, ((float)TF._Tmp_IMUAttThreadDT / 1000000.f));
			if (PF._flag_Filter_PID_D_ST1_CutOff)
				PITCHDTERM = pt1FilterApply4(&DF.DtermFilterPitch, PITCHDInput, PF._flag_Filter_PID_D_ST1_CutOff, ((float)TF._Tmp_IMUAttThreadDT / 1000000.f));
			if (PF._flag_Filter_PID_D_ST2_CutOff)
				PITCHDTERM = pt1FilterApply4(&DF.DtermFilterPitchST2, PITCHDTERM, PF._flag_Filter_PID_D_ST2_CutOff, ((float)TF._Tmp_IMUAttThreadDT / 1000000.f));
			PID_CaculateHyper((PF._uORB_PID_Pitch_Input),
							  (PITCHITERM * (TF._Tmp_IMUAttThreadDT / PID_DT_DEFAULT)),
							  (PITCHDTERM / (TF._Tmp_IMUAttThreadDT / PID_DT_DEFAULT)),
							  PF._uORB_Leveling_Pitch, PF._uORB_PID_I_Last_Value_Pitch, PF._uORB_PID_D_Last_Value_Pitch,
							  (PF._flag_PID_P_Pitch_Gain * PF._uORB_PID_TPA_Beta),
							  (PF._flag_PID_I_Pitch_Gain * PF._uORB_PID_I_Dynamic_Gain),
							  (PF._flag_PID_D_Pitch_Gain * PF._uORB_PID_TPA_Beta),
							  PF._flag_PID_I_Pitch_Max__Value);
			if (PF._uORB_Leveling_Pitch > PF._flag_PID_Level_Max)
				PF._uORB_Leveling_Pitch = PF._flag_PID_Level_Max;
			if (PF._uORB_Leveling_Pitch < PF._flag_PID_Level_Max * -1)
				PF._uORB_Leveling_Pitch = PF._flag_PID_Level_Max * -1;

			//Yaw PID Mix
			PID_CaculateExtend((((PF._uORB_PID_GYaw_Output + RF._uORB_RC_Out___Yaw) / 15.f) * PF._flag_PID_AngleRate___Yaw_Gain),
							   ((((PF._uORB_PID_GYaw_Output + RF._uORB_RC_Out___Yaw) / 15.f) * PF._flag_PID_AngleRate___Yaw_Gain) * (TF._Tmp_IMUAttThreadDT / PID_DT_DEFAULT)),
							   ((((PF._uORB_PID_GYaw_Output) / 15.f) * PF._flag_PID_AngleRate___Yaw_Gain) / (TF._Tmp_IMUAttThreadDT / PID_DT_DEFAULT)),
							   PF._uORB_Leveling___Yaw, PF._uORB_PID_I_Last_Value___Yaw, PF._uORB_PID_D_Last_Value___Yaw,
							   PF._flag_PID_P___Yaw_Gain, (PF._flag_PID_I___Yaw_Gain * PF._uORB_PID_I_Dynamic_Gain), PF._flag_PID_D___Yaw_Gain, PF._flag_PID_I___Yaw_Max__Value);

			if (PF._uORB_Leveling___Yaw > PF._flag_PID_Level_Max)
				PF._uORB_Leveling___Yaw = PF._flag_PID_Level_Max;
			if (PF._uORB_Leveling___Yaw < PF._flag_PID_Level_Max * -1)
				PF._uORB_Leveling___Yaw = PF._flag_PID_Level_Max * -1;
			PF._uORB_Leveling___Yaw *= EF._flag_YAWOut_Reverse;
		}
	};
	//reset output before caculate
	{
		EF._Tmp_B1_Speed = 0;
		EF._Tmp_A1_Speed = 0;
		EF._Tmp_A2_Speed = 0;
		EF._Tmp_B2_Speed = 0;
		EF._uORB_ESC_RPY_Max = 0;
		EF._uORB_ESC_RPY_Min = 0;
		//ESC Caculate
		if (AF.AutoPilotMode == APModeINFO::AltHold ||
			AF.AutoPilotMode == APModeINFO::PositionHold ||
			AF.AutoPilotMode == APModeINFO::SpeedHold ||
			AF.AutoPilotMode == APModeINFO::UserAuto)
			EF._uORB_Total_Throttle = PF._flag_PID_Hover_Throttle + PF._uORB_PID_Alt_Throttle;
		else
			EF._uORB_Total_Throttle = RF._uORB_RC_Out_Throttle;

		if (EF._uORB_Total_Throttle > RF._flag_RC_Max_PWM_Value - 200.f)
			EF._uORB_Total_Throttle = RF._flag_RC_Max_PWM_Value - 200.f;

		EF._Tmp_B1_Speed = -PF._uORB_Leveling__Roll + PF._uORB_Leveling_Pitch + PF._uORB_Leveling___Yaw;
		EF._Tmp_A1_Speed = -PF._uORB_Leveling__Roll - PF._uORB_Leveling_Pitch - PF._uORB_Leveling___Yaw;
		EF._Tmp_A2_Speed = +PF._uORB_Leveling__Roll - PF._uORB_Leveling_Pitch + PF._uORB_Leveling___Yaw;
		EF._Tmp_B2_Speed = +PF._uORB_Leveling__Roll + PF._uORB_Leveling_Pitch - PF._uORB_Leveling___Yaw;

		EF._uORB_ESC_RPY_Max = EF._Tmp_B1_Speed > EF._uORB_ESC_RPY_Max ? EF._Tmp_B1_Speed : EF._uORB_ESC_RPY_Max;
		EF._uORB_ESC_RPY_Max = EF._Tmp_A1_Speed > EF._uORB_ESC_RPY_Max ? EF._Tmp_A1_Speed : EF._uORB_ESC_RPY_Max;
		EF._uORB_ESC_RPY_Max = EF._Tmp_A2_Speed > EF._uORB_ESC_RPY_Max ? EF._Tmp_A2_Speed : EF._uORB_ESC_RPY_Max;
		EF._uORB_ESC_RPY_Max = EF._Tmp_B2_Speed > EF._uORB_ESC_RPY_Max ? EF._Tmp_B2_Speed : EF._uORB_ESC_RPY_Max;

		EF._uORB_ESC_RPY_Min = EF._Tmp_B1_Speed < EF._uORB_ESC_RPY_Min ? EF._Tmp_B1_Speed : EF._uORB_ESC_RPY_Min;
		EF._uORB_ESC_RPY_Min = EF._Tmp_A1_Speed < EF._uORB_ESC_RPY_Min ? EF._Tmp_A1_Speed : EF._uORB_ESC_RPY_Min;
		EF._uORB_ESC_RPY_Min = EF._Tmp_A2_Speed < EF._uORB_ESC_RPY_Min ? EF._Tmp_A2_Speed : EF._uORB_ESC_RPY_Min;
		EF._uORB_ESC_RPY_Min = EF._Tmp_B2_Speed < EF._uORB_ESC_RPY_Min ? EF._Tmp_B2_Speed : EF._uORB_ESC_RPY_Min;

		EF._uORB_ESC_RPY_Range = EF._uORB_ESC_RPY_Max - EF._uORB_ESC_RPY_Min;
		EF._uORB_ESC_MIX_Range = (float)EF._uORB_ESC_RPY_Range / (float)1000.f;
		PF._uORB_PID_I_Dynamic_Gain = (1.0f - (EF._uORB_ESC_MIX_Range >= 1.f ? 1.f : EF._uORB_ESC_MIX_Range)) / 1.0f;

		EF._uORB_Dynamic_ThrottleMin = 1000.f;
		EF._uORB_Dynamic_ThrottleMax = 2000.f;
		if (EF._uORB_ESC_MIX_Range >= 1.0f)
		{
			EF._Tmp_B1_Speed /= EF._uORB_ESC_MIX_Range;
			EF._Tmp_A1_Speed /= EF._uORB_ESC_MIX_Range;
			EF._Tmp_A2_Speed /= EF._uORB_ESC_MIX_Range;
			EF._Tmp_B2_Speed /= EF._uORB_ESC_MIX_Range;
			EF._uORB_Dynamic_ThrottleMin = EF._uORB_Dynamic_ThrottleMin + (1000.f / 2) - (1000.f * 0.33 / 2.f);
			EF._uORB_Dynamic_ThrottleMax = EF._uORB_Dynamic_ThrottleMax + (1000.f / 2) + (1000.f * 0.33 / 2.f);
		}
		else
		{
			EF._uORB_Dynamic_ThrottleMin =
				(EF._uORB_Dynamic_ThrottleMin + (EF._uORB_ESC_RPY_Range / 2)) < EF._uORB_Dynamic_ThrottleMin + (1000.f / 2.f) - (1000.f * 0.33 / 2.f) ? (EF._uORB_Dynamic_ThrottleMin + (EF._uORB_ESC_RPY_Range / 2)) : EF._uORB_Dynamic_ThrottleMin + (1000.f / 2.f) - (1000.f * 0.33 / 2.f);
			EF._uORB_Dynamic_ThrottleMax =
				(EF._uORB_Dynamic_ThrottleMax - (EF._uORB_ESC_RPY_Range / 2)) > EF._uORB_Dynamic_ThrottleMax + (1000.f / 2.f) + (1000.f * 0.33 / 2.f) ? (EF._uORB_Dynamic_ThrottleMax - (EF._uORB_ESC_RPY_Range / 2)) : EF._uORB_Dynamic_ThrottleMax + (1000.f / 2.f) + (1000.f * 0.33 / 2.f);
		}

		if (EF._uORB_Total_Throttle < EF._uORB_Dynamic_ThrottleMin)
			EF._uORB_Total_Throttle = EF._uORB_Dynamic_ThrottleMin;
		else if (EF._uORB_Total_Throttle > EF._uORB_Dynamic_ThrottleMax)
			EF._uORB_Total_Throttle = EF._uORB_Dynamic_ThrottleMax;

		EF._Tmp_B1_Speed += EF._uORB_Total_Throttle;
		EF._Tmp_A1_Speed += EF._uORB_Total_Throttle;
		EF._Tmp_A2_Speed += EF._uORB_Total_Throttle;
		EF._Tmp_B2_Speed += EF._uORB_Total_Throttle;

		EF._Tmp_A1_Speed = EF._Tmp_A1_Speed < RF._flag_RC_Min_PWM_Value ? RF._flag_RC_Min_PWM_Value : EF._Tmp_A1_Speed;
		EF._Tmp_A2_Speed = EF._Tmp_A2_Speed < RF._flag_RC_Min_PWM_Value ? RF._flag_RC_Min_PWM_Value : EF._Tmp_A2_Speed;
		EF._Tmp_B1_Speed = EF._Tmp_B1_Speed < RF._flag_RC_Min_PWM_Value ? RF._flag_RC_Min_PWM_Value : EF._Tmp_B1_Speed;
		EF._Tmp_B2_Speed = EF._Tmp_B2_Speed < RF._flag_RC_Min_PWM_Value ? RF._flag_RC_Min_PWM_Value : EF._Tmp_B2_Speed;

		EF._Tmp_A1_Speed = EF._Tmp_A1_Speed > RF._flag_RC_Max_PWM_Value ? RF._flag_RC_Max_PWM_Value : EF._Tmp_A1_Speed;
		EF._Tmp_A2_Speed = EF._Tmp_A2_Speed > RF._flag_RC_Max_PWM_Value ? RF._flag_RC_Max_PWM_Value : EF._Tmp_A2_Speed;
		EF._Tmp_B1_Speed = EF._Tmp_B1_Speed > RF._flag_RC_Max_PWM_Value ? RF._flag_RC_Max_PWM_Value : EF._Tmp_B1_Speed;
		EF._Tmp_B2_Speed = EF._Tmp_B2_Speed > RF._flag_RC_Max_PWM_Value ? RF._flag_RC_Max_PWM_Value : EF._Tmp_B2_Speed;

		EF._uORB_A1_Speed = ((EF._Flag_Max__Throttle - EF._Flag_Lazy_Throttle) * (((float)EF._Tmp_A1_Speed - (float)RF._flag_RC_Min_PWM_Value) / (float)(RF._flag_RC_Max_PWM_Value - RF._flag_RC_Min_PWM_Value))) + EF._Flag_Lazy_Throttle;
		EF._uORB_A2_Speed = ((EF._Flag_Max__Throttle - EF._Flag_Lazy_Throttle) * (((float)EF._Tmp_A2_Speed - (float)RF._flag_RC_Min_PWM_Value) / (float)(RF._flag_RC_Max_PWM_Value - RF._flag_RC_Min_PWM_Value))) + EF._Flag_Lazy_Throttle;
		EF._uORB_B1_Speed = ((EF._Flag_Max__Throttle - EF._Flag_Lazy_Throttle) * (((float)EF._Tmp_B1_Speed - (float)RF._flag_RC_Min_PWM_Value) / (float)(RF._flag_RC_Max_PWM_Value - RF._flag_RC_Min_PWM_Value))) + EF._Flag_Lazy_Throttle;
		EF._uORB_B2_Speed = ((EF._Flag_Max__Throttle - EF._Flag_Lazy_Throttle) * (((float)EF._Tmp_B2_Speed - (float)RF._flag_RC_Min_PWM_Value) / (float)(RF._flag_RC_Max_PWM_Value - RF._flag_RC_Min_PWM_Value))) + EF._Flag_Lazy_Throttle;

		EF._uORB_A1_Speed = EF._uORB_A1_Speed < EF._Flag_Lazy_Throttle ? EF._Flag_Lazy_Throttle : EF._uORB_A1_Speed;
		EF._uORB_A2_Speed = EF._uORB_A2_Speed < EF._Flag_Lazy_Throttle ? EF._Flag_Lazy_Throttle : EF._uORB_A2_Speed;
		EF._uORB_B1_Speed = EF._uORB_B1_Speed < EF._Flag_Lazy_Throttle ? EF._Flag_Lazy_Throttle : EF._uORB_B1_Speed;
		EF._uORB_B2_Speed = EF._uORB_B2_Speed < EF._Flag_Lazy_Throttle ? EF._Flag_Lazy_Throttle : EF._uORB_B2_Speed;

		EF._uORB_A1_Speed = EF._uORB_A1_Speed > EF._Flag_Max__Throttle ? EF._Flag_Max__Throttle : EF._uORB_A1_Speed;
		EF._uORB_A2_Speed = EF._uORB_A2_Speed > EF._Flag_Max__Throttle ? EF._Flag_Max__Throttle : EF._uORB_A2_Speed;
		EF._uORB_B1_Speed = EF._uORB_B1_Speed > EF._Flag_Max__Throttle ? EF._Flag_Max__Throttle : EF._uORB_B1_Speed;
		EF._uORB_B2_Speed = EF._uORB_B2_Speed > EF._Flag_Max__Throttle ? EF._Flag_Max__Throttle : EF._uORB_B2_Speed;
	}
	TF._Tmp_IMUAttThreadLast = GetTimestamp();
}

void SingleAPMAPI::RPiSingleAPM::NavigationUpdate()
{
	TF._Tmp_IMUNavThreadDT = GetTimestamp() - TF._Tmp_IMUNavThreadLast;

	// If accelerometer measurement is clipped - drop the acc weight to zero
	// and gradually restore weight back to 1.0 over time
	if (SF._uORB_MPU_Data._uORB_MPU9250_ACC_Clipped)
	{
		SF._uORB_Accel_Clipped_Count++;
		PF._uORB_Accel_Dynamic_Beta = 0.f;
	}
	else
	{
		const float relAlpha = (TF._Tmp_IMUNavThreadDT / 1000000.f) / ((TF._Tmp_IMUNavThreadDT / 1000000.f) + ACC_CLIPPING_RC_CONSTANT);
		PF._uORB_Accel_Dynamic_Beta = PF._uORB_Accel_Dynamic_Beta * (1.0f - relAlpha) + 1.0f * relAlpha;
	}

	SF._uORB_MPU_Data._uORB_Acceleration_X -= PF._uORB_PID_AccelX_Bias;
	SF._uORB_True_Movement_X += (int)SF._uORB_True_Speed_X * (TF._Tmp_IMUNavThreadDT / 1000000.f);
	SF._uORB_True_Movement_X += (int)SF._uORB_MPU_Data._uORB_Acceleration_X * pow((TF._Tmp_IMUNavThreadDT / 1000000.f), 2) / 2.f * PF._uORB_Accel_Dynamic_Beta;
	SF._uORB_True_Speed_X += (int)SF._uORB_MPU_Data._uORB_Acceleration_X * (TF._Tmp_IMUNavThreadDT / 1000000.f) * pow(PF._uORB_Accel_Dynamic_Beta, 2);
	//---------------------------------------------------------//
	SF._uORB_MPU_Data._uORB_Acceleration_Y -= PF._uORB_PID_AccelY_Bias;
	SF._uORB_True_Movement_Y += (int)SF._uORB_True_Speed_Y * (TF._Tmp_IMUNavThreadDT / 1000000.f);
	SF._uORB_True_Movement_Y += (int)SF._uORB_MPU_Data._uORB_Acceleration_Y * pow((TF._Tmp_IMUNavThreadDT / 1000000.f), 2) / 2.f * PF._uORB_Accel_Dynamic_Beta;
	SF._uORB_True_Speed_Y += (int)SF._uORB_MPU_Data._uORB_Acceleration_Y * (TF._Tmp_IMUNavThreadDT / 1000000.f) * pow(PF._uORB_Accel_Dynamic_Beta, 2);
	//---------------------------------------------------------//
	SF._uORB_MPU_Data._uORB_Acceleration_Z -= PF._uORB_PID_AccelZ_Bias;
	SF._uORB_True_Movement_Z += (int)SF._uORB_True_Speed_Z * (TF._Tmp_IMUNavThreadDT / 1000000.f);
	SF._uORB_True_Movement_Z += (int)SF._uORB_MPU_Data._uORB_Acceleration_Z * pow((TF._Tmp_IMUNavThreadDT / 1000000.f), 2) / 2.f * PF._uORB_Accel_Dynamic_Beta;
	SF._uORB_True_Speed_Z += (int)SF._uORB_MPU_Data._uORB_Acceleration_Z * (TF._Tmp_IMUNavThreadDT / 1000000.f) * pow(PF._uORB_Accel_Dynamic_Beta, 2);
	//---------------------------------------------------------//
	SF._uORB_Gryo_Body_Asix_X += ((float)SF._uORB_MPU_Data._uORB_Gryo_Pitch) * (TF._Tmp_IMUNavThreadDT / 1000000.f);
	SF._uORB_Gryo_Body_Asix_Y += ((float)SF._uORB_MPU_Data._uORB_Gryo__Roll) * (TF._Tmp_IMUNavThreadDT / 1000000.f);
	//POS Controller Reseter
	{
		if (!(AF.AutoPilotMode == APModeINFO::AltHold || AF.AutoPilotMode == APModeINFO::PositionHold ||
			  AF.AutoPilotMode == APModeINFO::SpeedHold || AF.AutoPilotMode == APModeINFO::UserAuto) ||
			AF._flag_ESC_ARMED)
		{
			PF._uORB_PID_Alt_Throttle = 0;
			PF._uORB_PID_I_Last_Value_SpeedZ = 0;
			PF._uORB_PID_D_Last_Value_SpeedZ = 0;
			PF._uORB_PID_AltHold_Target = SF._uORB_True_Movement_Z;
		}
		//
		if (!(AF.AutoPilotMode == APModeINFO::PositionHold || AF.AutoPilotMode == APModeINFO::SpeedHold || AF.AutoPilotMode == APModeINFO::UserAuto) ||
			AF._flag_ESC_ARMED)
		{
			SF._uORB_Flow_XOutput_Total = 0;
			SF._uORB_Flow_YOutput_Total = 0;
			PF._uORB_PID_PosX_Output = 0;
			PF._uORB_PID_PosY_Output = 0;
			PF._uORB_PID_PosXUserTarget = 0;
			PF._uORB_PID_PosYUserTarget = 0;
			PF._uORB_PID_I_Last_Value_SpeedX = 0;
			PF._uORB_PID_D_Last_Value_SpeedX = 0;
			PF._uORB_PID_I_Last_Value_SpeedY = 0;
			PF._uORB_PID_D_Last_Value_SpeedY = 0;
		}
	}
	//Flag Controller
	{
		if (AF._flag_ESC_ARMED)
		{
			AF._flag_IsINUHDiable = true;
			PF._uORB_PID_Sonar_GroundOffset = 0 - SF._uORB_BARO_Altitude;
			PF._uORB_PID_AccelX_Bias = 0;
			PF._uORB_PID_AccelY_Bias = 0;
			PF._uORB_PID_AccelZ_Bias = 0;
			PF._uORB_PID_Sonar_GroundValid = true;
			PF._uORB_PID_Sonar_GroundTimeOut = GetTimestamp() + 250000;
		}
		else
		{
			AF._flag_IsINUHDiable = false;
			if (SF._uORB_True_Movement_Z > 15)
			{
				if (GetTimestamp() > PF._uORB_PID_Sonar_GroundTimeOut)
				{
					PF._uORB_PID_Sonar_GroundValid = false;
				}
			}
		}
		//
		if (AF._flag_IsINUHDiable)
		{
			PF._uORB_Sonar_Dynamic_Beta = 1.f;
			PF._uORB_Baro_Dynamic_Beta = 1.f;
		}
		else
		{
			PF._uORB_Sonar_Dynamic_Beta = PF._flag_Sonar_Config_Beta;
			PF._uORB_Baro_Dynamic_Beta = PF._flag_Baro_Config_Beta;
		}
	}
	//AutoTakeOff Function
	{
		if (!AF._flag_IsAutoTakeoffRequire)
		{
			PF._uORB_PID_Alt_Accel_Max = PF._flag_PID_Alt_Accel_Max;
			PF._uORB_PID_Alt_Speed_Max = PF._flag_PID_Alt_Speed_Max;
			//Vertical SpeedControll
			if (RF._uORB_RC_Out_AltHoldSpeed != 0 || PF._uORB_PID_PosZUserSpeed != 0)
			{
				PF._uORB_PID_AltHold_Target = SF._uORB_True_Movement_Z;
			}
		}
		else if (AF._flag_IsAutoTakeoffRequire)
		{
			PF._uORB_PID_AltHold_Target = PF._flag_PID_Takeoff_Altitude;
			PF._uORB_PID_Alt_Accel_Max = PF._flag_PID_TakeOff_Accel_Max;
			PF._uORB_PID_Alt_Speed_Max = PF._flag_PID_TakeOff_Speed_Max;
			if (PF._uORB_PID_AltInput_Final > PF._flag_PID_Takeoff_Altitude)
			{
				AF._flag_IsAutoTakeoffRequire = false;
			}

			if (RF._uORB_RC_Out_AltHoldSpeed != 0 || PF._uORB_PID_PosZUserSpeed != 0)
			{
				AF._flag_IsAutoTakeoffRequire = false;
			}
		}
		if (AF.AutoPilotMode != APModeINFO::UserAuto)
			PF._uORB_PID_PosZUserSpeed = 0;
	}
	//AltHold Caculate
	{
		PF._uORB_PID_MoveZCorrection = 0;
		PF._uORB_PID_SpeedZCorrection = 0;
		//===============================================//
		if (AF._flag_BARO_Async)
		{
			PF._uORB_PID_BARO_AltInput = SF._uORB_BARO_Altitude + PF._uORB_PID_Sonar_GroundOffset;
			AF._flag_BARO_Async = false;
		}
		if (AF._flag_SonarData_Async)
		{
			PF._uORB_PID_Sonar_AltInput = SF._uORB_Flow_Altitude_Final / 10.f;
			AF._flag_SonarData_Async = false;
		}
		//===============================================//
		//Air cushion effect
		bool isAirCushionEffectDetected = (PF._uORB_PID_Sonar_GroundValid && (PF._uORB_PID_BARO_AltInput < -1.f * PF._uORB_PID_Sonar_GroundOffset));
		PF._uORB_PID_BARO_AltInput = (isAirCushionEffectDetected ? 0 : PF._uORB_PID_BARO_AltInput);
		//===============================================//
		if (AF._flag_IsSonarAvalible)
		{
			PF._uORB_PID_MoveZCorrection += (PF._uORB_PID_Sonar_AltInput - SF._uORB_True_Movement_Z) *
											PF._uORB_Sonar_Dynamic_Beta *
											(TF._Tmp_IMUNavThreadDT / 1000000.f);
			PF._uORB_PID_SpeedZCorrection += (PF._uORB_PID_Sonar_AltInput - SF._uORB_True_Movement_Z) *
											 (PF._uORB_Sonar_Dynamic_Beta / 1.15f) *
											 (TF._Tmp_IMUNavThreadDT / 1000000.f);
			PF._uORB_PID_AccelZ_Bias -= (PF._uORB_PID_Sonar_AltInput - SF._uORB_True_Movement_Z) *
										(PF._uORB_Sonar_Dynamic_Beta / 1.5f) * PF._uORB_AccelBias_Beta *
										(TF._Tmp_IMUNavThreadDT / 1000000.f);
			PF._uORB_PID_Sonar_GroundOffset = PF._uORB_PID_Sonar_AltInput - SF._uORB_BARO_Altitude;
		}
		if (DF._IsBAROEnable)
		{
			PF._uORB_PID_MoveZCorrection += (PF._uORB_PID_BARO_AltInput - SF._uORB_True_Movement_Z) *
											PF._uORB_Baro_Dynamic_Beta *
											(TF._Tmp_IMUNavThreadDT / 1000000.f);
			PF._uORB_PID_SpeedZCorrection += (PF._uORB_PID_BARO_AltInput - SF._uORB_True_Movement_Z) *
											 pow(PF._uORB_Baro_Dynamic_Beta, 2) *
											 (TF._Tmp_IMUNavThreadDT / 1000000.f);
			PF._uORB_PID_AccelZ_Bias -= (PF._uORB_PID_BARO_AltInput - SF._uORB_True_Movement_Z) *
										pow(PF._uORB_Baro_Dynamic_Beta, 2) * PF._uORB_AccelBias_Beta *
										(TF._Tmp_IMUNavThreadDT / 1000000.f);
			PF._uORB_PID_Sonar_AltInput = PF._uORB_PID_BARO_AltInput;
		}
		else
		{
			PF._uORB_PID_MoveZCorrection += (0.f - SF._uORB_True_Movement_Z) *
											SpeedUnusableRES *
											(TF._Tmp_IMUNavThreadDT / 1000000.f);
			PF._uORB_PID_SpeedZCorrection += (0.f - SF._uORB_True_Speed_Z) *
											 SpeedUnusableRES *
											 (TF._Tmp_IMUNavThreadDT / 1000000.f);
			PF._uORB_PID_AccelZ_Bias -= (0.f - SF._uORB_True_Speed_Z) *
										SpeedUnusableRES * PF._uORB_AccelBias_Beta *
										(TF._Tmp_IMUNavThreadDT / 1000000.f);
		}
		//===============================================//
		SF._uORB_True_Movement_Z += PF._uORB_PID_MoveZCorrection;
		SF._uORB_True_Speed_Z += PF._uORB_PID_SpeedZCorrection;
		//===============================================//
		double TargetSpeed = -1 * (SF._uORB_True_Movement_Z - PF._uORB_PID_AltHold_Target) * PF._flag_PID_P_Alt_Gain -
							 RF._uORB_RC_Out_AltHoldSpeed - PF._uORB_PID_PosZUserSpeed;
		TargetSpeed = TargetSpeed > PF._uORB_PID_Alt_Speed_Max ? PF._uORB_PID_Alt_Speed_Max : TargetSpeed;
		TargetSpeed = TargetSpeed < -1 * PF._uORB_PID_Alt_Speed_Max ? -1 * PF._uORB_PID_Alt_Speed_Max : TargetSpeed;
		double TargetAccel = (TargetSpeed - SF._uORB_True_Speed_Z) * PF._flag_PID_I_Alt_Gain;
		TargetAccel = TargetAccel > PF._uORB_PID_Alt_Accel_Max ? PF._uORB_PID_Alt_Accel_Max : TargetAccel;
		TargetAccel = TargetAccel < -1 * PF._uORB_PID_Alt_Accel_Max ? -1 * PF._uORB_PID_Alt_Accel_Max : TargetAccel;
		PF._uORB_PID_InputTarget = TargetAccel - SF._uORB_MPU_Data._uORB_Acceleration_Z;
		//
		if (AF.AutoPilotMode == APModeINFO::AltHold || AF.AutoPilotMode == APModeINFO::PositionHold ||
			AF.AutoPilotMode == APModeINFO::SpeedHold || AF.AutoPilotMode == APModeINFO::UserAuto)
		{
			PID_CaculateExtend(PF._uORB_PID_InputTarget, PF._uORB_PID_InputTarget, PF._uORB_PID_InputTarget,
							   PF._uORB_PID_Alt_Throttle, PF._uORB_PID_I_Last_Value_SpeedZ, PF._uORB_PID_D_Last_Value_SpeedZ,
							   PF._flag_PID_P_SpeedZ_Gain, PF._flag_PID_I_SpeedZ_Gain / 100.f, PF._flag_PID_D_SpeedZ_Gain,
							   PF._flag_PID_Alt_Level_Max);
			PF._uORB_PID_Alt_Throttle = pt1FilterApply4(&DF.ThrottleLPF, PF._uORB_PID_Alt_Throttle, FILTERTHROTTLELPFCUTOFF, (TF._Tmp_IMUNavThreadDT / 1000000.f));
			PF._uORB_PID_Alt_Throttle = PF._uORB_PID_Alt_Throttle > PF._flag_PID_Alt_Level_Max ? PF._flag_PID_Alt_Level_Max : PF._uORB_PID_Alt_Throttle;
			PF._uORB_PID_Alt_Throttle = PF._uORB_PID_Alt_Throttle < -1 * PF._flag_PID_Alt_Level_Max ? -1 * PF._flag_PID_Alt_Level_Max : PF._uORB_PID_Alt_Throttle;
		}
	}
	//PositionHold Caculate
	{
		if (AF.AutoPilotMode == APModeINFO::SpeedHold)
		{
			PF._uORB_PID_PosXUserSpeed = 0;
			PF._uORB_PID_PosYUserSpeed = 0;
			PF._uORB_PID_PosXUserTarget = 0;
			PF._uORB_PID_PosYUserTarget = 0;
			if (RF._uORB_RC_Out_PosHoldSpeedX != 0)
			{
				AF._flag_IsPositionXChange = true;
				AF._flag_IsBrakingXBlock = true;
			}
			else
			{
				if (AF._flag_IsBrakingXBlock)
				{
					AF._flag_IsBrakingXSet = true;
					AF._flag_IsBrakingXBlock = false;
				}
			}
			if (RF._uORB_RC_Out_PosHoldSpeedY != 0)
			{
				AF._flag_IsPositionYChange = true;
				AF._flag_IsBrakingYBlock = true;
			}
			else
			{
				if (AF._flag_IsBrakingYBlock)
				{
					AF._flag_IsBrakingYSet = true;
					AF._flag_IsBrakingYBlock = false;
				}
			}
		}
		else if (AF.AutoPilotMode == APModeINFO::PositionHold)
		{
			PF._uORB_PID_PosXUserSpeed = 0;
			PF._uORB_PID_PosYUserSpeed = 0;
			PF._uORB_PID_PosXUserTarget = 0;
			PF._uORB_PID_PosYUserTarget = 0;
			RF._uORB_RC_Out_PosHoldSpeedX = 0;
			RF._uORB_RC_Out_PosHoldSpeedY = 0;
			if (RF._uORB_RC_Out__Roll != 0)
			{
				AF._flag_IsPositionXChange = true;
				AF._flag_IsBrakingXBlock = true;
			}
			else
			{
				if (AF._flag_IsBrakingXBlock)
				{
					AF._flag_IsBrakingXSet = true;
					AF._flag_IsBrakingXBlock = false;
				}
			}
			if (RF._uORB_RC_Out_Pitch != 0)
			{
				AF._flag_IsPositionYChange = true;
				AF._flag_IsBrakingYBlock = true;
			}
			else
			{
				if (AF._flag_IsBrakingYBlock)
				{
					AF._flag_IsBrakingYSet = true;
					AF._flag_IsBrakingYBlock = false;
				}
			}
		}
		else if (AF.AutoPilotMode == APModeINFO::UserAuto)
		{
			AF._flag_IsBrakingXSet = false;
			AF._flag_IsBrakingYSet = false;
			RF._uORB_RC_Out_PosHoldSpeedX = 0;
			RF._uORB_RC_Out_PosHoldSpeedY = 0;
			if (PF._uORB_PID_PosXUserSpeed != 0)
				AF._flag_IsPositionXChange = true;
			else
				AF._flag_IsPositionXChange = false;
			if (PF._uORB_PID_PosYUserSpeed != 0)
				AF._flag_IsPositionYChange = true;
			else
				AF._flag_IsPositionYChange = false;
		}
		else
		{
			PF._uORB_PID_PosXUserSpeed = 0;
			PF._uORB_PID_PosYUserSpeed = 0;
			PF._uORB_PID_PosXUserTarget = 0;
			PF._uORB_PID_PosYUserTarget = 0;
			RF._uORB_RC_Out_PosHoldSpeedX = 0;
			RF._uORB_RC_Out_PosHoldSpeedY = 0;
		}

		if (AF._flag_FlowData_Async)
		{
			PF._uORB_PID_Flow_PosInput_X = SF._uORB_Flow_YOutput_Total;
			PF._uORB_PID_Flow_PosInput_Y = SF._uORB_Flow_XOutput_Total;
			AF._flag_FlowData_Async = false;
		}

		if (AF._flag_IsPositionXChange || AF._flag_IsBrakingXSet)
		{
			SF._uORB_True_Movement_X = 0;
			SF._uORB_Flow_YOutput_Total = 0;
			AF._flag_IsPositionXChange = false;
			if (-10.f < SF._uORB_True_Speed_X && SF._uORB_True_Speed_X < 10.f)
				AF._flag_IsBrakingXSet = false;
		}
		if (AF._flag_IsPositionYChange || AF._flag_IsBrakingYSet)
		{
			SF._uORB_True_Movement_Y = 0;
			SF._uORB_Flow_XOutput_Total = 0;
			AF._flag_IsPositionYChange = false;
			if (-10.f < SF._uORB_True_Speed_Y && SF._uORB_True_Speed_Y < 10.f)
				AF._flag_IsBrakingYSet = false;
		}

		PF._uORB_PID_MoveXCorrection = 0;
		PF._uORB_PID_SpeedXCorrection = 0;
		PF._uORB_PID_MoveYCorrection = 0;
		PF._uORB_PID_SpeedYCorrection = 0;

		if (AF._flag_IsFlowAvalible && AF._flag_IsSonarAvalible)
		{
			PF._uORB_PID_MoveXCorrection += (PF._uORB_PID_Flow_PosInput_X - SF._uORB_True_Movement_X) *
											PF._flag_Flow_Dynamic_Beta *
											(TF._Tmp_IMUNavThreadDT / 1000000.f);
			PF._uORB_PID_SpeedXCorrection += (SF._uORB_Flow_Speed_Y - SF._uORB_True_Speed_X) *
											 (PF._flag_Flow_Dynamic_Beta / 1.15f) *
											 (TF._Tmp_IMUNavThreadDT / 1000000.f);
			PF._uORB_PID_AccelX_Bias -= (SF._uORB_Flow_Speed_Y - SF._uORB_True_Speed_X) *
										(PF._flag_Flow_Dynamic_Beta / 1.15f) * PF._uORB_AccelBias_Beta *
										(TF._Tmp_IMUNavThreadDT / 1000000.f);

			PF._uORB_PID_MoveYCorrection += (PF._uORB_PID_Flow_PosInput_Y - SF._uORB_True_Movement_Y) *
											PF._flag_Flow_Dynamic_Beta *
											(TF._Tmp_IMUNavThreadDT / 1000000.f);
			PF._uORB_PID_SpeedYCorrection += (SF._uORB_Flow_Speed_X - SF._uORB_True_Speed_Y) *
											 (PF._flag_Flow_Dynamic_Beta / 1.15f) *
											 (TF._Tmp_IMUNavThreadDT / 1000000.f);
			PF._uORB_PID_AccelY_Bias -= (SF._uORB_Flow_Speed_X - SF._uORB_True_Speed_Y) *
										(PF._flag_Flow_Dynamic_Beta / 1.15f) * PF._uORB_AccelBias_Beta *
										(TF._Tmp_IMUNavThreadDT / 1000000.f);
		}
		else
		{
			PF._uORB_PID_MoveXCorrection += (0.f - SF._uORB_True_Movement_X) *
											SpeedUnusableRES *
											(TF._Tmp_IMUNavThreadDT / 1000000.f);
			PF._uORB_PID_SpeedXCorrection += (0.f - SF._uORB_True_Speed_X) *
											 SpeedUnusableRES *
											 (TF._Tmp_IMUNavThreadDT / 1000000.f);
			PF._uORB_PID_AccelX_Bias -= (0.f - SF._uORB_True_Speed_X) *
										SpeedUnusableRES * PF._uORB_AccelBias_Beta *
										(TF._Tmp_IMUNavThreadDT / 1000000.f);

			PF._uORB_PID_MoveYCorrection += (0.f - SF._uORB_True_Movement_Y) *
											SpeedUnusableRES *
											(TF._Tmp_IMUNavThreadDT / 1000000.f);
			PF._uORB_PID_SpeedYCorrection += (0.f - SF._uORB_True_Speed_Y) *
											 SpeedUnusableRES *
											 (TF._Tmp_IMUNavThreadDT / 1000000.f);
			PF._uORB_PID_AccelY_Bias -= (0.f - SF._uORB_True_Speed_Y) *
										SpeedUnusableRES * PF._uORB_AccelBias_Beta *
										(TF._Tmp_IMUNavThreadDT / 1000000.f);
		}

		SF._uORB_True_Movement_X += PF._uORB_PID_MoveXCorrection;
		SF._uORB_True_Speed_X += PF._uORB_PID_SpeedXCorrection;
		SF._uORB_True_Movement_Y += PF._uORB_PID_MoveYCorrection;
		SF._uORB_True_Speed_Y += PF._uORB_PID_SpeedYCorrection;

		//Pos and Braking Dynamic caculate
		{
			if (abs(SF._uORB_True_Speed_X) < 10.f)
				PF._uORB_PID_I_PosX_Dynamic_Gain = 5.f;
			else if (10.f < abs(SF._uORB_True_Speed_X) && abs(SF._uORB_True_Speed_X) < 20.f)
				PF._uORB_PID_I_PosX_Dynamic_Gain = PF._flag_PID_I_PosX_Gain * ((abs(SF._uORB_True_Speed_X) - 10.f) / 20.f) + 5.f;
			else if (abs(SF._uORB_True_Speed_Y) >= 20.f)
				PF._uORB_PID_I_PosX_Dynamic_Gain = PF._flag_PID_I_PosX_Gain;

			if (abs(SF._uORB_True_Speed_Y) <= 10.f)
				PF._uORB_PID_I_PosY_Dynamic_Gain = 5.f;
			else if (10.f < abs(SF._uORB_True_Speed_Y) && abs(SF._uORB_True_Speed_Y) < 20.f)
				PF._uORB_PID_I_PosY_Dynamic_Gain = PF._flag_PID_I_PosY_Gain * ((abs(SF._uORB_True_Speed_Y) - 10.f) / 20.f) + 5.f;
			else if (abs(SF._uORB_True_Speed_Y) >= 20.f)
				PF._uORB_PID_I_PosY_Dynamic_Gain = PF._flag_PID_I_PosY_Gain;

			if (AF._flag_IsBrakingXSet && abs(SF._uORB_True_Speed_X) > 45.f)
			{
				PF._uORB_PID_Pos_AccelX_Max = PF._flag_PID_Pos_Accel_Max * PF._flag_Braking_AccelMax_Gain;
				PF._uORB_PID_I_PosX_Dynamic_Gain = PF._flag_PID_I_PosX_Gain * PF._flag_Braking_Speed_Gain;
			}
			else
			{
				PF._uORB_PID_Pos_AccelX_Max = PF._flag_PID_Pos_Accel_Max;
			}
			if (AF._flag_IsBrakingYSet && abs(SF._uORB_True_Speed_Y) > 45.f)
			{
				PF._uORB_PID_Pos_AccelY_Max = PF._flag_PID_Pos_Accel_Max * PF._flag_Braking_AccelMax_Gain;
				PF._uORB_PID_I_PosY_Dynamic_Gain = PF._flag_PID_I_PosY_Gain * PF._flag_Braking_Speed_Gain;
			}
			else
			{
				PF._uORB_PID_Pos_AccelY_Max = PF._flag_PID_Pos_Accel_Max;
			}
		}
		//
		double TargetSpeedX = (SF._uORB_True_Movement_X - PF._uORB_PID_PosXUserTarget) * PF._flag_PID_P_PosX_Gain -
							  RF._uORB_RC_Out_PosHoldSpeedX - PF._uORB_PID_PosXUserSpeed;
		TargetSpeedX = TargetSpeedX > PF._flag_PID_Pos_Speed_Max ? PF._flag_PID_Pos_Speed_Max : TargetSpeedX;
		TargetSpeedX = TargetSpeedX < -1 * PF._flag_PID_Pos_Speed_Max ? -1 * PF._flag_PID_Pos_Speed_Max : TargetSpeedX;

		double TargetAccelX = (TargetSpeedX + SF._uORB_True_Speed_X) * PF._uORB_PID_I_PosX_Dynamic_Gain;
		TargetAccelX = TargetAccelX > PF._uORB_PID_Pos_AccelX_Max ? PF._uORB_PID_Pos_AccelX_Max : TargetAccelX;
		TargetAccelX = TargetAccelX < -1 * PF._uORB_PID_Pos_AccelX_Max ? -1 * PF._uORB_PID_Pos_AccelX_Max : TargetAccelX;
		PF._uORB_PID_PosXTarget = TargetAccelX + SF._uORB_MPU_Data._uORB_Acceleration_X;
		//
		double TargetSpeedY = (SF._uORB_True_Movement_Y - PF._uORB_PID_PosYUserTarget) * PF._flag_PID_P_PosY_Gain -
							  RF._uORB_RC_Out_PosHoldSpeedY - PF._uORB_PID_PosYUserSpeed;
		TargetSpeedY = TargetSpeedY > PF._flag_PID_Pos_Speed_Max ? PF._flag_PID_Pos_Speed_Max : TargetSpeedY;
		TargetSpeedY = TargetSpeedY < -1 * PF._flag_PID_Pos_Speed_Max ? -1 * PF._flag_PID_Pos_Speed_Max : TargetSpeedY;

		double TargetAccelY = (TargetSpeedY + SF._uORB_True_Speed_Y) * PF._uORB_PID_I_PosY_Dynamic_Gain;
		TargetAccelY = TargetAccelY > PF._uORB_PID_Pos_AccelY_Max ? PF._uORB_PID_Pos_AccelY_Max : TargetAccelY;
		TargetAccelY = TargetAccelY < -1 * PF._uORB_PID_Pos_AccelY_Max ? -1 * PF._uORB_PID_Pos_AccelY_Max : TargetAccelY;
		PF._uORB_PID_PosYTarget = TargetAccelY + SF._uORB_MPU_Data._uORB_Acceleration_Y;
		//
		if (AF.AutoPilotMode == APModeINFO::PositionHold || AF.AutoPilotMode == APModeINFO::SpeedHold ||
			(AF.AutoPilotMode == APModeINFO::UserAuto && AF._flag_IsFlowAvalible))
		{
			PID_CaculateExtend(PF._uORB_PID_PosXTarget, PF._uORB_PID_PosXTarget, PF._uORB_PID_PosXTarget,
							   PF._uORB_PID_PosX_Output, PF._uORB_PID_I_Last_Value_SpeedX, PF._uORB_PID_D_Last_Value_SpeedX,
							   PF._flag_PID_P_SpeedX_Gain, PF._flag_PID_I_SpeedX_Gain / 100.f, PF._flag_PID_D_SpeedX_Gain, PF._flag_PID_Pos_Level_Max);
			PF._uORB_PID_PosX_Output = PF._uORB_PID_PosX_Output > PF._flag_PID_Pos_Level_Max ? PF._flag_PID_Pos_Level_Max : PF._uORB_PID_PosX_Output;
			PF._uORB_PID_PosX_Output = PF._uORB_PID_PosX_Output < -1 * PF._flag_PID_Pos_Level_Max ? -1 * PF._flag_PID_Pos_Level_Max : PF._uORB_PID_PosX_Output;
			PID_CaculateExtend(PF._uORB_PID_PosYTarget, PF._uORB_PID_PosYTarget, PF._uORB_PID_PosYTarget,
							   PF._uORB_PID_PosY_Output, PF._uORB_PID_I_Last_Value_SpeedY, PF._uORB_PID_D_Last_Value_SpeedY,
							   PF._flag_PID_P_SpeedY_Gain, PF._flag_PID_I_SpeedY_Gain / 100.f, PF._flag_PID_D_SpeedY_Gain, PF._flag_PID_Pos_Level_Max);
			PF._uORB_PID_PosY_Output = PF._uORB_PID_PosY_Output > PF._flag_PID_Pos_Level_Max ? PF._flag_PID_Pos_Level_Max : PF._uORB_PID_PosY_Output;
			PF._uORB_PID_PosY_Output = PF._uORB_PID_PosY_Output < -1 * PF._flag_PID_Pos_Level_Max ? -1 * PF._flag_PID_Pos_Level_Max : PF._uORB_PID_PosY_Output;
			PF._uORB_PID_PosX_Output = pt1FilterApply4(&DF.POSOutLPF[0], PF._uORB_PID_PosX_Output, FILTERPOSOUTLPFCUTOFF, (TF._Tmp_IMUNavThreadDT / 1000000.f));
			PF._uORB_PID_PosY_Output = pt1FilterApply4(&DF.POSOutLPF[1], PF._uORB_PID_PosY_Output, FILTERPOSOUTLPFCUTOFF, (TF._Tmp_IMUNavThreadDT / 1000000.f));
		}
	}

	TF._Tmp_IMUNavThreadLast = GetTimestamp();
}

void SingleAPMAPI::RPiSingleAPM::SaftyCheck()
{
	// SIG System Signal detect
	if (SystemSignal == SIGTERM || SystemSignal == SIGINT)
		RPiSingleAPMDeInit();
	//Error ARMED
	if (AF._flag_Error == true)
	{
		AF._flag_ESC_ARMED = true;
	}
	//PreARM Checking
	{
		if (!AF._flag_PreARM_Check_Lock)
		{
			if (SF._uORB_MPU_Data._uORB_Accel__Roll - 3 < SF._uORB_MPU_Data._uORB_Real__Roll &&
				SF._uORB_MPU_Data._uORB_Real__Roll < SF._uORB_MPU_Data._uORB_Accel__Roll + 3)
			{
				AF._flag_PreARM_Check = true;
				AF._flag_PreARM_Check_Level &= ~FailedSafeFlag::_flag_PreARMFailed_AngleNotSync;
			}
			else
			{
				AF._flag_PreARM_Check = false;
				AF._flag_PreARM_Check_Level |= FailedSafeFlag::_flag_PreARMFailed_AngleNotSync;
			}
			if (SF._uORB_MPU_Data._uORB_Accel_Pitch - 3 < SF._uORB_MPU_Data._uORB_Real_Pitch &&
				SF._uORB_MPU_Data._uORB_Real_Pitch < SF._uORB_MPU_Data._uORB_Accel_Pitch + 3)
			{
				AF._flag_PreARM_Check = true;
				AF._flag_PreARM_Check_Level &= ~FailedSafeFlag::_flag_PreARMFailed_AngleNotSync;
			}
			else
			{
				AF._flag_PreARM_Check = false;
				AF._flag_PreARM_Check_Level |= FailedSafeFlag::_flag_PreARMFailed_AngleNotSync;
			}
			//
			if (SF._uORB_MPU_Data._uORB_Gryo__Roll > -1.f &&
				SF._uORB_MPU_Data._uORB_Gryo__Roll < 1.f &&
				SF._uORB_MPU_Data._uORB_Gryo_Pitch > -1.f &&
				SF._uORB_MPU_Data._uORB_Gryo_Pitch < 1.f &&
				SF._uORB_MPU_Data._uORB_Gryo___Yaw > -1.f &&
				SF._uORB_MPU_Data._uORB_Gryo___Yaw < 1.f)
			{
				AF._flag_PreARM_Check = true;
				AF._flag_PreARM_Check_Level &= ~FailedSafeFlag::_flag_PreARMFailed_GyroNotStable;
			}
			else
			{
				AF._flag_PreARM_Check = false;
				AF._flag_PreARM_Check_Level |= FailedSafeFlag::_flag_PreARMFailed_GyroNotStable;
			}
			//
			if ((int)SF._uORB_True_Speed_X == 0 || (int)SF._uORB_True_Speed_Y == 0 || SF._uORB_True_Speed_Z == 0)
			{
				AF._flag_PreARM_Check = true;
				AF._flag_PreARM_Check_Level &= ~FailedSafeFlag::_flag_PreARMFailed_NavigationNotSync;
			}
			else
			{
				AF._flag_PreARM_Check = false;
				AF._flag_PreARM_Check_Level |= FailedSafeFlag::_flag_PreARMFailed_NavigationNotSync;
			}
			if ((int)SF._uORB_MPU_Data._uORB_Acceleration_X > -10.f &&
				(int)SF._uORB_MPU_Data._uORB_Acceleration_X < 10.f &&
				(int)SF._uORB_MPU_Data._uORB_Acceleration_Y > -10.f &&
				(int)SF._uORB_MPU_Data._uORB_Acceleration_Y < 10.f &&
				(int)SF._uORB_MPU_Data._uORB_Acceleration_Z > -10.f &&
				(int)SF._uORB_MPU_Data._uORB_Acceleration_Z < 10.f)
			{
				AF._flag_PreARM_Check = true;
				AF._flag_PreARM_Check_Level &= ~FailedSafeFlag::_flag_PreARMFailed_AccelNotStable;
			}
			else
			{
				AF._flag_PreARM_Check = false;
				AF._flag_PreARM_Check_Level |= FailedSafeFlag::_flag_PreARMFailed_AccelNotStable;
			}
			AF._flag_PreARM_Check_Lock = true;
		}
	}

	// FailedSafeCheck
	{
		if (AF._flag_RC_Error)
			AF._flag_FailedSafe_Level |= FailedSafeFlag::_flag_FailedSafe_RCLose;
		else
			AF._flag_FailedSafe_Level &= ~FailedSafeFlag::_flag_FailedSafe_RCLose;
		if (AF._flag_FakeRC_Error)
			AF._flag_FailedSafe_Level |= FailedSafeFlag::_flag_FailedSafe_FakeRCLose;
		else
			AF._flag_FailedSafe_Level &= ~FailedSafeFlag::_flag_FailedSafe_FakeRCLose;
		if (AF._flag_RC_Error && AF._flag_FakeRC_Error)
			AF._flag_Error = true;

		if (AF._flag_AnagleOutOfLimit)
		{
			AF._flag_FailedSafe_Level |= FailedSafeFlag::_flag_FailedSafe_AngleLimit;
			AF._flag_Error = true;
		}
		else
			AF._flag_FailedSafe_Level &= ~FailedSafeFlag::_flag_FailedSafe_AngleLimit;
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
	else if (AF.AutoPilotMode == APModeINFO::SpeedHold)
	{
		std::cout << " SpeedHold         ";
	}
	else if (AF.AutoPilotMode == APModeINFO::RateHold)
	{
		std::cout << " RateHold        ";
	}
	else if (AF.AutoPilotMode == APModeINFO::UserAuto)
	{
		std::cout << " UserAuto          ";
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
	std::cout << " GryoPitch:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Gryo_Pitch << "    "
			  << " GryoRoll:    " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Gryo__Roll << "    "
			  << " GryoYaw:     " << std::setw(7) << std::setfill(' ') << (int)PF._uORB_PID_GYaw_Output << "    "
			  << "                        "
			  << std::endl;
	std::cout << " RealPitch:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Real_Pitch << "    "
			  << " RealRoll:    " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Real__Roll << "    "
			  << " RealYaw :    " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Real___Yaw << "    "
			  << std::endl;
	std::cout << " AccePitch:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Accel_Pitch << "    "
			  << " AcceRoll:    " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Accel__Roll << "    "
			  << " AccelG  :    " << std::setw(7) << std::setfill(' ') << SF._uORB_MPU_Data._uORB_MPU9250_A_Vector << "    "
			  << "                        "
			  << std::endl;
	std::cout << " AccelXG:     " << std::setw(7) << std::setfill(' ') << SF._uORB_MPU_Data._uORB_MPU9250_ADF_X << "    "
			  << " AccelYG:     " << std::setw(7) << std::setfill(' ') << SF._uORB_MPU_Data._uORB_MPU9250_ADF_Y << "    "
			  << " AccelZG:     " << std::setw(7) << std::setfill(' ') << SF._uORB_MPU_Data._uORB_MPU9250_ADF_Z << "    "
			  << "                        "
			  << std::endl;
	std::cout << " VIBEX:       " << std::setw(7) << std::setfill(' ') << (int)(SF._uORB_MPU_Data._uORB_Accel_VIBE_X * 1000.f) << "    "
			  << " VIBEY:       " << std::setw(7) << std::setfill(' ') << (int)(SF._uORB_MPU_Data._uORB_Accel_VIBE_X * 1000.f) << "    "
			  << " VIBEZ:       " << std::setw(7) << std::setfill(' ') << (int)(SF._uORB_MPU_Data._uORB_Accel_VIBE_X * 1000.f) << "    "
			  << "                        "
			  << std::endl;
	std::cout << " AccelClipped:" << std::setw(7) << std::setfill(' ') << SF._uORB_MPU_Data._uORB_MPU9250_ACC_Clipped << "    "
			  << " AccelClCount:" << std::setw(7) << std::setfill(' ') << SF._uORB_Accel_Clipped_Count << "    "
			  << " AccelDynamic:" << std::setw(7) << std::setfill(' ') << PF._uORB_Accel_Dynamic_Beta << "    "
			  << "                        "
			  << std::endl;
	std::cout << " MAGYAW:      " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MAG_Yaw << "    "
			  << " MAGSYAW:     " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MAG_StaticYaw << "    "
			  << " IMUTIME:     " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_MPU9250_IMUUpdateTime << "         \n"
			  << " MAGRawX:     " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MAG_RawX << "    "
			  << " MAGRawY:     " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MAG_RawY << "    "
			  << " MAGRawZ:     " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MAG_RawZ << "    "
			  << std::endl;
	std::cout << " AccelrationX:" << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Acceleration_X << "cms2"
			  << " AccelrationY:" << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Acceleration_Y << "cms2"
			  << " AccelrationZ:" << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Acceleration_Z << "cms2"
			  << "                        "
			  << std::endl;
	std::cout << " SpeedX:      " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_True_Speed_X << "cms "
			  << " SpeedY       " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_True_Speed_Y << "cms "
			  << " SpeedZ:      " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_True_Speed_Z << "cms "
			  << "                        "
			  << std::endl;
	std::cout << " MoveX:       " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_True_Movement_X << "cm  "
			  << " MoveY        " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_True_Movement_Y << "cm  "
			  << " MoveZ:       " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_True_Movement_Z << "cm  "
			  << "                        "
			  << std::endl;
	std::cout
		<< "BAROParseDataINFO:"
		<< "\n";
	std::cout << " ||FilterPressure:     " << std::setw(7) << std::setfill(' ') << std::setiosflags(std::ios::fixed) << std::setprecision(2)
			  << SF._uORB_BARO_Data.PressureHPA << " hpa";
	std::cout << " ||Altitude:           " << std::setw(7) << std::setfill(' ') << (int)PF._uORB_PID_BARO_AltInput << "  cm";
	std::cout << " ||Temp:               " << std::setw(7) << std::setfill(' ') << std::setiosflags(std::ios::fixed) << std::setprecision(2)
			  << (SF._uORB_BARO_Data.TemperatureC) << "   C";
	std::cout << " ||AltHoldTarget:      " << std::setw(7) << std::setfill(' ') << (int)PF._uORB_PID_AltHold_Target << "  cm\n";
	std::cout << " ||AltholdThrottle:    " << std::setw(7) << std::setfill(' ') << (int)PF._uORB_PID_Alt_Throttle << "    ";
	std::cout << " ||TargetSpeed:        " << std::setw(7) << std::setfill(' ') << (int)PF._uORB_PID_InputTarget << "    \n";
	std::cout
		<< "FlowParseDataINFO:"
		<< "\n";
	std::cout << " FlowXOut:       " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Flow_XOutput << "    ";
	std::cout << " ||FlowYOut:     " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Flow_YOutput << "    ";
	std::cout << " ||FlowAltitude: " << std::setw(7) << std::setfill(' ') << (int)PF._uORB_PID_Sonar_AltInput << "    ";
	std::cout << " ||FlowSpeedX:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Flow_Speed_X << "cm/s            \n";
	std::cout << " FlowXOutTotal:  " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Flow_XOutput_Total << "    ";
	std::cout << " ||FlowYOutTotal:" << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Flow_YOutput_Total << "    ";
	std::cout << " ||FlowSpeed:    " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Flow_ClimbeRate << "cm/s";
	std::cout << " ||FlowQuality:  " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Flow_Quality << "    ";
	std::cout << " ||FlowSpeedY:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Flow_Speed_Y << "cm/s            \n";

	std::cout << "GPSDataINFO:"
			  << "\n";
	std::cout << " GPSLAT:       " << std::setw(10) << std::setfill(' ') << (int)SF._uORB_GPS_COR_Lat
			  << " ||GPSNE:          " << SF._uORB_GPS_Data.lat_North_Mode << " -> " << SF._uORB_GPS_Data.lat_East_Mode
			  << " ||PosXOutput: " << std::setw(10) << std::setfill(' ') << PF._uORB_PID_PosX_Output
			  << "            \n";
	std::cout << " GPSLNG:       " << std::setw(10) << std::setfill(' ') << (int)SF._uORB_GPS_COR_Lng
			  << " ||GPSSATCount:" << std::setw(10) << std::setfill(' ') << SF._uORB_GPS_Data.satillitesCount
			  << " ||PosYOutput: " << std::setw(10) << std::setfill(' ') << PF._uORB_PID_PosY_Output
			  << "            \n";

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
	std::cout << "                            \n";
	std::cout << " DynamicCencterFreqX:" << SF._uORB_MPU_Data._uORB_Gyro_Dynamic_NotchCenterHZ[0] << "            \n";
	std::cout << " DynamicCencterFreqY:" << SF._uORB_MPU_Data._uORB_Gyro_Dynamic_NotchCenterHZ[1] << "            \n";
	std::cout << " DynamicCencterFreqZ:" << SF._uORB_MPU_Data._uORB_Gyro_Dynamic_NotchCenterHZ[2] << "            \n";
	std::cout << " TPATrust           :" << PF._uORB_PID_TPA_Beta << "            \n";
	std::cout << " \n";

	std::bitset<16> x(AF._flag_PreARM_Check_Level);
	std::cout << " PreARMCheckInfo:    " << x << "                  \n";
	std::bitset<16> s(AF._flag_FailedSafe_Level);
	std::cout << " FailedSafekInfo:    " << s << "                  \n";

	std::cout << " Flag_ESC_ARMED:         " << std::setw(3) << std::setfill(' ') << AF._flag_ESC_ARMED << " |";
	std::cout << " Flag_Error:             " << std::setw(3) << std::setfill(' ') << AF._flag_Error << " |";
	std::cout << " TakeOffing:             " << std::setw(3) << std::setfill(' ') << AF._flag_IsAutoTakeoffRequire << " |";
	std::cout << " UserARMRequire:         " << std::setw(3) << std::setfill(' ') << AF._flag_ESC_DISARMED_Request << " |";
	std::cout << " Flag_GPS_Error:         " << std::setw(3) << std::setfill(' ') << AF._flag_GPS_Error << "           \n";
	std::cout << " Flag_ClockingTime_Error:" << std::setw(3) << std::setfill(' ') << AF._flag_ClockingTime_Error << " |";
	std::cout << " Flag_RC_Disconnected:   " << std::setw(3) << std::setfill(' ') << AF._flag_RC_Disconnected << " |";
	std::cout << " Flag_Flow_ErrorClock:   " << std::setw(3) << std::setfill(' ') << AF.Flow_Lose_Clocking << " |";
	std::cout << " Flag_PreARMCheckPass:   " << std::setw(3) << std::setfill(' ') << AF._flag_PreARM_Check << " |";
	std::cout << " Flag_GPS_Disconnected:  " << std::setw(3) << std::setfill(' ') << AF._flag_GPS_Disconnected << "         \n";
	std::cout << " Flag_FakeRC_Error:      " << std::setw(3) << std::setfill(' ') << AF._flag_FakeRC_Error << " |";
	std::cout << " Flag_FakeRC_Disconnect: " << std::setw(3) << std::setfill(' ') << AF.FakeRC_Lose_Clocking << " |";
	std::cout << " Flag_FakeRC_Deprive:    " << std::setw(3) << std::setfill(' ') << AF._flag_FakeRC_Deprive << " |";
	std::cout << " Flag_MPUCalibrating:    " << std::setw(3) << std::setfill(' ') << AF._flag_MPUCalibratingSet << " |";
	std::cout << " Flag_IsNotTakeOff:      " << std::setw(3) << std::setfill(' ') << AF._flag_IsNotTakeOff << "           \n";
	std::cout << " Flag_IsFlowAvalible:    " << std::setw(3) << std::setfill(' ') << AF._flag_IsFlowAvalible << " |";
	std::cout << " Flag_IsSonarAvalible:   " << std::setw(3) << std::setfill(' ') << AF._flag_IsSonarAvalible << " |";
	std::cout << " GPS_Lose_Clocking:      " << std::setw(3) << std::setfill(' ') << AF.GPS_Lose_Clocking << " |";
	std::cout << " Flag_RC_Error:          " << std::setw(3) << std::setfill(' ') << AF._flag_RC_Error << " |";
	std::cout << " RC_Lose_Clocking:       " << std::setw(3) << std::setfill(' ') << AF.RC_Lose_Clocking << "                        \n\n";

	std::cout << " IMULoopTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_IMUThreadTimeLoop;
	std::cout << " IMUNextTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_IMUThreadTimeNext;
	std::cout << " IMUErrorTime:" << std::setw(7) << std::setfill(' ') << TF._flag_IMUErrorTimes;
	std::cout << " IMUMaxTime:  " << std::setw(7) << std::setfill(' ') << TF._Tmp_IMUThreadError;
	std::cout << " IMUNAVDT:    " << std::setw(7) << std::setfill(' ') << TF._Tmp_IMUNavThreadDT << "    \n";
	std::cout << " RXTLoopTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_RXTThreadTimeLoop;
	std::cout << " RXTNextTime: " << std::setw(7) << std::setfill(' ') << TF._Tmp_RXTThreadTimeNext;
	std::cout << " RXTErrorTime:" << std::setw(7) << std::setfill(' ') << TF._flag_RXTErrorTimes;
	std::cout << " RXTMaxTime:  " << std::setw(7) << std::setfill(' ') << TF._Tmp_RXTThreadError;
	std::cout << " IMUATTDT:    " << std::setw(7) << std::setfill(' ') << TF._Tmp_IMUAttThreadDT << "    \n";
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
}

int SingleAPMAPI::RPiSingleAPM::GetTimestamp()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * (uint64_t)1000000 + tv.tv_usec;
}

//=-----------------------------------------------------------------------------------------==//
void SingleAPMAPI::RPiSingleAPM::APMControllerFakeRC(int *ChannelData, bool IsError)
{
	AF._flag_IsFakeRCUpdated = true;
	if (IsError)
		AF._flag_FakeRC_Error = true;
	for (size_t i = 0; i < 15; i++)
	{
		RF._uORB_FakeRC_Channel_PWM[i] = ChannelData[i];
	}
}

void SingleAPMAPI::RPiSingleAPM::APMControllerARMED()
{
	AF._flag_Error = false;
	AF._flag_GPS_Error = false;
	AF._flag_ClockingTime_Error = false;

	AF._flag_ESC_ARMED = true;
	AF._flag_StartUP_Protect = false;
	AF._flag_IsAutoTakeoffLock = false;
	AF._flag_IsAutoTakeoffRequire = false;

	AF._flag_IsNotTakeOff = false;
	AF._flag_IsNotTakeOff_Lock = true;

	AF._flag_PreARM_Check_Lock = false;
}

void SingleAPMAPI::RPiSingleAPM::APMControllerDISARM(APModeINFO APMode)
{
	if (APMode == APModeINFO::AutoStable || APMode == APModeINFO::RateHold)
	{
		if (AF._flag_Device_setupFailed == false)
		{
			if (AF._flag_Error == false)
			{
				if (AF._flag_StartUP_Protect == false)
				{
					if (AF._flag_PreARM_Check)
						AF._flag_ESC_ARMED = false;
				}
			}
			else
			{
				AF._flag_ESC_ARMED = true;
			}
		}
	}
	else if (APMode == APModeINFO::AltHold || APMode == APModeINFO::PositionHold ||
			 APMode == APModeINFO::SpeedHold || APMode == APModeINFO::UserAuto)
	{
		if (AF._flag_Device_setupFailed == false)
		{
			if (AF._flag_Error == false)
			{
				if (AF._flag_StartUP_Protect == false)
				{
					if (!AF._flag_IsAutoTakeoffLock)
					{
						AF._flag_IsAutoTakeoffRequire = true;
						AF._flag_IsAutoTakeoffLock = true;
					}
					if (AF._flag_PreARM_Check)
						AF._flag_ESC_ARMED = false;
				}
			}
			else
			{
				AF._flag_ESC_ARMED = true;
			}
		}
	}
};

void SingleAPMAPI::RPiSingleAPM::APMControllerPosition(int x, int y, int z, bool resetHome)
{
	if (AF.AutoPilotMode == APModeINFO::UserAuto)
	{
		if (resetHome)
		{
			SF._uORB_Flow_XOutput_Total = 0;
			SF._uORB_Flow_YOutput_Total = 0;
			resetHome = false;
		}
		PF._uORB_PID_PosXUserTarget = x;
		PF._uORB_PID_PosYUserTarget = y;
		PF._uORB_PID_AltHold_Target = z;
	}
}

void SingleAPMAPI::RPiSingleAPM::APMControllerSpeed(int x, int y, int z)
{
	if (AF.AutoPilotMode == APModeINFO::UserAuto)
	{
		PF._uORB_PID_PosXUserSpeed = x;
		PF._uORB_PID_PosYUserSpeed = y;
		PF._uORB_PID_PosZUserSpeed = z;
	}
	else
	{
		PF._uORB_PID_PosXUserSpeed = 0;
		PF._uORB_PID_PosYUserSpeed = 0;
		PF._uORB_PID_PosZUserSpeed = 0;
	}
}

void SingleAPMAPI::RPiSingleAPM::APMControllerServo(int pin, int on, int off)
{
}