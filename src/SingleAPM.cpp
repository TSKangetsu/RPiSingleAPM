#include "SingleAPM.hpp"

int SingleAPMAPI::RPiSingleAPM::RPiSingleAPMInit(APMSettinngs APMInit)
{
	{
		struct timespec tv;
		clock_gettime(CLOCK_MONOTONIC, &tv);
		TF._flag_SystemStartUp_Time = ((tv.tv_sec * (uint64_t)1000000 + (tv.tv_nsec / 1000)));
	}
	//--------------------------------------------------------------------//
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
		AF._flag_IsINUHDisable = true;
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
		AF._flag_IsNAVAvalible = false;
		AF._flag_GPS_Recovered = true;
		AF._flag_GPS_Error = true;
		AF.AutoPilotMode = APModeINFO::AutoStable;
		AF._flag_FailedSafe_Level = 0;
		AF._flag_PreARM_Check_Level = 0;
		PF._uORB_Accel_Dynamic_Beta = PF._flag_Accel_Config_Beta;
		AF._flag_MAG_Cali_Failed = !DF._IsGPSEnable ? true : false;

		TF._Tmp_IMUNavThreadDT = GetTimestamp();
		TF._Tmp_IMUNavThreadLast = GetTimestamp();
		TF._Tmp_IMUAttThreadDT = GetTimestamp();
		TF._uORB_IMUAttThreadDT = GetTimestamp();
		TF._Tmp_IMUAttThreadLast = GetTimestamp();
	}
	ConfigReader(APMInit);
	//--------------------------------------------------------------------//
	{
#ifdef RPiDEBUGStart
		std::cout << "[RPiSingleAPM]ESCControllerIniting \n";
#endif
		DF.ESCDevice.reset(new ESCGenerator(EF.ESCControllerType, DF.I2CDevice.c_str(), I2CPCA_ADDR, EF.ESCPLFrequency));
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
		else if (RF.RC_Type == RCIsCRSF)
		{
#ifdef RPiDEBUGStart
			std::cout << "[RPiSingleAPM]Controller CRSF config comfirm\n";
#endif
			DF.CRSFInit.reset(new CRSF(DF.RCDevice.c_str()));
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
			// DF.MTF02Device.reset(new MTF02("/dev/i2c-7", 0x31));
			// DF.BaroDeviceD.reset(new BaroDevice(BaroType::BMP280, DF.I2CDevice.c_str(), I2CBARO_ADDR));
#ifdef RPiDEBUGStart
			std::cout << "Done!\n";
#endif
		}
	}
	//--------------------------------------------------------------------//
	{
		if (DF._IsGPSEnable)
		{
			try
			{
#ifdef RPiDEBUGStart
				std::cout << "[RPiSingleAPM]Waiting for GPS Data ... ";
				std::cout.flush();
#endif
				DF.GPSInit.reset(new GPSUart(DF.GPSDevice.c_str()));
				int _Tmp_MAG_Flip[3] = {(int)SF._flag_COMPASS_Flip_Pitch, (int)SF._flag_COMPASS_Flip__Roll, (int)SF._flag_COMPASS_Flip___Yaw};
				DF.CompassDevice.reset(new GPSI2CCompass(DF.I2CDevice.c_str(), I2CCOMPASS_ADDR, COMPASS_HMC5883L, _Tmp_MAG_Flip));
				DF.CompassDevice->CompassApply(SF._flag_COMPASS_Cali[CompassXOffset], SF._flag_COMPASS_Cali[CompassXScaler],
											   SF._flag_COMPASS_Cali[CompassYOffset], SF._flag_COMPASS_Cali[CompassYScaler],
											   SF._flag_COMPASS_Cali[CompassZOffset], SF._flag_COMPASS_Cali[CompassZScaler]);
#ifdef RPiDEBUGStart
				std::cout << "Done \n";
#endif
			}
			catch (int &e)
			{
#ifdef RPiDEBUGStart
				std::cout << "ERROR \n";
#endif
				DF._IsGPSEnable = false;
				AF._flag_GPS_Error = true;
			}
		}
		//--------------------------------------------------------------------//
		if (DF._IsFlowEnable)
		{
			try
			{
#ifdef RPiDEBUGStart
				std::cout << "[RPiSingleAPM]Setting UP FlowSensor... ";
				std::cout.flush();
#endif
				// DF.FlowInit.reset(new MSPUartFlow(DF.FlowDevice.c_str()));
				DF.MTF02Device.reset(new MTF02("/dev/i2c-7", 0x31));

#ifdef RPiDEBUGStart
				std::cout << "Done \n";
#endif
			}
			catch (int &e)
			{
#ifdef RPiDEBUGStart
				std::cout << "ERROR \n";
#endif
				DF._IsFlowEnable = false;
			}
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
		config.MPU_Flip__Roll = SF._flag_MPU_Flip__Roll;
		config.MPU_Flip_Pitch = SF._flag_MPU_Flip_Pitch;
		config.MPU_Flip___Yaw = SF._flag_MPU_Flip___Yaw;
		//
		config.TargetFreqency = TF._flag_IMUFlowFreq;
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
		config.AccelFilterNotchCutOff = 80;
		config.AccelFilterNotchCenterFreq = 100;
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
#ifdef RPiDEBUGStart
		std::cout << "[RPiSingleAPM]Init Extenal Monitor: \n";
		std::cout << "[RPiSingleAPM]ADC ....";
		std::cout.flush();
#endif
		try
		{
			DF.ADCDevice.reset(new ADS111x(DF.I2CDevice.c_str()));
#ifdef RPiDEBUGStart
			std::cout << "Check! \n";
#endif
			for (size_t i = 0; i < 20; i++)
			{
				SF._uORB_BAT_Voltage =
					DF.ADCDevice->ADS111xReadmV({
						.Pin = 5,
						.Range = ADS111x::SL_RangeFSR::V4_096,
						.DataRate = ADS111x::SL_DataRateSPS::SPS_860,
					}) /
					1000.f / ADC_FRONT_GAIN;
				usleep(15 * 1000);
			}
			SF._uORB_BAT_Scount = (int)(SF._uORB_BAT_Voltage / 3.7f);
			//
			DF._IsADCEnable = true;
		}
		catch (int &e)
		{

#ifdef RPiDEBUGStart
			std::cout << "\n[RPiSingleAPM] ADC No Detected.\n";
#endif
			//
			DF._IsADCEnable = false;
		}
		//
	}
	//--------------------------------------------------------------------//
	{
		pt1FilterInit(&DF.ThrottleLPF, FILTERTHROTTLELPFCUTOFF, 0.f);
		pt1FilterInit(&DF.POSOutLPF[0], FILTERPOSOUTLPFCUTOFF, 0.f);
		pt1FilterInit(&DF.POSOutLPF[1], FILTERPOSOUTLPFCUTOFF, 0.f);
		pt1FilterInit(&DF.GPSSpeedLPF[0], FILTERPOSOUTLPFCUTOFF, 0.f);
		pt1FilterInit(&DF.GPSSpeedLPF[1], FILTERPOSOUTLPFCUTOFF, 0.f);
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
		pt1FilterInit(&DF.IMUDtLPF, FILTERIMUDTLPFCUTOFF, 0.0f);
		biquadFilterInitLPF(&DF.MAGFilter[0], FILTERMAGCUTOFF, 1.f / (float)TF._flag_MAGFlowFreq * 1000000.f);
		biquadFilterInitLPF(&DF.MAGFilter[1], FILTERMAGCUTOFF, 1.f / (float)TF._flag_MAGFlowFreq * 1000000.f);
		biquadFilterInitLPF(&DF.MAGFilter[2], FILTERMAGCUTOFF, 1.f / (float)TF._flag_MAGFlowFreq * 1000000.f);
	}
	//--------------------------------------------------------------------//
	{
		if (DF._IsBlackBoxEnable)
		{
			char outTime[16];
			std::time_t t = std::time(NULL);
			std::strftime(outTime, sizeof(outTime), "%Y%m%d%H%M%S", std::localtime(&t));
			std::string file = (std::string(BlackBoxLogDir, sizeof(BlackBoxLogDir) - 1) +
								std::string("SingleflightBlackBox", sizeof("SingleflightBlackBox") - 1) +
								std::string(".BBL", sizeof(".BBL") - 1));
			DF.BlackBoxFile.open(file.c_str(), std::ios::out | std::ios::binary | std::ios::app);

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
				{.FrameName = "gyroRaw[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "gyroRaw[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "gyroRaw[2]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "accSmooth[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "accSmooth[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "accSmooth[2]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "magADC[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "magADC[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "magADC[2]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "motor[0]", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
				{.FrameName = "motor[1]", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
				{.FrameName = "motor[2]", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
				{.FrameName = "motor[3]", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
				{.FrameName = "Flow[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "Flow[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "FlowSpeed[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "FlowSpeed[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "FlowQuality", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "BaroAlt", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "SonarAlt", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "navAcc[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "navAcc[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "navAcc[2]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "navSpeed[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "navSpeed[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "navSpeed[2]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "navPosr[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "navPosr[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "navPosr[2]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "navhead[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "navhead[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "navhead[2]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "POSOut[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "POSOut[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "AltThrottle", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "DynamicNotch[0]", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
				{.FrameName = "DynamicNotch[1]", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
				{.FrameName = "DynamicNotch[2]", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
				{.FrameName = "vbatLatest", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
				{.FrameName = "IMUDT", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
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
				{.FrameName = "gyroRaw[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "gyroRaw[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "gyroRaw[2]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				{.FrameName = "accSmooth[0]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "accSmooth[1]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "accSmooth[2]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "magADC[0]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "magADC[1]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "magADC[2]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "motor[0]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "motor[1]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "motor[2]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "motor[3]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "Flow[0]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "Flow[1]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "FlowSpeed[0]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "FlowSpeed[1]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "FlowQuality", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "BaroAlt", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "SonarAlt", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "navAcc[0]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "navAcc[1]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "navAcc[2]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "navSpeed[0]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "navSpeed[1]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "navSpeed[2]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "navPosr[0]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "navPosr[1]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "navPosr[2]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "navhead[0]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "navhead[1]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "navhead[2]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "POSOut[0]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "POSOut[1]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "AltThrottle", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "DynamicNotch[0]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "DynamicNotch[1]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "DynamicNotch[2]", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "vbatLatest", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
				{.FrameName = "IMUDT", .FrameSigned = 1, .FramePredictor = 1, .FrameEncoder = 0},
			};

			std::vector<BlackboxList> BlackBoxGInfo = {};
			std::vector<BlackboxList> BlackBoxHInfo = {};
			if (DF._IsGPSEnable)
			{
				BlackBoxGInfo = {
					{.FrameName = "time", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
					{.FrameName = "GPS_fixType", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
					{.FrameName = "GPS_numSat", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
					{.FrameName = "GPS_HDOP", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
					{.FrameName = "GPS_coord[0]", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
					{.FrameName = "GPS_coord[1]", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
					{.FrameName = "GPS_speed[0]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
					{.FrameName = "GPS_speed[1]", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
					{.FrameName = "MAG_head", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
					{.FrameName = "GPS_altitude", .FrameSigned = 1, .FramePredictor = 0, .FrameEncoder = 0},
				};

				BlackBoxHInfo = {
					{.FrameName = "GPS_home[0]", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
					{.FrameName = "GPS_home[1]", .FrameSigned = 0, .FramePredictor = 0, .FrameEncoder = 1},
				};
			}

			std::string PINTER = TF._flag_BBQThreadINT.c_str();
			std::string PINTERNUM = PINTER.substr(0, PINTER.find('/'));
			std::string PINTERDEM = PINTER.substr(PINTERNUM.size() + 1, PINTER.find('\0'));
			TF._flag_P_Interval = std::atoi(PINTERDEM.c_str());

			// Only support 1/4,1/2,1/1
			if (!(TF._flag_P_Interval == 4 || TF._flag_P_Interval == 2 || TF._flag_P_Interval == 1))
				return -1;

			DF.BlackBoxDevice.reset(new BlackboxEncoder({
				.IInterval = BlackBoxIInterval,
				.PInterval = TF._flag_BBQThreadINT.c_str(),
				.FirmwareType = BlackBoxFirmware,
				.BlackBoxDataIInfo = BlackBoxIInfo,
				.BlackBoxDataPInfo = BlackBoxPInfo,
				.BlackBoxDataGInfo = BlackBoxGInfo,
				.BlackBoxDataHInfo = BlackBoxHInfo,
				.BlackBoxDataSInfo = {},
				.BlackBoxCustom = {
					"H Firmware revision:Singleflight 0.9.5 BETA Raspberrypi4B\n",
					"H acc_1G:" + std::to_string((int)MPU9250_ACCEL_LSB) + "\n",
					"H gyro_scale:0x3f800000\n",
					"H motorOutput:1000,2000\n",
					"H minthrottle:1000\n",
					"H maxthrottle:2000\n",
					"H vbatscale:110\n",
					("H vbatref:" + std::to_string(SF._uORB_BAT_Voltage * 112.8) + "\n"),
					"H vbatcellvoltage:33,35,43\n",
					("H looptime:" + std::to_string(1.f / (float)TF._flag_IMUFlowFreq * 1000000.f) + "\n"),
				},
			}));
		}
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

		ExtendMonitorTaskReg();

		DF.APMStatus += 1;
	}
}

void SingleAPMAPI::RPiSingleAPM::RPiSingleAPMDeInit()
{
	//--------------------------------------------------------------------//
	TF._flag_Block_Task_Running = false;
	TF._flag_BBQ_Task_Running = false;
	TF._flag_BBW_Task_Running = false;

	if (TF.IMUFlow)
		TF.IMUFlow->FlowStopAndWait();
	if (TF.RTXFlow)
		TF.RTXFlow->FlowStopAndWait();
	if (TF.TELFlow)
		TF.TELFlow->FlowStopAndWait();
	if (TF.ESCFlow)
		TF.ESCFlow->FlowStopAndWait();
	if (TF.ALTFlow)
		TF.ALTFlow->FlowStopAndWait();
	if (TF.GPSFlow)
		TF.GPSFlow->FlowStopAndWait();
	if (TF.MAGFlow)
		TF.MAGFlow->FlowStopAndWait();
	if (TF.OPFFlow)
		TF.OPFFlow->FlowStopAndWait();
	if (TF.EXTFlow)
		TF.EXTFlow->FlowStopAndWait();

	TF.IMUFlow.reset();
	TF.RTXFlow.reset();
	TF.TELFlow.reset();
	TF.ESCFlow.reset();
	TF.ALTFlow.reset();
	TF.GPSFlow.reset();
	TF.MAGFlow.reset();
	TF.OPFFlow.reset();
	TF.EXTFlow.reset();

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
	DF.ADCDevice.reset();
	//
	FileInjectQueue(DF.BlackBoxFile, DF.BlackBoxDevice->BlackboxEPush(BlackboxEvent::FLIGHT_LOG_EVENT_LOG_END, 0, 0, 0));
	DF.BlackBoxFile.close();
	//--------------------------------------------------------------------//
	{
		AF.RC_Lose_Clocking = 0;
		AF.GPS_Lose_Clocking = 0;
		AF.Flow_Lose_Clocking = 0;
		AF.FakeRC_Deprive_Clocking = 0;
		AF._flag_FakeRC_Disconnected = 0;
		AF._flag_ESC_ARMED = true;
		AF._flag_IsINUHDisable = true;
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
	TF.IMUFlow.reset(new FlowThread(
		[&]
		{
			AF._flag_MAG_Cali_Failed = !DF._IsGPSEnable ? true : AF._flag_MAG_Cali_Failed;
			DF.MPUDevice->MPUSensorApplyAHRS(SF._uORB_MAG_RawX, SF._uORB_MAG_RawY, SF._uORB_MAG_RawZ, !AF._flag_MAG_Cali_Failed);
			//
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
					SF._flag_MPUCalibratorWaitClock += TF.IMUFlow->TimeDT;
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
		},
		TF._flag_Sys_CPU_Asign, TF._flag_IMUFlowFreq));
}

void SingleAPMAPI::RPiSingleAPM::AltholdSensorsTaskReg()
{
	if (DF._IsBAROEnable)
	{
		TF.ALTFlow.reset(new FlowThread(
			[&]
			{
				SF.MTFData = DF.MTF02Device->MTF02DataGet();
				float DT = (float)TF.ALTFlow->TimeDT / 1000000.f;
				SF._uORB_BARO_Altitude = pt1FilterApply3(&DF.BAROLPF, (SF.MTFData.Distance), DT);
				//

				AF._flag_BARO_Async = true;
			},
			TF._flag_Sys_CPU_Asign, TF._flag_ALTFlowFreq));
	}
}

void SingleAPMAPI::RPiSingleAPM::ControllerTaskReg()
{
	TF.RTXFlow.reset(new FlowThread(
		[&]
		{
			// RC Read Parse
			{
				if (RF.RC_Type == RCIsSbus)
				{
					// TODO: full posix read support
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
					// TODO: full posix read support
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
				else if (RF.RC_Type == RCIsCRSF)
				{
					// FIXME: support lower than 100HZ update speed
					if (DF.CRSFInit->CRSFRead(RF._Tmp_RC_Data, 15000) > 0)
					{
						for (size_t i = 0; i < 16; i++)
						{
							RF._uORB_RC_Channel_PWM[i] = DF.CRSFInit->rcToUs(RF._Tmp_RC_Data[i]);
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
			// RC Out Caculation
			{
				if (!AF._flag_RC_Disconnected)
				{
					float DT = (float)TF.RTXFlow->TimeDT / 1000000.f;
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
			// RC Safty Checking
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
			// RC data out
			{
				RF._uORB_RC_Out__Roll = RF._Tmp_RC_Out__Roll;
				RF._uORB_RC_Out_Pitch = RF._Tmp_RC_Out_Pitch;
				RF._uORB_RC_Out___Yaw = RF._Tmp_RC_Out___Yaw;
				RF._uORB_RC_Out_Throttle = RF._Tmp_RC_Out_Throttle;
			}
			// RC UNLOCK Checking
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
			// flyMode Switch
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
			// stick to speed controller
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
		},
		TF._flag_Sys_CPU_Asign, TF._flag_RTXFlowFreq));

	if (RF.RC_Type == RCIsCRSF) // FIXME: I think only crsf support telemetry...?
	{
		TF.TELFlow.reset(new FlowThread(
			[&]
			{
				// TODO: should support all bat args?
				DF.CRSFInit->CRSFTelemtry(CRSFTelemetry::crsfFrameBatterySensor(
					crsfProtocol::CRSF_SYNC_BYTE,
					SF._uORB_BAT_Voltage * 10,
					0 * 10,
					0,
					80));

				DF.CRSFInit->CRSFTelemtry(CRSFTelemetry::crsfFrameAttitude(
					crsfProtocol::CRSF_SYNC_BYTE,
					(int)SF._uORB_MPU_Data._uORB_Real_Pitch * 10,
					(int)SF._uORB_MPU_Data._uORB_Real__Roll * -10,
					(int)SF._uORB_MPU_Data._uORB_Real___Yaw * 10));

				// TODO: FlightMode
				const char *flightMode = "OK";
				if (!AF._flag_PreARM_Check && AF._flag_ESC_ARMED)
				{
					flightMode = "WAIT";
				}
				else
				{
					switch (AF.AutoPilotMode)
					{
					case APModeINFO::RateHold:
						flightMode = "ACRO";
						break;
					case APModeINFO::AutoStable:
						flightMode = "ANGL";
						break;
					case APModeINFO::AltHold:
						flightMode = "AH";
						break;
					case APModeINFO::PositionHold:
						flightMode = "HOLD";
						break;
					case APModeINFO::SpeedHold:
						flightMode = "CRSZ";
						break;
					case APModeINFO::UserAuto:
						flightMode = "WP";
						break;
					}
				}

				DF.CRSFInit->CRSFTelemtry(
					CRSFTelemetry::crsfFrameFlightMode(
						crsfProtocol::CRSF_SYNC_BYTE,
						flightMode));

				DF.CRSFInit->CRSFTelemtry(
					CRSFTelemetry::crsfFrameVarioSensor(
						crsfProtocol::CRSF_SYNC_BYTE,
						SF._uORB_True_Speed_Z));

				DF.CRSFInit->CRSFTelemtry(
					CRSFTelemetry::crsfFrameGps(crsfProtocol::CRSF_SYNC_BYTE,
												SF._uORB_True_Movement_X * 100,				// currently use as debug nav
												SF._uORB_True_Movement_Y * 100,				// currently use as debug nav
												0,											// currently not support
												(SF._uORB_MPU_Data._uORB_Real___Yaw * 100), // FIXME: uint16 define in edgetx is int16, so when value over 32700, will -value
												(SF._uORB_True_Movement_Z / 100.f) + 1000,
												SF._uORB_GPS_Data.satillitesCount));
			},
			TF._flag_Sys_CPU_Asign, TF._flag_TELFlowFreq));
	}
}

void SingleAPMAPI::RPiSingleAPM::PositionTaskReg()
{
	if (DF._IsGPSEnable)
	{
		TF.GPSFlow.reset(new FlowThread(
			[&]
			{
				{
					//===============================================================================================//
					SF._uORB_GPS_Data = DF.GPSInit->GPSParse();
					//===============================================================================================//
					if (SF._uORB_GPS_Data.satillitesCount > 4 && !SF._uORB_GPS_Data.DataUnCorrect &&
						SF._uORB_GPS_Data.lat != 0 && SF._uORB_GPS_Data.lng != 0)
					{
						AF._flag_GPS_Disconnected = false;
					}
					else
					{
						AF._flag_GPS_Disconnected = true;
					}
					//===============================================================================================//
					if (SF._uORB_GPS_Data.lat != 0 && SF._uORB_GPS_Data.lng != 0 &&
						!AF._flag_GPS_Error && !AF._flag_GPS_Disconnected)
					{
						SF._uORB_GPS_COR_Lat = SF._uORB_GPS_Data.lat;
						SF._uORB_GPS_COR_Lng = SF._uORB_GPS_Data.lng;
					}
					//
					if (AF._flag_GPS_Recovered && !AF._flag_GPS_Error)
					{
						SF._uORB_GPS_Real_Y = 0;
						SF._uORB_GPS_Real_X = 0;
						SF._uORB_GPS_Speed_Y = 0;
						SF._uORB_GPS_Speed_X = 0;
						SF._Tmp_GPS_Last_Lat = SF._uORB_GPS_COR_Lat;
						SF._Tmp_GPS_Last_Lng = SF._uORB_GPS_COR_Lng;
						SF._uORB_GPS_Hold_Lat = SF._uORB_GPS_COR_Lat;
						SF._uORB_GPS_Hold_Lng = SF._uORB_GPS_COR_Lng;
						AF._flag_GPS_Recovered = false;
					}
					//===============================================================================================//
					if (!AF._flag_GPS_Error)
					{
						int _Tmp_GPS_Static_Lat = SF._uORB_GPS_COR_Lat - SF._uORB_GPS_Hold_Lat;
						int _Tmp_GPS_Static_Lng = SF._uORB_GPS_COR_Lng - SF._uORB_GPS_Hold_Lng;
						SF._uORB_NAV_Yaw = (SF._uORB_MPU_Data._uORB_Real___Yaw > 180 && SF._uORB_MPU_Data._uORB_Real___Yaw < 360) ? SF._uORB_MPU_Data._uORB_Real___Yaw - 360 : SF._uORB_MPU_Data._uORB_Real___Yaw;

						SF._uORB_GPS_Real_Y = -1 * (_Tmp_GPS_Static_Lat * cos((SF._uORB_NAV_Yaw * PI / 180.f)) + _Tmp_GPS_Static_Lng * sin((SF._uORB_NAV_Yaw * PI / 180.f))) * 11.f;
						SF._uORB_GPS_Real_X = (_Tmp_GPS_Static_Lat * sin((SF._uORB_NAV_Yaw * PI / -180.f)) + _Tmp_GPS_Static_Lng * cos((SF._uORB_NAV_Yaw * PI / 180.f))) * 11.f;

						// FIXME: TimeDT will zero, x /0 will cause nan, fxck
						if (TF.GPSFlow->TimeDT != 0)
						{
							SF._uORB_GPS_Speed_Y = -1 * (((float)((SF._uORB_GPS_COR_Lat - SF._Tmp_GPS_Last_Lat) * cos((SF._uORB_NAV_Yaw * PI / 180.f)) + (SF._uORB_GPS_COR_Lng - SF._Tmp_GPS_Last_Lng) * sin((SF._uORB_NAV_Yaw * PI / 180.f))) * 11.f) / ((float)TF.GPSFlow->TimeDT / 1000000.f));
							SF._uORB_GPS_Speed_X = ((float)((SF._uORB_GPS_COR_Lat - SF._Tmp_GPS_Last_Lat) * sin((SF._uORB_NAV_Yaw * PI / -180.f)) + (SF._uORB_GPS_COR_Lng - SF._Tmp_GPS_Last_Lng) * cos((SF._uORB_NAV_Yaw * PI / 180.f))) * 11.f) / ((float)TF.GPSFlow->TimeDT / 1000000.f);
						}

						SF._Tmp_GPS_Last_Lat = SF._uORB_GPS_COR_Lat;
						SF._Tmp_GPS_Last_Lng = SF._uORB_GPS_COR_Lng;
					}
					//===============================================================================================//
					if (AF._flag_GPS_Disconnected == true)
					{
						AF.GPS_Lose_Clocking += 1;
						if (AF.GPS_Lose_Clocking == 50)
						{
							SF._uORB_GPS_Real_Y = 0;
							SF._uORB_GPS_Real_X = 0;
							SF._uORB_GPS_Speed_Y = 0;
							SF._uORB_GPS_Speed_X = 0;
							AF._flag_GPS_Error = true;
							AF.GPS_Lose_Clocking = 0;
						}
					}
					else if (AF._flag_GPS_Disconnected == false)
					{
						AF.GPS_Lose_Clocking = 0;
						AF._flag_GPS_Error = false;
					}
					//
					if (AF._flag_GPS_Error)
					{
						AF._flag_GPS_Recovered = true;
					}
					//===============================================================================================//
					AF._flag_GPSData_Async = true;
					AF._flag_GPSData_AsyncB = true;
				}
			},
			TF._flag_Sys_CPU_Asign, TF._flag_GPSFlowFreq));

		TF.MAGFlow.reset(new FlowThread(
			[&]
			{
				{
					DF.I2CLock.lock();
					DF.CompassDevice->CompassGetRaw(SF._uORB_MAG_RawX, SF._uORB_MAG_RawY, SF._uORB_MAG_RawZ);
					DF.I2CLock.unlock();

					SF._uORB_MAG_RawX = biquadFilterApply(&DF.MAGFilter[0], SF._uORB_MAG_RawX);
					SF._uORB_MAG_RawY = biquadFilterApply(&DF.MAGFilter[1], SF._uORB_MAG_RawY);
					SF._uORB_MAG_RawZ = biquadFilterApply(&DF.MAGFilter[2], SF._uORB_MAG_RawZ);
					SF._uORB_MAG_Vector = sqrtf((SF._uORB_MAG_RawX * SF._uORB_MAG_RawX) +
												(SF._uORB_MAG_RawY * SF._uORB_MAG_RawY) +
												(SF._uORB_MAG_RawZ * SF._uORB_MAG_RawZ));

					AF._flag_MAG_Cali_Failed = false;
					AF._flag_MAG_Cali_Failed = SF._uORB_MAG_Vector > SF._flag_COMPASS_Cali[CompassVOffset] ? true : AF._flag_MAG_Cali_Failed;
					AF._flag_MAG_Cali_Failed = SF._uORB_MAG_Vector < SF._flag_COMPASS_Cali[CompassVScaler] ? true : AF._flag_MAG_Cali_Failed;
					DF.CompassDevice->CompassGetFixAngle(SF._uORB_MAG_StaticYaw, SF._uORB_MPU_Data._uORB_Real__Roll, SF._uORB_MPU_Data._uORB_Real_Pitch);
					DF.CompassDevice->CompassGetUnfixAngle(SF._uORB_MAG_Yaw);
				}
			},
			TF._flag_Sys_CPU_Asign, TF._flag_MAGFlowFreq));
	}

	if (DF._IsFlowEnable)
	{
		TF.OPFFlow.reset(new FlowThread(
			[&]
			{
				{
					SF._Tmp_FlowThreadTimeout++;
					// NOTE: Movement is Y header, so revered X&Y
					// SF._Tmp_Flow___Status = DF.FlowInit->MSPDataRead(SF._uORB_Flow_YOutput, SF._uORB_Flow_XOutput, SF._uORB_Flow_Quality, SF._Tmp_Flow_Altitude, SF._uORB_RF_Quality);
					SF.MTFData = DF.MTF02Device->MTF02DataGet();
					SF._uORB_Flow_YOutput = SF.MTFData.Pos_Y;
					SF._uORB_Flow_XOutput = SF.MTFData.Pos_X;
					SF._uORB_Flow_Quality = SF.MTFData.Pos_Strength;
					SF._Tmp_Flow_Altitude = SF.MTFData.Distance;
					SF._uORB_RF_Quality = SF.MTFData.Distance_Strength;
					//
					if (SF._Tmp_Flow___Status == 1)
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
					if (SF._Tmp_Flow___Status == 2)
					{
						if (SF._uORB_Flow_Quality < 20.f)
							AF._flag_IsFlowAvalible = false;
						else
							AF._flag_IsFlowAvalible = true;

						// FIXME: CP:Pitch header revert , Now Posout is revert. Consider change accel x to revert?
						SF._uORB_Flow_Body_Asix_X = SF._uORB_Gryo_Body_Asix_X * 10.f;
						SF._uORB_Flow_Body_Asix_Y = SF._uORB_Gryo_Body_Asix_Y * 10.f;
						SF._uORB_Flow_Filter_XOutput = ((float)SF._uORB_Flow_XOutput + SF._uORB_Flow_Body_Asix_X) * SF._uORB_Flow_Altitude / 100.f;
						SF._uORB_Flow_Filter_YOutput = ((float)SF._uORB_Flow_YOutput + SF._uORB_Flow_Body_Asix_Y) * SF._uORB_Flow_Altitude / 100.f;
						// FIXME: TimeDT will zero, x /0 will cause nan, fxck
						if (TF.OPFFlow->TimeDT != 0)
						{
							SF._uORB_Flow_Speed_X = (SF._uORB_Flow_Filter_XOutput / 50.f) / ((float)TF.OPFFlow->TimeDT * 3.f / 1000000.f);
							SF._uORB_Flow_Speed_Y = (SF._uORB_Flow_Filter_YOutput / 50.f) / ((float)TF.OPFFlow->TimeDT * 3.f / 1000000.f);
							SF._uORB_Flow_XOutput_Total += SF._uORB_Flow_Speed_X * ((float)TF.OPFFlow->TimeDT * 3.f / 1000000.f);
							SF._uORB_Flow_YOutput_Total += SF._uORB_Flow_Speed_Y * ((float)TF.OPFFlow->TimeDT * 3.f / 1000000.f);
						}

						SF._Tmp_FlowThreadTimeout = 0;
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
					if (SF._Tmp_FlowThreadTimeout > 3)
					{
						SF._uORB_Gryo_Body_Asix_X = 0;
						SF._uORB_Gryo_Body_Asix_Y = 0;
					}
				}
			},
			TF._flag_Sys_CPU_Asign));
	}
}

void SingleAPMAPI::RPiSingleAPM::ExtendMonitorTaskReg()
{
	TF.EXTFlow.reset(new FlowThread(
		[&]
		{
			//
			if (DF._IsADCEnable)
			{
				DF.I2CLock.lock();
				SF._uORB_BAT_Voltage =
					DF.ADCDevice->ADS111xReadmV({
						.Pin = ADC_VBAT_PIN,
						.Range = ADS111x::SL_RangeFSR::V4_096,
						.DataRate = ADS111x::SL_DataRateSPS::SPS_860,
					}) /
					1000.f / ADC_FRONT_GAIN;
				DF.I2CLock.unlock();
				//
				SF._uORB_BAT_SingleVol = SF._uORB_BAT_Voltage / (float)SF._uORB_BAT_Scount;
			}
		},
		TF._flag_Sys_CPU_Asign, TF._flag_EXTFlowFreq));
}

void SingleAPMAPI::RPiSingleAPM::ESCUpdateTaskReg()
{
	TF.ESCFlow.reset(new FlowThread(
		[&]
		{
			DF.I2CLock.lock();
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
			DF.I2CLock.unlock();
		},
		TF._flag_Sys_CPU_Asign, TF._flag_ESCFlowFreq));
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
		TF.BlackBoxQTask = std::thread(
			[&]
			{
				int AutoPilotModeLast = APModeINFO::AutoStable;
				bool isAltholdSONARSwitch = true;
				bool isNAVSwitch = true;
				//
				bool IsFirstStart = true;
				bool DISARMLOGLock = true;
				bool ARMLOGLock = true;
				int BlackOvertime = TF._Tmp_BBQThreadTimeup;
				TF._flag_BBQ_Task_Running = true;
				while (TF._flag_BBQ_Task_Running)
				{
					TF._Tmp_BBQThreadTimeStart = GetTimestamp();
					TF._Tmp_BBQThreadTimeNext = TF._Tmp_BBQThreadTimeStart - TF._Tmp_BBQThreadTimeEnd;
					//
					if (!AF._flag_ESC_ARMED)
					{
						if (IsFirstStart || (!ARMLOGLock && (GetTimestamp() - TF._Tmp_BBQThreadTimeup) - BlackOvertime > 10000000))
						{
							std::vector<uint8_t> header(DF.BlackBoxDevice->FullBlackboxHeader.begin(), DF.BlackBoxDevice->FullBlackboxHeader.end());
							TF.BlackBoxQeueue.push(header);
						}

						TF.BlackBoxQeueue.push(DF.BlackBoxDevice->BlackboxPIPush(
							{
								TF._Tmp_BBQThreadloopIteration,
								GetTimestamp() - TF._Tmp_BBQThreadTimeup,
								(int)PF._uORB_Leveling__Roll,
								(int)PF._uORB_Leveling_Pitch,
								(int)PF._uORB_Leveling___Yaw,
								(-1 * (RF._flag_RC_Mid_PWM_Value - RF._uORB_RC_Channel_PWM[0])),
								(-1 * (RF._flag_RC_Mid_PWM_Value - RF._uORB_RC_Channel_PWM[1])),
								(RF._flag_RC_Mid_PWM_Value - RF._uORB_RC_Channel_PWM[3]),
								(RF._uORB_RC_Channel_PWM[2]),
								(int)(SF._uORB_MPU_Data._uORB_Real__Roll * 100.f),
								(int)(SF._uORB_MPU_Data._uORB_Real_Pitch * 100.f),
								(int)(SF._uORB_MPU_Data._uORB_Real___Yaw * 100.f),
								(int)SF._uORB_MPU_Data._uORB_Gryo__Roll,
								(int)SF._uORB_MPU_Data._uORB_Gryo_Pitch,
								(int)SF._uORB_MPU_Data._uORB_Gryo___Yaw,
								(int)(SF._uORB_MPU_Data._uORB_MPU9250_G_X / MPU9250_GYRO_LSB),
								(int)(SF._uORB_MPU_Data._uORB_MPU9250_G_Y / MPU9250_GYRO_LSB),
								(int)(SF._uORB_MPU_Data._uORB_MPU9250_G_Z / MPU9250_GYRO_LSB),
								(int)(SF._uORB_MPU_Data._uORB_MPU9250_ADF_Y * MPU9250_ACCEL_LSB),
								(int)(SF._uORB_MPU_Data._uORB_MPU9250_ADF_X * MPU9250_ACCEL_LSB),
								(int)(SF._uORB_MPU_Data._uORB_MPU9250_ADF_Z * MPU9250_ACCEL_LSB),
								SF._uORB_MAG_RawX,
								SF._uORB_MAG_RawY,
								SF._uORB_MAG_RawZ,
								EF._uORB_B2_Speed,
								EF._uORB_A2_Speed,
								EF._uORB_B1_Speed,
								EF._uORB_A1_Speed,
								(int)SF._uORB_Flow_YOutput_Total,
								(int)SF._uORB_Flow_XOutput_Total,
								(int)SF._uORB_Flow_Speed_Y,
								(int)SF._uORB_Flow_Speed_X,
								(int)SF._uORB_Flow_Quality,
								(int)PF._uORB_PID_BARO_AltInput,
								(int)PF._uORB_PID_Sonar_AltInput,
								(int)SF._uORB_MPU_Data._uORB_Acceleration_X,
								(int)SF._uORB_MPU_Data._uORB_Acceleration_Y,
								(int)SF._uORB_MPU_Data._uORB_Acceleration_Z,
								(int)SF._uORB_True_Speed_X,
								(int)SF._uORB_True_Speed_Y,
								(int)SF._uORB_True_Speed_Z,
								(int)SF._uORB_True_Movement_X,
								(int)SF._uORB_True_Movement_Y,
								(int)SF._uORB_True_Movement_Z,
								(int)SF._uORB_MAG_Yaw,
								(int)SF._uORB_MAG_StaticYaw,
								(int)SF._uORB_MPU_Data._uORB_Real___Yaw,
								(int)PF._uORB_PID_PosX_Output,
								(int)PF._uORB_PID_PosY_Output,
								(int)PF._uORB_PID_Alt_Throttle,
								(int)SF._uORB_MPU_Data._uORB_Gyro_Dynamic_NotchCenterHZ[0],
								(int)SF._uORB_MPU_Data._uORB_Gyro_Dynamic_NotchCenterHZ[1],
								(int)SF._uORB_MPU_Data._uORB_Gyro_Dynamic_NotchCenterHZ[2],
								(int)(SF._uORB_BAT_Voltage * 112.8),
								(int)TF._uORB_IMUAttThreadDT,
							}));

						if (AF._flag_GPSData_AsyncB)
						{
							if (!SF._uORB_GPS_Data.DataUnCorrect)
							{
								TF.BlackBoxQeueue.push(DF.BlackBoxDevice->BlaclboxGPush({
									GetTimestamp() - TF._Tmp_BBQThreadTimeup,
									SF._uORB_GPS_Data.GPSQuality,
									SF._uORB_GPS_Data.satillitesCount,
									(int)(SF._uORB_GPS_Data.HDOP * 100),
									(int)(SF._uORB_GPS_COR_Lat * 10),
									(int)(SF._uORB_GPS_COR_Lng * 10),
									(int)(SF._uORB_GPS_Speed_X),
									(int)(SF._uORB_GPS_Speed_Y),
									(int)SF._uORB_MPU_Data._uORB_Real___Yaw,
									(int)(SF._uORB_GPS_Data.GPSAlititude),
								}));
								TF.BlackBoxQeueue.push(DF.BlackBoxDevice->BlaclboxHPush({
									(int)(SF._uORB_GPS_COR_Lat * 10),
									(int)(SF._uORB_GPS_COR_Lng * 10),
								}));
							}
							AF._flag_GPSData_AsyncB = false;
						}
						// Handle event flightmode
						if (AutoPilotModeLast != AF.AutoPilotMode)
						{
							// TODO: need to convert usr mode
							switch (AF.AutoPilotMode)
							{
							case APModeINFO::AutoStable:
								TF.BlackBoxQeueue.push(DF.BlackBoxDevice->BlackboxEPush(
									BlackboxEvent::FLIGHT_LOG_EVENT_FLIGHTMODE,
									0, 0, RCFlightModeEVENT::ANGLE, 0));
								break;
							case APModeINFO::AltHold:
								TF.BlackBoxQeueue.push(DF.BlackBoxDevice->BlackboxEPush(
									BlackboxEvent::FLIGHT_LOG_EVENT_FLIGHTMODE,
									0, 0, RCFlightModeEVENT::BARO | RCFlightModeEVENT::SONAR, 0));
								break;
							case APModeINFO::PositionHold:
								TF.BlackBoxQeueue.push(DF.BlackBoxDevice->BlackboxEPush(
									BlackboxEvent::FLIGHT_LOG_EVENT_FLIGHTMODE,
									0, 0, RCFlightModeEVENT::GPSHOLD | RCFlightModeEVENT::ANGLE, 0));
								break;
							case APModeINFO::SpeedHold:
								TF.BlackBoxQeueue.push(DF.BlackBoxDevice->BlackboxEPush(
									BlackboxEvent::FLIGHT_LOG_EVENT_FLIGHTMODE,
									0, 0, RCFlightModeEVENT::GPSHOLD | RCFlightModeEVENT::GPSHOME, 0));
								break;
							case APModeINFO::UserAuto:
								TF.BlackBoxQeueue.push(DF.BlackBoxDevice->BlackboxEPush(
									BlackboxEvent::FLIGHT_LOG_EVENT_FLIGHTMODE,
									0, 0, RCFlightModeEVENT::GPSHOLD | RCFlightModeEVENT::PASSTHRU, 0));
								break;
							case APModeINFO::RateHold:
								TF.BlackBoxQeueue.push(DF.BlackBoxDevice->BlackboxEPush(
									BlackboxEvent::FLIGHT_LOG_EVENT_FLIGHTMODE,
									0, 0, RCFlightModeEVENT::ACRO, 0));
								break;
							}
							AutoPilotModeLast = AF.AutoPilotMode;
						}
						// Handle Event from system
						if (isAltholdSONARSwitch != AF._flag_IsSonarAvalible)
						{
							if (AF._flag_IsSonarAvalible)
							{
								TF.BlackBoxQeueue.push(DF.BlackBoxDevice->BlackboxEPush(
									BlackboxEvent::FLIGHT_LOG_EVENT_FLIGHTMODE,
									0, 0, RCFlightModeEVENT::SONAR, RCFlightModeEVENT::BARO));
							}
							else
							{
								TF.BlackBoxQeueue.push(DF.BlackBoxDevice->BlackboxEPush(
									BlackboxEvent::FLIGHT_LOG_EVENT_FLIGHTMODE,
									0, 0, RCFlightModeEVENT::BARO, RCFlightModeEVENT::SONAR));
							}
							isAltholdSONARSwitch = AF._flag_IsSonarAvalible;
						}
						if (isNAVSwitch != AF._flag_IsNAVAvalible)
						{
							if (AF._flag_IsNAVAvalible)
							{
								TF.BlackBoxQeueue.push(DF.BlackBoxDevice->BlackboxEPush(
									BlackboxEvent::FLIGHT_LOG_EVENT_FLIGHTMODE,
									0, 0, RCFlightModeEVENT::GPSHOLD, 0));
							}
							else
							{
								TF.BlackBoxQeueue.push(DF.BlackBoxDevice->BlackboxEPush(
									BlackboxEvent::FLIGHT_LOG_EVENT_FLIGHTMODE,
									0, 0, 0, RCFlightModeEVENT::GPSHOLD));
							}
							isNAVSwitch = AF._flag_IsNAVAvalible;
						}
						//====================================================================//
						TF._Tmp_BBQThreadloopIteration += TF._flag_P_Interval;

						if (IsFirstStart)
						{
							TF.BlackBoxQeueue.push(
								DF.BlackBoxDevice->BlackboxEPush(
									BlackboxEvent::FLIGHT_LOG_EVENT_SYNC_BEEP,
									(GetTimestamp() - TF._Tmp_BBQThreadTimeup), 0, 0));
							IsFirstStart = false;
							ARMLOGLock = true;
						}
						else if (!ARMLOGLock)
						{
							if ((GetTimestamp() - TF._Tmp_BBQThreadTimeup) - BlackOvertime > 10000000)
							{
								TF.BlackBoxQeueue.push(
									DF.BlackBoxDevice->BlackboxEPush(
										BlackboxEvent::FLIGHT_LOG_EVENT_SYNC_BEEP,
										(GetTimestamp() - TF._Tmp_BBQThreadTimeup), 0, 0));
							}
							else
							{
								TF.BlackBoxQeueue.push(
									DF.BlackBoxDevice->BlackboxEPush(
										BlackboxEvent::FLIGHT_LOG_EVENT_LOGGING_RESUME,
										(GetTimestamp() - TF._Tmp_BBQThreadTimeup),
										TF._Tmp_BBQThreadloopIteration, 0));
								TF.BlackBoxQeueue.push(
									DF.BlackBoxDevice->BlackboxEPush(
										BlackboxEvent::FLIGHT_LOG_EVENT_SYNC_BEEP,
										(GetTimestamp() - TF._Tmp_BBQThreadTimeup), 0, 0));
							}
							ARMLOGLock = true;
						}

						DISARMLOGLock = false;
					}
					else
					{
						if (!IsFirstStart)
						{
							if (!DISARMLOGLock)
							{
								TF.BlackBoxQeueue.push(DF.BlackBoxDevice->BlackboxEPush(BlackboxEvent::FLIGHT_LOG_EVENT_DISARM, 0, 0, 4));
								BlackOvertime = GetTimestamp() - TF._Tmp_BBQThreadTimeup;
								DISARMLOGLock = true;
							}
						}
						ARMLOGLock = false;
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

		TF.BlackBoxWTask = std::thread(
			[&]
			{
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
		int datas[10] = {0};
		datas[0] = -5000;
		datas[2] = -5000;
		datas[4] = -5000;
		datas[6] = -5000;
		datas[1] = 5000;
		datas[3] = 5000;
		datas[5] = 5000;
		datas[7] = 5000;
		int rawx = 0;
		int rawy = 0;
		int rawz = 0;
		DF.CompassDevice->CompassCaliInit();
		while (true)
		{
			DF.CompassDevice->CompassGetRaw(rawx, rawy, rawz);
			DF.CompassDevice->CompassCalibration(true, datas);
			usleep(50 * 1000);
			if (SingleAPMAPI::SystemSignal == SIGINT)
			{
				DF.CompassDevice->CompassApply(datas[0], datas[1], datas[2], datas[3], datas[4], datas[5]);
				DF.CompassDevice->CompassGetRaw(rawx, rawy, rawz);
				SF._uORB_MAG_Vector = sqrtf((rawx * rawx) +
											(rawy * rawy) +
											(rawz * rawz));
				datas[6] = SF._uORB_MAG_Vector;
				datas[7] = SF._uORB_MAG_Vector;

				for (size_t i = 0; i < 200; i++)
				{
					DF.CompassDevice->CompassGetRaw(rawx, rawy, rawz);
					SF._uORB_MAG_Vector = sqrtf((rawx * rawx) +
												(rawy * rawy) +
												(rawz * rawz));
					datas[6] = SF._uORB_MAG_Vector > datas[6] ? SF._uORB_MAG_Vector : datas[6];
					datas[7] = SF._uORB_MAG_Vector < datas[7] ? SF._uORB_MAG_Vector : datas[7];
					usleep(20000);
				}

				data[0] = (int)datas[0];
				data[1] = (int)datas[1];
				data[2] = (int)datas[2];
				data[3] = (int)datas[3];
				data[4] = (int)datas[4];
				data[5] = (int)datas[5];
				data[6] = (int)(datas[6] + datas[6] * 0.2);
				data[7] = (int)(datas[7] - datas[7] * 0.2);
				break;
			}
		}
	}
	return -1;
}

//=-----------------------------------------------------------------------------------------==//

void SingleAPMAPI::RPiSingleAPM::PID_Caculate(float inputData, float &outputData,
											  float &last_I_Data, float &last_D_Data,
											  float P_Gain, float I_Gain, float D_Gain, float I_Max)
{
	// P caculate
	outputData = P_Gain * inputData;
	// D caculate
	outputData += D_Gain * (inputData - last_D_Data);
	last_D_Data = inputData;
	// I caculate
	last_I_Data += inputData * I_Gain;
	if (last_I_Data > I_Max)
		last_I_Data = I_Max;
	if (last_I_Data < I_Max * -1)
		last_I_Data = I_Max * -1;
	// P_I_D Mix OUTPUT
	outputData += last_I_Data;
}

void SingleAPMAPI::RPiSingleAPM::PID_CaculateExtend(float inputDataP, float inputDataI, float inputDataD, float &outputData,
													float &last_I_Data, float &last_D_Data,
													float P_Gain, float I_Gain, float D_Gain, float I_Max)
{
	// P caculate
	outputData = P_Gain * inputDataP;
	// D caculate
	outputData += D_Gain * (inputDataD - last_D_Data);
	last_D_Data = inputDataD;
	// I caculate
	last_I_Data += inputDataI * I_Gain;
	if (last_I_Data > I_Max)
		last_I_Data = I_Max;
	if (last_I_Data < I_Max * -1)
		last_I_Data = I_Max * -1;
	// P_I_D Mix OUTPUT
	outputData += last_I_Data;
}

void SingleAPMAPI::RPiSingleAPM::PID_CaculateHyper(float inputDataP, float inputDataI, float inputDataD, float &outputData,
												   float &last_I_Data, float &last_D_Data,
												   float P_Gain, float I_Gain, float D_Gain, float I_Max)
{
	// P caculate
	outputData = P_Gain * inputDataP;
	// D caculate
	outputData += D_Gain * inputDataD;
	// I caculate
	last_I_Data += inputDataI * I_Gain;
	if (last_I_Data > I_Max)
		last_I_Data = I_Max;
	if (last_I_Data < I_Max * -1)
		last_I_Data = I_Max * -1;
	// P_I_D Mix OUTPUT
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
	DF.I2CDevice = APMInit.DC.__I2CDevice;

	DF._IsGPSEnable = APMInit.DC._IsGPSEnable;
	DF._IsFlowEnable = APMInit.DC._IsFlowEnable;
	DF._IsRCSafeEnable = APMInit.DC._IsRCSafeEnable;
	DF._IsBAROEnable = APMInit.DC._IsBAROEnable;
	DF._IsBlackBoxEnable = APMInit.DC._IsBlackBoxEnable;

	TF._flag_IMUFlowFreq = APMInit.DC.IMU_Freqeuncy;
	TF._flag_RTXFlowFreq = APMInit.DC.RXT_Freqeuncy;
	TF._flag_ESCFlowFreq = APMInit.DC.ESC_Freqeuncy;
	TF._flag_BBQThreadINT = APMInit.DC.BBC_PInterval;
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
	SF._flag_MPU_Flip__Roll = APMInit.SC._flag_MPU_Flip__Roll;
	SF._flag_MPU_Flip_Pitch = APMInit.SC._flag_MPU_Flip_Pitch;
	SF._flag_MPU_Flip___Yaw = APMInit.SC._flag_MPU_Flip___Yaw;

	SF._flag_MPU_Accel_Cali[MPUAccelCaliX] = APMInit.SC._flag_MPU9250_A_X_Cali;
	SF._flag_MPU_Accel_Cali[MPUAccelCaliY] = APMInit.SC._flag_MPU9250_A_Y_Cali;
	SF._flag_MPU_Accel_Cali[MPUAccelCaliZ] = APMInit.SC._flag_MPU9250_A_Z_Cali;
	SF._flag_MPU_Accel_Cali[MPUAccelScalX] = APMInit.SC._flag_MPU9250_A_X_Scal;
	SF._flag_MPU_Accel_Cali[MPUAccelScalY] = APMInit.SC._flag_MPU9250_A_Y_Scal;
	SF._flag_MPU_Accel_Cali[MPUAccelScalZ] = APMInit.SC._flag_MPU9250_A_Z_Scal;
	SF._flag_MPU_Accel_Cali[MPUAccelTRIMPitch] = APMInit.SC._flag_Accel_Pitch_Cali;
	SF._flag_MPU_Accel_Cali[MPUAccelTRIM_Roll] = APMInit.SC._flag_Accel__Roll_Cali;

	SF._flag_COMPASS_Cali[CompassXOffset] = APMInit.SC._flag_COMPASS_X_Offset;
	SF._flag_COMPASS_Cali[CompassXScaler] = APMInit.SC._flag_COMPASS_X_Scaler;
	SF._flag_COMPASS_Cali[CompassYOffset] = APMInit.SC._flag_COMPASS_Y_Offset;
	SF._flag_COMPASS_Cali[CompassYScaler] = APMInit.SC._flag_COMPASS_Y_Scaler;
	SF._flag_COMPASS_Cali[CompassZOffset] = APMInit.SC._flag_COMPASS_Z_Offset;
	SF._flag_COMPASS_Cali[CompassZScaler] = APMInit.SC._flag_COMPASS_Z_Scaler;
	SF._flag_COMPASS_Cali[CompassVOffset] = APMInit.SC._flag_COMPASS_V_Offset;
	SF._flag_COMPASS_Cali[CompassVScaler] = APMInit.SC._flag_COMPASS_V_Scaler;
	SF._flag_COMPASS_Flip__Roll = APMInit.SC._flag_COMPASS_Flip__Roll;
	SF._flag_COMPASS_Flip_Pitch = APMInit.SC._flag_COMPASS_Flip_Pitch;
	SF._flag_COMPASS_Flip___Yaw = APMInit.SC._flag_COMPASS_Flip___Yaw;
	SF._flag_COMPASS_YAW_Offset = APMInit.SC._flag_COMPASS_YAW_Offset;
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
}

//=-----------------------------------------------------------------------------------------==//

void SingleAPMAPI::RPiSingleAPM::AttitudeUpdate()
{
	TF._Tmp_IMUAttThreadDT = GetTimestamp() - TF._Tmp_IMUAttThreadLast;

	// Roll PID Mix
	// FIXME: Before I or D controller, shoud filter DT, it's bumpping
	TF._uORB_IMUAttThreadDT = (int)pt1FilterApply4(&DF.IMUDtLPF, (int)TF._Tmp_IMUAttThreadDT, FILTERIMUDTLPFCUTOFF, 1.f / TF._flag_IMUFlowFreq);
	//
	// FIXME: https://github.com/TSKangetsu/RPiSingleAPM/issues/89
	// if (TF._Tmp_IMUAttThreadDT <= 0)
	// 	TF._Tmp_IMUAttThreadDT = TF._flag_IMUThreadTimeMax;
	// PID Checking
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
		// NotTakeoff Detect
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
		// Leveling PID MIX
		{
			// IMU SaftyChecking---------------------------------------------------------//
			if (SF._uORB_MPU_Data._uORB_Real_Pitch > 70.0 || SF._uORB_MPU_Data._uORB_Real_Pitch < -70.0 ||
				SF._uORB_MPU_Data._uORB_Real__Roll > 70.0 || SF._uORB_MPU_Data._uORB_Real__Roll < -70.0)
			{
				if (AF.AutoPilotMode == APModeINFO::AltHold ||
					AF.AutoPilotMode == APModeINFO::AutoStable ||
					AF.AutoPilotMode == APModeINFO::SpeedHold ||
					AF.AutoPilotMode == APModeINFO::PositionHold ||
					AF.AutoPilotMode == APModeINFO::UserAuto)
				{
					AF.AngleLimit_Out_Clocking += TF.IMUFlow->TimeDT;
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
															   PF._flag_Filter_AngleRate_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));
			else
				PF._uORB_PID_AngleRate__Roll = SF._uORB_MPU_Data._uORB_Real__Roll;
			//
			if (PF._flag_Filter_AngleRate_CutOff != 0)
				PF._uORB_PID_AngleRate_Pitch = pt1FilterApply4(&DF.AngleRateLPF[1], SF._uORB_MPU_Data._uORB_Real_Pitch,
															   PF._flag_Filter_AngleRate_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));
			else
				PF._uORB_PID_AngleRate_Pitch = SF._uORB_MPU_Data._uORB_Real_Pitch;
			//
			if (SF._flag_Filter_GYaw_CutOff != 0)
				PF._uORB_PID_GYaw_Output = pt1FilterApply4(&DF.AngleRateLPF[2], SF._uORB_MPU_Data._uORB_Gryo___Yaw,
														   SF._flag_Filter_GYaw_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));
			else
				PF._uORB_PID_GYaw_Output = SF._uORB_MPU_Data._uORB_Gryo___Yaw;
			//--------------------------------------------------------------------------//
			PF._uORB_PID__Roll_Input = 0;
			PF._uORB_PID_Pitch_Input = 0;
			//---MODE SWITCHER----------------------------------------------------------//
			if (AF.AutoPilotMode == APModeINFO::SpeedHold || AF.AutoPilotMode == APModeINFO::UserAuto)
			{
				// FIXME: CP:Pitch header revert , Now Posout is revert. Consider change accel x to revert?
				if (AF._flag_IsNAVAvalible)
				{
					PF._uORB_PID__Roll_Input += PF._uORB_PID_PosX_Output;
					PF._uORB_PID_Pitch_Input -= PF._uORB_PID_PosY_Output;
				}
				else
				{
					PF._uORB_PID__Roll_Input -= RF._uORB_RC_Out__Roll * PF._flag_PID_RCAngle__Roll_Gain;
					PF._uORB_PID_Pitch_Input -= RF._uORB_RC_Out_Pitch * PF._flag_PID_RCAngle_Pitch_Gain;
				}
			}

			else if (AF.AutoPilotMode == APModeINFO::PositionHold)
			{
				// FIXME: force reapply when navigation no avaliable
				// FIXME: change output to same as RC out
				// FIXME: CP:Pitch header revert , Now Posout is revert. Consider change accel x to revert?
				if (AF._flag_IsNAVAvalible)
				{
					// if (RF._uORB_RC_Out__Roll != 0 && !AF._flag_IsBrakingXSet)
					if (RF._uORB_RC_Out__Roll != 0)
						PF._uORB_PID__Roll_Input -= RF._uORB_RC_Out__Roll * PF._flag_PID_RCAngle__Roll_Gain;
					else
						PF._uORB_PID__Roll_Input += PF._uORB_PID_PosX_Output;
					// if (RF._uORB_RC_Out_Pitch != 0 && !AF._flag_IsBrakingYSet)
					if (RF._uORB_RC_Out_Pitch != 0)
						PF._uORB_PID_Pitch_Input -= RF._uORB_RC_Out_Pitch * PF._flag_PID_RCAngle_Pitch_Gain;
					else
						PF._uORB_PID_Pitch_Input -= PF._uORB_PID_PosY_Output;
				}
				else
				{
					PF._uORB_PID__Roll_Input -= RF._uORB_RC_Out__Roll * PF._flag_PID_RCAngle__Roll_Gain;
					PF._uORB_PID_Pitch_Input -= RF._uORB_RC_Out_Pitch * PF._flag_PID_RCAngle_Pitch_Gain;
				}
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
			// TPA caculate
			if (EF._uORB_Total_Throttle > PF._flag_PID_TPA_BreakPoint)
			{
				PF._uORB_PID_TPA_Beta = 1.f - ((1.f - PF._flag_PID_TPA_Trust) *
											   ((EF._uORB_Total_Throttle - PF._flag_PID_TPA_BreakPoint) /
												((RF._flag_RC_Max_PWM_Value - 200.f) - PF._flag_PID_TPA_BreakPoint)));
			}
			else
				PF._uORB_PID_TPA_Beta = 1.f;
			//--------------------------------------------------------------------------//
			float ROLLDInput = SF._uORB_MPU_Data._uORB_Gryo__Roll - PF._uORB_PID_D_Last_Value__Roll;
			PF._uORB_PID_D_Last_Value__Roll = SF._uORB_MPU_Data._uORB_Gryo__Roll;
			float ROLLITERM = PF._uORB_PID__Roll_Input;
			float ROLLDTERM = ROLLDInput;
			if (PF._flag_Filter_PID_I_CutOff)
				ROLLITERM = pt1FilterApply4(&DF.ItermFilterRoll, PF._uORB_PID__Roll_Input, PF._flag_Filter_PID_I_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));
			if (PF._flag_Filter_PID_D_ST1_CutOff)
				ROLLDTERM = pt1FilterApply4(&DF.DtermFilterRoll, ROLLDInput, PF._flag_Filter_PID_D_ST1_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));
			if (PF._flag_Filter_PID_D_ST2_CutOff)
				ROLLDTERM = pt1FilterApply4(&DF.DtermFilterRollST2, ROLLDTERM, PF._flag_Filter_PID_D_ST2_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));
			PID_CaculateHyper((PF._uORB_PID__Roll_Input),
							  (ROLLITERM * (TF._uORB_IMUAttThreadDT / PID_DT_DEFAULT)),
							  (ROLLDTERM / (TF._uORB_IMUAttThreadDT / PID_DT_DEFAULT)),
							  PF._uORB_Leveling__Roll, PF._uORB_PID_I_Last_Value__Roll, PF._uORB_PID_D_Last_Value__Roll,
							  (PF._flag_PID_P__Roll_Gain * PF._uORB_PID_TPA_Beta),
							  (PF._flag_PID_I__Roll_Gain * PF._uORB_PID_I_Dynamic_Gain),
							  (PF._flag_PID_D__Roll_Gain * PF._uORB_PID_TPA_Beta),
							  PF._flag_PID_I__Roll_Max__Value);
			if (PF._uORB_Leveling__Roll > PF._flag_PID_Level_Max)
				PF._uORB_Leveling__Roll = PF._flag_PID_Level_Max;
			if (PF._uORB_Leveling__Roll < PF._flag_PID_Level_Max * -1)
				PF._uORB_Leveling__Roll = PF._flag_PID_Level_Max * -1;

			// Pitch PID Mix
			float PITCHDInput = SF._uORB_MPU_Data._uORB_Gryo_Pitch - PF._uORB_PID_D_Last_Value_Pitch;
			PF._uORB_PID_D_Last_Value_Pitch = SF._uORB_MPU_Data._uORB_Gryo_Pitch;
			float PITCHITERM = PF._uORB_PID_Pitch_Input;
			float PITCHDTERM = PITCHDInput;
			if (PF._flag_Filter_PID_I_CutOff)
				PITCHITERM = pt1FilterApply4(&DF.ItermFilterPitch, PF._uORB_PID_Pitch_Input, PF._flag_Filter_PID_I_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));
			if (PF._flag_Filter_PID_D_ST1_CutOff)
				PITCHDTERM = pt1FilterApply4(&DF.DtermFilterPitch, PITCHDInput, PF._flag_Filter_PID_D_ST1_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));
			if (PF._flag_Filter_PID_D_ST2_CutOff)
				PITCHDTERM = pt1FilterApply4(&DF.DtermFilterPitchST2, PITCHDTERM, PF._flag_Filter_PID_D_ST2_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));
			PID_CaculateHyper((PF._uORB_PID_Pitch_Input),
							  (PITCHITERM * (TF._uORB_IMUAttThreadDT / PID_DT_DEFAULT)),
							  (PITCHDTERM / (TF._uORB_IMUAttThreadDT / PID_DT_DEFAULT)),
							  PF._uORB_Leveling_Pitch, PF._uORB_PID_I_Last_Value_Pitch, PF._uORB_PID_D_Last_Value_Pitch,
							  (PF._flag_PID_P_Pitch_Gain * PF._uORB_PID_TPA_Beta),
							  (PF._flag_PID_I_Pitch_Gain * PF._uORB_PID_I_Dynamic_Gain),
							  (PF._flag_PID_D_Pitch_Gain * PF._uORB_PID_TPA_Beta),
							  PF._flag_PID_I_Pitch_Max__Value);
			if (PF._uORB_Leveling_Pitch > PF._flag_PID_Level_Max)
				PF._uORB_Leveling_Pitch = PF._flag_PID_Level_Max;
			if (PF._uORB_Leveling_Pitch < PF._flag_PID_Level_Max * -1)
				PF._uORB_Leveling_Pitch = PF._flag_PID_Level_Max * -1;

			// Yaw PID Mix
			// FIXME: avoid blackbox catch reversing
			PID_CaculateExtend((((PF._uORB_PID_GYaw_Output + RF._uORB_RC_Out___Yaw) / 15.f) * PF._flag_PID_AngleRate___Yaw_Gain) * EF._flag_YAWOut_Reverse,
							   ((((PF._uORB_PID_GYaw_Output + RF._uORB_RC_Out___Yaw) / 15.f) * PF._flag_PID_AngleRate___Yaw_Gain) * (TF._uORB_IMUAttThreadDT / PID_DT_DEFAULT)) * EF._flag_YAWOut_Reverse,
							   ((((PF._uORB_PID_GYaw_Output) / 15.f) * PF._flag_PID_AngleRate___Yaw_Gain) / (TF._uORB_IMUAttThreadDT / PID_DT_DEFAULT)) * EF._flag_YAWOut_Reverse,
							   PF._uORB_Leveling___Yaw, PF._uORB_PID_I_Last_Value___Yaw, PF._uORB_PID_D_Last_Value___Yaw,
							   PF._flag_PID_P___Yaw_Gain, (PF._flag_PID_I___Yaw_Gain * PF._uORB_PID_I_Dynamic_Gain), PF._flag_PID_D___Yaw_Gain, PF._flag_PID_I___Yaw_Max__Value);

			if (PF._uORB_Leveling___Yaw > PF._flag_PID_Level_Max)
				PF._uORB_Leveling___Yaw = PF._flag_PID_Level_Max;
			if (PF._uORB_Leveling___Yaw < PF._flag_PID_Level_Max * -1)
				PF._uORB_Leveling___Yaw = PF._flag_PID_Level_Max * -1;
			// PF._uORB_Leveling___Yaw *= EF._flag_YAWOut_Reverse; // FIXME: avoid blackbox catch it
		}
	};
	// reset output before caculate
	{
		EF._Tmp_B1_Speed = 0;
		EF._Tmp_A1_Speed = 0;
		EF._Tmp_A2_Speed = 0;
		EF._Tmp_B2_Speed = 0;
		EF._uORB_ESC_RPY_Max = 0;
		EF._uORB_ESC_RPY_Min = 0;
		// ESC Caculate
		if (AF.AutoPilotMode == APModeINFO::AltHold ||
			AF.AutoPilotMode == APModeINFO::PositionHold ||
			AF.AutoPilotMode == APModeINFO::SpeedHold ||
			AF.AutoPilotMode == APModeINFO::UserAuto)
			EF._uORB_Total_Throttle = PF._flag_PID_Hover_Throttle + PF._uORB_PID_Alt_Throttle;
		else
			EF._uORB_Total_Throttle = RF._uORB_RC_Out_Throttle;

		if (EF._uORB_Total_Throttle > RF._flag_RC_Max_PWM_Value - 200.f)
			EF._uORB_Total_Throttle = RF._flag_RC_Max_PWM_Value - 200.f;

		EF._Tmp_B1_Speed = -PF._uORB_Leveling__Roll - PF._uORB_Leveling_Pitch + PF._uORB_Leveling___Yaw;
		EF._Tmp_A1_Speed = -PF._uORB_Leveling__Roll + PF._uORB_Leveling_Pitch - PF._uORB_Leveling___Yaw;
		EF._Tmp_A2_Speed = +PF._uORB_Leveling__Roll + PF._uORB_Leveling_Pitch + PF._uORB_Leveling___Yaw;
		EF._Tmp_B2_Speed = +PF._uORB_Leveling__Roll - PF._uORB_Leveling_Pitch - PF._uORB_Leveling___Yaw;

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
	// FIXME: https://github.com/TSKangetsu/RPiSingleAPM/issues/89
	// if (TF._Tmp_IMUNavThreadDT <= 0)
	// 	TF._Tmp_IMUNavThreadDT = 1.f / (float)ACCEL_UPDATE_HZ * 1000000.f;

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

	if ((AF._flag_IsFlowAvalible && AF._flag_IsSonarAvalible) || !AF._flag_GPS_Error)
		AF._flag_IsNAVAvalible = true;
	else
		AF._flag_IsNAVAvalible = false;

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
	// FIXME: CP:Pitch header revert , Now Posout is revert. Consider change accel y to revert?
	SF._uORB_Gryo_Body_Asix_X -= ((float)SF._uORB_MPU_Data._uORB_Gryo_Pitch) * (TF._Tmp_IMUNavThreadDT / 1000000.f);
	SF._uORB_Gryo_Body_Asix_Y += ((float)SF._uORB_MPU_Data._uORB_Gryo__Roll) * (TF._Tmp_IMUNavThreadDT / 1000000.f);
	// POS Controller Reseter
	{
		if (!(AF.AutoPilotMode == APModeINFO::AltHold || AF.AutoPilotMode == APModeINFO::PositionHold ||
			  AF.AutoPilotMode == APModeINFO::SpeedHold || AF.AutoPilotMode == APModeINFO::UserAuto) ||
			AF._flag_ESC_ARMED)
		{
			PF._uORB_PID_Alt_Throttle = 0;
			// PF._uORB_PID_I_Last_Value_SpeedZ = 0;
			// PF._uORB_PID_D_Last_Value_SpeedZ = 0;
			PF._uORB_PID_AltHold_Target = SF._uORB_True_Movement_Z;
		}
		//
		if (!(AF.AutoPilotMode == APModeINFO::PositionHold || AF.AutoPilotMode == APModeINFO::SpeedHold || AF.AutoPilotMode == APModeINFO::UserAuto) ||
			AF._flag_ESC_ARMED)
		{
			// FIXME: speed XY init too slow
			SF._uORB_True_Speed_X = 0;
			SF._uORB_True_Speed_Y = 0;
			//
			SF._uORB_Flow_XOutput_Total = 0;
			SF._uORB_Flow_YOutput_Total = 0;
			SF._uORB_GPS_Hold_Lat = SF._uORB_GPS_COR_Lat;
			SF._uORB_GPS_Hold_Lng = SF._uORB_GPS_COR_Lng;
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
	// Flag Controller
	{
		if (AF._flag_ESC_ARMED)
		{
			AF._flag_IsINUHDisable = true;
			PF._uORB_PID_Sonar_GroundOffset = 0 - SF._uORB_BARO_Altitude;
			PF._uORB_PID_AccelX_Bias = 0;
			PF._uORB_PID_AccelY_Bias = 0;
			PF._uORB_PID_AccelZ_Bias = 0;
			PF._uORB_PID_Sonar_GroundValid = true;
			PF._uORB_PID_Sonar_GroundTimeOut = GetTimestamp() + 250000;
		}
		else
		{
			AF._flag_IsINUHDisable = false;
			if (SF._uORB_True_Movement_Z > 15)
			{
				if (GetTimestamp() > PF._uORB_PID_Sonar_GroundTimeOut)
				{
					PF._uORB_PID_Sonar_GroundValid = false;
				}
			}
		}
		//
		if (AF._flag_IsINUHDisable)
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
	// AutoTakeOff Function
	{
		if (!AF._flag_IsAutoTakeoffRequire)
		{
			PF._uORB_PID_Alt_Accel_Max = PF._flag_PID_Alt_Accel_Max;
			PF._uORB_PID_Alt_Speed_Max = PF._flag_PID_Alt_Speed_Max;
			// Vertical SpeedControll
			if (RF._uORB_RC_Out_AltHoldSpeed != 0 || PF._uORB_PID_PosZUserSpeed != 0)
			{
				PF._uORB_PID_AltHold_Target = SF._uORB_True_Movement_Z;
			}
		}
		else if (AF._flag_IsAutoTakeoffRequire)
		{
			// TODO: suit to config outside
			PF._uORB_PID_AltHold_Target = PF._flag_PID_Takeoff_Altitude;
			PF._uORB_PID_Alt_Accel_Max = PF._flag_PID_TakeOff_Accel_Max;
			PF._uORB_PID_Alt_Speed_Max = PF._flag_PID_TakeOff_Speed_Max;
			if (SF._uORB_True_Movement_Z > PF._flag_PID_Takeoff_Altitude)
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
	// AltHold Caculate
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
		// Air cushion effect
		bool isAirCushionEffectDetected = (PF._uORB_PID_Sonar_GroundValid && (PF._uORB_PID_BARO_AltInput < -PF._uORB_PID_Sonar_GroundOffset));
		PF._uORB_PID_BARO_AltInput = (isAirCushionEffectDetected ? 0 : PF._uORB_PID_BARO_AltInput);
		//===============================================//
		// FIXME: Find a way to balance Sonar and Baro Altitude
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
		PF._uORB_PID_InputTarget = TargetSpeed - SF._uORB_True_Speed_Z;
		// double TargetAccel = (TargetSpeed - SF._uORB_True_Speed_Z) * PF._flag_PID_I_Alt_Gain;
		// TargetAccel = TargetAccel > PF._uORB_PID_Alt_Accel_Max ? PF._uORB_PID_Alt_Accel_Max : TargetAccel;
		// TargetAccel = TargetAccel < -1 * PF._uORB_PID_Alt_Accel_Max ? -1 * PF._uORB_PID_Alt_Accel_Max : TargetAccel;
		// PF._uORB_PID_InputTarget = TargetAccel - SF._uORB_MPU_Data._uORB_Acceleration_Z;
		//

		if (AF.AutoPilotMode == APModeINFO::AltHold || AF.AutoPilotMode == APModeINFO::PositionHold ||
			AF.AutoPilotMode == APModeINFO::SpeedHold || AF.AutoPilotMode == APModeINFO::UserAuto)
		{
			PID_CaculateExtend(PF._uORB_PID_InputTarget, PF._uORB_PID_InputTarget, PF._uORB_PID_InputTarget,
							   PF._uORB_PID_Alt_Throttle, PF._uORB_PID_I_Last_Value_SpeedZ, PF._uORB_PID_D_Last_Value_SpeedZ,
							   PF._flag_PID_P_SpeedZ_Gain, PF._flag_PID_I_SpeedZ_Gain / 100.f, PF._flag_PID_D_SpeedZ_Gain,
							   PF._flag_PID_Alt_Level_Max);
			PF._uORB_PID_Alt_Throttle = pt1FilterApply4(&DF.ThrottleLPF, PF._uORB_PID_Alt_Throttle, FILTERTHROTTLELPFCUTOFF, (TF._Tmp_IMUNavThreadDT / 1000000.f));
			std::cout << "_uORB_RC_Out_AltHoldSpeed:" << PF._uORB_PID_Alt_Throttle
					  << " \n";
			std::cout << "\033[1A";
			std::cout << "\033[K";
			PF._uORB_PID_Alt_Throttle = PF._uORB_PID_Alt_Throttle > PF._flag_PID_Alt_Level_Max ? PF._flag_PID_Alt_Level_Max : PF._uORB_PID_Alt_Throttle;
			PF._uORB_PID_Alt_Throttle = PF._uORB_PID_Alt_Throttle < -1 * PF._flag_PID_Alt_Level_Max ? -1 * PF._flag_PID_Alt_Level_Max : PF._uORB_PID_Alt_Throttle;
		}
	}
	// PositionHold Caculate
	{
		// Speed Brake Flag Manager
		// FIXME: Any better way?
		{
			if (AF.AutoPilotMode == APModeINFO::SpeedHold)
			{
				// Disable UserAuto Mode input
				PF._uORB_PID_PosXUserSpeed = 0;
				PF._uORB_PID_PosYUserSpeed = 0;
				PF._uORB_PID_PosXUserTarget = 0;
				PF._uORB_PID_PosYUserTarget = 0;
				//
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
				// Disable UserAuto Mode input and speedhold input
				PF._uORB_PID_PosXUserSpeed = 0;
				PF._uORB_PID_PosYUserSpeed = 0;
				PF._uORB_PID_PosXUserTarget = 0;
				PF._uORB_PID_PosYUserTarget = 0;
				RF._uORB_RC_Out_PosHoldSpeedX = 0;
				RF._uORB_RC_Out_PosHoldSpeedY = 0;
				//
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
			// Apply movement change
			if (AF._flag_IsPositionXChange || AF._flag_IsBrakingXSet)
			{
				SF._uORB_True_Movement_X = 0;
				SF._uORB_Flow_YOutput_Total = 0;
				SF._uORB_GPS_Hold_Lat = SF._uORB_GPS_COR_Lat;
				// TODO: apply adding Y when X is move, because global position has angle
				SF._uORB_GPS_Hold_Lng = SF._uORB_GPS_COR_Lng;
				AF._flag_IsPositionXChange = false;
				if (-10.f < SF._uORB_True_Speed_X && SF._uORB_True_Speed_X < 10.f)
					AF._flag_IsBrakingXSet = false;
			}
			if (AF._flag_IsPositionYChange || AF._flag_IsBrakingYSet)
			{
				SF._uORB_True_Movement_Y = 0;
				SF._uORB_Flow_XOutput_Total = 0;
				SF._uORB_GPS_Hold_Lng = SF._uORB_GPS_COR_Lng;
				// TODO: apply adding Y when X is move, because global position has angle
				SF._uORB_GPS_Hold_Lat = SF._uORB_GPS_COR_Lat;
				AF._flag_IsPositionYChange = false;
				if (-10.f < SF._uORB_True_Speed_Y && SF._uORB_True_Speed_Y < 10.f)
					AF._flag_IsBrakingYSet = false;
			}
		}

		// Static position input
		if (AF._flag_FlowData_Async)
		{
			PF._uORB_PID_Flow_PosInput_X = SF._uORB_Flow_YOutput_Total;
			PF._uORB_PID_Flow_PosInput_Y = SF._uORB_Flow_XOutput_Total;
			AF._flag_FlowData_Async = false;
		}
		if (AF._flag_GPSData_Async)
		{
			PF._uORB_PID_GPS_PosInput_X = SF._uORB_GPS_Real_X;
			PF._uORB_PID_GPS_PosInput_Y = SF._uORB_GPS_Real_Y;
			if (!AF._flag_GPS_Error)
			{
				PF._uORB_PID_Flow_PosInput_X = 0;
				PF._uORB_PID_Flow_PosInput_Y = 0;
			}
			AF._flag_GPSData_Async = false;
		}
		// Fast Update GPS
		SF._uORB_GPS_Speed_XF = pt1FilterApply4(&DF.GPSSpeedLPF[0], SF._uORB_GPS_Speed_X, 1.f, (TF._Tmp_IMUNavThreadDT / 1000000.f / 8.f));
		SF._uORB_GPS_Speed_YF = pt1FilterApply4(&DF.GPSSpeedLPF[1], SF._uORB_GPS_Speed_Y, 1.f, (TF._Tmp_IMUNavThreadDT / 1000000.f / 8.f));
		//
		PF._uORB_PID_MoveXCorrection = 0;
		PF._uORB_PID_SpeedXCorrection = 0;
		PF._uORB_PID_MoveYCorrection = 0;
		PF._uORB_PID_SpeedYCorrection = 0;

		if ((AF._flag_IsFlowAvalible && AF._flag_IsSonarAvalible) ||
			!AF._flag_GPS_Error)
		{
			if (!AF._flag_GPS_Error)
			{
				PF._uORB_PID_MoveXCorrection += (PF._uORB_PID_GPS_PosInput_X - SF._uORB_True_Movement_X) *
												PF._flag_GPS_Dynamic_Beta *
												(TF._Tmp_IMUNavThreadDT / 1000000.f);
				PF._uORB_PID_SpeedXCorrection += (SF._uORB_GPS_Speed_XF - SF._uORB_True_Speed_X) *
												 (PF._flag_GPS_Dynamic_Beta / 1.15f) *
												 (TF._Tmp_IMUNavThreadDT / 1000000.f);
				PF._uORB_PID_AccelX_Bias -= (SF._uORB_GPS_Speed_XF - SF._uORB_True_Speed_X) *
											(PF._flag_GPS_Dynamic_Beta / 1.15f) * PF._uORB_AccelBias_Beta *
											(TF._Tmp_IMUNavThreadDT / 1000000.f);

				PF._uORB_PID_MoveYCorrection += (PF._uORB_PID_GPS_PosInput_Y - SF._uORB_True_Movement_Y) *
												PF._flag_GPS_Dynamic_Beta *
												(TF._Tmp_IMUNavThreadDT / 1000000.f);
				PF._uORB_PID_SpeedYCorrection += (SF._uORB_GPS_Speed_YF - SF._uORB_True_Speed_Y) *
												 (PF._flag_GPS_Dynamic_Beta / 1.15f) *
												 (TF._Tmp_IMUNavThreadDT / 1000000.f);
				PF._uORB_PID_AccelY_Bias -= (SF._uORB_GPS_Speed_YF - SF._uORB_True_Speed_Y) *
											(PF._flag_GPS_Dynamic_Beta / 1.15f) * PF._uORB_AccelBias_Beta *
											(TF._Tmp_IMUNavThreadDT / 1000000.f);
			}

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
		// TODO: new BrakingMode Implement
		{
			// if (AF._flag_IsBrakingXSet && abs(SF._uORB_True_Speed_X) > 45.f)
			// {
			// }
			// if (AF._flag_IsBrakingYSet && abs(SF._uORB_True_Speed_Y) > 45.f)
			// {
			// }
		}
		//
		double TargetSpeedX = (SF._uORB_True_Movement_X - PF._uORB_PID_PosXUserTarget) * PF._flag_PID_P_PosX_Gain -
							  RF._uORB_RC_Out_PosHoldSpeedX - PF._uORB_PID_PosXUserSpeed;
		TargetSpeedX = TargetSpeedX > PF._flag_PID_Pos_Speed_Max ? PF._flag_PID_Pos_Speed_Max : TargetSpeedX;
		TargetSpeedX = TargetSpeedX < -1 * PF._flag_PID_Pos_Speed_Max ? -1 * PF._flag_PID_Pos_Speed_Max : TargetSpeedX;
		PF._uORB_PID_PosXTarget = (TargetSpeedX + SF._uORB_True_Speed_X);
		//
		double TargetSpeedY = (SF._uORB_True_Movement_Y - PF._uORB_PID_PosYUserTarget) * PF._flag_PID_P_PosY_Gain -
							  RF._uORB_RC_Out_PosHoldSpeedY - PF._uORB_PID_PosYUserSpeed;
		TargetSpeedY = TargetSpeedY > PF._flag_PID_Pos_Speed_Max ? PF._flag_PID_Pos_Speed_Max : TargetSpeedY;
		TargetSpeedY = TargetSpeedY < -1 * PF._flag_PID_Pos_Speed_Max ? -1 * PF._flag_PID_Pos_Speed_Max : TargetSpeedY;
		PF._uORB_PID_PosYTarget = (TargetSpeedY + SF._uORB_True_Speed_Y);
		//
		if ((AF.AutoPilotMode == APModeINFO::PositionHold || AF.AutoPilotMode == APModeINFO::SpeedHold ||
			 AF.AutoPilotMode == APModeINFO::UserAuto) &&
			AF._flag_IsNAVAvalible)
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

			PF._uORB_PID_PosX_Output = PF._uORB_PID_PosX_Output > PF._flag_PID_Pos_Level_Max ? PF._flag_PID_Pos_Level_Max : PF._uORB_PID_PosX_Output;
			PF._uORB_PID_PosX_Output = PF._uORB_PID_PosX_Output < -1 * PF._flag_PID_Pos_Level_Max ? -1 * PF._flag_PID_Pos_Level_Max : PF._uORB_PID_PosX_Output;

			// PF._uORB_PID_PosX_Output = pt1FilterApply4(&DF.POSOutLPF[0], PF._uORB_PID_PosX_Output, FILTERPOSOUTLPFCUTOFF, (TF._Tmp_IMUNavThreadDT / 1000000.f));
			// PF._uORB_PID_PosY_Output = pt1FilterApply4(&DF.POSOutLPF[1], PF._uORB_PID_PosY_Output, FILTERPOSOUTLPFCUTOFF, (TF._Tmp_IMUNavThreadDT / 1000000.f));
		}
	}

	TF._Tmp_IMUNavThreadLast = GetTimestamp();
}

void SingleAPMAPI::RPiSingleAPM::SaftyCheck()
{
	// SIG System Signal detect
	if (SystemSignal == SIGTERM || SystemSignal == SIGINT)
		RPiSingleAPMDeInit();
	// Error ARMED
	if (AF._flag_Error == true)
	{
		AF._flag_ESC_ARMED = true;
	}
	// PreARM Checking
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

	// std::cout << "_uORB_PID_I_Last_Value_SpeedZ:" << PF._uORB_PID_I_Last_Value_SpeedZ
	// 		  << " \n";
	// std::cout << "_uORB_PID_D_Last_Value_SpeedZ:" << PF._uORB_PID_D_Last_Value_SpeedZ
	// 		  << " \n";
	// std::cout << "_uORB_PID_Sonar_GroundOffset:" << PF._uORB_PID_Sonar_GroundOffset
	// 		  << " \n";
	// std::cout << "_uORB_PID_InputTarget:" << PF._uORB_PID_InputTarget
	//   << " \n";
	// std::cout << "_uORB_RC_Out_AltHoldSpeed:" << RF._uORB_RC_Out_AltHoldSpeed
	// 		  << " \n";
	// std::cout << "\033[2A";
	// std::cout << "\033[K";
	// std::cout << "\033[200A";
	// std::cout << "\033[K";
	// std::cout << "FlyMode:\n";
	// if (AF.AutoPilotMode == APModeINFO::AutoStable)
	// {
	// 	std::cout << " AutoStable        ";
	// }
	// else if (AF.AutoPilotMode == APModeINFO::AltHold)
	// {
	// 	std::cout << " AltHold           ";
	// }
	// else if (AF.AutoPilotMode == APModeINFO::PositionHold)
	// {
	// 	std::cout << " PositionHold      ";
	// }
	// else if (AF.AutoPilotMode == APModeINFO::SpeedHold)
	// {
	// 	std::cout << " SpeedHold         ";
	// }
	// else if (AF.AutoPilotMode == APModeINFO::RateHold)
	// {
	// 	std::cout << " RateHold        ";
	// }
	// else if (AF.AutoPilotMode == APModeINFO::UserAuto)
	// {
	// 	std::cout << " UserAuto          ";
	// }

	// std::cout << "\n";
	// std::cout << "ESCSpeedOutput:"
	// 		  << " \n";
	// std::cout << " A1 " << EF._uORB_A1_Speed << "    "
	// 		  << " A2 " << EF._uORB_A2_Speed << "                        "
	// 		  << "\n";
	// std::cout << " B1 " << EF._uORB_B1_Speed << "    "
	// 		  << " B2 " << EF._uORB_B2_Speed << "                        "
	// 		  << "\n";

	// std::cout << "IMUSenorData: "
	// 		  << " \n";
	// std::cout << " GryoPitch:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Gryo_Pitch << "    "
	// 		  << " GryoRoll:    " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Gryo__Roll << "    "
	// 		  << " GryoYaw:     " << std::setw(7) << std::setfill(' ') << (int)PF._uORB_PID_GYaw_Output << "    "
	// 		  << "                        "
	// 		  << std::endl;
	// std::cout << " RealPitch:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Real_Pitch << "    "
	// 		  << " RealRoll:    " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Real__Roll << "    "
	// 		  << " RealYaw :    " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Real___Yaw << "    "
	// 		  << std::endl;
	// std::cout << " AccePitch:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Accel_Pitch << "    "
	// 		  << " AcceRoll:    " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Accel__Roll << "    "
	// 		  << " AccelG  :    " << std::setw(7) << std::setfill(' ') << SF._uORB_MPU_Data._uORB_MPU9250_A_Vector << "    "
	// 		  << "                        "
	// 		  << std::endl;
	// std::cout << " AccelXG:     " << std::setw(7) << std::setfill(' ') << SF._uORB_MPU_Data._uORB_MPU9250_ADF_X << "    "
	// 		  << " AccelYG:     " << std::setw(7) << std::setfill(' ') << SF._uORB_MPU_Data._uORB_MPU9250_ADF_Y << "    "
	// 		  << " AccelZG:     " << std::setw(7) << std::setfill(' ') << SF._uORB_MPU_Data._uORB_MPU9250_ADF_Z << "    "
	// 		  << "                        "
	// 		  << std::endl;
	// std::cout << " VIBEX:       " << std::setw(7) << std::setfill(' ') << (int)(SF._uORB_MPU_Data._uORB_Accel_VIBE_X * 1000.f) << "    "
	// 		  << " VIBEY:       " << std::setw(7) << std::setfill(' ') << (int)(SF._uORB_MPU_Data._uORB_Accel_VIBE_X * 1000.f) << "    "
	// 		  << " VIBEZ:       " << std::setw(7) << std::setfill(' ') << (int)(SF._uORB_MPU_Data._uORB_Accel_VIBE_X * 1000.f) << "    "
	// 		  << "                        "
	// 		  << std::endl;
	// std::cout << " AccelClipped:" << std::setw(7) << std::setfill(' ') << SF._uORB_MPU_Data._uORB_MPU9250_ACC_Clipped << "    "
	// 		  << " AccelClCount:" << std::setw(7) << std::setfill(' ') << SF._uORB_Accel_Clipped_Count << "    "
	// 		  << " AccelDynamic:" << std::setw(7) << std::setfill(' ') << PF._uORB_Accel_Dynamic_Beta << "    "
	// 		  << "                        "
	// 		  << std::endl;
	// std::cout << " MAGYAW:      " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MAG_Yaw << "    "
	// 		  << " MAGSYAW:     " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MAG_StaticYaw << "    "
	// 		  << " NAVYAW:      " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_NAV_Yaw << "    "
	// 		  << " IMUTIME:     " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_MPU9250_IMUUpdateTime << "         \n"
	// 		  << " MAGRawX:     " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MAG_RawX << "    "
	// 		  << " MAGRawY:     " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MAG_RawY << "    "
	// 		  << " MAGRawZ:     " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MAG_RawZ << "    "
	// 		  << " MAGRawV:     " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MAG_Vector << "    "
	// 		  << std::endl;
	// std::cout << " AccelrationX:" << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Acceleration_X << "cms2"
	// 		  << " AccelrationY:" << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Acceleration_Y << "cms2"
	// 		  << " AccelrationZ:" << std::setw(7) << std::setfill(' ') << (int)SF._uORB_MPU_Data._uORB_Acceleration_Z << "cms2"
	// 		  << "                        "
	// 		  << std::endl;
	// std::cout << " SpeedX:      " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_True_Speed_X << "cms "
	// 		  << " SpeedY       " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_True_Speed_Y << "cms "
	// 		  << " SpeedZ:      " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_True_Speed_Z << "cms "
	// 		  << "                        "
	// 		  << std::endl;
	// std::cout << " MoveX:       " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_True_Movement_X << "cm  "
	// 		  << " MoveY        " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_True_Movement_Y << "cm  "
	// 		  << " MoveZ:       " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_True_Movement_Z << "cm  "
	// 		  << "                        "
	// 		  << std::endl;
	// std::cout
	// 	<< "BAROParseDataINFO:"
	// 	<< "\n";
	// std::cout << " ||FilterPressure:     " << std::setw(7) << std::setfill(' ') << std::setiosflags(std::ios::fixed) << std::setprecision(2)
	// 		  << SF._uORB_BARO_Data.PressureHPA << " hpa";
	// std::cout << " ||Altitude:           " << std::setw(7) << std::setfill(' ') << (int)PF._uORB_PID_BARO_AltInput << "  cm";
	// std::cout << " ||Temp:               " << std::setw(7) << std::setfill(' ') << std::setiosflags(std::ios::fixed) << std::setprecision(2)
	// 		  << (SF._uORB_BARO_Data.TemperatureC) << "   C";
	// std::cout << " ||AltHoldTarget:      " << std::setw(7) << std::setfill(' ') << (int)PF._uORB_PID_AltHold_Target << "  cm\n";
	// std::cout << " ||AltholdThrottle:    " << std::setw(7) << std::setfill(' ') << (int)PF._uORB_PID_Alt_Throttle << "    ";
	// std::cout << " ||TargetSpeed:        " << std::setw(7) << std::setfill(' ') << (int)PF._uORB_PID_InputTarget << "    \n";
	// std::cout
	// 	<< "FlowParseDataINFO:"
	// 	<< "\n";
	// std::cout << " FlowXOut:       " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Flow_XOutput << "    ";
	// std::cout << " ||FlowYOut:     " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Flow_YOutput << "    ";
	// std::cout << " ||FlowAltitude: " << std::setw(7) << std::setfill(' ') << (int)PF._uORB_PID_Sonar_AltInput << "    ";
	// std::cout << " ||FlowSpeedX:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Flow_Speed_X << "cm/s            \n";
	// std::cout << " FlowXOutTotal:  " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Flow_XOutput_Total << "    ";
	// std::cout << " ||FlowYOutTotal:" << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Flow_YOutput_Total << "    ";
	// std::cout << " ||FlowSpeed:    " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Flow_ClimbeRate << "cm/s";
	// std::cout << " ||FlowQuality:  " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Flow_Quality << "    ";
	// std::cout << " ||FlowSpeedY:   " << std::setw(7) << std::setfill(' ') << (int)SF._uORB_Flow_Speed_Y << "cm/s            \n";

	// std::cout << "GPSDataINFO:"
	// 		  << "\n";
	// std::cout << " GPSLAT:       " << std::setw(10) << std::setfill(' ') << (int)SF._uORB_GPS_COR_Lat
	// 		  << " ||GPSNE:          " << SF._uORB_GPS_Data.lat_North_Mode << " -> " << SF._uORB_GPS_Data.lat_East_Mode
	// 		  << " ||PosXOutput: " << std::setw(10) << std::setfill(' ') << PF._uORB_PID_PosX_Output
	// 		  << " ||GPSRealX:  " << std::setw(10) << std::setfill(' ') << SF._uORB_GPS_Real_X
	// 		  << " ||GPSSpeedX:  " << std::setw(10) << std::setfill(' ') << SF._uORB_GPS_Speed_XF
	// 		  << "            \n";
	// std::cout << " GPSLNG:       " << std::setw(10) << std::setfill(' ') << (int)SF._uORB_GPS_COR_Lng
	// 		  << " ||GPSSATCount:" << std::setw(10) << std::setfill(' ') << SF._uORB_GPS_Data.satillitesCount
	// 		  << " ||PosYOutput: " << std::setw(10) << std::setfill(' ') << PF._uORB_PID_PosY_Output
	// 		  << " ||GPSRealY:  " << std::setw(10) << std::setfill(' ') << SF._uORB_GPS_Real_Y
	// 		  << " ||GPSSpeedY:  " << std::setw(10) << std::setfill(' ') << SF._uORB_GPS_Speed_YF
	// 		  << "            \n";

	// std::cout << "RCOutPUTINFO:   "
	// 		  << "\n";
	// std::cout << " ChannelRoll  "
	// 		  << ": " << RF._uORB_RC_Out__Roll << std::setw(5) << std::setfill(' ') << " ||";
	// std::cout << " ChannelPitch "
	// 		  << ": " << RF._uORB_RC_Out_Pitch << std::setw(5) << std::setfill(' ') << " ||";
	// std::cout << " ChannelThrot "
	// 		  << ": " << RF._uORB_RC_Out_Throttle << std::setw(5) << std::setfill(' ') << " ||";
	// std::cout << " ChannelYaw   "
	// 		  << ": " << RF._uORB_RC_Out___Yaw << std::setw(5) << std::setfill(' ') << " \n";

	// std::cout << "ChannelINFO: "
	// 		  << " \n";
	// for (size_t i = 0; i < 16; i++)
	// {
	// 	std::cout << " " << RF._uORB_RC_Channel_PWM[i] << " ";
	// }
	// std::cout << "                            \n";

	// std::cout << "ADCINFO:     "
	// 		  << " \n";
	// std::cout << " voltage:" << std::setw(7) << std::setfill(' ') << std::setiosflags(std::ios::fixed) << std::setprecision(2) << SF._uORB_BAT_Voltage << " V";
	// std::cout << " ||sin_vol:" << std::setw(7) << std::setfill(' ') << std::setiosflags(std::ios::fixed) << std::setprecision(2) << SF._uORB_BAT_SingleVol << " V";
	// std::cout << " \n";

	// std::bitset<16> x(AF._flag_PreARM_Check_Level);
	// std::cout << " PreARMCheckInfo:    " << x << "                  \n";
	// std::bitset<16> s(AF._flag_FailedSafe_Level);
	// std::cout << " FailedSafekInfo:    " << s << "                  \n";

	// std::cout << " Flag_ESC_ARMED:         " << std::setw(3) << std::setfill(' ') << AF._flag_ESC_ARMED << " |";
	// std::cout << " Flag_Error:             " << std::setw(3) << std::setfill(' ') << AF._flag_Error << " |";
	// std::cout << " TakeOffing:             " << std::setw(3) << std::setfill(' ') << AF._flag_IsAutoTakeoffRequire << " |";
	// std::cout << " UserARMRequire:         " << std::setw(3) << std::setfill(' ') << AF._flag_ESC_DISARMED_Request << " |";
	// std::cout << " Flag_GPS_Error:         " << std::setw(3) << std::setfill(' ') << AF._flag_GPS_Error << "           \n";
	// std::cout << " Flag_RC_Disconnected:   " << std::setw(3) << std::setfill(' ') << AF._flag_RC_Disconnected << " |";
	// std::cout << " Flag_Flow_ErrorClock:   " << std::setw(3) << std::setfill(' ') << AF.Flow_Lose_Clocking << " |";
	// std::cout << " Flag_PreARMCheckPass:   " << std::setw(3) << std::setfill(' ') << AF._flag_PreARM_Check << " |";
	// std::cout << " Flag_GPS_Disconnected:  " << std::setw(3) << std::setfill(' ') << AF._flag_GPS_Disconnected << "         \n";
	// std::cout << " Flag_FakeRC_Error:      " << std::setw(3) << std::setfill(' ') << AF._flag_FakeRC_Error << " |";
	// std::cout << " Flag_FakeRC_Disconnect: " << std::setw(3) << std::setfill(' ') << AF.FakeRC_Lose_Clocking << " |";
	// std::cout << " Flag_MAG_Cali_Failed:   " << std::setw(3) << std::setfill(' ') << AF._flag_MAG_Cali_Failed << " |";
	// std::cout << " Flag_MPUCalibrating:    " << std::setw(3) << std::setfill(' ') << AF._flag_MPUCalibratingSet << " |";
	// std::cout << " Flag_IsNAVAvaliable:    " << std::setw(3) << std::setfill(' ') << AF._flag_IsNAVAvalible << "           \n";
	// std::cout << " Flag_IsFlowAvalible:    " << std::setw(3) << std::setfill(' ') << AF._flag_IsFlowAvalible << " |";
	// std::cout << " Flag_IsSonarAvalible:   " << std::setw(3) << std::setfill(' ') << AF._flag_IsSonarAvalible << " |";
	// std::cout << " GPS_Lose_Clocking:      " << std::setw(3) << std::setfill(' ') << AF.GPS_Lose_Clocking << " |";
	// std::cout << " Flag_RC_Error:          " << std::setw(3) << std::setfill(' ') << AF._flag_RC_Error << " |";
	// std::cout << " RC_Lose_Clocking:       " << std::setw(3) << std::setfill(' ') << AF.RC_Lose_Clocking << "                        \n\n";

	// std::cout << " IMUNAVDT:    " << std::setw(7) << std::setfill(' ') << TF._Tmp_IMUNavThreadDT << "    \n";
	// std::cout << " IMUATTDT:    " << std::setw(7) << std::setfill(' ') << TF._uORB_IMUAttThreadDT << "    \n";
	// std::cout << " IMU Freq:    " << std::setw(7) << std::setfill(' ') << (TF.IMUFlow ? TF.IMUFlow->RunClockHz : -1) << "    \n";
	// std::cout << " ESC Freq:    " << std::setw(7) << std::setfill(' ') << (TF.ESCFlow ? TF.ESCFlow->RunClockHz : -1) << "    \n";
	// std::cout << " RTX Freq:    " << std::setw(7) << std::setfill(' ') << (TF.RTXFlow ? TF.RTXFlow->RunClockHz : -1) << "    \n";
	// std::cout << " TEL Freq:    " << std::setw(7) << std::setfill(' ') << (TF.TELFlow ? TF.TELFlow->RunClockHz : -1) << "    \n";
	// std::cout << " ALT Freq:    " << std::setw(7) << std::setfill(' ') << (TF.ALTFlow ? TF.ALTFlow->RunClockHz : -1) << "    \n";
}

int SingleAPMAPI::RPiSingleAPM::GetTimestamp()
{
	struct timespec tv;
	clock_gettime(CLOCK_MONOTONIC, &tv);
	return (((tv.tv_sec * (uint64_t)1000000 + (tv.tv_nsec / 1000))) - TF._flag_SystemStartUp_Time);
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

void SingleAPMAPI::RPiSingleAPM::APMControllerServo(int pin, int PWMInUs)
{
	DF.I2CLock.lock();
	DF.ESCDevice->ESCUpdate(pin, PWMInUs);
	DF.I2CLock.unlock();
}