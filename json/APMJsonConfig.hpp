#pragma once
#include "Drive_Json.hpp"
#include "../src/SingleAPM.hpp"

using json = nlohmann::json;

namespace SingleAPMAPI
{
    void to_json(json &j, const SingleAPMAPI::APMSettinngs &p)
    {
        j = json{
            {
                "DeviceConfig",
                {
                    {"RC_Type", p.DC.RC_Type},
                    {"MPU9250_Type", p.DC.MPU9250_Type},
                    {"IMU_Freqeuncy", p.DC.IMU_Freqeuncy},
                    {"RXT_Freqeuncy", p.DC.RXT_Freqeuncy},
                    {"ESC_Freqeuncy", p.DC.ESC_Freqeuncy},
                    {"BBC_Freqeuncy", p.DC.BBC_Freqeuncy},
                    {"BBC_PInterval", p.DC.BBC_PInterval},
                    {"__RCDevice", p.DC.__RCDevice},
                    {"__GPSDevice", p.DC.__GPSDevice},
                    {"__FlowDevice", p.DC.__FlowDevice},
                    {"__MPUDeviceSPI", p.DC.__MPUDeviceSPI},
                    {"__I2CDevice", p.DC.__I2CDevice},
                    {"_IsGPSEnable", p.DC._IsGPSEnable},
                    {"_IsFlowEnable", p.DC._IsFlowEnable},
                    {"_IsRCSafeEnable", p.DC._IsRCSafeEnable},
                    {"_IsBAROEnable", p.DC._IsBAROEnable},
                    {"_IsBlackBoxEnable", p.DC._IsBlackBoxEnable},
                },
            },
            {
                "PIDConfig",
                {
                    {
                        "AltitudePID",
                        {
                            {"_flag_PID_Alt_Accel_Max", p.PC._flag_PID_Alt_Accel_Max},
                            {"_flag_PID_Alt_Level_Max", p.PC._flag_PID_Alt_Level_Max},
                            {"_flag_PID_Alt_Speed_Max", p.PC._flag_PID_Alt_Speed_Max},
                            {"_flag_PID_D_SpeedZ_Gain", p.PC._flag_PID_D_SpeedZ_Gain},
                            {"_flag_PID_Hover_Throttle", p.PC._flag_PID_Hover_Throttle},
                            {"_flag_PID_I_Alt_Gain", p.PC._flag_PID_I_Alt_Gain},
                            {"_flag_PID_I_SpeedZ_Gain", p.PC._flag_PID_I_SpeedZ_Gain},
                            {"_flag_PID_P_Alt_Gain", p.PC._flag_PID_P_Alt_Gain},
                            {"_flag_PID_P_SpeedZ_Gain", p.PC._flag_PID_P_SpeedZ_Gain},
                            {"_flag_PID_Takeoff_Altitude", p.PC._flag_PID_Takeoff_Altitude},
                        },
                    },
                    {
                        "AttitudePID",
                        {
                            {"_flag_PID_AngleRate_Pitch_Gain", p.PC._flag_PID_AngleRate_Pitch_Gain},
                            {"_flag_PID_AngleRate__Roll_Gain", p.PC._flag_PID_AngleRate__Roll_Gain},
                            {"_flag_PID_AngleRate___Yaw_Gain", p.PC._flag_PID_AngleRate___Yaw_Gain},
                            {"_flag_PID_D_Pitch_Gain", p.PC._flag_PID_D_Pitch_Gain},
                            {"_flag_PID_D__Roll_Gain", p.PC._flag_PID_D__Roll_Gain},
                            {"_flag_PID_D___Yaw_Gain", p.PC._flag_PID_D___Yaw_Gain},
                            {"_flag_PID_I_Pitch_Gain", p.PC._flag_PID_I_Pitch_Gain},
                            {"_flag_PID_I_Pitch_Max__Value", p.PC._flag_PID_I_Pitch_Max__Value},
                            {"_flag_PID_I__Roll_Gain", p.PC._flag_PID_I__Roll_Gain},
                            {"_flag_PID_I__Roll_Max__Value", p.PC._flag_PID_I__Roll_Max__Value},
                            {"_flag_PID_I___Yaw_Gain", p.PC._flag_PID_I___Yaw_Gain},
                            {"_flag_PID_I___Yaw_Max__Value", p.PC._flag_PID_I___Yaw_Max__Value},
                            {"_flag_PID_Level_Max", p.PC._flag_PID_Level_Max},
                            {"_flag_PID_P_Pitch_Gain", p.PC._flag_PID_P_Pitch_Gain},
                            {"_flag_PID_P__Roll_Gain", p.PC._flag_PID_P__Roll_Gain},
                            {"_flag_PID_P___Yaw_Gain", p.PC._flag_PID_P___Yaw_Gain},
                            {"_flag_PID_RCAngle_Pitch_Gain", p.PC._flag_PID_RCAngle_Pitch_Gain},
                            {"_flag_PID_RCAngle__Roll_Gain", p.PC._flag_PID_RCAngle__Roll_Gain},
                            {"_flag_PID_RCAngle___Yaw_Gain", p.PC._flag_PID_RCAngle___Yaw_Gain},
                            {"_flag_PID_RCRate_Pitch_Gain", p.PC._flag_PID_RCRate_Pitch_Gain},
                            {"_flag_PID_RCRate__Roll_Gain", p.PC._flag_PID_RCRate__Roll_Gain},
                            {"_flag_PID_RCRate___Yaw_Gain", p.PC._flag_PID_RCRate___Yaw_Gain},
                            {"_flag_PID_Rate_Limit", p.PC._flag_PID_Rate_Limit},
                            {"_flag_PID_TPA_BreakPoint", p.PC._flag_PID_TPA_BreakPoint},
                            {"_flag_PID_TPA_Trust", p.PC._flag_PID_TPA_Trust},
                        },
                    },
                    {
                        "NAVPID",
                        {
                            {"_flag_PID_D_SpeedX_Gain", p.PC._flag_PID_D_SpeedX_Gain},
                            {"_flag_PID_D_SpeedY_Gain", p.PC._flag_PID_D_SpeedY_Gain},
                            {"_flag_PID_I_PosX_Gain", p.PC._flag_PID_I_PosX_Gain},
                            {"_flag_PID_I_PosY_Gain", p.PC._flag_PID_I_PosY_Gain},
                            {"_flag_PID_I_SpeedX_Gain", p.PC._flag_PID_I_SpeedX_Gain},
                            {"_flag_PID_I_SpeedY_Gain", p.PC._flag_PID_I_SpeedY_Gain},
                            {"_flag_PID_P_PosX_Gain", p.PC._flag_PID_P_PosX_Gain},
                            {"_flag_PID_P_PosY_Gain", p.PC._flag_PID_P_PosY_Gain},
                            {"_flag_PID_P_SpeedX_Gain", p.PC._flag_PID_P_SpeedX_Gain},
                            {"_flag_PID_P_SpeedY_Gain", p.PC._flag_PID_P_SpeedY_Gain},
                            {"_flag_PID_PosMan_Speed_Max", p.PC._flag_PID_PosMan_Speed_Max},
                            {"_flag_PID_Pos_Level_Max", p.PC._flag_PID_Pos_Level_Max},
                            {"_flag_PID_Pos_Speed_Max", p.PC._flag_PID_Pos_Speed_Max},
                        },
                    },
                    {
                        "OutputConfig",
                        {
                            {"_flag_A1_Pin", p.OC._flag_A1_Pin},
                            {"_flag_A2_Pin", p.OC._flag_A2_Pin},
                            {"_flag_B1_Pin", p.OC._flag_B1_Pin},
                            {"_flag_B2_Pin", p.OC._flag_B2_Pin},
                            {"_flag_YAWOut_Reverse", p.OC._flag_YAWOut_Reverse},
                            {"_flag_ESC_Lazy_Per", p.OC._flag_ESC_Lazy_Per},
                            {"ESCPLFrequency", p.OC.ESCPLFrequency},
                            {"ESCControllerType", p.OC.ESCControllerType},
                        },
                    },
                },
            },
            {
                "SensorConfig",
                {
                    {"_flag_MPU_Flip__Roll", p.SC._flag_MPU_Flip__Roll},
                    {"_flag_MPU_Flip_Pitch", p.SC._flag_MPU_Flip_Pitch},
                    {"_flag_MPU_Flip___Yaw", p.SC._flag_MPU_Flip___Yaw},
                    {"_flag_Accel__Roll_Cali", p.SC._flag_Accel__Roll_Cali},
                    {"_flag_Accel_Pitch_Cali", p.SC._flag_Accel_Pitch_Cali},
                    {"_flag_MPU9250_A_X_Cali", p.SC._flag_MPU9250_A_X_Cali},
                    {"_flag_MPU9250_A_Y_Cali", p.SC._flag_MPU9250_A_Y_Cali},
                    {"_flag_MPU9250_A_Z_Cali", p.SC._flag_MPU9250_A_Z_Cali},
                    {"_flag_MPU9250_A_X_Scal", p.SC._flag_MPU9250_A_X_Scal},
                    {"_flag_MPU9250_A_Y_Scal", p.SC._flag_MPU9250_A_Y_Scal},
                    {"_flag_MPU9250_A_Z_Scal", p.SC._flag_MPU9250_A_Z_Scal},
                    {"_flag_COMPASS_X_Offset", p.SC._flag_COMPASS_X_Offset},
                    {"_flag_COMPASS_X_Scaler", p.SC._flag_COMPASS_X_Scaler},
                    {"_flag_COMPASS_Y_Offset", p.SC._flag_COMPASS_Y_Offset},
                    {"_flag_COMPASS_Y_Scaler", p.SC._flag_COMPASS_Y_Scaler},
                    {"_flag_COMPASS_Z_Offset", p.SC._flag_COMPASS_Z_Offset},
                    {"_flag_COMPASS_Z_Scaler", p.SC._flag_COMPASS_Z_Scaler},
                    {"_flag_COMPASS_V_Offset", p.SC._flag_COMPASS_V_Offset},
                    {"_flag_COMPASS_V_Scaler", p.SC._flag_COMPASS_V_Scaler},
                    {"_flag_COMPASS_Flip__Roll", p.SC._flag_COMPASS_Flip__Roll},
                    {"_flag_COMPASS_Flip_Pitch", p.SC._flag_COMPASS_Flip_Pitch},
                    {"_flag_COMPASS_Flip___Yaw", p.SC._flag_COMPASS_Flip___Yaw},
                    {"_flag_COMPASS_YAW_Offset", p.SC._flag_COMPASS_YAW_Offset},
                },
            },
            {
                "RCConfig",
                {
                    {"_flag_RC_Min_PWM_Value", p.RC._flag_RC_Min_PWM_Value},
                    {"_flag_RC_Mid_PWM_Value", p.RC._flag_RC_Mid_PWM_Value},
                    {"_flag_RC_Max_PWM_Value", p.RC._flag_RC_Max_PWM_Value},
                    {"_flag_RC_ARM_PWM_Value", p.RC._flag_RC_ARM_PWM_Value},
                    {"_flag_RC_ARM_PWM_Channel", p.RC._flag_RC_ARM_PWM_Channel},
                    {"_flag_RC_AP_RateHold_PWM_Value", p.RC._flag_RC_AP_RateHold_PWM_Value},
                    {"_flag_RC_AP_RateHold_PWM_Channel", p.RC._flag_RC_AP_RateHold_PWM_Channel},
                    {"_flag_RC_AP_AutoStable_PWM_Value", p.RC._flag_RC_AP_AutoStable_PWM_Value},
                    {"_flag_RC_AP_AutoStable_PWM_Channel", p.RC._flag_RC_AP_AutoStable_PWM_Channel},
                    {"_flag_RC_AP_AltHold_PWM_Value", p.RC._flag_RC_AP_AltHold_PWM_Value},
                    {"_flag_RC_AP_AltHold_PWM_Channel", p.RC._flag_RC_AP_AltHold_PWM_Channel},
                    {"_flag_RC_AP_PositionHold_PWM_Value", p.RC._flag_RC_AP_PositionHold_PWM_Value},
                    {"_flag_RC_AP_PositionHold_PWM_Channel", p.RC._flag_RC_AP_PositionHold_PWM_Channel},
                    {"_flag_RC_AP_SpeedHold_PWM_Value", p.RC._flag_RC_AP_SpeedHold_PWM_Value},
                    {"_flag_RC_AP_SpeedHold_PWM_Channel", p.RC._flag_RC_AP_SpeedHold_PWM_Channel},
                    {"_flag_RC_AP_UserAuto_PWM_Value", p.RC._flag_RC_AP_UserAuto_PWM_Value},
                    {"_flag_RC_AP_UserAuto_PWM_Channel", p.RC._flag_RC_AP_UserAuto_PWM_Channel},
                    {"_flag_RCIsReserv__Roll", p.RC._flag_RCIsReserv__Roll},
                    {"_flag_RCIsReserv_Pitch", p.RC._flag_RCIsReserv_Pitch},
                    {"_flag_RCIsReserv___Yaw", p.RC._flag_RCIsReserv___Yaw},
                },
            },
            {
                "FilterConfig",
                {
                    {"_flag_Filter_Gryo_Type", p.FC._flag_Filter_Gryo_Type},
                    {"_flag_Filter_GryoST2_Type", p.FC._flag_Filter_GryoST2_Type},
                    {"_flag_Filter_GYaw_CutOff", p.FC._flag_Filter_GYaw_CutOff},
                    {"_flag_Filter_Gryo_CutOff", p.FC._flag_Filter_Gryo_CutOff},
                    {"_flag_Filter_GryoST2_CutOff", p.FC._flag_Filter_GryoST2_CutOff},
                    {"_flag_Filter_Gryo_NotchFreq", p.FC._flag_Filter_Gryo_NotchFreq},
                    {"_flag_Filter_Gryo_NotchCutOff", p.FC._flag_Filter_Gryo_NotchCutOff},
                    {"_flag_Filter_Gryo_DynamicNotchRange", p.FC._flag_Filter_Gryo_DynamicNotchRange},
                    {"_flag_Filter_Gryo_DynamicNotchMinFreq", p.FC._flag_Filter_Gryo_DynamicNotchMinFreq},
                    {"_flag_Filter_Gryo_DynamicNotchEnable", p.FC._flag_Filter_Gryo_DynamicNotchEnable},
                    {"_flag_Filter_Accel_Type", p.FC._flag_Filter_Accel_Type},
                    {"_flag_Filter_Accel_CutOff", p.FC._flag_Filter_Accel_CutOff},
                    {"_flag_Filter_AngleMix_Alpha", p.FC._flag_Filter_AngleMix_Alpha},
                    {"_flag_Baro_Trust_Beta", p.FC._flag_Baro_Trust_Beta},
                    {"_flag_Accel_Trust_Beta", p.FC._flag_Accel_Trust_Beta},
                    {"_flag_Sonar_Trust_Beta", p.FC._flag_Sonar_Trust_Beta},
                    {"_flag_GPSAlt_Trust_Beta", p.FC._flag_GPSAlt_Trust_Beta},
                    {"_flag_AccelBias_Trust_Beta", p.FC._flag_AccelBias_Trust_Beta},
                    {"_flag_GPS_Config_Beta", p.FC._flag_GPS_Config_Beta},
                    {"_flag_Flow_Config_Beta", p.FC._flag_Flow_Config_Beta},
                    {"_flag_Braking_Speed_Gain", p.FC._flag_Braking_Speed_Gain},
                    {"_flag_Filter_RC_CutOff", p.FC._flag_Filter_RC_CutOff},
                    {"_flag_Filter_AngleRate_CutOff", p.FC._flag_Filter_AngleRate_CutOff},
                    {"_flag_Filter_PID_I_CutOff", p.FC._flag_Filter_PID_I_CutOff},
                    {"_flag_Filter_PID_D_ST1_CutOff", p.FC._flag_Filter_PID_D_ST1_CutOff},
                    {"_flag_Filter_PID_D_ST2_CutOff", p.FC._flag_Filter_PID_D_ST2_CutOff},
                },
            },
        };
    }
    void from_json(const json &j, SingleAPMAPI::APMSettinngs &p)
    {
        j.at("DeviceConfig").at("RC_Type").get_to(p.DC.RC_Type);
        j.at("DeviceConfig").at("MPU9250_Type").get_to(p.DC.MPU9250_Type);
        j.at("DeviceConfig").at("IMU_Freqeuncy").get_to(p.DC.IMU_Freqeuncy);
        j.at("DeviceConfig").at("RXT_Freqeuncy").get_to(p.DC.RXT_Freqeuncy);
        j.at("DeviceConfig").at("ESC_Freqeuncy").get_to(p.DC.ESC_Freqeuncy);
        j.at("DeviceConfig").at("BBC_Freqeuncy").get_to(p.DC.BBC_Freqeuncy);
        j.at("DeviceConfig").at("BBC_PInterval").get_to(p.DC.BBC_PInterval);
        j.at("DeviceConfig").at("__RCDevice").get_to(p.DC.__RCDevice);
        j.at("DeviceConfig").at("__GPSDevice").get_to(p.DC.__GPSDevice);
        j.at("DeviceConfig").at("__FlowDevice").get_to(p.DC.__FlowDevice);
        j.at("DeviceConfig").at("__MPUDeviceSPI").get_to(p.DC.__MPUDeviceSPI);
        j.at("DeviceConfig").at("__I2CDevice").get_to(p.DC.__I2CDevice);
        j.at("DeviceConfig").at("_IsGPSEnable").get_to(p.DC._IsGPSEnable);
        j.at("DeviceConfig").at("_IsFlowEnable").get_to(p.DC._IsFlowEnable);
        j.at("DeviceConfig").at("_IsRCSafeEnable").get_to(p.DC._IsRCSafeEnable);
        j.at("DeviceConfig").at("_IsBAROEnable").get_to(p.DC._IsBAROEnable);
        j.at("DeviceConfig").at("_IsBlackBoxEnable").get_to(p.DC._IsBlackBoxEnable);
        j.at("PIDConfig").at("AltitudePID").at("_flag_PID_Alt_Accel_Max").get_to(p.PC._flag_PID_Alt_Accel_Max);
        j.at("PIDConfig").at("AltitudePID").at("_flag_PID_Alt_Level_Max").get_to(p.PC._flag_PID_Alt_Level_Max);
        j.at("PIDConfig").at("AltitudePID").at("_flag_PID_Alt_Speed_Max").get_to(p.PC._flag_PID_Alt_Speed_Max);
        j.at("PIDConfig").at("AltitudePID").at("_flag_PID_D_SpeedZ_Gain").get_to(p.PC._flag_PID_D_SpeedZ_Gain);
        j.at("PIDConfig").at("AltitudePID").at("_flag_PID_Hover_Throttle").get_to(p.PC._flag_PID_Hover_Throttle);
        j.at("PIDConfig").at("AltitudePID").at("_flag_PID_I_Alt_Gain").get_to(p.PC._flag_PID_I_Alt_Gain);
        j.at("PIDConfig").at("AltitudePID").at("_flag_PID_I_SpeedZ_Gain").get_to(p.PC._flag_PID_I_SpeedZ_Gain);
        j.at("PIDConfig").at("AltitudePID").at("_flag_PID_P_Alt_Gain").get_to(p.PC._flag_PID_P_Alt_Gain);
        j.at("PIDConfig").at("AltitudePID").at("_flag_PID_P_SpeedZ_Gain").get_to(p.PC._flag_PID_P_SpeedZ_Gain);
        j.at("PIDConfig").at("AltitudePID").at("_flag_PID_Takeoff_Altitude").get_to(p.PC._flag_PID_Takeoff_Altitude);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_AngleRate_Pitch_Gain").get_to(p.PC._flag_PID_AngleRate_Pitch_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_AngleRate__Roll_Gain").get_to(p.PC._flag_PID_AngleRate__Roll_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_AngleRate___Yaw_Gain").get_to(p.PC._flag_PID_AngleRate___Yaw_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_D_Pitch_Gain").get_to(p.PC._flag_PID_D_Pitch_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_D__Roll_Gain").get_to(p.PC._flag_PID_D__Roll_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_D___Yaw_Gain").get_to(p.PC._flag_PID_D___Yaw_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_I_Pitch_Gain").get_to(p.PC._flag_PID_I_Pitch_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_I_Pitch_Max__Value").get_to(p.PC._flag_PID_I_Pitch_Max__Value);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_I__Roll_Gain").get_to(p.PC._flag_PID_I__Roll_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_I__Roll_Max__Value").get_to(p.PC._flag_PID_I__Roll_Max__Value);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_I___Yaw_Gain").get_to(p.PC._flag_PID_I___Yaw_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_I___Yaw_Max__Value").get_to(p.PC._flag_PID_I___Yaw_Max__Value);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_Level_Max").get_to(p.PC._flag_PID_Level_Max);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_P_Pitch_Gain").get_to(p.PC._flag_PID_P_Pitch_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_P__Roll_Gain").get_to(p.PC._flag_PID_P__Roll_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_P___Yaw_Gain").get_to(p.PC._flag_PID_P___Yaw_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_RCAngle_Pitch_Gain").get_to(p.PC._flag_PID_RCAngle_Pitch_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_RCAngle__Roll_Gain").get_to(p.PC._flag_PID_RCAngle__Roll_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_RCAngle___Yaw_Gain").get_to(p.PC._flag_PID_RCAngle___Yaw_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_RCRate_Pitch_Gain").get_to(p.PC._flag_PID_RCRate_Pitch_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_RCRate__Roll_Gain").get_to(p.PC._flag_PID_RCRate__Roll_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_RCRate___Yaw_Gain").get_to(p.PC._flag_PID_RCRate___Yaw_Gain);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_Rate_Limit").get_to(p.PC._flag_PID_Rate_Limit);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_TPA_BreakPoint").get_to(p.PC._flag_PID_TPA_BreakPoint);
        j.at("PIDConfig").at("AttitudePID").at("_flag_PID_TPA_Trust").get_to(p.PC._flag_PID_TPA_Trust);
        j.at("PIDConfig").at("NAVPID").at("_flag_PID_D_SpeedX_Gain").get_to(p.PC._flag_PID_D_SpeedX_Gain);
        j.at("PIDConfig").at("NAVPID").at("_flag_PID_D_SpeedY_Gain").get_to(p.PC._flag_PID_D_SpeedY_Gain);
        j.at("PIDConfig").at("NAVPID").at("_flag_PID_I_PosX_Gain").get_to(p.PC._flag_PID_I_PosX_Gain);
        j.at("PIDConfig").at("NAVPID").at("_flag_PID_I_PosY_Gain").get_to(p.PC._flag_PID_I_PosY_Gain);
        j.at("PIDConfig").at("NAVPID").at("_flag_PID_I_SpeedX_Gain").get_to(p.PC._flag_PID_I_SpeedX_Gain);
        j.at("PIDConfig").at("NAVPID").at("_flag_PID_I_SpeedY_Gain").get_to(p.PC._flag_PID_I_SpeedY_Gain);
        j.at("PIDConfig").at("NAVPID").at("_flag_PID_P_PosX_Gain").get_to(p.PC._flag_PID_P_PosX_Gain);
        j.at("PIDConfig").at("NAVPID").at("_flag_PID_P_PosY_Gain").get_to(p.PC._flag_PID_P_PosY_Gain);
        j.at("PIDConfig").at("NAVPID").at("_flag_PID_P_SpeedX_Gain").get_to(p.PC._flag_PID_P_SpeedX_Gain);
        j.at("PIDConfig").at("NAVPID").at("_flag_PID_P_SpeedY_Gain").get_to(p.PC._flag_PID_P_SpeedY_Gain);
        j.at("PIDConfig").at("NAVPID").at("_flag_PID_PosMan_Speed_Max").get_to(p.PC._flag_PID_PosMan_Speed_Max);
        j.at("PIDConfig").at("NAVPID").at("_flag_PID_Pos_Level_Max").get_to(p.PC._flag_PID_Pos_Level_Max);
        j.at("PIDConfig").at("NAVPID").at("_flag_PID_Pos_Speed_Max").get_to(p.PC._flag_PID_Pos_Speed_Max);
        j.at("PIDConfig").at("Output").at("_flag_A1_Pin").get_to(p.OC._flag_A1_Pin);
        j.at("PIDConfig").at("Output").at("_flag_A2_Pin").get_to(p.OC._flag_A2_Pin);
        j.at("PIDConfig").at("Output").at("_flag_B1_Pin").get_to(p.OC._flag_B1_Pin);
        j.at("PIDConfig").at("Output").at("_flag_B2_Pin").get_to(p.OC._flag_B2_Pin);
        j.at("PIDConfig").at("Output").at("_flag_YAWOut_Reverse").get_to(p.OC._flag_YAWOut_Reverse);
        j.at("PIDConfig").at("Output").at("_flag_ESC_Lazy_Per").get_to(p.OC._flag_ESC_Lazy_Per);
        j.at("PIDConfig").at("Output").at("ESCPLFrequency").get_to(p.OC.ESCPLFrequency);
        j.at("PIDConfig").at("Output").at("ESCControllerType").get_to(p.OC.ESCControllerType);
        j.at("SensorConfig").at("_flag_MPU_Flip__Roll").get_to(p.SC._flag_MPU_Flip__Roll);
        j.at("SensorConfig").at("_flag_MPU_Flip_Pitch").get_to(p.SC._flag_MPU_Flip_Pitch);
        j.at("SensorConfig").at("_flag_MPU_Flip___Yaw").get_to(p.SC._flag_MPU_Flip___Yaw);
        j.at("SensorConfig").at("_flag_Accel__Roll_Cali").get_to(p.SC._flag_Accel__Roll_Cali);
        j.at("SensorConfig").at("_flag_Accel_Pitch_Cali").get_to(p.SC._flag_Accel_Pitch_Cali);
        j.at("SensorConfig").at("_flag_MPU9250_A_X_Cali").get_to(p.SC._flag_MPU9250_A_X_Cali);
        j.at("SensorConfig").at("_flag_MPU9250_A_Y_Cali").get_to(p.SC._flag_MPU9250_A_Y_Cali);
        j.at("SensorConfig").at("_flag_MPU9250_A_Z_Cali").get_to(p.SC._flag_MPU9250_A_Z_Cali);
        j.at("SensorConfig").at("_flag_MPU9250_A_X_Scal").get_to(p.SC._flag_MPU9250_A_X_Scal);
        j.at("SensorConfig").at("_flag_MPU9250_A_Y_Scal").get_to(p.SC._flag_MPU9250_A_Y_Scal);
        j.at("SensorConfig").at("_flag_MPU9250_A_Z_Scal").get_to(p.SC._flag_MPU9250_A_Z_Scal);
        j.at("SensorConfig").at("_flag_COMPASS_X_Offset").get_to(p.SC._flag_COMPASS_X_Offset);
        j.at("SensorConfig").at("_flag_COMPASS_X_Scaler").get_to(p.SC._flag_COMPASS_X_Scaler);
        j.at("SensorConfig").at("_flag_COMPASS_Y_Offset").get_to(p.SC._flag_COMPASS_Y_Offset);
        j.at("SensorConfig").at("_flag_COMPASS_Y_Scaler").get_to(p.SC._flag_COMPASS_Y_Scaler);
        j.at("SensorConfig").at("_flag_COMPASS_Z_Offset").get_to(p.SC._flag_COMPASS_Z_Offset);
        j.at("SensorConfig").at("_flag_COMPASS_Z_Scaler").get_to(p.SC._flag_COMPASS_Z_Scaler);
        j.at("SensorConfig").at("_flag_COMPASS_V_Offset").get_to(p.SC._flag_COMPASS_V_Offset);
        j.at("SensorConfig").at("_flag_COMPASS_V_Scaler").get_to(p.SC._flag_COMPASS_V_Scaler);
        j.at("SensorConfig").at("_flag_COMPASS_Flip__Roll").get_to(p.SC._flag_COMPASS_Flip__Roll);
        j.at("SensorConfig").at("_flag_COMPASS_Flip_Pitch").get_to(p.SC._flag_COMPASS_Flip_Pitch);
        j.at("SensorConfig").at("_flag_COMPASS_Flip___Yaw").get_to(p.SC._flag_COMPASS_Flip___Yaw);
        j.at("SensorConfig").at("_flag_COMPASS_YAW_Offset").get_to(p.SC._flag_COMPASS_YAW_Offset);
        j.at("RCConfig").at("_flag_RC_Min_PWM_Value").get_to(p.RC._flag_RC_Min_PWM_Value);
        j.at("RCConfig").at("_flag_RC_Mid_PWM_Value").get_to(p.RC._flag_RC_Mid_PWM_Value);
        j.at("RCConfig").at("_flag_RC_Max_PWM_Value").get_to(p.RC._flag_RC_Max_PWM_Value);
        j.at("RCConfig").at("_flag_RC_ARM_PWM_Value").get_to(p.RC._flag_RC_ARM_PWM_Value);
        j.at("RCConfig").at("_flag_RC_ARM_PWM_Channel").get_to(p.RC._flag_RC_ARM_PWM_Channel);
        j.at("RCConfig").at("_flag_RC_AP_RateHold_PWM_Value").get_to(p.RC._flag_RC_AP_RateHold_PWM_Value);
        j.at("RCConfig").at("_flag_RC_AP_RateHold_PWM_Channel").get_to(p.RC._flag_RC_AP_RateHold_PWM_Channel);
        j.at("RCConfig").at("_flag_RC_AP_AutoStable_PWM_Value").get_to(p.RC._flag_RC_AP_AutoStable_PWM_Value);
        j.at("RCConfig").at("_flag_RC_AP_AutoStable_PWM_Channel").get_to(p.RC._flag_RC_AP_AutoStable_PWM_Channel);
        j.at("RCConfig").at("_flag_RC_AP_AltHold_PWM_Value").get_to(p.RC._flag_RC_AP_AltHold_PWM_Value);
        j.at("RCConfig").at("_flag_RC_AP_AltHold_PWM_Channel").get_to(p.RC._flag_RC_AP_AltHold_PWM_Channel);
        j.at("RCConfig").at("_flag_RC_AP_PositionHold_PWM_Value").get_to(p.RC._flag_RC_AP_PositionHold_PWM_Value);
        j.at("RCConfig").at("_flag_RC_AP_PositionHold_PWM_Channel").get_to(p.RC._flag_RC_AP_PositionHold_PWM_Channel);
        j.at("RCConfig").at("_flag_RC_AP_SpeedHold_PWM_Value").get_to(p.RC._flag_RC_AP_SpeedHold_PWM_Value);
        j.at("RCConfig").at("_flag_RC_AP_SpeedHold_PWM_Channel").get_to(p.RC._flag_RC_AP_SpeedHold_PWM_Channel);
        j.at("RCConfig").at("_flag_RC_AP_UserAuto_PWM_Value").get_to(p.RC._flag_RC_AP_UserAuto_PWM_Value);
        j.at("RCConfig").at("_flag_RC_AP_UserAuto_PWM_Channel").get_to(p.RC._flag_RC_AP_UserAuto_PWM_Channel);
        j.at("RCConfig").at("_flag_RCIsReserv__Roll").get_to(p.RC._flag_RCIsReserv__Roll);
        j.at("RCConfig").at("_flag_RCIsReserv_Pitch").get_to(p.RC._flag_RCIsReserv_Pitch);
        j.at("RCConfig").at("_flag_RCIsReserv___Yaw").get_to(p.RC._flag_RCIsReserv___Yaw);
        j.at("FilterConfig").at("_flag_Filter_Gryo_Type").get_to(p.FC._flag_Filter_Gryo_Type);
        j.at("FilterConfig").at("_flag_Filter_GryoST2_Type").get_to(p.FC._flag_Filter_GryoST2_Type);
        j.at("FilterConfig").at("_flag_Filter_GYaw_CutOff").get_to(p.FC._flag_Filter_GYaw_CutOff);
        j.at("FilterConfig").at("_flag_Filter_Gryo_CutOff").get_to(p.FC._flag_Filter_Gryo_CutOff);
        j.at("FilterConfig").at("_flag_Filter_GryoST2_CutOff").get_to(p.FC._flag_Filter_GryoST2_CutOff);
        j.at("FilterConfig").at("_flag_Filter_Gryo_NotchFreq").get_to(p.FC._flag_Filter_Gryo_NotchFreq);
        j.at("FilterConfig").at("_flag_Filter_Gryo_NotchCutOff").get_to(p.FC._flag_Filter_Gryo_NotchCutOff);
        j.at("FilterConfig").at("_flag_Filter_Gryo_DynamicNotchRange").get_to(p.FC._flag_Filter_Gryo_DynamicNotchRange);
        j.at("FilterConfig").at("_flag_Filter_Gryo_DynamicNotchMinFreq").get_to(p.FC._flag_Filter_Gryo_DynamicNotchMinFreq);
        j.at("FilterConfig").at("_flag_Filter_Gryo_DynamicNotchEnable").get_to(p.FC._flag_Filter_Gryo_DynamicNotchEnable);
        j.at("FilterConfig").at("_flag_Filter_Accel_Type").get_to(p.FC._flag_Filter_Accel_Type);
        j.at("FilterConfig").at("_flag_Filter_Accel_CutOff").get_to(p.FC._flag_Filter_Accel_CutOff);
        j.at("FilterConfig").at("_flag_Filter_AngleMix_Alpha").get_to(p.FC._flag_Filter_AngleMix_Alpha);
        j.at("FilterConfig").at("_flag_Baro_Trust_Beta").get_to(p.FC._flag_Baro_Trust_Beta);
        j.at("FilterConfig").at("_flag_Accel_Trust_Beta").get_to(p.FC._flag_Accel_Trust_Beta);
        j.at("FilterConfig").at("_flag_Sonar_Trust_Beta").get_to(p.FC._flag_Sonar_Trust_Beta);
        j.at("FilterConfig").at("_flag_GPSAlt_Trust_Beta").get_to(p.FC._flag_GPSAlt_Trust_Beta);
        j.at("FilterConfig").at("_flag_AccelBias_Trust_Beta").get_to(p.FC._flag_AccelBias_Trust_Beta);
        j.at("FilterConfig").at("_flag_GPS_Config_Beta").get_to(p.FC._flag_GPS_Config_Beta);
        j.at("FilterConfig").at("_flag_Flow_Config_Beta").get_to(p.FC._flag_Flow_Config_Beta);
        j.at("FilterConfig").at("_flag_Braking_Speed_Gain").get_to(p.FC._flag_Braking_Speed_Gain);
        j.at("FilterConfig").at("_flag_Filter_RC_CutOff").get_to(p.FC._flag_Filter_RC_CutOff);
        j.at("FilterConfig").at("_flag_Filter_AngleRate_CutOff").get_to(p.FC._flag_Filter_AngleRate_CutOff);
        j.at("FilterConfig").at("_flag_Filter_PID_I_CutOff").get_to(p.FC._flag_Filter_PID_I_CutOff);
        j.at("FilterConfig").at("_flag_Filter_PID_D_ST1_CutOff").get_to(p.FC._flag_Filter_PID_D_ST1_CutOff);
        j.at("FilterConfig").at("_flag_Filter_PID_D_ST2_CutOff").get_to(p.FC._flag_Filter_PID_D_ST2_CutOff);
    }

    SingleAPMAPI::APMSettinngs readConfigFromJson(nlohmann::json json)
    {
        return json.get<SingleAPMAPI::APMSettinngs>();
    }
}