#ifdef DEBUG
#include <iostream>
#endif
#include <ctime>
#include <math.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>
#define MPUTypeI2C 0
#define MPUTypeSPI 1

struct MPUData
{
    int _uORB_MPU9250_A_X = 0;
    int _uORB_MPU9250_A_Y = 0;
    int _uORB_MPU9250_A_Z = 0;
    int _uORB_MPU9250_G_X = 0;
    int _uORB_MPU9250_G_Y = 0;
    int _uORB_MPU9250_G_Z = 0;
    int _uORB_MPU9250_M_X = 0;
    int _uORB_MPU9250_M_Y = 0;
    int _uORB_MPU9250_M_Z = 0;

    float _uORB_Gryo__Roll = 0;
    float _uORB_Gryo_Pitch = 0;
    float _uORB_Gryo___Yaw = 0;
    float _uORB_Real__Roll = 0;
    float _uORB_Real_Pitch = 0;
    float _uORB_Accel__Roll = 0;
    float _uORB_Accel_Pitch = 0;
};

class RPiMPU9250
{
public:
    inline RPiMPU9250(int Type = MPUTypeSPI, bool IsBuildInCompassEnable = true,
                      int MPUSPIChannel = 1, unsigned char MPUI2CAddr = 0x68, int UpdateFreq = 250)
    {
        MPU9250_Type = Type;
        MPUUpdateFreq = UpdateFreq;
        MPU9250_I2CAddr = MPU9250_I2CAddr;
        MPU9250_SPI_Channel = MPUSPIChannel;
        CompassEnable = IsBuildInCompassEnable;

        if (Type == MPUTypeI2C)
        {
            MPU9250_fd = wiringPiSPISetup(MPU9250_SPI_Channel, MPU9250_SPI_Freq);
            MPU9250_SPI_Config[0] = 0x6b;
            MPU9250_SPI_Config[1] = 0x00;
            wiringPiSPIDataRW(MPU9250_SPI_Channel, MPU9250_SPI_Config, 2); //reset
            MPU9250_SPI_Config[0] = 0x1c;
            MPU9250_SPI_Config[1] = 0x10;
            wiringPiSPIDataRW(MPU9250_SPI_Channel, MPU9250_SPI_Config, 2); // Accel
            MPU9250_SPI_Config[0] = 0x1b;
            MPU9250_SPI_Config[1] = 0x08;
            wiringPiSPIDataRW(MPU9250_SPI_Channel, MPU9250_SPI_Config, 2); // Gryo
            MPU9250_SPI_Config[0] = 0x1a;
            MPU9250_SPI_Config[1] = 0x03;
            wiringPiSPIDataRW(MPU9250_SPI_Channel, MPU9250_SPI_Config, 2); //config
            if (CompassEnable)
            {
            }
        }
        else if (Type == MPUTypeI2C)
        {
            MPU9250_fd = wiringPiI2CSetup(MPU9250_I2CAddr);
            wiringPiI2CWriteReg8(MPU9250_fd, 107, 0x00); //reset
            wiringPiI2CWriteReg8(MPU9250_fd, 28, 0x10);  //Accel
            wiringPiI2CWriteReg8(MPU9250_fd, 27, 0x08);  //Gryo
            wiringPiI2CWriteReg8(MPU9250_fd, 26, 0x03);  //config
            if (CompassEnable)
            {
            }
        }
    };

    inline MPUData MPUSensorsDataGet()
    {
        SF._uORB_Gryo_Pitch = (SF._uORB_Gryo_Pitch * 0.7) + ((SF._uORB_MPU9250_G_X / MPU9250_LSB) * 0.3);
        SF._uORB_Gryo__Roll = (SF._uORB_Gryo__Roll * 0.7) + ((SF._uORB_MPU9250_G_Y / MPU9250_LSB) * 0.3);
        SF._uORB_Gryo___Yaw = (SF._uORB_Gryo___Yaw * 0.7) + ((SF._uORB_MPU9250_G_Z / MPU9250_LSB) * 0.3);

        SF._uORB_Real_Pitch += (SF._uORB_MPU9250_G_X / MPU9250_LSB) / MPUUpdateFreq;
        SF._uORB_Real__Roll += (SF._uORB_MPU9250_G_Y / MPU9250_LSB) / MPUUpdateFreq;
        SF._uORB_Real_Pitch -= SF._uORB_Real__Roll * sin((SF._uORB_Gryo___Yaw / MPUUpdateFreq / MPU9250_LSB) * (3.14 / 180));
        SF._uORB_Real__Roll += SF._uORB_Real_Pitch * sin((SF._uORB_Gryo___Yaw / MPUUpdateFreq / MPU9250_LSB) * (3.14 / 180));

        // SF._Tmp_IMU_Accel_Vector = sqrt((SF._uORB_MPU9250_A_X * SF._uORB_MPU9250_A_X) + (SF._uORB_MPU9250_A_Y * SF._uORB_MPU9250_A_Y) + (SF._uORB_MPU9250_A_Z * SF._uORB_MPU9250_A_Z));
        // if (abs(SF._uORB_MPU9250_A_X) < SF._Tmp_IMU_Accel_Vector)
        //     SF._uORB_Accel__Roll = asin((float)SF._uORB_MPU9250_A_X / SF._Tmp_IMU_Accel_Vector) * -57.296;
        // if (abs(SF._uORB_MPU9250_A_Y) < SF._Tmp_IMU_Accel_Vector)
        //     SF._uORB_Accel_Pitch = asin((float)SF._uORB_MPU9250_A_Y / SF._Tmp_IMU_Accel_Vector) * 57.296;
        // SF._uORB_Accel__Roll -= SF._flag_Accel__Roll_Cali;

        return SF;
    }

private:
    MPUData SF;
    int MPU9250_fd;
    int MPU9250_Type;
    bool CompassEnable;
    int MPUUpdateFreq = 250;
    float MPU9250_LSB = 65.5;
    int MPU9250_I2CAddr = 0x68;
    int MPU9250_SPI_Freq = 400;
    int MPU9250_SPI_Channel = 1;
    unsigned char MPU9250_SPI_Config[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    unsigned char Tmp_MPU9250_Buffer[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    unsigned char Tmp_MPU9250_SPI_Buffer[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    inline void IMUSensorsDataRead()
    {
        if (MPU9250_Type == MPUTypeI2C)
        {
            Tmp_MPU9250_Buffer[0] = wiringPiI2CReadReg8(MPU9250_fd, 0x3B);
            Tmp_MPU9250_Buffer[1] = wiringPiI2CReadReg8(MPU9250_fd, 0x3C);
            SF._uORB_MPU9250_A_X = (short)(Tmp_MPU9250_Buffer[0] << 8 | Tmp_MPU9250_Buffer[1]);
            Tmp_MPU9250_Buffer[2] = wiringPiI2CReadReg8(MPU9250_fd, 0x3D);
            Tmp_MPU9250_Buffer[3] = wiringPiI2CReadReg8(MPU9250_fd, 0x3E);
            SF._uORB_MPU9250_A_Y = (short)(Tmp_MPU9250_Buffer[2] << 8 | Tmp_MPU9250_Buffer[3]);
            Tmp_MPU9250_Buffer[4] = wiringPiI2CReadReg8(MPU9250_fd, 0x3F);
            Tmp_MPU9250_Buffer[5] = wiringPiI2CReadReg8(MPU9250_fd, 0x40);
            SF._uORB_MPU9250_A_Z = (short)(Tmp_MPU9250_Buffer[4] << 8 | Tmp_MPU9250_Buffer[5]);

            Tmp_MPU9250_Buffer[6] = wiringPiI2CReadReg8(MPU9250_fd, 0x43);
            Tmp_MPU9250_Buffer[7] = wiringPiI2CReadReg8(MPU9250_fd, 0x44);
            SF._uORB_MPU9250_G_X = (short)(Tmp_MPU9250_Buffer[6] << 8 | Tmp_MPU9250_Buffer[7]);
            Tmp_MPU9250_Buffer[8] = wiringPiI2CReadReg8(MPU9250_fd, 0x45);
            Tmp_MPU9250_Buffer[9] = wiringPiI2CReadReg8(MPU9250_fd, 0x46);
            SF._uORB_MPU9250_G_Y = (short)(Tmp_MPU9250_Buffer[8] << 8 | Tmp_MPU9250_Buffer[9]);
            Tmp_MPU9250_Buffer[10] = wiringPiI2CReadReg8(MPU9250_fd, 0x47);
            Tmp_MPU9250_Buffer[11] = wiringPiI2CReadReg8(MPU9250_fd, 0x48);
            SF._uORB_MPU9250_G_Z = (short)(Tmp_MPU9250_Buffer[10] << 8 | Tmp_MPU9250_Buffer[11]);
        }
        else if (MPU9250_Type == MPUTypeSPI)
        {
            Tmp_MPU9250_SPI_Buffer[0] = 0xBB;
            wiringPiSPIDataRW(MPU9250_SPI_Channel, Tmp_MPU9250_SPI_Buffer, 21);
            SF._uORB_MPU9250_A_X = (short)((int)Tmp_MPU9250_SPI_Buffer[1] << 8 | (int)Tmp_MPU9250_SPI_Buffer[2]);
            SF._uORB_MPU9250_A_Y = (short)((int)Tmp_MPU9250_SPI_Buffer[3] << 8 | (int)Tmp_MPU9250_SPI_Buffer[4]);
            SF._uORB_MPU9250_A_Z = (short)((int)Tmp_MPU9250_SPI_Buffer[5] << 8 | (int)Tmp_MPU9250_SPI_Buffer[6]);

            SF._uORB_MPU9250_G_X = (short)((int)Tmp_MPU9250_SPI_Buffer[9] << 8 | (int)Tmp_MPU9250_SPI_Buffer[10]);
            SF._uORB_MPU9250_G_Y = (short)((int)Tmp_MPU9250_SPI_Buffer[11] << 8 | (int)Tmp_MPU9250_SPI_Buffer[12]);
            SF._uORB_MPU9250_G_Z = (short)((int)Tmp_MPU9250_SPI_Buffer[13] << 8 | (int)Tmp_MPU9250_SPI_Buffer[14]);

            SF._uORB_MPU9250_M_X = (short)((int)Tmp_MPU9250_SPI_Buffer[16] << 8) | (int)Tmp_MPU9250_SPI_Buffer[15];
            SF._uORB_MPU9250_M_Y = (short)((int)Tmp_MPU9250_SPI_Buffer[18] << 8) | (int)Tmp_MPU9250_SPI_Buffer[17];
            SF._uORB_MPU9250_M_Z = (short)((int)Tmp_MPU9250_SPI_Buffer[20] << 8) | (int)Tmp_MPU9250_SPI_Buffer[19];
        }
    }
};