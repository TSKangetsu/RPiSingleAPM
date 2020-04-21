
# RPiSingleAPM


RPiSingleAPM is a c++ API for RaspberryPi , it was tested on RaspberryPi3B , 3B+ , 4B , using RPiSingleAPM can make RaspberryPi using
PCA9685 and MPU9250(MPU6050 , and MS5611) to build a Auto-Leveling quadCotper ，it means that you can easily using RaspberryPi with OpenVINO to make a 
computer-vision-base quadCotper

*\*This project is using GPL linsense*

## Code Dependence
- JSON_LIB : https://github.com/nlohmann/json

- wiringPi : http://wiringpi.com/

- all of them need to build and install at Raspbain

## Hardware Dependence
at least you need :
 - PCA9685 to drive 4 Electronic-Speed-Control , 

 - MPU6050 to use for Auto-Leveing the copter

 - Remote receiver now support IBUS , in future will fully support SBUS , you can use IBUS perfect
 
## Build Test Need
 - install nlohamnnJSON
 - using cmake build the code
 - move APMconfig.json to /etc/
 - cd build and run ./SingleAPM -r

*\* check RC value and change value at /etc/APMconfig.json , IBUS value is diffrent from SBUS , if no ckeck , it won armed*


## The way to use TestMdule

 1. compile and install nlohmann [JSON](https://github.com/nlohmann/json)
- **git clone https://github.com/nlohmann/json**

- **cd /clone file directory/**. 

- **git checkout v3.7.0** 切换到指定版本分支 Switch to the specified version branch

- **cd build**

- **cmake**

- **make**

- **make install**
2. install wiringPI (google youreself)
3. Compile SingleAPM (https://github.com/TSKangetsu/RPiSingleAPM)
- **git clone https://github.com/nlohmann/json**

- **cd RPiSingleAPM** 

- **mkdir build , cd build**

- **cmake ..**

- **make**

- **cd .. , mv APMconfig.json /etc/** 
move APMconfig.json within the project to /etc

- **cd build , mv SingleAPM /usr/bin，sudo chmod /usr/bin/SingleAPM 755**
Move SingleAPM executable file within build in /usr/bin/, and set the permission as 755
- **execute SingleAPM -r**

## 2.Configuration instructions of TestModule:
![网络图片](https://github.com/pluierry/picture/blob/master/readme%20pictures/TestModule%E7%9A%84%E9%A3%9E%E8%A1%8C%E9%85%8D%E7%BD%AE%E8%AF%B4%E6%98%8E.png?raw=true)
1.	Type_MPU9250: 
If the mpu9250 or mpu6000 sensor connected to raspberry PI is an SPI connection, select 1, 
Otherwise(I2C) select 0.
**Attention**：the I2C configuration of raspberry pie needs to be changed to high-speed mode）

**Changing the default I2C Speed**
```
The default I2C baudrate on the Pi 3 is 100Kbps (kilo bits (not bytes) per second). 
At this speed, clearing the interrupt register and reading the IMU data 
(14 bytes; 3 16 bit gyros, 1 16 bit temperature, 3 16 bit accels) takes about 6.5ms, which is unacceptably slow. 
Since the data is read while processing the ISR, the slow speed of the data read operation imposes an upper bound of 150Hz on the interrupt frequency. 
It is possible to increase the default I2C baudrate by modifying the /boot/config.
txt file as shown below.  
```  
👆Form <http://www.telesens.co/2017/03/11/imu-sampling-using-the-raspberry-pi/>

![网络图片](https://github.com/pluierry/picture/blob/master/readme%20pictures/%E6%9B%B4%E6%94%B9I2c%E7%9A%84%E9%80%9F%E5%BA%A6.png?raw=true)

You need to restart PI for the Settings to take effect

2. Type_RC: receiver mode: 0 for IBUS mode, 1 for SBUS mode, please put receiver at /dev/ttys0, this will be improved later  
3. Type_IMUFilter: gyro filter 0 is no filter, 1 is pt1 filter (currently not available), 2 is 50hz low-pass filter (recommend to use first-order filter)  
4. Type_IMUMixFilter: The fusion filter of gyroscope and accelerometer, 0 is the first order complementary filter, suitable for short time flight；1 is the Kalman filtering, suitable for long time operation.  
5. Update_Freqeucy: It can't be changed now，If you change this item, you may cause a personal accident.  
6. _flag_A1_Pin:
  _flag_A2_Pin:
  _flag_B1_Pin:
  _flag_B2_Pin:
  This is the PWM connection position of four-axis pca9685: fill in according to the pin number on pca9685  
  ![网络图片](https://github.com/pluierry/picture/blob/master/readme%20pictures/%E9%92%88%E5%8F%B7.png?raw=true)  

7. _flag_Accel_Pitch_Cali:
_flag_Accel__Roll_Cali:
Horizontal correction of aircraft acceleration sensor，it needs to be adjusted

8. PID related：The PID values of Pitch roll yaw need to be adjusted to be stable. It is not recommended to change the max_arbitration value and level_Max  
![网络图片](https://github.com/pluierry/picture/blob/master/readme%20pictures/PID%E7%9B%B8%E5%85%B3.png?raw=true)

9. The remote control related：   
**Matters needing attention：**
The PWM values of ibus and sbus have some different, it need to be adjusted by yourself, otherwise they cannot fly or lose control and put Personal safety at risk. The channel of flight control is unlocked in channel 5 of the remote control, please adjust by yourself
Reserv represents the reverse direction of the remote control and can only be 1 and -1. Changing to other values will endanger people's safety  
![网络图片](https://github.com/pluierry/picture/blob/master/readme%20pictures/%E9%81%A5%E6%8E%A7%E5%99%A8%E7%9B%B8%E5%85%B3.png?raw=true)
