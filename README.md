
- [The way to use TestMdule](#design-goals)
- [Configuration instructions of TestModule](#sponsors)
## 1.TestModule使用方法/The way to use TestMdule

 1. 编译安装 nlohmann [JSON](https://github.com/nlohmann/json)
   compile and install nlohmann [JSON](https://github.com/nlohmann/json)

 **方法method:**
- **git clone https://github.com/nlohmann/json**

- **cd /clone file directory/**. 

- **git checkout v3.7.0** 切换到指定版本分支 Switch to the specified version branch

- **cd build**

- **cmake**

- **make**

- **make install**

 2. 安装wiringPI，自行谷歌
 install wiringPI (google youreself)

 3. 编译SingleAPM https://github.com/TSKangetsu/RPiSingleAPM
 Compile SingleAPM (https://github.com/TSKangetsu/RPiSingleAPM)
- **git clone https://github.com/nlohmann/json**

- **cd RPiSingleAPM** 

- **mkdir build , cd build**

- **cmake ..**

- **make**

- **cd .. , mv APMconfig.json /etc/** 
移动项目内的APMconfig.json到/etc/下面
move APMconfig.json within the project to /etc

- **cd build , mv SingleAPM /usr/bin，sudo chmod /usr/bin/SingleAPM 755**
移动build内的SingleAPM可执行文件到 /usr/bin/内并设定权限为755
Move SingleAPM executable file within build in /usr/bin/, and set the permission as 755
- **execute SingleAPM -r**

## 2.Configuration instructions of TestModule:
![网络图片](https://github.com/pluierry/picture/blob/master/readme%20pictures/TestModule%E7%9A%84%E9%A3%9E%E8%A1%8C%E9%85%8D%E7%BD%AE%E8%AF%B4%E6%98%8E.png?raw=true)
1.	Type_MPU9250: 
如果连接树莓派的mpu9250 或mpu6000系传感器是SPI连接，则选择1，I2C为0
If the mpu9250 or mpu6000 sensor connected to raspberry PI is an SPI connection, select 1, 
Otherwise(I2C) select 0.
**注意**：树莓派的I2C配置需要更改为高速模式）
**Attention**：the I2C configuration of raspberry pie needs to be changed to high-speed mode）

**更改默认的I2C速度/Changing the default I2C Speed**
```
Pi 3的默认I2C波特率是100Kbps（每秒千比特（不是字节））。
以这种速度，清除中断寄存器并读取IMU数据（14字节；3个16位陀螺仪，1个16位温度，3个16位加速度）
大约需要6.5ms，这太慢了。
由于在处理ISR时会读取数据，因此数据读取操作的速度较慢，因此中断频率上限为150Hz。
如下所示，可以通过修改/boot/config.txt文件来增加默认的I2C波特率。
The default I2C baudrate on the Pi 3 is 100Kbps (kilo bits (not bytes) per second). 
At this speed, clearing the interrupt register and reading the IMU data 
(14 bytes; 3 16 bit gyros, 1 16 bit temperature, 3 16 bit accels) takes about 6.5ms, which is unacceptably slow. 
Since the data is read while processing the ISR, the slow speed of the data read operation imposes an upper bound of 150Hz on the interrupt frequency. 
It is possible to increase the default I2C baudrate by modifying the /boot/config.
txt file as shown below.  
```  
👆Form <http://www.telesens.co/2017/03/11/imu-sampling-using-the-raspberry-pi/>

![网络图片](https://github.com/pluierry/picture/blob/master/readme%20pictures/%E6%9B%B4%E6%94%B9I2c%E7%9A%84%E9%80%9F%E5%BA%A6.png?raw=true)

您需要重启PI才能使设备生效
You need to restart PI for the Settings to take effect

2. 接收器的模式：0 为IBUS模式， 1 为 SBUS模式，接收器请接在 /dev/ttyS0 ，日后会改进这一点
Type_RC: receiver mode: 0 for IBUS mode, 1 for SBUS mode, please put receiver at /dev/ttys0, this will be improved later
3. 陀螺仪的滤波器 0为不使用滤波器， 1为pt1滤波器(目前不可使用)  , 2 为50hz的低通滤波器（推荐一阶滤波）
Type_IMUFilter: gyro filter 0 is no filter, 1 is pt1 filter (currently not available), 2 is 50hz low-pass filter (recommend to use first-order filter)
4. Type_IMUMixFilter： 陀螺仪与加速度计的融合滤波器 ， 0 为一阶互补滤波 ，适合短时间飞行；1为卡尔曼滤波，适合长时间运行
Type_IMUMixFilter: The fusion filter of gyroscope and accelerometer, 0 is the first order complementary filter, suitable for short time flight；1 is the Kalman filtering, suitable for long time operation.
5. Update_Freqeucy: 目前不可更改，如果擅自更改此项可能会造成人身事故
It can't be changed now，If you change this item, you may cause a personal accident.
6. _flag_A1_Pin:
  _flag_A2_Pin:
  _flag_B1_Pin:
  _flag_B2_Pin:
  此为四轴的pca9685的pwm接线位置：按照pca9685上的针号来填
  This is the PWM connection position of four-axis pca9685: fill in according to the pin number on pca9685
  ![网络图片](https://github.com/pluierry/picture/blob/master/readme%20pictures/%E9%92%88%E5%8F%B7.png?raw=true)
7. _flag_Accel_Pitch_Cali:
_flag_Accel__Roll_Cali:
飞行器加速度传感器的水平修正值，需自行调整
Horizontal correction of aircraft acceleration sensor，it needs to be adjusted
8.
PID相关：Pitch roll yaw的PID相关数值，需要自行调试至稳定，不建议更改max__value和level_Max
PID related：The PID values of Pitch roll yaw need to be adjusted to be stable. It is not recommended to change the max_arbitration value and level_Max
![网络图片](https://github.com/pluierry/picture/blob/master/readme%20pictures/PID%E7%9B%B8%E5%85%B3.png?raw=true)
9. 
遥控器相关The remote control related：
**注意事项Matters needing attention：**
ibus和sbus的pwm值有许不同，需要自行调整，否则无法飞行或者失控危及人身安全，解锁飞控的通道在遥控器5通道，请自行调整。Reserv代表遥控方向反转 ，只能是1和-1 ，若更改为其他数值则会危及人身安全，需要注意。
The PWM values of ibus and sbus have some different, it need to be adjusted by yourself, otherwise they cannot fly or lose control and put Personal safety at risk. The channel of flight control is unlocked in channel 5 of the remote control, please adjust by yourself
Reserv represents the reverse direction of the remote control and can only be 1 and -1. Changing to other values will endanger people's safety
 ![网络图片](https://github.com/pluierry/picture/blob/master/readme%20pictures/%E9%81%A5%E6%8E%A7%E5%99%A8%E7%9B%B8%E5%85%B3.png?raw=true)
