# RPiSingleAPM

RPiSingleAPM is a c++ API for RaspberryPi , it was tested on RaspberryPi3B , 3B+ , 4B , using RPiSingleAPM can make RaspberryPi using
PCA9685 and MPU9250(MPU6050 , and MS5611) to build a Auto-Leveling quadCotper ，it means that you can easily using RaspberryPi with OpenVINO to make a 
computer-vision-base quadCotper

*\*This project is using GPL linsense*

## Code Dependence
JSON_LIB : https://github.com/nlohmann/json

wiringPi : http://wiringpi.com/

all of them need to build and install at Raspbain

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
 *\* check RC value and change value at /etc/APMconfig.json , IBUS value is diffrent from SBUS , if no ckeck , it won armed

### Other

*\* Althought the code now can fly , but now I have no time to write more , about 2020-1 will be well done*
