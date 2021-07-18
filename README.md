# SingleFlight

SingleFlight is flight controller C++ API used to fly quad-rotor craft.

This project differs from Baseflight and inav in that it focuses on running controller in ARM-Linux, and suit for indoor navigation.<\br>
This project use some parts of Cleanfilght and inav , but it's not a fork.<br/>

<img src="https://github.com/TSKangetsu/RPiSingleAPM/blob/master/Document/Header.jpg" style="transform:rotate(90deg);">

- GetStarted
  - [Features](#Features)
  - [HardwareSupport](#Supported-Hardware) 
  - [Special thanks](#Special-thanks)
  - [BuildConfigure](#BuildConfigure)
  - [Configure RaspberryPi For Controller](#Configure-RaspberryPi-For-Controller)
  
# Features
  * Run in Raspberrypi4B without any external controller
  * 2kHZ with Attitude Control and IMU navigation
  * 1KHZ ESCUpdate With PCA9685
  * Position hold , Speed hold with stick
  * Simple C++ API control copter's speed , position, attitidue. Custom Position sensor and RC input support
  * Auto-Takeoff support

# Supported Hardware
  * Attitude controll with MPU9250
  * Surface Tracking with <a href="http://www.mateksys.com/?portfolio=3901-l0x">Mateksys 3901-L0X: OPTICAL FLOW & LIDAR SENSOR </a>
  *  AltHold controll with Mateksys 3901-L0X , or MS5611 Pressure sensor
  *  GPS hold with M8N (Unimplment)
  *  QMC5883 or HMC5883 support (Uninplment)
  *  5 - 7 inch Race-Quadcopter test Pass
  *  PCA9685 PWM controller to ESC
  *  Sbus and Ibus RC support

# Special thanks
  * Attitude and PID , Looping core reference from <a href="https://www.youtube.com/channel/UCpJ5uKSLxP84TXQtwiRNm1g">Joop Brokking</a>
  * Navigation core reference from <a href="https://github.com/iNavFlight/inav">INAV</a>
  * Filter reference <a href="https://github.com/cleanflight/cleanflight">cleanflight</a>

# BuildConfigure 
### Build Test Module And Fly On RaspberryPi:
```SHELL
    #TestModule Is using Json lib for FlightController configure
    #You need to compile and install nlohmann/json in you RaspberryPi OS
    git clone https://github.com/nlohmann/json
    cd json && git checkout v3.7.0
    mkdir build && cd build
    cmake .. && make -j1 #prevent 1GB device OOM,if your Pi is over 2gb can use -j4
    sudo make install
    cd ../.. #Please make sure what things you are doing
    #Build RPiSingleAPM
    git clone https://github.com/TSKangetsu/RPiSingleAPM.git --recursive
    cd RPiSingleAPM && mkdir build
    cd build && cmake ..
    make -j1 && cp SingleAPM /usr/bin
    cd .. && cp APMconfig.json /etc/ #Copy APMConfig.json to /etc/ , this file has flying controller settings,like pid tunning
    SingleAPM -r # Run APM Programe If data fresh at screen , you can try to fly
```
### Use RPiSingleAPM for developing
```CMAKE
    #Do this in you git project if you using CMake
    git submodule add https://github.com/TSKangetsu/RPiSingleAPM [Dir-Where-you-want]
    #In CMAKE
    ...
    add_definitions(-DRPiDEBUGStart)
    add_definitions(-DRPiDEBUG)
    ...
    add_subdirectory([PATH-TO-RPiSingleAPM]/src)
    ...
    target_link_libraries([YOU-EXEC] RPiSingleAPI)
    #Attention!
    # If you use git submodule , when you clone a new dir, you need to add --recursive
    # or After Clone run:
    git submodule update --init --recursive
```

# Configure RaspberryPi For Controller
  - ### If you are RaspberryPi4B, and want to use GPS or FlowSensor. You can check <a href="https://raspberrypi.stackexchange.com/questions/104464/where-are-the-uarts-on-the-raspberry-pi-4">here</a>
```R
    # Edit /boot/config.txt
    core_freq = 250 
    # If you want to use software Serial for RC controller
    # If RC use hardware serial can be higher
    # If using Analog Video Output on PI4 should be 360 and use hardware serial
    dtparam=i2c_arm=on
    dtparam=i2c_baudrate=400000 #RaspberryPi3 Is max to 400khz,Pi4 can Up to 1MHZ
    dtparam=spi=on
    # This set i2c and spi enable for RaspberryPi 
    # or you can use sudo raspi-config to configure
    enable_uart=1
    dtoverlay=pi3-miniuart-bt # Disable Software Serial for GPIO
    dtoverlay=uart3 # if you are Pi4 that you can use more than 4 GPIO Uart
    dtoverlay=uart5 
    # Set Uart enable and use Hardware serial

    # Attention: If config.txt exist same object ,replase it
    # AfterChange Save and reboot RaspberryPi to effect the settings
```
  - ### AddOn: You need to Disable GPIO UART Login TTY. You can check <a href="https://www.raspberrypi.org/documentation/configuration/uart.md#:~:text=Disable%20Linux%20serial%20console&text=This%20can%20be%20done%20by,Select%20option%20P6%20%2D%20Serial%20Port.">here</a>
