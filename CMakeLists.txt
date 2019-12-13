IF(UNIX)
###########################code Definetion##################
	#this defintitions is for mpu9250 mode , choose one
	add_definitions(-DSPI_MPU9250)
	add_definitions(-DI2C_MPU9250)

	#this defintition is for Recvicer Mode , choose one
	add_definitions(-DSBUS_CONVERTER)
	add_definitions(-DIBUS_Serial)

	#this defintiion is for MS5611 Mode , choose one
	###add_definitions(-DI2C_MS5611)
#############################################################
    cmake_minimum_required(VERSION 3.10)
    set(CMAKE_C_COMPILER "/usr/bin/gcc")
    set(CMAKE_CXX_COMPILER "/usr/bin/g++")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
    find_library(wiringPi_LIB wiringPi)
	find_package(nlohmann_json REQUIRED)
	add_subdirectory(_thirdparty)
    add_executable(SingleAPM "SingleAPM.cpp" "SingleAPM.hpp")
    target_link_libraries( SingleAPM ${wiringPi_LIB})
	target_link_libraries( SingleAPM LIB)
ENDIF()