SET(NCURSES_ENABLE false  CACHE BOOL "ncurses console debug controller enable")

IF(UNIX)
###########################code Definetion##################
    add_definitions(-DRPiDEBUGStart)
    add_definitions(-DRPiDEBUG)
#############################################################
    cmake_minimum_required(VERSION 3.10)
    set(CMAKE_C_COMPILER "/usr/bin/gcc")
    set(CMAKE_CXX_COMPILER "/usr/bin/g++")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
    find_library(wiringPi_LIB wiringPi)
	find_package(nlohmann_json REQUIRED)
    add_subdirectory(src)
    add_executable(SingleAPM "TestModule.cpp")
    target_link_libraries( SingleAPM ${wiringPi_LIB})
    target_link_libraries( SingleAPM RPiSingleAPI)
ENDIF()