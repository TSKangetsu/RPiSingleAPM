cmake_minimum_required (VERSION 3.8)

project ("RaspberryPI_IBUS")
find_library(WIRING wiringPi)

add_subdirectory(src)

add_executable (RaspberryPI_IBUS "RaspberryPI_IBUS.cpp")
target_link_libraries(RaspberryPI_IBUS  ${WIRING})
target_link_libraries(RaspberryPI_IBUS ${Ibus})
