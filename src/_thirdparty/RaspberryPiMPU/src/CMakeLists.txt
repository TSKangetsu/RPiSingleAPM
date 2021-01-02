find_library(WIRING wiringPi)
aux_source_directory(MPU9250 MPU9250LIB)

add_library(RPiMPU ${MPU9250LIB})
target_link_libraries(RPiMPU ${WIRING})
