aux_source_directory(. RPiSingleAPI)
find_library(wiringPi_LIB wiringPi)
add_library(RPiSingleAPI ${RPiSingleAPI})
target_link_libraries( SingleAPM ${wiringPi_LIB})