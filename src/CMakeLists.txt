set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
#================================================================
aux_source_directory(. RPiSingleAPI)
#================================================================
find_library(wiringPi_LIB wiringPi)
find_package(Threads REQUIRED)
#================================================================
add_library(RPiSingleAPI ${RPiSingleAPI})
add_subdirectory(_thirdparty)
target_link_libraries(RPiSingleAPI PRIVATE SingleFlight3rd)
target_link_libraries(RPiSingleAPI PRIVATE Threads::Threads)
target_link_libraries(RPiSingleAPI PRIVATE ${wiringPi_LIB})