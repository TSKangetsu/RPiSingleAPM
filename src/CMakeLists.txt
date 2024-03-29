set(ISCXX17 FALSE)
set(LinkAsSingle TRUE)

if(NOT UNIX)
  message(STATUS "[SingleFlight] Don't Try to compile in Windows!")
endif()

message(STATUS "[SingleFlight] RPiSingleAPI is Linked!")
message(STATUS "[SingleFlight] RPiSingleAPI C   Compiler Check: " ${CMAKE_C_COMPILER})
message(STATUS "[SingleFlight] RPiSingleAPI CXX Compiler Check: " ${CMAKE_CXX_COMPILER})

foreach(i ${CMAKE_CXX_COMPILE_FEATURES})
  if(${i} MATCHES "cxx_std_17")
    message(STATUS "[SingleFlight] Compiler Check C++17 :" ${i})
    set(${ISCXX17} TRUE)
  endif()
endforeach()
if(${ISCXX17} MATCHES FALSE)
  message(STATUS "[SingleFlight] NOT SUPPROT C++ 17 STANDARD, API MAY NOT WORK!")
elseif(${ISCXX17} MATCHES TRUE)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  message(STATUS "[SingleFlight] Compiler C++17 is Set")
endif()

message(STATUS "[Tips]: add set(MPUDriver 'pigpio' CACHE  INTERNAL '') to your major cmake header to change SPI driver for Better imu freq")
if(${LinkAsSingle})
#================================================================
  message(STATUS "[SingleFlight] Link as Single .a mode")
  if(${MPUDriver} MATCHES "spidev")
    message(STATUS "[MPUInfo]MPUDriverType link comfirm: " ${MPUDriver})
  elseif(${MPUDriver} MATCHES "pigpio")
    message(STATUS "[MPUInfo]MPUDriverType link comfirm: " ${MPUDriver})
    find_library(PIGPIO pigpio)
    add_definitions(-DMPUSPI_PIGPIO)
  else()
    message(STATUS "[MPUInfo]MPUDriver no select! use spidev as default")
  endif()


  aux_source_directory("_thirdparty/" SingleFlight3rd)
  aux_source_directory("_thirdparty/EKFImplement/src/." EKFLIB)
  aux_source_directory("_thirdparty/HardwareGenerator/." HardwareGenerator)
  aux_source_directory("_thirdparty/RaspberryPiMPU/src" MPU)
  
  aux_source_directory(. RPiSingleAPI)
  find_package(Threads REQUIRED)

  add_library(RPiSingleAPI ${RPiSingleAPI} ${SingleFlight3rd} ${EKFLIB} ${HardwareGenerator} ${MPU})
  target_link_libraries(RPiSingleAPI PRIVATE Threads::Threads)

  if(${MPUDriver} MATCHES "spidev")
    # do not thing, spidev is linux build-in function
  elseif(${MPUDriver} MATCHES "pigpio")
    target_link_libraries(RPiSingleAPI PRIVATE ${PIGPIO})
  else()
    # this is spidev
  endif()
#================================================================
else()
  message(STATUS "[SingleFlight] API Link as multiple .a mode")
  aux_source_directory(. RPiSingleAPI)
  find_package(Threads REQUIRED)
  #================================================================
  add_library(RPiSingleAPI ${RPiSingleAPI})
  add_subdirectory(_thirdparty)
  target_link_libraries(RPiSingleAPI PRIVATE SingleFlight3rd)
  target_link_libraries(RPiSingleAPI PRIVATE Threads::Threads)
endif()