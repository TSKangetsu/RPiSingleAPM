set(MPUDriver "spidev" CACHE  INTERNAL "")

execute_process(
  COMMAND git log -1 --format=%h
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_COMMIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
project(SingleFlight)
IF(UNIX)
###########################code Definetion##################
    add_definitions(-DRPiDEBUGStart)
    add_definitions(-DRPiDEBUG)
    add_definitions(-DSELF_SIG_MANAGE)
#############################################################
    cmake_minimum_required(VERSION 3.10)
    find_package(Threads REQUIRED)
    add_subdirectory(src)
    add_executable(SingleFlight "TestModule.cpp")
    target_compile_definitions(SingleFlight PRIVATE "-DGIT_COMMIT_HASH=\"${GIT_COMMIT_HASH}\"")
    target_link_libraries(SingleFlight PRIVATE  RPiSingleAPI)
    target_link_libraries(SingleFlight PRIVATE Threads::Threads)
    
    INSTALL(TARGETS SingleFlight DESTINATION "/usr/bin/")
    INSTALL(FILES APMconfig.json DESTINATION "/boot/")
    set(CPACK_PACKAGE_NAME "SingleFlight")
    set(CPACK_PACKAGE_VERSION "0.9.0BETA") 
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "pigpio,wiringpi")
    set(CPACK_PACKAGE_DESCRIPTION "SingleFlight linux flight controller software")
    set(CPACK_PACKAGE_CONTACT "TSKangetsu x403070567@live.com")
    include(CPack)
ENDIF()