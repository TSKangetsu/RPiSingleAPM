set(MPUDriver "spidev" CACHE  INTERNAL "")
set(FULLSTATIC TRUE CACHE  INTERNAL bool)
set(OPENWRT FALSE CACHE  INTERNAL bool)

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
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os -s -Wstack-usage=1000")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -s -Wstack-usage=1000")
project(SingleFlight)


IF(FULLSTATIC)
  set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
  set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
  set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
  set(BUILD_SHARED_LIBS OFF)
  set(CMAKE_EXE_LINKER_FLAGS "-static -static-libstdc++ -static-libgcc")
  set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
ENDIF()

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
    target_link_libraries(SingleFlight PRIVATE RPiSingleAPI)
    target_link_libraries(SingleFlight PRIVATE Threads::Threads)

IF(OPENWRT)
    target_link_libraries(SingleFlight PRIVATE gcc_pic) # this option must link with musl in openwrt
ENDIF()

ENDIF()