SET(NCURSES_ENABLE false  CACHE BOOL "ncurses console debug controller enable")
#================================================================
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
aux_source_directory(. RPiSingleAPI)
#================================================================
find_library(wiringPi_LIB wiringPi)

if(NCURSES_ENABLE)
    add_definitions(-DNCURSES_ENABLE)
    find_library(NCURSES ncurses)
endif()
#================================================================
add_library(RPiSingleAPI ${RPiSingleAPI})
add_subdirectory(_thirdparty)
target_link_libraries( RPiSingleAPI ${wiringPi_LIB})
target_link_libraries( RPiSingleAPI LIB)

if(NCURSES_ENABLE)
    target_link_libraries( RPiSingleAPI ${NCURSES})
endif()