aux_source_directory(. LIB)
add_subdirectory(RaspberryPiMPU)
add_library(LIB ${LIB})
target_link_libraries(LIB RPiMPU)