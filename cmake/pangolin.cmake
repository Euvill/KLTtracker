
find_package(Pangolin)

include_directories(${Pangolin_INCLUDE_DIRS})

list(APPEND ALL_TARGET_LIBRARIES ${Pangolin_LIBRARIES})
