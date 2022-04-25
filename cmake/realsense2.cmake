find_package(realsense2 REQUIRED)

include_directories(${realsense2_INCLUDE_DIRS})

list(APPEND ALL_TARGET_LIBRARIES ${realsense2_LIBRARY})
