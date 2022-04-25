CMAKE_MINIMUM_REQUIRED(VERSION 3.10.2)
 
PROJECT(KLTtracker)
 
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

add_compile_options(-std=c++14)
add_definitions(-std=c++14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

set(ALL_TARGET_LIBRARIES "")
include_directories(include)
include(cmake/opencv.cmake)
include(cmake/pangolin.cmake)
include(cmake/realsense2.cmake)

file(GLOB_RECURSE ALL_SRCS "${PROJECT_SOURCE_DIR}/*.cpp")
file(GLOB_RECURSE NODE_SRCS "${PROJECT_SOURCE_DIR}/src/*_app.cpp")
file(GLOB_RECURSE COMPILE_SRCS "${PROJECT_SOURCE_DIR}/*CompilerId.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS} ${COMPILE_SRCS})

add_executable(KLTtracker src/KLTtracker_app.cpp ${ALL_SRCS})
target_link_libraries(KLTtracker ${ALL_TARGET_LIBRARIES})

#add_executable(PangolinViewer src/PangolinViewer_app.cpp ${ALL_SRCS})
#target_link_libraries(PangolinViewer ${ALL_TARGET_LIBRARIES})
