cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Boost REQUIRED COMPONENTS)
include_directories(${Boost_INCLUDE_DIRS})
#link_directories(/usr/local/include)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(visual_servo_image src/main.cpp)
rosbuild_add_executable(visual_servo_image src/Cam_Sensor.cpp)
rosbuild_add_executable(visual_servo_image src/Robot.cpp)
rosbuild_add_executable(visual_servo_image src/Controller.cpp)
#target_link_libraries(visual_servo_image   ${QT_LIBRARIES})
