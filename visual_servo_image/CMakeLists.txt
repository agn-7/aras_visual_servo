cmake_minimum_required(VERSION 2.8.3)
project(visual_servo_image)


# find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(catkin REQUIRED COMPONENTS
  OpenCV REQUIRED
  cv_bridge
  image_transport
  roscpp
  )


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

catkin_package(
  CATKIN_DEPENDS
    roscpp
    cv_bridge
    image_transport
  INCLUDE_DIRS
    include
  LIBRARIES ${PROJECT_NAME}
  )


# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
add_executable(${PROJECT_NAME} src/main.cpp src/Cam_Sensor.cpp src/Robot.cpp src/Controller.cpp)


# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)




