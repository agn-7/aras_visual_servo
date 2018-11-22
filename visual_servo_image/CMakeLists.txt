cmake_minimum_required(VERSION 2.8.3)
project(visual_servo_image)


find_package(Boost REQUIRED COMPONENTS)

catkin_package(
  CATKIN_DEPENDS roscpp
  LIBRARIES ${PROJECT_NAME}
  )


# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
add_executable(${PROJECT_NAME} src/main.cpp)

# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


include_directories(${Boost_INCLUDE_DIRS})

