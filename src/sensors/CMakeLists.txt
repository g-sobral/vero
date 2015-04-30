cmake_minimum_required(VERSION 2.8.3)
project(sensors)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin COMPONENTS roscpp geometry_msgs sensor_msgs)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(joy_teleop src/joy_teleop.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(joy_teleop
  ${catkin_LIBRARIES}
)
