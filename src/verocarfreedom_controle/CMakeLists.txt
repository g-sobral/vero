cmake_minimum_required(VERSION 2.8.3)
project(verocarfreedomdefs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp verocarfreedomdefs_msgs verocarfreedomdefs teleop_source_keyboard teleop_source_joystick sockutil watchdog)

###########
## Build ##
###########

## Declare a cpp executable
add_executable(verocarfreedom_controle_node
	src/verocarfreedom_controle.cpp
)

target_link_libraries(verocarfreedom_controle_node
  ${catkin_LIBRARIES}
	verocarfreedomdefs
)
