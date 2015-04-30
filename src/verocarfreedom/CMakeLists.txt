cmake_minimum_required(VERSION 2.8.3)
project(verocarfreedom)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin COMPONENTS geometry_msgs nav_msgs roscpp cv_bridge socketcan_lib tf verocarfreedomdefs verocarfreedomdefs_msgs watchdog sockutil)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp cv_bridge socketcan_lib tf verocarfreedomdefs verocarfreedomdefs_msgs watchdog sockutil
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
	include/${PROJECT_NAME}
)

## Declare a cpp library
add_library(${PROJECT_NAME} 
	src/driver.cpp
	src/cansafe.cpp
	src/encodermodel.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
	verocarfreedomdefs
)

## Declare a cpp executable
add_executable(verocarfreedom_test_node src/verocarfreedom_test_node.cpp)
add_executable(odom_roda_node src/odom_roda_node.cpp)

## Add dependencies to the executable
#add_dependencies(${PROJECT_NAME} verocarfreedomdefs)
add_dependencies(verocarfreedom_test_node ${PROJECT_NAME})
add_dependencies(odom_roda_node ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
target_link_libraries(verocarfreedom_test_node
  ${catkin_LIBRARIES}
	${PROJECT_NAME}
	#hokuyo_aist
	socketcan_lib
	verocarfreedomdefs
)

target_link_libraries(odom_roda_node
  ${catkin_LIBRARIES}
	${PROJECT_NAME}
	#hokuyo_aist
	socketcan_lib
	verocarfreedomdefs
	watchdog
)
