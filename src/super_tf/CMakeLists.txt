cmake_minimum_required(VERSION 2.8.3)
project(super_tf)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin COMPONENTS roscpp geometry_msgs nav_msgs tf verocarfreedomdefs_msgs)

catkin_package(
			LIBRARIES ${PROJECT_NAME}

)
###########
## Build ##
###########
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(super_tf_node src/super_tf_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(super_tf_node
  ${catkin_LIBRARIES}
)
