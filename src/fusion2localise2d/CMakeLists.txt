cmake_minimum_required(VERSION 2.4.6)
project(fusion2localise2d)
find_package(catkin REQUIRED COMPONENTS cv_bridge image_geometry  roscpp)

catkin_package(
	LIBRARIES ${PROJECT_NAME}
#	DEPENDS OpenCV
)

include_directories(${OpenCV_INCLUDE_DIRS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

add_executable(fusion2localise_node src/fusion2localise2d_node.cpp src/fusion2localise2d_node.h src/odomBasedDriver.cpp src/filter.cpp)
target_link_libraries(fusion2localise_node ${catkin_LIBRARIES} ${OpenCV_Libs})
