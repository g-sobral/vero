cmake_minimum_required(VERSION 2.8.3)
project(nodelet)

## Find catkin dependencies
find_package(catkin REQUIRED bondcpp message_generation pluginlib rosconsole roscpp std_msgs sensor_msgs geometry_msgs cv_bridge sockutil image_transport)

## Find Boost (only headers)
find_package(Boost REQUIRED)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   ESMTrack.msg
 )


## Add service files to be generated
add_service_files(DIRECTORY srv FILES NodeletList.srv  NodeletLoad.srv  NodeletUnload.srv)

## Generate messages
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nodeletlib uuid
  CATKIN_DEPENDS bondcpp message_runtime pluginlib rosconsole roscpp std_msgs geometry_msgs sockutil image_transport
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})

# Debug only, collects stats on how callbacks are doled out to worker threads
#add_definitions(-DNODELET_QUEUE_DEBUG)

add_library(nodeletlib src/nodelet_class.cpp src/loader.cpp src/callback_queue.cpp src/callback_queue_manager.cpp )
target_link_libraries(nodeletlib ${catkin_LIBRARIES} ${BOOST_LIBRARIES})
add_dependencies(nodeletlib ${nodelet_EXPORTED_TARGETS})

add_executable(nodeletesm src/nodelet.cpp src/esmlib041.cpp )
# it is a complete overlay, so we keep a exec with the old name for other stuff
add_executable(nodelet src/nodelet.cpp src/esmlib041.cpp )
# did not work: SET (ESMLIB -Wl,-whole-archive,-Bstatic /usr/lib/ESMlib.a -Wl,-no_whole-archive,-Bdynamic)
# what worked: it is really stupid:
# copy ESMlib.a into /usr/lib/libESMlib.a 
# (use "lib" in the beggining of the file name)
target_link_libraries(nodeletesm nodeletlib uuid ${catkin_LIBRARIES} ${BOOST_LIBRARIES} dvsutil /usr/lib/libESMlib.a  )
target_link_libraries(nodelet nodeletlib uuid ${catkin_LIBRARIES} ${BOOST_LIBRARIES} dvsutil /usr/lib/libESMlib.a  )

## install
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS nodeletlib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS nodeletesm nodelet
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
