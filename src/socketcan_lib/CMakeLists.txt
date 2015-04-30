cmake_minimum_required(VERSION 2.8.3)
project(socketcan_lib)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include)

add_library(${PROJECT_NAME} 
  src/socketcan.cpp
	src/lib.cpp
	src/candriver.cpp
)

add_executable(cantest_node src/test_node.cpp)
add_executable(cantest_escrita src/test_escrita.cpp)
add_executable(cantest_leitura src/test_leitura.cpp)

target_link_libraries(cantest_node
	${catkin_LIBRARIES}
	${PROJECT_NAME}
)

target_link_libraries(cantest_escrita
	${catkin_LIBRARIES}
	${PROJECT_NAME}
)

target_link_libraries(cantest_leitura
	${catkin_LIBRARIES}
	${PROJECT_NAME}
)

