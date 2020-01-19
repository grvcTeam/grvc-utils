cmake_minimum_required(VERSION 2.8.3)
project(rtkbasestation)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	mavros_msgs
	)

catkin_package(
	CATKIN_DEPENDS roscpp)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(rtkbase src/rtcm_mavros.cpp)

target_link_libraries(rtkbase ${catkin_LIBRARIES})
