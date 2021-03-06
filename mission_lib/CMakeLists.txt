cmake_minimum_required(VERSION 2.8.3)
project(mission_lib)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # message_generation
  std_msgs
  mavros
  mavros_msgs
  roscpp
  geodesy
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   State.msg
# )

# Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   SetHome.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs mavros_msgs
# )


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS mavros mavros_msgs roscpp geodesy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/mission_lib.cpp
)
## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_test_node src/mission_lib_test.cpp)
add_dependencies(${PROJECT_NAME}_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
