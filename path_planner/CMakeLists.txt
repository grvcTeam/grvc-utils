cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# add_definitions(-DVERBOSE)                    # Uncoment for outputting hints or warnings in the terminal when the path couldn't be found.
# add_definitions(-DWRITE_RESULTS_IN_TERMINAL)  # Uncoment for outputting in the terminal the path of points, path distance, computation time and other useful information.
# add_definitions(-DDRAW_IN_TERMINAL)           # Uncoment for drawing in the terminal the path with the geofence and map of obstacles. Draw both in the constructor and in the getPath method, in this one with the path.
# add_definitions(-DPLOT_GRAPH)                 # Uncoment for plotting a graphic (using matplotlib-cpp) of the path (grid path and real path), geofence and map of obstacles. Plot both in the constructor and in the getPath method, in this one with the path.

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  rospy
  roslib
  geodesy
)

find_package(PythonLibs 2.7)

# set(CURL_LIBRARY "-lcurl")
find_package(CURL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS std_msgs roscpp rospy roslib geodesy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
# include_directories(${CURL_INCLUDE_DIR})

add_library(${PROJECT_NAME} src/path_planner.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PYTHON_LIBRARIES} ${CURL_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(path_planner_tester src/path_planner_tester.cpp)
target_include_directories(path_planner_tester PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(path_planner_tester ${PROJECT_NAME} ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
add_dependencies(path_planner_tester ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
