cmake_minimum_required(VERSION 2.8.3)
project(carebot_auto_gmapping)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  tf
  geometry_msgs
  sensor_msgs
  nav_msgs
)

set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    roslib
    roscpp
    tf
    geometry_msgs
    sensor_msgs
    nav_msgs
)

####################### Obstacle avoidance ##################################
add_executable(carebot_automove_node
  src/main.cpp
  src/obstacle_avoidance.cpp)
target_link_libraries(carebot_automove_node
  ${catkin_LIBRARIES}
)

# Install excecutables
install(TARGETS carebot_automove_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

