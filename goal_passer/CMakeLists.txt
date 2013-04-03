cmake_minimum_required(VERSION 2.8.3)
project(goal_passer)

set(goal_passer_ROS_DEPS roscpp costmap_2d pluginlib nav_core)

find_package(catkin REQUIRED COMPONENTS ${goal_passer_ROS_DEPS})

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LINK_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${goal_passer_ROS_DEPS}
)

add_library(goal_passer src/goal_passer.cpp)
target_link_libraries(goal_passer ${catkin_LIBRARIES})

install(TARGETS goal_passer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
