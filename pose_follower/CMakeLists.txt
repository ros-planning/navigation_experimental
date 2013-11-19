cmake_minimum_required(VERSION 2.8.3)
project(pose_follower)
set(pose_follower_ROS_DEPS nav_core base_local_planner costmap_2d roscpp tf nav_msgs pluginlib)

find_package(catkin REQUIRED COMPONENTS ${pose_follower_ROS_DEPS})

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${pose_follower_ROS_DEPS}
)

add_library(pose_follower src/pose_follower.cpp)
target_link_libraries(pose_follower ${catkin_LIBRARIES})

install(TARGETS pose_follower
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


