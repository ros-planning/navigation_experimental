cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(pose_follower)
set(pose_follower_ROS_DEPS nav_core base_local_planner costmap_2d roscpp tf2_geometry_msgs tf2_ros nav_msgs pluginlib dynamic_reconfigure)

find_package(catkin REQUIRED COMPONENTS ${pose_follower_ROS_DEPS})

generate_dynamic_reconfigure_options(
  cfg/PoseFollower.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_follower
  CATKIN_DEPENDS ${pose_follower_ROS_DEPS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(pose_follower src/pose_follower.cpp)
add_dependencies(pose_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_follower ${catkin_LIBRARIES})

install(TARGETS pose_follower
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
