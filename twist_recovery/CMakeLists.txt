cmake_minimum_required(VERSION 2.8.3)
project(twist_recovery)

# Find ROS dependencies
set(THIS_PACKAGE_ROS_DEPS nav_core costmap_2d geometry_msgs pluginlib base_local_planner)
find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES twist_recovery
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
)

add_library(twist_recovery src/twist_recovery.cpp)
target_link_libraries(twist_recovery ${catkin_LIBRARIES})

install(TARGETS twist_recovery
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES twist_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
