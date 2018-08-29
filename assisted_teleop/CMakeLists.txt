cmake_minimum_required(VERSION 2.8.3)
project(assisted_teleop)

##############################################################################
# Find dependencies
##############################################################################

set(THIS_PACKAGE_ROS_DEPS
  tf
  costmap_2d
  roscpp
  roslib
  geometry_msgs
  move_base_msgs
  actionlib
  message_filters
  base_local_planner
  angles
  pluginlib
  sensor_msgs
  filters
)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

##############################################################################
# Define package
##############################################################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laser_scan_filters
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS EIGEN3
)

##############################################################################
# Build
##############################################################################

add_executable(assisted_teleop src/assisted_teleop.cpp)
target_link_libraries(assisted_teleop ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_library(laser_scan_filters src/laser_scan_filters.cpp)
target_link_libraries(laser_scan_filters ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

##############################################################################
# Install
##############################################################################

install(TARGETS assisted_teleop laser_scan_filters
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES filter_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
