cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(sbpl_recovery)

##############################################################################
# Find dependencies
##############################################################################

set(THIS_PACKAGE_ROS_DEPS roscpp nav_core sbpl_lattice_planner pose_follower
  costmap_2d pluginlib base_local_planner tf2_ros)
find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})

include_directories(include ${catkin_INCLUDE_DIRS})

##############################################################################
# Define package
##############################################################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sbpl_recovery
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
)

##############################################################################
# Build
##############################################################################

add_library(sbpl_recovery src/sbpl_recovery.cpp)
target_link_libraries(sbpl_recovery ${catkin_LIBRARIES})
target_compile_options(sbpl_recovery PUBLIC "-Wno-terminate")  # suppress warning from included SBPL header

##############################################################################
# Install
##############################################################################

install(TARGETS sbpl_recovery
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES recovery_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
