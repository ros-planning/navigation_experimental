cmake_minimum_required(VERSION 2.8.3)
project(sbpl_lattice_planner)

##############################################################################
# Find dependencies
##############################################################################

set(THIS_PACKAGE_ROS_DEPS roscpp costmap_2d nav_core pluginlib 
  geometry_msgs nav_msgs)
find_package(catkin REQUIRED COMPONENTS
  ${THIS_PACKAGE_ROS_DEPS} message_generation)

# Find SBPL
find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)

include_directories(include ${catkin_INCLUDE_DIRS} ${SBPL_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${SBPL_LIBRARY_DIRS})

##############################################################################
# Generate messages
##############################################################################

add_message_files(FILES SBPLLatticePlannerStats.msg)
generate_messages(DEPENDENCIES geometry_msgs)

##############################################################################
# Define package
##############################################################################

# Since sbpl is found through pkg-config, we need to manually add its
# libraries and include directories into catkin_package
catkin_package(
  INCLUDE_DIRS include ${SBPL_INCLUDE_DIRS}
  LIBRARIES ${SBPL_LIBRARIES}
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS} message_runtime
  DEPENDS
)

##############################################################################
# Build
##############################################################################

add_library(${PROJECT_NAME} src/sbpl_lattice_planner.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${SBPL_LIBRARIES} layers)

##############################################################################
# Install
##############################################################################

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES bgp_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
