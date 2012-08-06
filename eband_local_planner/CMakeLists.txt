cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

# we are using the boost::bind library
rosbuild_add_boost_directories()

# build the library
rosbuild_add_library(eband_local_planner src/eband_local_planner_ros.cpp src/eband_local_planner.cpp src/conversions_and_types.cpp src/eband_visualization.cpp src/eband_trajectory_controller.cpp)
target_link_libraries(eband_local_planner tinyxml)

rosbuild_download_data(https://code.ros.org/svn/data/trunk/willow_maps/willow-full-0.025.pgm launch/stage_launch/willow-full-0.025.pgm e66b17ee374f2d7657972efcb3e2e4f7)

rosbuild_add_executable(eband_action src/eband_action.cpp)
target_link_libraries(eband_action eband_local_planner)
