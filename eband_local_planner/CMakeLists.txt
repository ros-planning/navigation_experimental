cmake_minimum_required(VERSION 2.8.3)
project(eband_local_planner)

# Before this can be catkinized, the control_toolbox needs to be catknized.

#find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs)

#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})

#rosbuild_genmsg()

# we are using the boost::bind library
#rosbuild_add_boost_directories()

# build the library
#rosbuild_add_library(eband_local_planner src/eband_local_planner_ros.cpp src/eband_local_planner.cpp src/conversions_and_types.cpp src/eband_visualization.cpp src/eband_trajectory_controller.cpp)
#target_link_libraries(eband_local_planner tinyxml)

#rosbuild_download_data(https://code.ros.org/svn/data/trunk/willow_maps/willow-full-0.025.pgm launch/stage_launch/willow-full-0.025.pgm e66b17ee374f2d7657972efcb3e2e4f7)

#rosbuild_add_executable(eband_action src/eband_action.cpp)
#target_link_libraries(eband_action eband_local_planner)
