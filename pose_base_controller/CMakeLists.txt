cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(pose_base_controller)

find_package(catkin REQUIRED COMPONENTS actionlib move_base_msgs nav_msgs roscpp tf2_geometry_msgs tf2_ros geometry_msgs)
find_package(Boost COMPONENTS system filesystem thread REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib move_base_msgs nav_msgs roscpp tf2_geometry_msgs tf2_ros geometry_msgs
)

add_executable(pose_base_controller src/pose_base_controller.cpp)
target_link_libraries(pose_base_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS pose_base_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES example.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
