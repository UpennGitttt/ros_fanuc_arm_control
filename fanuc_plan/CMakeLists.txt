cmake_minimum_required(VERSION 3.0.2)
project(fanuc_plan)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  pluginlib
  geometric_shapes
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  roscpp
  std_msgs
  nav_msgs
  control_msgs
  controller_manager_msgs
)

catkin_package(
  # LIBRARIES
  # INCLUDE_DIRS
  # CATKIN_DEPENDS
  #   moveit_core
  #   moveit_visual_tools
  #   moveit_ros_planning_interface
  #   interactive_markers
  #   tf2_geometry_msgs
#  INCLUDE_DIRS include
#  LIBRARIES mm_plan
#  CATKIN_DEPENDS cmake_modules interactive_markers moveit_core moveit_ros_perception moveit_ros_planning_interface pluginlib roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  include "/usr/local/include/eigen3"
  ${catkin_INCLUDE_DIRS}
)


add_executable(fanuc_pick_place src/fanuc_pick_place.cpp
        src/fanuc_pick_place.cpp)
target_link_libraries(fanuc_pick_place ${catkin_LIBRARIES})


