cmake_minimum_required(VERSION 3.0.2)
project(behavior_path_planner)

add_compile_options(-std=c++14 -Wall)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  autoware_planning_msgs
  autoware_vehicle_msgs
  autoware_utils
  spline_interpolation
  geometry_msgs
  lanelet2_extension
  roscpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_package(OpenCV REQUIRED)
find_package(BehaviorTreeV3)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(behavior_path_planner
  src/main.cpp
  src/behavior_path_planner.cpp
  src/behavior_tree_manager.cpp
  src/route_handler.cpp
  src/utilities.cpp
  src/path_utilities.cpp
  src/path_shifter/path_shifter.cpp
  src/turn_signal_decider.cpp
  src/scene_module/scene_module_bt_node_interface.cpp
  src/scene_module/lane_following/lane_following_module.cpp
  src/scene_module/lane_change/lane_change_module.cpp
  src/scene_module/lane_change/util.cpp
)
add_dependencies(behavior_path_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_path_planner
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  BT::behaviortree_cpp_v3
)

install(
  TARGETS
  behavior_path_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
