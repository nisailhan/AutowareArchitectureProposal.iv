cmake_minimum_required(VERSION 3.0.2)
project(motion_velocity_smoother)

add_compile_options(-std=c++14 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  autoware_planning_msgs
  autoware_utils
  geometry_msgs
  tf2
  tf2_ros
  dynamic_reconfigure
  osqp_interface
)

generate_dynamic_reconfigure_options(cfg/MotionVelocitySmoother.cfg)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    autoware_planning_msgs
    autoware_utils
    geometry_msgs
    tf2
    tf2_ros
    osqp_interface
  LIBRARIES
    motion_velocity_smoother
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


set(MOTION_VELOCITY_SMOOTHER_SRC
  src/motion_velocity_smoother.cpp
  src/trajectory_utils.cpp
  src/linear_interpolation.cpp
)

set(SMOOTHER_SRC
  src/smoother/smoother_base.cpp
  src/smoother/l2_pseudo_jerk_smoother.cpp
  src/smoother/linf_pseudo_jerk_smoother.cpp
  src/smoother/jerk_filtered_smoother.cpp
)

add_library(smoother ${SMOOTHER_SRC})
add_executable(motion_velocity_smoother src/main.cpp ${MOTION_VELOCITY_SMOOTHER_SRC})
add_dependencies(motion_velocity_smoother ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} motion_velocity_smoother_gencfg)
target_link_libraries(motion_velocity_smoother smoother ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})

install(
  TARGETS
    motion_velocity_smoother
    smoother
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY
    scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
