cmake_minimum_required(VERSION 3.0.2)
project(planning_error_monitor)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  autoware_utils
  geometry_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES planning_error_monitor
   CATKIN_DEPENDS autoware_planning_msgs autoware_utils geometry_msgs roscpp std_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(planning_error_monitor
  src/main.cpp
  src/planning_error_monitor.cpp
)

add_dependencies(planning_error_monitor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(planning_error_monitor
  ${catkin_LIBRARIES}
)

install(TARGETS planning_error_monitor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Test
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_planning_error_monitor
    test/test_planning_error_monitor.test
    test/src/test_main.cpp
    test/src/test_planning_error_monitor_functions.cpp
    test/src/test_planning_error_monitor_pubsub.cpp
    src/planning_error_monitor.cpp
  )

  target_link_libraries(test_planning_error_monitor
    ${catkin_LIBRARIES}
  )
endif ()
