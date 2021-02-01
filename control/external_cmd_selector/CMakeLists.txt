cmake_minimum_required(VERSION 3.0.2)
project(external_cmd_selector)
add_compile_options(-std=c++14)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  autoware_vehicle_msgs
  autoware_control_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    autoware_vehicle_msgs
    autoware_control_msgs
    geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(external_cmd_selector_node
  src/external_cmd_selector/external_cmd_selector_node.cpp
  src/external_cmd_selector/main.cpp
)
add_dependencies(external_cmd_selector_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(external_cmd_selector_node ${catkin_LIBRARIES})

install(TARGETS
    external_cmd_selector_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
