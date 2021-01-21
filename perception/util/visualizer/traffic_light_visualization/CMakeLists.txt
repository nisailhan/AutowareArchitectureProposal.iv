cmake_minimum_required(VERSION 3.0.2)
project(traffic_light_visualization)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_lanelet2_msgs
  autoware_perception_msgs
  cv_bridge
  image_transport
  message_filters
  roscpp
  lanelet2_extension
  sensor_msgs
  nodelet
)

find_package(OpenCV REQUIRED)


catkin_package(
  CATKIN_DEPENDS
    autoware_lanelet2_msgs
    autoware_perception_msgs
    cv_bridge
    image_transport
    message_filters
    roscpp
    lanelet2_extension
    sensor_msgs
    nodelet
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

###########
## Build ##
###########

add_library(traffic_light_roi_visualizer_nodelet SHARED
  src/traffic_light_roi_visualizer/nodelet.cpp
)

add_dependencies(traffic_light_roi_visualizer_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(traffic_light_roi_visualizer_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(traffic_light_roi_visualizer_node
  src/traffic_light_roi_visualizer/node.cpp
)

add_dependencies(traffic_light_roi_visualizer_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(traffic_light_roi_visualizer_node
  traffic_light_roi_visualizer_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(traffic_light_map_visualizer_node
  src/traffic_light_map_visualizer/node.cpp
  src/traffic_light_map_visualizer/main.cpp
)

add_dependencies(traffic_light_map_visualizer_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(traffic_light_map_visualizer_node
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(
  TARGETS
    traffic_light_roi_visualizer_node
    traffic_light_roi_visualizer_nodelet
    traffic_light_map_visualizer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
  traffic_light_roi_visualizer.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
