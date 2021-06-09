cmake_minimum_required(VERSION 3.0.2)
project(control_performance_analysis)

add_compile_options(-std=c++14)
set(DEBUG_OPTIONS -Wall -Wextra -Wpedantic)


find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        autoware_control_msgs
        autoware_planning_msgs
        autoware_vehicle_msgs
        autoware_utils
        geometry_msgs
        std_msgs
        tf2
        tf2_eigen
        tf2_ros
        message_generation


)

## Message Generation
## Generate messages in the 'msg' folder
add_message_files(
        DIRECTORY msg
        FILES
        Error.msg
        ErrorStamped.msg
)
generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        autoware_control_msgs
        autoware_planning_msgs
        autoware_vehicle_msgs
        autoware_utils
        geometry_msgs
        std_msgs
        sensor_msgs
        message_runtime
        tf2
        tf2_eigen
        tf2_ros
)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_library(controller_performance_core
        src/control_performance_utils.cpp
        src/controller_performance_core.cpp)

target_link_libraries(controller_performance_core
        ${catkin_LIBRARIES})

add_dependencies(controller_performance_core
        ${catkin_EXPORTED_TARGETS})


# Controller Performance Node Instructions
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(control_performance_analysis
        src/controller_performance_node.cpp
        src/main.cpp)

target_link_libraries(control_performance_analysis
        controller_performance_core
        ${catkin_LIBRARIES})

add_dependencies(control_performance_analysis
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

## Install Instruction
install(
        TARGETS
        controller_performance_core
        control_performance_analysis
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install files
install(
        DIRECTORY
        config
        msg
        launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
