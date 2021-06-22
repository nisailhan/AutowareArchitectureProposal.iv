cmake_minimum_required(VERSION 3.0.2)
project(signal_processing)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  rosunit
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    signal_processing
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(lowpass_filter_1d src/lowpass_filter_1d.cpp)

target_link_libraries(lowpass_filter_1d
  ${catkin_LIBRARIES}
)

add_dependencies(lowpass_filter_1d
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS
    lowpass_filter_1d
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Test
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  file(GLOB_RECURSE test_files test/**/*.cpp)
  add_rostest_gtest(test_signal_processing test/test_signal_processing.test ${test_files})
  target_link_libraries(test_signal_processing
    lowpass_filter_1d
    ${catkin_LIBRARIES}
  )
endif()
