cmake_minimum_required(VERSION 2.8.3)
project(lidar_centerpoint)

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        autoware_perception_msgs
        tf2_geometry_msgs
)

set(VERBOSE OFF)
set(CMAKE_CXX_STANDARD 14)


# CUDA
find_package(CUDA)
if (CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS ${CUDA_TOOLKIT_ROOT_DIR}/lib64 ${CUDA_TOOLKIT_ROOT_DIR}/lib)
  if (VERBOSE)
      message(STATUS "CUDA_LIBRARIES: ${CUDA_LIBRARIES}")
      message(STATUS "CUBLAS_LIBRARIES: ${CUBLAS_LIBRARIES}")
  endif ()
  # Note: cublas_device was depreciated in CUDA version 9.2
  #       https://forums.developer.nvidia.com/t/where-can-i-find-libcublas-device-so-or-libcublas-device-a/67251/4
  #       In LibTorch, CUDA_cublas_device_LIBRARY is used.
  unset(CUDA_cublas_device_LIBRARY CACHE)
endif ()

# TensorRT
find_library(NVINFER_LIBRARIES nvinfer)
find_library(NVINFER_PLUGIN_LIBRARIES nvinfer_plugin)
find_Library(NVONNXPARSER_LIBRARIES nvonnxparser)
if (VERBOSE)
    message(STATUS "NVINFER_LIBRARIES: ${NVINFER_LIBRARIES}")
    message(STATUS "NVINFER_PLUGIN_LIBRARIES: ${NVINFER_PLUGIN_LIBRARIES}")
    message(STATUS "NVONNXPARSER_LIBRARIES: ${NVONNXPARSER_LIBRARIES}")
endif ()

# LibTorch
if (CUDA_FOUND)
  set(Torch_DIR /usr/local/libtorch/share/cmake/Torch)
  find_package(Torch)
  if (VERBOSE)
      message(STATUS "TORCH_INCLUDE_DIRS: ${TORCH_INCLUDE_DIRS}")
      message(STATUS "TORCH_LIBRARIES: ${TORCH_LIBRARIES}")
  endif ()
endif ()

function(download FILE_NAME GFILE_ID FILE_HASH)
    message(STATUS "Checking and downloading ${FILE_NAME}")
    set(FILE_PATH ${DATA_PATH}/${FILE_NAME})
    if (EXISTS ${FILE_PATH})
        file(MD5 ${FILE_PATH} EXISTING_FILE_HASH)
        if (NOT ${FILE_HASH} EQUAL ${EXISTING_FILE_HASH})
            message(STATUS "... file hash changes. Downloading now ...")
            execute_process(COMMAND gdown --quiet https://drive.google.com//uc?id=${GFILE_ID} -O ${FILE_PATH})
        endif()
    else()
        message(STATUS "... file doesn't exists. Downloading now ...")
        execute_process(COMMAND gdown --quiet https://drive.google.com//uc?id=${GFILE_ID} -O ${FILE_PATH})
    endif ()
endfunction()

if (CUDA_FOUND AND NVINFER_LIBRARIES AND NVINFER_PLUGIN_LIBRARIES AND NVONNXPARSER_LIBRARIES AND TORCH_FOUND)
  # Download trained models
  find_program(GDOWN_AVAIL "gdown")
  if (NOT GDOWN_AVAIL)
      message("gdown: command not found. External files could not be downloaded.")
  endif ()

  set(DATA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/data)
  execute_process(COMMAND mkdir -p ${DATA_PATH})

  download(centerpoint_vfe.pt 1NUOn3GILXO2uTVjONBHRZYLKLiLirBpK 825dc48c799d05b3785ac314afe49c0b)
  download(centerpoint_head.pt 1WxXtxKYWkhwW90MRHnHxqa1QkHu_bjFk 3164deaa33f3ec456379b53137808384)
  download(centerpoint_vfe.onnx 1TsNwIYVPat-rZxRRVhW7DpNWu2W7srox 14cc03fab1670267f1872edfa1b83a48)
  download(centerpoint_head.onnx 1nmdNpmnhHa15VsoCaHoh0oreL6pDux_k d27881e2c72363e2cd765358f9f479bc)

  catkin_package(
          INCLUDE_DIRS
          include
          CATKIN_DEPENDS
          roscpp
          pcl_ros
  )

  include_directories(
          include
          lib/include
          ${catkin_INCLUDE_DIRS}
          ${CUDA_INCLUDE_DIRS}
          ${TORCH_INCLUDE_DIRS}
  )

  ### centerpoint ###
  add_library(
          centerpoint
          lib/src/pointcloud_densification.cpp
          lib/src/voxel_generator.cpp
          lib/src/centerpoint_trt.cpp
          lib/src/tensorrt_wrapper.cpp
          lib/src/network_trt.cpp)

  target_link_libraries(
          centerpoint
          ${catkin_LIBRARIES}
          ${CUDA_LIBRARIES}
          ${CUBLAS_LIBRARIES}
          ${NVINFER_LIBRARIES}
          ${NVINFER_PLUGIN_LIBRARIES}
          ${NVONNXPARSER_LIBRARIES}
          ${TORCH_LIBRARIES}
  )

  ## node ##
  add_executable(
          lidar_centerpoint_node
          src/main.cpp
          src/node.cpp
  )

  target_link_libraries(
          lidar_centerpoint_node
          centerpoint
  )

  add_dependencies(lidar_centerpoint_node ${catkin_EXPORTED_TARGETS})

  install(
          TARGETS
          lidar_centerpoint_node
          centerpoint
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
          )

  install(DIRECTORY launch/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
          )

  install(DIRECTORY data/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
          )
else()
  find_package(catkin REQUIRED)
  catkin_package()

  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
endif ()
