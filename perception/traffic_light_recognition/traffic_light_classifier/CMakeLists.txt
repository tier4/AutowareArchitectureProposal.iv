cmake_minimum_required(VERSION 3.0.2)
project(traffic_light_classifier)

add_compile_options(-std=c++14)

option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)
# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if (CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if (CUDA_VERBOSE)
    message(STATUS "CUDA is available!")
    message(STATUS "CUDA Libs: ${CUDA_LIBRARIES}")
    message(STATUS "CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif ()
  set(CUDA_AVAIL ON)
else()
  message(STATUS "CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif (CUDA_FOUND)

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER NAMES nvinfer)
find_library(NVONNXPARSER nvonnxparser)
find_library(NVINFER_PLUGIN NAMES nvinfer_plugin)
if(NVINFER AND NVONNXPARSER AND NVINFER_PLUGIN)
  if (CUDA_VERBOSE)
    message(STATUS "TensorRT is available!")
    message(STATUS "NVINFER: ${NVINFER}")
    message(STATUS "NVPARSERS: ${NVPARSERS}")
    message(STATUS "NVINFER_PLUGIN: ${NVINFER_PLUGIN}")
    message(STATUS "NVONNXPARSER: ${NVONNXPARSER}")
  endif ()
  set(TRT_AVAIL ON)
else()
  message(STATUS "TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

# set flags for CUDNN availability
option(CUDNN_AVAIL "CUDNN available" OFF)
# try to find the CUDNN module
find_library(CUDNN_LIBRARY
  NAMES libcudnn.so${__cudnn_ver_suffix} libcudnn${__cudnn_ver_suffix}.dylib ${__cudnn_lib_win_name}
  PATHS $ENV{LD_LIBRARY_PATH} ${__libpath_cudart} ${CUDNN_ROOT_DIR} ${PC_CUDNN_LIBRARY_DIRS} ${CMAKE_INSTALL_PREFIX}
  PATH_SUFFIXES lib lib64 bin
  DOC "CUDNN library." )
if(CUDNN_LIBRARY)
  if (CUDA_VERBOSE)
    message(STATUS "CUDNN is available!")
    message(STATUS "CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  endif ()
  set(CUDNN_AVAIL ON)

else()
  message(STATUS "CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

# Download caffemodel and prototxt
set(PRETRAINED_MODEL_LINK "https://drive.google.com/uc?id=15CQceCn9TZDU6huKJacQvUnDiLHionb3")
set(PRETRAINED_MODEL_HASH 7dc31c696b0400ddfc2cc5521586fa51)
set(LAMP_LABEL_LINK "https://drive.google.com/uc?id=1D7n3oGSWLkWgxET6PcWqEzOhmmPcqM52")
set(LAMP_LABEL_HASH 20167c8e9a1f9d2ec7b0b0088c4100f0)

find_program(GDOWN_AVAIL "gdown")
if (NOT GDOWN_AVAIL)
  message(STATUS "gdown: command not found. External files could not be downloaded.")
endif()
set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
if (NOT EXISTS "${PATH}")
  execute_process(COMMAND mkdir -p ${PATH})
endif()
set(FILE "${PATH}/traffic_light_classifier_mobilenetv2.onnx")
message(STATUS "Checking and downloading traffic_light_classifier_mobilenetv2.onnx")
if (EXISTS "${FILE}")
  file(MD5 "${FILE}" EXISTING_FILE_HASH)
  if (NOT "${PRETRAINED_MODEL_HASH}" EQUAL "${EXISTING_FILE_HASH}")
    message(STATUS "... file hash changed. Downloading now ...")
    execute_process(COMMAND gdown --quiet "${PRETRAINED_MODEL_LINK}" -O ${PATH}/traffic_light_classifier_mobilenetv2.onnx)
  endif()
else()
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND gdown --quiet "${PRETRAINED_MODEL_LINK}" -O ${PATH}/traffic_light_classifier_mobilenetv2.onnx)
endif()

set(FILE "${PATH}/lamp_labels.txt")
message(STATUS "Checking and downloading lamp_labels.txt")
if (EXISTS "${FILE}")
  file(MD5 "${FILE}" EXISTING_FILE_HASH)
  if (NOT "${LAMP_LABEL_HASH}" EQUAL "${EXISTING_FILE_HASH}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND gdown --quiet "${LAMP_LABEL_LINK}" -O ${PATH}/lamp_labels.txt)
  endif()
else()
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND gdown --quiet "${LAMP_LABEL_LINK}" -O ${PATH}/lamp_labels.txt)
endif()

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  cv_bridge
  roscpp
  sensor_msgs
  image_transport
  message_filters
  dynamic_reconfigure
  nodelet
  )

generate_dynamic_reconfigure_options(cfg/HSVFilter.cfg)

find_package(OpenCV REQUIRED)

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  add_definitions(-DENABLE_GPU)

  include_directories(
    include
    utils
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    )

  add_library(libutils
    utils/trt_common.cpp
    )
  target_link_libraries(libutils
    ${OpenCV_LIBRARIES}
    ${NVINFER}
    ${NVONNXPARSER}
    ${NVINFER_PLUGIN}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${CUDNN_LIBRARY}
    )
  add_dependencies(libutils
    ${catkin_EXPORTED_TARGETS})

  add_library(traffic_light_classifier_nodelet SHARED
    src/color_classifier.cpp
    src/cnn_classifier.cpp
    src/nodelet.cpp
    )

  add_dependencies(traffic_light_classifier_nodelet
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

  target_link_libraries(traffic_light_classifier_nodelet
    libutils
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

  add_executable(traffic_light_classifier_node
    src/node.cpp
    )

  add_dependencies(traffic_light_classifier_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

  target_link_libraries(traffic_light_classifier_node
    libutils
    traffic_light_classifier_nodelet
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

  install(
    TARGETS
    traffic_light_classifier_node
    traffic_light_classifier_nodelet
    libutils
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

  install(
    FILES
    traffic_light_classifier.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

  install(
    DIRECTORY
    launch
    data
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

else()
  message(STATUS "CUDA and/or TensorRT were not found. build only color classifier")

  include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    )

  add_library(traffic_light_classifier_nodelet SHARED
    src/color_classifier.cpp
    src/nodelet.cpp
    )

  add_dependencies(traffic_light_classifier_nodelet
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

  target_link_libraries(traffic_light_classifier_nodelet
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

  add_executable(traffic_light_classifier_node
    src/node.cpp
    )

  add_dependencies(traffic_light_classifier_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

  target_link_libraries(traffic_light_classifier_node
    traffic_light_classifier_nodelet
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

  install(
    TARGETS
    traffic_light_classifier_node
    traffic_light_classifier_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

  install(
    FILES
    traffic_light_classifier.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

  install(
    DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

endif()
