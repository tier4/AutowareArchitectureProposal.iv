cmake_minimum_required(VERSION 3.5)
project(traffic_light_classifier)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)
set(CUDA_VERBOSE ON)
# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA QUIET)
if (CUDA_FOUND)
  find_package(CUDA REQUIRED)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if (CUDA_VERBOSE)
    message("CUDA is available!")
    message("CUDA Libs: ${CUDA_LIBRARIES}")
    message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif ()
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif (CUDA_FOUND)

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
set(TENSORRT_ROOT_DIR /home/nithilan/TensorRT-7.2.0.14.Ubuntu-18.04.x86_64-gnu.cuda-11.0.cudnn8.0/TensorRT-7.2.0.14)
set(TENSORRT_INCLUDE_DIRS /home/nithilan/TensorRT-7.2.0.14.Ubuntu-18.04.x86_64-gnu.cuda-11.0.cudnn8.0/TensorRT-7.2.0.14/include)
find_library(NVINFER NAMES libnvinfer.so${__tensorrt_ver_suffix}
  PATHS ${TENSORRT_ROOT_DIR}
  PATH_SUFFIXES lib lib64 bin
  DOC "TENSORRT library." )
find_library(NVONNXPARSER NAMES libnvonnxparser.so${__tensorrt_ver_suffix}
  PATHS ${TENSORRT_ROOT_DIR}
  PATH_SUFFIXES lib lib64 bin
  DOC "TENSORRT library." )
find_library(NVINFER_PLUGIN NAMES libnvinfer_plugin.so${__tensorrt_ver_suffix}
  PATHS ${TENSORRT_ROOT_DIR}
  PATH_SUFFIXES lib lib64 bin
  DOC "TENSORRT library." )
if(NVINFER AND NVONNXPARSER AND NVINFER_PLUGIN)
  if (CUDA_VERBOSE)
    message("TensorRT is available!")
    message("NVINFER: ${NVINFER}")
    message("NVPARSERS: ${NVPARSERS}")
    message("NVINFER_PLUGIN: ${NVINFER_PLUGIN}")
    message("NVONNXPARSER: ${NVONNXPARSER}")
  endif ()
  set(TRT_AVAIL ON)
else()
  message("TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

# set flags for CUDNN availability
option(CUDNN_AVAIL "CUDNN available" OFF)
# try to find the CUDNN module
set(CUDNN_ROOT_DIR /usr/local/cuda)
find_library(CUDNN_LIBRARY
  NAMES libcudnn.so${__cudnn_ver_suffix} libcudnn${__cudnn_ver_suffix}.dylib ${__cudnn_lib_win_name}
  PATHS $ENV{LD_LIBRARY_PATH} ${__libpath_cudart} ${CUDNN_ROOT_DIR} ${PC_CUDNN_LIBRARY_DIRS} ${CMAKE_INSTALL_PREFIX}
  PATH_SUFFIXES lib lib64 bin
  DOC "CUDNN library." )
if(CUDNN_LIBRARY)
  if (CUDA_VERBOSE)
    message("CUDNN is available!")
    message("CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  endif ()
  set(CUDNN_AVAIL ON)

else()
  message("CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

# Download caffemodel and prototxt
find_program(GDOWN_AVAIL "gdown")
if (NOT GDOWN_AVAIL)
  message("gdown: command not found. External files could not be downloaded.")
endif()
set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
if (NOT EXISTS "${PATH}")
  execute_process(COMMAND mkdir -p ${PATH})
endif()
set(FILE "${PATH}/traffic_light_classifier_mobilenetv2.onnx")
message(STATUS "Checking and downloading traffic_light_classifier_mobilenetv2.onnx")
if (NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND gdown "https://drive.google.com/uc?id=19M64ZAo0XNv-Ep2RDynrRipLg3YAm65e" -O ${PATH}/traffic_light_classifier_mobilenetv2.onnx)
endif()
set(FILE "${PATH}/lamp_labels.txt")
message(STATUS "Checking and downloading lamp_labels.txt")
if (NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND gdown "https://drive.google.com/uc?id=1C4XkFe-G58LcDJSVMp5xlwQniGVozgwW" -O ${PATH}/lamp_labels.txt)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

#generate_dynamic_reconfigure_options(cfg/HSVFilter.cfg)

add_definitions(-DENABLE_GPU)

  include_directories(
    include
    utils
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
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

  add_library(traffic_light_classifier_nodelet SHARED
    src/color_classifier.cpp
    src/cnn_classifier.cpp
    src/nodelet.cpp
    )

  target_link_libraries(traffic_light_classifier_nodelet
    libutils
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

  add_executable(traffic_light_classifier_node
    src/node.cpp
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
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    )

  install(
    FILES
    traffic_light_classifier.xml
    DESTINATION share/${PROJECT_NAME}
    )

  install(
    DIRECTORY
    launch
    data
    DESTINATION share/${PROJECT_NAME}
    )

