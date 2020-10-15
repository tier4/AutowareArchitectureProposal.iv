cmake_minimum_required(VERSION 3.5)
project(shape_estimation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
add_executable(shape_estimation
  src/main.cpp
  src/node.cpp
  src/shape_estimator.cpp
  src/model/normal/bounding_box.cpp
  src/model/normal/convex_hull.cpp
  src/model/normal/cylinder.cpp
  src/model/yaw_fixed/bounding_box.cpp
  src/filter/car_filter.cpp
  src/filter/bus_filter.cpp
  src/filter/truck_filter.cpp
  src/filter/no_filter.cpp
  src/corrector/normal/car_corrector.cpp
  src/corrector/normal/bus_corrector.cpp
  src/corrector/normal/truck_corrector.cpp
  src/corrector/normal/no_corrector.cpp
  src/corrector/yaw_fixed/car_corrector.cpp
  src/corrector/yaw_fixed/bus_corrector.cpp
  src/corrector/yaw_fixed/truck_corrector.cpp
  src/corrector/yaw_fixed/no_corrector.cpp
)

set(SHAPE_ESTIMATION_DEPENDENCIES
  rclcpp
  autoware_perception_msgs
  tf2
  tf2_geometry_msgs
  tf2_eigen
  PCL
  pcl_conversions
  OpenCV
)
ament_target_dependencies(shape_estimation ${SHAPE_ESTIMATION_DEPENDENCIES})


install(TARGETS shape_estimation
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
