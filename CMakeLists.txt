cmake_minimum_required(VERSION 3.9)

project(osqp_vendor)

find_package(ament_cmake REQUIRED)

macro(build_osqp)
  set(extra_cmake_args)

  get_property(multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
  if(NOT multi_config AND DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()

  include(ExternalProject)
  externalproject_add(osqp-v0.6.1.dev0
    GIT_REPOSITORY https://github.com/oxfordcontrol/osqp.git
    GIT_TAG v0.6.1.dev0
    GIT_SHALLOW ON
    TIMEOUT 60
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/osqp_install
      ${extra_cmake_args}
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/osqp_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
    PATTERN config.h EXCLUDE
  )
endmacro()

build_osqp()

ament_export_libraries(osqp)
ament_export_dependencies(osqp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS osqp_vendor-extras.cmake)
