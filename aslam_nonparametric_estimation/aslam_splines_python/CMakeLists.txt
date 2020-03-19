cmake_minimum_required(VERSION 2.8.3)
project(aslam_splines_python)

add_definitions(-w)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CATKIN_PACKAGE_DEPENDENCIES
  aslam_backend
  aslam_backend_expressions
  aslam_backend_python
  aslam_cameras
  aslam_splines
  aslam_time
  bsplines
  bsplines_python
  numpy_eigen
  python_module
  sm_common
  sm_kinematics
  sm_logging
  sm_opencv
  sm_property_tree
  sm_timing
  sparse_block_matrix
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(SUITESPARSE REQUIRED)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    SUITESPARSE
)

include_directories(
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${SUITESPARSE_INCLUDE_DIRS}
)

add_python_export_library(${PROJECT_NAME} python/aslam_splines
  src/spline_module.cpp
  src/BSplineMotionError.cpp
  src/SimpleSplineError.cpp
)

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)