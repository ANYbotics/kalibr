cmake_minimum_required(VERSION 2.8.3)
project(aslam_bsplines)

add_compile_options(-w)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CATKIN_PACKAGE_DEPENDENCIES
  aslam_sparse_block_matrix
  sm_common
  sm_eigen
  sm_kinematics
  sm_timing
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)
find_package(SUITESPARSE REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    SUITESPARSE
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${SUITESPARSE_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/BSpline.cpp
  src/BSplinePose.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
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
    include/${PROJECT_NAME}
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)