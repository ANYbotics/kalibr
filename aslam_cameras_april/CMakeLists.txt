cmake_minimum_required(VERSION 3.5.1)
project(aslam_cameras_april)

add_definitions(-DASLAM_USE_ROS)

set(CATKIN_PACKAGE_DEPENDENCIES
  aslam_apriltag2
  aslam_cameras
  sm_numpy_eigen
  sm_boost
  sm_common
  sm_eigen
  sm_logging
  sm_python
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization system) 

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
    OpenCV
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/GridCalibrationTargetAprilgrid.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_python_export_library(${PROJECT_NAME}_python python/aslam_cameras_april
  src/module.cpp
)
target_link_libraries(${PROJECT_NAME}_python
  ${PROJECT_NAME}
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