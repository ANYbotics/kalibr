cmake_minimum_required(VERSION 2.8.3)
project(aslam_cv_python)

add_definitions( -fPIC -w -std=c++0x)

set(CATKIN_PACKAGE_DEPENDENCIES
  aslam_cameras
  aslam_cv_serialization
  aslam_imgproc
  aslam_time
  python_module
  sm_common
  sm_eigen
  sm_kinematics
  sm_opencv
  sm_python
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS serialization) 

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_python_export_library(${PROJECT_NAME} python/aslam_cv
  src/module.cpp
  src/CameraGeometries.cpp
  src/CameraGeometry.cpp
  src/Time.cpp
#  src/CameraSystem.cpp
#  src/CameraSystemClasses.cpp
#  src/MatchingAlgorithms.cpp
#  src/MultiFrame.cpp
  src/Frontend.cpp
  src/Frame.cpp
#  src/Descriptors.cpp
  src/CameraProjections.cpp
  src/CameraShutters.cpp
#  src/ImageSynchronizer.cpp
#  src/FrameBuilder.cpp
#  src/ImageContainer.cpp
  src/GridCalibration.cpp
#  src/Landmark.cpp
  src/Undistorters.cpp
#  src/NCameras.cpp
#  src/MultiFrameFactory.cpp
  src/OmniUndistorter.cpp
  src/PinholeUndistorter.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
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