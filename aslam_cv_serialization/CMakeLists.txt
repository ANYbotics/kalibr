
cmake_minimum_required(VERSION 3.5.1)
project(aslam_cv_serialization)

add_compile_options(-w)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CATKIN_PACKAGE_DEPENDENCIES
  aslam_cameras
  aslam_time
  sm_boost
  sm_common
  sm_eigen
  sm_kinematics
  sm_logging
  sm_opencv
  sm_property_tree
  sm_random
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)
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
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

#common commands for building c++ executables and libraries
INCLUDE(autogen_cameras.cmake)
#INCLUDE(autogen_frames.cmake)

add_library(${PROJECT_NAME}
#  ${AUTOGEN_FRAME_CPP_FILES}
  ${AUTOGEN_CAMERA_CPP_FILES}
  src/FrameBaseSerialization.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
#add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)
#catkin_add_gtest(${PROJECT_NAME}_tests
#  test/test_main.cpp
#  test/testMultiFrame.cpp
#)
#target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

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