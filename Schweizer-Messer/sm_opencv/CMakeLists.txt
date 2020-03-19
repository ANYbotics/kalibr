cmake_minimum_required(VERSION 2.8.3)
project(sm_opencv)

add_compile_options(-w)
add_definitions(-std=c++0x -D__STRICT_ANSI__)

find_package(catkin REQUIRED
  COMPONENTS
    sm_common
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS
    include
    ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS
    sm_common
  DEPENDS
    Boost
    OpenCV
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)