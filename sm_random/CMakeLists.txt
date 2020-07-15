cmake_minimum_required(VERSION 3.5.1)
project(sm_random)

add_compile_options(-w)

find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  DEPENDS
    Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS})

##############
## Building ##
##############

add_library(${PROJECT_NAME}
  src/random.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
)

##################
## Installation ##
##################

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
  DIRECTORY
    include/${PROJECT_NAME}
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)