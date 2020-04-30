cmake_minimum_required(VERSION 2.8.3)
project(sm_numpy_eigen)

add_compile_options(-w)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    sm_python_module
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_python_setup()

add_definitions(${EIGEN3_DEFINITIONS})
include_directories("${PROJECT_SOURCE_DIR}/include/sm_numpy_eigen")

catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    sm_python_module
  DEPENDS
    Boost
)
add_definitions(-std=c++0x -D__STRICT_ANSI__)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

INCLUDE(src/autogen_files.cmake)

##############
## Building ##
##############

add_python_export_library(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/python/${PROJECT_NAME}
  ${AUTOGEN_FILES}
)

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
                      
# INCLUDE(src/autogen_test_files.cmake)
# add_python_export_library(${PROJECT_NAME}_test 
#   ${PROJECT_SOURCE_DIR}/python/${PROJECT_NAME} ${AUTOGEN_TEST_FILES}
# )

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

#############
## Testing ##
#############
# if(CATKIN_ENABLE_TESTING)

#   ## Add nosetest based cpp test target.
#   catkin_add_nosetests(test/sm_numpy_eigen_tests.py)

# endif()
