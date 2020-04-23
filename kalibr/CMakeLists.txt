cmake_minimum_required(VERSION 2.8.3)
project(kalibr)

add_definitions(-w)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CATKIN_PACKAGE_DEPENDENCIES
  aslam_backend
  aslam_backend_expressions
  aslam_backend_python
  aslam_cameras_april
  aslam_cv_python
  aslam_cv_backend_python
  aslam_incremental_calibration_python
  aslam_sparse_block_matrix
  aslam_splines_python
  sm_numpy_eigen
  sm_python
  sm_python_module
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

##################################
# error terms (+python export)
##################################
add_library(${PROJECT_NAME}_errorterms
  src/EuclideanError.cpp
  src/GyroscopeError.cpp
  src/AccelerometerError.cpp
)

target_link_libraries(${PROJECT_NAME}_errorterms
  ${catkin_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
)

add_python_export_library(${PROJECT_NAME}_errorterms_python python/${PROJECT_NAME}_errorterms/..
  src/module.cpp
)
target_link_libraries(${PROJECT_NAME}_errorterms_python
  ${PROJECT_NAME}_errorterms
)

##################################
# tests
##################################
# if(CATKIN_ENABLE_TESTING)

#   add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

#   catkin_add_gtest(${PROJECT_NAME}_test
#     test/test_main.cpp
#     test/TestErrorTerms.cpp
#   )
  
#   target_link_libraries(${PROJECT_NAME}_test
#     ${PROJECT_NAME}_errorterms
#   )

# endif()

##################################
# EXPORT
##################################
catkin_python_setup()

install(
  TARGETS
    ${PROJECT_NAME}_errorterms
    ${PROJECT_NAME}_errorterms_python
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)