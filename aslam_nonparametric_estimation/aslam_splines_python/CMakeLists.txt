cmake_minimum_required(VERSION 3.5.1)
project(aslam_splines_python)

add_definitions(-w)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  aslam_backend
  aslam_backend_expressions
  aslam_backend_python
  aslam_bsplines
  aslam_bsplines_python
  aslam_cameras
  aslam_sparse_block_matrix
  aslam_splines
  aslam_time
  sm_numpy_eigen
  sm_common
  sm_kinematics
  sm_logging
  sm_opencv
  sm_property_tree
  sm_python_module
  sm_timing
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

add_python_export_library(${PROJECT_NAME} python/aslam_splines_python
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

##############
##   Test   ##
##############
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/empty_test.cpp
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    include
  )
  target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  ###################
  ## Code_coverage ##
  ###################
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS
        test_${PROJECT_NAME}
    )
  endif()
endif()

#################
## Clang_tools ##
#################
#find_package(cmake_clang_tools QUIET)
#if(cmake_clang_tools_FOUND)
#  add_default_clang_tooling(
#    DISABLE_CLANG_FORMAT
#  )
#endif(cmake_clang_tools_FOUND)
