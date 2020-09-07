cmake_minimum_required(VERSION 3.5.1)
project(aslam_incremental_calibration_python)

add_definitions(-std=c++0x)

set(CATKIN_PACKAGE_DEPENDENCIES
  aslam_incremental_calibration
  sm_numpy_eigen
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

add_python_export_library(${PROJECT_NAME}
  src/aslam_incremental_calibration_python
  src/module.cpp
  src/OptimizationProblem.cpp
  src/IncrementalEstimator.cpp
  src/LinearSolver.cpp
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
    include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
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