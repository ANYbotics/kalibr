cmake_minimum_required(VERSION 3.5.1)
project(aslam_bsplines_python)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CATKIN_PACKAGE_DEPENDENCIES
  aslam_bsplines
  aslam_sparse_block_matrix
  sm_numpy_eigen
  sm_common
  sm_kinematics
  sm_python_module
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)
find_package(SUITESPARSE REQUIRED)

catkin_package(
  INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIR}
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
    ${EIGEN3_INCLUDE_DIR}
    ${SUITESPARSE_INCLUDE_DIRS}
)

# Set up the python exports.
SET(PY_PROJECT_NAME ${PROJECT_NAME})
SET(PY_PACKAGE_DIR python/aslam_bsplines_python)

add_python_export_library(${PY_PROJECT_NAME} ${PY_PACKAGE_DIR}
  src/SplinePython.cpp
  src/BSplinePython.cpp
  src/BSplinePosePython.cpp
)

install(
  TARGETS
    ${PY_PROJECT_NAME}
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

  target_include_directories(test_${PROJECT_NAME}
    PRIVATE
      include
    SYSTEM PUBLIC
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

