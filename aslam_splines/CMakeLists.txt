cmake_minimum_required(VERSION 3.5.1)
project(aslam_splines)

add_definitions(-w)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CATKIN_PACKAGE_DEPENDENCIES
  aslam_backend
  aslam_backend_expressions
  aslam_bsplines
  aslam_sparse_block_matrix
  sm_numpy_eigen
  sm_common
  sm_kinematics
  sm_timing
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(SUITESPARSE REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    SUITESPARSE
    Boost
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${SUITESPARSE_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/BSplinePoseDesignVariable.cpp
  src/BSplineExpressions.cpp
  src/EuclideanBSplineDesignVariable.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${Boost_LIBRARIES}
)

# if(CATKIN_ENABLE_TESTING)

#   catkin_add_gtest(${PROJECT_NAME}_test
#     test/test_main.cpp
#     test/TestBSplineExpressions.cpp
#     test/TestErrors.cpp
#     #  test/TestOPTBSpline.cpp
#   )

#   target_link_libraries(${PROJECT_NAME}_test
#     ${PROJECT_NAME}
#   )

# endif()

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