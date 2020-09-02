cmake_minimum_required(VERSION 3.5.1)
project(aslam_cv_error_terms)

add_definitions(-w)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CATKIN_PACKAGE_DEPENDENCIES
  aslam_backend
  aslam_backend_expressions
  aslam_cameras
  aslam_cv_backend
  aslam_sparse_block_matrix
  aslam_splines
  aslam_time
  sm_boost
  sm_eigen
  sm_kinematics
  sm_logging
  sm_opencv
  sm_property_tree
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

# if(CATKIN_ENABLE_TESTING)

# # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
#   add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

#   catkin_add_gtest(${PROJECT_NAME}_test
#     test/test_main.cpp
#     test/TestReprojectionError.cpp
#   )
#   target_link_libraries(${PROJECT_NAME}_test
#     ${catkin_LIBRARIES}
#     ${Boost_LIBRARIES}
#   )

# endif()

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

  target_include_directories(test_${PROJECT_NAME}
    PRIVATE
      include
    SYSTEM PUBLIC
      ${catkin_INCLUDE_DIRS}
  )

  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
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