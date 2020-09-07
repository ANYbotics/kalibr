cmake_minimum_required(VERSION 3.5.1)
project(aslam_backend_expressions)

SET(CMAKE_CXX_FLAGS "-std=c++0x")
add_compile_options(-w)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CATKIN_PACKAGE_DEPENDENCIES
  aslam_backend
  aslam_sparse_block_matrix
  sm_boost
  sm_kinematics
  sm_random
  sm_timing
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)
find_package(SUITESPARSE REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
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
    ${EIGEN3_INCLUDE_DIR}
    ${SUITESPARSE_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/RotationExpression.cpp
  src/RotationExpressionNode.cpp
  src/RotationQuaternion.cpp
  src/MappedRotationQuaternion.cpp
  src/Vector2RotationQuaternionExpressionAdapter.cpp

  src/MatrixExpression.cpp
  src/MatrixExpressionNode.cpp
  src/MatrixTransformation.cpp

  src/TransformationExpression.cpp

  src/TransformationExpressionNode.cpp
  src/TransformationBasic.cpp

  src/EuclideanExpression.cpp
  src/EuclideanExpressionNode.cpp
  src/EuclideanPoint.cpp
  src/MappedEuclideanPoint.cpp

  src/HomogeneousExpression.cpp
  src/HomogeneousExpressionNode.cpp
  src/HomogeneousPoint.cpp
  src/MappedHomogeneousPoint.cpp

  src/ScalarExpression.cpp
  src/ScalarExpressionNode.cpp
  src/Scalar.cpp

  src/EuclideanDirection.cpp

  src/ErrorTermTransformation.cpp
  src/ErrorTermEuclidean.cpp

  src/MapTransformation.cpp

  src/MatrixBasic.cpp
  src/MatrixExpression.cpp
  src/MatrixExpressionNode.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
)

# if(CATKIN_ENABLE_TESTING)

#   # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
#   add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

#   catkin_add_gtest(${PROJECT_NAME}_test
#     test/test_main.cpp
#     test/RotationExpression.cpp
#     test/HomogeneousExpression.cpp
#     test/MatrixAndEuclideanExpression.cpp
#     # \todo reenable after I can talk to Hannes.
#     # test/FixedPointNumberTest.cpp
#     test/GenericScalarExpressionTest.cpp
#     test/GenericMatrixExpression.cpp
#     test/QuaternionExpression.cpp
#     test/ScalarExpression.cpp
#     test/ErrorTest_Transformation.cpp
#     test/ErrorTest_Euclidean.cpp
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
