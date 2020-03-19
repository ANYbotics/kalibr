cmake_minimum_required(VERSION 2.8.3)
project(aslam_backend)

add_compile_options(-w)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CATKIN_PACKAGE_DEPENDENCIES
  sm_boost
  sm_logging
  sm_property_tree
  sm_random
  sm_timing
  sparse_block_matrix
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)
find_package(SUITESPARSE REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

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

if (PKG_CONFIG_FOUND)
  pkg_check_modules(TBB "tbb")
  if (TBB_FOUND)
    add_definitions(
      ${TBB_CFLAGS}
      ${TBB_CFLAGS_OTHER}
    )
    include_directories(SYSTEM
      ${TBB_INCLUDE_DIRS}
    )
  endif ()
endif ()

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${SUITESPARSE_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/MEstimatorPolicies.cpp
  src/JacobianContainer.cpp
  src/DesignVariable.cpp
  src/ErrorTerm.cpp
  src/OptimizationProblemBase.cpp
  src/Optimizer.cpp
  src/sparse_matrix_functions.cpp
  src/OptimizationProblem.cpp
  src/SimpleOptimizationProblem.cpp
  src/JacobianBuilder.cpp
  src/LinearSystemSolver.cpp
  src/BlockCholeskyLinearSystemSolver.cpp
  src/SparseCholeskyLinearSystemSolver.cpp
  src/SparseQrLinearSystemSolver.cpp
  src/Matrix.cpp
  src/DenseMatrix.cpp
  src/SparseBlockMatrixWrapper.cpp
  src/DenseQrLinearSystemSolver.cpp
  src/BlockCholeskyLinearSolverOptions.cpp
  src/SparseCholeskyLinearSolverOptions.cpp
  src/SparseQRLinearSolverOptions.cpp
  src/DenseQRLinearSolverOptions.cpp
  src/TrustRegionPolicy.cpp
  src/ErrorTermDs.cpp
  src/GaussNewtonTrustRegionPolicy.cpp
  src/LevenbergMarquardtTrustRegionPolicy.cpp
  src/Marginalizer.cpp
  src/MarginalizationPriorErrorTerm.cpp
  src/DogLegTrustRegionPolicy.cpp
  src/Optimizer2.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TBB_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)

  # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
  add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)
  catkin_add_gtest(${PROJECT_NAME}_test
    test/test_main.cpp
    test/JacobianContainer.cpp
    test/test_sparse_matrix_functions.cpp
    test/TestOptimizer.cpp
    test/TestOptimizationProblem.cpp
    test/CompressedColumnMatrixTest.cpp
    test/MatrixTestHarness.cpp
    test/DenseMatrixTest.cpp
    test/SparseMatrixTest.cpp
    test/LinearSolverTests.cpp
    test/ErrorTermTests.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test 
    ${PROJECT_NAME}
  )

endif()

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)