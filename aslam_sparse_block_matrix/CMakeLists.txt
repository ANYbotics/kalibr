cmake_minimum_required(VERSION 3.5.1)
project(aslam_sparse_block_matrix)

add_compile_options(-w)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

set(CATKIN_PACKAGE_DEPENDENCIES
  sm_common
  sm_eigen
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(SUITESPARSE REQUIRED)

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
  src/matrix_structure.cpp
  src/sparse_helper.cpp
  src/marginal_covariance_cholesky.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
)

# if(CATKIN_ENABLE_TESTING)

#   # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
#   add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

#   catkin_add_gtest( ${PROJECT_NAME}_tests
#     test/test_main.cpp
#     test/solver_tests.cpp
#     test/sparse_block_matrix_tests.cpp
#   )

#   target_link_libraries(${PROJECT_NAME}_tests
#     ${PROJECT_NAME}
#     ${TBB_LIBRARIES}
#     ${SUITESPARSE_LIBRARIES}
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
    include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)