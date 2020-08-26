cmake_minimum_required(VERSION 3.5.1)
project(sm_python)

add_compile_options(-w)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    sm_common
    sm_kinematics 
    sm_logging
    sm_matrix_archive
    sm_numpy_eigen
    sm_property_tree
    sm_python_module
    sm_timing
)
                                        
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization)


catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    sm_common
    sm_kinematics
    sm_logging
    sm_matrix_archive
    sm_numpy_eigen
    sm_property_tree
    sm_python_module
    sm_timing
)
add_definitions(-std=c++0x -D__STRICT_ANSI__)

include_directories(
  include 
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

# Set up the python exports.
SET(PY_PROJECT_NAME ${PROJECT_NAME})
SET(PY_PACKAGE_DIR python/sm_python)

##############
## Building ##
##############
add_python_export_library(${PY_PROJECT_NAME} ${PY_PACKAGE_DIR}
  src/module.cpp
  src/export_rotational_kinematics.cpp
  src/export_rotations.cpp
  src/export_transformations.cpp
  src/export_quaternion_algebra.cpp
  src/export_homogeneous.cpp
  src/exportTransformation.cpp
  src/exportHomogeneousPoint.cpp
  src/exportTimestampCorrector.cpp
  src/exportPropertyTree.cpp
  src/exportEigen.cpp
  src/exportUncertainVector.cpp
  src/exportMatrixArchive.cpp
  src/export_kinematics_property_tree.cpp
  src/export_eigen_property_tree.cpp
  src/Logging.cpp
  src/exportNsecTime.cpp
  src/random.cpp
)

target_link_libraries(${PY_PROJECT_NAME} ${Boost_LIBRARIES})

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
  endif(cmake_code_coverage_FOUND)
endif()
