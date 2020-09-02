cmake_minimum_required(VERSION 2.8)
project(aslam_apriltag2)

add_compile_options(-w)
add_definitions(-fPIC)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/)

find_package(catkin REQUIRED COMPONENTS cmake_modules)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS
      include
      ${EIGEN3_INCLUDE_DIR}
    LIBRARIES
      ${PROJECT_NAME}
    DEPENDS
      EIGEN3
      OpenCV
)

include_directories(
  include
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

#library
file(GLOB SOURCE_FILES "src/*.cc")
add_library(${PROJECT_NAME}
  ${SOURCE_FILES}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}  
  ${OpenCV_LIBRARIES}
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
  endif()
endif()