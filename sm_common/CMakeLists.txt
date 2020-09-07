cmake_minimum_required(VERSION 3.5.1)
project(sm_common)

add_compile_options(-w)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x -D__STRICT_ANSI__")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  DEPENDS
  CFG_EXTRAS export_flags.cmake
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/progress_info.cpp
)
target_link_libraries(${PROJECT_NAME} 
                      ${catkin_LIBRARIES} ${Boost_LIBRARIES})


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

#############
## Testing ##
#############
# if(CATKIN_ENABLE_TESTING)

#   # Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
#   add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

#   catkin_add_gtest(${PROJECT_NAME}-test
#     test/test_main.cpp
#     test/maths.cpp
#     test/serialization_macros.cpp
#     test/numerical_comparisons.cpp
#     test/hash_id.cpp
#     )
#   target_link_libraries(${PROJECT_NAME}-test pthread)

# endif()

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
