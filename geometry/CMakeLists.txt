cmake_minimum_required(VERSION 3.16.3)
project(polygon_coverage_geometry)

# find_package(catkin REQUIRED COMPONENTS rosconsole roslib rosunit polygon_coverage_solvers)

find_package(CGAL COMPONENTS Core)
include(${CGAL_USE_FILE})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

# catkin_package(
#         INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
#         LIBRARIES ${PROJECT_NAME}
# )
include_directories(
  include
  ../solvers/include/
  # ${catkin_INCLUDE_DIRS}
)

link_directories(
  ../solvers/build/
)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/bcd.cc
  src/boolean.cc
  src/cgal_comm.cc
  src/decomposition.cc
  src/offset.cc
  src/sweep.cc
  src/tcd.cc
  src/triangulation.cc
  src/visibility_graph.cc
  src/visibility_polygon.cc
  src/weakly_monotone.cc
)


target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
  ${CGAL_LIBRARIES}
  ${CGAL_3RD_PARTY_LIBRARIES}
  # polygon_coverage_solvers
)

# #########
# # TESTS #
# #########
include(GoogleTest)

set(EXECUTABLES
  bcd-test
  cgal_comm-test
  offset-test
  sweep-test
  visibility_graph-test
  visibility_polygon-test
  )
# Creates and executable for each cpp file
# Prepends with project name to avoid name collisions
foreach(executable ${EXECUTABLES})
  add_executable(${PROJECT_NAME}_${executable} test/${executable} )
  target_link_libraries(
    ${PROJECT_NAME}_${executable}
    PUBLIC
    ${PROJECT_NAME}
    gtest
    pthread)
  gtest_add_tests(
    TARGET ${PROJECT_NAME}_${executable}
    SOURCES test/${executable}.cpp
  )
endforeach()


# ##########
# # EXPORT #
# ##########
# install(TARGETS ${PROJECT_NAME}
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#         )

# install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.h"
# )

