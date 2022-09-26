cmake_minimum_required(VERSION 3.16.3)
project(polygon_coverage_planners)

# find_package(catkin REQUIRED COMPONENTS rosconsole roslib polygon_coverage_solvers polygon_coverage_geometry)

find_package(CGAL QUIET COMPONENTS Core)
include(${CGAL_USE_FILE})

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

# TODO(rikba): Make catkin package.
find_package(PkgConfig)
pkg_check_modules(MONO mono-2 REQUIRED)
include_directories(${MONO_INCLUDE_DIRS})

# catkin_package(
    # INCLUDE_DIRS
    # include
    # # ${catkin_INCLUDE_DIRS}
    # LIBRARIES
    # ${PROJECT_NAME}
# )
include_directories(
    include
    ../solvers/include/
    ../geometry/include/
    # ${catkin_INCLUDE_DIRS}
)

link_directories(
  ../solvers/build/lib/
  ../solvers/build/
  ../geometry/build/
)

#############
# LIBRARIES #
#############
add_library(
    ${PROJECT_NAME}
    src/cost_functions/path_cost_functions.cc
    src/graphs/gtspp_product_graph.cc
    src/graphs/sweep_plan_graph.cc
    src/timing.cc
    src/planners/polygon_stripmap_planner.cc
    src/planners/polygon_stripmap_planner_exact.cc
    src/planners/polygon_stripmap_planner_exact_preprocessed.cc
)
target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
    ${CGAL_LIBRARIES}
    ${CGAL_3RD_PARTY_LIBRARIES}
    # ${catkin_LIBRARIES}
    polygon_coverage_solvers_shared # you need the shared library
    polygon_coverage_geometry
)

#########
# TESTS #
#########
include(GoogleTest)

set(EXECUTABLES
  planners-test
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
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#         FILES_MATCHING PATTERN "*.h"
#         )

