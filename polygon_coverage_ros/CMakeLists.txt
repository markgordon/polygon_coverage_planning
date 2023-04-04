cmake_minimum_required(VERSION 3.16.3)
project(polygon_coverage_ros)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nav_msgs
        geometry_msgs
        trajectory_msgs
        visualization_msgs
        std_srvs
        eigen_conversions
        polygon_coverage_msgs
        polygon_coverage_geometry
        polygon_coverage_planners
        polygon_coverage_solvers
        )

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

find_package(CGAL QUIET COMPONENTS Core)
include(${CGAL_USE_FILE})


#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/coverage_planner.cc
  src/polygon_planner_base.cc
  src/ros_interface.cc
  src/shortest_path_planner.cc
)
target_link_libraries(${PROJECT_NAME} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${catkin_LIBRARIES})

############
# Binaries #
############
add_executable(coverage_planner
  src/coverage_planner_node.cc
)
target_link_libraries(coverage_planner ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(coverage_planner_exact
  src/coverage_planner_exact_node.cc
)
target_link_libraries(coverage_planner_exact ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(coverage_planner_exact_preprocessed
  src/coverage_planner_exact_preprocessed_node.cc
)
target_link_libraries(coverage_planner_exact_preprocessed ${PROJECT_NAME} ${catkin_LIBRARIES})

target_link_libraries(sweep_test ${PROJECT_NAME} ${catkin_LIBRARIES})

##########
# EXPORT #
##########
install(TARGETS coverage_planner coverage_planner_exact coverage_planner_exact_preprocessed
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

