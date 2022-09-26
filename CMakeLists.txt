cmake_minimum_required(VERSION 2.8.3)

project(polygon_coverage)
# set(CMAKE_CXX_FLAGS "-std=c++11")
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo, 2019-03-04 need Release for numeric stability!

add_subdirectory(solvers)
add_subdirectory(geometry)
add_subdirectory(planners)