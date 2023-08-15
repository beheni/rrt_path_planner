cmake_minimum_required(VERSION 3.15)
set(PROJECT_NAME  rrt_planner)
project(${PROJECT_NAME} C CXX)

set(CMAKE_CXX_STANDARD 98)

add_subdirectory(src)

INSTALL(PROGRAMS
        $<TARGET_FILE:path_planner_visualisation>
        DESTINATION bin)

INSTALL(PROGRAMS
        $<TARGET_FILE:rrt_path_planner>
        DESTINATION bin)

