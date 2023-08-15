set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)
set (INCLUDE_DIR ../include)
set (ALL_TARGETS path_planner_visualisation rrt_path_planner)

add_executable(path_planner_visualisation visualisation_main.cpp
        ${INCLUDE_DIR}/helper_funcs.h   helper_funcs.cpp
        ${INCLUDE_DIR}/parser.h         parser.cpp
        ${INCLUDE_DIR}/RRTGraph.h       RRTGraph.cpp
        ${INCLUDE_DIR}/RRTDraw.h        RRTDraw.cpp
        ${INCLUDE_DIR}/utility.h        utility.cpp
        ${INCLUDE_DIR}/errors.h)

add_executable(rrt_path_planner planner_main.cpp
        ${INCLUDE_DIR}/helper_funcs.h   helper_funcs.cpp
        ${INCLUDE_DIR}/parser.h         parser.cpp
        ${INCLUDE_DIR}/RRTGraph.h       RRTGraph.cpp
        ${INCLUDE_DIR}/utility.h        utility.cpp
        ${INCLUDE_DIR}/errors.h )

foreach (TARGET ${ALL_TARGETS})
    target_include_directories(${TARGET} PRIVATE ${INCLUDE_DIR} ../lib)
    target_link_libraries(${TARGET} PRIVATE
            ${CMAKE_SOURCE_DIR}/lib/libsfml-graphics.so.2.5
            ${CMAKE_SOURCE_DIR}/lib/libsfml-window.so.2.5
            ${CMAKE_SOURCE_DIR}/lib/libsfml-system.so.2.5)
endforeach()