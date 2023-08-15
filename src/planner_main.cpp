#include <iostream>
#include "errors.h"
#include "parser.h"
#include "RRTGraph.h"

int main(int argc, char *argv[]) {
    try {
        if (argc != 2) {
            throw arguments_count_error(
                    "Wrong arguments count. Proper usage: ./bin/rrt_path_planner <path-to-config>");
        }
        config_t config = parse_and_validate_config(argv[1]);
        RRTGraph rrt_graph = RRTGraph(config);
        rrt_graph.generate_rrt();
    }
    catch (const base_exception &ex) {
        std::cerr << ex.what() << std::endl;
        return ex.error_code;
    }
    return ALL_GOOD;
}