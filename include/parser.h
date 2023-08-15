#ifndef RRT_PLANNER_PARSER_H
#define RRT_PLANNER_PARSER_H

#include <string>

const char SEPARATOR = '=';
const char COMMENTER = '#';
const char BRACKET = '\"';

struct config_t {
    std::string mesh_file;
    std::string tree_file;
    std::string trajectory_file;
    double step_size;
    double start_x;
    double start_y;
    double end_x;
    double end_y;
    double robot_height;
    double robot_width;
};

config_t parse_and_validate_config(const std::string &file_name);

#endif //RRT_PLANNER_PARSER_H
