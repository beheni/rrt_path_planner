#ifndef RRT_PLANNER_ERRORS_H
#define RRT_PLANNER_ERRORS_H

#include <string>
#include <exception>
#include <stdexcept>

enum ERROR_CODES {
    ALL_GOOD = 0,
    ARGUMENTS_COUNT_ERROR = 1,
    RESULT_FILE_ERROR = 3,
    CONFIG_PARSE_ERROR = 4,
    MESH_FILE_ERROR = 5
};

struct base_exception : public std::runtime_error {
    int error_code;

    explicit base_exception(const std::string &message)
            : std::runtime_error(message) {}
};


struct config_parse_error : public base_exception {
    explicit config_parse_error(const std::string &message) : base_exception(message) {
        error_code = CONFIG_PARSE_ERROR;
    }
};

struct result_file_error : public base_exception {
    explicit result_file_error(const std::string &message) : base_exception(message) {
        error_code = RESULT_FILE_ERROR;
    }
};

struct arguments_count_error : public base_exception {
    explicit arguments_count_error(const std::string &message) : base_exception(message) {
        error_code = ARGUMENTS_COUNT_ERROR;
    }
};

struct mesh_file_error : public base_exception {
    explicit mesh_file_error(const std::string &message) : base_exception(message) {
        error_code = MESH_FILE_ERROR;
    }
};

#endif //RRT_PLANNER_ERRORS_H
