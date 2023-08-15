#include <map>
#include <string>
#include <cctype>
#include <sstream>
#include <utility>
#include <fstream>
#include <iostream>
#include <algorithm>
#include "parser.h"
#include "errors.h"


std::string check_presence(const std::string &param_name, const std::map<std::string, std::string> &params_map) {
    std::string res;
    try {
        res = params_map.at(param_name);
    }
    catch (const std::out_of_range &ex) {
        throw config_parse_error(param_name + " parameter is not in the config file");
    }
    return res;
}

double convert_to_double(const std::string &param_name, const std::map<std::string, std::string> &params_map) {
    std::stringstream iss;
    double converted_val = 0.0;
    std::string val_to_convert = check_presence(param_name, params_map);
    iss << val_to_convert;
    if (!(iss >> converted_val) || iss.rdbuf()->in_avail() != 0) {
        throw config_parse_error(
                param_name + " " + "value: " + params_map.at(param_name) +
                " is not convertible to double");
    }
    return converted_val;
}

void trim(std::string &str) {
    {
        str.erase(str.begin(), std::find_if(str.begin(), str.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
        str.erase(std::find_if(str.rbegin(), str.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(),
                  str.end());
    }
}

std::pair<std::string, std::string> parse_line(std::string &line) {
    size_t comment_pos = line.find(COMMENTER);
    if (comment_pos != std::string::npos) {
        line.erase(comment_pos);
    }
    size_t sep_pos = line.find(SEPARATOR);
    std::string param = line.substr(0, sep_pos);
    std::string raw_val = line.substr(sep_pos + 1, std::string::npos);
    trim(param);
    trim(raw_val);
    if (*raw_val.begin() == BRACKET) {
        raw_val.erase(0, 1);
    }
    if (*(raw_val.end() - 1) == BRACKET) {
        raw_val.erase(raw_val.end() - 1);
    }
    std::pair<std::string, std::string> pair = std::make_pair(param, raw_val);
    return pair;
}

config_t
convert_map_struct(const std::map<std::string, std::string> &params_map) {
    config_t final_parsed_data;
    final_parsed_data.mesh_file = check_presence("mesh_file", params_map);
    final_parsed_data.tree_file = check_presence("tree_file", params_map);
    final_parsed_data.trajectory_file = check_presence("trajectory_file", params_map);
    final_parsed_data.step_size = convert_to_double("step_size", params_map);
    final_parsed_data.start_x = convert_to_double("start_x", params_map);
    final_parsed_data.start_y = convert_to_double("start_y", params_map);
    final_parsed_data.end_x = convert_to_double("end_x", params_map);
    final_parsed_data.end_y = convert_to_double("end_y", params_map);
    final_parsed_data.robot_height = convert_to_double("robot_height", params_map);
    final_parsed_data.robot_width = convert_to_double("robot_width", params_map);
    return final_parsed_data;
}

config_t parseTOML_to_struct(const std::string &file_path) {
    std::ifstream cfg_file(file_path.c_str());
    if (!cfg_file.is_open()) {
        throw config_parse_error("Error in initial opening config file at path: " + file_path +
                                 "\nCheck the path to the file or does the file exist");
    }
    if (cfg_file.fail()) {
        throw config_parse_error("Error in initial reading config file at path: " + file_path);
    }
    std::map<std::string, std::string> parameters_map;
    std::string line;
    int line_number = 0;
    while (std::getline(cfg_file, line)) {
        ++line_number;
        if (line.empty()) {
            continue;
        }
        trim(line);
        if (*line.begin() == COMMENTER) {
            continue;
        }
        std::pair<std::string, std::string> param_val = parse_line(line);
        parameters_map[param_val.first] = param_val.second;
    }
    config_t parsed_data = convert_map_struct(parameters_map);
    return parsed_data;
}


config_t parse_and_validate_config(const std::string &file_name) {
    config_t data_in_struct;
    try {
        data_in_struct = parseTOML_to_struct(file_name);
    }
    catch (const base_exception &ex) {
        std::cerr << ex.what() << std::endl;
        exit(ex.error_code);
    }
    return data_in_struct;
}