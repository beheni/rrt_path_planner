#ifndef RRT_PLANNER_HELPER_FUNCS_H
#define RRT_PLANNER_HELPER_FUNCS_H

#include <vector>
#include <string>
#include <utility>
#include <sstream>
#include "utility.h"

double area(const Triangle &);

std::vector<Triangle> parse_triangles(const std::string &path);

double orientation(const Point &p1, const Point &p2, const Point &p3);

bool do_segments_intersect(const Point &p1, const Point &q1, const Point &p2, const Point &q2);

std::pair<double, double> find_coordinate_bounds(std::vector<Triangle> &);

template<typename T>
std::string num_to_str(T num) {
    std::stringstream tmp_ss;
    tmp_ss << num;
    std::string str = tmp_ss.str();
    return str;
}

#endif //RRT_PLANNER_HELPER_FUNCS_H
