#include <cmath>
#include <fstream>
#include <algorithm>
#include "errors.h"
#include "helper_funcs.h"

double area(const Triangle &tr) {
    Point v1 = tr.v1;
    Point v2 = tr.v2;
    Point v3 = tr.v3;
    return std::abs((v1.x * (v2.y - v3.y) + v2.x * (v3.y - v1.y) + v3.x * (v1.y - v2.y)) / 2.0);
}

std::vector<Triangle> parse_triangles(const std::string &path) {
    std::ifstream file(path.c_str());
    if (!file.is_open()) {
        throw mesh_file_error("Error opening mesh file at path: " + path);
    }
    std::vector<Triangle> triangles;
    std::string line;
    size_t line_number = 0;
    while (std::getline(file, line)) {
        ++line_number;
        std::istringstream iss(line);
        std::vector<double> line_data;
        double value;
        while (iss >> value) {
            line_data.push_back(value);
        }
        if (line_data.size() == 6) {
            triangles.push_back(Triangle(line_data));
        } else {
            std::string line_number_str = num_to_str(line_number);
            throw mesh_file_error("Wrong number of points in file: " + path + " in line number: " + line_number_str);
        }
    }
    return triangles;
}


double orientation(const Point &p1, const Point &p2, const Point &p3) {
    return (p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y);
}

/*Check whether two parts of the line intersect*/
bool do_segments_intersect(const Point &p1, const Point &q1, const Point &p2, const Point &q2) {
    double o1 = orientation(p1, q1, p2);
    double o2 = orientation(p1, q1, q2);
    double o3 = orientation(p2, q2, p1);
    double o4 = orientation(p2, q2, q1);

    if (o1 * o2 < 0 && o3 * o4 < 0) {
        return true;
    }
    return false;
}

/*Determines coordinate bounds to draw the screen of the proper size*/
std::pair<double, double> find_coordinate_bounds(std::vector<Triangle> &obstacles) {
    double max_x = 0;
    double max_y = 0;
    for (size_t i = 0; i < obstacles.size(); ++i) {
        max_x = fmax(max_x, fmax(obstacles[i].v1.x, fmax(obstacles[i].v2.x, obstacles[i].v3.x)));
        max_y = fmax(max_y, fmax(obstacles[i].v1.y, fmax(obstacles[i].v2.y, obstacles[i].v3.y)));
    }
    return std::make_pair(max_x, max_y);
}
