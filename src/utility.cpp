#include <cmath>
#include <vector>
#include <iostream>
#include "utility.h"
#include "helper_funcs.h"


Triangle::Triangle(const Point &v1, const Point &v2, const Point &v3) : v1(v1), v2(v2), v3(v3) {
    e1 = Edge(v1, v2);
    e2 = Edge(v2, v3);
    e3 = Edge(v3, v1);
}

Triangle::Triangle(const std::vector<double> &line_vertices) {
    v1 = Point(line_vertices[0], line_vertices[1]);
    v2 = Point(line_vertices[2], line_vertices[3]);
    v3 = Point(line_vertices[4], line_vertices[5]);
    e1 = Edge(v1, v2);
    e2 = Edge(v2, v3);
    e3 = Edge(v3, v1);
}

/*Performs all the tests to check whether the circle and the triangle intersect*/
bool Triangle::intersect_circle(const Point &centre, double radius) const {
    bool some_vertex_inside = v1.is_inside_circle(centre, radius) || v2.is_inside_circle(centre, radius) ||
                              v3.is_inside_circle(centre, radius);
    if (some_vertex_inside) {
        return true;
    }
    bool is_circle_inside = centre.is_inside_triangle(*this);
    if (is_circle_inside) {
        return true;
    }
    bool do_edge_crosses = centre.do_crosses_edge(e1, radius) || centre.do_crosses_edge(e2, radius) ||
                           centre.do_crosses_edge(e3, radius);
    return do_edge_crosses;
}

std::vector<Point> Triangle::get_vertices() const {
    std::vector<Point> vertices;
    vertices.push_back(v1);
    vertices.push_back(v2);
    vertices.push_back(v3);
    return vertices;
}

bool Point::is_inside_triangle(const Triangle &tr) const {
    double A = area(tr);
    Triangle tr1 = Triangle(tr.v1, tr.v2, *this);
    double A1 = area(tr1);
    Triangle tr2 = Triangle(tr.v1, *this, tr.v3);
    double A2 = area(tr2);
    Triangle tr3 = Triangle(*this, tr.v2, tr.v3);
    double A3 = area(tr3);
    return (A == A1 + A2 + A3);
}

std::vector<double> Point::get_coords() const {
    std::vector<double> coords;
    coords.push_back(x);
    coords.push_back(y);
    return coords;
}


Point::Point(double x, double y) : x(x), y(y) {}

Point::Point(const Point &other) {
    x = other.x;
    y = other.y;
}

bool Point::operator!=(const Point &other) const {
    return !((fabs(x - other.x) < EPS && fabs(y - other.y) < EPS));
}

Point::Point() {
    x = 0;
    y = 0;
}

bool Point::is_inside_obstacles(std::vector<Triangle> &obstacles) const {
    for (int i = 0; i < obstacles.size(); ++i) {
        if (is_inside_triangle(obstacles[i])) {
            return true;
        }
    }
    return false;
}

bool Point::do_crosses_edge(const Edge &edge, double radius) const {
    /*all the complexity is about handling the edge case when the distance from circle's center
    is smaller than radius, but they still to ont intersect*/
    double cx = x - edge.start_v.x;
    double cy = y - edge.start_v.y;
    double ex = edge.end_v.x - edge.start_v.x;
    double ey = edge.end_v.y - edge.start_v.y;
    double k = cx * ex + cy * ey;
    if (k > 0) {
        double len_edge = edge.start_v.distance(edge.end_v);
        k = k / len_edge;
        if (k < len_edge) {
            if (sqrt(cx * cx + cy * cy - k * k) <= radius) {
                return true;
            }
        }
    }
    return false;
}

double Point::distance(const Point &other) const {
    double x_diff = other.x - x;
    double y_diff = other.y - y;
    return sqrt((x_diff * x_diff) + (y_diff * y_diff));
}

bool Point::is_inside_circle(const Point &centre, double radius) const {
    return distance(centre) <= radius;
}

Edge::Edge(const Point &start, const Point &end) : start_v(start), end_v(end) {}

Edge::Edge(Node *start, Node *end) {
    start_v.x = start->coords.x;
    start_v.y = start->coords.y;
    end_v.x = end->coords.x;
    end_v.y = end->coords.y;
}

Edge::Edge() {
    start_v = Point();
    end_v = Point();
}

double Edge::dot_product(const Edge &other) const {
    double dx1 = end_v.x - start_v.x;
    double dy1 = end_v.y - start_v.y;
    double dx2 = other.end_v.x - other.start_v.x;
    double dy2 = other.end_v.y - other.start_v.y;
    return dx1 * dx2 + dy1 * dy2;
}

double Edge::cross_product(const Edge &other) const {
    double dx1 = end_v.x - start_v.x;
    double dy1 = end_v.y - start_v.y;
    double dx2 = other.end_v.x - other.start_v.x;
    double dy2 = other.end_v.y - other.start_v.y;
    return dx1 * dy2 - dx2 * dy1;
}

double Edge::angle_between(const Edge &other) const {
    double dot = dot_product(other);
    double cross = cross_product(other);
    double angle_rad = atan2(cross, dot);
    double angle = angle_rad * 180.0 / M_PI;
    return angle;
}

/*Checks whether the edge is collision free for our robot*/
bool Edge::is_collision(const std::vector<Triangle> &obstacles, double radius) const {
    for (size_t i = 0; i < obstacles.size(); ++i) {
        const Triangle &triangle = obstacles[i];
        if (do_segments_intersect(start_v, end_v, triangle.v1, triangle.v2) ||
            do_segments_intersect(start_v, end_v, triangle.v2, triangle.v3) ||
            do_segments_intersect(start_v, end_v, triangle.v3, triangle.v1) ||
            obstacles[i].intersect_circle(end_v, radius)) {
            return true;
        }
    }
    return false;
}

Node::Node(double x, double y, Node *left, Node *right, Node *parent) : left(left), right(right), parent(parent) {
    coords.x = x;
    coords.y = y;
}

Node::Node() {
    left = NULL;
    right = NULL;
    parent = NULL;
    coords.x = 0;
    coords.y = 0;
}

std::vector<double> Node::get_coords() const {
    return coords.get_coords();
}

double Node::distance(const Point &point) const {
    double x_diff = point.x - coords.x;
    double y_diff = point.y - coords.y;
    return sqrt((x_diff * x_diff) + (y_diff * y_diff));
}

std::ostream &operator<<(std::ostream &os, const Triangle &tr) {
    os << "Triangle with vertices: ";
    os << "(" << tr.v1.x << ", " << tr.v1.y << ")";
    os << " (" << tr.v2.x << ", " << tr.v2.y << ")";
    os << " (" << tr.v3.x << ", " << tr.v3.y << ")" << std::endl;
    return os;
}

