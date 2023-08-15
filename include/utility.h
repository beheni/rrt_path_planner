#ifndef RRT_PLANNER_UTILITY_H
#define RRT_PLANNER_UTILITY_H

#include <cmath>
#include <vector>
#include <fstream>

struct Triangle;
struct Edge;
struct Node;
struct Point;

const double EPS = 1e-6;

struct Point {
    double x, y;

    Point(double x, double y);

    Point(const Point &other);

    Point();

    bool is_inside_triangle(const Triangle &tr) const;

    bool is_inside_obstacles(std::vector<Triangle> &) const;

    double distance(const Point &other) const;

    bool is_inside_circle(const Point &centre, double radius) const;

    bool do_crosses_edge(const Edge &edge, double radius) const;

    std::vector<double> get_coords() const;

    bool operator!=(const Point &other) const;
};

struct Edge {
public:
    Point start_v, end_v;

    Edge();

    Edge(const Point &start, const Point &end);

    Edge(Node *start, Node *end);

    bool is_collision(const std::vector<Triangle> &obstacles, double radius) const;

    double dot_product(const Edge &other) const;

    double angle_between(const Edge &other) const;

    double cross_product(const Edge &other) const;
};

struct Triangle {
    Point v1, v2, v3;
    Edge e1, e2, e3;

    Triangle(const Point &v1, const Point &v2, const Point &v3);

    Triangle(const std::vector<double> &);

    std::vector<Point> get_vertices() const;

    bool intersect_circle(const Point &centre, double radius) const;
};

std::ostream &operator<<(std::ostream &os, const Triangle &tr);

struct Node {
public:
    Point coords;
    Node *left;
    Node *right;
    Node *parent;

    Node();

    Node(double x, double y, Node *left = NULL, Node *right = NULL, Node *parent = NULL);

    std::vector<double> get_coords() const;

    double distance(const Point &point) const;
};


#endif //RRT_PLANNER_UTILITY_H
