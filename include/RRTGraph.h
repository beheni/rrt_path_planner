#ifndef RRT_PLANNER_RRTGRAPH_H
#define RRT_PLANNER_RRTGRAPH_H

#include <cmath>
#include <vector>
#include <string>
#include <cstdlib>
#include "parser.h"
#include "utility.h"

class RRTGraph {
public:

    RRTGraph(const config_t &config);

    void generate_rrt();

    Point extend_tree();

    bool end_reached();

    ~RRTGraph();

    std::vector<Triangle> get_obstacles() const;

    std::vector<Node *> get_path_to_goal() const;

    std::vector<Node *> get_tree() const;

    std::vector<Edge> get_edges() const;

    double get_max_x() const;

    double get_max_y() const;

    double get_min_x() const;

    double get_min_y() const;

private:
    config_t config;
    Node *root;
    std::vector<Edge> edges;
    Edge original_direction;
    std::vector<Triangle> obstacles;
    Point ending_point, starting_point;
    std::vector<Node *> path_to_goal;
    std::vector<Node *> tree;
    double max_x, max_y, min_x, min_y;
    double circumscribed_circle_radius;

    Node *_add_node(Node *root, Node *node);

    Node *_get_nearest_node(Node *root_, const Point &target_point, int depth = 0);

    Node *get_nearest_node(const Point &target_point);

    Point generate_random_point();

    Node *step_near(Node *tree_node, Point &rand_point) const;

    void add_node(Node *new_node);

    void write_trajectory() const;

    void write_tree() const;

};

#endif //RRT_PLANNER_RRTGRAPH_H
