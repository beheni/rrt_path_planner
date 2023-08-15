#include <cstdlib>
#include <fstream>
#include <iostream>
#include "errors.h"
#include "parser.h"
#include "RRTGraph.h"
#include "helper_funcs.h"

#define DISCARD_POINTS_ON_OBSTACLES

RRTGraph::RRTGraph(const config_t &config) : config(config) {
    root = new Node(config.start_x, config.start_y, NULL, NULL, NULL);
    tree.push_back(root);
    ending_point = Point(config.end_x, config.end_y);
    starting_point = Point(config.start_x, config.start_y);
    obstacles = parse_triangles(config.mesh_file);
    std::pair<double, double> bounds = find_coordinate_bounds(obstacles);
    max_x = bounds.first;
    max_y = bounds.second;
    min_x = 0;
    min_y = 0;
    circumscribed_circle_radius =
            (sqrt(config.robot_width * config.robot_width + config.robot_height * config.robot_height)) / 2;
    original_direction = Edge(starting_point, Point(starting_point.x, starting_point.y + config.step_size));
}

/*Moves from tree_node into the direction of rand_point for a distance of length step_size*/
Node *RRTGraph::step_near(Node *tree_node, Point &rand_point) const {
    double ratio = config.step_size / tree_node->distance(rand_point);
    double new_x = tree_node->coords.x + ratio * (rand_point.x - tree_node->coords.x);
    double new_y = tree_node->coords.y + ratio * (rand_point.y - tree_node->coords.y);
    if (tree_node->distance(rand_point) < config.step_size) {
        if ((tree_node->distance(ending_point) < config.step_size)) {
            return new Node(ending_point.x, ending_point.y);
        }
        return tree_node;
    }
    return new Node(new_x, new_y);
}

Point RRTGraph::generate_random_point() {
    Point random_point;
    bool safe_point_found = false;
    int x = rand() % static_cast<int>((max_x - min_x + 1) + min_x);
    int y = rand() % static_cast<int>((max_y - min_y + 1) + min_y);
    random_point = Point(x, y);
#ifdef DISCARD_POINTS_ON_OBSTACLES //i am not sure about the impact of changing this macros
    while (!safe_point_found) {
        if (!random_point.is_inside_obstacles(obstacles)) {
            safe_point_found = true;
        } else {
            x = rand() % static_cast<int>((max_x - min_x + 1) + min_x);
            y = rand() % static_cast<int>((max_y - min_y + 1) + min_y);
            random_point = Point(x, y);
        }
    }
#endif
    return random_point;
}

/*Helper func to determine who is closer to target p1 or p2*/
Node *closer_distance(const Point &target, Node *p1, Node *p2) {
    if (p1 == NULL) {
        return p2;
    }
    if (p2 == NULL) {
        return p1;
    }
    double d1 = p1->distance(target);
    double d2 = p2->distance(target);

    if (d1 < d2) {
        return p1;
    } else {
        return p2;
    }
}

/*Wrapper around the uglier function*/
Node *RRTGraph::get_nearest_node(const Point &target_point) {
    return _get_nearest_node(root, target_point, 0);
}

Node *RRTGraph::_get_nearest_node(Node *root_, const Point &target_point, int depth) {
    if (root_ == NULL) {
        return NULL;
    }
    Node *nearest;
    int axis = depth % 2;

    Node *next_branch = NULL;
    Node *opposite_branch = NULL;
    if (target_point.get_coords()[axis] < root_->get_coords()[axis]) {
        next_branch = root_->left;
        opposite_branch = root_->right;
    } else {
        next_branch = root_->right;
        opposite_branch = root_->left;
    }
    /*Handles the edge case when the nearest node is not is the same partition as a target point*/
    nearest = closer_distance(target_point, _get_nearest_node(next_branch, target_point, depth + 1), root_);
    if (nearest->distance(target_point) > std::abs(target_point.get_coords()[axis] - root_->get_coords()[axis])) {
        nearest = closer_distance(target_point, _get_nearest_node(opposite_branch, target_point, depth + 1), nearest);
    }
    return nearest;
}

/*Checks is it time to stop*/
bool RRTGraph::end_reached() {
    double distance_to_end = get_nearest_node(Point(ending_point.x, ending_point.y))->distance(ending_point);
    bool end_reached = distance_to_end <= config.step_size;
    if (end_reached) {
        Node *ending_node = new Node(ending_point.x, ending_point.y, NULL, NULL, tree.back());
        add_node(ending_node);

        Node *last_node = tree.back();
        while (last_node->parent != NULL) {
            path_to_goal.push_back(last_node);
            last_node = last_node->parent;
        }
        path_to_goal.push_back(last_node);
    }

    return end_reached;
}

/*Another wrapper*/
void RRTGraph::add_node(Node *new_node) {
    _add_node(root, new_node);
    tree.push_back(new_node);
    edges.push_back(Edge(new_node->parent, new_node));
}

/*Traverses the tree from the root in order to find the proper place for new_node*/
Node *RRTGraph::_add_node(Node *root_, Node *new_node) {
    Node *x = root_;
    Node *y = NULL;
    int depth = 0;
    int axis = depth % 2;
    while (x != NULL) {
        y = x;
        if (new_node->get_coords()[axis] < (root_->get_coords()[axis])) {
            x = x->left;
        } else {
            x = x->right;
        }
    }
    if (y == NULL)
        y = new_node;
    if (new_node->get_coords()[axis] < (root_->get_coords()[axis])) {
        y->left = new_node;
    } else {
        y->right = new_node;
    }

    return y;
}

/*Performs one iteration of the algorithm until the new collision free edge is found*/
Point RRTGraph::extend_tree() {
    Point rand_point = generate_random_point();
    Node *step;
    Edge new_edge;
    Node *nearest = get_nearest_node(rand_point);
    bool safe_edge_found = false;
    while (!safe_edge_found) {
        step = step_near(nearest, rand_point);
        new_edge = Edge(nearest, step);
        if (!new_edge.is_collision(obstacles, circumscribed_circle_radius)) {
            safe_edge_found = true;
        } else {
            rand_point = generate_random_point();
            nearest = get_nearest_node(rand_point);
            delete step;
        }
    }
    if (nearest == step) {
        return rand_point;
    }
    step->parent = nearest;
    add_node(step);
    return rand_point;
}

/*Writes one of the resulting files*/
void RRTGraph::write_trajectory() const {
    std::ofstream file(config.trajectory_file.c_str());
    if (!file.is_open()) {
        throw result_file_error("Error opening file with for storing robot's path");
    }
    file << path_to_goal[path_to_goal.size() - 1]->coords.x << " "
         << path_to_goal[path_to_goal.size() - 1]->coords.y
         << " "
         << 0.0 << "\n";

    Edge robot_direction = original_direction;

    for (int i = static_cast<int>(path_to_goal.size() - 1); i > 0; --i) {
        Edge new_robot_direction = Edge(path_to_goal[i]->coords, path_to_goal[i - 1]->coords);
        file << path_to_goal[i - 1]->coords.x - path_to_goal[i]->coords.x << " "
             << path_to_goal[i - 1]->coords.y - path_to_goal[i]->coords.y << " "
             << new_robot_direction.angle_between(robot_direction) << "\n";
        robot_direction = new_robot_direction;
    }
}

/*Writes one of the resulting files*/
void RRTGraph::write_tree() const {
    std::ofstream file(config.tree_file.c_str());
    if (!file.is_open()) {
        throw result_file_error("Error opening file with for storing final tree");
    }
    for (size_t i = 0; i < edges.size(); ++i) {
        file << edges[i].start_v.x << " " << edges[i].start_v.y << " " << edges[i].end_v.x << " "
             << edges[i].end_v.y
             << "\n";
    }
    file << std::endl;
}

void RRTGraph::generate_rrt() {
    while (!end_reached()) {
        extend_tree();
    }
    write_trajectory();
    write_tree();
}

std::vector<Triangle> RRTGraph::get_obstacles() const {
    return obstacles;
}

std::vector<Node *> RRTGraph::get_path_to_goal() const {
    return path_to_goal;
}

std::vector<Node *> RRTGraph::get_tree() const {
    return tree;
}

std::vector<Edge> RRTGraph::get_edges() const {
    return edges;
}

double RRTGraph::get_max_x() const {
    return max_x;
}

double RRTGraph::get_max_y() const {
    return max_y;
}

double RRTGraph::get_min_x() const {
    return min_x;
}

double RRTGraph::get_min_y() const {
    return min_y;
}

RRTGraph::~RRTGraph() {
    for (int i = 0; i < tree.size(); i++) {
        delete tree.at(i);
    }
    tree.clear();
}
