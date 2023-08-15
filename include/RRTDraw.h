#ifndef RRT_PLANNER_RRTDRAW_H
#define RRT_PLANNER_RRTDRAW_H

#include <string>
#include <SFML/Graphics.hpp>
#include "parser.h"
#include "utility.h"
#include "RRTGraph.h"

class RRTDraw {
private:
    config_t config;
    Point starting_point, ending_point;
    std::vector<Edge> edges;
    std::vector<Triangle> obstacles;
public:
    RRTDraw(const config_t &config);

    void draw_obstacles(sf::RenderWindow &window, std::vector<Triangle> obstacles) const;

    void draw_nodes(sf::RenderWindow &window, const std::vector<Node *> &nodes) const;

    void draw_point(sf::RenderWindow &window, const Point &point, sf::Color, int radius) const;

    void draw_edges(sf::RenderWindow &window, const std::vector<Edge> &edges) const;

    void draw_path_to_goal(sf::RenderWindow &window, const std::vector<Node *> &path) const;

    void draw_init_positions(sf::RenderWindow &window) const;

    void draw_text(sf::RenderWindow &window, const std::string &text, const Point &position) const;

private:
    void draw_edge(sf::RenderWindow &window, const Edge &edge, sf::Color color) const;
};

#endif //RRT_PLANNER_RRTDRAW_H
