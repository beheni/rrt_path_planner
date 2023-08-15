#include "RRTDraw.h"

RRTDraw::RRTDraw(const config_t &config) : config(config) {
    starting_point = Point(config.start_x, config.start_y);
    ending_point = Point(config.end_x, config.end_y);
}

void RRTDraw::draw_edge(sf::RenderWindow &window, const Edge &edge, sf::Color color) const {
    sf::Vector2f point1(edge.start_v.x, edge.start_v.y);
    sf::Vector2f point2(edge.end_v.x, edge.end_v.y);
    sf::VertexArray line(sf::Lines, 2);
    line[0].position = point1;
    line[1].position = point2;
    line[0].color = color;
    line[1].color = color;
    window.draw(line);
}

void RRTDraw::draw_edges(sf::RenderWindow &window, const std::vector<Edge> &edges) const {
    for (size_t i = 0; i < edges.size(); ++i) {
        draw_edge(window, edges[i], sf::Color::White);
    }
}

void RRTDraw::draw_point(sf::RenderWindow &window, const Point &point, sf::Color color, int radius = 3) const {
    sf::CircleShape circle;
    circle.setFillColor(color);
    circle.setPosition(point.x, point.y);
    circle.setRadius(radius);
    circle.setOrigin(radius / 2, radius / 2);
    window.draw(circle);
}

void RRTDraw::draw_nodes(sf::RenderWindow &window, const std::vector<Node *> &nodes) const {
    sf::CircleShape random_circle;
    for (size_t i = 0; i < nodes.size(); ++i) {
        draw_point(window, nodes[i]->coords, sf::Color::Green);
    }
}

void RRTDraw::draw_obstacles(sf::RenderWindow &window, std::vector<Triangle> obstacles) const {
    std::vector<sf::ConvexShape> polygons;
    polygons.resize(obstacles.size());
    for (size_t i = 0; i < obstacles.size(); ++i) {
        polygons[i].setPointCount(3);
        polygons[i].setFillColor(sf::Color(89, 87, 98));
        std::vector<Point> vertices = obstacles[i].get_vertices();
        for (int j = 0; j < 3; j++)
            polygons[i].setPoint(j, sf::Vector2f(vertices[j].x, vertices[j].y));
    }
    for (size_t i = 0; i < polygons.size(); ++i) {
        window.draw(polygons[i]);
    }
}

/*Draws the "robot" and his goal*/
void RRTDraw::draw_init_positions(sf::RenderWindow &window) const {
    sf::RectangleShape rectangle;
    rectangle.setSize(sf::Vector2f(config.robot_width, config.robot_height)); // Set the size of the rectangle
    rectangle.setPosition(starting_point.x, starting_point.y); // Set the position of the rectangle
    rectangle.setFillColor(sf::Color(248, 24, 148));
    window.draw(rectangle);
    draw_point(window, ending_point, sf::Color::Blue, 4);
}

void RRTDraw::draw_path_to_goal(sf::RenderWindow &window, const std::vector<Node *> &path) const {
    for (size_t i = 0; i < path.size() - 1; ++i) {
        draw_edge(window, Edge(path[i], path[i + 1]), sf::Color::Red);
    }
}

void RRTDraw::draw_text(sf::RenderWindow &window, const std::string &text_to_write, const Point &position) const {
    sf::Font font;
    font.loadFromFile("fonts/arial.ttf");
    sf::Text text(text_to_write, font, 17);
    text.setFillColor(sf::Color::White);
    text.setPosition(position.x, position.y);
    window.draw(text);
}