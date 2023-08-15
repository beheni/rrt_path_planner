//visualization idea using SFML taken from https://github.com/nikhilchandak/Rapidly-Exploring-Random-Trees
#include <iostream>
#include <SFML/Graphics.hpp>
#include "errors.h"
#include "parser.h"
#include "RRTDraw.h"
#include "RRTGraph.h"
#include "helper_funcs.h"

//#define DRAW_NODES
//#define DRAW_RANDOM_POINT

int main(int argc, char *argv[]) {
    try {
        if (argc != 2) {
            throw arguments_count_error(
                    "Wrong arguments count. Proper usage: ./bin/path_planner_visualisation <path-to-config>");
        }
        sf::ContextSettings settings;
        settings.antialiasingLevel = 8;
        sf::Time delay_time = sf::milliseconds(1); //speed of the simulation
        config_t config = parse_and_validate_config(argv[1]);
        RRTGraph rrt_graph = RRTGraph(config);
        RRTDraw rrt_draw = RRTDraw(config);
        sf::RenderWindow window(
                sf::VideoMode(rrt_graph.get_max_x() + 10, rrt_graph.get_max_y() + 35), //magic numbers are for the text
                "Basic RRT Path Planner",
                sf::Style::Default, settings);
        bool is_paused = false;
        bool is_final = false;
        size_t iteration_number = 0;
        while (window.isOpen()) {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    window.close();
                    return 0;
                }
                if (event.type == sf::Event::KeyPressed) {
                    if (event.key.code == sf::Keyboard::Space && !is_final) {
                        is_paused = !is_paused;
                    }
                }
            }
            if (!is_paused) {
                rrt_draw.draw_obstacles(window, rrt_graph.get_obstacles());
                Point p = rrt_graph.extend_tree();
                iteration_number++;
#ifdef DRAW_NODES
                rrt_draw.draw_nodes(window, rrt_graph.get_tree());
#endif
#ifdef DRAW_RANDOM_POINT
                rrt_draw.draw_point(window, p, sf::Color::Yellow, 3);
#endif
                rrt_draw.draw_edges(window, rrt_graph.get_edges());
                if (rrt_graph.end_reached()) {
                    rrt_draw.draw_path_to_goal(window, rrt_graph.get_path_to_goal());
                    is_paused = true;
                    is_final = true;
                    double path_len = (rrt_graph.get_path_to_goal().size() - 1) * config.step_size +
                                      rrt_graph.get_edges().back().start_v.distance(rrt_graph.get_edges().back().end_v);
                    rrt_draw.draw_text(window, "Path found of length: " + num_to_str(path_len),
                                       Point(rrt_graph.get_max_x() - 240, rrt_graph.get_max_y() + 5));
                }
                rrt_draw.draw_text(window, "Iteration number: " + num_to_str(iteration_number), Point(5, 0));
                rrt_draw.draw_text(window, "Press Space to start/pause the animation",
                                   Point(10, rrt_graph.get_max_y() + 5));
                rrt_draw.draw_init_positions(window);
                window.display();
                sf::sleep(delay_time);
                window.clear();
            }
        }
    }
    catch (const base_exception &ex) {

        std::cerr << ex.what() << std::endl;
        return ex.error_code;
    }

    return ALL_GOOD;
}



