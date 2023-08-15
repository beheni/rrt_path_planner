# Simple 2D RRT-based Path Planner with Visualisation

## Prerequisites
+ gcc
+ cmake

## Compilation
In the project directory just do:
```
chmod +x compile.sh
./compile.sh
```
It makes two independent executables `rrt_path_planner` and `path_planner_visualisation` in the `bin` folder built in `Release`

The first executable solves the task and outputs into text files in the `result` folder.

The second one visualises every step of the algorithm.

### Important note
I used the coordinate system where `(0, 0)` point is in the upper left corner. It was 
more convenient for the visualisation (because SFML uses such coords by default) and isn't very hard to change.
## Usage
For both executables you need to provide the path to the `.cfg` file as. The format is described later.
```
./bin/rrt_path_planner configs/map.cfg
```

```
./bin/path_planner_visualisation configs/map.cfg
```

### Format of the config file
It requires the following options to customize the flow of the path planner.
```
mesh_file=2D/map/map.tri
tree_file=tree.txt
trajectory_file=traj.txt
step_size=10
start_x=100
start_y=100
end_x=530
end_y=410
robot_height=10
robot_width=10
```
`step_size` is the length of the edge in the rrt. All other options are pretty self-explanatory :+)
`mesh_file` is the path to **.tri** format that contains 2D triangle describing obstacles, one triangle per line in format:
`x1 y1 x2 y2 x3 y3`. See examples in `2D` folder. 

## Examples of Visualisation
Visualisation can be configured through the #DRAW_NODE and #DRAW_RANDOM_POINT macroses in [src/visualisation_main.cpp](https://mrs.felk.cvut.cz/gitlab/beheni/planningbase/-/blob/master/src/visualisation_main.cpp)
- Example with only edges 

https://github.com/beheni/rrt_path_planner/assets/91615487/c1d51a2b-819e-4bc4-9502-73ba1a3ab503
- Example with nodes

https://github.com/beheni/rrt_path_planner/assets/91615487/21f3c3ba-f7f0-4d58-ba36-37a5148472c2
- Example with the randomly sampled node

https://github.com/beheni/rrt_path_planner/assets/91615487/5f91c3e6-6243-40d7-8709-ce518c4524d8




