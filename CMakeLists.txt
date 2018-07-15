cmake_minimum_required(VERSION 3.10)
project(hybrid_astar_global_planner)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)
include_directories(include/hybrida_global_planner)
include_directories(/opt/ros/kinetic/include)
add_executable(hybrid_astar_global_planner
        include/hybrid_astar/algorithm.h
        include/hybrid_astar/bucketedqueue.h
        include/hybrid_astar/collisiondetection.h
        include/hybrid_astar/constants.h
        include/hybrid_astar/dubins.h
        include/hybrid_astar/dynamicvoronoi.h
        include/hybrid_astar/gradient.h
        include/hybrid_astar/helper.h
        include/hybrid_astar/lookup.h
        include/hybrid_astar/node2d.h
        include/hybrid_astar/node3d.h
        include/hybrid_astar/path.h
        include/hybrid_astar/planner.h
        include/hybrid_astar/point.h
        include/hybrid_astar/smoother.h
        include/hybrid_astar/vector2d.h
        include/hybrid_astar/visualize.h
        include/hybrida_global_planner/astar.h
        include/hybrida_global_planner/dijkstra.h
        include/hybrida_global_planner/expander.h
        include/hybrida_global_planner/gradient_path.h
        include/hybrida_global_planner/grid_path.h
        include/hybrida_global_planner/orientation_filter.h
        include/hybrida_global_planner/planner_core.h
        include/hybrida_global_planner/potential_calculator.h
        include/hybrida_global_planner/quadratic_calculator.h
        include/hybrida_global_planner/traceback.h

        src/astar.cpp
        src/dijkstra.cpp
        src/gradient_path.cpp
        src/grid_path.cpp
        src/orientation_filter.cpp
        src/plan_node.cpp
        src/planner_core.cpp
        src/quadratic_calculator.cpp
        src/hybrid_astar/algorithm.cpp
        src/hybrid_astar/bucketedqueue.cpp
        src/hybrid_astar/collisiondetection.cpp
        src/hybrid_astar/dubins.cpp
        src/hybrid_astar/dynamicvoronoi.cpp
        src/hybrid_astar/node2d.cpp
        src/hybrid_astar/node3d.cpp
        src/hybrid_astar/path.cpp
        src/hybrid_astar/planner.cpp
        src/hybrid_astar/smoother.cpp
        src/hybrid_astar/tf_broadcaster.cpp
        src/hybrid_astar/visualize.cpp
        )
