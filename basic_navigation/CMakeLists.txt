cmake_minimum_required(VERSION 2.8.3)
project(basic_navigation)

find_package(catkin REQUIRED
    my_simple_mobile_base
    costmap_2d
    global_planner
)

catkin_python_setup()

catkin_package()
