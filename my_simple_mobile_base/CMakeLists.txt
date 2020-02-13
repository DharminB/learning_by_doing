cmake_minimum_required(VERSION 2.8.3)
project(my_simple_mobile_base)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)

catkin_package(
)
