cmake_minimum_required(VERSION 2.8.3)
project(tic_tac_toe_with_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_simple_manipulator
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)
