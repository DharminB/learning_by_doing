cmake_minimum_required(VERSION 2.8.3)
project(my_simple_manipulator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  kdl_parser
  tf
  kdl_conversions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_simple_manipulator
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  ${PROJECT_SOURCE_DIR}/ros/src
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_node ros/src/test_node.cpp)
add_executable(arm_controller_without_gazebo
    ros/src/arm_controller_without_gazebo.cpp
    ros/src/pid_controller.cpp)
add_executable(arm_controller
    ros/src/arm_controller.cpp
    ros/src/pid_controller.cpp)

target_link_libraries(test_node ${catkin_LIBRARIES})
target_link_libraries(arm_controller_without_gazebo ${catkin_LIBRARIES})
target_link_libraries(arm_controller ${catkin_LIBRARIES})
