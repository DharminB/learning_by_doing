cmake_minimum_required(VERSION 2.8.3)
project(my_simple_manipulator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  tf
)

catkin_package(
)

include_directories(
  ${PROJECT_SOURCE_DIR}/ros/include
  ${PROJECT_SOURCE_DIR}/common/include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(test_node
#     ros/src/test_node.cpp)
# target_link_libraries(test_node
#     ${catkin_LIBRARIES})

# add_executable(arm_controller_without_gazebo
#     ros/src/arm_controller_without_gazebo.cpp
#     ros/src/pid_controller.cpp)
# target_link_libraries(arm_controller_without_gazebo
#    ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}
  common/src/kinematics.cpp
  common/src/pid_controller.cpp
  ros/src/arm_controller.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(arm_controller
    ros/src/arm_controller.cpp
)
target_link_libraries(arm_controller
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

# add_executable(youbot_kinematics
#     ros/src/youbot_kinematics.cpp)
# target_link_libraries(youbot_kinematics ${catkin_LIBRARIES})
