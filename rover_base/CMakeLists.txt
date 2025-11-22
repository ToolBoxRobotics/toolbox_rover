cmake_minimum_required(VERSION 3.0.2)
project(rover_base)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros rover_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(rover_controller src/rover_controller.cpp)
add_dependencies(rover_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(rover_controller ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/odom_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
