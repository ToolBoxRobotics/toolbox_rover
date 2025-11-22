cmake_minimum_required(VERSION 3.0.2)
project(rover_hardware)

find_package(catkin REQUIRED COMPONENTS
  roscpp hardware_interface controller_manager rover_msgs sensor_msgs
)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(rover_hw src/rover_hw.cpp)
add_executable(rover_hw_node src/rover_hw_node.cpp)
target_link_libraries(rover_hw_node rover_hw ${catkin_LIBRARIES})
add_dependencies(rover_hw_node ${catkin_EXPORTED_TARGETS})
