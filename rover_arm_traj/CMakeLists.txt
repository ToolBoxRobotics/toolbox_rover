cmake_minimum_required(VERSION 3.0.2)
project(rover_arm_traj)

find_package(catkin REQUIRED COMPONENTS
  roscpp actionlib control_msgs trajectory_msgs rover_msgs sensor_msgs
)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(traj_splitter src/traj_splitter.cpp)
target_link_libraries(traj_splitter ${catkin_LIBRARIES})
add_dependencies(traj_splitter ${catkin_EXPORTED_TARGETS})
