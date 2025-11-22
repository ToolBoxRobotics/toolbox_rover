# Toolbox Rover — ROS Noetic Full Stack

This repo contains a full ROS Noetic environment for a six‑wheeled, four‑corner‑steered rover with a 5‑axis stepper arm.

Packages:
- `rover_msgs` — custom messages
- `rover_base` — cmd_vel → wheel/steer commands + odom
- `rover_bringup` — Pi/PC launch
- `rover_description` — full URDF/Xacro + unified ros_control controllers
- `rover_sim` — Gazebo sim + bridge
- `rover_hardware` — ros_control RobotHW for real robot
- `rover_arm` — homing + safety clamping
- `rover_arm_traj` — FollowJointTrajectory action server that splits to ArmCmd
- `toolbox_rover_moveit` — MoveIt Setup Assistant–style package

See `README_install.md` for full install steps.
