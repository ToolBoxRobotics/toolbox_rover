#pragma once
#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/velocity_joint_interface.h>
#include <hardware_interface/position_joint_interface.h>
#include <rover_msgs/WheelCmd.h>
#include <rover_msgs/SteerCmd.h>
#include <rover_msgs/ArmCmd.h>
#include <sensor_msgs/JointState.h>

namespace rover_hardware {

class RoverHW : public hardware_interface::RobotHW {
public:
  RoverHW(ros::NodeHandle& nh);
  void read(const ros::Duration& period);
  void write(const ros::Duration& period);

private:
  ros::NodeHandle nh_;
  static const int NW = 6, NS = 4, NA = 5;
  std::string wheel_names_[NW];
  std::string steer_names_[NS];
  std::string arm_names_[NA];

  double wheel_pos_[NW], wheel_vel_[NW], wheel_eff_[NW];
  double steer_pos_[NS], steer_vel_[NS], steer_eff_[NS];
  double arm_pos_[NA],   arm_vel_[NA],   arm_eff_[NA];

  double wheel_cmd_[NW];
  double steer_cmd_[NS];
  double arm_cmd_[NA];

  hardware_interface::JointStateInterface jnt_state_if_;
  hardware_interface::VelocityJointInterface wheel_vel_if_;
  hardware_interface::PositionJointInterface steer_pos_if_;
  hardware_interface::PositionJointInterface arm_pos_if_;

  ros::Subscriber wheel_state_sub_;
  ros::Subscriber arm_state_sub_;
  ros::Publisher wheel_cmd_pub_;
  ros::Publisher steer_cmd_pub_;
  ros::Publisher arm_cmd_pub_;

  sensor_msgs::JointState last_wheel_js_;
  sensor_msgs::JointState last_arm_js_;
  bool got_wheels_{false}, got_arm_{false};

  void wheelStateCb(const sensor_msgs::JointState& js);
  void armStateCb(const sensor_msgs::JointState& js);
};

} // namespace
