#include "rover_hardware/rover_hw.h"
#include <cmath>
#include <algorithm>
#include <map>

using namespace rover_hardware;

RoverHW::RoverHW(ros::NodeHandle& nh): nh_(nh) {
  wheel_names_[0]="wheel_fl_wheel_joint";
  wheel_names_[1]="wheel_fr_wheel_joint";
  wheel_names_[2]="wheel_ml_wheel_joint";
  wheel_names_[3]="wheel_mr_wheel_joint";
  wheel_names_[4]="wheel_rl_wheel_joint";
  wheel_names_[5]="wheel_rr_wheel_joint";

  steer_names_[0]="wheel_fl_steer_joint";
  steer_names_[1]="wheel_fr_steer_joint";
  steer_names_[2]="wheel_rl_steer_joint";
  steer_names_[3]="wheel_rr_steer_joint";

  arm_names_[0]="joint1_link_joint";
  arm_names_[1]="joint2_link_joint";
  arm_names_[2]="joint3_link_joint";
  arm_names_[3]="joint4_link_joint";
  arm_names_[4]="joint5_link_joint";

  auto zero = [](double* a, int n){ for(int i=0;i<n;i++) a[i]=0.0; };
  zero(wheel_pos_,NW); zero(wheel_vel_,NW); zero(wheel_eff_,NW);
  zero(steer_pos_,NS); zero(steer_vel_,NS); zero(steer_eff_,NS);
  zero(arm_pos_,NA);   zero(arm_vel_,NA);   zero(arm_eff_,NA);
  zero(wheel_cmd_,NW); zero(steer_cmd_,NS); zero(arm_cmd_,NA);

  for(int i=0;i<NW;i++){
    hardware_interface::JointStateHandle sh(wheel_names_[i], &wheel_pos_[i], &wheel_vel_[i], &wheel_eff_[i]);
    jnt_state_if_.registerHandle(sh);
    hardware_interface::JointHandle vh(sh, &wheel_cmd_[i]);
    wheel_vel_if_.registerHandle(vh);
  }
  for(int i=0;i<NS;i++){
    hardware_interface::JointStateHandle sh(steer_names_[i], &steer_pos_[i], &steer_vel_[i], &steer_eff_[i]);
    jnt_state_if_.registerHandle(sh);
    hardware_interface::JointHandle ph(sh, &steer_cmd_[i]);
    steer_pos_if_.registerHandle(ph);
  }
  for(int i=0;i<NA;i++){
    hardware_interface::JointStateHandle sh(arm_names_[i], &arm_pos_[i], &arm_vel_[i], &arm_eff_[i]);
    jnt_state_if_.registerHandle(sh);
    hardware_interface::JointHandle ph(sh, &arm_cmd_[i]);
    arm_pos_if_.registerHandle(ph);
  }

  registerInterface(&jnt_state_if_);
  registerInterface(&wheel_vel_if_);
  registerInterface(&steer_pos_if_);
  registerInterface(&arm_pos_if_);

  wheel_state_sub_ = nh_.subscribe("/rover/wheel_states", 1, &RoverHW::wheelStateCb, this);
  arm_state_sub_   = nh_.subscribe("/arm/joint_states",  1, &RoverHW::armStateCb, this);

  wheel_cmd_pub_ = nh_.advertise<rover_msgs::WheelCmd>("/rover/wheel_cmd", 1);
  steer_cmd_pub_ = nh_.advertise<rover_msgs::SteerCmd>("/rover/steer_cmd", 1);
  arm_cmd_pub_   = nh_.advertise<rover_msgs::ArmCmd>("/arm/joint_cmd_raw", 1);
}

void RoverHW::wheelStateCb(const sensor_msgs::JointState& js){
  last_wheel_js_ = js; got_wheels_=true;
}
void RoverHW::armStateCb(const sensor_msgs::JointState& js){
  last_arm_js_ = js; got_arm_=true;
}

void RoverHW::read(const ros::Duration&){
  if(got_wheels_){
    for(int i=0;i<NW;i++){
      auto& name = wheel_names_[i];
      auto it = std::find(last_wheel_js_.name.begin(), last_wheel_js_.name.end(), name);
      if(it != last_wheel_js_.name.end()){
        int idx = it - last_wheel_js_.name.begin();
        if(idx < (int)last_wheel_js_.position.size()) wheel_pos_[i] = last_wheel_js_.position[idx];
        if(idx < (int)last_wheel_js_.velocity.size()) wheel_vel_[i] = last_wheel_js_.velocity[idx];
      }
    }
  }
  if(got_arm_){
    for(int i=0;i<NA;i++){
      auto& name = arm_names_[i];
      auto it = std::find(last_arm_js_.name.begin(), last_arm_js_.name.end(), name);
      if(it != last_arm_js_.name.end()){
        int idx = it - last_arm_js_.name.begin();
        if(idx < (int)last_arm_js_.position.size()) arm_pos_[i] = last_arm_js_.position[idx];
        if(idx < (int)last_arm_js_.velocity.size()) arm_vel_[i] = last_arm_js_.velocity[idx];
      }
    }
  }
  for(int i=0;i<NS;i++) steer_pos_[i] = steer_cmd_[i];
}

void RoverHW::write(const ros::Duration&){
  rover_msgs::WheelCmd wc; wc.wheel_rpm.resize(NW);
  rover_msgs::SteerCmd sc; sc.steer_deg.resize(NS);
  rover_msgs::ArmCmd ac;   ac.joint_deg.resize(NA); ac.speed_scale=1.0;

  for(int i=0;i<NW;i++){
    double rpm = wheel_cmd_[i] * 60.0/(2.0*M_PI);
    wc.wheel_rpm[i] = (float)rpm;
  }
  for(int i=0;i<NS;i++){
    double deg = steer_cmd_[i] * 180.0/M_PI;
    sc.steer_deg[i] = (float)deg;
  }
  for(int i=0;i<NA;i++){
    double deg = arm_cmd_[i] * 180.0/M_PI;
    ac.joint_deg[i] = (float)deg;
  }

  wheel_cmd_pub_.publish(wc);
  steer_cmd_pub_.publish(sc);
  arm_cmd_pub_.publish(ac);
}
