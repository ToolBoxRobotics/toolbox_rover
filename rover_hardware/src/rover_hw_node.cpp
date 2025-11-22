#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "rover_hardware/rover_hw.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "rover_hw_node");
  ros::NodeHandle nh;

  rover_hardware::RoverHW hw(nh);
  controller_manager::ControllerManager cm(&hw, nh);

  ros::Rate rate(50);
  ros::Time last = ros::Time::now();

  while(ros::ok()){
    ros::Time now = ros::Time::now();
    ros::Duration dt = now - last;
    last = now;

    hw.read(dt);
    cm.update(now, dt);
    hw.write(dt);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
