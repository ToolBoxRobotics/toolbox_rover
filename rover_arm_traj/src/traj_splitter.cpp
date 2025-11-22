\
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <rover_msgs/ArmCmd.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <map>

static const std::vector<std::string> ARM_JOINTS = {
  "joint1_link_joint","joint2_link_joint","joint3_link_joint",
  "joint4_link_joint","joint5_link_joint"
};

class TrajSplitter {
public:
  TrajSplitter(ros::NodeHandle& nh)
  : nh_(nh),
    as_(nh_, "follow_joint_trajectory",
        boost::bind(&TrajSplitter::executeCb, this, _1), false)
  {
    cmd_pub_ = nh_.advertise<rover_msgs::ArmCmd>("/arm/joint_cmd_raw", 1);
    js_sub_ = nh_.subscribe("/arm/joint_states", 1, &TrajSplitter::jsCb, this);
    max_vel_ = nh_.param("max_vel_deg_s", 60.0);
    as_.start();
  }

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  ros::Publisher cmd_pub_;
  ros::Subscriber js_sub_;
  sensor_msgs::JointState last_js_;
  bool got_js_{false};
  double max_vel_;

  void jsCb(const sensor_msgs::JointState& js){ last_js_=js; got_js_=true; }

  void executeCb(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal){
    control_msgs::FollowJointTrajectoryResult result;

    if(goal->trajectory.points.empty()){
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      as_.setAborted(result, "Empty trajectory");
      return;
    }

    std::map<std::string,int> idx;
    for(size_t i=0;i<goal->trajectory.joint_names.size();i++)
      idx[goal->trajectory.joint_names[i]] = (int)i;

    for(auto& j: ARM_JOINTS){
      if(idx.find(j)==idx.end()){
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
        as_.setAborted(result, "Trajectory missing joint: "+j);
        return;
      }
    }

    ros::Time t0 = ros::Time::now();
    for(size_t p=0; p<goal->trajectory.points.size(); p++){
      if(as_.isPreemptRequested()){
        result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
        as_.setPreempted(result, "Preempted");
        return;
      }

      auto& pt = goal->trajectory.points[p];
      ros::Time target_time = t0 + pt.time_from_start;
      ros::Rate spin(200);
      while(ros::ok() && ros::Time::now() < target_time){
        ros::spinOnce();
        spin.sleep();
      }

      rover_msgs::ArmCmd cmd;
      cmd.joint_deg.resize(5);

      for(int i=0;i<5;i++){
        double rad = pt.positions[idx[ARM_JOINTS[i]]];
        cmd.joint_deg[i] = (float)(rad * 180.0/M_PI);
      }

      double scale = 1.0;
      if(!pt.velocities.empty()){
        for(int i=0;i<5;i++){
          double vel_deg_s = std::abs(pt.velocities[idx[ARM_JOINTS[i]]]) * 180.0/M_PI;
          if(vel_deg_s > max_vel_) scale = std::min(scale, max_vel_/vel_deg_s);
        }
      }
      cmd.speed_scale = (float)scale;
      cmd_pub_.publish(cmd);
    }

    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    as_.setSucceeded(result, "Trajectory executed");
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "traj_splitter");
  ros::NodeHandle nh("~");
  TrajSplitter ts(nh);
  ros::spin();
  return 0;
}
