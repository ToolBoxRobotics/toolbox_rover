\
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <rover_msgs/WheelCmd.h>
#include <rover_msgs/SteerCmd.h>
#include <cmath>
#include <algorithm>

class RoverController {
public:
  RoverController(ros::NodeHandle& nh){
    nh.param("wheel_radius", Rw_, 0.12);
    nh.param("track_width", Tw_, 0.55);
    nh.param("wheelbase", Wb_, 0.75);
    nh.param("max_wheel_rpm", max_rpm_, 120.0);
    nh.param("max_steer_deg", max_steer_deg_, 35.0);

    cmd_sub_ = nh.subscribe("/cmd_vel", 10, &RoverController::cmdCb, this);
    wheel_pub_ = nh.advertise<rover_msgs::WheelCmd>("/rover/wheel_cmd", 10);
    steer_pub_ = nh.advertise<rover_msgs::SteerCmd>("/rover/steer_cmd", 10);
  }

private:
  ros::Subscriber cmd_sub_;
  ros::Publisher wheel_pub_, steer_pub_;
  double Rw_, Tw_, Wb_, max_rpm_, max_steer_deg_;

  void cmdCb(const geometry_msgs::Twist& cmd){
    double v = cmd.linear.x;
    double w = cmd.angular.z;

    rover_msgs::WheelCmd wc;
    rover_msgs::SteerCmd sc;
    wc.wheel_rpm.resize(6);
    sc.steer_deg.resize(4);

    const double eps = 1e-4;
    if (std::abs(w) < eps || std::abs(v) < eps){
      for(int i=0;i<4;i++) sc.steer_deg[i] = 0.0;
      double rpm = (v / (2*M_PI*Rw_)) * 60.0;
      rpm = std::clamp(rpm, -max_rpm_, max_rpm_);
      for(int i=0;i<6;i++) wc.wheel_rpm[i] = rpm;
      wheel_pub_.publish(wc);
      steer_pub_.publish(sc);
      return;
    }

    double R = v / w;

    double Lf = Wb_/2.0;
    double Lr = Wb_/2.0;
    double halfT = Tw_/2.0;

    auto steerRad = [&](double L, double side){
      return std::atan2(L, (R - side));
    };

    double fl = steerRad(Lf, +halfT);
    double fr = steerRad(Lf, -halfT);
    double rl = steerRad(-Lr, +halfT);
    double rr = steerRad(-Lr, -halfT);

    sc.steer_deg[0] = std::clamp(fl * 180.0/M_PI, -max_steer_deg_, max_steer_deg_);
    sc.steer_deg[1] = std::clamp(fr * 180.0/M_PI, -max_steer_deg_, max_steer_deg_);
    sc.steer_deg[2] = std::clamp(rl * 180.0/M_PI, -max_steer_deg_, max_steer_deg_);
    sc.steer_deg[3] = std::clamp(rr * 180.0/M_PI, -max_steer_deg_, max_steer_deg_);

    double x_pos[6] = { +Lf, +Lf, 0.0, 0.0, -Lr, -Lr };
    double y_pos[6] = { +halfT, -halfT, +halfT, -halfT, +halfT, -halfT };

    double max_abs_rpm = 0.0;
    for(int i=0;i<6;i++){
      double vx_i = v - w * y_pos[i];
      double vy_i = w * x_pos[i];
      double speed_i = std::sqrt(vx_i*vx_i + vy_i*vy_i);
      double rpm_i = (speed_i / (2*M_PI*Rw_)) * 60.0;
      rpm_i *= (vx_i >= 0 ? 1.0 : -1.0);
      wc.wheel_rpm[i] = rpm_i;
      max_abs_rpm = std::max(max_abs_rpm, std::abs(rpm_i));
    }

    if (max_abs_rpm > max_rpm_){
      double scale = max_rpm_ / max_abs_rpm;
      for(int i=0;i<6;i++) wc.wheel_rpm[i] *= scale;
    }

    wheel_pub_.publish(wc);
    steer_pub_.publish(sc);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "rover_controller");
  ros::NodeHandle nh("~");
  RoverController rc(nh);
  ros::spin();
  return 0;
}
