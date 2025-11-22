#!/usr/bin/env python3
import rospy, math
from rover_msgs.msg import WheelCmd, SteerCmd
from std_msgs.msg import Float64

class SimBridge:
    def __init__(self):
        self.wheel_pubs = [
            rospy.Publisher("/wheel_fl_wheel_controller/command", Float64, queue_size=1),
            rospy.Publisher("/wheel_fr_wheel_controller/command", Float64, queue_size=1),
            rospy.Publisher("/wheel_ml_wheel_controller/command", Float64, queue_size=1),
            rospy.Publisher("/wheel_mr_wheel_controller/command", Float64, queue_size=1),
            rospy.Publisher("/wheel_rl_wheel_controller/command", Float64, queue_size=1),
            rospy.Publisher("/wheel_rr_wheel_controller/command", Float64, queue_size=1),
        ]
        self.steer_pubs = [
            rospy.Publisher("/wheel_fl_steer_controller/command", Float64, queue_size=1),
            rospy.Publisher("/wheel_fr_steer_controller/command", Float64, queue_size=1),
            rospy.Publisher("/wheel_rl_steer_controller/command", Float64, queue_size=1),
            rospy.Publisher("/wheel_rr_steer_controller/command", Float64, queue_size=1),
        ]
        rospy.Subscriber("/rover/wheel_cmd", WheelCmd, self.wheel_cb)
        rospy.Subscriber("/rover/steer_cmd", SteerCmd, self.steer_cb)

    def wheel_cb(self, msg):
        for i, rpm in enumerate(msg.wheel_rpm[:6]):
            rad_s = rpm * 2.0*math.pi / 60.0
            self.wheel_pubs[i].publish(Float64(rad_s))

    def steer_cb(self, msg):
        for i, deg in enumerate(msg.steer_deg[:4]):
            rad = deg * math.pi/180.0
            self.steer_pubs[i].publish(Float64(rad))

if __name__ == "__main__":
    rospy.init_node("sim_bridge")
    SimBridge()
    rospy.spin()
