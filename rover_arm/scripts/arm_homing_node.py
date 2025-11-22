#!/usr/bin/env python3
import rospy, math, threading
from sensor_msgs.msg import JointState
from rover_msgs.msg import ArmCmd, ArmLimit
from std_srvs.srv import Trigger, TriggerResponse

JOINT_NAMES = [
    "joint1_link_joint","joint2_link_joint","joint3_link_joint",
    "joint4_link_joint","joint5_link_joint",
]
LIMITS = [
    (-math.pi, math.pi),
    (-1.57, 1.57),
    (-1.57, 1.57),
    (-2.2, 2.2),
    (-2.2, 2.2),
]

class ArmHomingSafety:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/arm/joint_cmd", ArmCmd, queue_size=1)
        self.js = None
        self.limits_hit = [False]*5
        self.lock = threading.Lock()

        rospy.Subscriber("/arm/joint_states", JointState, self.js_cb)
        rospy.Subscriber("/arm/limits", ArmLimit, self.lim_cb)
        self.home_srv = rospy.Service("/arm/homing", Trigger, self.home_cb)
        rospy.Subscriber("/arm/joint_cmd_raw", ArmCmd, self.raw_cmd_cb)

    def js_cb(self, msg):
        with self.lock:
            self.js = msg

    def lim_cb(self, msg):
        if len(msg.hit) >= 5:
            with self.lock:
                self.limits_hit = list(msg.hit[:5])

    def clamp_cmd(self, degs):
        out=[]
        for i,d in enumerate(degs):
            rad=d*math.pi/180.0
            lo,hi=LIMITS[i]
            rad=max(lo,min(hi,rad))
            out.append(rad*180.0/math.pi)
        return out

    def raw_cmd_cb(self, msg):
        if len(msg.joint_deg)<5: return
        safe=ArmCmd()
        safe.joint_deg=self.clamp_cmd(msg.joint_deg[:5])
        safe.speed_scale=max(0.05,min(1.0,msg.speed_scale))
        self.cmd_pub.publish(safe)

    def home_cb(self,_req):
        rospy.loginfo("Homing started")
        rate=rospy.Rate(20)
        targets=[0]*5
        speed=0.2
        while not rospy.is_shutdown():
            with self.lock:
                hit=self.limits_hit[:]
                js=self.js
            if all(hit): break
            if js is not None and (not any(hit)):
                for i in range(5):
                    try:
                        idx=js.name.index(JOINT_NAMES[i])
                        if abs(js.velocity[idx])<1e-3 and js.position[idx]<(LIMITS[i][0]*0.9):
                            hit[i]=True
                    except ValueError:
                        pass
            for i in range(5):
                targets[i]=0.0 if hit[i] else LIMITS[i][0]*180.0/math.pi
            cmd=ArmCmd()
            cmd.joint_deg=targets
            cmd.speed_scale=speed
            self.cmd_pub.publish(cmd)
            rate.sleep()
        zero=ArmCmd(); zero.joint_deg=[0,0,0,0,0]; zero.speed_scale=0.3
        self.cmd_pub.publish(zero)
        rospy.loginfo("Homing done")
        return TriggerResponse(success=True, message="Homed all joints")

if __name__=="__main__":
    rospy.init_node("arm_homing_safety")
    ArmHomingSafety()
    rospy.spin()
