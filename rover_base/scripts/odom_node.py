#!/usr/bin/env python3
import rospy, math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros

class OdomNode:
    def __init__(self):
        self.Rw = rospy.get_param("~wheel_radius", 0.12)
        self.Tw = rospy.get_param("~track_width", 0.55)
        self.Wb = rospy.get_param("~wheelbase", 0.75)

        self.x = self.y = self.yaw = 0.0
        self.last_time = rospy.Time.now()
        self.last_pos = [0.0]*6

        self.odom_pub = rospy.Publisher("/rover/odom", Odometry, queue_size=10)
        self.tf_pub = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/rover/wheel_states", JointState, self.cb)

    def cb(self, js: JointState):
        if len(js.position) < 6:
            return
        now = js.header.stamp if js.header.stamp else rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        if dt <= 0:
            return

        dtheta = [js.position[i] - self.last_pos[i] for i in range(6)]
        self.last_pos = list(js.position[:6])
        self.last_time = now

        left_idx = [0,2,4]
        right_idx = [1,3,5]
        dl = sum(dtheta[i] for i in left_idx)/3.0 * self.Rw
        dr = sum(dtheta[i] for i in right_idx)/3.0 * self.Rw

        dc = (dl + dr)/2.0
        dyaw = (dr - dl)/self.Tw

        self.yaw += dyaw
        self.x += dc * math.cos(self.yaw)
        self.y += dc * math.sin(self.yaw)

        q = Quaternion(0.0, 0.0, math.sin(self.yaw/2.0), math.cos(self.yaw/2.0))

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = dc/dt
        odom.twist.twist.angular.z = dyaw/dt
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_pub.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("rover_odom")
    OdomNode()
    rospy.spin()
