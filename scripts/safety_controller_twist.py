#!/usr/bin/env python
"""
safety_controller_twist.py
--------------------------
Real-time LIDAR obstacle guard — the final gate before /cmd_vel.

Operates OUTSIDE the delay chain intentionally: the safety check always
sees the world as it is RIGHT NOW, regardless of command latency.

If an obstacle is detected within `stop_distance` metres in the forward
arc, any forward linear velocity is zeroed while turning commands are
still passed through (so the operator can steer away).

Publishes /safety_status (std_msgs/Bool):
  True  = safety stop is ACTIVE (obstacle detected)
  False = path is clear

Parameters  — none (all hard-coded; tune stop_distance here if needed)

Subscribes : /cmd_vel_teleop  (geometry_msgs/Twist)
             /base_scan        (sensor_msgs/LaserScan)
Publishes  : /cmd_vel         (geometry_msgs/Twist)  — final robot command
             /safety_status   (std_msgs/Bool)
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class SafetyControllerTwist:
    def __init__(self):
        rospy.init_node('fetch_safety_twist', anonymous=False)

        self.stop_distance       = 0.3   # metres
        self.safe_to_move_forward = True

        self.cmd_sub    = rospy.Subscriber('/cmd_vel_teleop', Twist,     self.cmd_callback)
        self.scan_sub   = rospy.Subscriber('/base_scan',      LaserScan, self.scan_callback)
        self.cmd_pub    = rospy.Publisher('/cmd_vel',          Twist,     queue_size=10)
        self.status_pub = rospy.Publisher('/safety_status',    Bool,      queue_size=1)

        rospy.loginfo("Safety Controller ready | stop_distance=%.2fm", self.stop_distance)

    def scan_callback(self, scan_msg):
        ranges = np.array(scan_msg.ranges)

        # Sanitise sensor noise
        ranges[np.isnan(ranges)]  = 10.0
        ranges[np.isinf(ranges)]  = 10.0
        ranges[ranges < 0.05]     = 10.0

        # Forward arc: centre ±1/6 of the full scan
        mid   = len(ranges) // 2
        width = len(ranges) // 6
        front = ranges[mid - width : mid + width]

        self.safe_to_move_forward = np.min(front) >= self.stop_distance

        # True = safety stop active (UNSAFE)
        self.status_pub.publish(not self.safe_to_move_forward)

    def cmd_callback(self, twist_msg):
        safe_cmd = Twist()

        if twist_msg.linear.x > 0 and not self.safe_to_move_forward:
            safe_cmd.linear.x  = 0.0
            safe_cmd.angular.z = twist_msg.angular.z  # still allow turning
            rospy.logwarn_throttle(1, "Safety: blocking forward motion — obstacle within %.2fm",
                                   self.stop_distance)
        else:
            safe_cmd = twist_msg

        self.cmd_pub.publish(safe_cmd)


if __name__ == '__main__':
    try:
        SafetyControllerTwist()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
