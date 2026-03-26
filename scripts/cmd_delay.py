#!/usr/bin/env python
"""
cmd_delay.py
------------
Simulates one-way network latency on the command stream.

Messages arriving on /cmd_vel_raw are held in a timestamped deque
and only released after `delay` seconds have elapsed.  Set `delay`
to the same value used for the camera system so both the visual
feedback and the control commands share the same one-way lag.

Parameters
  ~delay  (float, default 0.0)  One-way delay in seconds.

Subscribes : /cmd_vel_raw     (geometry_msgs/Twist)
Publishes  : /cmd_vel_delayed (geometry_msgs/Twist)
"""

import rospy
from collections import deque
from geometry_msgs.msg import Twist


class CmdDelay:
    def __init__(self):
        rospy.init_node('cmd_delay_node')

        self.delay  = rospy.get_param('~delay', 0.0)
        self.buffer = deque()

        self.sub = rospy.Subscriber('/cmd_vel_raw',     Twist, self.cmd_callback)
        self.pub = rospy.Publisher('/cmd_vel_delayed',  Twist, queue_size=10)

        # Process buffer at 50 Hz — matches admittance controller rate
        rospy.Timer(rospy.Duration(0.02), self.process_buffer)

        rospy.loginfo("CMD Delay node ready | one-way delay=%.3fs", self.delay)

    def cmd_callback(self, msg):
        self.buffer.append((rospy.Time.now(), msg))

    def process_buffer(self, event):
        if not self.buffer:
            return

        now = rospy.Time.now()
        while self.buffer:
            timestamp, msg = self.buffer[0]
            if (now - timestamp).to_sec() >= self.delay:
                self.buffer.popleft()
                self.pub.publish(msg)
            else:
                break   # remaining messages are newer — stop here


if __name__ == '__main__':
    CmdDelay()
    rospy.spin()
