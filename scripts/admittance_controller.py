#!/usr/bin/env python
"""
admittance_controller.py
------------------------
Applies a virtual mass-damper model to smooth delayed joystick commands.

High network latency causes operators to over-correct and produce jerky
velocity profiles.  This node treats the incoming velocity target as an
applied "force" and integrates a second-order mass-damper equation:

    acceleration = (force - damping * velocity) / mass
    velocity    += acceleration * dt

A watchdog timer zeros the target if no command is received within
`cmd_timeout` seconds — prevents runaway motion if the upstream node dies.

Parameters
  ~virtual_mass     (float, default 0.5)   Inertia feel (higher = slower accel)
  ~virtual_damping  (float, default 1.0)   Resistance (higher = stops faster)
  ~max_linear_vel   (float, default 1.0)   m/s saturation limit
  ~max_angular_vel  (float, default 1.5)   rad/s saturation limit

Subscribes : /cmd_vel_delayed  (geometry_msgs/Twist)
Publishes  : /cmd_vel_teleop   (geometry_msgs/Twist)
"""

import rospy
import numpy as np
import threading
from geometry_msgs.msg import Twist


class AdmittanceController:
    def __init__(self):
        rospy.init_node('admittance_controller')

        # Physics parameters
        self.VIRTUAL_MASS    = rospy.get_param('~virtual_mass',    0.5)
        self.VIRTUAL_DAMPING = rospy.get_param('~virtual_damping', 1.0)
        self.MAX_LINEAR_VEL  = rospy.get_param('~max_linear_vel',  1.0)
        self.MAX_ANGULAR_VEL = rospy.get_param('~max_angular_vel', 1.5)

        # Watchdog: zero target if no command received within this window
        self.cmd_timeout   = 0.5   # seconds
        self.last_cmd_time = rospy.Time.now()

        # State
        self.target_vel            = Twist()
        self.commanded_linear_vel  = 0.0
        self.commanded_angular_vel = 0.0
        self.lock                  = threading.Lock()
        self.last_loop_time        = None

        self.sub  = rospy.Subscriber('/cmd_vel_delayed', Twist, self.cmd_callback)
        self.pub  = rospy.Publisher('/cmd_vel_teleop',   Twist, queue_size=1)
        self.rate = rospy.Rate(50)

        rospy.loginfo("Admittance Controller ready | mass=%.2f | damping=%.2f",
                      self.VIRTUAL_MASS, self.VIRTUAL_DAMPING)

    def cmd_callback(self, msg):
        with self.lock:
            self.last_cmd_time = rospy.Time.now()
            self.target_vel    = msg

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                target     = self.target_vel
                last_cmd_t = self.last_cmd_time

            # Watchdog — force stop if upstream signal is stale
            if (rospy.Time.now() - last_cmd_t).to_sec() > self.cmd_timeout:
                target.linear.x  = 0.0
                target.angular.z = 0.0

            # Time delta
            now = rospy.Time.now()
            if self.last_loop_time is None:
                self.last_loop_time = now
                self.rate.sleep()
                continue

            dt = (now - self.last_loop_time).to_sec()
            self.last_loop_time = now
            if dt <= 0:
                continue

            # Mass-damper integration
            accel_lin = (target.linear.x  - self.VIRTUAL_DAMPING * self.commanded_linear_vel)  / self.VIRTUAL_MASS
            accel_ang = (target.angular.z - self.VIRTUAL_DAMPING * self.commanded_angular_vel) / self.VIRTUAL_MASS

            self.commanded_linear_vel  += accel_lin * dt
            self.commanded_angular_vel += accel_ang * dt

            # Saturation
            self.commanded_linear_vel  = np.clip(self.commanded_linear_vel,
                                                 -self.MAX_LINEAR_VEL,  self.MAX_LINEAR_VEL)
            self.commanded_angular_vel = np.clip(self.commanded_angular_vel,
                                                 -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)

            # Snap to zero to prevent floating-point drift when inputs are off
            if abs(target.linear.x)  < 1e-3 and abs(self.commanded_linear_vel)  < 1e-3:
                self.commanded_linear_vel  = 0.0
            if abs(target.angular.z) < 1e-3 and abs(self.commanded_angular_vel) < 1e-3:
                self.commanded_angular_vel = 0.0

            out = Twist()
            out.linear.x  = self.commanded_linear_vel
            out.angular.z = self.commanded_angular_vel
            self.pub.publish(out)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        controller = AdmittanceController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
