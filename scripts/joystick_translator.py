#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
joystick_translator.py
----------------------
Reads raw joystick input from the ONN/Xbox controller and publishes
Twist commands to /cmd_vel_raw.

CONTROL SCHEMES
---------------
arcade
  Right Stick Vertical   (Axis 4) = Forward / Backward
  Left  Stick Horizontal (Axis 0) = Turn left / right
  Two independent axes -- natural car-style driving.

tank
  Right Stick Vertical   (Axis 4) = Forward / Backward
  Right Stick Horizontal (Axis 3) = Turn left / right  (same stick, both axes)
  Single stick controls all motion.

DEADMAN BUTTON
--------------
LB = Button 4 must be held for any motion in BOTH modes.
Robot stops immediately when released.

AXIS SIGN CONVENTION
--------------------
On the ONN/Xbox controller the vertical axes report +1.0 when the stick
is pushed FORWARD (away from you) and -1.0 when pulled back.
We use the raw value directly so forward stick = positive linear.x
(standard ROS convention). No negation needed.

Subscribes : /my_joy      (sensor_msgs/Joy)
Publishes  : /cmd_vel_raw (geometry_msgs/Twist)

Parameters
  ~control_scheme   (string, default 'arcade')   'arcade' or 'tank'
  ~scale_linear     (float,  default 0.5)         m/s at full stick deflection
  ~scale_angular    (float,  default 1.0)         rad/s at full stick deflection
  ~deadman_button   (int,    default 4)            Button index (LB)
  ~axis_linear      (int,    default 4)            Right Stick Vertical (both modes)
  ~axis_angular     (int,    default 0)            Left  Stick Horizontal (arcade)
  ~axis_turn_tank   (int,    default 3)            Right Stick Horizontal (tank)
"""

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoystickTranslator:
    def __init__(self):
        rospy.init_node('joystick_translator')

        # --- Parameters ---
        self.scale_linear   = rospy.get_param('~scale_linear',   0.5)
        self.scale_angular  = rospy.get_param('~scale_angular',  1.0)
        self.control_scheme = rospy.get_param('~control_scheme', 'arcade').lower().strip()
        self.deadman_button = rospy.get_param('~deadman_button', 4)

        # Axis indices
        # arcade: Right Stick Vertical  (Axis 4) = fwd/back
        #         Left  Stick Horizontal (Axis 0) = turn
        # tank:   Right Stick Vertical  (Axis 4) = fwd/back
        #         Right Stick Horizontal (Axis 3) = turn
        self.axis_linear    = rospy.get_param('~axis_linear',    4)  # fwd/back, both modes
        self.axis_angular   = rospy.get_param('~axis_angular',   0)  # turn, arcade mode
        self.axis_turn_tank = rospy.get_param('~axis_turn_tank', 3)  # turn, tank mode

        if self.control_scheme not in ('arcade', 'tank'):
            rospy.logwarn("Unknown control_scheme '%s', defaulting to 'arcade'",
                          self.control_scheme)
            self.control_scheme = 'arcade'

        self.pub = rospy.Publisher('/cmd_vel_raw', Twist, queue_size=1)
        rospy.Subscriber('/my_joy', Joy, self.callback)

        rospy.loginfo("Joystick Translator ready | scheme=%s | deadman=button%d",
                      self.control_scheme.upper(), self.deadman_button)

        if self.control_scheme == 'arcade':
            rospy.loginfo("ARCADE: fwd/back=Axis%d (Right Stick V) | turn=Axis%d (Left Stick H)",
                          self.axis_linear, self.axis_angular)
        else:
            rospy.loginfo("TANK: fwd/back=Axis%d (Right Stick V) | turn=Axis%d (Right Stick H)",
                          self.axis_linear, self.axis_turn_tank)

    def callback(self, data):
        twist = Twist()  # defaults all zeros -- robot stops if deadman not held

        if data.buttons[self.deadman_button] == 1:
            if self.control_scheme == 'arcade':
                twist.linear.x  = data.axes[self.axis_linear]   * self.scale_linear
                twist.angular.z = data.axes[self.axis_angular]  * self.scale_angular
            else:  # tank
                twist.linear.x  = data.axes[self.axis_linear]    * self.scale_linear
                twist.angular.z = data.axes[self.axis_turn_tank]  * self.scale_angular

        self.pub.publish(twist)


if __name__ == '__main__':
    try:
        JoystickTranslator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass