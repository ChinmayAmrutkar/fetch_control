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
  Left  Stick Horizontal (Axis 3) = Turn left / right
  Two independent axes -- natural car-style driving.

tank
  Right Stick Vertical   (Axis 4) = Forward / Backward
  Right Stick Horizontal (Axis 3) = Turn left / right  (same stick, both axes)
  Single stick controls all motion -- push forward to go forward,
  push left/right to turn, push diagonally to do both simultaneously.

DEADMAN BUTTON
--------------
LB = Button 4 must be held for any motion in BOTH modes.
Robot stops immediately when released.

AXIS SIGN CONVENTION
--------------------
On the ONN/Xbox controller, pushing the stick FORWARD (away from you)
gives a NEGATIVE axis value on ROS Joy. We negate it so forward = positive
linear velocity (standard ROS convention).

Subscribes : /my_joy      (sensor_msgs/Joy)
Publishes  : /cmd_vel_raw (geometry_msgs/Twist)

Parameters
  ~control_scheme   (string, default 'arcade')  'arcade' or 'tank'
  ~scale_linear     (float,  default 0.5)        m/s at full stick deflection
  ~scale_angular    (float,  default 1.0)        rad/s at full stick deflection
  ~deadman_button   (int,    default 4)           Button index (LB)
  ~axis_fwd_back    (int,    default 4)           Axis for forward/backward (both modes)
  ~axis_turn_arcade (int,    default 3)           Axis for turning (arcade mode, left stick X)
  ~axis_turn_tank   (int,    default 3)           Axis for turning (tank mode, right stick X)
"""

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoystickTranslator:
    def __init__(self):
        rospy.init_node('joystick_translator')

        # --- Parameters ---
        self.scale_linear    = rospy.get_param('~scale_linear',    0.5)
        self.scale_angular   = rospy.get_param('~scale_angular',   1.0)
        self.control_scheme  = rospy.get_param('~control_scheme',  'arcade').lower().strip()
        self.deadman_button  = rospy.get_param('~deadman_button',  4)

        # Axis indices
        # Both modes use Axis 4 (Right Stick Vertical) for forward/backward.
        # Arcade: turning is Left Stick Horizontal (Axis 3) -- separate stick.
        # Tank:   turning is Right Stick Horizontal (Axis 3) -- same stick as fwd/back.
        self.axis_fwd_back    = rospy.get_param('~axis_fwd_back',    4)
        self.axis_turn_arcade = rospy.get_param('~axis_turn_arcade', 3)
        self.axis_turn_tank   = rospy.get_param('~axis_turn_tank',   3)

        if self.control_scheme not in ('arcade', 'tank'):
            rospy.logwarn("Unknown control_scheme '%s', defaulting to 'arcade'",
                          self.control_scheme)
            self.control_scheme = 'arcade'

        self.pub = rospy.Publisher('/cmd_vel_raw', Twist, queue_size=1)
        rospy.Subscriber('/my_joy', Joy, self.joy_callback)

        rospy.loginfo(
            "Joystick Translator ready | scheme=%s | deadman=button%d | "
            "fwd_back=axis%d | turn=axis%d | scale_lin=%.2f | scale_ang=%.2f",
            self.control_scheme.upper(), self.deadman_button,
            self.axis_fwd_back,
            self.axis_turn_arcade if self.control_scheme == 'arcade' else self.axis_turn_tank,
            self.scale_linear, self.scale_angular
        )

        if self.control_scheme == 'arcade':
            rospy.loginfo("ARCADE: Right Stick (Axis%d) = fwd/back  |  "
                          "Left Stick (Axis%d) = turn",
                          self.axis_fwd_back, self.axis_turn_arcade)
        else:
            rospy.loginfo("TANK: Right Stick Vertical (Axis%d) = fwd/back  |  "
                          "Right Stick Horizontal (Axis%d) = turn",
                          self.axis_fwd_back, self.axis_turn_tank)

    def joy_callback(self, data):
        cmd = Twist()  # defaults to all zeros -- robot stops if deadman not held

        if data.buttons[self.deadman_button] == 1:

            if self.control_scheme == 'arcade':
                # Right Stick Vertical: negate because forward stick = negative axis value
                cmd.linear.x  = -data.axes[self.axis_fwd_back]    * self.scale_linear
                # Left Stick Horizontal: negate so stick-left = turn left (positive angular.z)
                cmd.angular.z = -data.axes[self.axis_turn_arcade]  * self.scale_angular

            else:  # tank
                # Right Stick Vertical: negate for same reason as above
                cmd.linear.x  = -data.axes[self.axis_fwd_back]  * self.scale_linear
                # Right Stick Horizontal: negate so stick-left = positive angular.z (turn left)
                cmd.angular.z = -data.axes[self.axis_turn_tank]  * self.scale_angular

        # If deadman not held, cmd stays at zero -- immediate stop
        self.pub.publish(cmd)


if __name__ == '__main__':
    try:
        JoystickTranslator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass