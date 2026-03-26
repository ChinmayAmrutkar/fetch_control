#!/usr/bin/env python
"""
experiment_logger.py
--------------------
Records all data streams needed for post-experiment analysis and writes
four synchronised CSV files per trial.

File naming:  {log_prefix}_{file_tag}_{timestamp}.csv
              e.g.  P01_TANK_MED_Delay0.25_A_mocap_2025-06-01-10-30-00.csv

Output files
  A_mocap       MoCap pose + raw & smoothed velocity + joystick input
  B_amcl        AMCL pose + odometry + joystick input
  C1_sync_event AMCL and MoCap together, triggered on every AMCL update
  C2_sync_fixed AMCL and MoCap together, sampled at fixed 20 Hz

Event column values written to every row:
  RUNNING              normal operation
  SAFETY_STOP          safety controller blocked a forward command
  SUCCESS_GOAL_REACHED supervisor confirmed goal reached
  MAX_TIME             trial clock expired

Parameters
  ~max_time         (float,  default 60.0)    Trial time limit in seconds
  ~rigid_body_name  (string, default 'Fetch') OptiTrack rigid body label
  ~log_prefix       (string, default 'TRIAL') Prepended to all filenames

Subscribes
  /cmd_vel_raw                      joystick intent (geometry_msgs/Twist)
  /odom                             wheel odometry  (nav_msgs/Odometry)
  /amcl_pose                        AMCL estimate   (geometry_msgs/PoseWithCovarianceStamped)
  /vrpn_client_node/<body>/pose     MoCap ground truth (geometry_msgs/PoseStamped)
  /safety_status                    obstacle stop flag (std_msgs/Bool)
  /goal_reached                     trial success flag (std_msgs/Bool)
"""

import rospy
import csv
import os
import datetime
import math
import numpy as np
from collections import deque
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion


class ExperimentLogger:
    def __init__(self):
        rospy.init_node('experiment_logger')

        # --- Parameters ---
        self.max_time        = rospy.get_param('~max_time',        60.0)
        self.rigid_body_name = rospy.get_param('~rigid_body_name', 'Fetch')
        self.log_prefix      = rospy.get_param('~log_prefix',      'TRIAL')

        # --- Log directory ---
        preferred = "/home/fetchuser/chinmay/fetch_teleop_ws/logs/"
        if os.path.exists(os.path.dirname(preferred)):
            self.log_dir = preferred
        else:
            self.log_dir = os.path.expanduser("~/fetch_control_ws/logs/")
        os.makedirs(self.log_dir, exist_ok=True)

        ts = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

        self.files = {
            'A_mocap':      os.path.join(self.log_dir, "{}_A_mocap_{}.csv".format(self.log_prefix, ts)),
            'B_amcl':       os.path.join(self.log_dir, "{}_B_amcl_{}.csv".format(self.log_prefix, ts)),
            'C1_sync_event':os.path.join(self.log_dir, "{}_C1_sync_event_{}.csv".format(self.log_prefix, ts)),
            'C2_sync_fixed':os.path.join(self.log_dir, "{}_C2_sync_fixed_{}.csv".format(self.log_prefix, ts)),
        }

        self.writers = {}
        self.handles = {}

        headers_common = ["Time_Sec", "Input_JX", "Input_JY", "Event_Msg"]

        self.init_csv('A_mocap', headers_common + [
            "MoCap_X", "MoCap_Y", "MoCap_Z", "MoCap_Theta",
            "Vel_Lin_Raw", "Vel_Ang_Raw",
            "Vel_Lin_Smooth", "Vel_Ang_Smooth",
        ])
        self.init_csv('B_amcl', headers_common + [
            "AMCL_X", "AMCL_Y", "AMCL_Z", "AMCL_Theta",
            "Odom_Vel_Lin", "Odom_Vel_Ang",
        ])
        headers_sync = headers_common + [
            "AMCL_X", "AMCL_Y", "AMCL_Theta",
            "MoCap_X", "MoCap_Y", "MoCap_Theta",
            "Odom_Vel_Lin", "Odom_Vel_Ang",
            "MoCap_Vel_Lin_Smooth", "MoCap_Vel_Ang_Smooth",
        ]
        self.init_csv('C1_sync_event', headers_sync)
        self.init_csv('C2_sync_fixed', headers_sync)

        # --- State ---
        self.start_time        = rospy.Time.now()
        self.experiment_active = True
        self.safety_triggered  = False
        self.goal_reached      = False

        self.current_input       = Twist()
        self.latest_odom_twist   = Twist()
        self.latest_amcl_pose    = None
        self.latest_mocap_pose   = None

        self.last_mocap_pos   = None
        self.last_mocap_time  = None
        self.last_mocap_theta = 0.0

        self.vel_lin_buffer = deque(maxlen=10)
        self.vel_ang_buffer = deque(maxlen=10)
        self.latest_mocap_vel_smooth = [0.0, 0.0]

        # --- Subscribers ---
        self.sub_input  = rospy.Subscriber('/cmd_vel_raw',   Twist,                        self.input_callback)
        self.sub_safety = rospy.Subscriber('/safety_status', Bool,                         self.safety_callback)
        self.sub_goal   = rospy.Subscriber('/goal_reached',  Bool,                         self.goal_callback)
        self.sub_odom   = rospy.Subscriber('/odom',          Odometry,                     self.odom_callback)
        self.sub_amcl   = rospy.Subscriber('/amcl_pose',     PoseWithCovarianceStamped,    self.amcl_callback)

        mocap_topic     = '/vrpn_client_node/{}/pose'.format(self.rigid_body_name)
        self.sub_mocap  = rospy.Subscriber(mocap_topic,      PoseStamped,                  self.mocap_callback)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.fixed_rate_loop)

        rospy.loginfo("Experiment Logger ready | prefix=%s | logs -> %s",
                      self.log_prefix, self.log_dir)

    # ------------------------------------------------------------------ helpers

    def init_csv(self, key, header):
        f = open(self.files[key], 'w')
        w = csv.writer(f)
        w.writerow(header)
        self.handles[key] = f
        self.writers[key] = w

    def get_time(self):
        return "{:.3f}".format((rospy.Time.now() - self.start_time).to_sec())

    def get_event(self):
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        if self.goal_reached:       return "SUCCESS_GOAL_REACHED"
        if self.safety_triggered:   return "SAFETY_STOP"
        if elapsed > self.max_time: return "MAX_TIME"
        return "RUNNING"

    def write_row(self, key, data):
        if not self.experiment_active:
            return
        if key in self.handles and not self.handles[key].closed:
            self.writers[key].writerow(data)

    def get_sync_row(self):
        return [
            self.get_time(),
            self.current_input.linear.x, self.current_input.angular.z,
            self.get_event(),
            self.latest_amcl_pose[0],  self.latest_amcl_pose[1],  self.latest_amcl_pose[3],
            self.latest_mocap_pose[0], self.latest_mocap_pose[1], self.latest_mocap_pose[3],
            self.latest_odom_twist.linear.x,  self.latest_odom_twist.angular.z,
            self.latest_mocap_vel_smooth[0],  self.latest_mocap_vel_smooth[1],
        ]

    # ------------------------------------------------------------------ callbacks

    def input_callback(self, msg):
        self.current_input = msg

    def safety_callback(self, msg):
        self.safety_triggered = msg.data

    def goal_callback(self, msg):
        self.goal_reached = msg.data
        if self.goal_reached and self.experiment_active:
            rospy.loginfo_once("LOGGER: Goal reached — marking logs.")

    def odom_callback(self, msg):
        self.latest_odom_twist = msg.twist.twist

    def amcl_callback(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.latest_amcl_pose = [p.x, p.y, p.z, theta]

        self.write_row('B_amcl', [
            self.get_time(),
            self.current_input.linear.x, self.current_input.angular.z,
            self.get_event(),
            p.x, p.y, p.z, theta,
            self.latest_odom_twist.linear.x, self.latest_odom_twist.angular.z,
        ])

        if self.latest_mocap_pose is not None:
            self.write_row('C1_sync_event', self.get_sync_row())

    def mocap_callback(self, msg):
        current_time = msg.header.stamp.to_sec()
        p = msg.pose.position
        o = msg.pose.orientation
        _, _, theta = euler_from_quaternion([o.x, o.y, o.z, o.w])

        vel_lin_raw = 0.0
        vel_ang_raw = 0.0

        if self.last_mocap_pos is not None:
            dt = current_time - self.last_mocap_time
            if dt > 0.0001:
                dist = math.sqrt((p.x - self.last_mocap_pos.x) ** 2 +
                                 (p.y - self.last_mocap_pos.y) ** 2)
                vel_lin_raw = dist / dt

                diff = theta - self.last_mocap_theta
                while diff >  math.pi: diff -= 2 * math.pi
                while diff < -math.pi: diff += 2 * math.pi
                vel_ang_raw = diff / dt

        self.last_mocap_pos   = p
        self.last_mocap_time  = current_time
        self.last_mocap_theta = theta
        self.latest_mocap_pose = [p.x, p.y, p.z, theta]

        self.vel_lin_buffer.append(vel_lin_raw)
        self.vel_ang_buffer.append(vel_ang_raw)
        vel_lin_smooth = sum(self.vel_lin_buffer) / len(self.vel_lin_buffer)
        vel_ang_smooth = sum(self.vel_ang_buffer) / len(self.vel_ang_buffer)
        self.latest_mocap_vel_smooth = [vel_lin_smooth, vel_ang_smooth]

        self.write_row('A_mocap', [
            self.get_time(),
            self.current_input.linear.x, self.current_input.angular.z,
            self.get_event(),
            p.x, p.y, p.z, theta,
            vel_lin_raw, vel_ang_raw,
            vel_lin_smooth, vel_ang_smooth,
        ])

    def fixed_rate_loop(self, event):
        if self.latest_amcl_pose is not None and self.latest_mocap_pose is not None:
            self.write_row('C2_sync_fixed', self.get_sync_row())

    # ------------------------------------------------------------------ shutdown

    def shutdown_hook(self):
        rospy.loginfo("Logger shutting down — flushing files...")
        self.experiment_active = False

        if hasattr(self, 'timer'):
            self.timer.shutdown()

        for sub in [self.sub_input, self.sub_safety, self.sub_goal,
                    self.sub_odom, self.sub_amcl, self.sub_mocap]:
            sub.unregister()

        for f in self.handles.values():
            if not f.closed:
                f.close()

        rospy.loginfo("Experiment logs saved to %s", self.log_dir)


if __name__ == '__main__':
    node = ExperimentLogger()
    rospy.on_shutdown(node.shutdown_hook)
    rospy.spin()
