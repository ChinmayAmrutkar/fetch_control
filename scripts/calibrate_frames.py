#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
calibrate_frames.py
-------------------
Computes the static transform (tf_x, tf_y, tf_yaw) that aligns the AMCL
map frame with the OptiTrack world frame.

Run this ONCE after saving a new map, before running any trials on that map.

Usage
-----
1.  roslaunch fetch_control localization.launch map_file:=/path/to/map.yaml
2.  In RViz, set a 2D Pose Estimate and watch the particle cloud.
3.  Wait until the particle cloud collapses to a tight cluster.
4.  Then run:

      rosrun fetch_control calibrate_frames.py _rigid_body_name:=Fetch8 _difficulty:=EASY

5.  This script will:
      - Wait for AMCL covariance to drop below the convergence threshold
        (so it never fires on a bad/spread-out particle cloud)
      - Collect 30 paired AMCL + MoCap samples (~3 seconds)
      - Average them to remove noise
      - Print the tf_x / tf_y / tf_yaw values
      - Automatically write them into run_trial.py MAP_DATABASE

HOW CONVERGENCE IS DETECTED
----------------------------
The /amcl_pose message carries a full 6x6 covariance matrix.
Elements [0,0] and [7,7] are the X and Y position variances (m^2).
When AMCL has not converged the particles are spread across the room ->
variance >> 0.1. Once the cloud collapses variance typically drops below
0.01 m^2 (= 10 cm std dev). We use 0.05 m^2 as a conservative threshold.

Parameters
  ~rigid_body_name  (string, default 'Fetch8')  OptiTrack rigid body label
  ~difficulty       (string, default '')         TRAIN/EASY/MED/HARD
                                                 If provided, values are
                                                 auto-written to MAP_DATABASE
  ~n_samples        (int,    default 30)         Samples to average
  ~cov_threshold    (float,  default 0.05)       Max acceptable XY variance
"""

import rospy
import math
import os
import re
import sys
import threading
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion


class FrameCalibrator:
    def __init__(self):
        rospy.init_node('frame_calibrator')

        self.rigid_body_name = rospy.get_param('~rigid_body_name', 'Fetch8')
        self.difficulty       = rospy.get_param('~difficulty',       '').upper().strip()
        self.n_samples        = rospy.get_param('~n_samples',        30)
        self.cov_threshold    = rospy.get_param('~cov_threshold',    0.05)

        self.mocap_topic = '/vrpn_client_node/{}/pose'.format(self.rigid_body_name)

        # Latest raw poses (updated by callbacks)
        self.amcl_pose      = None   # dict: x, y, z, theta
        self.amcl_cov_xy    = 999.0  # max(var_x, var_y) from covariance matrix
        self.mocap_pose     = None   # dict: x, y, z, theta
        self.amcl_converged = False

        # Sample buffers for averaging
        self.samples         = []    # list of (amcl_dict, mocap_dict) pairs
        self.collecting      = False
        self.done            = False

        self.lock = threading.Lock()

        rospy.Subscriber('/amcl_pose',     PoseWithCovarianceStamped, self.amcl_callback)
        rospy.Subscriber(self.mocap_topic, PoseStamped,               self.mocap_callback)

        self._print_banner()

    # ------------------------------------------------------------------ banner

    def _print_banner(self):
        print("\n" + "=" * 60)
        print("  FRAME CALIBRATOR")
        print("=" * 60)
        print("  Robot body   : {}".format(self.rigid_body_name))
        print("  Difficulty   : {}".format(self.difficulty if self.difficulty else "(not set - values will be printed only)"))
        print("  Samples      : {}".format(self.n_samples))
        print("  Cov threshold: {:.3f} m^2  (~{:.0f} cm std dev)".format(
              self.cov_threshold, math.sqrt(self.cov_threshold) * 100))
        print("=" * 60)
        print("\nStep 1: Set 2D Pose Estimate in RViz and wait for the")
        print("        particle cloud to collapse to a tight cluster.")
        print("\nThis script will proceed automatically once AMCL converges.\n")

    # ------------------------------------------------------------------ callbacks

    def amcl_callback(self, msg):
        p  = msg.pose.pose.position
        o  = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([o.x, o.y, o.z, o.w])

        # Covariance matrix is row-major 6x6.
        # Index [0]  = var_x,  index [7]  = var_y
        cov    = msg.pose.covariance
        var_x  = cov[0]
        var_y  = cov[7]
        max_var = max(var_x, var_y)

        with self.lock:
            self.amcl_pose   = {'x': p.x, 'y': p.y, 'z': p.z, 'theta': theta}
            self.amcl_cov_xy = max_var

            if not self.amcl_converged:
                converged = max_var < self.cov_threshold
                # Print live variance so operator can watch it drop
                sys.stdout.write("\r  AMCL variance: {:.4f} m^2  (threshold: {:.3f}) {}     ".format(
                    max_var,
                    self.cov_threshold,
                    "-> CONVERGED" if converged else "-> waiting..."
                ))
                sys.stdout.flush()
                if converged:
                    self.amcl_converged = True
                    print("\n\n  AMCL has converged. Collecting {} samples...".format(self.n_samples))
                    self.collecting = True
            elif self.collecting and not self.done:
                self._try_collect_sample()

    def mocap_callback(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        _, _, theta = euler_from_quaternion([o.x, o.y, o.z, o.w])
        with self.lock:
            self.mocap_pose = {'x': p.x, 'y': p.y, 'z': p.z, 'theta': theta}

    # ------------------------------------------------------------------ sampling

    def _try_collect_sample(self):
        """Called inside the AMCL callback (lock already held). Collect one paired sample."""
        if self.mocap_pose is None:
            return
        if self.done:
            return

        self.samples.append((dict(self.amcl_pose), dict(self.mocap_pose)))

        n = len(self.samples)
        sys.stdout.write("\r  Collecting samples: {}/{}     ".format(n, self.n_samples))
        sys.stdout.flush()

        if n >= self.n_samples:
            self.collecting = False
            self.done       = True

    # ------------------------------------------------------------------ main loop

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            with self.lock:
                done = self.done
            if done:
                self._compute_and_print()
                return
            rate.sleep()

    # ------------------------------------------------------------------ maths

    def _compute_and_print(self):
        print("\n\n  Computing transform from {} samples...".format(len(self.samples)))

        # Average each field independently
        def mean_field(samples, source, field):
            return sum(s[source][field] for s in samples) / float(len(samples))

        # Circular mean for angles (handles the -pi/pi wraparound correctly)
        def circular_mean(samples, source, field):
            sin_sum = sum(math.sin(s[source][field]) for s in samples)
            cos_sum = sum(math.cos(s[source][field]) for s in samples)
            return math.atan2(sin_sum, cos_sum)

        amcl_x     = mean_field(self.samples, 0, 'x')
        amcl_y     = mean_field(self.samples, 0, 'y')
        amcl_z     = mean_field(self.samples, 0, 'z')
        amcl_theta = circular_mean(self.samples, 0, 'theta')

        mocap_x     = mean_field(self.samples, 1, 'x')
        mocap_y     = mean_field(self.samples, 1, 'y')
        mocap_z     = mean_field(self.samples, 1, 'z')
        mocap_theta = circular_mean(self.samples, 1, 'theta')

        # ------------------------------------------------------------------
        # SE(2) transform: world = R(yaw_offset) * map + translation
        # ------------------------------------------------------------------
        yaw_offset = mocap_theta - amcl_theta

        # Rotate the AMCL position vector by yaw_offset, then find shift
        amcl_x_rot = amcl_x * math.cos(yaw_offset) - amcl_y * math.sin(yaw_offset)
        amcl_y_rot = amcl_x * math.sin(yaw_offset) + amcl_y * math.cos(yaw_offset)

        tf_x   = mocap_x - amcl_x_rot
        tf_y   = mocap_y - amcl_y_rot
        tf_z   = mocap_z - amcl_z
        tf_yaw = yaw_offset

        # ------------------------------------------------------------------
        # Print results
        # ------------------------------------------------------------------
        print("\n" + "=" * 60)
        print("  CALIBRATION RESULT  ({} samples averaged)".format(len(self.samples)))
        print("=" * 60)
        print("")
        print("  tf_x   : {:.4f}".format(tf_x))
        print("  tf_y   : {:.4f}".format(tf_y))
        print("  tf_yaw : {:.4f}  ({:.1f} deg)".format(tf_yaw, math.degrees(tf_yaw)))
        print("")
        print("  Static transform publisher args:")
        print("    {:.4f} {:.4f} {:.4f} {:.4f} 0 0  world map 100".format(
              tf_x, tf_y, tf_z, tf_yaw))

        # ------------------------------------------------------------------
        # Auto-write into run_trial.py MAP_DATABASE if difficulty was given
        # ------------------------------------------------------------------
        if self.difficulty in ("TRAIN", "EASY", "MED", "HARD"):
            self._patch_run_trial(tf_x, tf_y, tf_yaw)
        else:
            print("")
            print("  NOTE: No difficulty given (_difficulty:=EASY/MED/HARD/TRAIN).")
            print("  Paste the values above into MAP_DATABASE in run_trial.py manually.")

        print("=" * 60 + "\n")
        rospy.signal_shutdown("Calibration complete.")

    # ------------------------------------------------------------------ auto-patch

    def _patch_run_trial(self, tf_x, tf_y, tf_yaw):
        """
        Find run_trial.py on disk and update the MAP_DATABASE entry for
        self.difficulty with the new tf_x / tf_y / tf_yaw values.
        """
        # Search common locations
        candidates = [
            os.path.expanduser("~/fetch_control_ws/src/fetch_control/scripts/run_trial.py"),
            os.path.expanduser("~/chinmay/fetch_control_ws/src/fetch_control/scripts/run_trial.py"),
            os.path.expanduser("~/fetch_teleop_ws/src/fetch_control/scripts/run_trial.py"),
        ]
        # Also search via ROS package path if available
        try:
            import rospkg
            rp = rospkg.RosPack()
            pkg_path = rp.get_path('fetch_control')
            candidates.insert(0, os.path.join(pkg_path, 'scripts', 'run_trial.py'))
        except Exception:
            pass

        run_trial_path = None
        for c in candidates:
            if os.path.exists(c):
                run_trial_path = c
                break

        if run_trial_path is None:
            print("")
            print("  WARNING: Could not find run_trial.py to auto-update.")
            print("  Paste values manually into MAP_DATABASE.")
            return

        with open(run_trial_path, 'r') as f:
            content = f.read()

        # Strategy: find the block for this difficulty tag and replace
        # tf_x / tf_y / tf_yaw values using regex on the block.
        #
        # We look for the section:
        #   "tag":   "EASY",
        #   ...
        #   "tf_x":  <old>,  "tf_y": <old>, "tf_yaw": <old>,
        #
        # The block ends at the closing brace of that dict entry.

        tag = self.difficulty

        # Pattern matches the tf_x/tf_y/tf_yaw line(s) inside the block for `tag`
        # We do a two-pass replace: first locate the tag, then replace the three values.

        # Build pattern that finds the tag block and its tf_ values
        # This handles both single-line and split-line formats.
        pattern = (
            r'("tag"\s*:\s*"' + tag + r'".*?)'       # tag line
            r'("tf_x"\s*:\s*)[-+]?\d+\.?\d*'          # tf_x value
            r'(\s*,\s*"tf_y"\s*:\s*)[-+]?\d+\.?\d*'   # tf_y value
            r'(\s*,\s*"tf_yaw"\s*:\s*)[-+]?\d+\.?\d*' # tf_yaw value
        )

        replacement = (
            r'\g<1>'
            + r'\g<2>' + '{:.4f}'.format(tf_x)
            + r'\g<3>' + '{:.4f}'.format(tf_y)
            + r'\g<4>' + '{:.4f}'.format(tf_yaw)
        )

        new_content, n_subs = re.subn(pattern, replacement, content, flags=re.DOTALL)

        if n_subs == 0:
            print("")
            print("  WARNING: Could not auto-update run_trial.py (pattern not found).")
            print("  Please paste the values manually into MAP_DATABASE.")
            return

        with open(run_trial_path, 'w') as f:
            f.write(new_content)

        print("")
        print("  AUTO-UPDATED: {}".format(run_trial_path))
        print("  MAP_DATABASE['{}'] -> tf_x={:.4f}  tf_y={:.4f}  tf_yaw={:.4f}".format(
              tag, tf_x, tf_y, tf_yaw))


if __name__ == '__main__':
    try:
        cal = FrameCalibrator()
        cal.run()
    except rospy.ROSInterruptException:
        pass