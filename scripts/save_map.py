#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
save_map.py
-----------
Saves the current GMapping map to disk with a fixed difficulty-based filename.

Run this while build_map.launch is active, after you have driven the robot
around the full lab environment and the map looks correct in RViz.

     rosrun fetch_control save_map.py

You will be asked which difficulty this map is for (TRAIN / EASY / MED / HARD).
The map is saved directly with the correct filename so no renaming is needed:
     lab_map_train.yaml / lab_map_easy.yaml / lab_map_med.yaml / lab_map_hard.yaml

Files are saved to /home/fetchuser/chinmay/fetch_control_ws/my_maps/
"""

import rospy
import subprocess
import os
import sys

# Python 2 / 3 compatibility
try:
    input_fn = raw_input
except NameError:
    input_fn = input

# -----------------------------------------------------------------------
# Difficulty -> filename mapping (must match MAP_DATABASE in run_trial.py)
# -----------------------------------------------------------------------
DIFFICULTY_MAP = {
    "1": ("TRAIN", "lab_map_train"),
    "2": ("EASY",  "lab_map_easy"),
    "3": ("MED",   "lab_map_med"),
    "4": ("HARD",  "lab_map_hard"),
}

MAP_DIR = "/home/fetchuser/chinmay/fetch_control_ws/my_maps"


def save_map():
    rospy.init_node('map_saver_tool')

    os.makedirs(MAP_DIR) if not os.path.exists(MAP_DIR) else None

    # ------------------------------------------------------------------
    # Ask which difficulty
    # ------------------------------------------------------------------
    print("\n" + "=" * 50)
    print("  MAP SAVER")
    print("=" * 50)
    print("Which difficulty is this map for?")
    print("  [1] TRAIN  ->  lab_map_train.yaml")
    print("  [2] EASY   ->  lab_map_easy.yaml")
    print("  [3] MED    ->  lab_map_med.yaml")
    print("  [4] HARD   ->  lab_map_hard.yaml")

    choice = input_fn("\nEnter choice [1/2/3/4]: ").strip()

    if choice not in DIFFICULTY_MAP:
        rospy.logerr("Invalid choice '{}'. Exiting.".format(choice))
        sys.exit(1)

    tag, map_name = DIFFICULTY_MAP[choice]
    full_path = os.path.join(MAP_DIR, map_name)
    yaml_path = full_path + ".yaml"
    pgm_path  = full_path + ".pgm"

    # ------------------------------------------------------------------
    # Warn if map already exists
    # ------------------------------------------------------------------
    if os.path.exists(yaml_path):
        print("\n  WARNING: A map already exists for {} difficulty:".format(tag))
        print("           {}".format(yaml_path))
        confirm = input_fn("  Overwrite it? [y/N]: ").strip().lower()
        if confirm != 'y':
            print("  Cancelled. Map NOT saved.")
            rospy.signal_shutdown("User cancelled.")
            sys.exit(0)

    # ------------------------------------------------------------------
    # Save map
    # ------------------------------------------------------------------
    rospy.loginfo("Saving %s map to: %s", tag, full_path)
    print("\nSaving map... (this may take a moment)")

    try:
        subprocess.check_call(["rosrun", "map_server", "map_saver", "-f", full_path])
        print("\n" + "=" * 50)
        print("  MAP SAVED SUCCESSFULLY")
        print("=" * 50)
        print("  Difficulty : {}".format(tag))
        print("  YAML file  : {}.yaml".format(full_path))
        print("  PGM file   : {}.pgm".format(full_path))
        print("\nNext step: run calibrate_frames.py for this map.")
        print("=" * 50 + "\n")
    except subprocess.CalledProcessError as e:
        rospy.logerr("map_saver failed: %s", e)
        rospy.logerr("Is build_map.launch running? Is the map visible in RViz?")
        sys.exit(1)


if __name__ == '__main__':
    save_map()