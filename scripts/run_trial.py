#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
run_trial.py
------------
Experiment setup wizard for Fetch HRI teleoperation trials.

This is the single entry point for running any trial -- training or experiment.

WORKFLOW
--------
1. Training session
   - Select TRAIN map, any control scheme, delay = 0.0
   - Participant drives the figure-8 course freely
   - Logs are saved with TRAINING_ prefix

2. Experiment trials  (9 per participant)
   - 3 maps (EASY / MED / HARD) x 3 delays (0.25 / 0.50 / 0.75 s)
   - Experimenter chooses order and control scheme each time
   - Supervisor monitors goal and logs completion

USAGE
-----
    python run_trial.py

The wizard opens two terminal windows:
  [Localization + Logger]  AMCL + VRPN + CSV recorder
  [Teleop Control]         Joystick -> delay -> admittance -> safety

The supervisor runs in this (foreground) terminal for non-training trials.

MAP DATABASE
------------
tf_x / tf_y / tf_yaw values are filled automatically by calibrate_frames.py.
Do NOT edit them by hand unless you know what you are doing.
"""

import os
import sys
import time

# Python 2 compatibility
try:
    input_fn = raw_input
except NameError:
    input_fn = input


# =========================================================
# MAP DATABASE
# Values under tf_x / tf_y / tf_yaw are written automatically
# by calibrate_frames.py after each map calibration.
# =========================================================
MAP_DATABASE = {
    "TRAIN": {
        "yaml":   "lab_map_train.yaml",
        "tf_x":   0.0, "tf_y": 0.0, "tf_yaw": 0.0,
    },
    "EASY": {
        "yaml":   "lab_map_easy.yaml",
        "tf_x":   0.0, "tf_y": 0.0, "tf_yaw": 0.0,
    },
    "MED": {
        "yaml":   "lab_map_med.yaml",
        "tf_x":   0.0, "tf_y": 0.0, "tf_yaw": 0.0,
    },
    "HARD": {
        "yaml":   "lab_map_hard.yaml",
        "tf_x":   0.0, "tf_y": 0.0, "tf_yaw": 0.0,
    },
}

MAP_DIR = "/home/fetchuser/chinmay/fetch_control_ws/my_maps"

# Available delays for experiment trials
EXPERIMENT_DELAYS = [0.25, 0.50, 0.75]


# =========================================================
# Helpers
# =========================================================

def header(title):
    print("\n" + "=" * 60)
    print("  " + title)
    print("=" * 60)


def check_map_file(difficulty):
    """Return full path to map yaml, or None if it does not exist."""
    info = MAP_DATABASE[difficulty]
    path = os.path.join(MAP_DIR, info["yaml"])
    if os.path.exists(path):
        return path
    return None


def check_calibration(difficulty):
    """Return True if tf values have been set (non-zero) for this difficulty."""
    info = MAP_DATABASE[difficulty]
    # Consider calibrated if any value is non-zero
    return not (info["tf_x"] == 0.0 and info["tf_y"] == 0.0 and info["tf_yaw"] == 0.0)


def ask_control_scheme():
    print("\nControl scheme:")
    print("  [1] ARCADE  -- Right Stick: forward/back   |  Left Stick: turn")
    print("  [2] TANK    -- Right Stick: all directions  (single stick)")
    while True:
        choice = input_fn("  Select [1/2]: ").strip()
        if choice == "1":
            return "arcade", "ARCADE"
        elif choice == "2":
            return "tank", "TANK"
        else:
            print("  Please enter 1 or 2.")


def ask_rigid_body():
    rb = input_fn("\nOptiTrack rigid body name (default: Fetch8): ").strip()
    return rb if rb else "Fetch8"


def launch_terminal(title, cmd):
    """Open a gnome-terminal window running cmd."""
    full = "gnome-terminal --title='{}' -- bash -c '{}; exec bash'".format(title, cmd)
    os.system(full)


# =========================================================
# Trial runner  (shared by training + experiment)
# =========================================================

def run_trial(pid, scheme_str, ctrl_tag, difficulty, delay, rb_name, is_training):
    info        = MAP_DATABASE[difficulty]
    map_path    = os.path.join(MAP_DIR, info["yaml"])
    log_prefix  = "{}_{}_{}_Delay{:.2f}".format(
                    pid.upper(), ctrl_tag, difficulty, delay)

    header("TRIAL SUMMARY")
    print("  Participant   : {}".format(pid.upper()))
    print("  Control scheme: {}".format(ctrl_tag))
    print("  Map           : {}".format(difficulty))
    print("  Delay         : {:.2f} s (one-way)".format(delay))
    print("  Rigid body    : {}".format(rb_name))
    print("  Log prefix    : {}".format(log_prefix))
    if is_training:
        print("\n  [TRAINING MODE]  Supervisor will NOT be started.")
    print("\n  Camera system : start SEPARATELY on operator display machine")
    print("                  with delay = {:.2f} s".format(delay))

    input_fn("\n>>> Press ENTER to launch <<<")

    # ---- Build launch commands ----------------------------------------

    cmd_loc = (
        "roslaunch fetch_control localization.launch"
        " map_file:={map}"
        " tf_x:={tf_x}"
        " tf_y:={tf_y}"
        " tf_yaw:={tf_yaw}"
        " rigid_body:={rb}"
        " log_prefix:={lp}"
    ).format(
        map    = map_path,
        tf_x   = info["tf_x"],
        tf_y   = info["tf_y"],
        tf_yaw = info["tf_yaw"],
        rb     = rb_name,
        lp     = log_prefix,
    )

    cmd_teleop = (
        "roslaunch fetch_control teleop.launch"
        " input_device:=joystick"
        " delay_time:={delay}"
        " control_scheme:={scheme}"
    ).format(
        delay  = delay,
        scheme = scheme_str,
    )

    if not is_training:
        cmd_supervisor = (
            "rosrun fetch_control experiment_supervisor.py"
            " _robot_name:={rb}"
            " _goal_name:=Goal"
        ).format(rb=rb_name)

    # ---- Launch -----------------------------------------------------------

    print("\n[1/2] Launching Localization + Logger...")
    launch_terminal("Localization + Logger", cmd_loc)
    time.sleep(5)   # give AMCL time to load map and start publishing

    print("[2/2] Launching Teleop pipeline...")
    launch_terminal("Teleop Control", cmd_teleop)
    time.sleep(2)

    # ---- Supervisor / training wait ---------------------------------------

    if not is_training:
        print("\n" + "=" * 60)
        print("  All systems running. Starting supervisor in this window.")
        print("  Place the goal marker, then press ENTER in the supervisor.")
        print("=" * 60)
        os.system(cmd_supervisor)
    else:
        print("\n" + "=" * 60)
        print("  TRAINING SESSION ACTIVE")
        print("  Participant may drive freely. Press Ctrl+C when done.")
        print("=" * 60)
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n  Training session ended.")


# =========================================================
# Main wizard
# =========================================================

def main():
    header("FETCH HRI EXPERIMENT WIZARD")

    # ------------------------------------------------------------------
    # 1. Participant ID
    # ------------------------------------------------------------------
    pid = input_fn("Participant ID (e.g. P01): ").strip()
    if not pid:
        pid = "P00"

    # ------------------------------------------------------------------
    # 2. Session type
    # ------------------------------------------------------------------
    print("\nSession type:")
    print("  [1] Training  -- figure-8, no delay, no goal supervisor")
    print("  [2] Experiment trial  -- Easy / Med / Hard with delay")
    while True:
        session = input_fn("  Select [1/2]: ").strip()
        if session in ("1", "2"):
            break
        print("  Please enter 1 or 2.")

    is_training = (session == "1")

    # ------------------------------------------------------------------
    # 3. Control scheme
    # ------------------------------------------------------------------
    scheme_str, ctrl_tag = ask_control_scheme()

    # ------------------------------------------------------------------
    # 4. Rigid body name (once per session)
    # ------------------------------------------------------------------
    rb_name = ask_rigid_body()

    # ------------------------------------------------------------------
    # TRAINING PATH
    # ------------------------------------------------------------------
    if is_training:
        map_path = check_map_file("TRAIN")
        if map_path is None:
            print("\n  ERROR: Training map not found.")
            print("  Expected: {}".format(os.path.join(MAP_DIR, MAP_DATABASE["TRAIN"]["yaml"])))
            print("  Run: roslaunch fetch_control build_map.launch")
            print("       rosrun fetch_control save_map.py   -> choose TRAIN")
            sys.exit(1)

        run_trial(pid, scheme_str, ctrl_tag,
                  difficulty  = "TRAIN",
                  delay       = 0.0,
                  rb_name     = rb_name,
                  is_training = True)
        return

    # ------------------------------------------------------------------
    # EXPERIMENT PATH
    # ------------------------------------------------------------------

    # --- Map selection ---
    print("\nSelect map difficulty:")
    difficulties = ["EASY", "MED", "HARD"]
    for i, d in enumerate(difficulties, 1):
        map_ok  = "OK" if check_map_file(d) else "MISSING MAP"
        cal_ok  = "calibrated" if check_calibration(d) else "NOT calibrated"
        print("  [{}] {}   ({} | {})".format(i, d, map_ok, cal_ok))

    while True:
        d_choice = input_fn("  Select [1/2/3]: ").strip()
        if d_choice in ("1", "2", "3"):
            break
        print("  Please enter 1, 2, or 3.")

    difficulty = difficulties[int(d_choice) - 1]

    # Check map exists
    if check_map_file(difficulty) is None:
        print("\n  ERROR: Map file not found for {}.".format(difficulty))
        print("  Run: roslaunch fetch_control build_map.launch")
        print("       rosrun fetch_control save_map.py   -> choose {}".format(difficulty))
        sys.exit(1)

    # Warn if not calibrated
    if not check_calibration(difficulty):
        print("\n  WARNING: {} map has not been calibrated yet.".format(difficulty))
        print("  MoCap and AMCL positions may be misaligned.")
        print("  Run: rosrun fetch_control calibrate_frames.py "
              "_rigid_body_name:={} _difficulty:={}".format(rb_name, difficulty))
        cont = input_fn("  Continue anyway? [y/N]: ").strip().lower()
        if cont != 'y':
            sys.exit(0)

    # --- Delay selection ---
    print("\nOne-way command delay:")
    for i, d in enumerate(EXPERIMENT_DELAYS, 1):
        print("  [{}] {:.2f} s".format(i, d))

    while True:
        delay_choice = input_fn("  Select [1/2/3]: ").strip()
        if delay_choice in ("1", "2", "3"):
            break
        print("  Please enter 1, 2, or 3.")

    delay = EXPERIMENT_DELAYS[int(delay_choice) - 1]

    # --- Run the trial ---
    run_trial(pid, scheme_str, ctrl_tag,
              difficulty  = difficulty,
              delay       = delay,
              rb_name     = rb_name,
              is_training = False)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nWizard cancelled.")