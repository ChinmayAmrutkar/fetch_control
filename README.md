# fetch_control — HRI Teleoperation Control System

A ROS package for studying the effects of network latency on human teleoperation of the Fetch mobile robot. This workspace contains **only the control pipeline** — the camera/visual-feedback system runs independently on the operator's display machine with its own delay setting, keeping the two subsystems computationally isolated.

---

## Architecture

```
Joystick
   │
   ▼
joystick_translator.py  ──►  /cmd_vel_raw
   │
   ▼
cmd_delay.py            ──►  /cmd_vel_delayed   (holds for N seconds)
   │
   ▼
admittance_controller.py──►  /cmd_vel_teleop    (mass-damper smoothing)
   │
   ▼
safety_controller_twist.py ►  /cmd_vel          (LIDAR obstacle guard)
   │
   ▼
Fetch robot base
```

Running in parallel:

```
localization.launch:
  AMCL  +  OptiTrack VRPN  +  experiment_logger.py  →  4× CSV files
```

---

## Workspace structure

```
fetch_control_ws/
├── logs/                          Trial CSV files written here
├── my_maps/                       Saved .yaml + .pgm map files
└── src/
    └── fetch_control/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   ├── teleop.launch          Control pipeline (input → safety)
        │   ├── localization.launch    AMCL + MoCap + logger
        │   └── build_map.launch       GMapping SLAM for map creation
        └── scripts/
            ├── run_trial.py           Experiment wizard (start here)
            ├── joystick_translator.py Joystick → /cmd_vel_raw
            ├── cmd_delay.py           Simulated network latency
            ├── admittance_controller.py Mass-damper smoother
            ├── safety_controller_twist.py LIDAR safety guard
            ├── experiment_logger.py   4× CSV data recorder
            ├── calibrate_frames.py    MoCap ↔ map frame alignment
            └── save_map.py            Save GMapping map to disk
```

---

## Setup

### 1. Create and build the workspace

```bash
mkdir -p ~/fetch_control_ws/src
cd ~/fetch_control_ws/src
# Copy the fetch_control package folder here, then:
cd ~/fetch_control_ws
catkin_make
source devel/setup.bash
```

Add to `~/.bashrc` so sourcing is automatic:

```bash
echo "source ~/fetch_control_ws/devel/setup.bash" >> ~/.bashrc
```

### 2. Dependencies

```bash
sudo apt install ros-melodic-joy
sudo apt install ros-melodic-teleop-twist-keyboard
sudo apt install ros-melodic-vrpn-client-ros
sudo apt install ros-melodic-topic-tools
# fetch_navigation is included with the Fetch robot ROS packages
```

---

## One-time map setup (per lab configuration)

### Step 1 — Build the map

```bash
roslaunch fetch_control build_map.launch
```

Drive the robot around the entire lab area. When the map looks complete in RViz:

```bash
rosrun fetch_control save_map.py
```

The map is saved to `~/fetch_control_ws/my_maps/lab_map_<timestamp>.yaml`.  
Rename it to match the filename in `MAP_DATABASE` (e.g. `lab_map_easy.yaml`).

### Step 2 — Calibrate the MoCap ↔ map transform

This computes the static transform that aligns the AMCL map frame with the OptiTrack world frame. Run it **once per map**, with the VRPN client running.

```bash
# Terminal 1 — start localization (no logger needed)
roslaunch fetch_control localization.launch map_file:=/full/path/to/map.yaml

# Terminal 2 — set a 2D Pose Estimate in RViz, wait for AMCL to converge, then:
rosrun fetch_control calibrate_frames.py _rigid_body_name:=Fetch8
```

Copy the printed `tf_x`, `tf_y`, `tf_yaw` values into `MAP_DATABASE` in `run_trial.py`.

---

## Running an experiment

```bash
python ~/fetch_control_ws/src/fetch_control/scripts/run_trial.py
```

The wizard will prompt for:

| Prompt | Options | Notes |
|---|---|---|
| Participant ID | e.g. `P01` | Prepended to all log filenames |
| Control scheme | `1` arcade / `2` tank | Sets joystick axis mapping |
| Map | `0` TRAIN · `1` EASY · `2` MED · `3` HARD | Loads the corresponding .yaml |
| One-way delay | `0.0`, `0.25`, `0.5`, `0.75` s | Applied to commands; set camera to the same value |
| Rigid body name | e.g. `Fetch8` | Must match the name in OptiTrack Motive |

After confirmation, the wizard:

1. Opens a `Localization + Logger` terminal (AMCL + MoCap + CSV recorder)
2. Opens a `Teleop Control` terminal (joystick → delay → admittance → safety)
3. Stays in the foreground running the **experiment supervisor** (non-training trials)

> **Camera system:** start it separately on the operator's display machine and set its delay to the same one-way value.

### Training trials

Select map `0` (TRAIN / figure-8). The supervisor is skipped — the participant drives freely. Log files are prefixed with `TRAIN_...` and can be discarded after the session if not needed.

---

## Log files

Each trial produces four CSV files in `~/fetch_control_ws/logs/` (or `/home/fetchuser/chinmay/fetch_control_ws/logs/` on the Fetch PC):

| File | Contents | Trigger |
|---|---|---|
| `*_A_mocap_*.csv` | MoCap pose, raw velocity, smoothed velocity, joystick | Every MoCap frame (~100 Hz) |
| `*_B_amcl_*.csv` | AMCL pose, odometry, joystick | Every AMCL update |
| `*_C1_sync_event_*.csv` | AMCL + MoCap + odometry together | Every AMCL update (when MoCap also available) |
| `*_C2_sync_fixed_*.csv` | AMCL + MoCap + odometry together | Fixed 20 Hz timer |

All files share the same columns for joystick input (`Input_JX`, `Input_JY`) and an `Event_Msg` column with values:

- `RUNNING` — normal operation
- `SAFETY_STOP` — LIDAR guard blocked a forward command
- `SUCCESS_GOAL_REACHED` — supervisor confirmed goal reached
- `MAX_TIME` — 120-second trial clock expired

Filenames follow the pattern:  
`{PID}_{SCHEME}_{MAP}_Delay{D}_{tag}_{timestamp}.csv`  
e.g. `P03_ARCADE_MED_Delay0.25_C2_sync_fixed_2025-06-01-10-30-00.csv`

---

## Node reference

### joystick_translator.py

Reads `/my_joy` (remapped from `/joy` to avoid conflicts with the Fetch internal driver) and publishes `geometry_msgs/Twist` to `/cmd_vel_raw`.

| Parameter | Default | Description |
|---|---|---|
| `~control_scheme` | `arcade` | `arcade` or `tank` |
| `~scale_linear` | `0.5` | Max linear speed (m/s) |
| `~scale_angular` | `1.0` | Max angular speed (rad/s) |
| `~deadman_button` | `4` | Button index that must be held (LB) |
| `~axis_linear` | `1` | Left stick Y axis index |
| `~axis_angular` | `0` | Left stick X axis index |
| `~axis_right_y` | `4` | Right stick Y axis index (tank mode) |

### cmd_delay.py

Buffers `/cmd_vel_raw` in a timestamped deque and only releases messages after `delay` seconds.

| Parameter | Default | Description |
|---|---|---|
| `~delay` | `0.0` | One-way delay in seconds |

### admittance_controller.py

Applies a virtual mass-damper to smooth delayed commands. Treats the incoming velocity as a "force" and integrates `a = (F − cv) / m`.

| Parameter | Default | Description |
|---|---|---|
| `~virtual_mass` | `0.5` | Inertia (higher = slower acceleration) |
| `~virtual_damping` | `1.0` | Resistance (higher = stops faster) |
| `~max_linear_vel` | `1.0` | m/s saturation limit |
| `~max_angular_vel` | `1.5` | rad/s saturation limit |

Includes a **0.5 s watchdog**: if no command arrives the target is forced to zero, preventing runaway if the upstream node dies.

### safety_controller_twist.py

Real-time LIDAR guard. Operates entirely outside the delay chain. Monitors the forward arc (centre ±1/6 of the full scan). If an obstacle is within `0.3 m`, forward velocity is zeroed while turning is still permitted.

Publishes `/safety_status` (`Bool`, `True` = stop active) so the logger can mark events.

### experiment_logger.py

Subscribes to all data streams and writes four CSV files. Shuts down cleanly on ROS shutdown, flushing and closing all file handles.

### calibrate_frames.py

One-shot utility. Computes the rigid transform between the AMCL map frame and the OptiTrack world frame from a single pair of simultaneous poses. Prints the result and exits.

### save_map.py

Wraps `rosrun map_server map_saver` with an auto-timestamped filename.

---

## Controller layout (ONN / Xbox)

```
LB (Button 4) = Deadman — must be held for any motion

Arcade mode:
  Right Stick Vertical   (Axis 4) = Forward / Backward
  Left  Stick Horizontal (Axis 3) = Turn left / right

Tank mode:
  Left  Stick Vertical (Axis 1) = Left track
  Right Stick Vertical (Axis 4) = Right track
```

---

## ROS topic map

```
/my_joy               sensor_msgs/Joy          joystick driver output
/cmd_vel_raw          geometry_msgs/Twist      raw joystick intent
/cmd_vel_delayed      geometry_msgs/Twist      after network delay
/cmd_vel_teleop       geometry_msgs/Twist      after admittance smoothing
/cmd_vel              geometry_msgs/Twist      final command to robot base
/safety_status        std_msgs/Bool            True = obstacle stop active
/goal_reached         std_msgs/Bool            True = supervisor confirmed goal
/amcl_pose            geometry_msgs/PoseWithCovarianceStamped
/odom                 nav_msgs/Odometry
/base_scan            sensor_msgs/LaserScan
/vrpn_client_node/<body>/pose  geometry_msgs/PoseStamped
```

---

## Separation from the camera system

The camera system (`cam_server_V3.py` and its client) runs on a **separate machine** with its own delay parameter set to match the command delay. This decoupling eliminates base CPU load from video processing on the Fetch PC and ensures the control pipeline latency is deterministic.

The two systems share only these implicit synchronisation points:

- The experimenter sets both delays to the same value before each trial.
- The camera client subscribes to `/safety_status` and `/goal_reached` over the ROS network for screen overlays.

---

## License

BSD — based on original Fetch Robotics teleoperation work.
