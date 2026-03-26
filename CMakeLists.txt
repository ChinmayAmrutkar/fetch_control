cmake_minimum_required(VERSION 2.8.3)
project(fetch_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
)

catkin_package()

# Mark Python scripts as executable
catkin_install_python(PROGRAMS
  scripts/joystick_translator.py
  scripts/cmd_delay.py
  scripts/admittance_controller.py
  scripts/safety_controller_twist.py
  scripts/experiment_logger.py
  scripts/experiment_supervisor.py
  scripts/calibrate_frames.py
  scripts/save_map.py
  scripts/run_trial.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
