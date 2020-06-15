On a Ubuntu computer installed with ROS, install ROS joystick driver

  sudo apt-get install ros-melodic-joystick-drivers

﻿Unzip ground_based_autonomy_slimmed folder, in a terminal, go to the folder

  catkin_make

  source devel/setup.sh

Then, launch Gazebo simulator and use joystick to drive the vehicle

  roslaunch vehicle_simulator vehicle_simulator.launch

Use right joystick to navigate, pressing lower-left button enters way-point following mode where speed is set by pushing the right joystick, pressing lower-right button cancels obstacle check, and pressing upper-right button clears terrain map.


Code is tested with RC controller

  https://www.amazon.com/EasySMX-Wireless-Controller-Gamepads-Vibration/dp/B07MVYSSFV


In "vehicle_simulator/launch/vehicle_simulator.launch", uncomment

  <include file="$(find waypoint_example)/launch/waypoint_example.launch" />

to launch way-point following example. The example also shows how to use navigation boundary.


In "local_planner/launch/local_planner.launch", uncomment

  <include file="$(find terrain_analysis)/launch/terrain_analysis.launch" />

and set "useTerrainAnalysis = true" to use terrain analysis in obstacle avoidance.
