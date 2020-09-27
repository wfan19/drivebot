# Drivebot Simulation
Drivebot is a basic diff-drive robot simulated in Gazebo. The purpose of the drivebot project is to:
1. Create a testing sandbox for ros Robot_Localization configuration, tuning, and benchmarking
2. Familiarize myself with proper ros-Gazebo simulation setups and configurations

## Quick start
To run the drivebot simulation:

Clone this repository to your catkin_workspace (Eg `~/catkin_ws/src`)

`catkin_make` just to be sure:

`$ catkin_make`

To launch the simulation:

`$ roslaunch drivebot_gazebo drivebot_sim.launch`

To launch the movement controller, ekf, and rviz:

`$ roslaunch drivebot_control drivebot_control.launch`

Now, just boot up RQT, and start controlling the car through the `/drivebot/drive_controller/cmd_vel`

## TODO
- Add a robot_localization config that uses wheel odometry and input (cmd_vel)


## Misc notes:
- Gazebo starts paused, so you have to make sure to unpause it to run the simulation