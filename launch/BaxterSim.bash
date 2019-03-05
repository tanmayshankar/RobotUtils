#!/bin/bash

# source_baxter
xvfb-run-safe roslaunch baxter_gazebo baxter_world.launch &
sleep 10
roslaunch RobotUtils publish_description.launch &
rosrun baxter_interface joint_trajectory_action_server.py &
xvfb-run-safe roslaunch RobotUtils baxter_camera.launch &
sleep 10
# rosrun RobotUtils Baxter_Ambidextrous_Planning.py joint_states:=/robot/joint_states &
