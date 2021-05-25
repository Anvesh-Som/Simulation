#!/bin/sh

#============== NOTE ==========
#Since I am using ROS Melodic, which is not supported by this course, I used turtlebot3 which has completely different repositories and file structures
#Hence There are some additional things need to be done which aren't asked in the course
# 1) export the model of turtlebot and workspace => 
export TURTLEBOT3_MODEL=waffle
export TURTLEBOT_WORLD="/home/anvesh/simulation_ws/src/map/last_world.world"
# then the following are already sourced in my personal Linux installation (change names according to your pc)
#	source /opt/ros/melodic/setup.bash
#	source /path/to/catkin_workspace/devel/setup.bash

xterm  -e  "roslaunch turtlebot3_gazebo turtlebot3_udacity_world.launch" &
sleep 5

# gmapping => had to use turtlebot3 variations instead of turtlebot gmapping_demo (includes rviz too unlike turtlebot gmapping_demo.launch in ros kinetic)
xterm -e "roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping" &
#xterm  -e  "roslaunch turtlebot_gazebo gmapping_demo.launch" & 
sleep 5

# VIEW_NAVIGATION: observe map in Rviz
#xterm  -e  "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
#xterm  -e  "roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch" &
#sleep 5

# teleop: operating robot using keyboard
xterm -e "source simulation_ws/devel/setup.bash;
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch"
