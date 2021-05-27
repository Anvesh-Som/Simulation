#!/bin/sh

#============== NOTE ==========
#Since I am using ROS Melodic, which is not supported by this course, I used turtlebot3 which has completely different repositories and file structures
#Hence There are some additional things need to be done which aren't asked in the course



# 1) export the model of turtlebot and other environment variables => 
export TURTLEBOT3_MODEL=waffle
export TURTLEBOT_WORLD="$(rospack find map)/last_world.world"
export TURTLEBOT_MAP_FILE="$(rospack find map)/map.yaml"



# then the following are already sourced in my personal Linux installation (PLEASE DO SOURCE MANUALLY FIRST)) 
#	source /opt/ros/melodic/setup.bash
#	source /path/to/catkin_workspace/devel/setup.bash



sleep 1
xterm  -e  "roslaunch turtlebot3_gazebo turtlebot3_udacity_world.launch" &
sleep 5

# gmapping/slam via amcl=> had to use turtlebot3 variations instead of turtlebot_gazebo amcl_demo.launch
xterm -e "roslaunch turtlebot3_navigation turtlebot3_navigation.launch" &
#basically same as amcl_demo.launch , launches amcl.launch itself and has inlcuded rviz launch file too.
sleep 5

# pick_objects node: pre defined poses
xterm -e "source simulation_ws/devel/setup.bash;
rosrun pick_objects pick_objects" &

# add marker node: adds virtual objects
xterm -e "source simulation_ws/devel/setup.bash;
rosrun add_markers add_markers"
