#!/bin/sh
cd "$(dirname "$0")" # go to directory of script

export TURTLEBOT_GAZEBO_WORLD_FILE="/home/robond/catkin_ws/src/map/world.sdf"
echo "Using world file $TURTLEBOT_GAZEBO_WORLD_FILE"

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch "
