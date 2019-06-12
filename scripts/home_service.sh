#!/bin/sh
cd "$(dirname "$0")" # go to directory of script

export TURTLEBOT_GAZEBO_WORLD_FILE="/home/robond/catkin_ws/src/map/world.sdf"
export TURTLEBOT_GAZEBO_MAP_FILE="/home/robond/catkin_ws/src/map/world.yaml"

echo "Using world file $TURTLEBOT_GAZEBO_WORLD_FILE"
echo "Using map file $TURTLEBOT_GAZEBO_MAP_FILE"

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm  -e  " rosrun pick_objects pick_objects "
sleep 5
xterm  -e  " rosrun add_markers add_markers "
