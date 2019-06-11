#!/bin/sh
cd "$(dirname "$0")" # go to directory of script

# xterm  -e  " gazebo " &
# sleep 5
# xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
# sleep 5
# xterm  -e  " rosrun rviz rviz" 
xterm -e " roslaunch ../turtlebot_interactions/turtlebot_rviz_launchers/launch"