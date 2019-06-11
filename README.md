# Home Service Robot

This project is my implementation of the "Home Service Robot" project from the Udacity "Robotics Software Engineer" nanodegree.

## Packages used

* gmapping: https://github.com/ros-perception/slam_gmapping
* turtlebot_teleop: https://github.com/turtlebot/turtlebot
* turtlebot_rviz_launchers: https://github.com/turtlebot/turtlebot_interactions
* turtlebot_gazebo: https://github.com/turtlebot/turtlebot_simulator

To install all required packages, run:
```
catkin_make
source devel/setup.bash
rosdep -i install gmapping
rosdep -i install turtlebot_teleop
rosdep -i install turtlebot_rviz_launchers
rosdep -i install turtlebot_gazebo
catkin_make
source devel/setup.bash
```
