# Home Service Robot

This project is my implementation of the "Home Service Robot" project from the Udacity "Robotics Software Engineer" nanodegree.

## test_slam.sh

To test if SLAM is working correctly, run `src/scripts/test_slam.sh`.
You might need to adapt TURTLEBOT_GAZEBO_WORLD_FILE to point to the proper world file.

## test_nagigation.sh

To test if AMCL is working correctly, run `src/scripts/test_navigation.sh`.
As above, you might need to adapt the path variables.

If the map is not aligned, you might need to adapt the robot's initial pose using the "2D Pose estimate" button in rviz.

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
