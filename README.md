# Home Service Robot

This project is my implementation of the "Home Service Robot" project from the Udacity "Robotics Software Engineer" nanodegree.

## test_slam.sh

To test if SLAM is working correctly, run `src/scripts/test_slam.sh`.

*Note:* You might need to adapt TURTLEBOT_GAZEBO_WORLD_FILE to point to the proper world file.

## test_nagigation.sh

To test if AMCL is working correctly, run `src/scripts/test_navigation.sh`.
As above, you might need to adapt the path variables.

*Note:* If the map is not aligned, you might need to adapt the robot's initial pose using the "2D Pose estimate" button in rviz.

## pick_objects.sh

Will direct the robot to a pickup and dropoff locations.

*Note:* As described above, you might need to perform "2D Pose Estimation" to align the map first.

## add_markers.sh

Will add markers at pickup and dropoff locations.

*Note:* You need to add a Marker in rviz to see it.

## home_service.sh

Will simulate pickup and dropoff of the robot.

## Packages used

* gmapping for SLAM: https://github.com/ros-perception/slam_gmapping
* turtlebot_teleop for keyboard control (not used): https://github.com/turtlebot/turtlebot
* turtlebot_rviz_launchers for rviz configurations: https://github.com/turtlebot/turtlebot_interactions
* turtlebot_gazebo for the turtlebot simulation: https://github.com/turtlebot/turtlebot_simulator

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
