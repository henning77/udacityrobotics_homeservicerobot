#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

float pickup_x = -0.5;
float pickup_y = 5.0;
float dropoff_x = 0.0;
float dropoff_y = 1.0;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickup;

  // set up the frame parameters
  pickup.target_pose.header.frame_id = "map";
  pickup.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pickup.target_pose.pose.position.x = pickup_x;
  pickup.target_pose.pose.position.y = pickup_y;
  pickup.target_pose.pose.orientation.w = 1.0;

   // Send the pickup position and orientation for the robot to reach
  ROS_INFO("Sending pickup goal");
  ac.sendGoal(pickup);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its pickup
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, we reached the pickup position, will wait 5 seconds");
    ros::Duration(5.0).sleep();
  } else {
    ROS_INFO("We failed to reach the pickup positionm, will abort");
    ros::Duration(5.0).sleep();
    return 0;
  }


  move_base_msgs::MoveBaseGoal dropoff;

  // set up the frame parameters
  dropoff.target_pose.header.frame_id = "map";
  dropoff.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  dropoff.target_pose.pose.position.x = dropoff_x;
  dropoff.target_pose.pose.position.y = dropoff_y;
  dropoff.target_pose.pose.orientation.w = 1.0;

   // Send the dropoff position and orientation for the robot to reach
  ROS_INFO("Sending dropoff goal");
  ac.sendGoal(dropoff);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its dropoff
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, we reached the dropoff position, will wait 5 seconds");
    ros::Duration(5.0).sleep();
  } else {
    ROS_INFO("We failed to reach the dropoff positionm, will abort");
    ros::Duration(5.0).sleep();
    return 0;
  }

  return 0;
}