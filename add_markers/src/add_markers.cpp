#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// 0=Show pickup marker, 1=Hide marker, 2=Show dropoff marker
int phase = 0;

float pickup_x = -0.5;
float pickup_y = 5.0;
float dropoff_x = 0.0;
float dropoff_y = 1.0;

uint32_t shape = visualization_msgs::Marker::CUBE;
visualization_msgs::Marker marker;

ros::Publisher marker_pub;

void update_marker() {
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = shape;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    if (phase == 0) {
        ROS_INFO("Adding pickup marker");

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Pickup location
        marker.pose.position.x = pickup_x;
        marker.pose.position.y = pickup_y;
        marker.pose.position.z = 0;
    } else if (phase == 1) {
        ROS_INFO("Hiding marker");
        marker.action = visualization_msgs::Marker::DELETE;

    } else if (phase == 2) {
        ROS_INFO("Adding dropoff marker");
        marker.action = visualization_msgs::Marker::ADD;

        // Dropoff location
        marker.pose.position.x = dropoff_x;
        marker.pose.position.y = dropoff_y;
        marker.pose.position.z = 0;
    }

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    marker_pub.publish(marker);
}

void callback_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    float pose_x = msg->pose.pose.position.x;
    float pose_y = msg->pose.pose.position.y;

    float dist_pickup = abs(pose_x - pickup_x) + abs(pose_y - pickup_y);
    float dist_dropoff = abs(pose_x - dropoff_x) + abs(pose_y - dropoff_y);

    if (dist_pickup < 0.05) {
        ROS_INFO("At pickup");
        if (phase == 0) {
            ROS_INFO("Will pick up marker");
            phase++;
            update_marker();
        }
    }
    if (dist_dropoff < 0.05) {
        ROS_INFO("At dropoff");
        if (phase == 1) {
            ROS_INFO("Will drop off marker");
            phase++;
            update_marker();
        }
    }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber odom_sub = n.subscribe("/amcl_pose", 10, callback_pose);

    // Show pickup marker
    update_marker();

    ros::spin();
    return 0;
}