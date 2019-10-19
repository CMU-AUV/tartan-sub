#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"

ros::Publisher target_pub;

void callback(const nav_msgs::Odometry::ConstPtr& msg) {
  target_pub.publish(msg->pose.pose);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_to_pose_converter");

  ros::NodeHandle nh("~");

  target_pub = nh.advertise<geometry_msgs::Pose>("/rexrov/pose", 1);

  ros::Subscriber sub = nh.subscribe("/rexrov/pose_gt", 1, callback);

  ros::spin();

  return 0;
}

