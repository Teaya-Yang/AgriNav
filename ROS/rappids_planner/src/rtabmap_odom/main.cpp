#include "ros/ros.h"
#include <nav_msgs/Odometry.h>

//remove the covariances of the pose and twist
//which sometimes causes Rtabmap to fail
nav_msgs::Odometry ov_msckf_odom;
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    ov_msckf_odom.header = msg->header;
    ov_msckf_odom.child_frame_id = msg->child_frame_id;
    ov_msckf_odom.pose.pose = msg->pose.pose;
    ov_msckf_odom.twist.twist = msg->twist.twist;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtabmap_odom");
  ros::NodeHandle n;
  ros::Publisher odom_rtabmap_pub = n.advertise<nav_msgs::Odometry>("/ov_msckf/rtabmap_odom", 1);
  ros::Subscriber odom_sub = n.subscribe("/ov_msckf/odomimu", 1, odomCallback);
  
  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    odom_rtabmap_pub.publish(ov_msckf_odom);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}