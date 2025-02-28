#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

void imuCallback(const sensor_msgs::Imu& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(-0.03, 0.007, 0.016) );
  tf::Quaternion q;
  q.setEulerZYX(0, -1.570796, 1.570796);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, msg.header.stamp, "imu", "d455_link"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_imu_d455_link");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/d455/imu", 1, &imuCallback);

  ros::spin();
  return 0;
};