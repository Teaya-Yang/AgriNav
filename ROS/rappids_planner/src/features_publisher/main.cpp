#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <random>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "features_publisher");
  ros::NodeHandle n;
  ros::Publisher features_pub = n.advertise<sensor_msgs::PointCloud>("/ov_msckf/loop_feats", 1);

  ros::Rate loop_rate(15);
   
  const int half_feature_number = 100;
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution1(0.0, 10.0);
  std::uniform_real_distribution<double> distribution2(0.0, 2.0);
  std::uniform_real_distribution<double> distribution3(8.0, 10.0);
  std::uniform_real_distribution<double> distribution4(0.0, 10.0);

  while (ros::ok())
  {
    sensor_msgs::PointCloud msg;
    geometry_msgs::Point32 feature;
    for(int count = 0; count < half_feature_number; count++){
        //add a random feature generator here
        feature.x = distribution1(generator);
        feature.y = distribution2(generator);
        feature.z = 0.0;
        msg.points.push_back(feature);
    }
    for(int count = 0; count < half_feature_number; count++){
        //add a random feature generator here
        feature.x = distribution3(generator);
        feature.y = distribution4(generator);
        feature.z = 0.0;
        msg.points.push_back(feature);
    }

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";

    features_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}