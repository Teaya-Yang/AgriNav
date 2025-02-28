#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include "hiperlab_rostools/mocap_output.h"
#include <iostream>
#include <thread>

using namespace std;

bool gotFirstMocap = false;
bool gotFirstOdom = false;
tf::Transform firstMocapTF;
tf::Transform firstOdomTF;

void CallbackMocap(const hiperlab_rostools::mocap_output& msg) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg.posx, msg.posy, msg.posz));
  transform.setRotation(
      tf::Quaternion(msg.attq1, msg.attq2, msg.attq3, msg.attq0));
  br.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "world",
                           "/Vehicle" + std::to_string(msg.vehicleID)));
  if (!gotFirstMocap) {
    firstMocapTF = transform;
    gotFirstMocap = true;
  }
}

void rosThreadFn() {
  // We run this function as a separate thread, allows us to continuously service any subscribers.
  ros::spin();
}

int main(int argc, char** argv) {

  int numVehicles = argc - 1;
  if (numVehicles < 1) {
    cout << "ERROR: Must specify at least one vehicle ID\n"
         << "\t You can input a list of vehicle IDs\n";
    return -1;
  }

  int vehicleId[numVehicles];
  for (unsigned i = 0; i < numVehicles; i++) {
    vehicleId[i] = atol(argv[1 + i]);
    if (vehicleId[i] <= 0 || vehicleId[i] > 255) {
      cout << "ERROR: invalid vehicle ID <" << argv[1 + i] << ">\n";
      return -1;
    }
  }

  std::string vehIDstr[numVehicles];
  for (unsigned i = 0; i < numVehicles; i++) {
    vehIDstr[i] = std::to_string(vehicleId[i]);
    ros::init(argc, argv, "mocap_to_rviz" + vehIDstr[i]);
  }

  ros::NodeHandle n;
  ros::Rate r(60);  // 60 Hz
  ros::Publisher quad_pub = n.advertise<visualization_msgs::Marker>(
      "quad_marker", 1);

  std::thread rosThread(rosThreadFn);

  ros::Subscriber sub_mocap[numVehicles];
  for (unsigned i = 0; i < numVehicles; i++) {
    sub_mocap[i] = n.subscribe("mocap_output" + vehIDstr[i], 1,
                                          CallbackMocap);
    cout << "Subscribed to mocap_output" << vehIDstr[i] << "\n";
  }

  while (ros::ok()) {
    r.sleep();
  }
}