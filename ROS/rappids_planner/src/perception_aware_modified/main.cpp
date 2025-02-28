#include <thread>
#include "ros/ros.h"
#include <std_srvs/Empty.h>
#include "rappids_perception_aware.hpp"

//Definitions:
double const mainLoopFrequency = 50;  //Hz

int main(int argc, char **argv) {
  ros::init(argc, argv, "rappids_node");
  ROS_INFO("initializing the rappids node.");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  bool is_sim = false;
  bool open_loop_test = false;
  bool getParam = true; 
  getParam = getParam && private_nh.getParam("planner_specs/is_sim", is_sim);
  getParam = getParam && private_nh.getParam("planner_specs/open_loop_test", open_loop_test);
  if (!getParam){
      throw std::invalid_argument( "main.cpp: ROS parameters not loaded properly!" );
  }

  rappids::RappidsPerceptionAware veh(nh, private_nh);
  ros::Rate loop_rate(mainLoopFrequency);

  //thread number == number of subscribers
  int thread_number;
  if (is_sim){
    thread_number = 4;
  } else{
    thread_number = 6;
  }
  ros::AsyncSpinner spinner(thread_number);
  
  bool shouldStart = false;
  spinner.start();
  while (ros::ok()) {
    veh.Run(shouldStart);
    if (is_sim){
      std_srvs::Empty srv;
      if (ros::service::call("/gazebo/unpause_physics", srv)){
        shouldStart = true;
        break;
      }else{
        ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }else if(!open_loop_test){
      if (veh.GetIsArmed() && veh.GetIsInOffboardMode()) {
        shouldStart = true;
        ROS_INFO("Vehicle armed and is in off board mode");
        break;
      }else{
        ROS_INFO("Waiting for vehicle to be armed and put in offboard mode.");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    }else{
      ROS_INFO("Openloop test with the bag file, state machine directly going to StageFlight.");
      veh.SetFlightStage(rappids::RappidsPerceptionAware::FlightStage::StageFlight);
    }
    loop_rate.sleep();
  }
  
  while (ros::ok() && !veh.GetIsFlightFinished()) {
    veh.Run(shouldStart);
    if (veh.GetIsFlightFinished()) {
      break;
    }
    loop_rate.sleep();
  }

  spinner.stop();
  ros::shutdown();
  return 0;
}
