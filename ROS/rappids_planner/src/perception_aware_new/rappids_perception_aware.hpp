#pragma once

#include "../../include/math/rotation.hpp"
#include "../../include/math/vec3.hpp"
#include "ros/ros.h"

#include <geometry_msgs/TwistStamped.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <visualization_msgs/Marker.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <vector>
#include "../../include/rappids/depth_image_planner.hpp"
#include "rappids_planner/planner_diagnostics.h"
#include "rappids_planner/drone_status.h"
#include <mutex>

// Below are mine
#include "Common/DataTypes/RadioTypes.hpp"
#include "Common/DataTypes/TelemetryPacket.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Offboard/VIOStateEstimator.hpp"
#include "Components/Offboard/QuadcopterController.hpp"
#include "Components/Offboard/SafetyNet.hpp"
#include "Components/Logic/QuadcopterConstants.hpp"

#include "hiperlab_rostools/estimator_output.h"
#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/joystick_values.h"
#include "hiperlab_rostools/telemetry.h"
#include "hiperlab_rostools/LoopPoseData.h"
#include <std_msgs/Bool.h> 


using namespace CommonMath;
using namespace RectangularPyramidPlanner;

namespace rappids {

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0), yaw(0.0) {
  }

  WaypointWithTime(double t, Vec3 p, Vec3 v, Vec3 a, double y, double yr)
      : waiting_time(t),
        yaw(y),
        yawrate(yr) {
    position = Eigen::Vector3d(p.x, p.y, p.z);
    velocity = Eigen::Vector3d(v.x, v.y, v.z);
    acceleration = Eigen::Vector3d(a.x, a.y, a.z);
  }

  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  double yaw;
  double yawrate;
  double waiting_time;
};
const int64_t kNanoSecondsInSecond = 1000000000;

class RappidsPerceptionAware {
 public:
  enum FlightStage{
    StageWaitForStart = 0,
    StageTakeoff = 1,
    StageFlight = 2,
    StageLanding = 3,
    StageComplete = 4,
  };

  RappidsPerceptionAware(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

  void InitializeParams();
  void CallbackOdometry(const hiperlab_rostools::LoopPoseData& msg); 
  void CallbackCallbackTelemetry(const hiperlab_rostools::telemetry& msg);
  void CallbackFeatures(const sensor_msgs::PointCloud& msg);
  void CallbackDepthImages(const sensor_msgs::ImageConstPtr& msg,
                           const sensor_msgs::CameraInfoConstPtr& info_msg);
  bool GetIsArmed() {
    return _armed;
  }
  bool GetIsInOffboardMode() {
    return _inOffboardMode;
  }
  bool GetIsFlightFinished() {
    return _flightFinished;
  }
  void publishReferenceCmd(const WaypointWithTime& current_waypoint);
  void publishPlannerDiagnostics(DepthImagePlanner& planner, 
                                 const CommonMath::Trajectory& trajectory_parameters,
                                 const Vec3& vel_pic, 
                                 const Vec3& acc_pic, 
                                 const Vec3& g_pic, 
                                 const uint64& traj_seq,
                                 const bool& traj_found, 
                                 const double& speed_cost, 
                                 const double& perc_cost); // planner internal logging
  void SetFlightStage(const FlightStage& flight_stage){
    _flightStage = flight_stage;
  }
  void Run(bool shouldStart, bool shouldStop);
  void PublishEstimate(VIOStateEstimator::VIOEstimatedState estState);


inline geometry_msgs::Vector3 toGeometricMsgVec3(const Vec3& rhs) {
    geometry_msgs::Vector3 lhs;
    lhs.x = rhs.x;
    lhs.y = rhs.y;
    lhs.z = rhs.z;
    return lhs;
}

inline geometry_msgs::Quaternion toGeometricMsgQuaternion(const Rotation& rhs) {
    geometry_msgs::Quaternion lhs;
    lhs.w = rhs[0];
    lhs.x = rhs[1];
    lhs.y = rhs[2];
    lhs.z = rhs[3];
    return lhs;
}

 protected:
 
  //check if the goal has been reached
  bool isGoalReached(){
      if ((_est_pos-Vec3(_goal_posx,_goal_posy,_goal_posz)).GetNorm2() <= _pos_tolerance){
          return true;
      }else{
          return false;
      }
  }

  void visualizeControl();

  Rotation _globalAtt;  //Rotates vectors in the global frame (from openvins) to odometry frame
  Rotation _depthCamAtt;  // Rotates vectors in camera frame to quad frame
  ros::Time _trajResetTime;

  //subscriber and publishers
  ros::Subscriber _subTracking, _subTelemetry, _subFeatures, _subVehState, _subRC;
  image_transport::ImageTransport _it;
  image_transport::CameraSubscriber _subDepthImages;
  ros::Publisher _ref_pub, _marker_pub, _pub_planner_diagnotics, _pub_flight_stage, _pubCmd;

  //state estimation
  std_msgs::Header _odom_header; //for setting the frame of the visualization markers
  Vec3 _est_pos; //vehicle position in the odometry frame
  Vec3 _est_vel; //vehicle position in the odometry frame
  Vec3 _est_acc; //vehicle position in the odometry frame
  Rotation _est_att; //vehicle attitude in the odometry frame

  std::shared_ptr<
      RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator> _traj;
  Rotation _trajAtt;
  Vec3 _trajOffset;
  Vec3 _lastcmdPos;
  Vec3 _lastcmdVel;
  Vec3 _lastcmdAcc;
  Vec3d cmdPos;  
  Vec3d _finalPosition;  


  //vehicle status
  bool _armed;
  bool _inOffboardMode;
  bool _flightFinished;
  //planner status
  bool _planner_started;
  bool _execute_last_traj; //true if the current traj can already move us close to the goal 
  bool _goal_reached;  //true if the last traj from the planner finishes execution
  FlightStage _flightStage, _lastFlightStage;
  ros::Time _stateStartTime;  //keep track of time we've been in current stage
  //planner params:
  bool _is_sim;
  bool _perception_aware;
  double _k_perc; //coefficient of the perception-aware cost term
  double _comp_time;
  double _min_sample_depth, _max_sample_depth, _min_sample_time, _max_sample_time;
  double _physical_vehicle_radius, _vehicle_radius_for_planning;
  double _minimum_collision_distance;
  bool _use_px4_odom;
  //trajectory params:
  double _min_allowed_thrust, _max_allowed_thrust;
  double _max_allowed_ang_vel, _max_allowed_vel;
  double _min_allowed_height;
  //depth camera params:
  double _depthScale;   // pixel value to m
  double _camera_fps; // the fps of the camera
  double _exposure_time; //the exposure time of the camera in [s]
  //goal params:
  double _goal_posx, _goal_posy, _goal_posz;
  double _pos_tolerance;
  double _desired_yaw;
  Vec3 _pos_before_landing;

  //! The vector containing the features in the camera frame
  std::vector<Vec3> _features_odom;
  std::mutex _feature_array_mutex;

  //! ROS node handle.
  ros::NodeHandle& _nh;
  ros::NodeHandle& _private_nh;

  std::shared_ptr<VIOStateEstimator> _est;
  double _systemLatencyTime; 

};
}
