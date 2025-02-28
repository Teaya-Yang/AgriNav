#include "rappids_perception_aware.hpp"
#include <std_msgs/Bool.h> 

using namespace rappids;
using namespace CommonMath;
using namespace std;

RappidsPerceptionAware::RappidsPerceptionAware(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    : _nh(nh),
      _private_nh(private_nh),
      _depthCamAtt{},
      _it(nh),
      _systemLatencyTime(0),
      _finalPosition(Vec3d(0,0,0))
      _est_pos(Vec3(0, 0, 0)),
      _est_vel(Vec3(0, 0, 0)),
      _est_acc(Vec3(0, 0, 0)),
      _lastcmdPos(Vec3(0, 0, 0)),
      _lastcmdVel(Vec3(0, 0, 0)),
      _lastcmdAcc(Vec3(0, 0, 0)),
      _est_att(Rotation::Identity()),
      _trajAtt(Rotation::Identity()),
      _trajOffset(Vec3(0, 0, 0)),
      _is_sim(true),
      _perception_aware(true),
      _planner_started(false),
      _execute_last_traj(false),
      _goal_reached(false),  
      _armed(false),
      _flightFinished(false),
      _inOffboardMode(false),
      _flightStage(StageWaitForStart),
      _lastFlightStage(StageComplete),
      _stateStartTime(ros::Time::now()),
      _desired_yaw(0),
      _pos_before_landing(Vec3(0, 0, 0)),
      _use_px4_odom(false){
  InitializeParams();

  //subscribe to features
  _subFeatures = _nh.subscribe("/ov_msckf/loop_feats", 1, &RappidsPerceptionAware::CallbackFeatures, this);
  
  //publishers for helper topics
  _marker_pub = _nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  _pub_planner_diagnotics = _nh.advertise<rappids_planner::planner_diagnostics>("planner_diagnostics", 10);
  _pub_flight_stage = _nh.advertise<rappids_planner::drone_status>("drone_status", 1);
  _ref_pub = _nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("trajectory_raw", 10);

  
  //Rotates vectors in camera frame to quad frame
  // _depthCamAtt = Rotation::FromEulerYPR(-90.0 * M_PI / 180.0, 0.0 * M_PI / 180.0, -90.0 * M_PI / 180.0);
  _depthCamAtt = Rotation::FromEulerYPR(180.0 * M_PI / 180.0, 0.0 * M_PI / 180.0, 0.0 * M_PI / 180.0); // For us, depth camera is mounted to the back...

  //Everything should be the same, sim or not
  // 1) odometry 2)telemetry 3) Depth image

  // 1) odometry
  _subTracking = _nh.subscribe("/odom_data", 1, &RappidsPerceptionAware::CallbackOdometry, this);

  // 2) Telemetry
  _subTelemetry = _nh.subscribe("/telemetry" + std::to_string(_id), 1, &RappidsPerceptionAware::CallbackTelemetry, this);

  // 3) Depth image
  _subDepthImages = _it.subscribeCamera("/d455/depth/image_rect_raw", 1, &RappidsPerceptionAware::CallbackDepthImages, this, image_transport::TransportHints());

  // 4) Publish command
  _pubCmd = _nh.advertise<hiperlab_rostools::radio_command>("radio_command" + std::to_string(_id), 1)

  ROS_INFO("Rappids perception aware planner initialized.");
}

void RappidsPerceptionAware::InitializeParams() {
  bool getParam = true; 
  getParam = getParam && _private_nh.getParam("planner_specs/is_sim", _is_sim);
  getParam = getParam && _private_nh.getParam("planner_specs/perception_aware", _perception_aware);
  getParam = getParam && _private_nh.getParam("planner_specs/k_perc", _k_perc);
  getParam = getParam && _private_nh.getParam("planner_specs/computation_time", _comp_time);
  getParam = getParam && _private_nh.getParam("planner_specs/physical_vehicle_radius", _physical_vehicle_radius);
  getParam = getParam && _private_nh.getParam("planner_specs/vehicle_radius_for_planning", _vehicle_radius_for_planning);
  getParam = getParam && _private_nh.getParam("planner_specs/minimum_collision_distance", _minimum_collision_distance);
  getParam = getParam && _private_nh.getParam("planner_specs/min_sample_depth", _min_sample_depth);
  getParam = getParam && _private_nh.getParam("planner_specs/max_sample_depth", _max_sample_depth);
  getParam = getParam && _private_nh.getParam("planner_specs/min_sample_time", _min_sample_time);
  getParam = getParam && _private_nh.getParam("planner_specs/max_sample_time", _max_sample_time);
  getParam = getParam && _private_nh.getParam("planner_specs/use_px4_odom", _use_px4_odom);

  getParam = getParam && _private_nh.getParam("depth_camera/depth_scale", _depthScale);
  getParam = getParam && _private_nh.getParam("depth_camera/fps", _camera_fps);
  getParam = getParam && _private_nh.getParam("depth_camera/exposure_time", _exposure_time);

  getParam = getParam && _private_nh.getParam("trajectory/min_allowed_thrust", _min_allowed_thrust);
  getParam = getParam && _private_nh.getParam("trajectory/max_allowed_thrust", _max_allowed_thrust);
  getParam = getParam && _private_nh.getParam("trajectory/max_allowed_ang_vel", _max_allowed_ang_vel);
  getParam = getParam && _private_nh.getParam("trajectory/max_allowed_vel", _max_allowed_vel);
  getParam = getParam && _private_nh.getParam("trajectory/min_allowed_height", _min_allowed_height);

  getParam = getParam && _private_nh.getParam("goal/posx", _goal_posx);
  getParam = getParam && _private_nh.getParam("goal/posy", _goal_posy);
  getParam = getParam && _private_nh.getParam("goal/posz", _goal_posz);
  getParam = getParam && _private_nh.getParam("goal/pos_tolerance", _pos_tolerance);
  if (!getParam){
      throw std::invalid_argument( "ROS parameters not loaded properly!" );
  }
}


void RappidsPerceptionAware::CallbackOdometry(const hiperlab_rostools::LoopPoseData& msg) {
  _est->UpdateWithVIO(
        Vec3d(msg.position.x, msg.position.y, msg.position.z),
        Rotationd(msg.m4, msg.m1, msg.m2, msg.m3),
        Vec3d(msg.linear_velocity.x, msg.linear_velocity.y, msg.linear_velocity.z),
        Vec3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z));
  //ROS_INFO("received odometry");
}

void RappidsPerceptionAware::CallbackTelemetry(const hiperlab_rostools::telemetry& msg) {
  _lastTelWarnings = msg.warnings;
}

void RappidsPerceptionAware::CallbackFeatures(const sensor_msgs::PointCloud &msg){
  _feature_array_mutex.lock();
  _features_odom.clear();
  //
  for(const geometry_msgs::Point32& feature : msg.points){
    // active feature points in the global frame
    Vec3 featurePointOdom = Vec3(-feature.x, -feature.y, feature.z);
    _features_odom.push_back(featurePointOdom);
  }    
  _feature_array_mutex.unlock();
}

void RappidsPerceptionAware::CallbackDepthImages(
    const sensor_msgs::ImageConstPtr &msg,
    const sensor_msgs::CameraInfoConstPtr &info_msg) {
  if(_flightStage == StageFlight){
    ros::Time planningStartTime = ros::Time::now();
    CV_Assert(msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat depthImg = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;

    //ROS_INFO("received image");

    double fx = info_msg->K[0];
    double fy = info_msg->K[4];
    double cx = info_msg->K[2];
    double cy = info_msg->K[5];

    //ROS_INFO("fx: %6f / fy: %6f / cx: %6f / cy: %6f ", fx, fy, cx, cy);  `

    // Initialize the planner
    DepthImagePlanner planner(depthImg, _depthScale, fx, cx, cy, _physical_vehicle_radius, 
                              _vehicle_radius_for_planning, _minimum_collision_distance,
                              _min_allowed_thrust, _max_allowed_thrust,
                              _max_allowed_ang_vel, _max_allowed_vel, _min_allowed_height);

    std::random_device rd;
    // Set random seed to be the seq ID of a depth image
    planner.SetRandomSeed(rd());

    // Find trajectories at the stereo camera's picture frame:
    // camera's pointed direction is Z direction.
    Vec3 vel_picture = _depthCamAtt.Inverse() * estState.att.Inverse() * _lastcmdVel;
    Vec3 acc_picture = _depthCamAtt.Inverse() * estState.att.Inverse() * _lastcmdAcc;
    Vec3 g_picture = _depthCamAtt.Inverse() * estState.att.Inverse() * Vec3(0, 0, -9.81);

    RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator traj(Vec3(0, 0, 0), vel_picture, acc_picture, g_picture);
    std::vector<TrajectoryTest> trajectories;

    // vechicle's current position and goal in odometry's frame
    Vec3 current_goal_odom = Vec3(_goal_posx, _goal_posy, _goal_posz);
    // current goal in the picture's frame
    Vec3 current_goal_picture = _depthCamAtt.Inverse() *
                                    estState.att.Inverse() *
                                    (current_goal_odom - estState.pos);

    SamplingParameters samplingParameters;
    samplingParameters.minDepth = _min_sample_depth;
    samplingParameters.maxDepth = _max_sample_depth;
    samplingParameters.minTime = _min_sample_time;
    samplingParameters.maxTime = _max_sample_time;

    // Find trajectories
    bool trajectory_found;
    double speed_cost = 0;
    double perception_cost = 0;
    if(_perception_aware){
      _feature_array_mutex.lock();
      std::vector<Vec3> features_odom = _features_odom;
      _feature_array_mutex.unlock();

      trajectory_found = planner.FindLowestCostTrajectory(
          traj, trajectories, speed_cost, perception_cost, 
          _k_perc, _comp_time, samplingParameters, 
          current_goal_picture, features_odom, _depthCamAtt, 
          estState.att, estState.pos, _camera_fps, _exposure_time);
    }else{
      trajectory_found = planner.FindLowestCostTrajectory(
          traj, trajectories, speed_cost, _comp_time,
          samplingParameters, current_goal_picture, 
          estState.att * _depthCamAtt, estState.pos); 
    }
    if (trajectory_found) {
      _traj = std::make_shared<RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator>(traj);
      _planner_started = true;

      // Record the rotation between the trajectory and the world frame
      _trajAtt = estState.att * _depthCamAtt;
      _trajOffset = estState.pos;
      _trajResetTime = planningStartTime;
      //ROS_INFO("Generated: %d", planner.GetNumTrajectoriesGenerated());

      //if the end of a trajectory can already reach the target, stop trajectory generation
      Vec3 traj_end_pos = _trajAtt * _traj->GetPosition(traj.GetFinalTime()) + _trajOffset;
      
      //only check if x and y positions are satisfied
      if (_flightStage == StageFlight && ((traj_end_pos - Vec3(_goal_posx, _goal_posy, traj_end_pos.z)).GetNorm2() <= _pos_tolerance) || traj_end_pos.x > _goal_posx){
        _execute_last_traj = true;
        ROS_INFO("Stopped depth image callback. Executing the last trajectory.");
        ROS_INFO_STREAM("Last trajectory's end: " << traj_end_pos.x << " " << traj_end_pos.y << " " << traj_end_pos.z);
      }
    } else{
      ROS_INFO("Trajectory not found.");
    }

    publishPlannerDiagnostics(planner, traj.GetTrajectory(), vel_picture, acc_picture, g_picture,
                              msg->header.seq, trajectory_found, speed_cost, perception_cost);

    if (_execute_last_traj){
      _subDepthImages.shutdown();
    }
  }
}

void RappidsPerceptionAware::Run(bool shouldStart, bool shouldStop) {
  rappids_planner::drone_status drone_status_msg;
  drone_status_msg.header.stamp = ros::Time::now();
  drone_status_msg.flight_stage = _flightStage;
  _pub_flight_stage.publish(drone_status_msg);

  // Get estimate
  VIOStateEstimator::VIOEstimatedState estState = _est->GetPrediction(
      _systemLatencyTime);
  PublishEstimate(estState);

  // Create radio command
  hiperlab_rostools::radio_command cmdMsg;
  uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];

  // Check for flight stage change
  bool stageChange = _flightStage != _lastFlightStage;
  if (stageChange) {
    _lastFlightStage = _flightStage;
    _stateStartTime = ros::Time::now();
  }

  WaypointWithTime current_waypoint;
  
  switch (_flightStage) {
    case StageWaitForStart:
      if (stageChange) {
        ROS_INFO("Wait for start.");
      }
      if (shouldStart){
        _flightStage = StageSpoolUp;
      }
      break;
    
    case StageSpoolUp:
      if (stageChange) {
        ROS_INFO("Spooling up motors.");
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      {
        double const motorSpoolUpTime = 0.5;  //[s]
        double const spoolUpThrustByWeight = 0.25;  //[]

        _est->SetPredictedValues(Vec3d(0, 0, 0), Vec3d(0, 0, 0));
        double cmdThrust = 9.81 * spoolUpThrustByWeight;
        Vec3d cmdAngVel(0, 0, 0);
        RadioTypes::RadioMessageDecoded::CreateRatesCommand(0, float(cmdThrust),
                                                            Vec3f(cmdAngVel),
                                                            rawMsg);
        cmdMsg.debugtype = RadioTypes::externalRatesCmd;
        cmdMsg.debugvals[0] = float(cmdThrust);
        cmdMsg.debugvals[1] = float(cmdAngVel.x);
        cmdMsg.debugvals[2] = float(cmdAngVel.y);
        cmdMsg.debugvals[3] = float(cmdAngVel.z);
        for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE;
            i++) {
          cmdMsg.raw[i] = rawMsg[i];
        }

        if ((ros::Time::now() - _stateStartTime).toSec() > motorSpoolUpTime) {
          _flightStage = StageTakeoff;
        }
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }
      break;

    case StageTakeoff:
      if (stageChange){
        ROS_INFO("Taking off");
        Vec3 desired_direction_odom = Vec3(_goal_posx,_goal_posy,_goal_posz) - estState.pos;    
        // atan2 to return -pi to pi
        _desired_yaw = atan2(desired_direction_odom.y, desired_direction_odom.x);  
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      {
        //take off at 0.5 m/s
        double const takeOffTime = _goal_posz * 2.0; //[s]
        double frac = (ros::Time::now() - _stateStartTime).toSec()
            / takeOffTime;
        if (frac <= 1.0){
          current_waypoint = WaypointWithTime(0, Vec3(0, 0, frac*_goal_posz), Vec3(0, 0, 0), 
                                        Vec3(0, 0, 0), frac*_desired_yaw, 0);
        } else if (frac <= 2.0){
          current_waypoint = WaypointWithTime(0, Vec3(0, 0, _goal_posz), Vec3(0, 0, 0), 
                                        Vec3(0, 0, 0), _desired_yaw, 0);
        } else {
          _flightStage = StageFlight;
          current_waypoint = WaypointWithTime(0, Vec3(0, 0, _goal_posz), Vec3(0, 0, 0), 
                                        Vec3(0, 0, 0), _desired_yaw, 0);
        }
        cmdPos.x = current_waypoint.position(0,0);
        cmdPos.y = current_waypoint.position(1,0);
        cmdPos.z = current_waypoint.position(2,0);
        cmdMsg = RunControllerAndUpdateEstimator(estState, cmdPos,
                                          Vec3d(0, 0, 0),
                                          Vec3d(0, 0, 0));

      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }
      break;

    case StageFlight:
      if (stageChange){
        ROS_INFO("Starting rappids flight!");
      }
      
      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }

      if (shouldStop) {
        _flightStage = StageLanding;
      }

      {
        // if (!_goal_reached && isGoalReached()){
        //   _goal_reached = true;
        //   ROS_INFO("Goal reached!");
        // }

        if (_planner_started) {
          if(_goal_reached){
            //After arrived at the goal, hover for "hoverTime", then land.
            double const hoverTime = 3.0; //[s]
            if ((ros::Time::now() - _stateStartTime).toSec() > hoverTime){
              _flightStage = StageLanding;
              _pos_before_landing = estSate.pos;
            }
            current_waypoint = WaypointWithTime(0, estState.pos, Vec3(0,0,0), 
                                          Vec3(0,0,0), _desired_yaw, 0);
          }else{
            double traj_exec_time = (ros::Time::now() - _trajResetTime).toSec();
            double duration = _traj->GetFinalTime();
            Vec3 traj_end_pos = _trajAtt * _traj->GetPosition(duration) + _trajOffset;

            if (traj_exec_time <= duration){
              //in the picture's frame
              Vec3 trajPos = _traj->GetPosition(traj_exec_time);
              Vec3 trajVel = _traj->GetVelocity(traj_exec_time);
              Vec3 trajAcc = _traj->GetAcceleration(traj_exec_time);

              //convert to the odometry's frame
              Vec3 refPos = _trajAtt * trajPos + _trajOffset;
              Vec3 refVel = _trajAtt * trajVel;
              Vec3 refAcc = _trajAtt * trajAcc;
              
              //fly with fixed yaw when getting close to the goal
              if(!_execute_last_traj){
                // in odometry's (world) Frame
                Vec3 desired_direction_odom = Vec3(_goal_posx,_goal_posy,_goal_posz) - refPos;    
                // atan2 to return -pi to pi
                _desired_yaw = atan2(desired_direction_odom.y, desired_direction_odom.x);  
              }

              current_waypoint = WaypointWithTime(0, refPos, refVel, refAcc, _desired_yaw, 0);
            } else{
              //when trajectory finishes publication
              current_waypoint = WaypointWithTime(0, traj_end_pos, Vec3(0,0,0), Vec3(0,0,0), _desired_yaw, 0);
              if (!_goal_reached && _execute_last_traj){
                _goal_reached = true;
                ROS_INFO("Goal reached!");
              }
            }
          }
          visualizeControl();
        } else{
          current_waypoint = WaypointWithTime(0, estState.pos, Vec3(0,0,0), Vec3(0,0,0), _desired_yaw, 0);
          ROS_INFO("Planner not started yet! Hovering at current position!");
        }
        cmdPos.x = current_waypoint.position(0,0);
        cmdPos.y = current_waypoint.position(1,0);
        cmdPos.z = current_waypoint.position(2,0);
        cmdMsg = RunControllerAndUpdateEstimator(estState, cmdPos,
                                          Vec3d(0, 0, 0),
                                          Vec3d(0, 0, 0));
      }
      break;

    case StageLanding:
      if (stageChange){
        ROS_INFO("Landing");
      }
      {
        double const LANDING_SPEED = 0.1;
        _finalPosition = estState.pos;
        Vec3d cmdPos = _finalPosition
            + (ros::Time::now() - _stateStartTime).toSec() * Vec3d(0, 0, -LANDING_SPEED);
        if (cmdPos.z < -0.2) {
          _flightStage = StageComplete;
        }
        cmdMsg = RunControllerAndUpdateEstimator(estState, cmdPos,
                                                 Vec3d(0, 0, -LANDING_SPEED),
                                                 Vec3d(0, 0, 0));

      }
      break;

    case StageComplete:
      ROS_INFO("Flight finished. Exit off board flight.");
      _flightFinished = true;
      break;
  }

}

void RappidsPerceptionAware::publishReferenceCmd(const WaypointWithTime& current_waypoint){
    _lastcmdPos.x = current_waypoint.position(0,0);
    _lastcmdPos.y = current_waypoint.position(1,0);
    _lastcmdPos.z = current_waypoint.position(2,0);    
    _lastcmdVel.x = current_waypoint.velocity(0,0);
    _lastcmdVel.y = current_waypoint.velocity(1,0);
    _lastcmdVel.z = current_waypoint.velocity(2,0);
    _lastcmdAcc.x = current_waypoint.acceleration(0,0);
    _lastcmdAcc.y = current_waypoint.acceleration(1,0);
    _lastcmdAcc.z = current_waypoint.acceleration(2,0);
  if(_is_sim){
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = current_waypoint.position;
    trajectory_point.velocity_W = current_waypoint.velocity;
    trajectory_point.acceleration_W = current_waypoint.acceleration;
    trajectory_point.setFromYaw(current_waypoint.yaw);
    trajectory_point.setFromYawRate(current_waypoint.yawrate);
    trajectory_point.time_from_start_ns = static_cast<int64_t>(current_waypoint.waiting_time * kNanoSecondsInSecond);

    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_msg.header.frame_id = "world";
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, "base_link", &trajectory_msg);

    _ref_pub.publish(trajectory_msg);
  }else{
    mavros_msgs::PositionTarget cmd;
    cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    cmd.position.x = current_waypoint.position(0,0);
    cmd.position.y = current_waypoint.position(1,0);
    cmd.position.z = current_waypoint.position(2,0);
    
    //ROS_INFO_STREAM("Current waypoint1: " << cmd.position.x << " " << cmd.position.y << " ");

    cmd.velocity.x = current_waypoint.velocity(0,0);
    cmd.velocity.y = current_waypoint.velocity(1,0);
    cmd.velocity.z = current_waypoint.velocity(2,0);

    cmd.acceleration_or_force.x = current_waypoint.acceleration(0,0);
    cmd.acceleration_or_force.y = current_waypoint.acceleration(1,0);
    cmd.acceleration_or_force.z = current_waypoint.acceleration(2,0);

    cmd.yaw = current_waypoint.yaw;
    cmd.yaw_rate = current_waypoint.yawrate;

    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "odom";
    _ref_pub.publish(cmd);      
  }
}

 // should not use _traj* variables, which is only for found collision-free trajectories
void RappidsPerceptionAware::publishPlannerDiagnostics(DepthImagePlanner& planner, 
                                 const CommonMath::Trajectory& trajectory_parameters, const Vec3& vel_pic, 
                                 const Vec3& acc_pic, const Vec3& g_pic, const uint64& traj_seq,
                                 const bool& traj_found, const double& speed_cost, 
                                 const double& perc_cost)   
{
  // ROS_INFO("%6d collision-free / %6d pyramids / %6d velocity / %6d "
  //       "feasible / %6d low cost / %6d generated",
  //       planner.GetNumCollisionFree(), planner.GetNumPyramids(),
  //       planner.GetNumVelocityChecks(),
  //       planner.GetNumCollisionChecks(),
  //       planner.GetNumCostChecks(),
  //       planner.GetNumTrajectoriesGenerated());  

  rappids_planner::planner_diagnostics planner_msg = {};
  planner_msg.header.stamp = ros::Time::now();

  // input
  planner_msg.random_seed = planner.GetRandomSeed();
  planner_msg.velocity_picture = toGeometricMsgVec3(vel_pic);
  planner_msg.acceleration_picture = toGeometricMsgVec3(acc_pic);
  planner_msg.gravity_picture = toGeometricMsgVec3(g_pic);
  planner_msg.goal_odom = toGeometricMsgVec3(Vec3(_goal_posx, _goal_posy, _goal_posz));
  planner_msg.flight_stage = _flightStage;

  // output
  planner_msg.trajectory_id = traj_seq;

  planner_msg.trajectory_found = traj_found;
  planner_msg.speed_cost = speed_cost;
  planner_msg.perception_cost = perc_cost;
  planner_msg.NumCollisionFree = planner.GetNumCollisionFree();
  planner_msg.NumPyramids = planner.GetNumPyramids();
  planner_msg.NumVelocityChecks = planner.GetNumVelocityChecks();
  planner_msg.NumCollisionChecks = planner.GetNumCollisionChecks();
  planner_msg.NumCostChecks = planner.GetNumCostChecks();
  planner_msg.NumTrajectoriesGenerated = planner.GetNumTrajectoriesGenerated();

  //trajectory coeffs (in the picture frame)
  if (traj_found) {
  planner_msg.coeff0 = toGeometricMsgVec3(trajectory_parameters[0]);
  planner_msg.coeff1 = toGeometricMsgVec3(trajectory_parameters[1]);
  planner_msg.coeff2 = toGeometricMsgVec3(trajectory_parameters[2]);
  planner_msg.coeff3 = toGeometricMsgVec3(trajectory_parameters[3]);
  planner_msg.coeff4 = toGeometricMsgVec3(trajectory_parameters[4]);
  planner_msg.coeff5 = toGeometricMsgVec3(trajectory_parameters[5]);
  planner_msg.duration = ros::Time(trajectory_parameters.GetEndTime());

  planner_msg.trajectory_reset_time = _trajResetTime;
  planner_msg.trajectory_transform.translation = toGeometricMsgVec3(_trajOffset);
  planner_msg.trajectory_transform.rotation = toGeometricMsgQuaternion(_trajAtt);
  }
  _pub_planner_diagnotics.publish(planner_msg);
}

void RappidsPerceptionAware::visualizeControl() {    
    
    double sampling_interval = 0.1;

    // create line
    visualization_msgs::Marker traj_marker;
    traj_marker.header = _odom_header;
    traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
    traj_marker.action = visualization_msgs::Marker::ADD;
    traj_marker.id = 0;
    traj_marker.color.a = 0.7;
    traj_marker.color.r = 0.0;
    traj_marker.color.g = 0.0;
    traj_marker.color.b = 1.0;
    traj_marker.scale.x = 0.05;
    traj_marker.pose.orientation.w = 1.0;

    for (float time = 0; time <= _traj->GetFinalTime();
         time += sampling_interval) {
        Vec3 pos_odom_sampled =
            _trajAtt * _traj->GetPosition(time) + _trajOffset;
        geometry_msgs::Point p;
        p.x = pos_odom_sampled.x;
        p.y = pos_odom_sampled.y;
        p.z = pos_odom_sampled.z;
        traj_marker.points.push_back(p);
    }
    //push back the end position of the trajectory
    Vec3 trajectory_end = _trajAtt * _traj->GetPosition(_traj->GetFinalTime()) + _trajOffset;
    geometry_msgs::Point p;
    p.x = trajectory_end.x;
    p.y = trajectory_end.y;
    p.z = trajectory_end.z;
    traj_marker.points.push_back(p);
    _marker_pub.publish(traj_marker);

    // // current pose on the trajectory
    // auto traj_exec_time = (ros::Time::now() - _trajResetTime).toSec();
    // if (traj_exec_time > _traj->GetFinalTime()) {
    //     traj_exec_time = _traj->GetFinalTime();
    // }
    // Vec3 pos_odom_current =
    //     _trajAtt * _traj->GetPosition(traj_exec_time) + _trajOffset;
    // visualization_msgs::Marker current_pos_marker;
    // current_pos_marker.header = _odom_header;
    // current_pos_marker.type = visualization_msgs::Marker::SPHERE;
    // current_pos_marker.action = visualization_msgs::Marker::ADD;
    // current_pos_marker.id = 1;
    // current_pos_marker.color.a = 0.7;
    // current_pos_marker.color.r = 1.0;
    // current_pos_marker.color.g = 1.0;
    // current_pos_marker.color.b = 0.0;
    // current_pos_marker.scale.x = 0.668;
    // current_pos_marker.scale.y = 0.668;
    // current_pos_marker.scale.z = 0.215;
    // current_pos_marker.pose.position.x = pos_odom_current.x;
    // current_pos_marker.pose.position.y = pos_odom_current.y;
    // current_pos_marker.pose.position.z = pos_odom_current.z;
    // current_pos_marker.pose.orientation.w = 1.0f;
    // _marker_pub.publish(current_pos_marker);

    // add a sphere for the goal
    visualization_msgs::Marker goal_marker;
    goal_marker.header = _odom_header;
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.id = 2;
    goal_marker.color.a = 0.7;
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;
    // radius is the goal tolerance
    // scale is the radius
    goal_marker.scale.x = _pos_tolerance*2;
    goal_marker.scale.y = _pos_tolerance*2;
    goal_marker.scale.z = _pos_tolerance*2;
    goal_marker.pose.position.x = _goal_posx;
    goal_marker.pose.position.y = _goal_posy;
    goal_marker.pose.position.z = _goal_posz;
    goal_marker.pose.orientation.w = 1.0f;
    _marker_pub.publish(goal_marker);
}
