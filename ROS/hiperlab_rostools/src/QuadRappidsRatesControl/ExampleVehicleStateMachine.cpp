#include "ExampleVehicleStateMachine.hpp"

using namespace std;
using namespace Offboard;
using namespace RectangularPyramidPlanner;

ExampleVehicleStateMachine::ExampleVehicleStateMachine() {
  _name = "INVALID";
  _id = 0;

  _flightStage = StageWaitForStart;
  _lastFlightStage = StageComplete;
  _systemLatencyTime = 0;

  _vehicleIsReadyForProgramToExit = false;

  _initPosition = Vec3d(0, 0, 0);
  _finalPosition = Vec3d(0, 0, 0);
  _desiredPosition = Vec3d(0, 0, 0);
  _checkPosition = Vec3d(0, 0, 0.65);
  _desiredYawAngle = 0;
  _lastTelWarnings = 0;

  heartbeat_alive_ = false;

  _useVIO = true;

  disconnectionTime = 1.0;  //[s]

  //Rappids
  _cmdYawAngle = 0.0;
  _lastPos = Vec3d(0, 0, 0);
  _lastVel = Vec3d(0, 0, 0);
  _lastAcc = Vec3d(0, 0, 0);
  _lastYaw = 0;

  _depthScale =  10.0 / (256.0);
  _depthCamAtt = Rotationd::FromEulerYPR(90.0 * M_PI / 180.0, 0 * M_PI / 180.0,
                                         -90 * M_PI / 180.0);
  _depthImageHeight = 0;
  _depthImageWidth = 0;
  _focalLength = 0;
  _plannedTrajCount = 0;

  _trajAtt = Rotationd::Identity();
  _trajOffset = Vec3d(0, 0, 0);
  _imageReady = false;
  _startPlan = false;
  _previousThrust = 9.81;
  _physicalVehicleRadius = 0.2;
  _vehicleRadiusPlanning = 0.2;
  _minCollisionDist = 1.0;
  _plannedTrajDuration = 0;
  _lookAheadTime = 0.2;
  _goalWorld = Vec3d(0.0, 0.0, 0.0);
  _lastGoal = _goalWorld;
  _depthImageCount = 0;
  _firstTrajReady = false;
  shouldPrint = false;
  _onlyFlyFirstTraj = false;

}

void ExampleVehicleStateMachine::CallbackDepthImages(
    const sensor_msgs::ImageConstPtr &msg) {

  std_msgs::Header h = msg->header;
  ros::Time publishTime = h.stamp;
  ros::Time receiveImageTime = ros::Time::now();
  ros::Duration transMissionTime = receiveImageTime - publishTime;
  CV_Assert(msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1);
  cv::Mat depthImage = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;

  _depthImageWidth = depthImage.cols;
  _depthImageHeight = depthImage.rows;
  _focalLength = _depthImageWidth / 2.0;
  // cout << _focalLength << "\n";
  

  ros::Time finishDecoding = ros::Time::now();
  DepthImagePlanner planner(depthImage, _depthScale, _focalLength,
                            _depthImageWidth / 2.0, _depthImageHeight / 2.0,
                            _physicalVehicleRadius, _vehicleRadiusPlanning,
                            _minCollisionDist);

  planner.SetRandomSeed(_depthImageCount);  // Trajectory ID = Depth image ID (=> Random seed)

  ros::Duration decodingTime = finishDecoding - receiveImageTime;
  double offsetTime = (finishDecoding - publishTime).toSec();
  double compTime = 0.05;
  VIOStateEstimator::VIOEstimatedState estState = _est->GetPrediction(_systemLatencyTime);

  _estAtt = estState.att;
  _estPos = estState.pos;

  Vec3d vel = _depthCamAtt.Inverse() * _estAtt.Inverse() * estState.vel;
  Vec3d acc = _depthCamAtt.Inverse() * _estAtt.Inverse()
      * (Vec3d(0, 0, 1) * _previousThrust - Vec3d(0, 0, 9.81));
  //  Vec3d g = _depthCamAtt.Inverse() * Vec3d(0, 0, -9.81);
  Vec3d g = _depthCamAtt.Inverse() * _estAtt.Inverse() * Vec3d(0, 0, -9.81);

  RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator candidateTraj(
      Vec3d(0, 0, 0), vel, acc, g);

  std::vector<TrajectoryTest> trajectories;
  if (_startPlan) {
    ros::Time planningStartTime = ros::Time::now();

    if (_onlyFlyFirstTraj && _firstTrajReady) {
    return;  // Skip further planning if the first trajectory is already ready.
    }

    bool planResult = planner.FindLowestCostTrajectoryRandomCandidates(
        candidateTraj, trajectories, compTime, (void*) this,
        &ExampleVehicleStateMachine::GetTrajCostWrapper);

    if (planResult) {
      _plannedTrajCount++;
      _trajAtt = _estAtt * _depthCamAtt;
      _trajOffset = _estPos;
      _plannedTraj = std::make_shared<
          RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator>(
          candidateTraj);
      _plannedTrajDuration = _plannedTraj->GetFinalTime();
      // cout<<"Get traj for"<<_plannedTrajDuration<<"\n";
      _trajResetTime = ros::Time::now();
      _timeOnPlannedTraj->Reset();

      {
        double traj_y, traj_p, traj_r;
        _trajAtt.ToEulerYPR(traj_y, traj_p, traj_r);
        CommonMath::Trajectory trajToRecord = candidateTraj.GetTrajectory();
        std::vector<Vec3d> trajCoeffs = trajToRecord.GetCoeffs();

        // Record the trajectory
        _plannedTrajLog << _plannedTrajCount << ",";
        _plannedTrajLog << trajCoeffs[0].x << "," << trajCoeffs[0].y << ","
            << trajCoeffs[0].z << ",";
        _plannedTrajLog << trajCoeffs[1].x << "," << trajCoeffs[1].y << ","
            << trajCoeffs[1].z << ",";
        _plannedTrajLog << trajCoeffs[2].x << "," << trajCoeffs[2].y << ","
            << trajCoeffs[2].z << ",";
        _plannedTrajLog << trajCoeffs[3].x << "," << trajCoeffs[3].y << ","
            << trajCoeffs[3].z << ",";
        _plannedTrajLog << trajCoeffs[4].x << "," << trajCoeffs[4].y << ","
            << trajCoeffs[4].z << ",";
        _plannedTrajLog << trajCoeffs[5].x << "," << trajCoeffs[5].y << ","
            << trajCoeffs[5].z << ",";
        _plannedTrajLog << traj_y << "," << traj_p << "," << traj_r << ",";
        _plannedTrajLog << _trajOffset.x << "," << _trajOffset.y << ","
            << _trajOffset.z << ",";
        _plannedTrajLog << trajToRecord.GetStartTime() << ","
            << trajToRecord.GetEndTime() << "\n";
      }

      if (not _firstTrajReady) {
        _firstTrajReady = true;
      }
    }

    // planner internal logging
    // should not use _traj* variables, which is only for found collision-free trajectories
    {
      hiperlab_rostools::planner_diagnostics planner_msg = { };
      planner_msg.header.stamp = ros::Time::now();
      // input
      planner_msg.input.random_seed = planner.GetRandomSeed();
      planner_msg.input.velocity_D = Vec3dToVector3(vel);
      planner_msg.input.acceleration_D = Vec3dToVector3(acc);
      planner_msg.input.gravity_D = Vec3dToVector3(g);
      planner_msg.input.goal_W = Vec3dToVector3(_goalWorld);

      // output
      CommonMath::Trajectory trajectory_parameters =
          candidateTraj.GetTrajectory();
      planner_msg.output.trajectory_id = msg->header.seq;

      planner_msg.output.planner_statistics.trajectory_found = planResult;
      planner_msg.output.planner_statistics.NumCollisionFree = planner
          .GetNumCollisionFree();
      planner_msg.output.planner_statistics.NumPyramids =
          planner.GetNumPyramids();
      planner_msg.output.planner_statistics.NumVelocityChecks = planner
          .GetNumVelocityChecks();
      planner_msg.output.planner_statistics.NumCollisionChecks = planner
          .GetNumCollisionChecks();
      planner_msg.output.planner_statistics.NumCostChecks = planner
          .GetNumCostChecks();
      planner_msg.output.planner_statistics.NumTrajectoriesGenerated = planner
          .GetNumTrajectoriesGenerated();

      planner_msg.output.trajectory_parameters_D.coeff0 = Vec3dToVector3(
          trajectory_parameters[0]);
      planner_msg.output.trajectory_parameters_D.coeff1 = Vec3dToVector3(
          trajectory_parameters[1]);
      planner_msg.output.trajectory_parameters_D.coeff2 = Vec3dToVector3(
          trajectory_parameters[2]);
      planner_msg.output.trajectory_parameters_D.coeff3 = Vec3dToVector3(
          trajectory_parameters[3]);
      planner_msg.output.trajectory_parameters_D.coeff4 = Vec3dToVector3(
          trajectory_parameters[4]);
      planner_msg.output.trajectory_parameters_D.coeff5 = Vec3dToVector3(
          trajectory_parameters[5]);
      planner_msg.output.trajectory_parameters_D.duration = ros::Time(
          trajectory_parameters.GetEndTime());

      planner_msg.output.trajectory_reset_time = planningStartTime;
      planner_msg.output.trajectory_transform.translation = Vec3dToVector3(
          _trajOffset);
      planner_msg.output.trajectory_transform.rotation = RotationdToQuaternion(
          _trajAtt);
      _pubPlannerDiagnotics->publish(planner_msg);
    }

  }

  if (_depthImageCount < 1000) {
    _imageReceiveLog << _depthImageCount << ",";
    _imageReceiveLog << transMissionTime.toSec() << ",";
    _imageReceiveLog << decodingTime.toSec() << ",";
    _imageReceiveLog << "\n";
  } else if (_depthImageCount == 1000) {
    _imageReceiveLog.close();
  }

  if (_plannedTrajCount > 1000) {
    _plannedTrajLog.close();
  }

  _depthImageCount++;
}


void ExampleVehicleStateMachine::CallbackHeartbeat(const std_msgs::Bool::ConstPtr& msg) {
  heartbeat_alive_ = msg->data;  // Update the heartbeat status
  // ROS_INFO("Heartbeat received in callback: %d", heartbeat_alive_);
  if (heartbeat_alive_){
    _heartbeatTimer->Reset();
  }
  
}


void ExampleVehicleStateMachine::CallbackVIOEstimator(
    const nav_msgs::Odometry & msg) {
  // if (_est->GetID() == msg.vehicleID) {

  // check for NaN
// position in map/world fixed frame
    double x = msg.pose.pose.position.x;
    double y = msg.pose.pose.position.y;
    double z = msg.pose.pose.position.z;

    // orientation in map/world fixed frame
    double qw = msg.pose.pose.orientation.w;
    double qx = msg.pose.pose.orientation.x;
    double qy = msg.pose.pose.orientation.y;
    double qz = msg.pose.pose.orientation.z;

    // velocity in map/world fixed frame
    double vx = msg.twist.twist.linear.x;
    double vy = msg.twist.twist.linear.y;
    double vz = msg.twist.twist.linear.z;

    // position in map/world fixed frame
    double wx = msg.twist.twist.angular.x;
    double wy = msg.twist.twist.angular.y;
    double wz = msg.twist.twist.angular.z;

    // check pose data
    if (std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(qw) || std::isnan(qx) || std::isnan(qy) || std::isnan(qz)){
      std::cerr<<"Warning: nan detected in pose, skip this update\n";
      return;
    }

    // check twist data, only update with pose if nan detected
    if (std::isnan(vx) || std::isnan(vy) || std::isnan(vz) || std::isnan(wx) || std::isnan(wy) || std::isnan(wz)){
      std::cerr<<"Warning: nan detected in twist, update with pose only\n";
      _est->UpdateWithMeasurement(Vec3d(x, y, z), Rotationd(qw,  qx,  qy,  qz));
      return;
    }

    // all data are valid, update with full estimation
    _est->UpdateWithVIO(
      Vec3d(x, y, z), Rotationd(qw,  qx,  qy,  qz), Vec3d(vx, vy, vz), Vec3d(wx, wy, wz));
  // }
}

// void ExampleVehicleStateMachine::CallbackEstimator(
//     const hiperlab_rostools::mocap_output& msg) {
//   if (_est->GetID() == msg.vehicleID) {
//     _est->UpdateWithMeasurement(
//         Vec3d(msg.posx, msg.posy, msg.posz),
//         Rotationd(msg.attq0, msg.attq1, msg.attq2, msg.attq3));
//   }
// }

void ExampleVehicleStateMachine::CallbackTelemetry(
    const hiperlab_rostools::telemetry& msg) {
  _lastTelWarnings = msg.warnings;
}

void ExampleVehicleStateMachine::Initialize(int id, std::string name,
                                            ros::NodeHandle &n,
                                            BaseTimer* timer,
                                            double systemLatencyTime) {
  _id = id;
  stringstream ss;
  ss << "[" << name << " (" << _id << ")]: ";
  _name = ss.str();

  //set up networking stuff:

  image_transport::ImageTransport _it(n); // Depth image transporter

  _subDepthImages = _it.subscribe(
      "/d455/depth/image_rect_raw", 1, &ExampleVehicleStateMachine::CallbackDepthImages, this); 

  if (_useVIO){
    _subVIO.reset(
    new ros::Subscriber(
        n.subscribe("odom_data", 1,
                    &ExampleVehicleStateMachine::CallbackVIOEstimator, this)));
  }
  // else{
  //    _subMocap.reset(
  //   new ros::Subscriber(
  //       n.subscribe("mocap_output" + std::to_string(_id), 1,
  //                   &ExampleVehicleStateMachine::CallbackEstimator, this)));
  // }

  _subTelemetry.reset(
      new ros::Subscriber(
          n.subscribe("telemetry" + std::to_string(_id), 1,
                      &ExampleVehicleStateMachine::CallbackTelemetry, this)));

  _subHeartbeat.reset(
      new ros::Subscriber(
          n.subscribe("heartbeat_signal", 10,
                       &ExampleVehicleStateMachine::CallbackHeartbeat, this)));

  _pubEstimate.reset(
      new ros::Publisher(
          n.advertise<hiperlab_rostools::estimator_output>(
              "estimator" + std::to_string(_id), 1)));
  _pubCmd.reset(
      new ros::Publisher(
          n.advertise<hiperlab_rostools::radio_command>(
              "radio_command" + std::to_string(_id), 1)));
  
  _pubPlannerDiagnotics.reset(
      new ros::Publisher(
          n.advertise < hiperlab_rostools::planner_diagnostics
              > ("planner_diagnostics", 10)));


  //set up components:
  _est.reset(new VIOStateEstimator(timer, _id, systemLatencyTime));
  _systemLatencyTime = systemLatencyTime;
  _ctrl.reset(new QuadcopterController());
  _safetyNet.reset(new SafetyNet());
  // _safetyNet->SetSafeCorners(Vec3d(-100,-100,-2.0), Vec3d(100,100,20), 0.0);
  _safetyNet->SetSafeCorners(Vec3d(-20.0, -20.0, -20.0), Vec3d(+20.0, +20.0, 20.0), 0.0);
  _flightStage = StageWaitForStart;
  _lastFlightStage = StageComplete;
  _stageTimer.reset(new Timer(timer));
  _heartbeatTimer.reset(new Timer(timer));
  _timeOnPlannedTraj.reset(new Timer(timer));


  // Initialize control parameters
  Onboard::QuadcopterConstants::QuadcopterType quadcopterType =
      Onboard::QuadcopterConstants::GetVehicleTypeFromID(_id);
  Onboard::QuadcopterConstants vehConsts(quadcopterType);
  _ctrl->SetParameters(vehConsts.posControl_natFreq,
                       vehConsts.posControl_damping,
                       vehConsts.attControl_timeConst_xy,
                       vehConsts.attControl_timeConst_z);
  double armLength = vehConsts.armLength;
  _physicalVehicleRadius = armLength * 2.0;
  _vehicleRadiusPlanning = armLength * 2.0 * 1.5;
  _depthCamAtt = Rotationd::FromEulerYPR(90.0 * M_PI / 180.0, 0 * M_PI / 180.0,
                                         -90 * M_PI / 180.0);
  _goalWorld = Vec3d(-10.0, 0.0, 1.5);
  cout << _name << "Created. Goal at (" << _goalWorld.x << "," << _goalWorld.y << "," << _goalWorld.z << ")\n";
}

void ExampleVehicleStateMachine::Run(bool shouldStart, bool shouldStop, bool jsButtonGreen, bool jsButtonBlue, bool jsButtonYellow) {

  // Check for flight stage change
  bool stageChange = _flightStage != _lastFlightStage;
  _lastFlightStage = _flightStage;
  if (stageChange) {
    _stageTimer->Reset();
  }


  if (_heartbeatTimer->GetSeconds<double>() > disconnectionTime){
    ROS_WARN("Heartbeat lost! Switching to EMERGENCY stage.");
    _flightStage = StageCrashing;
  }


  // Get the current state estimate and publish to ROS
  VIOStateEstimator::VIOEstimatedState estState = _est->GetPrediction(
      _systemLatencyTime);
  _safetyNet->UpdateWithEstimator(estState,
                                  _est->GetTimeSinceLastGoodMeasurement());
  PublishEstimate(estState);

  // Create radio message depending on the current flight stage
  hiperlab_rostools::radio_command cmdMsg;
  uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];
  switch (_flightStage) {
    case StageWaitForStart:
      if (stageChange) {
        cout << _name << "Waiting for start signal.\n";
      }
      if (shouldStart) {
        _flightStage = StageSpoolUp;
      }
      break;

    case StageSpoolUp:
      if (stageChange) {
        cout << _name << "Spooling up motors.\n";
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

        if (_stageTimer->GetSeconds<double>() > motorSpoolUpTime) {
          _flightStage = StageTakeoff;
        }
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }
      break;

    case StageTakeoff:
      if (stageChange) {
        cout << _name << "Taking off.\n";
        _initPosition = estState.pos;
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      {
        double const takeOffTime = 10.0;  //[s]
        double frac = _stageTimer->GetSeconds<double>() / takeOffTime;
        if (frac >= 1.0) {
          _flightStage = StageVIOCheck;
          frac = 1.0;
        }
        Vec3d cmdPos = (1 - frac) * _initPosition + frac * _desiredPosition;
        cmdMsg = RunControllerAndUpdateEstimator(estState, cmdPos,
                                                 Vec3d(0, 0, 0),
                                                 Vec3d(0, 0, 0));
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }
      break;

    case StageVIOCheck:
      if (stageChange) {
        cout << _name << "Entering checking stage.\n";
        shouldPrint = true;
      }
      if (_stageTimer -> GetSeconds<double>() > 0.5d && shouldPrint) {
        cout << _name << "Current estimate is: (" << estState.pos.x << "," << estState.pos.y << "," << estState.pos.z << "). Proceed?\n";
        shouldPrint = false;
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }
      
      if (jsButtonGreen) {
        _flightStage = StageFlight;
        _startPlan = true;
      }

      cmdMsg = RunControllerAndUpdateEstimator(estState, _desiredPosition,
                                               Vec3d(0, 0, 0), Vec3d(0, 0, 0));

      if (shouldStop) {
        _flightStage = StageLanding;
      }

      break;
    
    // case StageContinueTakeoff:
    //   if (stageChange) {
    //     cout << _name << "Continuing.\n";
    //     _initPosition = estState.pos;
    //   }

    //   if (!_safetyNet->GetIsSafe()) {
    //     _flightStage = StageEmergency;
    //   }

    //   {
    //     double const takeOffTime = 4.0;  //[s]
    //     double frac = _stageTimer->GetSeconds<double>() / takeOffTime;
    //     if (frac >= 1.0) {
    //       _flightStage = StageHover;
    //       frac = 1.0;
    //     }
    //     Vec3d cmdPos = (1 - frac) * _initPosition + frac * _desiredPosition;
    //     cmdMsg = RunControllerAndUpdateEstimator(estState, cmdPos,
    //                                              Vec3d(0, 0, 0),
    //                                              Vec3d(0, 0, 0));
    //   }

    //   if (HaveLowBattery()) {
    //     printf("LOW BATTERY!\n");
    //     _flightStage = StageLanding;
    //   }
    //   break;


    case StageHover:
      if (stageChange) {
        cout << _name << "Entering hover stage.\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }
      
      if (jsButtonGreen) {
        _flightStage = StageFlight;
        _startPlan = true;
      }

      cmdMsg = RunControllerAndUpdateEstimator(estState, _desiredPosition,
                                               Vec3d(0, 0, 0), Vec3d(0, 0, 0));

      if (shouldStop) {
        _flightStage = StageLanding;
      }

      break;

    case StageFlight:
      if (stageChange) {
        cout << _name << "Entering flight stage (RAPPIDS).\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }

      if (not _firstTrajReady) {
        cmdMsg = RunControllerAndUpdateEstimator(estState, _desiredPosition,
                                                 Vec3d(0, 0, 0),
                                                 Vec3d(0, 0, 0));
      } else {
        static Vec3d refPos;
        Vec3d refVel;
        Vec3d refAcc;
        double refThrust;
        Vec3d refAngVel;

        Rotationf cmdAttOutput;
        Vec3d cmdAngVelOutput;
        double cmdThrustOutput;

        double t;

        t = (ros::Time::now() - _trajResetTime).toSec();
        static Vec3d hold_pos = _estPos;  // current est pose when it's enabled
        if (t > _plannedTraj->GetFinalTime()) {
          //Mode 1: When the vehicle reached its final goal point (Desired action: Hovering)
          // POSITION CONTROL HOVERING
          refPos = hold_pos;
          refVel = Vec3d(0, 0, 0);
          refAcc = Vec3d(0, 0, 0);
          refAngVel = Vec3d(0, 0, 0);
          refThrust = 9.80665;
          // cout << "Holding..." <<"\n";
        } else {
          //Mode 2: Vehicle tracking collision-avoidance trajectory
          hold_pos = _estPos;
          // RATE CONTROL RAPPIDS
          // in Depth Camera Frame
          // Vec3d trajPos = _plannedTraj->GetPosition(t+_lookAheadTime);
          Vec3d trajPos = _plannedTraj->GetPosition(t);
          Vec3d trajVel = _plannedTraj->GetVelocity(t);
          Vec3d trajAcc = _plannedTraj->GetAcceleration(t);

          // in World Frame
          refPos = _trajAtt * trajPos + _trajOffset;  // R * x + p
          refVel = _trajAtt * trajVel;
          refAcc = _trajAtt * trajAcc;

          const double loopRate = 100;
          // TODO Depth to Body (_est_att.Inverse() * _trajAtt) can be hard-coded
          refAngVel = _estAtt.Inverse() * _trajAtt
              * _plannedTraj->GetOmega(t, 1.0 / loopRate);  // in Body Frame
          refThrust = _plannedTraj->GetThrust(t);
        
        // This block changes the desired yaw to towards the goal point
        //  Vec3d desired_direction_w = _goalWorld - _estPos;    // in World Frame
        //  _desiredYawAngle = atan2(desired_direction_w.y,
        //                           desired_direction_w.x);  // atan2 to return -pi to pi
        }
        // if ((refPos - hold_pos).GetNorm2() > 0.5){
        //   cout << "Trajectory is hard, hold instead\n";
        //   refPos = hold_pos;
        //   refVel = Vec3d(0, 0, 0);
        //   refAcc = Vec3d(0, 0, 0);
        // }
        cmdMsg = RunTrackingControllerAndUpdateEstimator(estState, refPos,
                                                         refVel, refAcc,
                                                         refThrust, refAngVel,
                                                         cmdThrustOutput,
                                                         cmdAttOutput,
                                                         cmdAngVelOutput);

        // cmdMsg = RunControllerAndUpdateEstimator(estState, refPos,refVel,refAcc);

      }

      if (shouldStop) {
        _flightStage = StageLanding;
      }
      
      {
        // Changing waypoint
        Vec3d desired_direction_w = _goalWorld - _estPos;
        float dist_to_goal = desired_direction_w.GetNorm2();

        // What to do if waypoint is reached
        if (dist_to_goal < 0.6 || _estPos.x < _goalWorld.x) {
          // If goal is reached
          cout << _name << "Final goal reached.\n";
          _flightStage = StageLanding;
        }
      }

      break;

    case StageLanding:
      if (stageChange) {
        cout << _name << "Starting landing.\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      {
        double const LANDING_SPEED = 0.5;  //m/s
        _finalPosition = estState.pos;
        Vec3d cmdPos = _finalPosition
            + _stageTimer->GetSeconds<double>() * Vec3d(0, 0, -LANDING_SPEED);
        if (cmdPos.z < -0) {
          _flightStage = StageComplete;
        }
        cmdMsg = RunControllerAndUpdateEstimator(estState, cmdPos,
                                                 Vec3d(0, 0, -LANDING_SPEED),
                                                 Vec3d(0, 0, 0));
      }
      break;

    case StageCrashLanding:
      if (stageChange) {
        cout << _name << "Crash Landing.\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      {
        double const crashLandingTime = 2.5;  //[s]
        double const ThrustRatio = 0.7;  //[]

        double cmdThrust = 9.81 * ThrustRatio;
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

        if (_stageTimer->GetSeconds<double>() > crashLandingTime) {
          _flightStage = StageComplete;
        }
      }
      break;

    case StageCrashing:
      if (stageChange) {
        cout << _name << "Emergency crashing.\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      {
        double const crashTime = 4.0;  //[s]
        double const ThrustRatio = 0.8;  //[]

        double cmdThrust = 9.81 * ThrustRatio;
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

        if (_stageTimer->GetSeconds<double>() > crashTime) {
          _flightStage = StageEmergency;
        }
      }
      break;


    case StageComplete:
      if (stageChange) {
        cout << _name << "Landing complete. Idling.\n";
      }

      {
        _est->SetPredictedValues(Vec3d(0, 0, 0), Vec3d(0, 0, 0));

        //Publish the commands:
        RadioTypes::RadioMessageDecoded::CreateIdleCommand(0, rawMsg);
        for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE;
            i++) {
          cmdMsg.raw[i] = rawMsg[i];
        }
        cmdMsg.debugtype = RadioTypes::idleCommand;
      }

      if (_stageTimer->GetSeconds<double>() > 1.0) {
        cout << _name << "Exiting.\n";
        _vehicleIsReadyForProgramToExit = true;
      }
      break;

    default:
    case StageEmergency:
      if (stageChange) {
        cout << _name << "Emergency stage! Safety net = <"
             << _safetyNet->GetStatusString() << ">.\n";
      }

      RadioTypes::RadioMessageDecoded::CreateKillCommand(0, rawMsg);
      for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE;
          i++) {
        cmdMsg.raw[i] = rawMsg[i];
      }
      cmdMsg.debugtype = RadioTypes::emergencyKill;
      break;
  }

  cmdMsg.header.stamp = ros::Time::now();

  _pubCmd->publish(cmdMsg);

}

void ExampleVehicleStateMachine::PublishEstimate(
    VIOStateEstimator::VIOEstimatedState estState) {
  // Publish the current state estimate

  hiperlab_rostools::estimator_output estOutMsg;
  estOutMsg.header.stamp = ros::Time::now();
  estOutMsg.vehicleID = _est->GetID();

  estOutMsg.posx = estState.pos.x;
  estOutMsg.posy = estState.pos.y;
  estOutMsg.posz = estState.pos.z;

  estOutMsg.velx = estState.vel.x;
  estOutMsg.vely = estState.vel.y;
  estOutMsg.velz = estState.vel.z;

  estOutMsg.attq0 = estState.att[0];
  estOutMsg.attq1 = estState.att[1];
  estOutMsg.attq2 = estState.att[2];
  estOutMsg.attq3 = estState.att[3];

  estOutMsg.attyaw = estState.att.ToEulerYPR().x;
  estOutMsg.attpitch = estState.att.ToEulerYPR().y;
  estOutMsg.attroll = estState.att.ToEulerYPR().z;

  estOutMsg.angvelx = estState.angVel.x;
  estOutMsg.angvely = estState.angVel.y;
  estOutMsg.angvelz = estState.angVel.z;

  _pubEstimate->publish(estOutMsg);
}

hiperlab_rostools::radio_command ExampleVehicleStateMachine::RunControllerAndUpdateEstimator(
    VIOStateEstimator::VIOEstimatedState estState, Vec3d desPos,
    Vec3d desVel, Vec3d desAcc) {
  // Run the rates controller
  Vec3d cmdAngVel;
  double cmdThrust;
  _ctrl->Run(estState.pos, estState.vel, estState.att, desPos, desVel, desAcc,
             _desiredYawAngle, cmdAngVel, cmdThrust);

  //Tell the estimator:
  _est->SetPredictedValues(
      cmdAngVel,
      (estState.att * Vec3d(0, 0, 1) * cmdThrust - Vec3d(0, 0, 9.81)));

  //Create and return the radio command
  uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];
  RadioTypes::RadioMessageDecoded::CreateRatesCommand(0, float(cmdThrust),
                                                      Vec3f(cmdAngVel), rawMsg);
  hiperlab_rostools::radio_command cmdMsg;
  for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++) {
    cmdMsg.raw[i] = rawMsg[i];
  }
  cmdMsg.debugvals[0] = float(cmdThrust);
  cmdMsg.debugvals[1] = float(cmdAngVel.x);
  cmdMsg.debugvals[2] = float(cmdAngVel.y);
  cmdMsg.debugvals[3] = float(cmdAngVel.z);
  cmdMsg.debugtype = RadioTypes::externalRatesCmd;

  return cmdMsg;
}

hiperlab_rostools::radio_command ExampleVehicleStateMachine::RunTrackingControllerAndUpdateEstimator(
    VIOStateEstimator::VIOEstimatedState estState, Vec3d refPos,
    Vec3d refVel, Vec3d refAcc, double refThrust, Vec3d refAngVel,
    double &cmdThrustOutput, Rotationf &cmdAttOutput, Vec3d &cmdAngVelOutput) {

// Run the rates controller
  Vec3d cmdAngVel;
  double cmdThrust;
  Rotationf cmdAtt;

  _ctrl->RunTracking(estState.pos, estState.vel, estState.att, refPos, refVel,
                     refAcc, _desiredYawAngle, refThrust, refAngVel, cmdAngVel,
                     cmdThrust, cmdAtt);

  cmdThrustOutput = cmdThrust;
  cmdAttOutput = cmdAtt;
  cmdAngVelOutput = cmdAngVel;

//Create and return the radio command
  uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];
  RadioTypes::RadioMessageDecoded::CreateRatesCommand(0, float(cmdThrust),
                                                      Vec3f(cmdAngVel), rawMsg);
  hiperlab_rostools::radio_command cmdMsg;
  for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++) {
    cmdMsg.raw[i] = rawMsg[i];
  }
  cmdMsg.debugvals[0] = float(cmdThrust);
  cmdMsg.debugvals[1] = float(cmdAngVel.x);
  cmdMsg.debugvals[2] = float(cmdAngVel.y);
  cmdMsg.debugvals[3] = float(cmdAngVel.z);
  cmdMsg.debugtype = RadioTypes::externalRatesCmd;
  return cmdMsg;
  if (!std::isnan(cmdThrust)) {
    // do not update when NaN
    _previousThrust = cmdThrust;  //used for the accelermation term of init state for generating new traj
  }

}

