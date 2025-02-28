#include "ExampleVehicleStateMachine.hpp"

using namespace std;
using namespace Offboard;
#include <std_msgs/Bool.h> 


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
  _desiredYawAngle = 0;
  _lastTelWarnings = 0;

  heartbeat_alive_ = false;

  _useVIO = true;
  _cornerPosition1 = Vec3d(0, 0, 0.7);
  _cornerPosition2 = Vec3d(-0.5, 0, 0.7);
  _cornerPosition3 = Vec3d(-0.5, -0.5, 0.7);
  _cornerPosition4 = Vec3d(0, -0.5, 0.7);
  // _cornerPosition1 = Vec3d(0, 0, 1.5);
  // _cornerPosition2 = Vec3d(-0.5, 0, 1.5);
  // _cornerPosition3 = Vec3d(-0.5, -0.5, 1.5);
  // _cornerPosition4 = Vec3d(0, -0.5, 1.5);
  disconnectionTime = 1.0;  //[s]

}

void ExampleVehicleStateMachine::CallbackHeartbeat(const std_msgs::Bool::ConstPtr& msg) {
  heartbeat_alive_ = msg->data;  // Update the heartbeat status
  // ROS_INFO("Heartbeat received in callback: %d", heartbeat_alive_);
  if (heartbeat_alive_){
    _heartbeatTimer->Reset();
  }
  
}


void ExampleVehicleStateMachine::CallbackVIOEstimator(
    const nav_msgs::Odometry& msg) {

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

  // if (_est->GetID() == msg.vehicleID) {
    _est->UpdateWithVIO(
        Vec3d(x, y, z), Rotationd(qw,  qx,  qy,  qz), Vec3d(vx, vy, vz), Vec3d(wx, wy, wz));
  // }
}

void ExampleVehicleStateMachine::CallbackEstimator(
    const hiperlab_rostools::mocap_output& msg) {
  if (_mocap->GetID() == msg.vehicleID) {
    _mocap->UpdateWithMeasurement(
        Vec3d(msg.posx, msg.posy, msg.posz),
        Rotationd(msg.attq0, msg.attq1, msg.attq2, msg.attq3));
  }
}

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


  _subVIO.reset(
  new ros::Subscriber(
      n.subscribe("odom_data", 1,
                  &ExampleVehicleStateMachine::CallbackVIOEstimator, this)));
  
  _subMocap.reset(
  new ros::Subscriber(
      n.subscribe("mocap_output" + std::to_string(_id), 1,
                  &ExampleVehicleStateMachine::CallbackEstimator, this)));

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
  _pubMocapEstimate.reset(
      new ros::Publisher(
          n.advertise<hiperlab_rostools::estimator_output>(
              "mocap_estimator" + std::to_string(_id), 1)));
  _pubCmd.reset(
      new ros::Publisher(
          n.advertise<hiperlab_rostools::radio_command>(
              "radio_command" + std::to_string(_id), 1)));


  //set up components:
  _est.reset(new VIOStateEstimator(timer, _id, systemLatencyTime));
  _mocap.reset(new MocapStateEstimator(timer, _id, systemLatencyTime));
  _systemLatencyTime = systemLatencyTime;
  _ctrl.reset(new QuadcopterController());
  _safetyNet.reset(new SafetyNet());

  _flightStage = StageWaitForStart;
  _lastFlightStage = StageComplete;
  _stageTimer.reset(new Timer(timer));
  _heartbeatTimer.reset(new Timer(timer));


  // Initialize control parameters
  Onboard::QuadcopterConstants::QuadcopterType quadcopterType =
      Onboard::QuadcopterConstants::GetVehicleTypeFromID(_id);
  Onboard::QuadcopterConstants vehConsts(quadcopterType);
  _ctrl->SetParameters(vehConsts.posControl_natFreq,
                       vehConsts.posControl_damping,
                       vehConsts.attControl_timeConst_xy,
                       vehConsts.attControl_timeConst_z);

  cout << _name << "Created.\n";
}

void ExampleVehicleStateMachine::Run(bool shouldStart, bool shouldStop, bool jsButtonGreen, bool jsButtonBlue) {

  // Check for flight stage change
  bool stageChange = _flightStage != _lastFlightStage;
  _lastFlightStage = _flightStage;
  if (stageChange) {
    _stageTimer->Reset();
  }


  if (_heartbeatTimer->GetSeconds<double>() > disconnectionTime){
    ROS_WARN("Heartbeat lost! Switching to EMERGENCY stage.");
    _flightStage = StageEmergency;
  }


  // Get the current state estimate and publish to ROS
  VIOStateEstimator::VIOEstimatedState estState = _est->GetPrediction(
      _systemLatencyTime);
  MocapStateEstimator::MocapEstimatedState mocapState = _mocap->GetPrediction(
      _systemLatencyTime);
  _safetyNet->UpdateWithEstimator(estState,
                                  _est->GetTimeSinceLastGoodMeasurement());
  PublishEstimate(estState);
  PublishMocapEstimate(mocapState);

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
        double const takeOffTime = 4.0;  //[s]
        double frac = _stageTimer->GetSeconds<double>() / takeOffTime;
        if (frac >= 1.0) {
          _flightStage = StageFlight;
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

    case StageFlight:
      if (stageChange) {
        cout << _name << "Entering flight stage.\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }

      cmdMsg = RunControllerAndUpdateEstimator(estState, _desiredPosition,
                                               Vec3d(0, 0, 0), Vec3d(0, 0, 0));

      if (shouldStop) {
        _flightStage = StageLanding;
      }

      if (jsButtonGreen){
        _flightStage = StageSquare1;
      }
      break;

    case StageSquare1:
      if (stageChange) {
        cout << _name << "Square corner 1.\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }

      cmdMsg = RunControllerAndUpdateEstimator(estState, _cornerPosition1,
                                               Vec3d(0, 0, 0), Vec3d(0, 0, 0));

      if (shouldStop) {
        _flightStage = StageLanding;
      }

      if (jsButtonBlue){
        _flightStage = StageSquare2;
      }
      break;
    
    case StageSquare2:
      if (stageChange) {
        cout << _name << "Square corner 2.\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }

      cmdMsg = RunControllerAndUpdateEstimator(estState, _cornerPosition2,
                                               Vec3d(0, 0, 0), Vec3d(0, 0, 0));

      if (shouldStop) {
        _flightStage = StageLanding;
      }

      if (jsButtonGreen){
        _flightStage = StageSquare3;
      }
      break;

    case StageSquare3:
      if (stageChange) {
        cout << _name << "Square corner 3.\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }

      cmdMsg = RunControllerAndUpdateEstimator(estState, _cornerPosition3,
                                               Vec3d(0, 0, 0), Vec3d(0, 0, 0));

      if (shouldStop) {
        _flightStage = StageLanding;
      }

      if (jsButtonBlue){
        _flightStage = StageSquare4;
      }
      break;

    case StageSquare4:
      if (stageChange) {
        cout << _name << "Square corner 4.\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }

      cmdMsg = RunControllerAndUpdateEstimator(estState, _cornerPosition4,
                                               Vec3d(0, 0, 0), Vec3d(0, 0, 0));

      if (shouldStop) {
        _flightStage = StageLanding;
      }

      if (jsButtonGreen){
        _flightStage = StageSquare1;
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
        double const LANDING_SPEED = 0.1;  //m/s
        _finalPosition = estState.pos;
        Vec3d cmdPos = _finalPosition
            + _stageTimer->GetSeconds<double>() * Vec3d(0, 0, -LANDING_SPEED);
        if (cmdPos.z < -0.2) {
          _flightStage = StageComplete;
        }
        cmdMsg = RunControllerAndUpdateEstimator(estState, cmdPos,
                                                 Vec3d(0, 0, -LANDING_SPEED),
                                                 Vec3d(0, 0, 0));
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

void ExampleVehicleStateMachine::PublishMocapEstimate(
    MocapStateEstimator::MocapEstimatedState mocapState) {
  // Publish the current state estimate

  hiperlab_rostools::estimator_output estOutMsg;
  estOutMsg.header.stamp = ros::Time::now();
  estOutMsg.vehicleID = _mocap->GetID();

  estOutMsg.posx = mocapState.pos.x;
  estOutMsg.posy = mocapState.pos.y;
  estOutMsg.posz = mocapState.pos.z;

  estOutMsg.velx = mocapState.vel.x;
  estOutMsg.vely = mocapState.vel.y;
  estOutMsg.velz = mocapState.vel.z;

  estOutMsg.attq0 = mocapState.att[0];
  estOutMsg.attq1 = mocapState.att[1];
  estOutMsg.attq2 = mocapState.att[2];
  estOutMsg.attq3 = mocapState.att[3];

  estOutMsg.attyaw = mocapState.att.ToEulerYPR().x;
  estOutMsg.attpitch = mocapState.att.ToEulerYPR().y;
  estOutMsg.attroll = mocapState.att.ToEulerYPR().z;

  estOutMsg.angvelx = mocapState.angVel.x;
  estOutMsg.angvely = mocapState.angVel.y;
  estOutMsg.angvelz = mocapState.angVel.z;

  _pubMocapEstimate->publish(estOutMsg);
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