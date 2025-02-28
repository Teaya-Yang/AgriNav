#include "ExampleVehicleStateMachine.hpp"

using namespace std;
using namespace Offboard;

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
  _useVIO = false; //Always make sure this node turns this to false.
  _cornerPosition1 = Vec3d(0, 0, 0.7);
  _cornerPosition2 = Vec3d(-0.5, 0, 0.7);
  _cornerPosition3 = Vec3d(-0.5, -0.5, 0.7);
  _cornerPosition4 = Vec3d(0, -0.5, 0.7);
  // _cornerPosition1 = Vec3d(0, 0, 1.5);
  // _cornerPosition2 = Vec3d(-0.5, 0, 1.5);
  // _cornerPosition3 = Vec3d(-0.5, -0.5, 1.5);
  // _cornerPosition4 = Vec3d(0, -0.5, 1.5);
}

void ExampleVehicleStateMachine::CallbackVIOEstimator(
    const hiperlab_rostools::LoopPoseData& msg) {
  // if (_est->GetID() == msg.vehicleID) {
    _est->UpdateWithMeasurement(
        Vec3d(msg.position.x, msg.position.y, msg.position.z),
        Rotationd(msg.m4, msg.m1, msg.m2, msg.m3));
  // }
}

void ExampleVehicleStateMachine::CallbackEstimator(
    const hiperlab_rostools::mocap_output& msg) {
  if (_est->GetID() == msg.vehicleID) {
    _est->UpdateWithMeasurement(
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


  if (_useVIO){
    _subMocap.reset(
    new ros::Subscriber(
        n.subscribe("odom_data", 1,
                    &ExampleVehicleStateMachine::CallbackVIOEstimator, this)));
  }
  else{
     _subMocap.reset(
    new ros::Subscriber(
        n.subscribe("mocap_output" + std::to_string(_id), 1,
                    &ExampleVehicleStateMachine::CallbackEstimator, this)));
  }

  _subTelemetry.reset(
      new ros::Subscriber(
          n.subscribe("telemetry" + std::to_string(_id), 1,
                      &ExampleVehicleStateMachine::CallbackTelemetry, this)));

  _pubEstimate.reset(
      new ros::Publisher(
          n.advertise<hiperlab_rostools::estimator_output>(
              "estimator" + std::to_string(_id), 1)));
  _pubCmd.reset(
      new ros::Publisher(
          n.advertise<hiperlab_rostools::radio_command>(
              "radio_command" + std::to_string(_id), 1)));

  //set up components:
  _est.reset(new MocapStateEstimator(timer, _id, systemLatencyTime));
  _systemLatencyTime = systemLatencyTime;
  _ctrl.reset(new QuadcopterController());
  _safetyNet.reset(new SafetyNet());

  _flightStage = StageWaitForStart;
  _lastFlightStage = StageComplete;
  _stageTimer.reset(new Timer(timer));

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

  // Get the current state estimate and publish to ROS
  MocapStateEstimator::MocapEstimatedState estState = _est->GetPrediction(
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

      if (shouldStop) {
        _flightStage = StageLanding;
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

        double const landingTime = 4.0;  //[s]
        double frac = _stageTimer->GetSeconds<double>() / landingTime;
        Vec3d cmdPos = (1 - frac) * _finalPosition + frac *  Vec3d(0, 0, 0.0);

        if (_stageTimer->GetSeconds<double>()>(landingTime+4.0)) {
          _flightStage = StageComplete;
        }
        cmdMsg = RunControllerAndUpdateEstimator(estState, cmdPos,
                                                 Vec3d(0, 0, 0),
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
    MocapStateEstimator::MocapEstimatedState estState) {
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
    MocapStateEstimator::MocapEstimatedState estState, Vec3d desPos,
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
