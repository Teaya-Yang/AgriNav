#include "VehicleStateMachine_demoThrow.hpp"

using namespace std;
using namespace Offboard;

double const RECOVER_DESCENT_SPEED = 0.5;
double const RECOVERY_TIME = 4.0;  // [s]
double const RECOVERY_HEIGHT = 3.0;  // [m]
double const THROWN_DETECTION_HEIGHT = 2.0;  // [m]
double const RETURN_TIME = 3.0f;  //[s]

VehicleStateMachine_demoThrow::VehicleStateMachine_demoThrow() {
  _name = "INVALID";
  _id = 0;

  _flightStage = StageWaitForStart;
  _lastFlightStage = StageComplete;
  _systemLatencyTime = 0;

  _vehicleIsReadyForProgramToExit = false;

  _throwReturnPos = Vec3d(0, 0, 0);
  _throwRecoverPos = Vec3d(0, 0, 0);
  _desiredYawAngle = 0;
  _stageStartCmdPos = Vec3d(0, 0, 0);
  _lastCmdPosition = Vec3d(0, 0, 0);
  _grabBeginningAtt = Rotationd::Identity();
  _lastTelWarnings = 0;
}

void VehicleStateMachine_demoThrow::CallbackEstimator(
    const hiperlab_rostools::mocap_output& msg) {
  if (_est->GetID() == msg.vehicleID) {
    _est->UpdateWithMeasurement(
        Vec3d(msg.posx, msg.posy, msg.posz),
        Rotationd(msg.attq0, msg.attq1, msg.attq2, msg.attq3));
  }
}

void VehicleStateMachine_demoThrow::CallbackTelemetry(
    const hiperlab_rostools::telemetry& msg) {
  _lastTelWarnings = msg.warnings;
}

void VehicleStateMachine_demoThrow::Initialize(int id, std::string name,
                                               ros::NodeHandle &n,
                                               BaseTimer* timer,
                                               double systemLatencyTime) {
  _id = id;
  stringstream ss;
  ss << "[" << name << " (" << _id << ")]: ";
  _name = ss.str();

  //set up networking stuff:
  _subMocap.reset(
      new ros::Subscriber(
          n.subscribe("mocap_output" + std::to_string(_id), 1,
                      &VehicleStateMachine_demoThrow::CallbackEstimator,
                      this)));

  _subTelemetry.reset(
      new ros::Subscriber(
          n.subscribe("telemetry" + std::to_string(_id), 1,
                      &VehicleStateMachine_demoThrow::CallbackTelemetry,
                      this)));

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

  _ctrl->SetMinMaxAcceleration(-2 * 9.81, 2 * 9.81);

  cout << _name << "Created.\n";
}

void VehicleStateMachine_demoThrow::Run(bool shouldStart, bool shouldStop) {
  //Publish estimator data:
  hiperlab_rostools::estimator_output estOutMsg;
  estOutMsg.header.stamp = ros::Time::now();
  estOutMsg.vehicleID = _est->GetID();

  MocapStateEstimator::MocapEstimatedState estState = _est->GetPrediction(
      _systemLatencyTime);
  _safetyNet->UpdateWithEstimator(estState,
                                  _est->GetTimeSinceLastGoodMeasurement());

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

  hiperlab_rostools::radio_command cmdMsg;
  uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];

  //use this to detect a change in the state machine
  bool stageChange = _flightStage != _lastFlightStage;
  _lastFlightStage = _flightStage;
  if (stageChange) {
    _stageTimer->Reset();
    _stageStartCmdPos = _lastCmdPosition;
  }

  switch (_flightStage) {
    case StageWaitForStart:
      if (stageChange) {
        cout << _name << "Waiting for start signal.\n";
      }
      if (shouldStart) {
        _flightStage = StageWaitForThrow;
      }
      break;

    case StageWaitForThrow:
      if (stageChange) {
        cout << _name << "Waiting to be thrown\n";
        cout << "Hit red button to stop program & exit.\n";
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageComplete;
      }

      {

        //this maneuver should cause us to end upside down
        double cmdThrust(2.0);
        Vec3d cmdAngVel(0, 0, 0);

        unsigned flags = 0;

        _lastCmdPosition = estState.pos;

        flags |= RadioTypes::ReservedFlags::disableOnboardStateSafetyChecks;  //disable the upside-down check
        //Publish the commands:
        RadioTypes::RadioMessageDecoded::CreateRatesCommand(flags,
                                                            float(cmdThrust),
                                                            Vec3f(cmdAngVel),
                                                            rawMsg);
        cmdMsg.debugvals[0] = float(cmdThrust);
        cmdMsg.debugvals[1] = float(cmdAngVel.x);
        cmdMsg.debugvals[2] = float(cmdAngVel.y);
        cmdMsg.debugvals[3] = float(cmdAngVel.z);
        cmdMsg.debugtype = RadioTypes::externalRatesCmd;
      }

      if (estState.pos.z > THROWN_DETECTION_HEIGHT) {
        printf("throw detected!\n");
        _flightStage = StageDisturbanceRecover;
      }

      if (shouldStop) {
        _flightStage = StageComplete;
      }
      break;

    case StageDisturbanceRecover:
      if (stageChange) {
        cout << _name << "Recovery stage\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }

      {

        Vec3d cmdAngVel;
        double cmdThrust;
        _lastCmdPosition = _throwRecoverPos;
        _desiredYawAngle = 0;
        _ctrl->Run(estState.pos, estState.vel, estState.att, _lastCmdPosition,
                   Vec3d(0, 0, 0), Vec3d(0, 0, 0), _desiredYawAngle, cmdAngVel,
                   cmdThrust);

        //Tell the estimator:
        _est->SetPredictedValues(
            cmdAngVel,
            (estState.att * Vec3d(0, 0, 1) * cmdThrust - Vec3d(0, 0, 9.81)));

        unsigned flags = 0;
        if (_stageTimer->GetSeconds<double>() < 1.0) {
          flags |= RadioTypes::ReservedFlags::disableOnboardStateSafetyChecks;  //disable the upside-down check
        }
        //Publish the commands:
        RadioTypes::RadioMessageDecoded::CreateRatesCommand(flags,
                                                            float(cmdThrust),
                                                            Vec3f(cmdAngVel),
                                                            rawMsg);
        cmdMsg.debugvals[0] = float(cmdThrust);
        cmdMsg.debugvals[1] = float(cmdAngVel.x);
        cmdMsg.debugvals[2] = float(cmdAngVel.y);
        cmdMsg.debugvals[3] = float(cmdAngVel.z);
        cmdMsg.debugtype = RadioTypes::externalRatesCmd;

        cmdMsg.debugvals[5] = 0;
        cmdMsg.debugvals[6] = 0;
        cmdMsg.debugvals[7] = 0;
      }

      if (shouldStop) {
        _flightStage = StageLanding;
      }

      if (_stageTimer->GetSeconds<double>() > RECOVERY_TIME) {
        _flightStage = StageDisturbanceReturn;
      }

      break;
    case StageDisturbanceReturn:
      if (stageChange) {
        cout << _name << "Return to hover stage\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }

      {
        Vec3d cmdAngVel;
        double cmdThrust;
        double l = _stageTimer->GetSeconds<double>() / RETURN_TIME;
        _lastCmdPosition = _throwRecoverPos * (1 - l) + _throwReturnPos * (l);
        Vec3d cmdVel = (_throwReturnPos - _throwRecoverPos) / RETURN_TIME;

        unsigned flags = 0;
        if (_stageTimer->GetSeconds<double>() > 0.8 * RETURN_TIME) {
          flags |= RadioTypes::ReservedFlags::disableOnboardStateSafetyChecks;  //disable the upside-down check
        }

        if (_stageTimer->GetSeconds<double>() > 1.2 * RETURN_TIME) {
          //some magic numbers...
          double GRAB_DETECT_ANGLE = M_PI / 3;
          if ((_grabBeginningAtt.Inverse() * estState.att).GetAngle()
              > M_PI / 3) {
            _flightStage = StageWaitForThrow;
          }
        } else if (_stageTimer->GetSeconds<double>() > 1 * RETURN_TIME) {
          _grabBeginningAtt = estState.att;
        }

        if (l >= 1.0) {
          _lastCmdPosition = _throwReturnPos;
          cmdVel = Vec3d(0, 0, 0);

        }

        _desiredYawAngle = 0;
        _ctrl->Run(estState.pos, estState.vel, estState.att, _lastCmdPosition,
                   cmdVel, Vec3d(0, 0, 0), _desiredYawAngle, cmdAngVel,
                   cmdThrust);

        //Tell the estimator:
        _est->SetPredictedValues(
            cmdAngVel,
            (estState.att * Vec3d(0, 0, 1) * cmdThrust - Vec3d(0, 0, 9.81)));

        //Publish the commands:
        RadioTypes::RadioMessageDecoded::CreateRatesCommand(flags,
                                                            float(cmdThrust),
                                                            Vec3f(cmdAngVel),
                                                            rawMsg);
        cmdMsg.debugvals[0] = float(cmdThrust);
        cmdMsg.debugvals[1] = float(cmdAngVel.x);
        cmdMsg.debugvals[2] = float(cmdAngVel.y);
        cmdMsg.debugvals[3] = float(cmdAngVel.z);
        cmdMsg.debugtype = RadioTypes::externalRatesCmd;

      }

      if (shouldStop) {
        _flightStage = StageLanding;
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
        Vec3d cmdAngVel;
        double cmdThrust;
        double const LANDING_SPEED = 0.5;  //m/s
        _lastCmdPosition = _stageStartCmdPos
            + _stageTimer->GetSeconds<double>() * Vec3d(0, 0, -LANDING_SPEED);
        if (_lastCmdPosition.z < 0) {
          _flightStage = StageComplete;
        }

        _desiredYawAngle = 0;
        _ctrl->Run(estState.pos, estState.vel, estState.att, _lastCmdPosition,
                   Vec3d(0, 0, -LANDING_SPEED), Vec3d(0, 0, 0),
                   _desiredYawAngle, cmdAngVel, cmdThrust);

        //Tell the estimator:
        _est->SetPredictedValues(
            cmdAngVel,
            (estState.att * Vec3d(0, 0, 1) * cmdThrust - Vec3d(0, 0, 9.81)));

        unsigned flags = 0;
        //Publish the commands:
        RadioTypes::RadioMessageDecoded::CreateRatesCommand(flags,
                                                            float(cmdThrust),
                                                            Vec3f(cmdAngVel),
                                                            rawMsg);
        cmdMsg.debugvals[0] = float(cmdThrust);
        cmdMsg.debugvals[1] = float(cmdAngVel.x);
        cmdMsg.debugvals[2] = float(cmdAngVel.y);
        cmdMsg.debugvals[3] = float(cmdAngVel.z);
        cmdMsg.debugtype = RadioTypes::externalRatesCmd;

      }
      break;

    case StageComplete:
      if (stageChange) {
        cout << _name << "Done. Motors off.\n";
      }

      {
        _est->SetPredictedValues(Vec3d(0, 0, 0), Vec3d(0, 0, 0));

        //Publish the commands:
        RadioTypes::RadioMessageDecoded::CreateIdleCommand(0, rawMsg);

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
      cmdMsg.debugtype = RadioTypes::emergencyKill;
      break;

  }

  for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++) {
    cmdMsg.raw[i] = rawMsg[i];
  }
  cmdMsg.header.stamp = ros::Time::now();

  _pubCmd->publish(cmdMsg);

}
