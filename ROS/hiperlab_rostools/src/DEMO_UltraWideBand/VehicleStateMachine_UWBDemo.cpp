#include "VehicleStateMachine_UWBDemo.hpp"

#include "Common/Misc/TerminalColors.hpp"

using namespace std;
using namespace Offboard;
using namespace TerminalColors;

VehicleStateMachine_UWBDemo::VehicleStateMachine_UWBDemo() {
  _name = "INVALID";
  _id = 0;

  _flightStage = StageWaitForStart;
  _lastFlightStage = StageComplete;
  _systemLatencyTime = 0;

  _vehicleIsReadyForProgramToExit = false;

  _initPosition = Vec3d(0, 0, 0);
  _desiredPosition = Vec3d(0, 0, 0);
  _desiredYawAngle = 0;
  _measMode = MeasMocap;

  _lastTelWarnings = 0;

  _circleTrajectory.center = Vec3d(0, 0, 0);
  _circleTrajectory.angVel = 0;
  _circleTrajectory.center = Vec3d(0, 0, 0);
  _circleTrajectory.phase = 0;
  _circleTrajectory.dt = 0;
  _circleTrajectory.lastButtonState = true;

  _uwbEst.pos = Vec3d(0, 0, 0);
  _uwbEst.vel = Vec3d(0, 0, 0);
  _uwbEst.att = Rotationd::Identity();

  _uwbEstError.pos = Vec3d(0, 0, 0);
  _uwbEstError.vel = Vec3d(0, 0, 0);
  _uwbEstError.att = Rotationd::Identity();
  _uwbEstError.lpTimeConst = 1;  //s
}

void VehicleStateMachine_UWBDemo::CallbackEstimator(
    const hiperlab_rostools::mocap_output& msg) {
  if (_id == msg.vehicleID) {
    _est->UpdateWithMeasurement(
        Vec3d(msg.posx, msg.posy, msg.posz),
        Rotationd(msg.attq0, msg.attq1, msg.attq2, msg.attq3));
  }
}

void VehicleStateMachine_UWBDemo::CallbackTelemetry(
    const hiperlab_rostools::telemetry& msg) {
  _lastTelWarnings = msg.warnings;

  for (int i = 0; i < 3; i++) {
    _uwbEst.pos[i] = msg.position[i];
  }

  Rotationd a = Rotationd::FromVectorPartOfQuaternion(
      Vec3d(msg.attitude[0], msg.attitude[1], msg.attitude[2]));
  for (int i = 0; i < 4; i++) {
    _uwbEst.att[i] = a[i];
  }
}

void VehicleStateMachine_UWBDemo::Initialize(int id, std::string name,
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
                      &VehicleStateMachine_UWBDemo::CallbackEstimator, this)));

  _subTelemetry.reset(
      new ros::Subscriber(
          n.subscribe("telemetry" + std::to_string(_id), 1,
                      &VehicleStateMachine_UWBDemo::CallbackTelemetry, this)));

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
  _timerRun.reset(new Timer(timer));
  _measMode = MeasMocap;

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

void VehicleStateMachine_UWBDemo::PrintEstimationError() {
  double const ACCEPTABLE_ERR_POS = 1.0;
  double const ACCEPTABLE_ERR_ATT = 20 * M_PI / 180;

  bool posOK = _uwbEstError.pos.GetNorm2() < ACCEPTABLE_ERR_POS;
  bool attOK = _uwbEstError.att.GetAngle() < ACCEPTABLE_ERR_ATT;

  printf("%s est err: ", _name.c_str());
  SetTerminalColor((posOK and attOK) ? GREEN : RED);
  printf("[%dmm, %ddeg] ", int(_uwbEstError.pos.GetNorm2() * 1e3 + 0.5),
         int(_uwbEstError.att.GetAngle() * 180 / M_PI + 0.5));
  SetTerminalColor(posOK ? GREEN : RED);
  printf("e_pos = <%3.3f,%3.3f,%3.3f>m ", _uwbEstError.pos.x,
         _uwbEstError.pos.y, _uwbEstError.pos.z);
  SetTerminalColor(attOK ? GREEN : RED);
  double y, p, r;
  _uwbEstError.att.ToEulerYPR(y, p, r);
  y *= 180 / M_PI;
  p *= 180 / M_PI;
  r *= 180 / M_PI;
  printf("\te_att = YPR<%3.1f,%3.1f,%3.1f>deg", y, p, r);
  ResetTerminalColor();
  printf("\n");
}

void VehicleStateMachine_UWBDemo::Run(bool shouldStart, bool shouldStop,
                                      bool btnGreen, bool btnYellow,
                                      bool btnBlue) {
  double dt = _timerRun->GetSeconds_f();
  _timerRun->Reset();

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

  //update the estimator's error:
  //annoying volatile stuff:
  Vec3d uwbEstPos = Vec3d(_uwbEst.pos.x, _uwbEst.pos.y, _uwbEst.pos.z);
  Vec3d uwbEstVel = Vec3d(_uwbEst.vel.x, _uwbEst.vel.y, _uwbEst.vel.z);
  Rotationd uwbEstAtt = Rotationd(_uwbEst.att[0], _uwbEst.att[1],
                                  _uwbEst.att[2], _uwbEst.att[3]);
  Vec3d errPos = estState.pos - uwbEstPos;
  Vec3d errVel = estState.vel - uwbEstVel;
  Rotationd errAtt = estState.att.Inverse() * uwbEstAtt;
  //filter errors:
  double c = exp(-dt / _uwbEstError.lpTimeConst);
  _uwbEstError.pos = c * _uwbEstError.pos + (1 - c) * errPos;
  _uwbEstError.vel = c * _uwbEstError.vel + (1 - c) * errVel;
  _uwbEstError.att = Rotationd::FromRotationVector(
      c * (_uwbEstError.att.Inverse() * errAtt).ToRotationVector()) * errAtt;

  hiperlab_rostools::radio_command cmdMsg;
  uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];

  //use this to detect a change in the state machine
  bool stageChange = _flightStage != _lastFlightStage;
  _lastFlightStage = _flightStage;
  if (stageChange) {
    _stageTimer->Reset();
  }

  if (btnYellow and (_measMode == MeasMocap)) {
    cout << _name << "Switching to UWB mode.\n";
    _measMode = MeasUWB;
  } else if (btnGreen and (_measMode == MeasUWB)) {
    cout << _name << "Switching to Mocap mode.\n";
    _measMode = MeasMocap;
  }

  bool circleTrajSwitch = btnBlue && !_circleTrajectory.lastButtonState;
  _circleTrajectory.lastButtonState = btnBlue;

  switch (_flightStage) {
    case StageWaitForStart:
      if (stageChange) {
        cout << _name << "Waiting for start signal.\n";
      }
      if (shouldStart) {
        _flightStage = StageTakeoff;
      }
      break;

    case StageTakeoff:
      if (stageChange) {
        cout << _name << "Taking off\n";
        _initPosition = estState.pos;
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
        double const takeOffTime = 2.0;  //[s]
        double frac = _stageTimer->GetSeconds<double>() / takeOffTime;
        if (frac >= 1.0) {
          _flightStage = StageFlightHover;
          frac = 1.0;
        }
        Vec3d cmdPos = (1 - frac) * _initPosition + frac * _desiredPosition;

        if (_measMode == MeasMocap) {
          _ctrl->Run(estState.pos, estState.vel, estState.att, cmdPos,
                     Vec3d(0, 0, 0), Vec3d(0, 0, 0), _desiredYawAngle,
                     cmdAngVel, cmdThrust);

          //Tell the estimator:
          _est->SetPredictedValues(
              cmdAngVel,
              (estState.att * Vec3d(0, 0, 1) * cmdThrust - Vec3d(0, 0, 9.81)));

          //Publish the commands:
          RadioTypes::RadioMessageDecoded::CreateRatesCommand(0,
                                                              float(cmdThrust),
                                                              Vec3f(cmdAngVel),
                                                              rawMsg);
          cmdMsg.debugvals[0] = float(cmdThrust);
          cmdMsg.debugvals[1] = float(cmdAngVel.x);
          cmdMsg.debugvals[2] = float(cmdAngVel.y);
          cmdMsg.debugvals[3] = float(cmdAngVel.z);
          cmdMsg.debugtype = RadioTypes::externalRatesCmd;
        } else {
          //Tell the estimator:
          _est->SetPredictedValues(Vec3d(0, 0, 0), Vec3d(0, 0, 0));

          //Publish the commands:
          RadioTypes::RadioMessageDecoded::CreatePositionCommand(0, cmdPos,
                                                                 Vec3d(0, 0, 0),  //cmd vel
                                                                 Vec3d(0, 0, 0),  //cmd acc
                                                                 0,  //yaw
                                                                 rawMsg);
          cmdMsg.debugvals[0] = cmdPos.x;
          cmdMsg.debugvals[1] = cmdPos.y;
          cmdMsg.debugvals[2] = cmdPos.z;
          cmdMsg.debugtype = RadioTypes::positionCommand;
        }

      }
      break;

    case StageFlightHover:
      if (stageChange) {
        cout << _name << "Entering hover flight stage.\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }

      if (_measMode == MeasMocap) {
        Vec3d cmdAngVel;
        double cmdThrust;
        _ctrl->Run(estState.pos, estState.vel, estState.att, _desiredPosition,
                   Vec3d(0, 0, 0), Vec3d(0, 0, 0), _desiredYawAngle, cmdAngVel,
                   cmdThrust);

        //Tell the estimator:
        _est->SetPredictedValues(
            cmdAngVel,
            (estState.att * Vec3d(0, 0, 1) * cmdThrust - Vec3d(0, 0, 9.81)));

        //Publish the commands:
        RadioTypes::RadioMessageDecoded::CreateRatesCommand(0, float(cmdThrust),
                                                            Vec3f(cmdAngVel),
                                                            rawMsg);
        cmdMsg.debugvals[0] = float(cmdThrust);
        cmdMsg.debugvals[1] = float(cmdAngVel.x);
        cmdMsg.debugvals[2] = float(cmdAngVel.y);
        cmdMsg.debugvals[3] = float(cmdAngVel.z);
        cmdMsg.debugtype = RadioTypes::externalRatesCmd;
      } else {
        //Tell the estimator:
        _est->SetPredictedValues(Vec3d(0, 0, 0), Vec3d(0, 0, 0));

        //Publish the commands:
        RadioTypes::RadioMessageDecoded::CreatePositionCommand(0,
                                                               _desiredPosition,
                                                               Vec3d(0, 0, 0),  //cmd vel
                                                               Vec3d(0, 0, 0),  //cmd acc
                                                               0,  //yaw
                                                               rawMsg);
        cmdMsg.debugvals[0] = _desiredPosition.x;
        cmdMsg.debugvals[1] = _desiredPosition.y;
        cmdMsg.debugvals[2] = _desiredPosition.z;
        cmdMsg.debugtype = RadioTypes::positionCommand;
      }

      if (shouldStop) {
        _flightStage = StageLanding;
      }

      if (circleTrajSwitch) {
        _flightStage = StageFlightFlyCicle;
      }

      break;

    case StageFlightFlyCicle:
      if (stageChange) {
        //compute phase & radius:
        Vec3d distFromCenter = _desiredPosition - _circleTrajectory.center;
        distFromCenter.z = 0;
        if (distFromCenter.GetNorm2() < 1e-6) {
          _circleTrajectory.phase = 0;
        } else {
          _circleTrajectory.phase = atan2(distFromCenter.y, distFromCenter.x);
        }
        cout << _name << "Entering circle trajectory flight stage. Phase = "
             << _circleTrajectory.phase * 180 / M_PI << "deg \n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }

      //scope for desvel
      {
        _desiredPosition = _circleTrajectory.center
            + Vec3d(cos(_circleTrajectory.phase), +sin(_circleTrajectory.phase),
                    0) * _circleTrajectory.radius;
        Vec3d desVel = Vec3d(-sin(_circleTrajectory.phase),
                             +cos(_circleTrajectory.phase), 0)
            * _circleTrajectory.angVel * _circleTrajectory.radius;
        Vec3d desAcc = Vec3d(-cos(_circleTrajectory.phase),
                             -sin(_circleTrajectory.phase), 0)
            * _circleTrajectory.angVel * _circleTrajectory.angVel
            * _circleTrajectory.radius;

        _circleTrajectory.phase += _circleTrajectory.angVel
            * _circleTrajectory.dt;

        if (_circleTrajectory.phase > 2 * M_PI) {
          _circleTrajectory.phase -= 2 * M_PI;
        }

        if (_measMode == MeasMocap) {
          Vec3d cmdAngVel;
          double cmdThrust;
          _ctrl->Run(estState.pos, estState.vel, estState.att, _desiredPosition,
                     desVel, desAcc, _desiredYawAngle, cmdAngVel, cmdThrust);

          //Tell the estimator:
          _est->SetPredictedValues(
              cmdAngVel,
              (estState.att * Vec3d(0, 0, 1) * cmdThrust - Vec3d(0, 0, 9.81)));

          //Publish the commands:
          RadioTypes::RadioMessageDecoded::CreateRatesCommand(0,
                                                              float(cmdThrust),
                                                              Vec3f(cmdAngVel),
                                                              rawMsg);
          cmdMsg.debugvals[0] = float(cmdThrust);
          cmdMsg.debugvals[1] = float(cmdAngVel.x);
          cmdMsg.debugvals[2] = float(cmdAngVel.y);
          cmdMsg.debugvals[3] = float(cmdAngVel.z);
          cmdMsg.debugtype = RadioTypes::externalRatesCmd;
        } else {
          //Tell the estimator:
          _est->SetPredictedValues(Vec3d(0, 0, 0), Vec3d(0, 0, 0));

          //Publish the commands:
          RadioTypes::RadioMessageDecoded::CreatePositionCommand(
              0, _desiredPosition, desVel, desAcc, 0, rawMsg);
          cmdMsg.debugvals[0] = _desiredPosition.x;
          cmdMsg.debugvals[1] = _desiredPosition.y;
          cmdMsg.debugvals[2] = _desiredPosition.z;
          cmdMsg.debugtype = RadioTypes::positionCommand;
        }
      }

      if (shouldStop) {
        _flightStage = StageLanding;
      }

      if (circleTrajSwitch) {
        _flightStage = StageFlightHover;
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
        Vec3d cmdVel = Vec3d(0, 0, -LANDING_SPEED);
        Vec3d cmdPos = _desiredPosition
            + _stageTimer->GetSeconds<double>() * cmdVel;
        if (cmdPos.z < 0) {
          _flightStage = StageComplete;
        }

        if (_measMode == MeasMocap) {
          Vec3d cmdAngVel;
          double cmdThrust;
          _ctrl->Run(estState.pos, estState.vel, estState.att, cmdPos, cmdVel,
                     Vec3d(0, 0, 0), _desiredYawAngle, cmdAngVel, cmdThrust);

          //Tell the estimator:
          _est->SetPredictedValues(
              cmdAngVel,
              (estState.att * Vec3d(0, 0, 1) * cmdThrust - Vec3d(0, 0, 9.81)));

          //Publish the commands:
          RadioTypes::RadioMessageDecoded::CreateRatesCommand(0,
                                                              float(cmdThrust),
                                                              Vec3f(cmdAngVel),
                                                              rawMsg);
          cmdMsg.debugvals[0] = float(cmdThrust);
          cmdMsg.debugvals[1] = float(cmdAngVel.x);
          cmdMsg.debugvals[2] = float(cmdAngVel.y);
          cmdMsg.debugvals[3] = float(cmdAngVel.z);
          cmdMsg.debugtype = RadioTypes::externalRatesCmd;
        } else {
          //Tell the estimator:
          _est->SetPredictedValues(Vec3d(0, 0, 0), Vec3d(0, 0, 0));

          //Publish the commands:
          RadioTypes::RadioMessageDecoded::CreatePositionCommand(0, cmdPos,
                                                                 cmdVel,  //cmd vel
                                                                 Vec3d(0, 0, 0),  //cmd acc
                                                                 0,  //yaw
                                                                 rawMsg);
          cmdMsg.debugvals[0] = cmdPos.x;
          cmdMsg.debugvals[1] = cmdPos.y;
          cmdMsg.debugvals[2] = cmdPos.z;
          cmdMsg.debugtype = RadioTypes::positionCommand;
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

        cmdMsg.debugtype = RadioTypes::idleCommand;
      }
      if (_stageTimer->GetSeconds<double>() > 1.0) {
        if (!_vehicleIsReadyForProgramToExit) {
          cout << _name << "Exiting.\n";
        }
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

