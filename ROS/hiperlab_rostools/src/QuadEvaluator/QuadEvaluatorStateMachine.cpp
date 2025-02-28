#include "QuadEvaluatorStateMachine.hpp"

using namespace std;
using namespace Offboard;

QuadEvaluatorStateMachine::QuadEvaluatorStateMachine() {
  _name = "INVALID";
  _id = 0;

  _flightStage = StageWaitForStart;
  _lastFlightStage = StageComplete;
  _systemLatencyTime = 0;

  _vehicleIsReadyForProgramToExit = false;

  _initPosition = Vec3d(0, 0, 0);
  _desiredPosition = Vec3d(0, 0, 1.5);
  _lastTelWarnings = 0;

  for (unsigned i = 0; i < NUM_YAW_ANGLES; i++) {
    for (unsigned m = 0; m < 4; m++) {
      _avgForce[m][i].Reset();
    }

    for (unsigned m = 0; m < 3; m++) {
      _avgPos[m][i].Reset();
      _avgAtt[m][i].Reset();
    }
  }

  for (unsigned m = 0; m < 3; m++) {
    _avgRateGyro[m].Reset();
    _avgAcc[m].Reset();
  }

  _lastTelForces[0] = _lastTelForces[1] = _lastTelForces[2] =
      _lastTelForces[3] = 0;
  _lastMocapAtt = _lastMocapPos = Vec3d(0, 0, 0);

  _lastTelRateGyro = Vec3d(0, 0, 0);
  _lastTelAcc = Vec3d(0, 0, 0);

  _avgCntr = 0;

  _timeSettle = 5.0;
  _timeAverage = 5.0;

  _mass = 0;
}

void QuadEvaluatorStateMachine::CallbackEstimator(
    const hiperlab_rostools::mocap_output& msg) {
  if (_est->GetID() == msg.vehicleID) {
    _est->UpdateWithMeasurement(
        Vec3d(msg.posx, msg.posy, msg.posz),
        Rotationd(msg.attq0, msg.attq1, msg.attq2, msg.attq3));
    _lastMocapPos = Vec3d(msg.posx, msg.posy, msg.posz);
    _lastMocapAtt = Vec3d(msg.attyaw, msg.attpitch, msg.attroll);
  }
}

void QuadEvaluatorStateMachine::CallbackTelemetry(
    const hiperlab_rostools::telemetry& msg) {
  _lastTelWarnings = msg.warnings;
  _lastTelForces[0] = msg.motorForces[0];
  _lastTelForces[1] = msg.motorForces[1];
  _lastTelForces[2] = msg.motorForces[2];
  _lastTelForces[3] = msg.motorForces[3];
  _lastTelRateGyro = Vec3d(msg.rateGyro[0], msg.rateGyro[1], msg.rateGyro[2]);
  _lastTelAcc = Vec3d(msg.accelerometer[0], msg.accelerometer[1],
                      msg.accelerometer[2]);
}

void QuadEvaluatorStateMachine::Initialize(int id, std::string name,
                                           ros::NodeHandle &n, BaseTimer* timer,
                                           double systemLatencyTime) {
  _id = id;
  stringstream ss;
  ss << "[" << name << " (" << _id << ")]: ";
  _name = ss.str();

  //set up networking stuff:
  _subMocap.reset(
      new ros::Subscriber(
          n.subscribe("mocap_output" + std::to_string(_id), 1,
                      &QuadEvaluatorStateMachine::CallbackEstimator, this)));

  _subTelemetry.reset(
      new ros::Subscriber(
          n.subscribe("telemetry" + std::to_string(_id), 1,
                      &QuadEvaluatorStateMachine::CallbackTelemetry, this)));

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

void QuadEvaluatorStateMachine::Run(bool shouldStart, bool shouldStop) {

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

      {
        double const takeOffTime = 2.0;  //[s]
        double frac = _stageTimer->GetSeconds<double>() / takeOffTime;
        if (frac >= 1.0) {
          _flightStage = StageSettle;
          frac = 1.0;
        }
        Vec3d cmdPos = (1 - frac) * _initPosition + frac * _desiredPosition;
        cmdMsg = RunControllerAndUpdateEstimator(estState, cmdPos,
                                                 Vec3d(0, 0, 0), Vec3d(0, 0, 0),
                                                 0);
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }
      break;

    case StageSettle:
      if (stageChange) {
        cout << _name << "Entering settling stage.\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }

      cmdMsg = RunControllerAndUpdateEstimator(estState, _desiredPosition,
                                               Vec3d(0, 0, 0), Vec3d(0, 0, 0),
                                               _avgCntr * (M_PI / 2));

      if (shouldStop) {
        _flightStage = StageLanding;
      }

      if (_stageTimer->GetSeconds_d() > _timeSettle) {
        _flightStage = StageRunAveraging;
      }
      break;

    case StageRunAveraging:
      if (stageChange) {
        cout << _name << "Entering averaging stage #" << int(_avgCntr) << ".\n";
      }

      if (!_safetyNet->GetIsSafe()) {
        _flightStage = StageEmergency;
      }

      if (HaveLowBattery()) {
        printf("LOW BATTERY!\n");
        _flightStage = StageLanding;
      }

      cmdMsg = RunControllerAndUpdateEstimator(estState, _desiredPosition,
                                               Vec3d(0, 0, 0), Vec3d(0, 0, 0),
                                               _avgCntr * (M_PI / 2));

      for (unsigned mot = 0; mot < 4; mot++) {
        double const expForce = _mass * 9.81 / 4;
        _avgForce[mot][_avgCntr].Update(_lastTelForces[mot] / expForce);
      }

      for (unsigned ax = 0; ax < 3; ax++) {
        _avgPos[ax][_avgCntr].Update((_lastMocapPos - _desiredPosition)[ax]);
        _avgAtt[ax][_avgCntr].Update(_lastMocapAtt[ax] * 180 / M_PI);
        _avgAcc[ax].Update(_lastTelAcc[ax]);
        _avgRateGyro[ax].Update(_lastTelRateGyro[ax]);
      }

      if (shouldStop) {
        _flightStage = StageLanding;
      }

      if (_stageTimer->GetSeconds_d() > _timeAverage) {
        _avgCntr++;
        if (_avgCntr >= NUM_YAW_ANGLES) {
          _flightStage = StageLanding;
        } else {
          _flightStage = StageSettle;
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
        Vec3d cmdPos = _desiredPosition
            + _stageTimer->GetSeconds<double>() * Vec3d(0, 0, -LANDING_SPEED);
        if (cmdPos.z < 0) {
          _flightStage = StageComplete;
        }
        cmdMsg = RunControllerAndUpdateEstimator(estState, cmdPos,
                                                 Vec3d(0, 0, -LANDING_SPEED),
                                                 Vec3d(0, 0, 0), 0);
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

void QuadEvaluatorStateMachine::PublishEstimate(
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

hiperlab_rostools::radio_command QuadEvaluatorStateMachine::RunControllerAndUpdateEstimator(
    MocapStateEstimator::MocapEstimatedState estState, Vec3d desPos,
    Vec3d desVel, Vec3d desAcc, float desYaw) {
  // Run the rates controller
  Vec3d cmdAngVel;
  double cmdThrust;
  _ctrl->Run(estState.pos, estState.vel, estState.att, desPos, desVel, desAcc,
             desYaw, cmdAngVel, cmdThrust);

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

void QuadEvaluatorStateMachine::PrintResults() {

  printf("\n\n");
  printf(" ++------------++\n");
  printf(" ++------------++\n");
  printf(" ||   RESULTS  ||\n");
  printf(" ++------------++\n");
  printf(" ++------------++\n");
  printf("\n");
  printf("Format is to print: MEAN+/-(1-STDDEV)\n");
  printf(
      "Motor calibration: average forces [-] over multiple runs, divided by exp. force. \n");
  printf("  Should be very similar (both across runs, and accross motors).\n");
  for (unsigned i = 0; i < NUM_YAW_ANGLES; i++) {
    printf("%d: ", int(i));
    for (unsigned mot = 0; mot < 4; mot++) {
      _avgForce[mot][i].PrintStats();
      printf("\t");
    }
    printf("\n");
  }
  printf("\n");

  printf("Position tracking error [m].\n");
  printf(
      "  Should be very similar across runs (and, small, ideally). Does error rotate with vehicle?\n");
  for (unsigned i = 0; i < NUM_YAW_ANGLES; i++) {
    printf("%d: ", int(i));
    for (unsigned ax = 0; ax < 3; ax++) {
      _avgPos[ax][i].PrintStats();
      printf("\t");
    }
    printf(
        "\t||.||=%5.3f\n",
        Vec3d(_avgPos[0][i].GetMean(), _avgPos[1][i].GetMean(),
              _avgPos[2][i].GetMean()).GetNorm2());
  }
  printf("\n\n");

  printf("Attitude [deg].\n");
  printf("  roll/pitch should be zero. Does error rotate with vehicle?\n");
  for (unsigned i = 0; i < NUM_YAW_ANGLES; i++) {
    printf("%d: ", int(i));
    for (unsigned ax = 0; ax < 3; ax++) {
      _avgAtt[ax][i].PrintStats();
      printf("\t");
    }
    printf("\n");
  }
  printf("\n");

  printf("IMU averages:\n");
  printf("  Rate gyro (should be zero) [rad/s]: ");
  for (unsigned ax = 0; ax < 3; ax++) {
    _avgRateGyro[ax].PrintStats();
    printf("\t");
  }
  printf("\n");
  printf("  Acc (should be (0,0,9.81)) [m/s**2]: ");
  for (unsigned ax = 0; ax < 3; ax++) {
    _avgAcc[ax].PrintStats();
    printf("\t");
  }
  printf("\n");
  printf("=====================================================\n");
}

