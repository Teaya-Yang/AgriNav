#include <iostream>
#include <thread>

#include "ros/ros.h"

#include "hiperlab_rostools/mocap_output.h"
#include "hiperlab_rostools/estimator_output.h"
#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/joystick_values.h"

#include "Common/DataTypes/RadioTypes.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Offboard/MocapStateEstimator.hpp"
#include "Components/Offboard/QuadcopterController.hpp"
#include "Components/Offboard/SafetyNet.hpp"
#include "Components/Logic/QuadcopterConstants.hpp"

using namespace Offboard;
using namespace std;

shared_ptr<MocapStateEstimator> est;

//Definitions:
double const mainLoopFrequency = 50;  //Hz
double const systemLatencyTime = 30e-3;  // latency in measurements & commands[s]

double const MAX_CALIB_POS_ERR = 1.0;  //[m]
double const MAX_CALIB_VEL_ERR = 0.4;  //[m/s]
double const CALIBRATION_TIME = 10.0;  //[s]

double const CALIB_HEIGHT = 1.25;  // [m]

enum FlightStage {
  StageWaitForJoysticStart,
  StageTakeoff,
  StageFlight,
  StageCalibration,
  StageLanding,
  StageComplete,
  StageEmergency,
};

void callback_estimator(const hiperlab_rostools::mocap_output& msg) {

  if (est->GetID() == msg.vehicleID) {
    est->UpdateWithMeasurement(
        Vec3d(msg.posx, msg.posy, msg.posz),
        Rotationd(msg.attq0, msg.attq1, msg.attq2, msg.attq3));
  }
}

bool volatile jsButtonStart = false;
bool volatile jsButtonStop = false;
void callback_joystick(const hiperlab_rostools::joystick_values& msg) {
  jsButtonStart = msg.buttonStart > 0;
  jsButtonStop = msg.buttonRed > 0;
}

void rosThreadFn() {
  // We run this function as a separate thread, allows us to continuously service any subscribers.
  ros::spin();
}

int main(int argc, char **argv) {

  if (argc < 2) {
    printf("ERROR: Must specify the vehicle ID\n");
    return -1;
  }

  int const vehicleId = atol(argv[1]);
  if (vehicleId <= 0 || vehicleId > 255) {
    printf("ERROR: invalid vehicle ID\n");
    return -1;
  }

  ros::init(argc, argv, "quad_mocap_rates_control" + std::to_string(vehicleId));

  ros::NodeHandle n;
  ros::Subscriber subMocap = n.subscribe(
      "mocap_output" + std::to_string(vehicleId), 1, callback_estimator);
  ros::Subscriber subJoystick = n.subscribe("joystick_values", 1,
                                            callback_joystick);
  ros::Publisher pubEstimate = n.advertise<hiperlab_rostools::estimator_output>(
      "estimator" + std::to_string(vehicleId), 1);
  ros::Publisher pubCmd = n.advertise<hiperlab_rostools::radio_command>(
      "radio_command" + std::to_string(vehicleId), 1);

  //Set everything up.
  HardwareTimer timer;

  est.reset(new MocapStateEstimator(&timer, vehicleId, systemLatencyTime));
  QuadcopterController ctrl;
  SafetyNet safetyNet;

  Onboard::QuadcopterConstants::QuadcopterType quadcopterType =
      Onboard::QuadcopterConstants::GetVehicleTypeFromID(vehicleId);
  Onboard::QuadcopterConstants vehConsts(quadcopterType);
  ctrl.SetParameters(vehConsts.posControl_natFreq, vehConsts.posControl_damping,
                     vehConsts.attControl_timeConst_xy,
                     vehConsts.attControl_timeConst_z);

  Vec3d desiredPosition(0, 0, CALIB_HEIGHT);
  double desiredYawAngle = 0 * M_PI / 180.0;

  ros::Rate loop_rate(mainLoopFrequency);

  Timer emergencyTimer(&timer);
  double const EMERGENCY_BUTTON_PERIOD = 0.5;  //if you hold the land button down, it triggers an emergency.

  cout << "Starting main controller.\n";

  thread rosThread(rosThreadFn);

  bool firstPanic = true;

  cout << "Waiting for estimator init...\n";
  while (ros::ok()) {
    loop_rate.sleep();
    if (est->GetIsInitialized()) {
      break;
    }
  }
  cout << "Est initialized.\n";

  cout << "Waiting for joystick start button...\n";
  while (ros::ok()) {
    loop_rate.sleep();
    if (jsButtonStart) {
      //force to release button:
      while (jsButtonStart) {
        loop_rate.sleep();
      }
      break;
    }
  }
  cout << "Continuing. Hit start again to take off.\n";

  FlightStage flightStage = StageWaitForJoysticStart;
  FlightStage lastFlightStage = StageComplete;
  Timer stageTimer(&timer);  //keep track of time we've been in current stage
  Vec3d initPosition;
  bool shouldQuit = false;

  while (ros::ok() && !shouldQuit) {
    //Publish estimator data:
    hiperlab_rostools::estimator_output estOutMsg;
    estOutMsg.header.stamp = ros::Time::now();
    estOutMsg.vehicleID = est->GetID();

    MocapStateEstimator::MocapEstimatedState estState = est->GetPrediction(
        systemLatencyTime);
    safetyNet.UpdateWithEstimator(estState,
                                  est->GetTimeSinceLastGoodMeasurement());

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

    pubEstimate.publish(estOutMsg);

    if (!jsButtonStop) {
      emergencyTimer.Reset();
    }

    if (emergencyTimer.GetSeconds<double>() > EMERGENCY_BUTTON_PERIOD) {
      printf("Panic button!\n");
      safetyNet.SetUnsafe();
    }

    hiperlab_rostools::radio_command cmdMsg;
    uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];

    //use this to detect a change in the state machine
    bool stageChange = flightStage != lastFlightStage;
    lastFlightStage = flightStage;
    if (stageChange) {
      stageTimer.Reset();
    }

    switch (flightStage) {
      case StageWaitForJoysticStart:
        if (stageChange) {
          cout
              << "Waiting for Joystick start command. Run with argument `--no-js` to disable. \n";
        }

        if (jsButtonStart) {
          if (!safetyNet.GetIsSafe()) {
            //Don't take off if unsafe!
            flightStage = StageEmergency;
          } else {
            flightStage = StageTakeoff;
          }
        }

        RadioTypes::RadioMessageDecoded::CreateIdleCommand(0, rawMsg);

        cmdMsg.debugtype = RadioTypes::idleCommand;
        break;

      case StageTakeoff:
        if (stageChange) {
          cout << "Taking off\n";
          initPosition = estState.pos;

          desiredPosition.x = est->GetPrediction(0).pos.x;
          desiredPosition.y = est->GetPrediction(0).pos.y;
          cout << "Desired position setpoint updated = <" << desiredPosition.x
               << "," << desiredPosition.y << "," << desiredPosition.z << ">\n";
        }

        if (!safetyNet.GetIsSafe()) {
          flightStage = StageEmergency;
        }

        {
          Vec3d cmdAngVel;
          double cmdThrust;
          double const takeOffTime = 2.0;  //[s]
          double frac = stageTimer.GetSeconds<double>() / takeOffTime;
          if (frac >= 1.0) {
            flightStage = StageFlight;
            frac = 1.0;
          }
          Vec3d cmdPos = (1 - frac) * initPosition + frac * desiredPosition;
          ctrl.Run(estState.pos, estState.vel, estState.att, cmdPos,
                   Vec3d(0, 0, 0), Vec3d(0, 0, 0), desiredYawAngle, cmdAngVel,
                   cmdThrust);

          //Tell the estimator:
          est->SetPredictedValues(
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

        }
        break;

      case StageFlight:
        if (stageChange) {
          cout << "Entering flight stage\n";
        }

        if (!safetyNet.GetIsSafe()) {
          flightStage = StageEmergency;
        }

        {
          Vec3d cmdAngVel;
          double cmdThrust;
          ctrl.Run(estState.pos, estState.vel, estState.att, desiredPosition,
                   Vec3d(0, 0, 0), Vec3d(0, 0, 0), desiredYawAngle, cmdAngVel,
                   cmdThrust);

          //Tell the estimator:
          est->SetPredictedValues(
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

          bool shouldCalibrateMotors = true;
          double posErr = (estState.pos - desiredPosition).GetNorm2();
          double velErr = estState.vel.GetNorm2();
          if (posErr > MAX_CALIB_POS_ERR) {
            shouldCalibrateMotors = false;
          }
          if (velErr > MAX_CALIB_VEL_ERR) {
            shouldCalibrateMotors = false;
          }

          if (!shouldCalibrateMotors) {
            static unsigned counter = 0;
            counter++;
            if (counter > 50) {
              printf(
                  "pos err = %3.3fm, vel err = %3.3fm/s (max: %3.3f, %3.3f)\n",
                  posErr, velErr, MAX_CALIB_POS_ERR, MAX_CALIB_VEL_ERR);
              counter = 0;
            }
          }

          if (shouldCalibrateMotors) {
            flightStage = StageCalibration;
          }
        }

        if (jsButtonStop) {
          flightStage = StageLanding;
        }
        break;

      case StageCalibration:
        if (stageChange) {
          cout << "Entering calibration stage\n";
        }

        if (!safetyNet.GetIsSafe()) {
          flightStage = StageEmergency;
        }

        {
          bool shouldCalibrate = true;
          if ((estState.pos - desiredPosition).GetNorm2() > MAX_CALIB_POS_ERR) {
            cout << "Abandoning calibration, excessive position error!\n";
            flightStage = StageFlight;
            shouldCalibrate = false;
          }
          if (estState.vel.GetNorm2() > MAX_CALIB_VEL_ERR) {
            cout << "Abandoning calibration, excessive velocity!\n";
            flightStage = StageFlight;
            shouldCalibrate = false;
          }
          if (stageTimer.GetSeconds<double>() > CALIBRATION_TIME) {
            cout << "Calibration time complete.\n";
            flightStage = StageLanding;
          }

          Vec3d cmdAngVel;
          double cmdThrust;
          bool flags = 0;
          ctrl.Run(estState.pos, estState.vel, estState.att, desiredPosition,
                   Vec3d(0, 0, 0), Vec3d(0, 0, 0), desiredYawAngle, cmdAngVel,
                   cmdThrust);

          //Tell the estimator:
          est->SetPredictedValues(
              cmdAngVel,
              (estState.att * Vec3d(0, 0, 1) * cmdThrust - Vec3d(0, 0, 9.81)));

          //Publish the commands:
          if (shouldCalibrate) {
            flags = RadioTypes::ReservedFlags::calibrateMotors;
          }
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

        if (jsButtonStop) {
          flightStage = StageLanding;
        }
        break;

      case StageLanding:
        if (stageChange) {
          cout << "Starting landing.\n";
        }

        if (!safetyNet.GetIsSafe()) {
          flightStage = StageEmergency;
        }

        {
          Vec3d cmdAngVel;
          double cmdThrust;
          double const LANDING_SPEED = 0.5;  //m/s
          Vec3d cmdPos = desiredPosition
              + stageTimer.GetSeconds<double>() * Vec3d(0, 0, -LANDING_SPEED);
          if (cmdPos.z < 0) {
            flightStage = StageComplete;
          }

          ctrl.Run(estState.pos, estState.vel, estState.att, cmdPos,
                   Vec3d(0, 0, -LANDING_SPEED), Vec3d(0, 0, 0), desiredYawAngle,
                   cmdAngVel, cmdThrust);

          //Tell the estimator:
          est->SetPredictedValues(
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

        }
        break;

      case StageComplete:
        if (stageChange) {
          cout << "Landing complete. Idling.\n";
        }

        {
          est->SetPredictedValues(Vec3d(0, 0, 0), Vec3d(0, 0, 0));

          //Publish the commands:
          RadioTypes::RadioMessageDecoded::CreateIdleCommand(0, rawMsg);

          cmdMsg.debugtype = RadioTypes::idleCommand;
        }
        if (stageTimer.GetSeconds<double>() > 1.0) {
          cout << "Exiting.";
          shouldQuit = true;
        }
        break;

      default:
      case StageEmergency:
        if (stageChange) {
          cout << "Emergency stage! Safety net = <"
               << safetyNet.GetStatusString() << ">.\n";
        }

        RadioTypes::RadioMessageDecoded::CreateKillCommand(0, rawMsg);
        cmdMsg.debugtype = RadioTypes::emergencyKill;
        break;

    }

    for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++) {
      cmdMsg.raw[i] = rawMsg[i];
    }
    cmdMsg.header.stamp = ros::Time::now();

    pubCmd.publish(cmdMsg);

    loop_rate.sleep();
  }

  rosThread.join();
  return 0;
}
