#include <iostream>
#include <thread>

#include "ros/ros.h"

#include "VehicleStateMachine_UWBDemo.hpp"
#include "Common/Misc/TerminalColors.hpp"

using namespace Offboard;
using namespace std;

shared_ptr<MocapStateEstimator> est;

//Definitions:
double const mainLoopFrequency = 50;  //Hz
double const systemLatencyTime = 30e-3;  // latency in measurements & commands[s]

double const DES_HEIGHT = 1.5;
double const CIRCLE_RADIUS = 1.0;  //[m]
double const CIRCLE_ANG_VEL = 0.5;  //[rad/s]
Vec3d const CIRCLE_CENTER = Vec3d(0, 0, DES_HEIGHT);  //[rad/s]

Vec3d MIN_SAFE_CORNER(-2, -2, -0.5);
Vec3d MAX_SAFE_CORNER(+2, +2, +4.0);
double MIN_SAFE_HEIGHT(1);

bool useJoystick = true;

bool volatile jsButtonStart = false;
bool volatile jsButtonStop = false;
bool volatile jsButtonGreen = false;
bool volatile jsButtonYellow = false;
bool volatile jsButtonBlue = false;
void callback_joystick(const hiperlab_rostools::joystick_values& msg) {
  jsButtonStart = msg.buttonStart > 0;
  jsButtonStop = msg.buttonRed > 0;
  jsButtonGreen = msg.buttonGreen;
  jsButtonYellow = msg.buttonYellow;
  jsButtonBlue = msg.buttonBlue;
}

void rosThreadFn() {
  // We run this function as a separate thread, allows us to continuously service any subscribers.
  ros::spin();
}

int main(int argc, char **argv) {

  int vehIDUWB, vehIDMocap;
  if (argc < 3) {
    cout << "ERROR: Did not specify vehicle IDs, must specify two.\n";
    cout << "First is UWB, second is mocap only.";
    cout << "Can set mocap to zero to disable.\n";
    cout << "Add `--no-js` to disable the joystick automatically start.\n";
    cout << "Add `--uwb` to immediately start with UWB.\n";
    return -1;
  }

  vehIDUWB = atol(argv[1]);
  if (vehIDUWB <= 0 || vehIDUWB > 255) {
    printf("ERROR: invalid vehicle ID for UWB quad\n");
    return -1;
  }

  vehIDMocap = atol(argv[2]);
  if (vehIDMocap < 0 || vehIDMocap > 255) {
    printf("ERROR: invalid vehicle ID for mocap quad\n");
    return -1;
  }

  if (vehIDMocap == 0) {
    cout << "-> Disabled mocap quad.\n";
  }

  bool disableJs = false;
  bool forceUseUWB = false;
  if (argc > 3) {
    for (int i = 3; i < argc; i++) {
      if (!strcmp(argv[i], "--no-js")) {
        cout << "Disabling joystick!\n";
        disableJs = true;
        continue;
      }

      if (!strcmp(argv[i], "--uwb")) {
        cout << "Forcing use of UWB!\n";
        forceUseUWB = true;
        continue;
      }
      cout << "Unknown option at " << int(i) << "<" << argv[i] << ">\n";
    }
  }

  ros::init(
      argc,
      argv,
      "demo_ultra_wideband" + std::to_string(vehIDUWB) + "_"
          + std::to_string(vehIDMocap));

  ros::NodeHandle n;
  ros::Subscriber subJoystick = n.subscribe("joystick_values", 1,
                                            callback_joystick);
  //Set everything up.
  HardwareTimer timer;

  VehicleStateMachine_UWBDemo vehUWB, vehMocap;
  vehUWB.Initialize(vehIDUWB, "uwb", n, &timer, systemLatencyTime);
  if (vehIDMocap) {
    vehMocap.Initialize(vehIDMocap, "mocap", n, &timer, systemLatencyTime);
  }

  Vec3d desiredPosition(0, 0, DES_HEIGHT);

  vehUWB.SetDesiredYaw(0);
  vehUWB.SetDesiredPosition(Vec3d(0, +1.0, DES_HEIGHT));
  vehUWB.SetCircleTrajectoryParameters(CIRCLE_CENTER, CIRCLE_ANG_VEL,
                                       CIRCLE_RADIUS, 1.0 / mainLoopFrequency);
  vehUWB.SetSafeCorners(MIN_SAFE_CORNER, MAX_SAFE_CORNER, MIN_SAFE_HEIGHT);

  if (vehIDMocap) {
    vehMocap.SetDesiredPosition(Vec3d(0, -1.0, DES_HEIGHT));
    vehMocap.SetDesiredYaw(0);
    vehMocap.SetSafeCorners(MIN_SAFE_CORNER, MAX_SAFE_CORNER, MIN_SAFE_HEIGHT);
    vehMocap.SetCircleTrajectoryParameters(CIRCLE_CENTER, CIRCLE_ANG_VEL,
                                           CIRCLE_RADIUS,
                                           1.0 / mainLoopFrequency);
  }

  ros::Rate loop_rate(mainLoopFrequency);

  Timer emergencyTimer(&timer);
  double const EMERGENCY_BUTTON_PERIOD = 0.5;  //if you hold the land button down, it triggers an emergency.

  cout << "Starting main controller.\n";

  thread rosThread(rosThreadFn);

  bool firstPanic = true;

  Vec3d initPosition;

  cout << "Waiting for estimator init...\n";
  while (ros::ok()) {
    loop_rate.sleep();
    bool allInit = true;
    allInit &= vehUWB.GetIsEstInitialized();
    if (vehIDMocap) {
      allInit &= vehMocap.GetIsEstInitialized();
    }
    if (allInit) {
      break;
    }
  }
  cout << "Est initialized.\n";

  if (disableJs) {
    jsButtonStart = true;
  } else {
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
    cout << "Continuing.\n";
  }

  cout << "\n----------------------------------------------";
  cout << "\nHit :";
  TerminalColors::SetTerminalColor(TerminalColors::YELLOW);
  cout << "\t[Yellow] to use UWB\n";
  TerminalColors::SetTerminalColor(TerminalColors::GREEN);
  cout << "\t[Green] to use Mocap\n";
  TerminalColors::SetTerminalColor(TerminalColors::BLUE);
  cout << "\t[Blue] to toggle flying circle\n";
  TerminalColors::SetTerminalColor(TerminalColors::RED);
  cout << "\t[Red] to land (hold to panic)\n";
  TerminalColors::ResetTerminalColor();
  cout << "\n----------------------------------------------";
  cout << "\n\n";

  double periodPrintUWBPerf = 1.0;
  Timer tPrint(&timer);

  while (ros::ok()) {
    if (!jsButtonStop) {
      emergencyTimer.Reset();
    }

    if (emergencyTimer.GetSeconds<double>() > EMERGENCY_BUTTON_PERIOD) {
      printf("Panic button!\n");
      vehUWB.SetExternalPanic();
      if (vehIDMocap) {
        vehMocap.SetExternalPanic();
      }
    }

    vehUWB.Run(jsButtonStart, jsButtonStop, jsButtonGreen, jsButtonYellow,
               jsButtonBlue);
    if (vehIDMocap) {
      vehMocap.Run(jsButtonStart, jsButtonStop, false, false, jsButtonBlue);
    }

    bool allReadyToExit = true;
    allReadyToExit &= vehUWB.GetIsReadyToExit();
    if (vehIDMocap) {
      allReadyToExit &= vehMocap.GetIsReadyToExit();
    }
    if (allReadyToExit) {
      break;
    }

    if (tPrint.GetSeconds_d() > periodPrintUWBPerf) {
      vehUWB.PrintEstimationError();
      tPrint.Reset();
    }

    loop_rate.sleep();
  }

  ros::shutdown();
  rosThread.join();
  return 0;
}
