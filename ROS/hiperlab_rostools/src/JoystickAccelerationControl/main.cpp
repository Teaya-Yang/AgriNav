#include <iostream>
#include <thread>
#include <stdint.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "hiperlab_rostools/mocap_output.h"
#include "hiperlab_rostools/estimator_output.h"
#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/joystick_values.h"

#include "Common/DataTypes/RadioTypes.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Common/Time/Timer.hpp"

using namespace std;

double const CMD_MAX_THRUST_VAL = 9.81 * 2.0;
double const CMD_MAX_YAW_RATE = 5;  //rad/s
double const JS_THRUST_ZERO_VAL = 0.2;

bool volatile jsButtonStart = false;
bool volatile jsButtonStop = false;
bool volatile haveReceivedJoystick = false;
double volatile jsThrust(0), jsYaw(0), js_x(0), js_y(0);
void callback_joystick(const hiperlab_rostools::joystick_values& msg) {
  jsButtonStart = msg.buttonStart > 0;
  jsButtonStop = msg.buttonRed > 0;

  jsThrust = msg.axes[0];
  jsYaw = -msg.axes[1];
  js_x = msg.axes[2];
  js_y = -msg.axes[3];
  haveReceivedJoystick = true;
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

  ros::init(argc, argv, "test_motors" + std::to_string(vehicleId));

  ros::NodeHandle n;
  ros::Publisher pubCmd = n.advertise<hiperlab_rostools::radio_command>(
      "radio_command" + std::to_string(vehicleId), 1);
  ros::Subscriber subJoystick = n.subscribe("joystick_values", 1,
                                            callback_joystick);

  HardwareTimer timer;

  double const mainLoopFrequency = 50;  //Hz

  ros::AsyncSpinner spinner(0);
  spinner.start();
  printf("Waiting for joystick messages:");
  while (!haveReceivedJoystick) {
    sleep(1);
    printf(".");
    if (!ros::ok()) {
      return -1;
    }
  }
  printf("\nJoystick OK, continuing.\n");

  const int START_TIME = 3;
  printf("To start: release <start> button, then hold it for %d seconds.\n",
         START_TIME);
  Timer startTimer(&timer);
  while (startTimer.GetSeconds<double>() < START_TIME) {
    if (!jsButtonStart) {
      startTimer.Reset();
    }
    if (!ros::ok()) {
      return -1;
    }
    usleep(10 * 1000);
  }
  printf("Starting\n");

  ros::Rate loop_rate(mainLoopFrequency);
  while (ros::ok()) {
//    printf("%f,\t%f,\t%f,\t%f\r", cmdAcc.x, cmdAcc.y, cmdAcc.z, cmdYawRate);

    uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];
    uint8_t flags = 0;
    hiperlab_rostools::radio_command cmdMsg;
    cmdMsg.header.stamp = ros::Time::now();

    if (jsThrust > JS_THRUST_ZERO_VAL) {
      Vec3d cmdAcc = Vec3d(js_x, js_y, 1) * CMD_MAX_THRUST_VAL
          * (jsThrust - JS_THRUST_ZERO_VAL) / (1 - JS_THRUST_ZERO_VAL)
          - Vec3d(0, 0, 9.81);
      double cmdYawRate = CMD_MAX_YAW_RATE * jsYaw;

      RadioTypes::RadioMessageDecoded::CreateAccelerationCommand(
          flags, Vec3f(cmdAcc), float(cmdYawRate), rawMsg);
      cmdMsg.debugvals[0] = float(cmdYawRate);
      cmdMsg.debugvals[1] = float(cmdAcc.x);
      cmdMsg.debugvals[2] = float(cmdAcc.y);
      cmdMsg.debugvals[3] = float(cmdAcc.z);
      cmdMsg.debugtype = RadioTypes::externalAccelerationCmd;
    } else {
      RadioTypes::RadioMessageDecoded::CreateIdleCommand(flags, rawMsg);

      cmdMsg.debugvals[0] = 0;
      cmdMsg.debugvals[1] = 0;
      cmdMsg.debugvals[2] = 0;
      cmdMsg.debugvals[3] = 0;
      cmdMsg.debugtype = RadioTypes::idleCommand;
    }

    for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++) {
      cmdMsg.raw[i] = rawMsg[i];
    }
    pubCmd.publish(cmdMsg);

    RadioTypes::RadioMessageDecoded sent(rawMsg);

    loop_rate.sleep();
  }

  return 0;
}
