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

bool volatile jsButtonStop = false;
void callback_joystick(const hiperlab_rostools::joystick_values& msg) {
  jsButtonStop = msg.buttonRed > 0;
}

int main(int argc, char **argv) {

  if (argc < 4 || (argc - 2)%2 != 0) {
    printf("argc = %d\n", argc);
    printf("argc[2] = %7.2f\n", atof(argv[2]));
    printf("ERROR: Must specify the vehicle ID, and the thrust value (in G)\n");
    printf("E.g. rosrun hiperlab_rostools testMotors 13 0.3 0.4 5.0 2.0\n");
    printf("  to run with vehicle 13, at a thrust of 0.3 for 5 seconds, and at a thrust of 0.4 for 2 seconds\n");
    printf("\n\nMust have the joystick running.\n");
    printf(
        "Usage: Start the program. If you press no joystick button, nothing should happen\n");
    printf(
        "\tAs long as you're holding down the start button, the motors should run.\n");
    printf("\tIf you press the red button, you'll actively kill the vehicle\n");
    return -1;
  }

  int vehicleId = atol(argv[1]);
  if (vehicleId <= 0 || vehicleId > 255) {
    printf("ERROR: invalid vehicle ID\n");
    return -1;
  }

  ros::init(argc, argv, "test_motors_individual_timed" + std::to_string(vehicleId));

  ros::NodeHandle n;
  ros::Publisher pubCmd = n.advertise<hiperlab_rostools::radio_command>(
      "radio_command" + std::to_string(vehicleId), 1);
  ros::Subscriber subJoystick = n.subscribe("joystick_values", 1,
                                            callback_joystick);

  int steps = (argc - 2)/2;
  double CMD_MOTOR_FORCES[steps]; // [m/s^2]
  double MOTOR_RUN_TIME[steps]; // [m/s^2]
  double TOTAL_RUN_TIME = 0;
  for (int i = 0; i < steps; i++) {
    CMD_MOTOR_FORCES[i] = atof(argv[2+i]) * 9.81 / 4.0;
  }
  for (int i = 0; i < steps; i++) {
    MOTOR_RUN_TIME[i] = atof(argv[2+steps+i]);
    TOTAL_RUN_TIME += MOTOR_RUN_TIME[i];
  }


  printf("TOTAL_RUN_TIME = %7.2f\n", TOTAL_RUN_TIME);
  double const TIME_TO_START = 1.0; // [s] to wait before running the motors
  double const TIME_TO_STOP = 1.0; // [s] send an idle command for this long before killing

  //Set everything up.
  HardwareTimer timer;

  double const mainLoopFrequency = 50;  //Hz
  ros::Rate loop_rate(mainLoopFrequency);

  printf("Press js red button to kill the vehicle.\n");

  Timer motorTimer(&timer);
  double const startTime = motorTimer.GetSeconds_d();
  double currentTime = 0.0;

  while (ros::ok() && (currentTime < TIME_TO_START + TOTAL_RUN_TIME + TIME_TO_STOP)) {
    ros::spinOnce();
    currentTime = motorTimer.GetSeconds_d() - startTime;

    hiperlab_rostools::radio_command cmdMsg;
    cmdMsg.header.stamp = ros::Time::now();
    uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];
    uint8_t flags = 0;

    if (jsButtonStop) {
      cout << "KILLING THE VEHICLE!\n";
      RadioTypes::RadioMessageDecoded::CreateKillCommand(flags, rawMsg);
      cmdMsg.debugtype = RadioTypes::emergencyKill;
    } else if (currentTime < TIME_TO_START || currentTime > TIME_TO_START + TOTAL_RUN_TIME) {
      RadioTypes::RadioMessageDecoded::CreateIdleCommand(flags, rawMsg);
      cmdMsg.debugtype = RadioTypes::idleCommand;
    } else {
      /* determine which thrust step we are at */
      int currentStep = 0;
      float PAST_RUN_TIME = TIME_TO_START;
      for (int i = 0; i < steps; i++) {
        PAST_RUN_TIME += MOTOR_RUN_TIME[i];
        if (currentTime > PAST_RUN_TIME) {
          currentStep += 1;
        }
      }
      /* use the thrust of the current thrust step */
      float cmdMotorForcesFloat[4];
      for (int i = 0; i < 4; i++) {
        cmdMotorForcesFloat[i] = float(CMD_MOTOR_FORCES[currentStep]);
      }
      /* run the motors */
      RadioTypes::RadioMessageDecoded::CreateMotorForcesCommand(flags,
                                                                cmdMotorForcesFloat,
                                                                rawMsg);
      cmdMsg.debugtype = RadioTypes::externalMotorForcesCmd;

      for (int i = 0; i < 4; i++) {
        cmdMsg.debugvals[i] = cmdMotorForcesFloat[i];
      }
    }
    for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++) {
      cmdMsg.raw[i] = rawMsg[i];
    }
    cmdMsg.header.stamp = ros::Time::now();

    pubCmd.publish(cmdMsg);

    loop_rate.sleep();
  }

  return 0;
}

