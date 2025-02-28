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

using namespace std;

bool volatile jsButtonStart = false;
bool volatile jsButtonStop = false;
void callback_joystick(const hiperlab_rostools::joystick_values &msg)
{
  jsButtonStart = msg.buttonStart > 0;
  jsButtonStop = msg.buttonRed > 0;
}

int main(int argc, char **argv)
{

  if (argc < 3)
  {
    printf("ERROR: Must specify the vehicle ID, and the thrust value (in G)\n");
    printf("E.g. rosrun hiperlab_rostools testMotors 13 0.3\n");
    printf("  to run with vehicle 13, at thrust of 0.3\n");
    printf("\n\nMust have the joystick running.\n");
    printf(
        "Usage: Start the program. If you press no joystick button, nothing should happen\n");
    printf(
        "\tAs long as you're holding down the start button, the motors should run.\n");
    printf("\tIf you press the red button, you'll actively kill the vehicle\n");
    return -1;
  }

  int vehicleId = atol(argv[1]);
  if (vehicleId <= 0 || vehicleId > 255)
  {
    printf("ERROR: invalid vehicle ID\n");
    return -1;
  }

  ros::init(argc, argv, "test_motors" + std::to_string(vehicleId));

  ros::NodeHandle n;
  ros::Publisher pubCmd = n.advertise<hiperlab_rostools::radio_command>(
      "radio_command" + std::to_string(vehicleId), 1);
  ros::Subscriber subJoystick = n.subscribe("joystick_values", 1,
                                            callback_joystick);

  double CMD_THRUST_VAL = 9.81;

  CMD_THRUST_VAL = atof(argv[2]) * 9.81;

  // Set everything up.
  HardwareTimer timer;

  double const mainLoopFrequency = 100; // Hz
  ros::Rate loop_rate(mainLoopFrequency);

  printf("Starting mainloop. Vehicle ID = <%d>, thrust value is <%.3f>g\n",
         vehicleId, CMD_THRUST_VAL);
  printf("Hold start js button to run motors, red to kill the vehicle.\n");

  while (ros::ok())
  {
    ros::spinOnce();

    hiperlab_rostools::radio_command cmdMsg;
    cmdMsg.header.stamp = ros::Time::now();
    uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];
    uint8_t flags = 0;

    if (jsButtonStop)
    {
      cout << "KILLING THE VEHICLE!\n";
      RadioTypes::RadioMessageDecoded::CreateKillCommand(flags, rawMsg);
      cmdMsg.debugtype = RadioTypes::emergencyKill;
    }
    else if (!jsButtonStart)
    {
      RadioTypes::RadioMessageDecoded::CreateIdleCommand(flags, rawMsg);
      cmdMsg.debugtype = RadioTypes::idleCommand;
    }
    else
    {
      // run the motors
      Vec3d cmdAngVel(0, 0, 0);
      double cmdThrust(CMD_THRUST_VAL);
      RadioTypes::RadioMessageDecoded::CreateRatesCommand(flags,
                                                          float(cmdThrust),
                                                          Vec3f(cmdAngVel),
                                                          rawMsg);
      cmdMsg.debugtype = RadioTypes::externalRatesCmd;

      cmdMsg.debugvals[0] = float(cmdThrust);
      cmdMsg.debugvals[1] = float(cmdAngVel.x);
      cmdMsg.debugvals[2] = float(cmdAngVel.y);
      cmdMsg.debugvals[3] = float(cmdAngVel.z);
    }
    for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++)
    {
      cmdMsg.raw[i] = rawMsg[i];
    }
    cmdMsg.header.stamp = ros::Time::now();

    pubCmd.publish(cmdMsg);

    loop_rate.sleep();
  }

  return 0;
}
