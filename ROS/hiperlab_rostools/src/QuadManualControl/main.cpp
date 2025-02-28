#include <iostream>
#include <thread>

#include "ros/ros.h"

#include "Common/DataTypes/RadioTypes.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Common/Time/Timer.hpp"

#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/joystick_values.h"

using namespace std;

/* Variables */
const double mainLoopFrequency = 100; // Hz
bool shouldStart = false;
bool shouldStop = false;
float joystickValues[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // joystick 4 axis values
const float lateralAccGain = 0.2f * 9.81f;          // lateral acceleration gain
const float verticalAccGain = 0.2f * 9.81f;         // vertical acceleration gain
const float yawRateGain = 1.0f;                     // yaw rate gain
const float neutralThrust = 0.95f * 9.81f;
bool vehicleIsReadyForProgramToExit = false;
enum FlightStage
{
    StageIdle,
    StageFlight,
    StageEmergency,
};
FlightStage flightStage, lastFlightStage;

/* ROS callbacks */
void callback_joystick(const hiperlab_rostools::joystick_values &msg)
{
    shouldStart = msg.buttonStart > 0;
    shouldStop = msg.buttonRed > 0;
    for (int i = 0; i < 4; i++)
    {
        joystickValues[i] = msg.axes[i];
    }
}

/* ROS spinner */
void rosThreadFn()
{
    ros::spin();
}

/* Main loop */
int main(int argc, char **argv)
{
    /* Get vehicle ID */
    if (argc < 2)
    {
        printf("ERROR: Must specify the vehicle ID\n");
        return -1;
    }
    const int vehicleId = atol(argv[1]);
    if (vehicleId <= 0 || vehicleId > 255)
    {
        printf("ERROR: invalid vehicle ID\n");
        return -1;
    }

    /* ROS setup */
    ros::init(argc, argv, "quad_manual_control" + std::to_string(vehicleId));
    ros::NodeHandle n;
    ros::Subscriber subJoystick = n.subscribe("joystick_values", 1, callback_joystick);
    ros::Publisher pubCmd = n.advertise<hiperlab_rostools::radio_command>("radio_command" + std::to_string(vehicleId), 1);

    /* Estimator and controller setup */
    HardwareTimer timer;

    /* Main loop setup */
    ros::Rate loop_rate(mainLoopFrequency);
    Timer emergencyTimer(&timer);
    double const EMERGENCY_BUTTON_PERIOD = 0.5; // if you hold the land button down, it triggers an emergency.

    /* Start ROS spinner */
    thread rosThread(rosThreadFn);

    /* Enter flight loop */
    cout << "Press start to start the vehicle.\n";
    while (ros::ok())
    {
        loop_rate.sleep();
        /* Manual vehicle shutdown */
        if (!shouldStop)
        {
            emergencyTimer.Reset();
        }
        if (emergencyTimer.GetSeconds<double>() > EMERGENCY_BUTTON_PERIOD)
        {
            printf("Panic button!\n");
            flightStage = StageEmergency;
        }

        /* Check for stage change */
        bool stageChange = flightStage != lastFlightStage;
        lastFlightStage = flightStage;

        /* Run the controller */
        hiperlab_rostools::radio_command cmdMsg;
        uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];
        switch (flightStage)
        {
        case StageIdle:
            if (stageChange)
            {
                cout << "Entering idle stage.\n";
            }
            {
                RadioTypes::RadioMessageDecoded::CreateIdleCommand(0, rawMsg);
                for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++)
                {
                    cmdMsg.raw[i] = rawMsg[i];
                }
            }
            if (shouldStart)
            {
                flightStage = StageFlight;
            }
            break;
        case StageFlight:
            if (stageChange)
            {
                cout << "Entering flight stage.\n";
            }
            {
                /* Create the acceleration command based on the joystick values */
                const Vec3f cmdAcc(joystickValues[2] * lateralAccGain, -joystickValues[3] * lateralAccGain, neutralThrust + joystickValues[0] * verticalAccGain - 9.81f);
                const float desYawRate = -joystickValues[1] * yawRateGain;
                /* Print the commanded acceleration and desired yaw rate */
                printf("Commanded acceleration: %f %f %f\n", cmdAcc.x, cmdAcc.y, cmdAcc.z);
                printf("Desired yaw rate: %f\n", desYawRate);
                /* Create the radio command */
                RadioTypes::RadioMessageDecoded::CreateAccelerationCommand(0, cmdAcc, desYawRate, rawMsg);
                for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++)
                {
                    cmdMsg.raw[i] = rawMsg[i];
                }
            }
            if (shouldStop)
            {
                flightStage = StageIdle;
            }
            break;
        default:
        case StageEmergency:
            if (stageChange)
            {
                cout << "Emergency stage!\n";
            }
            {
                RadioTypes::RadioMessageDecoded::CreateKillCommand(0, rawMsg);
                for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++)
                {
                    cmdMsg.raw[i] = rawMsg[i];
                }
            }
            break;
        }
        cmdMsg.header.stamp = ros::Time::now();
        pubCmd.publish(cmdMsg);
    }

    /* Exit */
    ros::shutdown();
    return 0;
}
