#include <iostream>
#include <thread>

#include "ros/ros.h"

#include "ExampleVehicleStateMachine.hpp"

using namespace Offboard;
using namespace std;

shared_ptr<VIOStateEstimator> est;

// Definitions:
double const mainLoopFrequency = 50;   // Hz
double const systemLatencyTime = 0.0; // latency in measurements & commands[s]

bool useJoystick = true;

bool volatile jsButtonStart = false;
bool volatile jsButtonStop = false;
bool volatile jsButtonGreen = false;
bool volatile jsButtonBlue = false;
bool volatile jsButtonYellow = false;

void callback_joystick(const hiperlab_rostools::joystick_values &msg)
{
    jsButtonStart = msg.buttonStart > 0;
    jsButtonStop = msg.buttonRed > 0;
    jsButtonGreen = msg.buttonGreen > 0;
    jsButtonBlue = msg.buttonBlue > 0;
    jsButtonYellow =  msg.buttonYellow >0;
}

// void rosThreadFn()
// {
//     // We run this function as a separate thread, allows us to continuously service any subscribers.
//     ros::spin();
// }

int main(int argc, char **argv)
{

    if (argc < 2)
    {
        printf("ERROR: Must specify the vehicle ID\n");
        return -1;
    }

    int const vehicleId = atol(argv[1]);
    if (vehicleId <= 0 || vehicleId > 255)
    {
        printf("ERROR: invalid vehicle ID\n");
        return -1;
    }

    ros::init(argc, argv, "quad_rappids_rates_control" + std::to_string(vehicleId));

    for (int i = 2; i < argc; i++)
    {
        if (!strcmp(argv[i], "--no-js"))
        {
            printf("Disabling joystick use.\n");
            useJoystick = false;
        }
    }

    ros::NodeHandle n;
    ros::Subscriber subJoystick = n.subscribe("joystick_values", 1, callback_joystick);
    // Set everything up.
    HardwareTimer timer;

    ExampleVehicleStateMachine veh;
    veh.Initialize(vehicleId, "rates vehicle", n, &timer, systemLatencyTime);

    // Vec3d desiredPosition(0, 0, 0.7);
    Vec3d desiredPosition(0, 0, 1.5);

    double desiredYawAngle = 0 * M_PI / 180.0;

    cout << "Desired position setpoint = <" << desiredPosition.x << "," << desiredPosition.y << "," << desiredPosition.z << ">\n";

    veh.SetDesiredPosition(desiredPosition);
    veh.SetDesiredYaw(desiredYawAngle);

    ros::Rate loop_rate(mainLoopFrequency);

    //thread number == number of subscribers
    int thread_number = 6;
    ros::AsyncSpinner spinner(thread_number);

    Timer emergencyTimer(&timer);
    Timer smallEmergencyTimer(&timer);

    double const EMERGENCY_BUTTON_PERIOD = 0.5; // if you hold the land button down, it triggers an emergency.

    cout << "Starting main controller.\n";


    bool firstPanic = true;

    Vec3d initPosition;
    bool shouldQuit = false;

    spinner.start();

    cout << "Waiting for estimator init...\n";
    while (ros::ok())
    {
        loop_rate.sleep();
        if (veh.GetIsEstInitialized())
        {
            break;
        }
    }
    cout << "Est initialized.\n";

    cout << "Waiting for joystick start button...\n";
    while (ros::ok())
    {
        loop_rate.sleep();
        if (jsButtonStart)
        {
            // force to release button:
            while (jsButtonStart)
            {
                loop_rate.sleep();
            }
            break;
        }
    }
    cout << "Continuing. Hit start again to take off.\n";

    while (ros::ok() && !shouldQuit)
    {
        if (!jsButtonStop)
        {
            emergencyTimer.Reset();
        }

        if (!jsButtonYellow)
        {
            smallEmergencyTimer.Reset();
        }

        if (emergencyTimer.GetSeconds<double>() > EMERGENCY_BUTTON_PERIOD)
        {
            printf("Panic button!\n");
            veh.SetExternalPanic();
        }

        if (smallEmergencyTimer.GetSeconds<double>() > EMERGENCY_BUTTON_PERIOD)
        {
            printf("Soft Panic button!\n");
            veh.EnterCrashingStage();
        }

        veh.Run(jsButtonStart, jsButtonStop, jsButtonGreen, jsButtonBlue, jsButtonYellow);

        if (veh.GetIsReadyToExit())
        {
            break;
        }

        loop_rate.sleep();
    }

    spinner.stop();
    ros::shutdown();
    return 0;
}




