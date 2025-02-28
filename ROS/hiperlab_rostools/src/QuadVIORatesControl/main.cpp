#include <iostream>
#include <thread>

#include "ros/ros.h"

#include "ExampleVehicleStateMachine.hpp"

using namespace Offboard;
using namespace std;

shared_ptr<VIOStateEstimator> est;
shared_ptr<MocapStateEstimator> mocap;

// Definitions:
double const mainLoopFrequency = 100;   // Hz
double const systemLatencyTime = 0.0; // latency in measurements & commands[s]

bool useJoystick = true;

bool volatile jsButtonStart = false;
bool volatile jsButtonStop = false;
bool volatile jsButtonGreen = false;
bool volatile jsButtonBlue = false;

void callback_joystick(const hiperlab_rostools::joystick_values &msg)
{
    jsButtonStart = msg.buttonStart > 0;
    jsButtonStop = msg.buttonRed > 0;
    jsButtonGreen = msg.buttonGreen > 0;
    jsButtonBlue = msg.buttonBlue > 0;
}

void rosThreadFn()
{
    // We run this function as a separate thread, allows us to continuously service any subscribers.
    ros::spin();
}

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

    ros::init(argc, argv, "quad_vio_rates_control" + std::to_string(vehicleId));

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

    Vec3d desiredPosition(0, 0, 0.7);
    // Vec3d desiredPosition(0, 0, 1.5);

    double desiredYawAngle = 0 * M_PI / 180.0;

    cout << "Desired position setpoint = <" << desiredPosition.x << "," << desiredPosition.y << "," << desiredPosition.z << ">\n";

    veh.SetDesiredPosition(desiredPosition);
    veh.SetDesiredYaw(desiredYawAngle);

    ros::Rate loop_rate(mainLoopFrequency);

    Timer emergencyTimer(&timer);
    double const EMERGENCY_BUTTON_PERIOD = 0.5; // if you hold the land button down, it triggers an emergency.

    cout << "Starting main controller.\n";

    thread rosThread(rosThreadFn);

    bool firstPanic = true;

    Vec3d initPosition;
    bool shouldQuit = false;

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

        if (emergencyTimer.GetSeconds<double>() > EMERGENCY_BUTTON_PERIOD)
        {
            printf("Panic button!\n");
            veh.SetExternalPanic();
        }

        veh.Run(jsButtonStart, jsButtonStop, jsButtonGreen, jsButtonBlue);

        if (veh.GetIsReadyToExit())
        {
            break;
        }

        loop_rate.sleep();
    }

    ros::shutdown();
    rosThread.join();
    return 0;
}





// #include <iostream>
// #include <thread>
// #include <chrono>

// #include "ros/ros.h"

// #include "ExampleVehicleStateMachine.hpp"

// using namespace Offboard;
// using namespace std;

// shared_ptr<MocapStateEstimator> est;

// // Definitions:
// double const mainLoopFrequency = 100;   // Hz
// double const systemLatencyTime = 30e-3; // latency in measurements & commands[s]
// double const positionHoldTime = 8.0;    // seconds to hold each position

// bool useJoystick = true;

// bool volatile jsButtonStart = false;
// bool volatile jsButtonStop = false;
// void callback_joystick(const hiperlab_rostools::joystick_values &msg)
// {
//     jsButtonStart = msg.buttonStart > 0;
//     jsButtonStop = msg.buttonRed > 0;
// }

// void rosThreadFn()
// {
//     // We run this function as a separate thread, allows us to continuously service any subscribers.
//     ros::spin();
// }

// int main(int argc, char **argv)
// {
//     if (argc < 2)
//     {
//         printf("ERROR: Must specify the vehicle ID\n");
//         return -1;
//     }

//     int const vehicleId = atol(argv[1]);
//     if (vehicleId <= 0 || vehicleId > 255)
//     {
//         printf("ERROR: invalid vehicle ID\n");
//         return -1;
//     }

//     ros::init(argc, argv, "quad_mocap_rates_control" + std::to_string(vehicleId));

//     for (int i = 2; i < argc; i++)
//     {
//         if (!strcmp(argv[i], "--no-js"))
//         {
//             printf("Disabling joystick use.\n");
//             useJoystick = false;
//         }
//     }

//     ros::NodeHandle n;
//     ros::Subscriber subJoystick = n.subscribe("joystick_values", 1, callback_joystick);
//     // Set everything up.
//     HardwareTimer timer;

//     ExampleVehicleStateMachine veh;
//     veh.Initialize(vehicleId, "rates vehicle", n, &timer, systemLatencyTime);

//     // Define the square trajectory corners
//     Vec3d squareCorners[4] = {
//         Vec3d(0, 0, 0.7),
//         Vec3d(0.5, 0, 0.7),
//         Vec3d(0.5, 0.5, 0.7),
//         Vec3d(0, 0.5, 0.7)
//     };

//     // Vec3d squareCorners[4] = {
//     //     Vec3d(-0.25, -0.25, 1.5),
//     //     Vec3d(0.25, -0.25, 1.5),
//     //     Vec3d(0.25, 0.25, 1.5),
//     //     Vec3d(-0.25, 0.25, 1.5)
//     // };

//     //     Vec3d squareCorners[4] = {
//     //     Vec3d(0, 0, 1.55),
//     //     Vec3d(1, 0, 1.55),
//     //     Vec3d(1, 1, 1.55),
//     //     Vec3d(0, 1, 1.55)
//     // };

//     int currentCorner = 0;
//     double desiredYawAngle = 0 * M_PI / 180.0;

//     cout << "Desired position setpoint = <" << squareCorners[currentCorner].x << "," << squareCorners[currentCorner].y << "," << squareCorners[currentCorner].z << ">\n";

//     veh.SetDesiredPosition(squareCorners[currentCorner]);
//     veh.SetDesiredYaw(desiredYawAngle);

//     ros::Rate loop_rate(mainLoopFrequency);

//     Timer emergencyTimer(&timer);
//     double const EMERGENCY_BUTTON_PERIOD = 0.5; // if you hold the land button down, it triggers an emergency.

//     cout << "Starting main controller.\n";

//     thread rosThread(rosThreadFn);

//     bool firstPanic = true;

//     Vec3d initPosition;
//     bool shouldQuit = false;

//     cout << "Waiting for estimator init...\n";
//     while (ros::ok())
//     {
//         loop_rate.sleep();
//         if (veh.GetIsEstInitialized())
//         {
//             break;
//         }
//     }
//     cout << "Est initialized.\n";

//     cout << "Waiting for joystick start button...\n";
//     while (ros::ok())
//     {
//         loop_rate.sleep();
//         if (jsButtonStart)
//         {
//             // force to release button:
//             while (jsButtonStart)
//             {
//                 loop_rate.sleep();
//             }
//             break;
//         }
//     }
//     cout << "Continuing. Hit start again to take off.\n";

//     while (ros::ok() && !shouldQuit)
//     {
//         if (!jsButtonStop)
//         {
//             emergencyTimer.Reset();
//         }

//         if (emergencyTimer.GetSeconds<double>() > EMERGENCY_BUTTON_PERIOD)
//         {
//             printf("Panic button!\n");
//             veh.SetExternalPanic();
//         }

//         veh.Run(jsButtonStart, jsButtonStop);

//         // Hold at the current corner for a specified time
//         static auto positionStartTime = std::chrono::steady_clock::now();
//         auto currentTime = std::chrono::steady_clock::now();
//         double elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - positionStartTime).count();
//         cout <<"Time elapsed :<"<< elapsedTime<<">\n";

//         if (elapsedTime >= positionHoldTime)
//         {   

//             // cout <<"Time elapsed :<"<< elapsedTime<<">\n";
//             currentCorner = (currentCorner + 1) % 4;
//             veh.SetDesiredPosition(squareCorners[currentCorner]);
//             cout << "Moving to next corner: <" << squareCorners[currentCorner].x << "," << squareCorners[currentCorner].y << "," << squareCorners[currentCorner].z << ">\n";
//             positionStartTime = std::chrono::steady_clock::now();
//         }

//         if (veh.GetIsReadyToExit())
//         {
//             break;
//         }

//         loop_rate.sleep();
//     }

//     ros::shutdown();
//     rosThread.join();
//     return 0;
// }