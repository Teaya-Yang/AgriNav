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
bool shouldFly = false;
bool shouldDrive = false;
bool shouldTest = false;
float joystickValues[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // joystick 4 axis values
const float lateralAccGain = 0.2f * 9.81f;          // lateral acceleration gain
const float verticalAccGain = 0.2f * 9.81f;         // vertical acceleration gain
const float yawRateGain = 1.0f;                     // yaw rate gain
const float neutralThrust = 0.95f * 9.81f;
int testCounter = 0;
uint16_t testDshotValue = 47;
float desMotorSpeedsDefault[4] = {0, 0, 0, 0};
bool vehicleIsReadyForProgramToExit = false;
enum FlightStage
{
    StageIdle,
    StageFlight,
    StageDrive,
    StageTest,
    StageEmergency,
};
FlightStage flightStage, lastFlightStage;

/* ROS callbacks */
void callback_joystick(const hiperlab_rostools::joystick_values &msg)
{
    shouldStart = msg.buttonStart > 0;
    shouldStop = msg.buttonRed > 0;
    shouldFly = msg.buttonYellow > 0;
    shouldDrive = msg.buttonGreen > 0;
    shouldTest = msg.buttonBlue > 0;
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
    ros::init(argc, argv, "quad_manual_hybrid_control" + std::to_string(vehicleId));
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
                if (shouldFly)
                {
                    flightStage = StageFlight;
                }
                else if (shouldDrive)
                {
                    flightStage = StageDrive;
                }
                else if (shouldTest)
                {
                    flightStage = StageTest;
                }
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
        case StageDrive:
            if (stageChange)
            {
                cout << "Entering drive stage.\n";
            }
            {
                float power[4] = {0, 0, 0, 0}; // 0 mean not spinning, 1 means full speed
                uint16_t base = 1049;
                uint16_t range = 2047 - base;
                // TODO: Compute "power" from joystick values to allow the vehicle to translate and rotate on the ground
                power[0] = joystickValues[0] > 0 ? joystickValues[0] : 0;
                power[1] = joystickValues[1] > 0 ? joystickValues[1] : 0;
                power[2] = joystickValues[2] > 0 ? joystickValues[2] : 0;
                power[3] = joystickValues[3] > 0 ? joystickValues[3] : 0;

                // const float w_i = //idle motor speed
                // const float k_1 = //inverse of wheel radius
                // const float k_2 = //distance of wheel from vehicle centre
                // const float k_3 = //reduction ratio of motor to wheel
                // float v_x = joystickValues[0];
                // float w_z = joystickValues[1];
                // float w_r = joystickValues[2];
                // float w_l = joystickValues[3];
                // power[0] = -k_3 * w_r + w_i;
                // power[1] = k_3 * w_r + w_i;
                // power[2] = k_3 * w_l + w_i;
                // power[3] = -k_3 * w_l + w_i;

                // Do not change anything below this line
                uint16_t dshot_values[4] = {base + range * power[0], base + range * power[1], base + range * power[2], base + range * power[3]};
                RadioTypes::RadioMessageDecoded::CreateSpeedsCommand(1, desMotorSpeedsDefault, rawMsg, dshot_values[0], dshot_values[1], dshot_values[2], dshot_values[3]);
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
        case StageTest:
            if (stageChange)
            {
                testCounter = 0;
                cout << "Entering test stage.\n";
            }
            {
                testCounter++;
                if (testCounter < 100)
                {
                    testDshotValue = 47 + 400;
                }
                else if (testCounter < 200)
                {
                    testDshotValue = 47 + 800;
                }
                else if (testCounter < 300)
                {
                    testDshotValue = 47 + 1200;
                }
                else if (testCounter < 400)
                {
                    testDshotValue = 47 + 800;
                }
                else if (testCounter < 500)
                {
                    testDshotValue = 47 + 400;
                }
                else
                {
                    shouldStop = true;
                }
                uint16_t dshot_values[4] = {testDshotValue, testDshotValue, testDshotValue, testDshotValue};
                printf("dshot_values: %d %d %d %d\n", dshot_values[0], dshot_values[1], dshot_values[2], dshot_values[3]);
                RadioTypes::RadioMessageDecoded::CreateSpeedsCommand(1, desMotorSpeedsDefault, rawMsg, dshot_values[0], dshot_values[1], dshot_values[2], dshot_values[3]);
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
