#include <iostream>
#include <thread>

#include "ros/ros.h"

#include "Common/DataTypes/RadioTypes.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Common/Time/Timer.hpp"

#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/telemetry.h"
#include "hiperlab_rostools/joystick_values.h"

using namespace std;

/* Variables */
const double mainLoopFrequency = 100; // Hz
bool shouldStart = false;
bool shouldStop = false;
bool shouldAuto = false;
bool shouldManual = false;
bool vehicleIsReadyForProgramToExit = false;
enum FlightStage
{
    StageWaitForStart,
    StageSpoolUp,
    StageTakeoff,
    StageFlight,
    StageManual,
    StageLanding,
    StageComplete,
    StageEmergency,
};
FlightStage flightStage, lastFlightStage;

/* Set the desired position and yaw angle */
Vec3d desPos(0, 0, 2);
Vec3d initPos(0, 0, 0);
double desYaw = 0 * M_PI / 180.0;
double initYaw = 0 * M_PI / 180.0;
// acceleration control gains
const float horizontalAccGain = 0.2f * 9.81f;          // lateral acceleration gain
const float verticalAccGain = 0.2f * 9.81f;         // vertical acceleration gain
const float yawRateGain = 1.0f;                     // yaw rate gain
const float neutralThrust = 0.95f * 9.81f;
// position control gains
const float horizontalPosGain = 1.0f / mainLoopFrequency;
const float verticalPosGain = 1.0f / mainLoopFrequency;
const float yawGain = 1.0f / mainLoopFrequency;
float joystickValues[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // joystick 4 axis values

/* ROS and hiperlab messages */
hiperlab_rostools::telemetry telMsg;

/* ROS callbacks */
void callback_joystick(const hiperlab_rostools::joystick_values &msg)
{
    shouldStart = msg.buttonStart > 0;
    shouldStop = msg.buttonRed > 0;
    shouldAuto = msg.buttonYellow > 0;
    shouldManual = msg.buttonGreen > 0;
    for (int i = 0; i < 4; i++)
    {
        joystickValues[i] = msg.axes[i];
    }
}

void callback_telemetry(const hiperlab_rostools::telemetry &msg)
{
    telMsg = msg;
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
    ros::init(argc, argv, "quad_outdoor_control" + std::to_string(vehicleId));
    ros::NodeHandle n;
    ros::Subscriber subJoystick = n.subscribe("joystick_values", 1, callback_joystick);
    ros::Subscriber subTelemetry = n.subscribe("telemetry" + std::to_string(vehicleId), 10, callback_telemetry);
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
    Timer stageTimer(&timer);
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
        if (stageChange)
        {
            stageTimer.Reset();
        }

        /* Run the controller */
        hiperlab_rostools::radio_command cmdMsg;
        uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];
        switch (flightStage)
        {
        case StageWaitForStart:
            if (stageChange)
            {
                cout << "Press start and select mode (yellow: auto, green: manual) to start the vehicle.\n";
            }
            if (shouldStart)
            {
                if (shouldAuto)
                {
                    flightStage = StageSpoolUp;
                }
                else if (shouldManual)
                {
                    flightStage = StageManual;
                }
            }
            break;
        case StageSpoolUp:
            if (stageChange)
            {
                cout << "Spooling up motors.\n";
            }
            {
                /* Set motor speeds */
                const double motorSpoolUpTime = 0.5; //[s]
                const double spoolUpSpeed = 200;     //[rad/s]
                float desMotorSpeeds[4] = {spoolUpSpeed, spoolUpSpeed, spoolUpSpeed, spoolUpSpeed};
                RadioTypes::RadioMessageDecoded::CreateSpeedsCommand(0, desMotorSpeeds, rawMsg);
                for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++)
                {
                    cmdMsg.raw[i] = rawMsg[i];
                }
                /* Wait for motors to spool up */
                if (stageTimer.GetSeconds<double>() > motorSpoolUpTime)
                {
                    flightStage = StageTakeoff;
                }
            }
            break;
        case StageTakeoff:
            if (stageChange)
            {
                cout << "Taking off.\n";
                initPos = Vec3d(telMsg.position[0], telMsg.position[1], telMsg.position[2]);
                initYaw = telMsg.attitudeYPR[0];
                desPos += initPos;
                desYaw += initYaw; 
            }
            {
                double const takeOffTime = 2.0; //[s]
                double frac = stageTimer.GetSeconds<double>() / takeOffTime;
                if (frac >= 1.0)
                {
                    flightStage = StageFlight;
                    frac = 1.0;
                }
                Vec3d cmdPos = (1 - frac) * initPos + frac * desPos;
                double cmdYaw = (1 - frac) * initYaw + frac * desYaw;
                RadioTypes::RadioMessageDecoded::CreatePositionCommand(0, cmdPos, Vec3d(0, 0, 0), Vec3d(0, 0, 0), cmdYaw, rawMsg);
            }
            if (shouldManual)
            {
                flightStage = StageManual;
            }
            break;
        case StageFlight:
            if (stageChange)
            {
                cout << "Entering flight stage.\n";
            }
            desPos.x += joystickValues[2] * horizontalPosGain;
            desPos.y -= joystickValues[3] * horizontalPosGain;
            desPos.z += joystickValues[0] * verticalPosGain;
            desYaw += joystickValues[1] * yawGain;
            printf("Desired position: %f %f %f\n", desPos.x, desPos.y, desPos.z);
            printf("Desired yaw: %f\n", desYaw);
            RadioTypes::RadioMessageDecoded::CreatePositionCommand(0, desPos, Vec3d(0, 0, 0), Vec3d(0, 0, 0), desYaw, rawMsg);
            if (shouldStop)
            {
                flightStage = StageLanding;
            }
            if (shouldManual)
            {
                flightStage = StageManual;
            }
            break;

        case StageLanding:
            if (stageChange)
            {
                cout << "Starting landing.\n";
            }
            {
                double const LANDING_SPEED = 0.5; // m/s
                Vec3d cmdPos = desPos + stageTimer.GetSeconds<double>() * Vec3d(0, 0, -LANDING_SPEED);
                if (cmdPos.z < initPos.z)
                {
                    flightStage = StageComplete;
                }
                RadioTypes::RadioMessageDecoded::CreatePositionCommand(0, cmdPos, Vec3d(0, 0, -LANDING_SPEED), Vec3d(0, 0, 0), desYaw, rawMsg);
            }
            if (shouldManual)
            {
                flightStage = StageManual;
            }
            break;

        case StageManual:
            if (stageChange)
            {
                cout << "Entering manual stage.\n";
            }
            {
                /* Create the acceleration command based on the joystick values */
                const Vec3f cmdAcc(joystickValues[2] * horizontalAccGain, -joystickValues[3] * horizontalAccGain, neutralThrust + joystickValues[0] * verticalAccGain - 9.81f);
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
                flightStage = StageComplete;
            }
            if (shouldAuto)
            {
                desPos = Vec3d(telMsg.position[0], telMsg.position[1], telMsg.position[2]);
                desYaw = telMsg.attitudeYPR[0];
                flightStage = StageFlight;
            }
            break;

        case StageComplete:
            if (stageChange)
            {
                cout << "Landing complete. Idling.\n";
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
                if (shouldAuto)
                {
                    flightStage = StageSpoolUp;
                }
                else if (shouldManual)
                {
                    flightStage = StageManual;
                }
            }

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
