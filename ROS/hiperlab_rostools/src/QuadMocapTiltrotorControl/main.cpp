#include <iostream>
#include <thread>

#include "ros/ros.h"

#include "Common/DataTypes/RadioTypes.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Offboard/MocapStateEstimator.hpp"
#include "Components/Offboard/SafetyNet.hpp"
#include "Components/Logic/QuadcopterConstants.hpp"
#include "Components/Logic/TiltrotorController.hpp"

#include "hiperlab_rostools/mocap_output.h"
#include "hiperlab_rostools/estimator_output.h"
#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/joystick_values.h"

using namespace std;

/* Variables */
const double mainLoopFrequency = 100;   // Hz
const double systemLatencyTime = 30e-3; // latency in measurements & commands[s]
bool shouldStart = false;
bool shouldStop = false;
float joystickValues[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // joystick 4 axis values
bool vehicleIsReadyForProgramToExit = false;
enum FlightStage
{
    StageWaitForStart,
    StageSpoolUp,
    StageTakeoff,
    StageFlight,
    StageLanding,
    StageComplete,
    StageEmergency,
};
FlightStage flightStage, lastFlightStage;

// States
Vec3d desPos(0, 0, 2);
Vec3d initPos(0, 0, 0);
float desYaw = 0 * M_PI / 180.0;
float desPitch = 0 * M_PI / 180.0;
float pitchLimit[2] = {-90 * M_PI / 180.0, 15 * M_PI / 180.0};
float xLimit[2] = {-0.5, 0.5};
float maxPitchRate = 1; // [rad/s]

/* Classes */
shared_ptr<Offboard::MocapStateEstimator> est;
Offboard::SafetyNet safetyNet;
TiltrotorController tiltrotorController;

/* Functions */
void cap(float &val, float limit[2])
{
    if (val < limit[0])
    {
        val = limit[0];
    }
    else if (val > limit[1])
    {
        val = limit[1];
    }
}

/* ROS callbacks */
void callback_joystick(const hiperlab_rostools::joystick_values &msg)
{
    shouldStart = msg.buttonStart > 0;
    shouldStop = msg.buttonRed > 0;
    for (int i = 0; i < 4; i++)
    {
        joystickValues[i] = msg.axes[i];
    }
    /* Convert the first axis to desired pitch angle */
    float desPitchRaw = desPitch + joystickValues[0] * maxPitchRate / mainLoopFrequency;
    cap(desPitchRaw, pitchLimit);
    desPitch = desPitchRaw;
    /* Convert the third axis to desired x position */
    float desXRaw = joystickValues[2] * 0.5;
    cap(desXRaw, xLimit);
    desPos.x = desXRaw;
}

void callback_estimator(const hiperlab_rostools::mocap_output &msg)
{
    if (est->GetID() == msg.vehicleID)
    {
        est->UpdateWithMeasurement(Vec3d(msg.posx, msg.posy, msg.posz), Rotationd(msg.attq0, msg.attq1, msg.attq2, msg.attq3));
    }
}

/* ROS spinner */
void rosThreadFn()
{
    ros::spin();
}

/* Rates controller */
hiperlab_rostools::radio_command RunControllerAndUpdateEstimator(Offboard::MocapStateEstimator::MocapEstimatedState estState, TiltrotorController tiltrotorController, Vec3d desPos, Vec3d desVel, float desPitch, float desYaw)
{
    // Get states
    Vec3d curPos = estState.pos;
    Vec3d curVel = estState.vel;
    Rotationd curAtt = estState.att;

    // Run controller
    Vec3d desAcc = tiltrotorController.get_acceleration(desPos, curPos, desVel, curVel);
    Vec3d desThrust = tiltrotorController.get_thrust(desAcc);
    tiltrotorController.get_angular_velocity(desThrust, curAtt, desPitch, desYaw);

    // Update the estimator
    est->SetPredictedValues(tiltrotorController.desAngVel, desAcc);

    // Print the variables
    // printf("Desired thrust: %f %f %f\n", desThrust.x, desThrust.y, desThrust.z);
    // printf("Desired pitch angle: %f\n", desPitch);
    // printf("Desired yaw angle: %f\n", desYaw);
    printf("Desired norm thrust: %f\n", tiltrotorController.desThrustNorm);
    printf("Desired angular velocity: %f %f %f\n", tiltrotorController.desAngVel.x, tiltrotorController.desAngVel.y, tiltrotorController.desAngVel.z);
    printf("Net tilt angle: %f\n", tiltrotorController.netTiltAngle);
    // printf("Desired x position: %f\n", desPos.x);

    // Create and return the radio command
    uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];
    RadioTypes::RadioMessageDecoded::CreateTiltrotorCommand(0, tiltrotorController.desThrustNorm, tiltrotorController.desAngVel, tiltrotorController.netTiltAngle, rawMsg);
    hiperlab_rostools::radio_command cmdMsg;
    for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++)
    {
        cmdMsg.raw[i] = rawMsg[i];
    }
    return cmdMsg;
}

/* Main loop */
int main(int argc, char **argv)
{
    /* Get vehicle ID */
    if (argc < 2)
    {
        printf("ERROR: Must specify the vehicle ID and tilt controller type\n");
        return -1;
    }
    const int vehicleId = atol(argv[1]);
    if (vehicleId <= 0 || vehicleId > 255)
    {
        printf("ERROR: invalid vehicle ID\n");
        return -1;
    }

    /* ROS setup */
    ros::init(argc, argv, "quad_mocap_tiltrotor_control" + std::to_string(vehicleId));
    ros::NodeHandle n;
    ros::Subscriber subMocap = n.subscribe("mocap_output" + std::to_string(vehicleId), 1, callback_estimator);
    ros::Subscriber subJoystick = n.subscribe("joystick_values", 1, callback_joystick);
    ros::Publisher pubEstimate = n.advertise<hiperlab_rostools::estimator_output>("estimator" + std::to_string(vehicleId), 1);
    ros::Publisher pubCmd = n.advertise<hiperlab_rostools::radio_command>("radio_command" + std::to_string(vehicleId), 1);

    /* Estimator and controller setup */
    HardwareTimer timer;
    est.reset(new Offboard::MocapStateEstimator(&timer, vehicleId, systemLatencyTime));
    Onboard::QuadcopterConstants::QuadcopterType quadcopterType = Onboard::QuadcopterConstants::GetVehicleTypeFromID(vehicleId);
    Onboard::QuadcopterConstants consts(quadcopterType);
    // Tiltrotor controller setup
    float natFreq = consts.posControl_natFreq;
    float dampingRatio = consts.posControl_damping;
    float timeConstAngleRP = consts.attControl_timeConst_xy;
    float timeConstAngleY = consts.attControl_timeConst_z;
    float timeConstRatesRP = consts.angVelControl_timeConst_xy;
    float timeConstRatesY = consts.angVelControl_timeConst_z;
    float mass = consts.mass;
    Matrix<float, 3, 3> inertiaMatrix = ZeroMatrix<float, 3, 3>();
    inertiaMatrix(0, 0) = consts.Ixx; inertiaMatrix(1, 1) = consts.Iyy; inertiaMatrix(2, 2) = consts.Izz;
    float lx = consts.lx;
    float ly = consts.ly;
    float thrustToTorque = consts.propellerTorqueFromThrust * consts.prop0SpinDir; 
    tiltrotorController = TiltrotorController(natFreq, dampingRatio, timeConstAngleRP, timeConstAngleY, timeConstRatesRP, timeConstRatesY, mass, inertiaMatrix, lx, ly, thrustToTorque);

    /* Main loop setup */
    ros::Rate loop_rate(mainLoopFrequency);
    Timer emergencyTimer(&timer);
    double const EMERGENCY_BUTTON_PERIOD = 0.5; // if you hold the land button down, it triggers an emergency.

    /* Start ROS spinner */
    thread rosThread(rosThreadFn);

    /* Wait for Mocap */
    cout << "Waiting for estimator init...\n";
    while (ros::ok())
    {
        loop_rate.sleep();
        if (est->GetIsInitialized())
        {
            break;
        }
    }
    cout << "Est initialized.\n";

    /* Wait for joystick start button */
    cout << "Waiting for joystick start button...\n";
    while (ros::ok())
    {
        loop_rate.sleep();
        if (shouldStart)
        {
            // force to release button:
            while (shouldStart)
            {
                loop_rate.sleep();
            }
            break;
        }
    }
    cout << "Continuing. Hit start again to take off.\n";

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
            safetyNet.SetUnsafe();
        }

        /* Update estimator */
        Offboard::MocapStateEstimator::MocapEstimatedState estState = est->GetPrediction(systemLatencyTime);
        safetyNet.UpdateWithEstimator(estState, est->GetTimeSinceLastGoodMeasurement());

        /* Publish estimator output */
        hiperlab_rostools::estimator_output estOutMsg;
        estOutMsg.header.stamp = ros::Time::now();
        estOutMsg.vehicleID = est->GetID();
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

        /* Update controller */
        if (!safetyNet.GetIsSafe())
        {
            flightStage = StageEmergency;
        }
        bool stageChange = flightStage != lastFlightStage;
        lastFlightStage = flightStage;
        if (stageChange)
        {
            stageTimer.Reset();
        }
        hiperlab_rostools::radio_command cmdMsg;
        uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];
        switch (flightStage)
        {
        case StageWaitForStart:
            if (stageChange)
            {
                cout << "Waiting for start signal.\n";
            }
            if (shouldStart)
            {
                flightStage = StageSpoolUp;
            }
            break;

        case StageSpoolUp:
            if (stageChange)
            {
                cout << "Spooling up motors.\n";
            }
            {
                /* Vehicle will not move, so set predicted values to zero */
                est->SetPredictedValues(Vec3d(0, 0, 0), Vec3d(0, 0, 0));
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
                initPos = estState.pos;
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
                cmdMsg = RunControllerAndUpdateEstimator(estState, tiltrotorController, cmdPos, Vec3d(0, 0, 0), desPitch, desYaw);
            }
            break;

        case StageFlight:
            if (stageChange)
            {
                cout << "Entering flight stage.\n";
            }
            cmdMsg = RunControllerAndUpdateEstimator(estState, tiltrotorController, desPos, Vec3d(0, 0, 0), desPitch, desYaw);

            if (shouldStop)
            {
                flightStage = StageLanding;
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
                if (cmdPos.z < 0)
                {
                    flightStage = StageComplete;
                }
                cmdMsg = RunControllerAndUpdateEstimator(estState, tiltrotorController, cmdPos, Vec3d(0, 0, -LANDING_SPEED), desPitch, desYaw);
            }
            break;

        case StageComplete:
            if (stageChange)
            {
                cout << "Landing complete. Idling.\n";
            }

            {
                est->SetPredictedValues(Vec3d(0, 0, 0), Vec3d(0, 0, 0));
                RadioTypes::RadioMessageDecoded::CreateIdleCommand(0, rawMsg);
                for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++)
                {
                    cmdMsg.raw[i] = rawMsg[i];
                }
            }

            if (stageTimer.GetSeconds<double>() > 1.0)
            {
                cout << "Exiting.\n";
                vehicleIsReadyForProgramToExit = true;
            }
            break;

        default:
        case StageEmergency:
            if (stageChange)
            {
                cout << "Emergency stage! Safety net = <"
                     << safetyNet.GetStatusString() << ">.\n";
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

        /* Check if vehicle is ready to exit */
        if (vehicleIsReadyForProgramToExit)
        {
            break;
        }
    }

    /* Exit */
    ros::shutdown();
    return 0;
}
