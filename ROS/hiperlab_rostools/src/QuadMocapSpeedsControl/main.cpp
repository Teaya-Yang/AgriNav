#include <iostream>
#include <thread>

#include "ros/ros.h"

#include "Common/DataTypes/RadioTypes.hpp"
#include "Common/DataTypes/TelemetryPacket.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Offboard/MocapStateEstimator.hpp"
#include "Components/Offboard/QuadcopterController.hpp"
#include "Components/Offboard/SafetyNet.hpp"
#include "Components/Logic/QuadcopterConstants.hpp"
#include "Components/Logic/QuadcopterAngularVelocityController.hpp"
#include "Components/Logic/QuadcopterMixer.hpp"

#include "hiperlab_rostools/mocap_output.h"
#include "hiperlab_rostools/estimator_output.h"
#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/joystick_values.h"
#include "hiperlab_rostools/telemetry.h"

using namespace std;

/* Variables */
const double mainLoopFrequency = 100;   // Hz
const double systemLatencyTime = 30e-3; // latency in measurements & commands[s]
bool shouldStart = false;
bool shouldStop = false;
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

/* Set the desired position and yaw angle */
Vec3d desiredPosition(0, 0, 1.5);
Vec3d initPosition(0, 0, 0);
double desiredYawAngle = 0 * M_PI / 180.0;

/* Classes */
shared_ptr<Offboard::MocapStateEstimator> est;
Offboard::QuadcopterController ctrl;
Offboard::SafetyNet safetyNet;
Onboard::QuadcopterAngularVelocityController angVelCtrl;
Onboard::QuadcopterMixer mixer;

/* ROS and hiperlab messages */
hiperlab_rostools::telemetry telMsg;

/* ROS callbacks */
void callback_joystick(const hiperlab_rostools::joystick_values &msg)
{
    shouldStart = msg.buttonStart > 0;
    shouldStop = msg.buttonRed > 0;
}

void callback_estimator(const hiperlab_rostools::mocap_output &msg)
{
    if (est->GetID() == msg.vehicleID)
    {
        est->UpdateWithMeasurement(Vec3d(msg.posx, msg.posy, msg.posz), Rotationd(msg.attq0, msg.attq1, msg.attq2, msg.attq3));
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

/* Rates controller */
hiperlab_rostools::radio_command RunControllerAndUpdateEstimator(Offboard::MocapStateEstimator::MocapEstimatedState estState, Vec3d desPos, Vec3d desVel, Vec3d desAcc, float mass)
{
    /* Run the rates controller */
    Vec3d cmdAngVel;
    double cmdThrust;
    ctrl.Run(estState.pos, estState.vel, estState.att, desPos, desVel, desAcc, desiredYawAngle, cmdAngVel, cmdThrust);

    /* Update the estimator with the predicted values */
    est->SetPredictedValues(cmdAngVel, (estState.att * Vec3d(0, 0, 1) * cmdThrust - Vec3d(0, 0, 9.81)));

    /* Run the mixer */
    float desMotorForces[4];
    float desMotorSpeeds[4];
    Vec3f estAngVel = Vec3f(telMsg.rateGyro[0], telMsg.rateGyro[1], telMsg.rateGyro[2]);
    float totalNormThrust = cmdThrust;
    Vec3f desAngVel = Vec3f(cmdAngVel.x, cmdAngVel.y, cmdAngVel.z);
    const Vec3f desTorque = angVelCtrl.GetDesiredTorques(desAngVel, estAngVel);
    mixer.GetMotorForces(totalNormThrust * mass, desTorque, desMotorForces);
    mixer.PropellerSpeedsFromThrust(desMotorForces, desMotorSpeeds);
    /* Print the variables */
    printf("Commanded thrust: %f\n", cmdThrust);
    printf("Desired angular velocity: %f %f %f\n", desAngVel.x, desAngVel.y, desAngVel.z);
    printf("Desired torque: %f %f %f\n", desTorque.x, desTorque.y, desTorque.z);
    printf("Desired motor forces: %f %f %f %f\n", desMotorForces[0], desMotorForces[1], desMotorForces[2], desMotorForces[3]);
    printf("Motor speeds: %f %f %f %f\n", desMotorSpeeds[0], desMotorSpeeds[1], desMotorSpeeds[2], desMotorSpeeds[3]);

    /* Create and return the radio command */
    uint8_t rawMsg[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];
    RadioTypes::RadioMessageDecoded::CreateSpeedsCommand(0, desMotorSpeeds, rawMsg);
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
    ros::init(argc, argv, "quad_mocap_speeds_control" + std::to_string(vehicleId));
    ros::NodeHandle n;
    ros::Subscriber subMocap = n.subscribe("mocap_output" + std::to_string(vehicleId), 1, callback_estimator);
    ros::Subscriber subTelemetry = n.subscribe("telemetry" + std::to_string(vehicleId), 10, callback_telemetry);
    ros::Subscriber subJoystick = n.subscribe("joystick_values", 1, callback_joystick);
    ros::Publisher pubEstimate = n.advertise<hiperlab_rostools::estimator_output>("estimator" + std::to_string(vehicleId), 1);
    ros::Publisher pubCmd = n.advertise<hiperlab_rostools::radio_command>("radio_command" + std::to_string(vehicleId), 1);

    /* Estimator and controller setup */
    HardwareTimer timer;
    est.reset(new Offboard::MocapStateEstimator(&timer, vehicleId, systemLatencyTime));
    Onboard::QuadcopterConstants::QuadcopterType quadcopterType = Onboard::QuadcopterConstants::GetVehicleTypeFromID(vehicleId);
    Onboard::QuadcopterConstants consts(quadcopterType);
    ctrl.SetParameters(consts.posControl_natFreq, consts.posControl_damping, consts.attControl_timeConst_xy, consts.attControl_timeConst_z);
    angVelCtrl.SetParameters(consts.angVelControl_timeConst_xy, consts.angVelControl_timeConst_z, consts.inertiaMatrix);
    mixer.SetParameters(consts.armLength, consts.propellerThrustFromSpeedSqr, consts.propellerTorqueFromThrust, consts.prop0SpinDir, consts.maxThrustPerPropeller, consts.minThrustPerPropeller, consts.maxCmdTotalThrust);

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
                initPosition = estState.pos;
            }
            {
                double const takeOffTime = 2.0; //[s]
                double frac = stageTimer.GetSeconds<double>() / takeOffTime;
                if (frac >= 1.0)
                {
                    flightStage = StageFlight;
                    frac = 1.0;
                }
                Vec3d cmdPos = (1 - frac) * initPosition + frac * desiredPosition;
                cmdMsg = RunControllerAndUpdateEstimator(estState, cmdPos, Vec3d(0, 0, 0), Vec3d(0, 0, 0), consts.mass);
            }
            break;

        case StageFlight:
            if (stageChange)
            {
                cout << "Entering flight stage.\n";
            }
            cmdMsg = RunControllerAndUpdateEstimator(estState, desiredPosition, Vec3d(0, 0, 0), Vec3d(0, 0, 0), consts.mass);

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
                Vec3d cmdPos = desiredPosition + stageTimer.GetSeconds<double>() * Vec3d(0, 0, -LANDING_SPEED);
                if (cmdPos.z < 0)
                {
                    flightStage = StageComplete;
                }
                cmdMsg = RunControllerAndUpdateEstimator(estState, cmdPos, Vec3d(0, 0, -LANDING_SPEED), Vec3d(0, 0, 0), consts.mass);
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