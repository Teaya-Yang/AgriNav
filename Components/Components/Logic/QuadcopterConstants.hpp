/* A file containing some constants, which ideally we later do more nicely.
 *
 * These are used by the controller, in the logic.
 *
 * Effectively, centralized hard-coding.
 */

#pragma once
#include "Common/Math/Matrix.hpp"

namespace Onboard
{

    class QuadcopterConstants
    {
    public:
        enum QuadcopterType
        {
            QC_TYPE_INVALID = 0,
            QC_TYPE_CF_STANDARD = 1,
            QC_TYPE_CF_BIGMOTORSPROPS = 2,
            QC_TYPE_CF_FEEDTHROUGH = 3,
            QC_TYPE_CF_LARGEQUAD = 4,
            QC_TYPE_CF_MINIQUAD = 5,
            QC_TYPE_CF_MINIQUAD_3SMOTORS = 6,
            /* New PX4-ESP architecture */
            QC_TYPE_PX4_LARGEQUAD = 10,
            QC_TYPE_PX4_LARGEQUAD_3D = 11,
            QC_TYPE_PX4_TILTROTOR = 12,
            QC_TYPE_COUNT, // number of different types
        };

        enum MotorType
        {
            CF_BRUSHED_MOTORS = 0, // Starts cf_motors
            ESC_MOTORS = 1,        // Starts esc_motors
            PIXRACER_MOTORS = 2,   // Starts pixracer_motors, for ESP style architecture
        };

        enum FCType
        {
            FC_TYPE_INVALID = 0,
            FC_TYPE_CF = 1,
            FC_TYPE_PX4 = 2,
        };

        QuadcopterConstants(QuadcopterType t)
        {

            // default control parameters:
            posControl_natFreq = 2.0f;
            posControl_damping = 0.7f;
            angVelControl_timeConst_xy = 0.03f;
            attControl_timeConst_xy = 0.20f;
            angVelControl_timeConst_z = 0.5f;
            attControl_timeConst_z = 1.0f;

            // default motor parameters:
            motorTimeConst = 0;
            motorInertia = 0;
            motorMinSpeed = 0;
            motorMaxSpeed = 10000; // Faster than the props will ever spin, basically no limit

            minThrustPerPropeller = 0.0f;
            maxCmdTotalThrust = -1; //-1 means that it will be computed in the mixer
            bidirectional = 0;

            float const perCellLowVoltage = 3.0f; //[V]
            lowBatteryThreshold = 0.0f;           // invalid

            voltageCurrentCalibrationConsts[0][0] = 0.f;
            voltageCurrentCalibrationConsts[0][1] = 0.f;
            voltageCurrentCalibrationConsts[1][0] = 0.f;
            voltageCurrentCalibrationConsts[1][1] = 0.f;

            armAngle = M_PI / 4;

            switch (t)
            {
            case QC_TYPE_CF_STANDARD:
                mass = 38e-3;
                Ixx = 16e-6f;
                Iyy = Ixx;
                Izz = 29e-6f;
                armLength = 46e-3f;

                IMU_yaw = 0;
                IMU_pitch = 0;
                IMU_roll = 0;

                propellerThrustFromSpeedSqr = float(3.58e-8f); //[N/(rad/s)**2] Standard CF motors and props
                propellerTorqueFromThrust = 0.0006;            //[N.m/N]
                prop0SpinDir = 1;                              // prop zero spins upwards

                motorType = CF_BRUSHED_MOTORS;
                FCType = FC_TYPE_CF;

                CFspeedToPWMConsts[0][0] = -86.19993685f;
                CFspeedToPWMConsts[0][1] = 22.87189816f;
                CFspeedToPWMConsts[1][0] = 0.30208677f;
                CFspeedToPWMConsts[1][1] = -0.07345602f;
                CFspeedToPWMConsts[2][0] = -1.59346434e-05f;
                CFspeedToPWMConsts[2][1] = 1.53209239e-05f;
                motorMaxSpeed = GetMaxCFSpeedFromPWMConsts(CFspeedToPWMConsts);
                maxThrustPerPropeller = propellerThrustFromSpeedSqr * powf(motorMaxSpeed, 2);
                maxCmdTotalThrust = 0.9f * maxThrustPerPropeller * 4;

                angVelControl_timeConst_xy = 0.04f;
                attControl_timeConst_xy = 0.40f;

                lowBatteryThreshold = 1 * perCellLowVoltage; // 1 cell lipo

                linDragCoeffBx = 0.0f; // [N/(m/s)] todo
                linDragCoeffBy = 0.0f; // [N/(m/s)] todo
                linDragCoeffBz = 0.0f; // [N/(m/s)] todo

                valid = true;
                break;
            case QC_TYPE_CF_BIGMOTORSPROPS:
                mass = 39e-3;
                Ixx = 30e-6f;
                Iyy = Ixx;
                Izz = 60e-6f;
                armLength = 48e-3f;

                IMU_yaw = 0;
                IMU_pitch = 0;
                IMU_roll = 0;

                propellerThrustFromSpeedSqr = float(4.14e-8f); //[N/(rad/s)**2]  //Standard CF motors and props
                propellerTorqueFromThrust = 0.001;             //??? [N.m/N]
                prop0SpinDir = 1;                              // prop zero spins upwards

                motorType = CF_BRUSHED_MOTORS;
                FCType = FC_TYPE_CF;
                CFspeedToPWMConsts[0][0] = -379.31113434f;
                CFspeedToPWMConsts[0][1] = 84.84738207f;
                CFspeedToPWMConsts[1][0] = 0.65309704f;
                CFspeedToPWMConsts[1][1] = -0.13852527f;
                CFspeedToPWMConsts[2][0] = -1.34462353e-04f;
                CFspeedToPWMConsts[2][1] = 3.57662798e-05f;
                motorMaxSpeed = GetMaxCFSpeedFromPWMConsts(CFspeedToPWMConsts);
                maxThrustPerPropeller = propellerThrustFromSpeedSqr * powf(motorMaxSpeed, 2);
                maxCmdTotalThrust = 0.8f * maxThrustPerPropeller * 4;

                lowBatteryThreshold = 1 * perCellLowVoltage; // 1 cell lipo

                linDragCoeffBx = 0.0206185f; // [N/(m/s)]
                linDragCoeffBy = 0.0216621f; // [N/(m/s)]
                linDragCoeffBz = 0.0f;       // [N/(m/s)] todo

                valid = true;
                break;
            case QC_TYPE_CF_FEEDTHROUGH:
                // Use arbitrary constants since the vehicle will not be allowed to fly using this type
                valid = false;
                mass = 1;
                Ixx = 1;
                Iyy = Ixx;
                Izz = 1;
                armLength = 1;

                IMU_yaw = 0;
                IMU_pitch = 0;
                IMU_roll = 0;

                propellerThrustFromSpeedSqr = 0;
                propellerTorqueFromThrust = 0;
                maxThrustPerPropeller = 0;
                prop0SpinDir = 0;

                motorType = CF_BRUSHED_MOTORS;
                FCType = FC_TYPE_CF;
                CFspeedToPWMConsts[0][0] = 0.0f;
                CFspeedToPWMConsts[0][1] = 0.0f;
                CFspeedToPWMConsts[1][0] = 1.0f;
                CFspeedToPWMConsts[1][1] = 0.0f;
                CFspeedToPWMConsts[2][0] = 0.0f;
                CFspeedToPWMConsts[2][1] = 0.0f;

                lowBatteryThreshold = 1 * perCellLowVoltage; // 1 cell lipo (?)

                linDragCoeffBx = 0.0f; // [N/(m/s)]
                linDragCoeffBy = 0.0f; // [N/(m/s)]
                linDragCoeffBz = 0.0f; // [N/(m/s)]

                break;
            case QC_TYPE_CF_LARGEQUAD:
                mass = 0.695;           // [kg]
                Ixx = 0.004406f; // Measured from CAD, [kg*m^2]
                Iyy = Ixx;
                Izz = 0.008611f;
                armLength = 0.166f;

                IMU_yaw = 0; //[rad]
                IMU_pitch = 0;
                IMU_roll = 0;

                // Measured using test stand. Speeds measured with optical tachometer.
                // Data is here under "Propeller 2": https://docs.google.com/spreadsheets/d/1G8jk59uORHqtUQ7bAEJloMTFE8x1uWwJqDx5DdCS6mc/edit?usp=sharing
                propellerThrustFromSpeedSqr = 7.64e-6f; //[N/(rad/s)**2]
                propellerTorqueFromThrust = 0.0140f;    // [N.m/N]
                prop0SpinDir = 1;                       // prop zero spins upwards

                motorType = ESC_MOTORS;
                FCType = FC_TYPE_CF;
                ESCspeedToPWMConsts[0] = 972.0f;
                ESCspeedToPWMConsts[1] = 0.742f;
                motorMaxSpeed = GetMaxESCSpeedFromPWMConsts(ESCspeedToPWMConsts);
                maxThrustPerPropeller = propellerThrustFromSpeedSqr * powf(motorMaxSpeed, 2);

                lowBatteryThreshold = 3 * perCellLowVoltage; // 3 cell lipo

                voltageCurrentCalibrationConsts[0][0] = 0.0073869f;
                voltageCurrentCalibrationConsts[0][1] = 0.f;
                voltageCurrentCalibrationConsts[1][0] = 0.02230f; // Calibration measurement by Andrea
                voltageCurrentCalibrationConsts[1][1] = 0.f;

                // LQR attitude control gains computed by Nathan
                // These values should probably be tuned
                angVelControl_timeConst_xy = 0.0457f;
                attControl_timeConst_xy = 0.0914f;
                angVelControl_timeConst_z = 0.2545f;
                attControl_timeConst_z = 0.5089f;

                // Measured by Andrea
                linDragCoeffBx = 0.1286181f; // [N/(m/s)]
                linDragCoeffBy = 0.1286181f; // [N/(m/s)]
                linDragCoeffBz = 0.1286181f; // [N/(m/s)]

                valid = true;
                break;
            case QC_TYPE_CF_MINIQUAD:
                mass = 0.142;          // [kg] check vehicle and change accordingly
                Ixx = 92.7e-6f; // Measured from CAD, [kg*m^2]
                Iyy = Ixx;
                Izz = 158.57e-6f;
                armLength = 58e-3f; //[m]

                IMU_yaw = 0; //[rad]
                IMU_pitch = 0;
                IMU_roll = 0;

                // Measured using test stand. Speeds measured with optical tachometer.
                propellerThrustFromSpeedSqr = 4.32e-8f; //[N/(rad/s)**2]
                propellerTorqueFromThrust = 0.00808f;   // [N.m/N]
                prop0SpinDir = 1;                       // prop zero spins upwards

                motorType = ESC_MOTORS;
                FCType = FC_TYPE_CF;
                ESCspeedToPWMConsts[0] = 999.0f;
                ESCspeedToPWMConsts[1] = 0.14f;
                motorMaxSpeed = GetMaxESCSpeedFromPWMConsts(ESCspeedToPWMConsts);
                maxThrustPerPropeller = propellerThrustFromSpeedSqr * powf(motorMaxSpeed, 2);
                minThrustPerPropeller = 0.03f; //[N], 1/10 of hover thrust
                maxCmdTotalThrust = 0.7f * (maxThrustPerPropeller * 4);

                lowBatteryThreshold = 2 * perCellLowVoltage;

                voltageCurrentCalibrationConsts[0][0] = 0.0038815f;
                voltageCurrentCalibrationConsts[0][1] = 0.0224488f;
                voltageCurrentCalibrationConsts[1][0] = 0.f;
                voltageCurrentCalibrationConsts[1][1] = 0.f;

                posControl_natFreq = 2.0f; // 3.0f;
                posControl_damping = 0.7f;

                angVelControl_timeConst_xy = 0.04f; // can be tightened to 0.03
                attControl_timeConst_xy = angVelControl_timeConst_xy * 2;
                angVelControl_timeConst_z = angVelControl_timeConst_xy * 5; // 0.15f;
                attControl_timeConst_z = angVelControl_timeConst_z * 2;

                linDragCoeffBx = 0.0f; // [N/(m/s)]
                linDragCoeffBy = 0.0f; // [N/(m/s)]
                linDragCoeffBz = 0.0f; // [N/(m/s)]

                valid = true;
                break;

            case QC_TYPE_CF_MINIQUAD_3SMOTORS:
                mass = 0.240;           // [kg] check vehicle and change accordingly
                Ixx = 190.0e-6f; // Calculated by eyeballing and intuition
                Iyy = Ixx;
                Izz = 255.0e-6f;
                armLength = 58e-3f; //[m]

                IMU_yaw = 0; //[rad]
                IMU_pitch = 0;
                IMU_roll = 0;

                // Measured using test stand. Speeds measured with optical tachometer.
                // Below coefficients are for 3-inch props
                propellerThrustFromSpeedSqr = 1.145e-7f; //[N/(rad/s)**2]
                propellerTorqueFromThrust = 0.005119f;   // [N.m/N]
                prop0SpinDir = 1;                        // prop zero spins upwards

                motorType = ESC_MOTORS;
                FCType = FC_TYPE_CF;
                // Below characterization is when using 3S battery with 6000kv motors.
                ESCspeedToPWMConsts[0] = 1007.02f;
                ESCspeedToPWMConsts[1] = 0.14152f;
                motorMaxSpeed = GetMaxESCSpeedFromPWMConsts(ESCspeedToPWMConsts);
                maxThrustPerPropeller = propellerThrustFromSpeedSqr * powf(motorMaxSpeed, 2);
                minThrustPerPropeller = 0.03f; //[N], 1/10 of hover thrust
                maxCmdTotalThrust = 0.7f * (maxThrustPerPropeller * 4);

                lowBatteryThreshold = 3 * perCellLowVoltage;

                voltageCurrentCalibrationConsts[0][0] = 0.0038815f;
                voltageCurrentCalibrationConsts[0][1] = 0.0224488f;
                voltageCurrentCalibrationConsts[1][0] = 0.f;
                voltageCurrentCalibrationConsts[1][1] = 0.f;

                posControl_natFreq = 2.0f; // 3.0f;
                posControl_damping = 0.7f;

                angVelControl_timeConst_xy = 0.04f; // can be tightened to 0.03
                attControl_timeConst_xy = angVelControl_timeConst_xy * 2;
                angVelControl_timeConst_z = angVelControl_timeConst_xy * 5; // 0.15f;
                attControl_timeConst_z = angVelControl_timeConst_z * 2;

                linDragCoeffBx = 0.0f; // [N/(m/s)]
                linDragCoeffBy = 0.0f; // [N/(m/s)]
                linDragCoeffBz = 0.0f; // [N/(m/s)]

                valid = true;
                break;
            /* New PX4-ESP architecture */
            case QC_TYPE_PX4_LARGEQUAD:
                // Frame rotation
                IMU_yaw = 0; //[rad]
                IMU_pitch = 0;
                IMU_roll = M_PI;

                // Vehicle parameters
                mass = 1.547; // 1.215 0.985;       // [kg]
                Ixx = 7.5e-3; // Measured from CAD, [kg*m^2]
                Iyy = 8.5e-3;
                Izz = 15.5e-3;
                lx = 0.11;
                ly = 0.138;
                propellerThrustFromSpeedSqr = 7.64e-6f; //[N/(rad/s)**2]
                propellerTorqueFromThrust = 0.0140f;    // [N.m/N]
                prop0SpinDir = 1;                       // prop zero spins upwards

                // Controller parameters
                posControl_natFreq = 2.0f;
                posControl_damping = 0.7f;
                attControl_timeConst_xy = 0.15f;
                attControl_timeConst_z = 0.3f;
                angVelControl_timeConst_xy = 0.05f;
                angVelControl_timeConst_z = 0.1f;

                // Other parameters
                armLength = 0.1777f;
                armAngle = 0.8851f;
                maxThrustPerPropeller = 10;             // [N], kind of a guess
                motorType = PIXRACER_MOTORS;
                FCType = FC_TYPE_PX4;
                lowBatteryThreshold = 3 * perCellLowVoltage; // 3 cell lipo
                valid = true;
                break;

            case QC_TYPE_PX4_LARGEQUAD_3D:
                // Frame rotation
                IMU_yaw = 0; //[rad]
                IMU_pitch = 0;
                IMU_roll = M_PI;

                // Vehicle parameters
                mass = 0.92;       // [kg]
                Ixx = 8.5e-3; // Measured from CAD, [kg*m^2]
                Iyy = 7.5e-3;
                Izz = 15.5e-3;
                lx = 0.11;
                ly = 0.138;
                propellerThrustFromSpeedSqr = 7.64e-6f; //[N/(rad/s)**2]
                propellerTorqueFromThrust = 0.0140f;    // [N.m/N]
                prop0SpinDir = -1;                       // prop zero spins upwards
                bidirectional = 1;

                // Controller parameters
                posControl_natFreq = 2.0f;
                posControl_damping = 0.7f;
                attControl_timeConst_xy = 0.15f;
                attControl_timeConst_z = 0.3f;
                angVelControl_timeConst_xy = 0.05f;
                angVelControl_timeConst_z = 0.1f;

                // Other parameters
                armLength = 0.1777f;
                armAngle = 0.8851f;
                maxThrustPerPropeller = 10;             // [N], kind of a guess
                motorType = PIXRACER_MOTORS;
                FCType = FC_TYPE_PX4;
                lowBatteryThreshold = 3 * perCellLowVoltage; // 3 cell lipo
                valid = true;
                break;

            case QC_TYPE_PX4_TILTROTOR:
                // Frame rotation
                IMU_yaw = 0; //[rad]
                IMU_pitch = 0;
                IMU_roll = M_PI;

                // Vehicle parameters
                mass = 1.0;       // [kg]
                Ixx = 7.5e-3; // Measured from CAD, [kg*m^2]
                Iyy = 8.5e-3;
                Izz = 15.5e-3;
                lx = 0.11;
                ly = 0.138;
                propellerThrustFromSpeedSqr = 7.64e-6f; //[N/(rad/s)**2]
                propellerTorqueFromThrust = 0.0140f;    // [N.m/N]
                prop0SpinDir = -1;                       // prop zero spins downwards

                // Controller parameters
                posControl_natFreq = 2.0f;
                posControl_damping = 0.7f;
                attControl_timeConst_xy = 0.15f;
                attControl_timeConst_z = 0.3f;
                angVelControl_timeConst_xy = 0.05f;
                angVelControl_timeConst_z = 0.1f;

                // Other parameters
                armLength = 0.1777f;
                armAngle = 0.8851f;
                maxThrustPerPropeller = 10;             // [N], kind of a guess
                motorType = PIXRACER_MOTORS;
                FCType = FC_TYPE_PX4;
                lowBatteryThreshold = 3 * perCellLowVoltage; // 3 cell lipo
                valid = true;
                break;

            default:
                valid = false;
                mass = 1;
                Ixx = 1;
                Iyy = Ixx;
                Izz = 1;
                armLength = 1;
                armAngle = M_PI / 4;

                IMU_yaw = 0;
                IMU_pitch = 0;
                IMU_roll = 0;

                propellerThrustFromSpeedSqr = 0;
                propellerTorqueFromThrust = 0;
                maxThrustPerPropeller = 0;
                prop0SpinDir = 0;

                motorType = CF_BRUSHED_MOTORS;
                FCType = FC_TYPE_INVALID;
                CFspeedToPWMConsts[0][0] = 0;
                CFspeedToPWMConsts[0][1] = 0;
                CFspeedToPWMConsts[1][0] = 0;
                CFspeedToPWMConsts[1][1] = 0;
                CFspeedToPWMConsts[2][0] = 0;
                CFspeedToPWMConsts[2][1] = 0;

                ESCspeedToPWMConsts[0] = 0;
                ESCspeedToPWMConsts[1] = 0;

                motorMaxSpeed = 0;

                break;
            }

            inertiaMatrix(0, 0) = Ixx;
            inertiaMatrix(1, 1) = Iyy;
            inertiaMatrix(2, 2) = Izz;

            return;
        }

        static char const *GetNameStringFromID(unsigned id) { return GetNameString(GetVehicleTypeFromID(id)); }

        static char const *GetNameString(QuadcopterType t)
        {
            switch (t)
            {
            case QC_TYPE_CF_STANDARD:
                return "QC_TYPE_CF_STANDARD";
            case QC_TYPE_CF_BIGMOTORSPROPS:
                return "QC_TYPE_CF_BIGMOTORSPROPS";
            case QC_TYPE_CF_LARGEQUAD:
                return "QC_TYPE_CF_LARGEQUAD";
            case QC_TYPE_CF_MINIQUAD:
                return "QC_TYPE_CF_MINIQUAD";
            case QC_TYPE_CF_MINIQUAD_3SMOTORS:
                return "QC_TYPE_CF_MINIQUAD_3SMOTORS";
            case QC_TYPE_CF_FEEDTHROUGH:
                return "QC_TYPE_CF_FEEDTHROUGH";
            /* New PX4-ESP architecture */
            case QC_TYPE_PX4_LARGEQUAD:
                return "QC_TYPE_PX4_LARGEQUAD";
            case QC_TYPE_PX4_LARGEQUAD_3D:
                return "QC_TYPE_PX4_LARGEQUAD_3D";
            case QC_TYPE_PX4_TILTROTOR:
                return "QC_TYPE_PX4_TILTROTOR";
            default:
                return "INVALID_TYPE!";
            }
        }

        static QuadcopterType GetVehicleTypeFromID(unsigned int id)
        {
            // This function maps each vehicle ID to the quadcopter type
            // When updating this function, also update the wiki page here:
            // https://github.com/muellerlab/wiki/wiki/Vehicle-IDs

            switch (id)
            {
            case 4:
            case 10:
                return QC_TYPE_CF_STANDARD;
            case 2:
            case 5:
            case 6:
            case 7:
            case 9:
            case 12:
            case 15:
            case 17:
                return QC_TYPE_CF_BIGMOTORSPROPS;
            case 13:
            case 14:
            case 18:
            case 19:
                return QC_TYPE_CF_LARGEQUAD;
            case 1:
            case 3:
            case 16:
            case 20:
            case 21:
            case 22:
            case 24:
            case 26:
                return QC_TYPE_CF_MINIQUAD;
            case 11:
                return QC_TYPE_CF_MINIQUAD_3SMOTORS;
            /* New PX4-ESP architecture */
            case 101: // Carlo #1
            case 102: // Carlo
            case 103: // Carlo
            case 104: // Carlo
            case 105: // Carlo
            case 110: // Roman
            case 111: //Teaya
                return QC_TYPE_PX4_LARGEQUAD;
            case 107: // MEng 97
                return QC_TYPE_PX4_LARGEQUAD_3D;
            case 100: // Original PX4-ESP quadcopter by Jerry
            case 106: // Junior tiltrotor
                return QC_TYPE_PX4_TILTROTOR;
            default:
                return QC_TYPE_INVALID;
            }
        }

        float mass, Ixx, Iyy, Izz, armLength, armAngle, lx, ly;
        float propellerThrustFromSpeedSqr, propellerTorqueFromThrust, maxThrustPerPropeller, minThrustPerPropeller;
        float maxCmdTotalThrust; // effectively, how much margin to leave for attitude control. See the Mixer.
        int prop0SpinDir; int bidirectional;
        float linDragCoeffBx, linDragCoeffBy, linDragCoeffBz;

        // motorInertia is the inertia of the combined motor rotor and propeller
        float motorTimeConst, motorInertia, motorMinSpeed, motorMaxSpeed; // Units in [s], [kg*m**2], [rad/s], [rad/s]

        MotorType motorType; // Chooses which embedded motor app to run (cf_motors or esc_motors)
        FCType FCType;       // Chooses which flight controller to use (cf or px4)
        // Constants mapping desired crazyflie motor speed and current battery level to PWM (used by cf_motors)
        float CFspeedToPWMConsts[3][2];
        // Constants mapping desired ESC motor speed and current battery level to PWM (used by esc_motors)
        float ESCspeedToPWMConsts[2];

        // Constants mapping adc values linearly to battery voltage and current consumption
        float voltageCurrentCalibrationConsts[2][2];

        // controller parameters:
        float posControl_natFreq;
        float posControl_damping;
        float angVelControl_timeConst_xy;
        float attControl_timeConst_xy;
        float angVelControl_timeConst_z;
        float attControl_timeConst_z;

        // IMU frame's rotation angles w.r.t. UAV's frame
        float IMU_yaw; //[rad]
        float IMU_pitch;
        float IMU_roll;

        float lowBatteryThreshold;

        bool valid;

        Matrix<float, 3, 3> inertiaMatrix = ZeroMatrix<float, 3, 3>();

    private:
        float GetMaxCFSpeedFromPWMConsts(float CFspeedToPWMConstsIn[3][2])
        {
            /* Calculates the theoretical maximum speed of the crazyflie propellers based
             * on the experimentally measured constants mapping the desired propeller speed
             * and current battery voltage to the output PWM. The maximum PWM signal we can
             * output is 255 (i.e. full duty cycle). Assume the battery is fully charged.
             */
            int MAX_PWM = 255;    // Corresponds to fully duty cycle
            float MAX_BATT = 4.1; // Voltage of battery at maximum charge

            float k_1 = CFspeedToPWMConstsIn[0][0] + CFspeedToPWMConstsIn[0][1] * MAX_BATT;
            float k_2 = CFspeedToPWMConstsIn[1][0] + CFspeedToPWMConstsIn[1][1] * MAX_BATT;
            float k_3 = CFspeedToPWMConstsIn[2][0] + CFspeedToPWMConstsIn[2][1] * MAX_BATT;

            // outpwm = int(k_1 + (k_2 + k_3 * speed) * speed);
            // Solve for speed using quadratic formula
            float maximumMotorSpeed = (-k_2 + sqrtf(powf(k_2, 2) - 4 * k_3 * (k_1 - MAX_PWM))) / (2 * k_3);

            return maximumMotorSpeed;
        }

        float GetMaxESCSpeedFromPWMConsts(float ESCspeedToPWMConstsIn[2])
        {
            /* Calculates the theoretical maximum speed of the ESC motors based on the
             * constants mapping the desired propeller speed to PWM command. The maximum
             * output signal we can output is 2000, which corresponds to full speed as set
             * in the ESC firmware. Speed to output PWM is a linear mapping.
             */
            int ESC_PERIOD_MAX = 2000; // Corresponds to fully duty cycle

            float maximumMotorSpeed = (ESC_PERIOD_MAX - ESCspeedToPWMConstsIn[0]) / ESCspeedToPWMConstsIn[1];

            return maximumMotorSpeed;
        }
    };

} // namespace Onboard
// namespace Onboard
