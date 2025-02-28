#pragma once

#include "Common/Math/Rotation.hpp"
#include "Common/Math/Vec3.hpp"

// author: Jerry Tang
class TiltrotorController
{
public:
    // Constructor
    TiltrotorController() {}
    TiltrotorController(float natFreq, float dampingRatio, float timeConstAngleRP, float timeConstAngleY, float timeConstRatesRP, float timeConstRatesY, float mass, Matrix<float, 3, 3> inertiaMatrix, float lx, float ly, float thrustToTorque)
        : natFreq(natFreq),
          dampingRatio(dampingRatio),
          timeConstAngleRP(timeConstAngleRP),
          timeConstAngleY(timeConstAngleY),
          timeConstRatesRP(timeConstRatesRP),
          timeConstRatesY(timeConstRatesY),
          mass(mass),
          inertiaMatrix(inertiaMatrix),
          lx(lx),
          ly(ly),
          k(thrustToTorque) {}

    // Position controller section
    Vec3f get_acceleration(Vec3f desPos, Vec3f curPos, Vec3f desVel, Vec3f curVel)
    {
        Vec3f desAcc = natFreq * natFreq * (desPos - curPos) + 2 * dampingRatio * natFreq * (desVel - curVel);
        return desAcc;
    }

    Vec3f get_thrust(Vec3f desAcc)
    {
        Vec3f desThrust = desAcc + Vec3f(0, 0, 9.81);
        return desThrust;
    }

    // Attitude controller section
    void get_angular_velocity(Vec3f desThrust, Rotationf curAtt, float desPitch, float desYaw)
    {
        // Compute desired thrust direction
        desThrustNorm = desThrust.GetNorm2();
        Vec3f desThrustDir = desThrust / desThrustNorm;
        Vec3f e3 = Vec3f(0, 0, 1);
        float angle = acosf(desThrustDir.Dot(e3));
        Vec3f rotAx = e3.Cross(desThrustDir);
        float n = rotAx.GetNorm2();
        Rotationf thrustAtt;
        if (n < 1e-6)
        {
            thrustAtt = Rotationf::Identity();
        }
        else
        {
            thrustAtt = Rotationf::FromRotationVector(rotAx * (angle / n));
        }
        thrustAtt = thrustAtt * Rotationf::FromRotationVector(Vec3f(0, 0, desYaw));
        // Find the zero pitch attitude
        Matrix<float, 3, 3> thrustAttMat = thrustAtt.GetRotationMatrix();
        Vec3f zeroPitchX = Vec3f(thrustAttMat(0, 1), thrustAttMat(1, 1), thrustAttMat(2, 1)).Cross(e3);
        zeroPitchX = zeroPitchX / zeroPitchX.GetNorm2();
        Vec3f zeroPitchZ = zeroPitchX.Cross(Vec3f(thrustAttMat(0, 1), thrustAttMat(1, 1), thrustAttMat(2, 1)));
        Matrix<float, 3, 3> zeroPitchAttMat;
        zeroPitchAttMat(0, 0) = zeroPitchX.x;
        zeroPitchAttMat(1, 0) = zeroPitchX.y;
        zeroPitchAttMat(2, 0) = zeroPitchX.z;
        zeroPitchAttMat(0, 1) = thrustAttMat(0, 1);
        zeroPitchAttMat(1, 1) = thrustAttMat(1, 1);
        zeroPitchAttMat(2, 1) = thrustAttMat(2, 1);
        zeroPitchAttMat(0, 2) = zeroPitchZ.x;
        zeroPitchAttMat(1, 2) = zeroPitchZ.y;
        zeroPitchAttMat(2, 2) = zeroPitchZ.z;
        // Compute desired attitude and the net tilt angle
        Rotationf desAtt = Rotationf::FromRotationMatrix(zeroPitchAttMat) * Rotationf::FromRotationVector(Vec3f(0, desPitch, 0));
        Matrix<float, 3, 3> desAttMat = desAtt.GetRotationMatrix();
        netTiltAngle = acosf(fmaxf(fminf(Vec3f(desAttMat(0, 2), desAttMat(1, 2), desAttMat(2, 2)).Dot(Vec3f(thrustAttMat(0, 2), thrustAttMat(1, 2), thrustAttMat(2, 2))), 1), -1));
        Vec3f netTiltAxis = Vec3f(desAttMat(0, 2), desAttMat(1, 2), desAttMat(2, 2)).Cross(Vec3f(thrustAttMat(0, 2), thrustAttMat(1, 2), thrustAttMat(2, 2)));
        netTiltAngle = sign(netTiltAxis.Dot(Vec3f(thrustAttMat(0, 1), thrustAttMat(1, 1), thrustAttMat(2, 1)))) * netTiltAngle;
        // Compute desired rates
        float timeConstAnglePitch = timeConstAngleRP;
        float timeConstAngleRoll = timeConstAngleRP * fabsf(cosf(netTiltAngle)) + timeConstAngleY * fabsf(sinf(netTiltAngle));
        float timeConstAngleYaw = timeConstAngleRP * fabsf(sinf(netTiltAngle)) + timeConstAngleY * fabsf(cosf(netTiltAngle));
        Vec3f desRotVec = (curAtt.Inverse() * desAtt).ToRotationVector();
        desAngVel = Vec3f(desRotVec.x / timeConstAngleRoll, desRotVec.y / timeConstAnglePitch, desRotVec.z / timeConstAngleYaw);
    }

    Vec3f get_angular_acceleration(Vec3f desAngVel, Vec3f curAngVel, float netTiltAngle)
    {
        // Compute desired angular acceleration
        float timeConstRatesPitch = timeConstRatesRP;
        float timeConstRatesRoll = timeConstRatesRP * fabsf(cosf(netTiltAngle)) + timeConstRatesY * fabsf(sinf(netTiltAngle));
        float timeConstRatesYaw = timeConstRatesRP * fabsf(sinf(netTiltAngle)) + timeConstRatesY * fabsf(cosf(netTiltAngle));
        Vec3f errAngVel = desAngVel - curAngVel;
        Vec3f desAngAcc = Vec3f(errAngVel.x / timeConstRatesRoll, errAngVel.y / timeConstRatesPitch, errAngVel.z / timeConstRatesYaw);
        return desAngAcc;
    }

    // Mixer section
    void get_motor_force_cmd(float desThrustNorm, Vec3f desAngAcc, float netTiltAngle)
    {
        float totalThrust = mass * desThrustNorm;
        Vec3f moment = inertiaMatrix * desAngAcc;
        // Process pitch moment
        float dfPitch = moment.y / lx / 2;
        float frontThrust[2] = {totalThrust / 2 * sinf(netTiltAngle), totalThrust / 2 * cosf(netTiltAngle) - dfPitch};
        float rearThrust[2] = {totalThrust / 2 * sinf(netTiltAngle), totalThrust / 2 * cosf(netTiltAngle) + dfPitch};
        float thetaf = atanf(frontThrust[0] / frontThrust[1]) + M_PI * (frontThrust[1] < 0);
        float thetar = atanf(rearThrust[0] / rearThrust[1]) + M_PI * (rearThrust[1] < 0);
        desTiltAngles[0] = thetaf;
        desTiltAngles[1] = thetar;
        // Process roll and yaw moments with the now fixed tilt angles
        float M_inv[4][4] = {{1.0f / 2.0f, 0, -(k * cosf(thetar) + ly * sinf(thetar)) / (2 * (sinf(thetaf - thetar) * k * k + 2 * cosf(thetaf - thetar) * k * ly - sinf(thetaf - thetar) * ly * ly)), -(ly * cosf(thetar) - k * sinf(thetar)) / (2 * (sinf(thetaf - thetar) * k * k + 2 * cosf(thetaf - thetar) * k * ly - sinf(thetaf - thetar) * ly * ly))},
                             {0, 1.0f / 2.0f, -(k * cosf(thetaf) - ly * sinf(thetaf)) / (2 * (sinf(thetaf - thetar) * k * k + 2 * cosf(thetaf - thetar) * k * ly - sinf(thetaf - thetar) * ly * ly)), (ly * cosf(thetaf) + k * sinf(thetaf)) / (2 * sinf(thetaf - thetar) * k * k + 4 * cosf(thetaf - thetar) * k * ly - 2 * sinf(thetaf - thetar) * ly * ly)},
                             {0, 1.0f / 2.0f, (k * cosf(thetaf) - ly * sinf(thetaf)) / (2 * sinf(thetaf - thetar) * k * k + 4 * cosf(thetaf - thetar) * k * ly - 2 * sinf(thetaf - thetar) * ly * ly), -(ly * cosf(thetaf) + k * sinf(thetaf)) / (2 * (sinf(thetaf - thetar) * k * k + 2 * cosf(thetaf - thetar) * k * ly - sinf(thetaf - thetar) * ly * ly))},
                             {1.0f / 2.0f, 0, (k * cosf(thetar) + ly * sinf(thetar)) / (2 * sinf(thetaf - thetar) * k * k + 4 * cosf(thetaf - thetar) * k * ly - 2 * sinf(thetaf - thetar) * ly * ly), (ly * cosf(thetar) - k * sinf(thetar)) / (2 * sinf(thetaf - thetar) * k * k + 4 * cosf(thetaf - thetar) * k * ly - 2 * sinf(thetaf - thetar) * ly * ly)}};
        float u[4] = {norm(frontThrust, 2), norm(rearThrust, 2), moment.x, moment.z};
        matrix_vector_multiply(&M_inv[0][0], u, motForceCmds, 4);
        // debug[0] = motForceCmds[0];
        // debug[1] = motForceCmds[1];
        // debug[2] = motForceCmds[2];
        // debug[3] = motForceCmds[3];
        // debug[4] = desTiltAngles[0];
        // debug[5] = desTiltAngles[1];
        // debug[6] = 0;
        // debug[7] = 0;
        // debug[8] = 0;
    }

    // sinfgle controller callback
    void run(Vec3f desPos, Vec3f curPos, Vec3f desVel, Vec3f curVel, Rotationf curAtt, Vec3f curAngVel, float desPitch, float desYaw)
    {
        Vec3f desAcc = get_acceleration(desPos, curPos, desVel, curVel);
        Vec3f desThrust = get_thrust(desAcc);
        get_angular_velocity(desThrust, curAtt, desPitch, desYaw);
        Vec3f desAngAcc = get_angular_acceleration(desAngVel, curAngVel, netTiltAngle);
        get_motor_force_cmd(desThrustNorm, desAngAcc, netTiltAngle);
    }

    // Function returns
    float desThrustNorm;
    Vec3f desAngVel;
    float netTiltAngle;
    float motForceCmds[4];
    float desTiltAngles[2];
    float debug[9];

private:
    // Properties
    float natFreq;
    float dampingRatio;
    float timeConstAngleRP;
    float timeConstAngleY;
    float timeConstRatesRP;
    float timeConstRatesY;
    float mass;
    Matrix<float, 3, 3> inertiaMatrix;
    float lx;
    float ly;
    float k;
    int sign(float x)
    {
        if (x > 0)
            return 1;
        if (x < 0)
            return -1;
        return 0;
    }
    float norm(float x[], int n)
    {
        float sum = 0.0f;
        for (int i = 0; i < n; i++)
        {
            sum += x[i] * x[i];
        }
        return sqrtf(sum);
    }
    void matrix_vector_multiply(float *A, float B[], float C[], int m)
    {
        for (int i = 0; i < m; i++)
        {
            C[i] = 0;
            for (int k = 0; k < m; k++)
            {
                C[i] += A[i * m + k] * B[k];
            }
        }
    }
};