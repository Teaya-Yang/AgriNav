#include "Components/Logic/QuadcopterLogic.hpp"
#include <cerrno>

using namespace Onboard;

QuadcopterLogic::QuadcopterLogic(BaseTimer *const timer, float onboardLogicPeriod)
    : _timer(timer), _onboardPeriod(onboardLogicPeriod), _radioCmdPeriod(0.01f), _kf(timer), _timeSinceLastUwb(timer), _timeSinceLastRadioMessage(timer),
      _monitorCmdRate(timer, _radioCmdPeriod, 1, _radioCmdPeriod),               // hardcode 1 rad/s cuttoff frq for filter
      _monitorMainLoopPeriod(timer, onboardLogicPeriod, 50, onboardLogicPeriod), // hardcode 50 rad/s cuttoff frq for filter
      _monitorUWBEstimatorResets(timer), _battThresholdCriticalVoltage(1e3f), _battThresholdWarningVoltage(1e3f)
{
    ResetCounters();
}

void QuadcopterLogic::ResetCounters(void)
{
    _cycleCounter = 0;
    for (int i = 0; i < 4; i++)
    {
        _desMotorSpeeds[i] = 0;
        _desMotorForcesForTelemetry[i] = 0;
    }
    _state = FS_UNINITIALIZED;
    _mass = 0;

    _battMeas.isNew = false;
    _battMeas.count = 0;
    _battMeas.voltageRaw = 0;
    _battMeas.currentRaw = 0;

    _imuRateGyro.isNew = false;
    _imuRateGyro.count = 0;
    _imuRateGyro.rawMeas = Vec3f(0, 0, 0);

    _imuAccelerometer.isNew = false;
    _imuAccelerometer.count = 0;
    _imuAccelerometer.rawMeas = Vec3f(0, 0, 0);

    _imuTemperature.isNew = false;
    _imuTemperature.count = 0;
    _imuTemperature.rawMeas = 25; //[C]

    _radioMessage.isNew = false;
    _radioMessage.count = 0;
    _uwbRangeMeas.count = 0;
    _uwbRangeMeas.isNew = false;
    _uwbRangeMeas.range = 0;
    _uwbRangeMeas.targetId = 0;
    _shouldStartNewUWBConversation = false;

    _numRangingTargets = 0;
    _nextRangingTargetIdx = 0;

    _desPos = Vec3f(0, 0, 0.5f);

    _testMotorsOn = false;
    _testMotorsThrustFrac = 0;

    _battThresholdCriticalVoltage = 0;

    _shouldWriteParameters = false;

    _propellerCalibration.currentlyRunning = false;
    for (int i = 0; i < 4; i++)
    {
        _propellerCalibration.activeFactors[i] = 1.0f;
        _propellerCalibration.accumulators[i] = 0.0f;
    }
    _propellerCalibration.accumulatorCount = 0;
    _propellerCalibration.minAccumulatorCount = 750;

    // Hard coded for now, max 30% change!
    _propellerCalibration.calibrationFactorMin = 0.7f;
    _propellerCalibration.calibrationFactorMax = 1.0f / _propellerCalibration.calibrationFactorMin;

    for (int i = 0; i < NUM_DEBUG_VARS; i++)
    {
        _debug[i] = 0;
    }

    _firstPanicReason = PANIC_NO_PANIC;

    _myId = 0;

    _telPacketCounter = 0;

    _telWarnings = 0;

    ResetGyroCalibration();
}

void QuadcopterLogic::Initialise(QuadcopterConstants::QuadcopterType type, uint8_t vehId)
{
    ResetCounters();
    // control (tuning) parameters:

    const float lowPassFilter_accel_cutoff = 100.0f;                //[rad/s] well below the control time constants.
    const float lowPassFilter_gyro_cutoff = 200.0f;                 //[rad/s] well below the control time constants.
    const float lowPassFilter_batt_cutoff = 0.5f * float(2 * M_PI); // cut off at 2Hz.
    const float lowPassFilter_temp_cutoff = 0.5f * float(2 * M_PI); // cut off at 2Hz.

    _myId = vehId;

    // physical constants:
    _quadType = type;
    QuadcopterConstants consts(_quadType);

    _mass = consts.mass;

    _IMU_yaw = consts.IMU_yaw;
    _IMU_pitch = consts.IMU_pitch;
    _IMU_roll = consts.IMU_roll;
    _R = Rotationf::FromEulerYPR(_IMU_yaw, _IMU_pitch, _IMU_roll);

    _battThresholdCriticalVoltage = consts.lowBatteryThreshold;
    _battThresholdWarningVoltage = 1.05f * _battThresholdCriticalVoltage; // within 5%, hardcoded for now.

    float quadArmLength = consts.armLength;
    float quadArmAngle = consts.armAngle;
    float propellerThrustFromSpeedSqr = consts.propellerThrustFromSpeedSqr;
    float propellerTorqueFromThrust = consts.propellerTorqueFromThrust;
    int prop0SpinDir = consts.prop0SpinDir;

    // note that the filters need a cutoff frequency
    _imuAccelerometer.lowPass.Initialise(_onboardPeriod, lowPassFilter_accel_cutoff, _imuAccelerometer.rawMeas);
    _imuRateGyro.lowPass.Initialise(_onboardPeriod, lowPassFilter_gyro_cutoff, _imuRateGyro.rawMeas);
    _imuTemperature.lowPass.Initialise(_onboardPeriod, lowPassFilter_temp_cutoff, _imuTemperature.rawMeas);

    _battVoltageLowPass.Initialise(_onboardPeriod, lowPassFilter_batt_cutoff,
                                   consts.lowBatteryThreshold * 1.2f); // init at 20% above min value

    _posCtrl.SetParameters(consts.posControl_natFreq, consts.posControl_damping);
    _attCtr.SetParameters(consts.attControl_timeConst_xy, consts.attControl_timeConst_z);
    _angVelCtrl.SetParameters(consts.angVelControl_timeConst_xy, consts.angVelControl_timeConst_z, consts.inertiaMatrix);
    _mixer.SetParameters(quadArmLength, quadArmAngle, propellerThrustFromSpeedSqr, propellerTorqueFromThrust, prop0SpinDir, consts.maxThrustPerPropeller,
                         consts.minThrustPerPropeller, consts.maxCmdTotalThrust);
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
    _tiltrotorController = TiltrotorController(natFreq, dampingRatio, timeConstAngleRP, timeConstAngleY, timeConstRatesRP, timeConstRatesY, mass, inertiaMatrix, lx, ly, thrustToTorque);
    bidirectional = consts.bidirectional;

    _timer.Reset();

    if (consts.valid)
    {
        _state = FS_IDLE;
    }
    else
    {
        _state = FS_KILLED;
        _firstPanicReason = PANIC_KILLED_INTERNALLY;
    }

    _kf.Reset();

    for (int i = 0; i < 8; i++)
    {
        status_bits[i] = false;
    }

    /* Reset tilt angles */
    for (int i = 0; i < 2; i++)
    {
        lobot_servo.angle_command[i] = 0;
        lobot_servo.angle_feedback[i] = 0;
        lobot_servo.temperature_feedback[i] = 0;
        lobot_servo.voltage_feedback[i] = 0;
    }
}

void QuadcopterLogic::Run()
{
    // Quadcopter main loop
    if (_state == FS_UNINITIALIZED)
    {
        return;
    }
    _cycleCounter++;
    _monitorMainLoopPeriod.Update();
    dshot_direct = false;

    UpdateEstimator();             // Updates Kalman filter with accelerometer, rate gyro, and ultrawideband measurements
    ParseIncomingCommunications(); // Sets _state depending on which type of radio command we receive
    UpdateWarnings();              // Will send warnings over telemetry if something is wrong
    CheckPanicReasons();           // Will set _state to FS_PANIC if we fail a safety check

    // Temporary: output temperature to _debug. Feel free to remove:
    // _debug[0] = _imuTemperature.lowPass.GetValue();

    // If we're running a motor test:
    if (_testMotorsOn)
    {
        Vec3f const desTorque2 = _angVelCtrl.GetDesiredTorques(Vec3f(0, 0, 0), _kf.GetAngularVelocity());
        _mixer.GetMotorForces(_testMotorsThrustFrac * 9.81f * _mass, desTorque2, _desMotorForcesForTelemetry);

        // convert forces to speeds
        _mixer.PropellerSpeedsFromThrust(_desMotorForcesForTelemetry, _desMotorSpeeds);
        return;
    }

    /* Debug variables */
    // esp_debug_vals[0] = _desMotorForcesForTelemetry[0];
    // esp_debug_vals[1] = _desMotorForcesForTelemetry[1];
    // esp_debug_vals[2] = _desMotorForcesForTelemetry[2];
    // esp_debug_vals[3] = _desMotorForcesForTelemetry[3];
    // esp_debug_vals[4] = lobot_servo.angle_command[0];
    // esp_debug_vals[5] = lobot_servo.angle_command[1];
    // esp_debug_vals[6] = 0;
    // esp_debug_vals[7] = 0;
    // esp_debug_vals[0] = distance_sensor.distance;
    // esp_debug_vals[1] = vehicle_gps_position.eph;
    // esp_debug_vals[2] = vehicle_gps_position.epv;
    // esp_debug_vals[3] = float(vehicle_gps_position.satellites_used);
    // esp_debug_vals[4] = vehicle_attitude.ypr[0];
    // esp_debug_vals[5] = vehicle_attitude.ypr[1];
    // esp_debug_vals[6] = vehicle_attitude.ypr[2];
    // esp_debug_vals[7] = lobot_servo.angle_command[0];

    // Run the controller
    switch (_state)
    {
    case FS_FULLY_AUTONOMOUS:
        RunControllerFullyAutonomous();
        return;

    case FS_EXTERNAL_ACCELERATION_CONTROL_WITH_YAW:
        RunControllerExternalAccelerationWithYaw();
        return;

    case FS_EXTERNAL_ACCELERATION_CONTROL:
        RunControllerExternalAcceleration();
        return;

    case FS_EXTERNAL_RATES_CONTROL:
        RunControllerExternalRatesControl();
        return;

    /* Control motor speeds directly */
    case FS_EXTERNAL_SPEEDS_CONTROL:
        RunControllerExternalSpeedsControl();
        return;

    case FS_EXTERNAL_TILTROTOR_CONTROL:
        RunControllerExternalTiltrotor();
        return;

    default:
    case FS_PANIC:
    case FS_KILLED:
        break;
    }

    /* Idle state */
    for (int i = 0; i < 4; i++)
    {
        _desMotorSpeeds[i] = 0;
        _desMotorForcesForTelemetry[i] = 0;
    }
    for (int i = 0; i < 8; i++) {
        esp_debug_vals[i] = 0;
    }
    lobot_servo.angle_command[0] = 0;
    lobot_servo.angle_command[1] = 0;
    return;
}

void QuadcopterLogic::UpdateEstimator()
{
    if (_imuRateGyro.isNew && _imuAccelerometer.isNew)
    {

        _kf.Predict(_imuRateGyro.lowPass.GetValue(), _imuAccelerometer.lowPass.GetValue());

        if (_gyroCalibrationEnabled)
        {
            // use raw measurement for bias calculation
            _gyroCalibrationAccumulated += _imuRateGyro.rawMeas;
            _gyroCalibrationNumSamples++;
        }

        _imuRateGyro.isNew = false;
        _imuAccelerometer.isNew = false;
    }

    _shouldStartNewUWBConversation = false;
    // ugly HACK to get comms started
    // TODO: How do we resolve this in a nicer way?
    if (_cycleCounter == 100)
    {
        // force the first message
        _shouldStartNewUWBConversation = true;
    }

    if (_uwbRangeMeas.isNew)
    {
        _uwbRangeMeas.isNew = false;
        _shouldStartNewUWBConversation = true;

        // keep track of the diagnostics:
        UpdateRangingDiagnostics(_uwbRangeMeas.targetId, _uwbRangeMeas.range, _uwbRangeMeas.failure);

        if (_uwbRangeMeas.failure)
        {
            // retry the same range.
            //_nextRangingTargetIdx = _nextRangingTargetIdx;
            // Don't retry same range:
            _nextRangingTargetIdx = (_nextRangingTargetIdx + 1) % _numRangingTargets;
        }
        else
        {
            // successful ranging

            _uwbRangeMeas.count++;

            // set up next ranging:
            _nextRangingTargetIdx = (_nextRangingTargetIdx + 1) % _numRangingTargets;

            Vec3f targetPos;
            // make sure we know where this anchor is located:
            if (0 == GetRangingTargetPosition(_uwbRangeMeas.targetId, targetPos))
            {
                _kf.UpdateWithRangeMeasurement(targetPos, _uwbRangeMeas.range);
            }
        }
    }
}

void QuadcopterLogic::ParseIncomingCommunications()
{
    if (_radioMessage.isNew)
    {
        _radioMessage.isNew = false;

        // Never leave panic or killed state until the vehicle is reset
        if ((_state != FS_PANIC) && (_state != FS_KILLED))
        {
            switch (_radioMessage.msg.type)
            {
            case RadioTypes::emergencyKill:
                _state = FS_KILLED;
                if (!_firstPanicReason)
                {
                    _firstPanicReason = PANIC_KILLED_EXTERNALLY;
                }
                break;
            case RadioTypes::positionCommand:
                _state = FS_FULLY_AUTONOMOUS;
                break;
            case RadioTypes::externalAccelerationWithYawCmd:
                _state = FS_EXTERNAL_ACCELERATION_CONTROL_WITH_YAW;
                break;
            case RadioTypes::externalAccelerationCmd:
                _state = FS_EXTERNAL_ACCELERATION_CONTROL;
                break;
            case RadioTypes::externalRatesCmd:
                _state = FS_EXTERNAL_RATES_CONTROL;
                break;
            case RadioTypes::externalSpeedsCmd:
                _state = FS_EXTERNAL_SPEEDS_CONTROL;
                break;
            case RadioTypes::externalTiltrotorCmd:
                _state = FS_EXTERNAL_TILTROTOR_CONTROL;
                break;
            case RadioTypes::idleCommand:
                _state = FS_IDLE;
                break;
            } // switch (_radioMessage.msg.type)
        }     // if ((_state != FS_PANIC) && (_state != FS_KILLED))
    }
}

void QuadcopterLogic::UpdateWarnings()
{
    // Update battery measurement data
    _battMeas.isNew = false;

    if (_battMeas.voltageFiltered <= _battThresholdWarningVoltage)
    {
        _telWarnings |= TelemetryPacket::WARN_LOW_BATT;
    }

    // Warn if we are receiving commands 10% slower or faster than expected
    if (fabsf(_monitorCmdRate.lpDt - _radioCmdPeriod) > (0.1f * _radioCmdPeriod))
    {
        _telWarnings |= TelemetryPacket::WARN_CMD_RATE;
    }

    // check for large groups of dropped packets
    const int WARN_BATCH_CMD_DROP_NUM = 3;
    if (_timeSinceLastRadioMessage.GetSeconds_f() > WARN_BATCH_CMD_DROP_NUM * _radioCmdPeriod)
    {
        _telWarnings |= TelemetryPacket::WARN_CMD_BATCH_DROP;
    }

    // Warn if we are running 5% slower or faster than expected
    if (fabsf(_monitorMainLoopPeriod.lpDt - _onboardPeriod) > (0.05f * _onboardPeriod))
    {
        _telWarnings |= TelemetryPacket::WARN_ONBOARD_FREQ;
    }

    // onboard estimator resets:
    if (_kf.GetWasResetSinceLastCheck())
    {
        _monitorUWBEstimatorResets.timeSinceLast.Reset();
    }

    const float WARNING_WINDOW_TIME_EST_RESET = 0.02f; // Send reset signal for 10 cycles to ensure warning is received
    if (_monitorUWBEstimatorResets.timeSinceLast.GetSeconds_f() < WARNING_WINDOW_TIME_EST_RESET)
    {
        _telWarnings |= TelemetryPacket::WARN_UWB_RESET;
    }
}

void QuadcopterLogic::CheckPanicReasons()
{
    // temp variables for easier access:
    Vec3f const estPos = _kf.GetPosition();
    Rotationf const estAtt = _kf.GetAttitude();

    int unsafeReason = 0;
    if (GetAreMotorsRunning())
    {
        const float MIN_SANE_ESTIMATOR_HEIGHT = -2.0f;
        if ((estPos.z < MIN_SANE_ESTIMATOR_HEIGHT) and !(_radioMessage.msg.flags & RadioTypes::ReservedFlags::disableOnboardStateSafetyChecks))
        {
            unsafeReason = PANIC_ONBOARD_ESTIMATE_CRAZY;
        }

        const unsigned NO_UWB_PANIC_TIMEOUT_us = 1500 * 1000;
        const unsigned NO_RADIO_CMD_PANIC_TIMEOUT_us = 1500 * 1000;
        if ((_timeSinceLastUwb.GetMicroSeconds() > NO_UWB_PANIC_TIMEOUT_us) && (FS_FULLY_AUTONOMOUS == _state))
        {
            unsafeReason = PANIC_UWB_TIMEOUT;
        }

        if ((estAtt * Vec3f(0, 0, 1)).z < -0.7 and !(_radioMessage.msg.flags & RadioTypes::ReservedFlags::disableOnboardStateSafetyChecks))
        {
            // pointing downwards
            unsafeReason = PANIC_UPSIDE_DOWN;
        }

        if (_timeSinceLastRadioMessage.GetMicroSeconds() > NO_RADIO_CMD_PANIC_TIMEOUT_us)
        {
            // Lost the radio connection
            unsafeReason = PANIC_RADIO_CMD_TIMEOUT;
        }

        if (_battMeas.voltageFiltered <= _battThresholdCriticalVoltage)
        {
            // Battery too low
            unsafeReason = PANIC_LOW_BATTERY;
        }
    }
    // TODO: More state safety checks

    if (unsafeReason && AreInSafetyCriticalFlightState())
    {
        if (_state != FS_PANIC)
        {
            _state = FS_PANIC;
            _firstPanicReason = unsafeReason;
        }
    }
}

void QuadcopterLogic::RunControllerFullyAutonomous()
{
    Vec3f const estPos = vehicle_odometry.position;
    Vec3f const estVel = vehicle_odometry.velocity;
    Rotationf const estAtt = vehicle_odometry.q;
    Vec3f const estAngVel = vehicle_odometry.omega;

    // read the radio:
    for (int i = 0; i < 3; i++)
    {
        _desPos[i] = _radioMessage.msg.floats[i];
    }
    float desYaw = _radioMessage.msg.floats[9];

    // todo: Yaw angle

    // Run the controller
    Vec3f const desAcc = _posCtrl.GetDesAcceleration(estPos, estVel, _desPos);

    Vec3f const desProperAcc = desAcc + Vec3f(0, 0, 9.81f);
    // Compute total thrust, thrust direction:
    float const normDesProperAcc = desProperAcc.GetNorm2();
    Vec3f const desThrustDir = desProperAcc / normDesProperAcc;

    // this ensures that we always produce the required z-acceleration (up to some maximum)
    float const MIN_THRUST_CORR_FAC = 1.00f;
    float const thrustCorrFactor = (estAtt * Vec3f(0, 0, 1)).z;
    float const thrustCorrFactorSaturated = thrustCorrFactor < MIN_THRUST_CORR_FAC ? MIN_THRUST_CORR_FAC : thrustCorrFactor;
    const float totalNormThrust = normDesProperAcc / thrustCorrFactorSaturated;

    // Construct the attitude that would match this desired thrust direction
    Rotationf desAtt;

    Vec3f const e3(0, 0, 1);
    const float cosAngle = desThrustDir.Dot(e3);

    errno = 0;
    float angle = acosf(cosAngle);
    if (errno)
    {
        // acos failed:
        if (cosAngle < 0)
        {
            angle = float(M_PI);
        }
        else
        {
            angle = 0;
        }
    }

    Vec3f rotAx = e3.Cross(desThrustDir);
    const float n = rotAx.GetNorm2();
    if (n < 1e-6f)
    {
        desAtt = Rotationf::Identity();
    }
    else
    {
        desAtt = Rotationf::FromRotationVector(rotAx * (angle / n));
    }
    desAtt = desAtt * Rotationf::FromRotationVector(Vec3f(0, 0, desYaw));

    Vec3f const desAngVel = _attCtr.GetDesiredAngularVelocity(desAtt, estAtt);
    Vec3f const desTorque = _angVelCtrl.GetDesiredTorques(desAngVel, estAngVel);

    _mixer.GetMotorForces(totalNormThrust * _mass, desTorque, _desMotorForcesForTelemetry);
    // convert forces to speeds
    _mixer.PropellerSpeedsFromThrust(_desMotorForcesForTelemetry, _desMotorSpeeds);

    return;
}

void QuadcopterLogic::RunControllerExternalAccelerationWithYaw()
{
    Rotationf const estAtt = vehicle_odometry.q;
    Vec3f const estAngVel = vehicle_odometry.omega;

    Vec3f const desAcc = Vec3f(_radioMessage.msg.floats[0], _radioMessage.msg.floats[1], _radioMessage.msg.floats[2]);
    float const desYaw = _radioMessage.msg.floats[3];

    // Magic number!
    if (desAcc.z < -9.81f / 2)
    {
        for (int i = 0; i < 4; i++)
        {
            _desMotorSpeeds[i] = 0;
            _desMotorForcesForTelemetry[i] = 0;
        }
        return;
    }

    // Run the controller
    Vec3f const desProperAcc = desAcc + Vec3f(0, 0, 9.81f);

    // Compute total thrust, thrust direction:
    const float totalNormThrust = desProperAcc.GetNorm2();
    Vec3f const desThrustDir = desProperAcc / totalNormThrust;

    // Construct the attitude that would match this desired thrust direction
    Rotationf desAtt;

    Vec3f const e3(0, 0, 1);
    const float cosAngle = desThrustDir.Dot(e3);

    errno = 0;
    float angle = acosf(cosAngle);
    if (errno)
    {
        // acos failed:
        if (cosAngle < 0)
        {
            angle = float(M_PI);
        }
        else
        {
            angle = 0;
        }
    }

    Vec3f rotAx = e3.Cross(desThrustDir);
    const float n = rotAx.GetNorm2();
    if (n < 1e-6f)
    {
        desAtt = Rotationf::Identity();
    }
    else
    {
        desAtt = Rotationf::FromRotationVector(rotAx * (angle / n));
    }

    // we need to construct a zero-yaw attitude estimate, since the controller is sending us commands in this frame
    float y, p, r;
    estAtt.ToEulerYPR(y, p, r);
    Rotationf estAttNoYaw = Rotationf::FromEulerYPR(0, p, r);

    Vec3f desAngVel = _attCtr.GetDesiredAngularVelocity(desAtt, estAttNoYaw);
    desAngVel.z = (desYaw - y) * (1.0f / _attCtr.GetTimeConst_z());

    Vec3f const desTorque = _angVelCtrl.GetDesiredTorques(desAngVel, estAngVel);

    _mixer.GetMotorForces(totalNormThrust * _mass, desTorque, _desMotorForcesForTelemetry);
    // convert forces to speeds
    _mixer.PropellerSpeedsFromThrust(_desMotorForcesForTelemetry, _desMotorSpeeds);

    return;
}

void QuadcopterLogic::RunControllerExternalAcceleration()
{
    Rotationf const estAtt = _kf.GetAttitude();
    Vec3f const estAngVel = _kf.GetAngularVelocity();

    Vec3f const desAcc = Vec3f(_radioMessage.msg.floats[0], _radioMessage.msg.floats[1], _radioMessage.msg.floats[2]);
    float const desYawRate = _radioMessage.msg.floats[3];

    // Magic number!
    if (desAcc.z < -9.81f / 2)
    {
        for (int i = 0; i < 4; i++)
        {
            _desMotorSpeeds[i] = 0;
            _desMotorForcesForTelemetry[i] = 0;
        }
        return;
    }

    // Run the controller
    Vec3f const desProperAcc = desAcc + Vec3f(0, 0, 9.81f);

    // Compute total thrust, thrust direction:
    const float totalNormThrust = desProperAcc.GetNorm2();
    Vec3f const desThrustDir = desProperAcc / totalNormThrust;

    // Construct the attitude that would match this desired thrust direction
    Rotationf desAtt;

    Vec3f const e3(0, 0, 1);
    const float cosAngle = desThrustDir.Dot(e3);

    errno = 0;
    float angle = acosf(cosAngle);
    if (errno)
    {
        // acos failed:
        if (cosAngle < 0)
        {
            angle = float(M_PI);
        }
        else
        {
            angle = 0;
        }
    }

    Vec3f rotAx = e3.Cross(desThrustDir);
    const float n = rotAx.GetNorm2();
    if (n < 1e-6f)
    {
        desAtt = Rotationf::Identity();
    }
    else
    {
        desAtt = Rotationf::FromRotationVector(rotAx * (angle / n));
    }

    // we need to construct a zero-yaw attitude estimate, since the controller is sending us commands in this frame
    float y, p, r;
    estAtt.ToEulerYPR(y, p, r);
    Rotationf estAttNoYaw = Rotationf::FromEulerYPR(0, p, r);

    Vec3f desAngVel = _attCtr.GetDesiredAngularVelocity(desAtt, estAttNoYaw);
    desAngVel.z = desYawRate;

    Vec3f const desTorque = _angVelCtrl.GetDesiredTorques(desAngVel, estAngVel);

    _mixer.GetMotorForces(totalNormThrust * _mass, desTorque, _desMotorForcesForTelemetry);
    // convert forces to speeds
    _mixer.PropellerSpeedsFromThrust(_desMotorForcesForTelemetry, _desMotorSpeeds);

    return;
}

void QuadcopterLogic::RunControllerExternalTiltrotor()
{
    // Update states
    Vec3f curAngVel = vehicle_odometry.omega;
    // Update inputs
    float desThrustNorm = _radioMessage.msg.floats[0];
    Vec3f desAngVel = Vec3f(_radioMessage.msg.floats[1], _radioMessage.msg.floats[2], _radioMessage.msg.floats[3]);
    float netTiltAngle = _radioMessage.msg.floats[4];
    // Run controller
    Vec3f desAngAcc = _tiltrotorController.get_angular_acceleration(desAngVel, curAngVel, netTiltAngle);
    _tiltrotorController.get_motor_force_cmd(desThrustNorm, desAngAcc, netTiltAngle);
    // Updates outputs
    for (int i = 0; i < 4; i++) {
        _desMotorForcesForTelemetry[i] = _tiltrotorController.motForceCmds[i];
    }
    _mixer.PropellerSpeedsFromThrust(_desMotorForcesForTelemetry, _desMotorSpeeds);
    // Lower prop thrust compensation
    float x = netTiltAngle;
    float f = -0.3399*x*x*x + 1.208*x*x - 1.143*x + 1.326;
    f = (netTiltAngle < 0.67) * 1 + (netTiltAngle >= 0.67 && netTiltAngle < 1.44)*f + (netTiltAngle >= 1.44) * 1.1661;
    for (int i = 1; i < 3; i++) {
        _desMotorSpeeds[i] = _desMotorSpeeds[i] * f;
    }
    for (int i = 0; i < 2; i++) {
        lobot_servo.angle_command[i] = _tiltrotorController.desTiltAngles[i];
    }

    esp_debug_vals[0] = desThrustNorm;
    esp_debug_vals[1] = desAngVel.x;
    esp_debug_vals[2] = desAngVel.y;
    esp_debug_vals[3] = desAngVel.z;
    esp_debug_vals[4] = netTiltAngle;
    return;
}

void QuadcopterLogic::RunControllerExternalRatesControl()
{
    Vec3f const estAngVel = _kf.GetAngularVelocity();

    float totalNormThrust = _radioMessage.msg.floats[0];
    Vec3f desAngVel = Vec3f(_radioMessage.msg.floats[1], _radioMessage.msg.floats[2], _radioMessage.msg.floats[3]);

    Vec3f const desTorque = _angVelCtrl.GetDesiredTorques(desAngVel, estAngVel);

    _mixer.GetMotorForces(totalNormThrust * _mass, desTorque, _desMotorForcesForTelemetry);
    _mixer.PropellerSpeedsFromThrust(_desMotorForcesForTelemetry, _desMotorSpeeds);

    // Propeller calibration:
    if (_radioMessage.msg.flags & RadioTypes::ReservedFlags::calibrateMotors)
    {
        // run the motor constant calibration
        if (!_propellerCalibration.currentlyRunning)
        {
            // start a new calibration!
            _propellerCalibration.currentlyRunning = true;
            _propellerCalibration.accumulatorCount = 0;
            for (int i = 0; i < 4; i++)
            {
                _propellerCalibration.accumulators[i] = 0;
            }
        }

        // keep track of what we're commanding
        for (int i = 0; i < 4; i++)
        {
            _propellerCalibration.accumulators[i] += _mixer.GetUncorrectedForce(_desMotorSpeeds[i]);
        }
        _propellerCalibration.accumulatorCount++;
    }
    else if (_propellerCalibration.currentlyRunning)
    {
        // just finished running calibration. Compute & store!
        _propellerCalibration.currentlyRunning = false;

        // do we have enough data?
        if (_propellerCalibration.accumulatorCount >= _propellerCalibration.minAccumulatorCount)
        {
            // in hover, each propeller should support one quarter vehicle weight
            float truePerPropellerForce = _mass * 9.81f / 4.0f;
            for (int i = 0; i < 4; i++)
            {
                float f = (_propellerCalibration.accumulatorCount * truePerPropellerForce) / _propellerCalibration.accumulators[i];
                // apply bounds:
                if (f > _propellerCalibration.calibrationFactorMax)
                {
                    f = _propellerCalibration.calibrationFactorMax;
                }
                if (f < _propellerCalibration.calibrationFactorMin)
                {
                    f = _propellerCalibration.calibrationFactorMin;
                }
                _propellerCalibration.activeFactors[i] = f;
            }
            _mixer.SetPropellerCorrectionFactors(_propellerCalibration.activeFactors);
            _shouldWriteParameters = true;
        }
    }

    return;
}

void QuadcopterLogic::RunControllerExternalSpeedsControl()
{
    for (int i = 0; i < 4; i++)
    {
        _desMotorSpeeds[i] = _radioMessage.msg.floats[i];
        _desMotorForcesForTelemetry[i] = _mixer.GetUncorrectedForce(_desMotorSpeeds[i]);
        dshot_direct = _radioMessage.msg.flags;
        dshot_values[i] = _radioMessage.msg.dshot_values[i];
        esp_debug_vals[i] = (dshot_values[i] - 47.0f)/20.0f;
    }
    return;
}

int QuadcopterLogic::GetRangingTargetPosition(uint8_t const id, Vec3f &posOut)
{
    for (unsigned i = 0; i < _numRangingTargets; i++)
    {
        if (id == _rangingTargets[i].id)
        {
            posOut = _rangingTargets[i].position;
            return 0;
        }
    }

    // We don't know this ID!
    return -1;
}

void QuadcopterLogic::UpdateRangingDiagnostics(unsigned id, float range, bool failed)
{
    for (unsigned i = 0; i < _numRangingTargets; i++)
    {
        if (id == _rangingTargets[i].id)
        {
            if (failed)
            {
                _rangingTargets[i].numBadMeas++;
                _rangingTargets[i].lastMeas = -1.0f;
            }
            else
            {
                _rangingTargets[i].numGoodMeas++;
                _rangingTargets[i].lastMeas = range;
            }
            return;
        }
    }
    // We don't know this ID!
    return;
}

/* PX4-ESP telemetry packet construction */
void QuadcopterLogic::GetESPTelemetryDataPacket(TelemetryPacket::esp_data_packet_t &esp_data_packet)
{
    /* Create a telemetry packet */
    TelemetryPacket::TelemetryPacket telemetry_packet;
    /* Feed the packet with data */
    telemetry_packet.accel[0] = _imuAccelerometer.lowPass.GetValue().x;
    telemetry_packet.accel[1] = _imuAccelerometer.lowPass.GetValue().y;
    telemetry_packet.accel[2] = _imuAccelerometer.lowPass.GetValue().z;
    // telemetry_packet.gyro[0] = _imuRateGyro.lowPass.GetValue().x;
    // telemetry_packet.gyro[1] = _imuRateGyro.lowPass.GetValue().y;
    // telemetry_packet.gyro[2] = _imuRateGyro.lowPass.GetValue().z;
    /* Use KF value for external speed control */
    telemetry_packet.gyro[0] = _kf.GetAngularVelocity().x;
    telemetry_packet.gyro[1] = _kf.GetAngularVelocity().y;
    telemetry_packet.gyro[2] = _kf.GetAngularVelocity().z;
    telemetry_packet.position[0] = _kf.GetPosition().x;
    telemetry_packet.position[1] = _kf.GetPosition().y;
    telemetry_packet.position[2] = _kf.GetPosition().z;
    telemetry_packet.velocity[0] = _kf.GetVelocity().x;
    telemetry_packet.velocity[1] = _kf.GetVelocity().y;
    telemetry_packet.velocity[2] = _kf.GetVelocity().z;
    Vec3f att = _kf.GetAttitude().ToVectorPartOfQuaternion();
    telemetry_packet.attitude[0] = att.x;
    telemetry_packet.attitude[1] = att.y;
    telemetry_packet.attitude[2] = att.z;
    for (int i = 0; i < 4; i++)
    {
        telemetry_packet.motorForces[i] = _desMotorForcesForTelemetry[i];
        telemetry_packet.esc_rpm[i] = esc_telemetry.rpm[i];
    }
    telemetry_packet.voltage = _battMeas.voltageRaw;
    telemetry_packet.current = _battMeas.currentRaw;
    telemetry_packet.temperature =
        (esc_telemetry.temperature[0] + esc_telemetry.temperature[1] + esc_telemetry.temperature[2] + esc_telemetry.temperature[3]) / 4.0f;
    telemetry_packet.panicReason = _firstPanicReason;
    telemetry_packet.warnings = _telWarnings;
    // for (int i = 0; i < 8; i++) {
    //     status_bits[i] = i % 2;
    //     telemetry_packet.status |= (status_bits[i] << i);
    // }
    for (int i = 0; i < TelemetryPacket::ESPPacketSize::DEBUG; i++)
    {
        telemetry_packet.esp_debug_vals[i] = esp_debug_vals[i];
    }
    /* Encode the packet into raw bytes */
    EncodeESPTelemetryPacket(telemetry_packet, esp_data_packet);
    /* Reset warnings */
    _telWarnings = 0;
}

/* Constructs a telemetry packet using currently stored data */
void QuadcopterLogic::GetTelemetryDataPackets(TelemetryPacket::data_packet_t &dataPacket1, TelemetryPacket::data_packet_t &dataPacket2,
                                              TelemetryPacket::data_packet_t &dataPacket3, TelemetryPacket::data_packet_t &dataPacket4)
{

    // Packet 1
    TelemetryPacket::TelemetryPacket telemetryPacket; // telemetry packet
    telemetryPacket.type = TelemetryPacket::PACKET_TYPE_QUAD_TELEMETRY_PT1;
    telemetryPacket.packetNumber = _telPacketCounter % 256; // mod to fit in uint8

    telemetryPacket.accel[0] = _imuAccelerometer.lowPass.GetValue().x;
    telemetryPacket.accel[1] = _imuAccelerometer.lowPass.GetValue().y;
    telemetryPacket.accel[2] = _imuAccelerometer.lowPass.GetValue().z;

    // use the bias corrected value:
    telemetryPacket.gyro[0] = _imuRateGyro.lowPass.GetValue().x;
    telemetryPacket.gyro[1] = _imuRateGyro.lowPass.GetValue().y;
    telemetryPacket.gyro[2] = _imuRateGyro.lowPass.GetValue().z;

    for (int i = 0; i < 4; i++)
    {
        telemetryPacket.motorForces[i] = _desMotorForcesForTelemetry[i];
    }

    telemetryPacket.position[0] = _kf.GetPosition().x;
    telemetryPacket.position[1] = _kf.GetPosition().y;
    telemetryPacket.position[2] = _kf.GetPosition().z;

    telemetryPacket.battVoltage = _battMeas.voltageRaw;

    EncodeTelemetryPacket(telemetryPacket, dataPacket1);

    // Packet 2
    telemetryPacket.type = TelemetryPacket::PACKET_TYPE_QUAD_TELEMETRY_PT2;
    telemetryPacket.packetNumber = _telPacketCounter % 256; // mod for uint8

    telemetryPacket.velocity[0] = _kf.GetVelocity().x;
    telemetryPacket.velocity[1] = _kf.GetVelocity().y;
    telemetryPacket.velocity[2] = _kf.GetVelocity().z;

    Vec3f att = _kf.GetAttitude().ToVectorPartOfQuaternion();
    telemetryPacket.attitude[0] = att.x;
    telemetryPacket.attitude[1] = att.y;
    telemetryPacket.attitude[2] = att.z;

    for (int i = 0; i < TelemetryPacket::TelemetryPacket::NUM_DEBUG_FLOATS; i++)
    {
        telemetryPacket.debugVals[i] = _debug[i];
    }

    telemetryPacket.panicReason = _firstPanicReason;
    // encode the warnings:
    telemetryPacket.warnings = _telWarnings;

    EncodeTelemetryPacket(telemetryPacket, dataPacket2);

    if (TelemetryPacket::NUM_PACKETS_OVER_TELEMETRY >= 3)
    {
        // Packet 3
        telemetryPacket.type = TelemetryPacket::PACKET_TYPE_QUAD_TELEMETRY_PT3;
        telemetryPacket.packetNumber = _telPacketCounter % 256; // mod for uint8
        for (unsigned i = 0; i < TelemetryPacket::TelemetryPacket::NUM_FLOATS_PER_PACKET; i++)
        {
            telemetryPacket.customPacket1[i] = 0.f;
        }

        EncodeTelemetryPacket(telemetryPacket, dataPacket3);
    }

    if (TelemetryPacket::NUM_PACKETS_OVER_TELEMETRY >= 4)
    {
        // Packet 4
        telemetryPacket.type = TelemetryPacket::PACKET_TYPE_QUAD_TELEMETRY_PT4;
        telemetryPacket.packetNumber = _telPacketCounter % 256; // mod for uint8
        for (unsigned i = 0; i < TelemetryPacket::TelemetryPacket::NUM_FLOATS_PER_PACKET; i++)
        {
            telemetryPacket.customPacket2[i] = 0.f;
        }

        EncodeTelemetryPacket(telemetryPacket, dataPacket4);
    }

    // Increment counter for next packet
    _telPacketCounter++;

    // Reset warnings after sending them out
    _telWarnings = 0;
}

void QuadcopterLogic::PrintStatus() const // for debugging purposes
{
    printf("Quad logic debugging status, over %d cycles (avg dt = %3.5f, expected dt = %3.5f)\n", _cycleCounter, double(_monitorMainLoopPeriod.lpDt),
           double(_onboardPeriod));
    printf("Quad type = <%s>\n", QuadcopterConstants::GetNameString(_quadType));
    printf("Vehicle Id = <%d>\n", _myId);
    printf("\tState = ");
    switch (_state)
    {
    case FS_FULLY_AUTONOMOUS:
        printf("FS_FULLY_AUTONOMOUS");
        break;
    case FS_IDLE:
        printf("FS_IDLE");
        break;
    case FS_KILLED:
        printf("FS_KILLED");
        break;
    case FS_PANIC:
        printf("FS_PANIC");
        break;
    case FS_EXTERNAL_ACCELERATION_CONTROL:
        printf("FS_EXTERNAL_ACCELERATION_CONTROL");
        break;
    case FS_EXTERNAL_RATES_CONTROL:
        printf("FS_EXTERNAL_RATES_CONTROL");
        break;
    case FS_EXTERNAL_SPEEDS_CONTROL:
        printf("FS_EXTERNAL_SPEEDS_CONTROL");
        break;
    case FS_EXTERNAL_TILTROTOR_CONTROL:
        printf("FS_EXTERNAL_TILTROTOR_CONTROL");
        break;
    default:
        printf("<<UNKNOWN (%d)>>", int(_state));
    }
    printf("\n");

    printf("\tBattery:\n");
    printf("\t\tnum meas = %d, last = [(%.3fV) %.3fV, %.3fA]\n", _battMeas.count, double(_battMeas.voltageFiltered), double(_battMeas.voltageRaw),
           double(_battMeas.currentRaw));
    printf("\tAccelerometer:\n");
    printf("\t\tnum meas = %d, last = (%.3f, %.3f, %.3f)m/s**2\n", _imuAccelerometer.count, double(_imuAccelerometer.lowPass.GetValue().x),
           double(_imuAccelerometer.lowPass.GetValue().y), double(_imuAccelerometer.lowPass.GetValue().z));
    printf("\tTemperature:\n");
    printf("\t\tnum meas = %d, last = %.3fdeg C\n", _imuTemperature.count, double(_imuTemperature.lowPass.GetValue()));
    printf("\tRate gyro (raw):\n");
    printf("\t\tnum meas = %d, last = (%.3f, %.3f, %.3f)rad/s\n", _imuRateGyro.count, double(_imuRateGyro.rawMeas.x), double(_imuRateGyro.rawMeas.y),
           double(_imuRateGyro.rawMeas.z));
    printf("\tRate gyro (corr):\n");
    printf("\t\tnum meas = %d, last = (%.3f, %.3f, %.3f)rad/s\n", _imuRateGyro.count, double(_imuRateGyro.lowPass.GetValue().x),
           double(_imuRateGyro.lowPass.GetValue().y), double(_imuRateGyro.lowPass.GetValue().z));
    if (_uwbRangeMeas.count)
    {
        printf("\tUWB ranges:\n");
        printf("\t\tnum meas = %d, last id = %d, range = %.3fm\n", _uwbRangeMeas.count, _uwbRangeMeas.targetId, double(_uwbRangeMeas.range));
        printf("\t\tnext target = %d; all targets:\n", GetNextUWBRangingTarget());

        unsigned totalNumGood = 0;
        unsigned totalNumBad = 0;
        for (unsigned i = 0; i < _numRangingTargets; i++)
        {
            totalNumGood += _rangingTargets[i].numGoodMeas;
            totalNumBad += _rangingTargets[i].numBadMeas;
            float successRate = float(_rangingTargets[i].numGoodMeas) / float(_rangingTargets[i].numGoodMeas + _rangingTargets[i].numBadMeas);
            printf("\t\t\tID=%d, pos=<%.3f, %.3f, %.3f>m, total meas=%d (good: %d%%), last=%.3fm\n", _rangingTargets[i].id,
                   double(_rangingTargets[i].position.x), double(_rangingTargets[i].position.y), double(_rangingTargets[i].position.z),
                   int(_rangingTargets[i].numGoodMeas + _rangingTargets[i].numBadMeas), int(successRate * 100 + 0.5f), double(_rangingTargets[i].lastMeas));
        }
        float successRate = float(totalNumGood) / float(totalNumGood + totalNumBad);
        printf("\t\ttotal meas = %d (good = %d%%)\n", int(totalNumBad + totalNumGood), int(successRate * 100 + 0.5f));
    }
    else
    {
        printf("\tNo UWB measurements.\n");
    }
    printf("\tEstimator:\n");
    float y, p, r;
    _kf.GetAttitude().ToEulerYPR(y, p, r);

    printf("\t\tinit = %d,%d\n", int(_kf.GetIsIMUInitialized()), int(_kf.GetIsUWBInitialized()));
    printf("\t\tpos     = (%.3f, %.3f, %.3f)m\n", double(_kf.GetPosition().x), double(_kf.GetPosition().y), double(_kf.GetPosition().z));
    printf("\t\tvel     = (%.3f, %.3f, %.3f)m/s\n", double(_kf.GetVelocity().x), double(_kf.GetVelocity().y), double(_kf.GetVelocity().z));
    printf("\t\tatt YPR = (%.3f, %.3f, %.3f)rad\n", double(y), double(p), double(r));
    printf("\t\tangVel  = (%.3f, %.3f, %.3f)rad/s\n", double(_kf.GetAngularVelocity().x), double(_kf.GetAngularVelocity().y),
           double(_kf.GetAngularVelocity().z));
    printf("\t\tnum rejected meas = %d; num resets = %d\n", _kf.GetMeasurementRejectionCount(), _kf.GetResetCount());

    printf("\tDesired motor speeds:\n");
    printf("\t\t[%.3f, %.3f, %.3f, %.3f]\n", double(_desMotorSpeeds[0]), double(_desMotorSpeeds[1]), double(_desMotorSpeeds[2]), double(_desMotorSpeeds[3]));
    printf("\tPropeller correction factors:\n");
    printf("\t\t[%.3f, %.3f, %.3f, %.3f]\n", double(_mixer.GetPropellerCorrectionFactors(0)), double(_mixer.GetPropellerCorrectionFactors(1)),
           double(_mixer.GetPropellerCorrectionFactors(2)), double(_mixer.GetPropellerCorrectionFactors(3)));
    printf("\tRadio:\n");
    printf("\t\tcount = %d, type = %d, flags = %d\n\t\t", _radioMessage.count, _radioMessage.msg.type, _radioMessage.msg.flags);
    for (int i = 0; i < RadioTypes::RadioMessageDecoded::NUM_RADIO_FLOAT_FIELDS; i++)
    {
        printf("%.3f, ", double(_radioMessage.msg.floats[i]));
    }
    printf("\n");

    printf("\tNum telemetry sent out: %d\n", _telPacketCounter);

    printf("\tRadio command dt = %3.5fs (%3.5fs expected)\n", double(_monitorCmdRate.lpDt), double(_radioCmdPeriod));

    printf("\tDebug variables:\n");
    printf("\t\t[");
    for (int i = 0; i < NUM_DEBUG_VARS; i++)
    {
        printf("%.3f, ", double(_debug[i]));
    }
    printf("]\n");

    /* PX4-ESP printouts */
    printf("\tVehicle Odometry:\n");
    printf("\t\tposition = (%.3f, %.3f, %.3f)\n", double(vehicle_odometry.position[0]), double(vehicle_odometry.position[1]), double(vehicle_odometry.position[2]));
    printf("\t\tvelocity = (%.3f, %.3f, %.3f)\n", double(vehicle_odometry.velocity[0]), double(vehicle_odometry.velocity[1]), double(vehicle_odometry.velocity[2]));
    printf("\t\tq = (%.3f, %.3f, %.3f, %.3f)\n", double(vehicle_odometry.q[0]), double(vehicle_odometry.q[1]), double(vehicle_odometry.q[2]), double(vehicle_odometry.q[3]));
    printf("\t\tomega = (%.3f, %.3f, %.3f)\n", double(vehicle_odometry.omega[0]), double(vehicle_odometry.omega[1]), double(vehicle_odometry.omega[2]));
    printf("\t\tYPR = (%.3f, %.3f, %.3f)\n", double(vehicle_odometry.ypr[0]), double(vehicle_odometry.ypr[1]), double(vehicle_odometry.ypr[2]));

    printf("\tGPS:\n");
    printf("\t\teph, epv, num_sat = (%.3f, %.3f, %d)\n", double(vehicle_gps_position.eph), double(vehicle_gps_position.eph), vehicle_gps_position.satellites_used);
    printf("\tLocal Position:\n");
    printf("\t\tposition = (%.3f, %.3f, %.3f)\n", double(vehicle_local_position.position[0]), double(vehicle_local_position.position[1]), double(vehicle_local_position.position[2]));
    printf("\tVehicle Attitude:\n");
    printf("\t\tPX4 YPR = (%.3f, %.3f, %.3f)\n", double(vehicle_attitude.ypr[0]), double(vehicle_attitude.ypr[1]), double(vehicle_attitude.ypr[2]));

    printf("\tESP debug variables:\n");
    printf("\t\t[");
    for (int i = 0; i < TelemetryPacket::ESPPacketSize::DEBUG; i++)
    {
        printf("%.3f, ", double(esp_debug_vals[i]));
    }
    printf("]\n");

    /* Print status bits */
    printf("\tStatus bits:\n");
    printf("\t\t[");
    for (int i = 0; i < 8; i++)
    {
        printf("%d, ", status_bits[i]);
    }
    printf("]\n");

    printf("\tpanic reason = %d: %s \n", _firstPanicReason, GetPanicReasonString(PanicReason(_firstPanicReason)));

    printf("\twarnings:");
    for (int i = 0; i < 8; i++)
    {
        if ((_telWarnings >> i) & 0x01)
        {
            printf("%d ", 1 << i);
        }
    }
    printf("\n");

    printf("\n\n\n");
}
