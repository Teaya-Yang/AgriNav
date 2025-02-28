#pragma once

#include "ros/ros.h"

#include "Common/Math/Vec3.hpp"
#include "Common/DataTypes/RadioTypes.hpp"
#include "Common/DataTypes/TelemetryPacket.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Offboard/MocapStateEstimator.hpp"
#include "Components/Offboard/QuadcopterController.hpp"
#include "Components/Offboard/SafetyNet.hpp"
#include "Components/Logic/QuadcopterConstants.hpp"

#include "hiperlab_rostools/mocap_output.h"
#include "hiperlab_rostools/estimator_output.h"
#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/joystick_values.h"
#include "hiperlab_rostools/telemetry.h"

namespace Offboard {

class VehicleStateMachine_UWBDemo {
 public:
  enum FlightStage {
    StageWaitForStart,
    StageTakeoff,
    StageFlightHover,
    StageFlightFlyCicle,
    StageLanding,
    StageComplete,
    StageEmergency,
  };

  enum MeasMode {
    MeasMocap,
    MeasUWB,
  };

  VehicleStateMachine_UWBDemo();

  bool GetIsEstInitialized(void) const {
    return _est->GetIsInitialized();
  }

  bool GetIsReadyToExit(void) const {
    return _vehicleIsReadyForProgramToExit;
  }

  void Initialize(int id, std::string name, ros::NodeHandle &n,
                  BaseTimer* timer, double systemLatencyTime);

  void CallbackEstimator(const hiperlab_rostools::mocap_output& msg);
  void CallbackTelemetry(const hiperlab_rostools::telemetry& msg);

  void Run(bool shouldStart, bool shouldStop, bool btnGreen, bool btnYellow,
           bool btnBlue);

  void SetDesiredPosition(Vec3d newPos) {
    _desiredPosition = newPos;
  }

  void SetDesiredYaw(double newYaw) {
    _desiredYawAngle = newYaw;
  }

  void SetExternalPanic() {
    _safetyNet->SetUnsafe();
  }

  void SetSafeCorners(Vec3d minCorner, Vec3d maxCorner,
                      double minNormalHeight) {
    _safetyNet->SetSafeCorners(minCorner, maxCorner, minNormalHeight);
  }

  void SetCircleTrajectoryParameters(Vec3d center, double angVel, double radius,
                                     double dt) {
    _circleTrajectory.center = center;
    _circleTrajectory.angVel = angVel;
    _circleTrajectory.radius = radius;
    _circleTrajectory.dt = dt;
  }

  void PrintEstimationError();

 private:
  bool HaveLowBattery() {
    return _lastTelWarnings & TelemetryPacket::WARN_LOW_BATT;
  }
  int _id;
  std::shared_ptr<MocapStateEstimator> _est;
  double _systemLatencyTime;  //[s] as used by estimator
  std::shared_ptr<QuadcopterController> _ctrl;
  std::shared_ptr<SafetyNet> _safetyNet;
  std::string _name;

  std::shared_ptr<ros::Subscriber> _subMocap, _subTelemetry;
  std::shared_ptr<ros::Publisher> _pubEstimate, _pubCmd;

//state info:
  FlightStage _flightStage, _lastFlightStage;
  std::shared_ptr<Timer> _stageTimer;  //keep track of time we've been in current stage
  std::shared_ptr<Timer> _timerRun;

  Vec3d _initPosition;
  Vec3d _desiredPosition;
  double _desiredYawAngle;

  struct {
    Vec3<volatile double> pos, vel;
    Rotation<volatile double> att;
  } _uwbEst;

  struct UWBEstError {
    Vec3d pos, vel;
    Rotationd att;
    double lpTimeConst;  //[s]
  } _uwbEstError;

  bool _vehicleIsReadyForProgramToExit;
  volatile uint8_t _lastTelWarnings;

  struct {
    Vec3d center;
    double angVel, radius, dt, phase;
    bool lastButtonState;
  } _circleTrajectory;

  MeasMode _measMode;
};

}  // namespace Offboard

