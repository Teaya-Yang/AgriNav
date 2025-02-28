#pragma once

#include "ros/ros.h"

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

struct Statistics {
  void Reset() {
    _sumVals = 0;
    _sumValsSqr = 0;
    _numCounts = 0;
  }
  void Update(double in) {
    _sumVals += in;
    _sumValsSqr += in * in;
    _numCounts++;
  }
  double GetMean() {
    return _sumVals / _numCounts;
  }
  double GetVariance() {
    return (_sumValsSqr - _numCounts * GetMean() * GetMean()) / (_numCounts - 1);
  }
  double GetStdDev() {
    return sqrt(GetVariance());
  }
  void PrintStats() {
    printf("%+5.3f", GetMean());
    printf("+/-(");
    printf("%5.3f", GetStdDev());
    printf(")");
  }
 private:
  double _sumVals, _sumValsSqr;
  unsigned _numCounts;
};

class QuadEvaluatorStateMachine {
 public:
  enum FlightStage {
    StageWaitForStart,
    StageTakeoff,
    StageSettle,
    StageRunAveraging,
    StageLanding,
    StageComplete,
    StageEmergency,
  };

  QuadEvaluatorStateMachine();

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

  void Run(bool shouldStart, bool shouldStop);

  void PublishEstimate(MocapStateEstimator::MocapEstimatedState estState);

  hiperlab_rostools::radio_command RunControllerAndUpdateEstimator(
      MocapStateEstimator::MocapEstimatedState estState, Vec3d desPos,
      Vec3d desVel, Vec3d desAcc, float desYaw);

  void SetDesiredPosition(Vec3d newPos) {
    _desiredPosition = newPos;
  }

  void SetExternalPanic() {
    _safetyNet->SetUnsafe();
  }

  void SetMass(double m) {
    _mass = m;
  }

  void PrintResults();

 private:
  bool HaveLowBattery() const {
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

  volatile uint8_t _lastTelWarnings;

  Vec3d _initPosition;
  Vec3d _desiredPosition;
  bool _vehicleIsReadyForProgramToExit;

  enum {
    NUM_YAW_ANGLES = 5,
  };

  Statistics _avgForce[4][NUM_YAW_ANGLES];
  Statistics _avgPos[3][NUM_YAW_ANGLES];
  Statistics _avgAtt[3][NUM_YAW_ANGLES];
  Statistics _avgRateGyro[3];
  Statistics _avgAcc[3];

  double _lastTelForces[4];
  Vec3d _lastMocapPos, _lastMocapAtt;
  Vec3d _lastTelRateGyro, _lastTelAcc;

  unsigned _avgCntr;
  double _timeSettle, _timeAverage;
  double _mass;
};

}  // namespace Offboard
