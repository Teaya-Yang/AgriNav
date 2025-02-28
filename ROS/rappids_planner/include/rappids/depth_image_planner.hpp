/*!
 * Rectangular Pyramid Partitioning using Integrated Depth Sensors
 *
 * Copyright 2020 by Nathan Bucki <nathan_bucki@berkeley.edu>
 *
 * This code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the code.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include <memory>
#include <iostream>
#include <chrono>
#include <stdint.h>
#include <random>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../math/vec3.hpp"
#include "../math/rotation.hpp"
#include "../math/trajectory.hpp"
#include "../math/quartic.h"
#include "rapid_trajectory_generator.hpp"
#include "pyramid.hpp"
#include "monotonic_trajectory.hpp"
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace CommonMath;
using namespace RapidQuadrocopterTrajectoryGenerator;

namespace RectangularPyramidPlanner {

enum TrajectoryTestResult {
  None = 0,
  LowCost = 1 << 0,
  DynamicsFeasible = 1 << 1,
  VelocityAdmissible = 1 << 2,
  CollisionFree = 1 << 3,
};
inline TrajectoryTestResult operator~ (TrajectoryTestResult a) { return (TrajectoryTestResult)~(int)a; }
inline TrajectoryTestResult operator| (TrajectoryTestResult a, TrajectoryTestResult b) { return (TrajectoryTestResult)((int)a | (int)b); }
inline TrajectoryTestResult operator& (TrajectoryTestResult a, TrajectoryTestResult b) { return (TrajectoryTestResult)((int)a & (int)b); }
inline TrajectoryTestResult operator^ (TrajectoryTestResult a, TrajectoryTestResult b) { return (TrajectoryTestResult)((int)a ^ (int)b); }


struct TrajectoryTest {
  TrajectoryTest(TrajectoryTestResult result, RapidTrajectoryGenerator traj) : result(result), traj(traj) {
  }

  TrajectoryTestResult result;
  RapidTrajectoryGenerator traj;
};

struct SamplingParameters{
  double minDepth;
  double maxDepth; //Minimum and maximum sample depth.
  double minTime;
  double maxTime; //The minimum and maximum sampling time.
};

class DepthImagePlanner {
 public:

  //! Constructor. Requires a depth image and the related camera intrinsics
  /*!
   * @param depthImage The depth image stored as a 16 bit single channel image (CV_16UC1)
   * @param depthScale The conversion between pixel value and the associated depth value.
   * For example: depthInMeters = depthScale * valueStoredInPixel
   * @param focalLength The focal length of the camera that took the depth image.
   * @param principalPointX The location in pixel coordinates of the center of the image horizontally (usually half the width)
   * @param principalPointY The location in pixel coordinates of the center of the image vertically (usually half the height)
   * @param physicalVehicleRadius The true radius of the vehicle. Any depth values closer than this distance to the camera
   * will be ignored. [meters]
   * @param vehicleRadiusForPlanning We plan as if the vehicle has this radius. This value should be slightly larger than
   * physicalVehicleRadius to account for pose estimation errors and trajectory tracking errors. [meters]
   * @param minimumCollisionDistance We do not perform collision checking on parts of the candidate trajectory closer than
   * this distance to the camera. Obstacles closer than this distance can still occlude other parts of the trajectory (causing
   * them to be labeled as in collision). This is used to enforce field of view constraints as well; we assume there is an
   * obstacle just outside of the field of view this distance away from the camera. [meters]
   */
  DepthImagePlanner(cv::Mat depthImage, double depthScale, double focalLength,
                    double principalPointX, double principalPointY,
                    double physicalVehicleRadius,
                    double vehicleRadiusForPlanning,
                    double minimumCollisionDistance,
                    double minimumAllowedThrust,
                    double maximumAllowedThrust,
                    double maximumAllowedAngularVelocity,
                    double maximumAllowedVelocity, 
                    double minimumAllowedHeight);

  //! Finds the trajectory with the lowest user-provided cost. The default settings of
  //! RandomTrajectoryGenerator are used to generate candidate trajectories.
  /*!
   * @param trajectory A trajectory that defines the state of the vehicle when the depth image was taken
   * @param allocatedComputationTime The planner will exit after this amount of time has passed [seconds]
   * @param samplingParams The parameters about motion primitive sampling
   * @param currentGoalImage The current goal point in the image frame.
   * @return True if a feasible trajectory was found, false otherwise. If a feasible trajectory was found,
   * the argument trajectory will be updated to contain the lowest cost trajectory found.
   */
  bool FindLowestCostTrajectory(
      RapidTrajectoryGenerator& trajectory,
      std::vector<TrajectoryTest>& trajectories,
      double& speed_cost,
      const double& allocatedComputationTime,
      const SamplingParameters& samplingParams,
      const Vec3& currentGoalImage,
      const Rotation& trajAtt,
      const Vec3& trajInitPos);
  
  //! When perception-awareness is considered
  //! Finds the trajectory with the lowest user-provided cost. The default settings of
  //! RandomTrajectoryGenerator are used to generate candidate trajectories.
  /*!
   * @param trajectory A trajectory that defines the state of the vehicle when the depth image was taken
   * @param allocatedComputationTime The planner will exit after this amount of time has passed [seconds]
   * @param samplingParams The parameters about motion primitive sampling
   * @param currentGoalImage The current goal point in the image frame.
   * @param featuresOdom The VIO feature points in the odometry frame.
   * @param camAtt The attitude of the camera in the vehicle's body frame.
   * @param estVehicleAtt The attitude of the vehicle in the odometry frame.
   * @param estVehiclePos The position of the vehicle in the odometry frame.
   * @param imageFPS The frame rate of the depth image.
   * @return True if a feasible trajectory was found, false otherwise. If a feasible trajectory was found,
   * the argument trajectory will be updated to contain the lowest cost trajectory found.
   */
  bool FindLowestCostTrajectory(
      RapidTrajectoryGenerator& trajectory,
      std::vector<TrajectoryTest>& trajectories,
      double& speed_cost,
      double& perception_cost,
      const double& k_perc,
      const double& allocatedComputationTime,
      const SamplingParameters& samplingParams,
      const Vec3& currentGoalImage,
      const std::vector<Vec3>& featuresOdom,
      const Rotation& camAtt,
      const Rotation& estVehicleAtt,
      const Vec3& estVehiclePos,
      const int imageFPS,
      const double exposureTime);

  /*!
   * @return A vector containing all of the generated pyramids
   */
  std::vector<Pyramid> GetPyramids() {
    return _pyramids;
  }
  /*!
   * @return The number of pyramids generated
   */
  int GetNumPyramids() {
    return _pyramids.size();
  }
  /*!
   * @return The number of candidate trajectories generated by the planner
   */
  int GetNumTrajectoriesGenerated() {
    return _numTrajectoriesGenerated;
  }
  /*!
   * @return The number of trajectories checked for collisions (i.e. not filtered by cost or dynamic feasibility first)
   */
  int GetNumCollisionChecks() {
    return _numCollisionChecks;
  }
  /*!
   * @return The number of trajectories checked for velocities
   */
  int GetNumVelocityChecks() {
    return _numVelocityChecks;
  }
  /*!
   * @return The number of trajectories checked for cost
   */
  int GetNumCostChecks() {
    return _numCostChecks;
  }
  /*!
   * @return The number of trajectories collision-free
   */
  int GetNumCollisionFree() {
    return _numCollisionFree;
  }
  /*!
   * @return The time spent inside the InflatePyramid function [seconds]
   */
  double GetInflatePyramidTime() {
    return _pyramidGenTimeNanoseconds * 1e-9;
  }
  double GetFocalLength() {
    return _focalLength;
  }
  double GetPrincipalPointX() {
    return _cx;
  }
  double GetPrincipalPointY() {
    return _cy;
  }
  int GetImageWidth() {
    return _imageWidth;
  }
  int GetImageHeight() {
    return _imageHeight;
  }
  int GetImageEdgeOffset(){
    return int (_focalLength * _trueVehicleRadius / _minCheckingDist);
  }
  int GetRandomSeed() {
    return _randomSeed;
  }
  void SetRandomSeed(int rdseed) {  // Change random seed from the default 0
    _randomSeed = rdseed;
  }

  //! Sets the parameters used to check the dynamic feasibility of each candidate trajectory.
  /*!
   * @param minimumAllowedThrust The minimum allowed thrust (used for checking dynamic feasibility) [m/s^2]
   * @param maximumAllowedThrust The maximum allowed thrust (used for checking dynamic feasibility) [m/s^2]
   * @param maximumAllowedAngularVelocity The maximum allowed angular velocity (used for checking dynamic feasibility) [rad/s]
   * @param minimumSectionTime Minimum time section to test when checking for dynamic feasibility [seconds]
   */
  void SetDynamicFeasiblityParameters(double minimumAllowedThrust,
                                      double maximumAllowedThrust,
                                      double maximumAllowedAngularVelocity,
                                      double minimumSectionTime) {
    _minimumAllowedThrust = minimumAllowedThrust;
    _maximumAllowedThrust = maximumAllowedThrust;
    _maximumAllowedAngularVelocity = maximumAllowedAngularVelocity;
    _minimumSectionTimeDynamicFeas = minimumSectionTime;
  }
  //! After this amount of time has been spent generating pyramids, no more pyramids will be generated (i.e. instead of trying to
  //! generate a new pyramid, the trajectory will simply be labeled as in-collision). [seconds]
  void SetMaxPyramidGenTime(double maxPyramidGenTime) {
    _maxPyramidGenTime = maxPyramidGenTime;
  }
  //! The maximum number of pyramids we allow the planner to generate. If the planner tries to generate a new pyramid,
  //! the trajectory will simply be labeled as in-collision.
  void SetMaxNumberOfPyramids(int maxNumPyramids) {
    _maxNumPyramids = maxNumPyramids;
  }

  //! Computes the 3D position of a point given a position in pixel coordinates and a depth
  /*!
   * @param x Input: The horizontal position of the pixel (should be between zero and the image width)
   * @param y Input: The vertical position of the pixel (should be between zero and the image height)
   * @param depth Input: The Z depth of the point [meters]
   * @param outPoint Output: The 3D position of the point (X points towards right edge of image, Y towards
   * bottom edge of image, and Z into the image)
   */
  void DeprojectPixelToPoint(double x, double y, double depth,
                             CommonMath::Vec3& outPoint) {
    outPoint = depth
        * CommonMath::Vec3((x - _cx) / _focalLength, (y - _cy) / _focalLength,
                           1);
  }

  //! Projects 3D point into the image, returning the pixel coordinates
  /*!
   * @param point Input: The position of the point in 3D space
   * @param outX Output: The horizontal position of the projected point in pixel coordinates
   * @param outY Output: The vertical position of the projected point in pixel coordinates
   */
  void ProjectPointToPixel(CommonMath::Vec3 point, double& outX, double& outY) {
    outX = point.x * _focalLength / point.z + _cx;
    outY = point.y * _focalLength / point.z + _cy;
  }

  //! Class used to generate random candidate trajectories for the planner to evaluate.
  //! All generated trajectories come to rest at the end of their duration.
  class RandomTrajectoryGenerator {
   public:
    //! Constructor allowing for custom bounds on the sampling distributions used to generate the trajectories.
    /*!
     * @param minXpix Should be between zero and maxXpix [units of pixels]
     * @param maxXpix Should be between minXpix and the image width [units of pixels]
     * @param minYpix Should be between zero and maxYpix [units of pixels]
     * @param maxYpix Should be between minYpix and the image height [units of pixels]
     * @param minDepth Only sample trajectories that come to rest farther than this distance from the focal point [meters]
     * @param maxDepth Only sample trajectories that come to rest closer than this distance from the focal point [meters]
     * @param minTime Minimum trajectory duration [seconds]
     * @param maxTime Maximum trajectory duration [seconds]
     * @param rdSeed Random seed
     * @param planner Pointer to the DepthImagePlanner object that will use the generated trajectories.
     * We include this pointer so that we can call the DeprojectPixelToPoint function defined by the
     * DepthImagePlanner object which uses the associated camera intrinsics.
     */
    RandomTrajectoryGenerator(const SamplingParameters& samplingParams,
                              DepthImagePlanner *planner)
        : _pixelX(planner->GetImageEdgeOffset(), planner->GetImageWidth() - planner->GetImageEdgeOffset()),
          _pixelY(planner->GetImageEdgeOffset(), planner->GetImageHeight() - planner->GetImageEdgeOffset()),
          _depth(samplingParams.minDepth, samplingParams.maxDepth),
          _time(samplingParams.minTime, samplingParams.maxTime),
          _gen(planner->GetRandomSeed()),
          _planner(planner) {
    }
    //! Returns a random candidate trajectory to be evaluated by the planner.
    int GetNextCandidateTrajectory(
      RapidTrajectoryGenerator& nextTraj) {
      CommonMath::Vec3 posf(0, 0, 0);
      double pixel_x = _pixelX(_gen);
      double pixel_y = _pixelY(_gen);
      double pixel_depth = _depth(_gen);
      _planner->DeprojectPixelToPoint(pixel_x, pixel_y, pixel_depth, posf);
      nextTraj.Reset();
      nextTraj.SetGoalPosition(posf);
      nextTraj.SetGoalVelocity(CommonMath::Vec3(0, 0, 0));
      nextTraj.SetGoalAcceleration(CommonMath::Vec3(0, 0, 0));
      nextTraj.Generate(_time(_gen));
      return 0;
    }

   private:
    //! [pixels]
    std::uniform_real_distribution<> _pixelX;
    //! [pixels]
    std::uniform_real_distribution<> _pixelY;
    //! [meters]
    std::uniform_real_distribution<> _depth;
    //! [seconds]
    std::uniform_real_distribution<> _time;
    std::random_device _rd;
    std::mt19937 _gen;
    //! Pointer to DepthImagePlanner where camera intrinsics are defined
    DepthImagePlanner* _planner;
  };

 private:

  //! Collision checking algorithm as described by Algorithm 1 in the RAPPIDS paper.
  //! The given trajectory will be checked for collisions, and new pyramids will be
  //! generated and added to the set of generated pyramids as needed.
  /*!
   * @param trajectory Candidate trajectory to be checked for collisions. We assume the
   * candidate trajectory is written in the camera-fixed frame and has an initial
   * position of (0, 0, 0).
   * @return True if the trajectory is found to be collision free, false otherwise
   */
  bool IsCollisionFree(CommonMath::Trajectory trajectory);

  //! Get the cost of a trajectory
  double GetTrajCost(const RapidTrajectoryGenerator &traj,
                      CommonMath::Vec3 currentGoalImage);

  //! Get the cost of a trajectory considering perception
  double GetPerceptionAwareTrajCost(
        const RapidTrajectoryGenerator &traj,
        CommonMath::Vec3 currentGoalImage,       
        const std::vector<Vec3>& featuresOdom,
        const Rotation& camAtt,
        const Rotation& estVehicleAtt,
        const Vec3& estVehiclePos,
        const double poseSampleInterval);

  //! Get the cost of trajectory considering perception (only consider position)
  double GetPerceptionAwareTrajCostPosOnly(
        const RapidTrajectoryGenerator &traj,
        CommonMath::Vec3 currentGoalImage,       
        const std::vector<Vec3>& featuresOdom,
        const Rotation& camAtt,
        const Rotation& estVehicleAtt,
        const Vec3& estVehiclePos,
        const double poseSampleInterval,
        const double exposureTime);
        
  //! Get the feature points in the current camera's frame
  void GetFeaturesCamFrame(std::vector<Vec3>& featuresCam, 
                           std::vector<std::vector<double>>& featuresPixelSpeed,
                           const RapidTrajectoryGenerator &traj, 
                           const double time, const Vec3& goalCam, 
                           const std::vector<Vec3>& featuresOdom, 
                           const Rotation& camAtt, const Rotation& estVehicleAtt,
                           const Vec3& estVehiclePos); 

  //! Splits the candidate trajectory into sections with monotonically changing depth
  //! using the methods described in Section II.B of the RAPPIDS paper.
  /*!
   * @param trajectory The candidate trajectory to be split into its sections with monotonically changing depth
   * @return A vector of sections of the trajectory with monotonically changing depth
   */
  std::vector<MonotonicTrajectory> GetMonotonicSections(
      CommonMath::Trajectory trajectory);

  //! Tries to find an existing pyramid that contains the given sample point
  /*!
   * @param pixelX The x-coordinate of the sample point in pixel coordinates
   * @param pixelY The y-coordinate of the sample point in pixel coordinates
   * @param depth The depth of the sample point in meters
   * @param outPyramid A pyramid that contains the sample point, if one is found
   * @return True a pyramid containing the sample point was found, false otherwise
   */
  bool FindContainingPyramid(double pixelX, double pixelY, double depth,
                             Pyramid &outPyramid);

  //! Computes the time in seconds at which the given trajectory collides with the given pyramid (if it does at all)
  //! using the methods described in Section II.C of the RAPPIDS paper.
  /*!
   * @param monoTraj A trajectory with monotonically changing depth
   * @param pyramid The pyramid to be used for collision checking
   * @param outCollisionTime The time in seconds at which the trajectory collides with a lateral face of the pyramid.
   * If the trajectory collides with lateral faces of the pyramid multiple times, this is the time at which
   * the collision with the deepest depth occurs.
   * @return True if the trajectory collides with at least one lateral face of the pyramid
   */
  bool FindDeepestCollisionTime(MonotonicTrajectory monoTraj, Pyramid pyramid,
                                double& outCollisionTime);

  //! Attempts to generate a pyramid that contains a given point.
  /*!
   * @param x0 The sample point's horizontal pixel coordinate
   * @param y0 The sample point's vertical pixel coordinate
   * @param minimumDepth The minimum depth of the base plane of the pyramid (i.e. the depth of the sample point)
   * @param outPyramid The created pyramid (can be undefined if no pyramid could be created)
   * @return True if a pyramid was successfully created, false otherwise
   */
  bool InflatePyramid(int x0, int y0, double minimumDepth, Pyramid &outPyramid);

  //! Class used to find trajectory that moves the fastest in the desired exploration direction
  class ExplorationCost {
   public:
    //! Constructor. Defines desired exploration direction as written in the camera-fixed frame. For example,
    //! a value of (0, 0, 1) would give trajectories that travel the fastest in the direction
    //! that the camera is pointing the lowest cost.
    ExplorationCost(CommonMath::Vec3 explorationDirection)
        : _explorationDirection(explorationDirection) {
    }
    //! Returns the cost of a given trajectory. Note that because each candidate trajectory is written in
    //! the camera-fixed frame, the initial position of each candidate trajectory is always (0, 0, 0) because
    //! we fix the trajectories to originate at the focal point of the camera. Thus, we only need to evaluate
    //! the end point of the trajectory.
    double GetCost(RapidTrajectoryGenerator& traj) {
      double duration = traj.GetFinalTime();
      return -_explorationDirection.Dot(traj.GetPosition(duration)) / duration;
    }
   private:
    CommonMath::Vec3 _explorationDirection;
  };

  //! Pointer to the array where the depth image is stored
  const uint16_t* _depthData;
  //! The conversion between pixel value and the associated depth value. For example: depthInMeters = depthScale * valueStoredInPixel
  double _depthScale;
  //! Focal length of camera used to take depth image
  double _focalLength;
  //! The location in pixel coordinates of the center of the image horizontally (usually half the width)
  double _cx;
  //! The location in pixel coordinates of the center of the image vertically (usually half the height)
  double _cy;
  //! Width of the image in pixels
  int _imageWidth;
  //! Height of the image in pixels
  int _imageHeight;
  //! Random seed
  int _randomSeed = 0;
  //! The true radius of the vehicle. Any depth values closer than this distance to the camera will be ignored. [meters]
  double _trueVehicleRadius;
  //! We plan as if the vehicle has this radius. This value should be slightly larger than _trueVehicleRadius to account
  //! for pose estimation errors and trajectory tracking errors. [meters]
  double _vehicleRadiusForPlanning;
  //!We do not perform collision checking on parts of the candidate trajectory closer than this distance to the camera.
  //! Obstacles closer than this distance can still occlude other parts of the trajectory (causing them to be labeled
  //! as in collision). This is used to enforce field of view constraints as well; we assume there is an obstacle
  //! just outside of the field of view this distance away from the camera. [meters]
  double _minCheckingDist;

  //! The minimum allowed thrust (used for checking dynamic feasibility) [m/s^2]
  double _minimumAllowedThrust;
  //! The maximum allowed thrust (used for checking dynamic feasibility) [m/s^2]
  double _maximumAllowedThrust;
  //! The maximum allowed angular velocity (used for checking dynamic feasibility) [rad/s]
  double _maximumAllowedAngularVelocity;
  //! The maximum allowed velocity (used for checking state feasibility) [m/s]
  double _maximumAllowedVelocity;
  //! The minimum allowed height 
  double _minimumAllowedHeight;
  //! Minimum time section to test when checking for dynamic feasibility [seconds]
  double _minimumSectionTimeDynamicFeas;

  //! After this amount of time has been spent generating pyramids, no more pyramids will be generated (i.e. instead of trying to
  //! generate a new pyramid, the trajectory will simply be labeled as in-collision). This limit is not set by default. [seconds]
  double _maxPyramidGenTime;
  //! The amount of time spent generating pyramids [nanoseconds]
  double _pyramidGenTimeNanoseconds;
  //! The maximum number of pyramids we allow the planner to generate. This limit is not set by default.
  int _maxNumPyramids;

  //! Time allocated for running the planner in seconds. The planner checks whether it should exit after evaluating each trajectory.
  double _allocatedComputationTime;
  //! The time at which the planner starts.
  std::chrono::high_resolution_clock::time_point _startTime;

  //! Counter used to keep track of how many trajectories we have generated/evaluated
  int _numTrajectoriesGenerated;
  //! Counter used to keep track of how many trajectories we have checked for collisions
  int _numCollisionChecks;
  //! Counter used to keep track of how many trajectories we have checked for velocity
  int _numVelocityChecks;
  //! Counter used to keep track of how many trajectories we have checked for cost
  int _numCostChecks;
  //! Counter used to keep track of how many trajectories we have found collision-free
  int _numCollisionFree;

  //! If the endpoint of a trajectory is within this many pixels of the lateral face of a pyramid, we will not use that pyramid
  //! for collision checking. This is to prevent the scenario where a trajectory intersects the lateral face of a pyramid, and
  //! then the same pyramid is repeatedly used for collision checking (entering an infinite loop).
  int _pyramidSearchPixelBuffer;

  //! The list of all pyramids found by the planner. This list is ordered based on the depth of the base plane of each pyramid.
  std::vector<Pyramid> _pyramids;

};
}  // namespace RectangularPyramidPlanner