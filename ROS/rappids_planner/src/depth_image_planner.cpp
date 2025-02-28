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

#include "../include/rappids/depth_image_planner.hpp"

using namespace std::chrono;
using namespace CommonMath;
using namespace RapidQuadrocopterTrajectoryGenerator;
using namespace RectangularPyramidPlanner;

double const ZEROTOL = 1e-6;

DepthImagePlanner::DepthImagePlanner(cv::Mat depthImage, double depthScale,
                                     double focalLength, double principalPointX,
                                     double principalPointY,
                                     double physicalVehicleRadius,
                                     double vehicleRadiusForPlanning,
                                     double minimumCollisionDistance,
                                     double minimumAllowedThrust,
                                     double maximumAllowedThrust,
                                     double maximumAllowedAngularVelocity,
                                     double maximumAllowedVelocity,
                                     double minimumAllowedHeight)
    : _depthScale(depthScale),
      _focalLength(focalLength),
      _cx(principalPointX),
      _cy(principalPointY),
      _imageWidth(depthImage.cols),
      _imageHeight(depthImage.rows),
      _trueVehicleRadius(physicalVehicleRadius),
      _vehicleRadiusForPlanning(vehicleRadiusForPlanning),
      _minCheckingDist(minimumCollisionDistance),
      _minimumAllowedThrust(minimumAllowedThrust),  // By default don't allow negative thrust
      _maximumAllowedThrust(maximumAllowedThrust),  // By default limit maximum thrust to about 3g (30 m/s^2)
      _maximumAllowedAngularVelocity(maximumAllowedAngularVelocity),  // By default limit maximum angular velocity to 20 rad/s
      _minimumAllowedHeight(minimumAllowedHeight),
      _maximumAllowedVelocity(maximumAllowedVelocity),  // By default limit maximum velocity to 1 m/s
      _minimumSectionTimeDynamicFeas(0.02),  // By default restrict the dynamic feasibility check to a minimum section duration of 20ms
      _maxPyramidGenTime(1000),  // Don't limit pyramid generation time by default [seconds].
      _pyramidGenTimeNanoseconds(0),
      _maxNumPyramids(INT_MAX),  // Don't limit the number of generated pyramids by default
      _allocatedComputationTime(0),  // To be set when the planner is called
      _numTrajectoriesGenerated(0),
      _numCostChecks(0),
      _numCollisionChecks(0),
      _numCollisionFree(0),
      _numVelocityChecks(0),
      _pyramidSearchPixelBuffer(2)  // A sample point must be more than 2 pixels away from the edge of a pyramid to use that pyramid for collision checking
{
  CV_Assert(depthImage.type() == CV_16UC1);
  _depthData = reinterpret_cast<const uint16_t*>(depthImage.data);
}

bool DepthImagePlanner::FindLowestCostTrajectory(
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
    const double exposureTime) {

  // Start timing the planner
  _startTime = high_resolution_clock::now();
  _allocatedComputationTime = allocatedComputationTime;

  bool feasibleTrajFound = false;
  double bestCost = 1e8;

  // Get the initial state of the vehicle so we can initialize all of the candidate trajectories
  Vec3 pos0 = trajectory.GetPosition(0);
  Vec3 vel0 = trajectory.GetVelocity(0);
  Vec3 acc0 = trajectory.GetAcceleration(0);
  Vec3 grav = trajectory.GetGravityVector();

  // We assume that the initial state is written in the camera-fixed frame
  // assert(pos0.x == 0 && pos0.y == 0 && pos0.z == 0);

  RandomTrajectoryGenerator randomTrajGenerator(samplingParams, this);

  std::vector<int> perception_cost_time;
  std::vector<int> collision_checking_time;
  RapidTrajectoryGenerator candidateTraj(pos0, vel0, acc0, grav);

  Rotation trajAtt = estVehicleAtt * camAtt;
  while (true) {
    if (duration_cast<microseconds>(high_resolution_clock::now() - _startTime)
        .count() > int(_allocatedComputationTime * 1e6)) {
      double sum;
      double mean;
      if (!perception_cost_time.empty()){
        sum = std::accumulate(perception_cost_time.begin(), perception_cost_time.end(), 0.0);
        mean = sum / perception_cost_time.size();
        std::cout << "Average cost evaluation time for a trajectory: " << mean << " nano seconds." << std::endl;
      }
      if(!collision_checking_time.empty()){
        sum = std::accumulate(collision_checking_time.begin(), collision_checking_time.end(), 0.0);
        mean = sum / collision_checking_time.size();
        std::cout << "Average collision checking time for a trajectory: " << mean << " nano seconds." << std::endl;
      }
      break;
    }

    // Get the next candidate trajectory to evaluate using the provided trajectory generator
    int returnVal =  randomTrajGenerator.GetNextCandidateTrajectory(candidateTraj);

    if (returnVal < 0) {
      // There are no more candidate trajectories to check. This case should only be reached if
      // the candidate trajectory generator is designed to only give a finite number of candidates
      // (e.g. when using a gridded approach instead of using random search)
      break;
    }
    _numTrajectoriesGenerated++;

    TrajectoryTestResult result = TrajectoryTestResult::None;

    const bool SKIP_FEASIBILITY_CHECK = false;
    const bool SKIP_VELOCITY_CHECK = false;
    const bool SKIP_COLLISION_CHECK = false;
    const bool SKIP_COST_CHECK = false;
    const bool KEEP_TRAJECTORIES = false;
    // First check whether the trajectory is dynamically feasible
    RapidTrajectoryGenerator::InputFeasibilityResult res = candidateTraj
        .CheckInputFeasibility(_minimumAllowedThrust, _maximumAllowedThrust,
                                _maximumAllowedAngularVelocity,
                                _minimumSectionTimeDynamicFeas);
    Vec3 final_pos = trajAtt * candidateTraj.GetPosition(candidateTraj.GetFinalTime()) + estVehiclePos;
    //added check for minimum allowed height
    if (SKIP_FEASIBILITY_CHECK || (res == RapidTrajectoryGenerator::InputFeasible && final_pos.z > _minimumAllowedHeight)) {
      if (res == RapidTrajectoryGenerator::InputFeasible) {
        result = result | TrajectoryTestResult::DynamicsFeasible;
        _numCollisionChecks++;
      }

      // Check whether the trajectory is velocity state feasible
      RapidTrajectoryGenerator::StateFeasibilityResult isVelocityAdmissible = candidateTraj.CheckVelocityFeasibility(_maximumAllowedVelocity);
      if (SKIP_VELOCITY_CHECK || isVelocityAdmissible == RapidTrajectoryGenerator::StateFeasibilityResult::StateFeasible) {
        
        if (isVelocityAdmissible == RapidTrajectoryGenerator::StateFeasibilityResult::StateFeasible) {
          result = result | TrajectoryTestResult::VelocityAdmissible;
          _numVelocityChecks++;
        }

        // Check whether the trajectory collides with obstacles
        auto begin = std::chrono::high_resolution_clock::now();
        bool isCollisionFree = IsCollisionFree(candidateTraj.GetTrajectory());
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
        // std::cout << "Collision checking time: " << elapsed.count() << " nano seconds." << std::endl;
        collision_checking_time.push_back(elapsed.count());

        if (SKIP_COLLISION_CHECK || isCollisionFree) {
          if (isCollisionFree) {
            result = result | TrajectoryTestResult::CollisionFree;
            _numCollisionFree++;
          }
          auto begin = std::chrono::high_resolution_clock::now();
          //Check whether the trajectory's cost is lower than current best
          // double cost = GetPerceptionAwareTrajCost(candidateTraj, currentGoalImage, featuresOdom,
          //                                         camAtt, estVehicleAtt, estVehiclePos, 1.0/imageFPS)
          //               + GetTrajCost(candidateTraj, currentGoalImage);
          double current_perception_cost = GetPerceptionAwareTrajCostPosOnly(candidateTraj, currentGoalImage, featuresOdom,
                                                  camAtt, estVehicleAtt, estVehiclePos, 1.0/imageFPS, exposureTime);
          auto end = std::chrono::high_resolution_clock::now();
          auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
          // std::cout << "Cost calculation time: " << elapsed.count() << " nano seconds." << std::endl;
          perception_cost_time.push_back(elapsed.count());

          double current_speed_cost = GetTrajCost(candidateTraj, currentGoalImage);
          double total_cost = current_speed_cost + k_perc * current_perception_cost;
          if (SKIP_COST_CHECK || total_cost < bestCost){
            result = result | TrajectoryTestResult::LowCost;
            _numCostChecks++;
            bestCost = total_cost;
            perception_cost = current_perception_cost;
            speed_cost = current_speed_cost;
            feasibleTrajFound = true;
            trajectory = RapidTrajectoryGenerator(candidateTraj);
          }
        }
      }
    }

    if (KEEP_TRAJECTORIES){
      trajectories.emplace_back(result, candidateTraj);
    }
  }

  if (feasibleTrajFound) {
    return true;
  } else {
    return false;
  }
}

bool DepthImagePlanner::FindLowestCostTrajectory(
    RapidTrajectoryGenerator& trajectory,
    std::vector<TrajectoryTest>& trajectories,
    double& speed_cost,
    const double& allocatedComputationTime,
    const SamplingParameters& samplingParams,
    const Vec3& currentGoalImage,
    const Rotation& trajAtt,
    const Vec3& trajInitPos) {

  // Start timing the planner
  _startTime = high_resolution_clock::now();
  _allocatedComputationTime = allocatedComputationTime;

  bool feasibleTrajFound = false;
  double bestCost = 1e9;

  // Get the initial state of the vehicle so we can initialize all of the candidate trajectories
  Vec3 pos0 = trajectory.GetPosition(0);
  Vec3 vel0 = trajectory.GetVelocity(0);
  Vec3 acc0 = trajectory.GetAcceleration(0);
  Vec3 grav = trajectory.GetGravityVector();

  // We assume that the initial state is written in the camera-fixed frame
  // assert(pos0.x == 0 && pos0.y == 0 && pos0.z == 0);

  RandomTrajectoryGenerator randomTrajGenerator(samplingParams, this);

  RapidTrajectoryGenerator candidateTraj(pos0, vel0, acc0, grav);
  while (true) {

    if (duration_cast<microseconds>(high_resolution_clock::now() - _startTime)
        .count() > int(_allocatedComputationTime * 1e6)) {
      break;
    }

    // Get the next candidate trajectory to evaluate using the provided trajectory generator
    int returnVal =  randomTrajGenerator.GetNextCandidateTrajectory(candidateTraj);

    if (returnVal < 0) {
      // There are no more candidate trajectories to check. This case should only be reached if
      // the candidate trajectory generator is designed to only give a finite number of candidates
      // (e.g. when using a gridded approach instead of using random search)
      break;
    }
    _numTrajectoriesGenerated++;

    // Compute the cost of the trajectory
    double current_speed_cost = GetTrajCost(candidateTraj, currentGoalImage);

    TrajectoryTestResult result = TrajectoryTestResult::None;

    const bool SKIP_COST_CHECK = false;
    const bool SKIP_FEASIBILITY_CHECK = false;
    const bool SKIP_VELOCITY_CHECK = false;
    const bool SKIP_COLLISION_CHECK = false;
    const bool KEEP_TRAJECTORIES = false;
    if (SKIP_COST_CHECK || current_speed_cost < bestCost) {
      // The trajectory is a lower cost than lowest cost trajectory found so far
      if (current_speed_cost < bestCost){
        result = result | TrajectoryTestResult::LowCost;
        _numCostChecks++;
      }

      // Check whether the trajectory is dynamically feasible
      RapidTrajectoryGenerator::InputFeasibilityResult res = candidateTraj
          .CheckInputFeasibility(_minimumAllowedThrust, _maximumAllowedThrust,
                                 _maximumAllowedAngularVelocity,
                                 _minimumSectionTimeDynamicFeas);
      Vec3 final_pos = trajAtt * candidateTraj.GetPosition(candidateTraj.GetFinalTime()) + trajInitPos;
      if (SKIP_FEASIBILITY_CHECK || (res == RapidTrajectoryGenerator::InputFeasible && final_pos.z > _minimumAllowedHeight)) {
        // The trajectory is dynamically feasible
        if (res == RapidTrajectoryGenerator::InputFeasible) {
          result = result | TrajectoryTestResult::DynamicsFeasible;
          _numCollisionChecks++;
        }

        RapidTrajectoryGenerator::StateFeasibilityResult isVelocityAdmissible = candidateTraj.CheckVelocityFeasibility(_maximumAllowedVelocity);
        if (SKIP_VELOCITY_CHECK || isVelocityAdmissible == RapidTrajectoryGenerator::StateFeasibilityResult::StateFeasible) {
          // The trajectory is velocity state feasible
          if (isVelocityAdmissible == RapidTrajectoryGenerator::StateFeasibilityResult::StateFeasible) {
            result = result | TrajectoryTestResult::VelocityAdmissible;
            _numVelocityChecks++;
          }

          // Check whether the trajectory collides with obstacles
          bool isCollisionFree = IsCollisionFree(candidateTraj.GetTrajectory());
          if (SKIP_COLLISION_CHECK || isCollisionFree) {
            // The trajectory is collision-free
            if (isCollisionFree) {
              result = result | TrajectoryTestResult::CollisionFree;
              feasibleTrajFound = true;
              bestCost = current_speed_cost;
              speed_cost = current_speed_cost;
              _numCollisionFree++;
              trajectory = RapidTrajectoryGenerator(candidateTraj);
            }
          }
        }
      }
    }

    if (KEEP_TRAJECTORIES){
      trajectories.emplace_back(result, candidateTraj);
    }
  }

  if (feasibleTrajFound) {
    return true;
  } else {
    return false;
  }
}

bool DepthImagePlanner::IsCollisionFree(Trajectory trajectory) {

  // Split trajectory into sections with monotonically changing depth
  std::vector<MonotonicTrajectory> monotonicSections = GetMonotonicSections(
      trajectory);
  while (monotonicSections.size() > 0) {

    // Check if we've used up all of our computation time
    if (duration_cast<microseconds>(high_resolution_clock::now() - _startTime)
        .count() > int(_allocatedComputationTime * 1e6)) {
      return false;
    }

    // Get a monotonic section to check
    MonotonicTrajectory monoTraj = monotonicSections.back();
    monotonicSections.pop_back();

    // Find the pixel corresponding to the endpoint of this section (deepest depth)
    Vec3 startPoint, endPoint;
    if (monoTraj.increasingDepth) {
      startPoint = monoTraj.GetValue(monoTraj.GetStartTime());
      endPoint = monoTraj.GetValue(monoTraj.GetEndTime());
    } else {
      startPoint = monoTraj.GetValue(monoTraj.GetEndTime());
      endPoint = monoTraj.GetValue(monoTraj.GetStartTime());
    }

    // Ignore the trajectory section if it's closer than the minimum collision checking distance
    if (startPoint.z < _minCheckingDist && endPoint.z < _minCheckingDist) {
      continue;
    }

    // Try to find pyramid that contains endPoint
    double endPointPixel[2];
    ProjectPointToPixel(endPoint, endPointPixel[0], endPointPixel[1]);
    Pyramid collisionCheckPyramid;
    bool pyramidFound = FindContainingPyramid(endPointPixel[0],
                                              endPointPixel[1], endPoint.z,
                                              collisionCheckPyramid);
    if (!pyramidFound) {
      // No pyramids containing endPoint were found, try to make a new pyramid
      if (_pyramids.size() >= _maxNumPyramids
          || _pyramidGenTimeNanoseconds > _maxPyramidGenTime * 1e9) {
        // We've already exceeded the maximum number of allowed pyramids or
        // the maximum time allocated for pyramid generation.
        return false;
      }

      high_resolution_clock::time_point startInflate =
          high_resolution_clock::now();
      bool pyramidGenerated = InflatePyramid(endPointPixel[0], endPointPixel[1],
                                             endPoint.z, collisionCheckPyramid);
      _pyramidGenTimeNanoseconds += duration_cast<nanoseconds>(
          high_resolution_clock::now() - startInflate).count();

      if (pyramidGenerated) {
        // Insert the new pyramid into the list of pyramids found so far
        auto index = std::lower_bound(_pyramids.begin(), _pyramids.end(),
                                      collisionCheckPyramid.depth);
        _pyramids.insert(index, collisionCheckPyramid);
      } else {
        // No pyramid could be formed, so there must be a collision
        return false;
      }
    }

    // Check if/when the trajectory intersects a lateral face of the given pyramid.
    double collisionTime;
    bool collidesWithPyramid = FindDeepestCollisionTime(monoTraj,
                                                        collisionCheckPyramid,
                                                        collisionTime);

    if (collidesWithPyramid) {
      // The trajectory collides with at least lateral face of the pyramid. Split the trajectory where it intersects,
      // and add the section outside the pyramid for further collision checking.
      if (monoTraj.increasingDepth) {
        monotonicSections.push_back(
            MonotonicTrajectory(monoTraj.GetCoeffs(), monoTraj.GetStartTime(),
                                collisionTime));
      } else {
        monotonicSections.push_back(
            MonotonicTrajectory(monoTraj.GetCoeffs(), collisionTime,
                                monoTraj.GetEndTime()));
      }
    }
  }
  return true;
}

double DepthImagePlanner::GetTrajCost(
    const RapidTrajectoryGenerator &traj, Vec3 currentGoalImage) {
    double duration = traj.GetFinalTime();

    // traj end point position in picture's frame
    Vec3 end_traj_pos_picture = traj.GetPosition(duration);

    double traj_start_to_goal_distance =
        currentGoalImage.GetNorm2();
    double traj_end_to_goal_distance =
        (currentGoalImage - end_traj_pos_picture).GetNorm2();

    return -(traj_start_to_goal_distance - traj_end_to_goal_distance) /
           duration;
}

double DepthImagePlanner::GetPerceptionAwareTrajCost(
    const RapidTrajectoryGenerator &traj, Vec3 currentGoalImage, 
    const std::vector<Vec3>& featuresOdom,
    const Rotation& camAtt, const Rotation& estVehicleAtt,
    const Vec3& estVehiclePos,
    const double poseSampleInterval) {
    double duration = traj.GetFinalTime();
    // traj end point position in picture's frame
    Vec3 end_traj_pos_picture = traj.GetPosition(duration);

    double perceptionCost = 0.0;
    int camPoseNum = 0;
    for (double time = 0; time < duration; time += poseSampleInterval){
      ++camPoseNum;
      //first convert the features from the odometry frame
      std::vector<Vec3> featuresCam;
      std::vector<std::vector<double>> featuresPixelSpeed;
      GetFeaturesCamFrame(featuresCam, featuresPixelSpeed, traj, time, currentGoalImage, 
                          featuresOdom, camAtt, estVehicleAtt, estVehiclePos);

      //derivative of re-projection error w.r.t. camera extrinsic left perturbation xi in se(3)
      std::vector<std::vector<double>> J; 
      std::vector<double> Var; //variance of the features
      for(Vec3 feature : featuresCam){
          double feature_z_square = feature.z*feature.z;
          std::vector<double> grad_row1{_focalLength/feature.z, 
                                        0, 
                                        -_focalLength*feature.x/feature_z_square, 
                                        -_focalLength*feature.x*feature.y/feature_z_square,
                                        _focalLength+_focalLength*(feature.x*feature.x)/feature_z_square,
                                        -_focalLength*feature.y/feature.z};
          
          std::vector<double> grad_row2{0,
                                        _focalLength/feature.z,
                                        -_focalLength*feature.y/feature_z_square,
                                        -_focalLength-_focalLength*(feature.y*feature.y)/feature_z_square,
                                        _focalLength*feature.x*feature.y/feature_z_square,
                                        _focalLength*feature.x/feature.z};
          J.push_back(grad_row1);
          J.push_back(grad_row2);
          Var.push_back(1.0);
          Var.push_back(1.0);
      }

      Eigen::Matrix<double, 6, 6> H; //information matrix (inverse of covariance)
      H.setZero();
      //Manually calculate H = J^T * inv(Var) * J
      // for (int i = 0; i < 6; ++i){
      //   for (int j = 0; j < 6; ++j){
      //     for (int k = 0; k < J.size(); ++k){
      //       H(i,j) += J[k][i] * 1.0/Var[k] * J[k][j];
      //     }
      //   }
      // }
      // TODO
      for (int k = 0; k < J.size(); ++k){
        // initialization
        for (int i = 0; i < 6; ++i){
          for (int j = 0; j < 6; ++j){
            H(i,j) += J[k][i] * J[k][j] / Var[k];
          }
        }
      }

      //perceptionCost += H.determinant();
      Eigen::Matrix<double, 6, 6> Sigma = H.inverse();
      perceptionCost += std::sqrt(Sigma(0,0)) + std::sqrt(Sigma(1,1)) + std::sqrt(Sigma(2,2));
    }
  //perceptionCost = std::pow(M_E, std::log(std::pow(perceptionCost, 1.0/(6*camPoseNum))));
  perceptionCost /= camPoseNum;
  return perceptionCost;
}

double DepthImagePlanner::GetPerceptionAwareTrajCostPosOnly(
    const RapidTrajectoryGenerator &traj, Vec3 currentGoalImage, 
    const std::vector<Vec3>& featuresOdom,
    const Rotation& camAtt, const Rotation& estVehicleAtt,
    const Vec3& estVehiclePos,
    const double poseSampleInterval,
    const double exposureTime) {
    double duration = traj.GetFinalTime();
    // traj end point position in picture's frame
    Vec3 end_traj_pos_picture = traj.GetPosition(duration);

    double perceptionCost = 0.0;
    for (double time = 0; time < duration; time += poseSampleInterval){
      //first convert the features from the odometry frame
      std::vector<Vec3> featuresCam;
      std::vector<std::vector<double>> featuresPixelSpeed;
      GetFeaturesCamFrame(featuresCam, featuresPixelSpeed, traj, time, currentGoalImage, 
                          featuresOdom, camAtt, estVehicleAtt, estVehiclePos);

      //derivative of re-projection error w.r.t. camera extrinsic left perturbation xi in se(3)
      Eigen::Matrix3d H = Eigen::Matrix3d::Zero(); //information matrix (inverse of covariance)

      // for(Vec3 feature : featuresCam){
      //   Eigen::Matrix2d varInv = Eigen::Matrix2d::Identity(); //variance of the features
      //   Eigen::Matrix<double, 2, 3> J; 
      //   J << 1, 0, -feature.x/feature.z,
      //        0, 1, -feature.y/feature.z;
      //   double temp = _focalLength / feature.z * _focalLength / feature.z;
      //   H += J.transpose() * varInv * J * temp;
      // }

      std::vector<double> featurePixelSpeed;
      double featureSpeedNorm, uSpeed, vSpeed, sigma1Square, sigma2Square;
      double tempa, tempb, tempd, invDetVar;
      std::vector<double> invVar(4, 0); 
      double invFeaturez, temp1, temp2, temp3;
      double invHdet;
      //order:
      //[invVar[0], invVar[1],
      // invVar[2], invVar[3]]
      //When feature number <=2 triangulation will fail. 
      if (featuresCam.size() > 2){
        for(int i = 0; i < featuresCam.size(); ++i){
          //calculate feature's variance
          featurePixelSpeed = featuresPixelSpeed[i];
          featureSpeedNorm = std::sqrt(featurePixelSpeed[0]*featurePixelSpeed[0] 
                                       + featurePixelSpeed[1]*featurePixelSpeed[1]);
          if (std::abs(featureSpeedNorm) < ZEROTOL){
            uSpeed = 0.0;
            vSpeed = 0.0;
          } else{                    
            uSpeed = featurePixelSpeed[0]/featureSpeedNorm; //normalized u speed
            vSpeed = featurePixelSpeed[1]/featureSpeedNorm; //normalized v speed
          }
          sigma1Square = featureSpeedNorm * exposureTime * featureSpeedNorm * exposureTime / 12.0;
          sigma2Square = 9.0;
          //Variance of a feature:
          //[tempa, tempb
          // tempb, tempd] 
          tempa = (sigma1Square+sigma2Square)*uSpeed*uSpeed + sigma2Square*vSpeed*vSpeed;
          tempb = sigma1Square * uSpeed * vSpeed;
          tempd = sigma2Square*uSpeed*uSpeed + (sigma1Square+sigma2Square)*vSpeed*vSpeed;
          //Calculate the inverse of the feature variance manually
          invDetVar = 1.0 / (tempa * tempd - tempb * tempb);        
          invVar[0] = invDetVar * tempd;
          invVar[1] = -invDetVar * tempb;
          invVar[2] = invVar[1];
          invVar[3] = invDetVar * tempa;

          //calculate the information matrix
          Vec3 feature = featuresCam[i];
          //std::vector<double> J1{1, 0, -feature.x/feature.z};
          //std::vector<double> J2{0, 1, -feature.y/feature.z};
          if (feature.z < ZEROTOL){
            invFeaturez = 0.0;
          } else{
            invFeaturez = 1.0 / feature.z;
          }
          temp1 = -feature.x*invFeaturez;
          temp2 = -feature.y*invFeaturez;
          temp3 = _focalLength * invFeaturez * _focalLength * invFeaturez;

          H(0,0) += invVar[0] * temp3;
          H(0,1) += invVar[1] * temp3;
          H(0,2) += (temp1*invVar[0]+temp2*invVar[1])*temp3;
          H(1,1) += invVar[3] * temp3;
          H(1,2) += (temp1*invVar[2]+temp2*invVar[3])*temp3;
          //H(2,2) = temp1*temp1*invVar[0]+temp1*temp2*(invVar[1]+invVar[2])+temp2*temp2*invVar[3];
          H(2,2) += H(0,2) * temp1 + H(1,2) * temp2;
        }
        H(1,0) = H(0,1);
        H(2,0) = H(0,2);
        H(2,1) = H(1,2);
      } else{
        //We'll terminate the evaluation of this trajectory and return a very large cost.
        //Since triangulation fails.
        std::cout<< "Triangulation fails" << std::endl;
        return 1e9;
      }
      // //print H for debugging
      // std::cout << std::endl;
      // for (int i = 0; i < 3; ++i){
      //   std::cout << "[" ;
      //   for (int j = 0; j < 3; ++j){
      //     std::cout << H(i,j) << " ";
      //   }
      //   std::cout << "]" << std::endl;
      // }
      //perceptionCost += H.determinant();
      
      //perceptionCost: sum of sqrt(H.Inv()[0,0]) + sqrt(H.Inv()[1,1]) + sqrt(H.Inv()[2,2]) 
      // std::cout << "H determinant: " << H.determinant() << std::endl;
      if (H.determinant() < ZEROTOL) continue;
      invHdet = 1.0 / H.determinant();
      perceptionCost += std::sqrt(std::max(invHdet * (H(2,2)*H(1,1) - H(2,1)*H(1,2)), 0.0))
                      + std::sqrt(std::max(invHdet * (H(2,2)*H(0,0) - H(2,0)*H(0,2)), 0.0))
                      + std::sqrt(std::max(invHdet * (H(1,1)*H(0,0) - H(1,0)*H(0,1)), 0.0));
    } 
  //perceptionCost = std::pow(M_E, std::log(std::pow(perceptionCost, 1.0/(6*camPoseNum))));
  perceptionCost /= floor(duration / poseSampleInterval);
  return perceptionCost;
}

void DepthImagePlanner::GetFeaturesCamFrame(std::vector<Vec3>& featuresCam, 
                                            std::vector<std::vector<double>>& featuresPixelSpeed,
                                            const RapidTrajectoryGenerator &traj, 
                                            const double time, const Vec3& goalCam,
                                            const std::vector<Vec3>& featuresOdom,
                                            const Rotation& camAtt, const Rotation& estVehicleAtt,
                                            const Vec3& estVehiclePos){

  //predict the position of the vehicle in the odometry's frame
  Rotation cameraToOdom = estVehicleAtt * camAtt;
  Vec3 predPos = cameraToOdom * traj.GetPosition(time) + estVehiclePos;
 
  //predict the attitude of the vehicle, assuming perfect trajectory tracking
  //such that z axis of the vehicle matches the proper acceleration's direction
  Vec3 trajAcc = cameraToOdom * traj.GetAcceleration(time);
  Vec3 properAcc = trajAcc + Vec3(0.0, 0.0, 9.81);
  double const normProperAcc = properAcc.GetNorm2();
  Vec3 const cmdThrustDir = properAcc / normProperAcc;

  Rotation predVehicleAtt;
  Vec3 e3(0.0, 0.0, 1.0);
  const double cosAngle = cmdThrustDir.Dot(e3);
  double angle;
  if (cosAngle >= (1 - 1e-12f)) {
    angle = 0;
  } else if (cosAngle <= -(1 - 1e-12f)) {
    angle = M_PI;
  } else {
    angle = acos(cosAngle);  //magnitude of desired rotation
  }
  Vec3 rotAx = e3.Cross(cmdThrustDir);
  const double n = rotAx.GetNorm2();
  if (n < 1e-6f) {
    predVehicleAtt = Rotation::Identity();
  } else {
    predVehicleAtt = Rotation::FromRotationVector(rotAx * (angle / n));
  }
  //yaw angle
  Vec3 goalOdom = cameraToOdom * goalCam + estVehiclePos;
  Vec3 predDirectionOdom = goalOdom - predPos;    
  double desiredYawAngle = atan2(predDirectionOdom.y, predDirectionOdom.x);  
  predVehicleAtt = predVehicleAtt * Rotation::FromRotationVector(Vec3(0, 0, desiredYawAngle));
  
  //calculate the rotation rate
  double Rob[9]; //transformation matrix from the body to world frame
  predVehicleAtt.GetRotationMatrix(Rob);
  //unit vector for the x axis of the body frame (expressed in odom frame)
  Vec3 xb(Rob[0], Rob[3], Rob[6]); 
  Vec3 yb(Rob[1], Rob[4], Rob[7]);
  Vec3 zb(Rob[2], Rob[5], Rob[8]);
  Vec3 jerk = cameraToOdom * traj.GetJerk(time);
  Vec3 hw = 1.0 / normProperAcc * (jerk - zb.Dot(jerk) * zb);
  double p, q, r; //body rate represented in the body frame
  p = - hw.Dot(yb);
  q = hw.Dot(xb);
  Vec3 predDirectionOdomRate = -(cameraToOdom * traj.GetVelocity(time));
  double denumTemp = pow(predDirectionOdom.x,2) + pow(predDirectionOdom.y, 2); 
  if (denumTemp < 1e-12f) {
    r = 0.0;
  }else{
    r = (-predDirectionOdom.y / denumTemp * predDirectionOdomRate.x
         + predDirectionOdom.x / denumTemp * predDirectionOdomRate.y) * e3.Dot(zb);
  }

  //predict feature points that will be in camera FOV
  Rotation tempRot = camAtt.Inverse() * predVehicleAtt.Inverse();
  featuresCam.clear();
  for (const Vec3& featureOdom : featuresOdom){
    Vec3 featureCam = tempRot * (featureOdom - predPos);
    if (featureCam.z > ZEROTOL){ //feature cannot be in the back of the camera
      std::vector<double> featurePixelSpeed{0.0, 0,0};
      double u = _focalLength * featureCam.x / featureCam.z + _cx;
      if (u > 0 && u < _imageWidth){
        double v = _focalLength * featureCam.y / featureCam.z + _cy;
        if (v > 0 && v < _imageHeight){
          featuresCam.push_back(featureCam);
          featurePixelSpeed[0] = _focalLength / pow(featureCam.z, 2) * (p * featureCam.z - featureCam.x * r);
          featurePixelSpeed[1] = _focalLength / pow(featureCam.z, 2) * (q * featureCam.z - featureCam.y * r);
          featuresPixelSpeed.push_back(featurePixelSpeed);
        }
      }
    }
  }
}

std::vector<MonotonicTrajectory> DepthImagePlanner::GetMonotonicSections(
    Trajectory trajectory) {
  // This function exploits the property described in Section II.B of the RAPPIDS paper

  // Compute the coefficients of \dot{d}_z(t)
  std::vector<Vec3> trajDerivativeCoeffs = trajectory.GetDerivativeCoeffs();
  double c[5] = { trajDerivativeCoeffs[0].z, trajDerivativeCoeffs[1].z,
      trajDerivativeCoeffs[2].z, trajDerivativeCoeffs[3].z,
      trajDerivativeCoeffs[4].z };  // Just shortening the names

  // Compute the times at which the trajectory changes direction along the z-axis
  double roots[6];
  roots[0] = trajectory.GetStartTime();
  roots[1] = trajectory.GetEndTime();
  size_t rootCount;
  if (fabs(c[0]) > 1e-6) {
    rootCount = Quartic::solve_quartic(c[1] / c[0], c[2] / c[0], c[3] / c[0],
                                       c[4] / c[0], roots + 2);
  } else {
    rootCount = Quartic::solveP3(c[2] / c[1], c[3] / c[1], c[4] / c[1],
                                 roots + 2);
  }
  std::sort(roots, roots + rootCount + 2);  // Use rootCount + 2 because we include the start and end point

  std::vector<MonotonicTrajectory> monotonicSections;
// We don't iterate until rootCount + 2 because we need to find pairs of roots
  for (unsigned i = 0; i < rootCount + 1; i++) {
    if (roots[i] < trajectory.GetStartTime()) {
      // Skip root if it's before start time
      continue;
    } else if (fabs(roots[i] - roots[i + 1]) < 1e-6) {
      // Skip root because it's a duplicate
      continue;
    } else if (roots[i] >= trajectory.GetEndTime()) {
      // We're done because the roots are in ascending order
      break;
    }
    // Add a section between the current root and the next root after checking that the next root is valid
    // We already know that roots[i+1] is greater than the start time because roots[i] is greater than the start time and roots is sorted
    if (roots[i + 1] <= trajectory.GetEndTime()) {
      monotonicSections.push_back(
          MonotonicTrajectory(trajectory.GetCoeffs(), roots[i], roots[i + 1]));
    } else {
      // We're done because the next section is out of the range
      break;
    }
  }
  std::sort(monotonicSections.begin(), monotonicSections.end());
  return monotonicSections;
}

bool DepthImagePlanner::FindContainingPyramid(double pixelX, double pixelY,
                                              double depth,
                                              Pyramid &outPyramid) {
  // This function searches _pyramids for those with base planes at deeper
  // depths than endPoint.z
  auto firstPyramidIndex = std::lower_bound(_pyramids.begin(), _pyramids.end(),
                                            depth);
  if (firstPyramidIndex != _pyramids.end()) {
    // At least one pyramid exists that has a base plane deeper than endPoint.z
    for (std::vector<Pyramid>::iterator it = firstPyramidIndex;
        it != _pyramids.end(); ++it) {
      // Check whether endPoint is inside the pyramid
      // We need to use the _pyramidSearchPixelBuffer offset here because otherwise we'll try to
      // collision check with the pyramid we just exited while checking the previous section
      if ((*it).leftPixBound + _pyramidSearchPixelBuffer < pixelX
          && pixelX < (*it).rightPixBound - _pyramidSearchPixelBuffer
          && (*it).topPixBound + _pyramidSearchPixelBuffer < pixelY
          && pixelY < (*it).bottomPixBound - _pyramidSearchPixelBuffer) {
        outPyramid = *it;
        return true;
      }
    }
  }
  return false;
}

bool DepthImagePlanner::FindDeepestCollisionTime(MonotonicTrajectory monoTraj,
                                                 Pyramid pyramid,
                                                 double& outCollisionTime) {
  // This function exploits the property described in Section II.C of the RAPPIDS paper

  bool collidesWithPyramid = false;
  if (monoTraj.increasingDepth) {
    outCollisionTime = monoTraj.GetStartTime();
  } else {
    outCollisionTime = monoTraj.GetEndTime();
  }
  std::vector<Vec3> coeffs = monoTraj.GetCoeffs();
  for (Vec3 normal : pyramid.planeNormals) {

    // Compute the coefficients of d(t) (distance to the lateral face of the pyramid)
    double c[5] = { 0, 0, 0, 0, 0 };
    for (int dim = 0; dim < 3; dim++) {
      c[0] += normal[dim] * coeffs[0][dim];  //t**5
      c[1] += normal[dim] * coeffs[1][dim];  //t**4
      c[2] += normal[dim] * coeffs[2][dim];  //t**3
      c[3] += normal[dim] * coeffs[3][dim];  //t**2
      c[4] += normal[dim] * coeffs[4][dim];  //t
      // coeffs[5] = (0,0,0) because trajectory is at (0,0,0) at t = 0
    }

    // Find the times at which the trajectory intersects the plane
    double roots[4];
    size_t rootCount;
    if (fabs(c[0]) > 1e-6) {
      rootCount = Quartic::solve_quartic(c[1] / c[0], c[2] / c[0], c[3] / c[0],
                                         c[4] / c[0], roots);
    } else {
      rootCount = Quartic::solveP3(c[2] / c[1], c[3] / c[1], c[4] / c[1],
                                   roots);
    }
    std::sort(roots, roots + rootCount);
    if (monoTraj.increasingDepth) {
      // Search backward in time (decreasing depth)
      for (int i = rootCount - 1; i >= 0; i--) {
        if (roots[i] > monoTraj.GetEndTime()) {
          continue;
        } else if (roots[i] > monoTraj.GetStartTime()) {
          if (roots[i] > outCollisionTime) {
            // This may seem unnecessary because we are searching an ordered list, but this check is needed
            // because we are checking multiple lateral faces of the pyramid for collisions
            outCollisionTime = roots[i];
            collidesWithPyramid = true;
            break;
          }
        } else {
          break;
        }
      }
    } else {
      // Search forward in time (decreasing depth)
      for (int i = 0; i < int(rootCount); i++) {
        if (roots[i] < monoTraj.GetStartTime()) {
          continue;
        } else if (roots[i] < monoTraj.GetEndTime()) {
          if (roots[i] < outCollisionTime) {
            outCollisionTime = roots[i];
            collidesWithPyramid = true;
            break;
          }
        } else {
          break;
        }
      }
    }
  }
  return collidesWithPyramid;
}

bool DepthImagePlanner::InflatePyramid(int x0, int y0, double minimumDepth,
                                       Pyramid &outPyramid) {
  // This function is briefly described by Section III.A. of the RAPPIDS paper

  // First check if the sample point violates the field of view constraints
  int imageEdgeOffset = GetImageEdgeOffset();
  if (x0 <= imageEdgeOffset + _pyramidSearchPixelBuffer + 1
      || x0 > _imageWidth - imageEdgeOffset - _pyramidSearchPixelBuffer - 1
      || y0 <= imageEdgeOffset + _pyramidSearchPixelBuffer + 1
      || y0 > _imageHeight - imageEdgeOffset - _pyramidSearchPixelBuffer - 1) {
    // Sample point could be in collision with something outside the FOV
    return false;
  }

  // The base plane of the pyramid must be deeper than this depth (written in pixel depth units)
  uint16_t minimumPyramidDepth = uint16_t(
      (minimumDepth + _vehicleRadiusForPlanning) / _depthScale);

  // This is the minimum "radius" (really the width/height divided by two) of a valid pyramid
  int initPixSearchRadius = _focalLength * _vehicleRadiusForPlanning
      / (_depthScale * minimumPyramidDepth);

  if (2 * initPixSearchRadius
      >= std::min(_imageWidth, _imageHeight) - 2 * imageEdgeOffset) {
    // The minimum size of the pyramid is larger than the maximum pyramid size
    return false;
  }

  // These edges are the edges of the expanded pyramid before it is shrunk to the final size
  int leftEdge, topEdge, rightEdge, bottomEdge;
  if (y0 - initPixSearchRadius < imageEdgeOffset) {
    topEdge = imageEdgeOffset;
    bottomEdge = topEdge + 2 * initPixSearchRadius;
  } else {
    bottomEdge = std::min(_imageHeight - imageEdgeOffset - 1,
                          y0 + initPixSearchRadius);
    topEdge = bottomEdge - 2 * initPixSearchRadius;
  }
  if (x0 - initPixSearchRadius < imageEdgeOffset) {
    leftEdge = imageEdgeOffset;
    rightEdge = leftEdge + 2 * initPixSearchRadius;
  } else {
    rightEdge = std::min(_imageWidth - imageEdgeOffset - 1,
                         x0 + initPixSearchRadius);
    leftEdge = rightEdge - 2 * initPixSearchRadius;
  }

  // We don't look at any pixels closer than this distance (e.g. if the propellers are in the field of view)
  uint16_t ignoreDist = uint16_t(_trueVehicleRadius / _depthScale);
  // For reading the depth value stored in a given pixel.
  uint16_t pixDist;

  for (int y = topEdge; y < bottomEdge; y++) {
    for (int x = leftEdge; x < rightEdge; x++) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist <= minimumPyramidDepth && pixDist > ignoreDist) {
        // We are unable to inflate a rectangle that will meet the minimum size requirements
        return false;
      }
    }
  }

  // Store the minimum depth pixel value of the expanded pyramid. The base plane of the final pyramid will be
  // this value minus the vehicle radius.
  uint16_t maxDepthExpandedPyramid = std::numeric_limits<uint16_t>::max();

  // We search each edge of the rectangle until we hit a pixel value closer than minimumPyramidDepth
  // This creates a spiral search pattern around the initial sample point
  // Once all four sides of the pyramid hit either the FOV constraint or a pixel closer than
  // minimumPyramidDepth, we will shrink the pyramid based on the vehicle radius.
  bool rightFree = true, topFree = true, leftFree = true, bottomFree = true;
  while (rightFree || topFree || leftFree || bottomFree) {
    if (rightFree) {
      if (rightEdge < _imageWidth - imageEdgeOffset - 1) {
        for (int y = topEdge; y <= bottomEdge; y++) {
          pixDist = _depthData[y * _imageWidth + rightEdge + 1];
          if (pixDist > ignoreDist) {
            if (pixDist < minimumPyramidDepth) {
              rightFree = false;
              rightEdge--;  // Negate the ++ after breaking loop
              break;
            }
            maxDepthExpandedPyramid = std::min(maxDepthExpandedPyramid,
                                               pixDist);
          }
        }
        rightEdge++;
      } else {
        rightFree = false;
      }
    }
    if (topFree) {
      if (topEdge > imageEdgeOffset) {
        for (int x = leftEdge; x <= rightEdge; x++) {
          pixDist = _depthData[(topEdge - 1) * _imageWidth + x];
          if (pixDist > ignoreDist) {
            if (pixDist < minimumPyramidDepth) {
              topFree = false;
              topEdge++;  // Negate the -- after breaking loop
              break;
            }
            maxDepthExpandedPyramid = std::min(maxDepthExpandedPyramid,
                                               pixDist);
          }
        }
        topEdge--;
      } else {
        topFree = false;
      }
    }
    if (leftFree) {
      if (leftEdge > imageEdgeOffset) {
        for (int y = topEdge; y <= bottomEdge; y++) {
          pixDist = _depthData[y * _imageWidth + leftEdge - 1];
          if (pixDist > ignoreDist) {
            if (pixDist < minimumPyramidDepth) {
              leftFree = false;
              leftEdge++;  // Negate the -- after breaking loop
              break;
            }
            maxDepthExpandedPyramid = std::min(maxDepthExpandedPyramid,
                                               pixDist);
          }
        }
        leftEdge--;
      } else {
        leftFree = false;
      }
    }
    if (bottomFree) {
      if (bottomEdge < _imageHeight - imageEdgeOffset - 1) {
        for (int x = leftEdge; x <= rightEdge; x++) {
          pixDist = _depthData[(bottomEdge + 1) * _imageWidth + x];
          if (pixDist > ignoreDist) {
            if (pixDist < minimumPyramidDepth) {
              bottomFree = false;
              bottomEdge--;  // Negate the ++ after breaking loop
              break;
            }
            maxDepthExpandedPyramid = std::min(maxDepthExpandedPyramid,
                                               pixDist);
          }
        }
        bottomEdge++;
      } else {
        bottomFree = false;
      }
    }
  }

  // Next, shrink the pyramid according to the vehicle radius
  // Number of pixels to shrink final pyramid. Found by searching outside the boundaries of the expanded pyramid.
  // These edges will be the edges of the final pyramid.
  int rightEdgeShrunk = _imageWidth - 1 - imageEdgeOffset;
  int leftEdgeShrunk = imageEdgeOffset;
  int topEdgeShrunk = imageEdgeOffset;
  int bottomEdgeShrunk = _imageHeight - 1 - imageEdgeOffset;
  int numerator = _focalLength * _vehicleRadiusForPlanning / _depthScale;

  // First check the area between each edge and the edge of the image
  // Check right side
  for (int x = rightEdge; x < _imageWidth; x++) {
    for (int y = topEdge; y <= bottomEdge; y++) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist > ignoreDist && pixDist < maxDepthExpandedPyramid) {
        // The pixel is farther away than the minimum checking distance
        if (numerator > (x - rightEdgeShrunk) * pixDist) {
          int rightShrinkTemp = x - int(numerator / pixDist);
          if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer) {
            // Shrinking from right will make pyramid invalid
            // Can we shrink from top or bottom instead?
            int topShrinkTemp = y + int(numerator / pixDist);
            int bottomShrinkTemp = y - int(numerator / pixDist);
            if (y0 < topShrinkTemp + _pyramidSearchPixelBuffer
                && y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
              // We can't shrink either edge
              return false;
            } else if (y0 < topShrinkTemp + _pyramidSearchPixelBuffer) {
              // We can't shrink the upper edge, so shrink the lower edge
              bottomEdgeShrunk = bottomShrinkTemp;
            } else if (y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
              // We can't shrink the lower edge, so shrink the upper edge
              topEdgeShrunk = topShrinkTemp;
            } else {
              // We can shrink either edge and still have a feasible pyramid, choose the edge that removes the least area
              int uShrinkLostArea = (topShrinkTemp - topEdgeShrunk);
              int dShrinkLostArea = (bottomEdgeShrunk - bottomShrinkTemp);
              if (dShrinkLostArea > uShrinkLostArea) {
                // We lose more area shrinking the bottom side, so shrink the top side
                topEdgeShrunk = topShrinkTemp;
              } else {
                // We lose more area shrinking the top side, so shrink the bottom side
                rightEdgeShrunk = bottomShrinkTemp;
              }
            }
          } else {
            rightEdgeShrunk = rightShrinkTemp;
          }
        }
      }
    }
  }
  // Check left side
  for (int x = leftEdge; x >= 0; x--) {
    for (int y = topEdge; y <= bottomEdge; y++) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist > ignoreDist && pixDist < maxDepthExpandedPyramid) {
        if ((leftEdgeShrunk - x) * pixDist < numerator) {
          int leftShrinkTemp = x + int(numerator / pixDist);
          if (x0 < leftShrinkTemp + _pyramidSearchPixelBuffer) {
            // Shrinking from left will make pyramid invalid
            // Can we shrink from top or bottom instead?
            int topShrinkTemp = y + int(numerator / pixDist);
            int bottomShrinkTemp = y - int(numerator / pixDist);
            if (y0 < topShrinkTemp + _pyramidSearchPixelBuffer
                && y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
              // We can't shrink either edge
              return false;
            } else if (y0 < topShrinkTemp + _pyramidSearchPixelBuffer) {
              // We can't shrink the upper edge, so shrink the lower edge
              bottomEdgeShrunk = bottomShrinkTemp;
            } else if (y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
              // We can't shrink the lower edge, so shrink the upper edge
              topEdgeShrunk = topShrinkTemp;
            } else {
              // We can shrink either edge and still have a feasible pyramid, choose the edge that removes the least area
              int uShrinkLostArea = (topShrinkTemp - topEdgeShrunk);
              int dShrinkLostArea = (bottomEdgeShrunk - bottomShrinkTemp);
              if (dShrinkLostArea > uShrinkLostArea) {
                // We lose more area shrinking the bottom side, so shrink the top side
                topEdgeShrunk = topShrinkTemp;
              } else {
                // We lose more area shrinking the top side, so shrink the bottom side
                bottomEdgeShrunk = bottomShrinkTemp;
              }
            }
          } else {
            leftEdgeShrunk = leftShrinkTemp;
          }
        }
      }
    }
  }
  if (leftEdgeShrunk + _pyramidSearchPixelBuffer
      > rightEdgeShrunk - _pyramidSearchPixelBuffer) {
    // We shrunk the left and right sides so much that the pyramid is too small!
    return false;
  }

  // Check top side
  for (int y = topEdge; y >= 0; y--) {
    for (int x = leftEdge; x <= rightEdge; x++) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist > ignoreDist && pixDist < maxDepthExpandedPyramid) {
        if ((topEdgeShrunk - y) * pixDist < numerator) {
          int topShrinkTemp = y + int(numerator / pixDist);
          if (y0 < topShrinkTemp + _pyramidSearchPixelBuffer) {
            // Shrinking from top will make pyramid invalid
            // Can we shrink from left or right instead?
            int rightShrinkTemp = x - int(numerator / pixDist);
            int leftShrinkTemp = x + int(numerator / pixDist);
            if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer
                && x0 < leftShrinkTemp + _pyramidSearchPixelBuffer) {
              // We can't shrink either edge
              return false;
            } else if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer) {
              // We can't shrink the upper right, so shrink the left edge
              leftEdgeShrunk = leftShrinkTemp;
            } else if (x0 < leftShrinkTemp + _pyramidSearchPixelBuffer) {
              // We can't shrink the left edge, so shrink the right edge
              rightEdgeShrunk = rightShrinkTemp;
            } else {
              // We can shrink either edge and still have a feasible pyramid, choose the edge that removes the least area
              int rShrinkLostArea = (rightEdgeShrunk - rightShrinkTemp);
              int lShrinkLostArea = (leftShrinkTemp - leftEdgeShrunk);
              if (rShrinkLostArea > lShrinkLostArea) {
                // We lose more area shrinking the right side, so shrink the left side
                leftEdgeShrunk = leftShrinkTemp;
              } else {
                // We lose more area shrinking the left side, so shrink the right side
                rightEdgeShrunk = rightShrinkTemp;
              }
            }
          } else {
            topEdgeShrunk = topShrinkTemp;
          }
        }
      }
    }
  }
  // Check bottom side
  for (int y = bottomEdge; y < _imageHeight; y++) {
    for (int x = leftEdge; x <= rightEdge; x++) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist > ignoreDist && pixDist < maxDepthExpandedPyramid) {
        // The pixel is farther away than the minimum checking distance
        if (numerator > (y - bottomEdgeShrunk) * pixDist) {
          int bottomShrinkTemp = y - int(numerator / pixDist);
          if (y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
            // Shrinking from top will make pyramid invalid
            // Can we shrink from left or right instead?
            int rightShrinkTemp = x - int(numerator / pixDist);
            int leftShrinkTemp = x + int(numerator / pixDist);
            if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer
                && x0 < leftShrinkTemp + _pyramidSearchPixelBuffer) {
              // We can't shrink either edge
              return false;
            } else if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer) {
              // We can't shrink the upper right, so shrink the left edge
              leftEdgeShrunk = leftShrinkTemp;
            } else if (x0 < leftShrinkTemp + _pyramidSearchPixelBuffer) {
              // We can't shrink the left edge, so shrink the right edge
              rightEdgeShrunk = rightShrinkTemp;
            } else {
              // We can shrink either edge and still have a feasible pyramid, choose the edge that removes the least area
              int rShrinkLostArea = (rightEdgeShrunk - rightShrinkTemp);
              int lShrinkLostArea = (leftShrinkTemp - leftEdgeShrunk);
              if (rShrinkLostArea > lShrinkLostArea) {
                // We lose more area shrinking the right side, so shrink the left side
                leftEdgeShrunk = leftShrinkTemp;
              } else {
                // We lose more area shrinking the left side, so shrink the right side
                rightEdgeShrunk = rightShrinkTemp;
              }
            }
          } else {
            bottomEdgeShrunk = bottomShrinkTemp;
          }
        }
      }
    }
  }
  if (topEdgeShrunk + _pyramidSearchPixelBuffer
      > bottomEdgeShrunk - _pyramidSearchPixelBuffer) {
    // We shrunk the top and bottom sides so much that the pyramid has no volume!
    return false;
  }

  // Next, check the corners that we ignored before
  // Check top right corner
  for (int y = topEdge; y >= 0; y--) {
    for (int x = rightEdge; x < _imageWidth; x++) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist > ignoreDist && pixDist < maxDepthExpandedPyramid) {
        if (numerator > (x - rightEdgeShrunk) * pixDist
            && (topEdgeShrunk - y) * pixDist < numerator) {
          // Both right and top edges could shrink
          int rightShrinkTemp = x - int(numerator / pixDist);
          int topShrinkTemp = y + int(numerator / pixDist);
          if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer
              && y0 < topShrinkTemp + _pyramidSearchPixelBuffer) {
            // Shrinking either edge makes the pyramid exclude the starting point
            return false;
          } else if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer) {
            // Shrinking right edge makes pyramid exclude the starting point, so shrink the top edge
            topEdgeShrunk = topShrinkTemp;
          } else if (y0 < topShrinkTemp + _pyramidSearchPixelBuffer) {
            // Shrinking top edge makes pyramid exclude the starting point, so shrink the right edge
            rightEdgeShrunk = rightShrinkTemp;
          } else {
            // We can shrink either edge and still have a feasible pyramid, choose the edge that removes the least area
            int rShrinkLostArea = (rightEdgeShrunk - rightShrinkTemp)
                * (bottomEdgeShrunk - topEdgeShrunk);
            int uShrinkLostArea = (topShrinkTemp - topEdgeShrunk)
                * (rightEdgeShrunk - leftEdgeShrunk);
            if (rShrinkLostArea > uShrinkLostArea) {
              // We lose more area shrinking the right side, so shrink the top side
              topEdgeShrunk = topShrinkTemp;
            } else {
              // We lose more area shrinking the top side, so shrink the right side
              rightEdgeShrunk = rightShrinkTemp;
            }
          }
        }
      }
    }
  }
  // Check bottom right corner
  for (int y = bottomEdge; y < _imageHeight; y++) {
    for (int x = rightEdge; x < _imageWidth; x++) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist > ignoreDist && pixDist < maxDepthExpandedPyramid) {
        if (numerator > (x - rightEdgeShrunk) * pixDist
            && numerator > (y - bottomEdgeShrunk) * pixDist) {
          // Both right and bottom edges could shrink
          int rightShrinkTemp = x - int(numerator / pixDist);
          int bottomShrinkTemp = y - int(numerator / pixDist);
          if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer
              && y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
            // Shrinking either edge makes the pyramid exclude the starting point
            return false;
          } else if (x0 > rightShrinkTemp - _pyramidSearchPixelBuffer) {
            // Shrinking right edge makes pyramid exclude the starting point, so shrink the bottom edge
            bottomEdgeShrunk = bottomShrinkTemp;
          } else if (y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
            // Shrinking bottom edge makes pyramid exclude the starting point, so shrink the right edge
            rightEdgeShrunk = rightShrinkTemp;
          } else {
            // We can shrink either edge and still have a feasible pyramid, choose the edge that removes the least area
            int rShrinkLostArea = (rightEdgeShrunk - rightShrinkTemp)
                * (bottomEdgeShrunk - topEdgeShrunk);
            int dShrinkLostArea = (bottomEdgeShrunk - bottomShrinkTemp)
                * (rightEdgeShrunk - leftEdgeShrunk);
            if (rShrinkLostArea > dShrinkLostArea) {
              // We lose more area shrinking the right side, so shrink the bottom side
              bottomEdgeShrunk = bottomShrinkTemp;
            } else {
              // We lose more area shrinking the bottom side, so shrink the right side
              rightEdgeShrunk = rightShrinkTemp;
            }
          }
        }
      }
    }
  }
  // Check top left corner
  for (int y = topEdge; y >= 0; y--) {
    for (int x = leftEdge; x >= 0; x--) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist > ignoreDist && pixDist < maxDepthExpandedPyramid) {
        if ((leftEdgeShrunk - x) * pixDist < numerator
            && (topEdgeShrunk - y) * pixDist < numerator) {
          // Both left and top edges could shrink
          int leftShrinkTemp = x + int(numerator / pixDist);
          int topShrinkTemp = y + int(numerator / pixDist);
          if (x0 < leftShrinkTemp + _pyramidSearchPixelBuffer
              && y0 < topShrinkTemp + _pyramidSearchPixelBuffer) {
            // Shrinking either edge makes the pyramid exclude the starting point
            return false;
          } else if (x0 < leftShrinkTemp + _pyramidSearchPixelBuffer) {
            // Shrinking left edge makes pyramid exclude the starting point, so shrink the top edge
            topEdgeShrunk = topShrinkTemp;
          } else if (y0 < topShrinkTemp + _pyramidSearchPixelBuffer) {
            // Shrinking top edge makes pyramid exclude the starting point, so shrink the left edge
            leftEdgeShrunk = leftShrinkTemp;
          } else {
            // We can shrink either edge and still have a feasible pyramid, choose the edge that removes the least area
            int lShrinkLostArea = (leftShrinkTemp - leftEdgeShrunk)
                * (bottomEdgeShrunk - topEdgeShrunk);
            int uShrinkLostArea = (topShrinkTemp - topEdgeShrunk)
                * (rightEdgeShrunk - leftEdgeShrunk);
            if (lShrinkLostArea > uShrinkLostArea) {
              // We lose more area shrinking the left side, so shrink the top side
              topEdgeShrunk = topShrinkTemp;
            } else {
              // We lose more area shrinking the top side, so shrink the left side
              leftEdgeShrunk = leftShrinkTemp;
            }
          }
        }
      }
    }
  }
  // Check bottom left corner
  for (int y = bottomEdge; y < _imageHeight; y++) {
    for (int x = leftEdge; x >= 0; x--) {
      pixDist = _depthData[y * _imageWidth + x];
      if (pixDist > ignoreDist && pixDist < maxDepthExpandedPyramid) {
        if ((leftEdgeShrunk - x) * pixDist < numerator
            && numerator > (y - bottomEdgeShrunk) * pixDist) {
          // Both left and bottom edges could shrink
          int leftShrinkTemp = x + int(numerator / pixDist);
          int bottomShrinkTemp = y - int(numerator / pixDist);
          if (x0 < leftShrinkTemp + _pyramidSearchPixelBuffer
              && y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
            // Shrinking either edge makes the pyramid exclude the starting point
            return false;
          } else if (x0 < leftShrinkTemp + _pyramidSearchPixelBuffer) {
            // Shrinking left edge makes pyramid exclude the starting point, so shrink the bottom edge
            bottomEdgeShrunk = bottomShrinkTemp;
          } else if (y0 > bottomShrinkTemp - _pyramidSearchPixelBuffer) {
            // Shrinking bottom edge makes pyramid exclude the starting point, so shrink the left edge
            leftEdgeShrunk = leftShrinkTemp;
          } else {
            // We can shrink either edge and still have a feasible pyramid, choose the edge that removes the least area
            int lShrinkLostArea = (leftShrinkTemp - leftEdgeShrunk)
                * (bottomEdgeShrunk - topEdgeShrunk);
            int dShrinkLostArea = (bottomEdgeShrunk - bottomShrinkTemp)
                * (rightEdgeShrunk - leftEdgeShrunk);
            if (lShrinkLostArea > dShrinkLostArea) {
              // We lose more area shrinking the left side, so shrink the bottom side
              bottomEdgeShrunk = bottomShrinkTemp;
            } else {
              // We lose more area shrinking the bottom side, so shrink the left side
              leftEdgeShrunk = leftShrinkTemp;
            }
          }
        }
      }
    }
  }

  int edgesFinal[4] = { rightEdgeShrunk, topEdgeShrunk, leftEdgeShrunk,
      bottomEdgeShrunk };
  double depth = maxDepthExpandedPyramid * _depthScale
      - _vehicleRadiusForPlanning;

  // Create a new pyramid
  Vec3 corners[4];
  // Top right
  DeprojectPixelToPoint(double(edgesFinal[0]), double(edgesFinal[1]), depth,
                        corners[0]);
  // Top left
  DeprojectPixelToPoint(double(edgesFinal[2]), double(edgesFinal[1]), depth,
                        corners[1]);
  // Bottom left
  DeprojectPixelToPoint(double(edgesFinal[2]), double(edgesFinal[3]), depth,
                        corners[2]);
  // Bottom right
  DeprojectPixelToPoint(double(edgesFinal[0]), double(edgesFinal[3]), depth,
                        corners[3]);
  outPyramid = Pyramid(depth, edgesFinal, corners);

  return true;
}