/*!
 * Rapid trajectory generation for quadrocopters
 *
 * Copyright 2014 by Mark W. Mueller <mwm@mwm.im>
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

/* Modified the original files from https://github.com/markwmuller/RapidQuadrocopterTrajectories
 Xiangyu Wu (wuxiangyu@berkeley.edu)

Changes:
  - used template to make the codes work on MCU
  - merged RapidTrajectoryGenerator.cpp and RapidTrajectoryGenerator.hpp into this single file
*/

/*
 * Modified by Gaurav Jalan (gaurav.jalan@berkeley/edu)
 *
 * Changes:
 * - implemented new Serbian root solver (CMakeLists.txt under Components also modified)
 * - templates for tsqrt moved from old "root solver" library to here
 */

#pragma once
#include "Common/Math/Vec3.hpp"
#include "SingleAxisTrajectory.hpp"
#include <algorithm>
#include <limits>
#include "Common/Math/RootFinder.hpp"
#include <cerrno>

namespace RapidQuadrocopterTrajectoryGenerator
{

//! A quadrocopter state interception trajectory.
/*!
 * A quadrocopter state interception trajectory. The trajectory starts at a
 * state defined by the vehicle's position, velocity, and acceleration. The
 * acceleration can be calculated directly from the quadrocopter's attitude
 * and thrust value. The trajectory duration is fixed, and given by the user.
 *
 * The trajectory goal state can include any combination of components from
 * the quadrocopter's position, velocity, and acceleration. The acceleration
 * allows to encode the direction of the quadrocopter's thrust at the end time.
 *
 * The trajectories are generated without consideration for any constraints,
 * and are optimal with respect to the integral of the jerk squared (which is
 * equivalent to an upper bound on a product of the inputs).
 *
 * The trajectories can then be tested with respect to input constraints
 * (thrust/body rates) with an efficient, recursive algorithm. Whether linear
 * combinations of states along the trajectory remain within some bounds can
 * also be tested efficiently.
 *
 * For more information, please see the publication `A computationally efficient motion primitive for quadrocopter trajectory generation', avaible here: http://www.mwm.im/research/publications/
 *
 * NOTE: in the publication, axes are 1-indexed, while here they are
 * zero-indexed.
 */

//Real can be float or double (float for onboard, else double).
template<typename Real>
class RapidTrajectoryGenerator
{
  static inline Real tacos(Real in);
  static inline Real tpow(Real base, Real power);
  static inline Real tabs(Real in);
  static inline Real tsqrt(Real in);

public:

  enum InputFeasibilityResult
  {
    InputFeasible = 0,//!<The trajectory is input feasible
    InputIndeterminable = 1,//!<Cannot determine whether the trajectory is feasible with respect to the inputs
    InputInfeasibleThrustHigh = 2,//!<Trajectory is infeasible, failed max. thrust test first
    InputInfeasibleThrustLow = 3,//!<Trajectory is infeasible, failed min. thrust test first
    InputInfeasibleRates = 4,//!<Trajectory is infeasible, failed max. rates test first
  };

  enum StateFeasibilityResult
  {
    StateFeasible = 0,//!<The trajectory is feasible w.r.t. the test
    StateInfeasible = 1,//!<Trajectory is infeasible
  };

  //! Constructor, user must define initial state, and the direction of gravity.
  RapidTrajectoryGenerator(const Vec3<Real> x0, const Vec3<Real> v0, const Vec3<Real> a0, const Vec3<Real> gravity)
  {
    //initialise each axis:
    Reset();
    for(int i=0; i<3; i++) _axis[i].SetInitialState(x0[i],v0[i],a0[i]);
    _grav = gravity;
  }

  //set the final state for all axes:
  //! Fix the full position at the end time (see also the per-axis functions).
  void SetGoalPosition(const Vec3<Real> in)
  {
    for(unsigned i=0;i<3;i++) SetGoalPositionInAxis(i, in[i]);
  }
  //! Fix the full velocity at the end time (see also the per-axis functions).
  void SetGoalVelocity(const Vec3<Real> in)
  {
    for(int i=0;i<3;i++) SetGoalVelocityInAxis(i, in[i]);
  }
  //! Fix the full acceleration at the end time (see also the per-axis functions).
  void SetGoalAcceleration(const Vec3<Real> in)
  {
    for(int i=0;i<3;i++) SetGoalAccelerationInAxis(i, in[i]);
  }

  //set final state per axis:
  //! Fix the position at the end time in one axis. If not set, it is left free.
  void SetGoalPositionInAxis(const unsigned axNum, const Real in){_axis[axNum].SetGoalPosition(in);};
  //! Fix the velocity at the end time in one axis. If not set, it is left free.
  void SetGoalVelocityInAxis(const unsigned axNum, const Real in){_axis[axNum].SetGoalVelocity(in);};
  //! Fix the acceleration at the end time in one axis. If not set, it is left free.
  void SetGoalAccelerationInAxis(const unsigned axNum, const Real in){_axis[axNum].SetGoalAcceleration(in);};

  //! Reset the trajectory, clearing any end state constraints.
  void Reset(void)
  {
    for(int i=0; i<3; i++)
    {
      _axis[i].Reset();
    }
    _tf = 0;
  }

  /*! Calculate the optimal trajectory of duration `timeToGo`.
   *
   * Calculate the full trajectory, for all the parameters defined so far.
   * @param timeToGo The trajectory duration, in [s].
   */
  void Generate(const Real timeToGo)
  {
    _tf = timeToGo;
    for(int i=0;i<3;i++)
    {
      _axis[i].GenerateTrajectory(_tf);
    }
  }

  /*! Test the trajectory for input feasibility.
   *
   * Test whether the inputs required along the trajectory are within the allowable limits.
   * Note that the test either
   *   - proves feasibility,
   *   - proves infeasibility,
   *   - fails to prove anything ("indeterminate")
   *
   * The user must also specify a minimumTimeSection, which then determines the
   * precision of tests (and thus limit the number of recursion steps).
   *
   * Refer to the paper for a full discussion on these tests.
   *
   * Note that if the result is not feasible, the result is that of the first
   * section which tested infeasible/indeterminate.
   *
   * @param fminAllowed Minimum thrust value inputs allowed [m/s**2].
   * @param fmaxAllowed Maximum thrust value inputs allowed [m/s**2].
   * @param wmaxAllowed Maximum body rates input allowed [rad/s].
   * @param minTimeSection Minimum time section to test during the recursion [s].
   * @return an instance of InputFeasibilityResult.
   */
  InputFeasibilityResult CheckInputFeasibility(Real fminAllowed, Real fmaxAllowed, Real wmaxAllowed, Real minTimeSection)
  {
    //required thrust limits along trajectory
    Real t1 = 0;
    Real t2 = _tf;

    return CheckInputFeasibilitySection(fminAllowed, fmaxAllowed, wmaxAllowed, t1, t2, minTimeSection);
  }

  /*! Test the trajectory for position feasibility.
   *
   * Test whether the position flown along the trajectory is feasible with
   * respect to a user defined boundary (modelled as a plane). The plane is
   * defined by a point lying on the plane, and the normal of the plane.
   *
   * No test is done for degenerate normals (of the form (0,0,0)).
   *
   * @param boundaryPoint The coordinates of any point on the plane defining the boundary.
   * @param boundaryNormal A Vector pointing in the allowable direction, away from the boundary.
   * @return An instance of StateFeasibilityResult.
   */

  StateFeasibilityResult CheckPositionFeasibility(Vec3<Real> boundaryPoint, Vec3<Real> boundaryNormal)
  {
    //Ensure that the normal is a unit vector:
    boundaryNormal = boundaryNormal.GetUnitVector();

    //first, we will build the polynomial describing the velocity of the a
    //quadrocopter in the direction of the normal. Then we will solve for
    //the zeros of this, which give us the times when the position is at a
    //critical point. Then we evaluate the position at these points, and at
    //the trajectory beginning and end, to see whether we are feasible.

    //need to check that the trajectory stays inside the safe box throughout the flight:

    //the coefficients of the quartic equation: x(t) = c[0]t**4 + c[1]*t**3 + c[2]*t**2 + c[3]*t + c[4]
    Real c[5] = { 0, 0, 0, 0, 0 };

    for(int dim=0; dim<3; dim++)
    {
      c[0] += boundaryNormal[dim]*_axis[dim].GetParamAlpha() / Real(24.0); //t**4
      c[1] += boundaryNormal[dim]*_axis[dim].GetParamBeta()  / Real(6.0); //t**3
      c[2] += boundaryNormal[dim]*_axis[dim].GetParamGamma() / Real(2.0); //t**2
      c[3] += boundaryNormal[dim]*_axis[dim].GetInitialAcceleration(); //t
      c[4] += boundaryNormal[dim]*_axis[dim].GetInitialVelocity(); //1
    }

    //Solve the roots (we prepend the times 0 and tf):
    Real roots[6];
    roots[0] = 0;
    roots[1] = _tf;

    size_t rootCount;
    if(tabs(c[0]) > Real(1e-6))
    {
      rootCount = RootFinder::solve_quartic<Real>(c[1] / c[0], c[2] / c[0], c[3] / c[0], c[4] / c[0], roots + 2);
    }
    else
    {
      rootCount = RootFinder::solve_cubic<Real>(c[2] / c[1], c[3] / c[1], c[4] / c[1], roots + 2);
    }

    for(unsigned i=0; i<(rootCount+2); i++)
    {
      //don't evaluate points outside the domain
      if(roots[i] < 0) continue;
      if(roots[i] > _tf) continue;

      if((GetPosition(roots[i]) - boundaryPoint).Dot(boundaryNormal) <= 0)
      {
        //touching, or on the wrong side of, the boundary!
        return StateInfeasible;
      }
    }
    return StateFeasible;
  }

  //! Return the jerk along the trajectory at time t
  Vec3<Real> GetJerk(Real t) const         {return Vec3<Real>(_axis[0].GetJerk(t), _axis[1].GetJerk(t), _axis[2].GetJerk(t));};
  //! Return the acceleration along the trajectory at time t
  Vec3<Real> GetAcceleration(Real t) const {return Vec3<Real>(_axis[0].GetAcceleration(t), _axis[1].GetAcceleration(t), _axis[2].GetAcceleration(t));};
  //! Return the velocity along the trajectory at time t
  Vec3<Real> GetVelocity(Real t) const     {return Vec3<Real>(_axis[0].GetVelocity(t), _axis[1].GetVelocity(t), _axis[2].GetVelocity(t));};
  //! Return the position along the trajectory at time t
  Vec3<Real> GetPosition(Real t) const     {return Vec3<Real>(_axis[0].GetPosition(t), _axis[1].GetPosition(t), _axis[2].GetPosition(t));};

  //! Return the quadrocopter's normal Vector along the trajectory at time t
  Vec3<Real>   GetNormalVector(Real t) const {return (GetAcceleration(t) - _grav).GetUnitVector();};
  //! Return the quadrocopter's thrust input along the trajectory at time t
  Real GetThrust(Real t) const     {return (GetAcceleration(t) - _grav).GetNorm2();};
  /*! Return the quadrocopter's body rates along the trajectory at time t
   *
   * Returns the required body rates along the trajectory. These are expressed
   * in the world frame (in which the trajectory was planned). To convert them
   * to (p,q,r), this needs to be rotated by the quadrocopter's attitude.
   *
   * The rates are calculated by taking the rates required to rotate the
   * quadrocopter's normal at time `t` to that at time `t+dt`. Therefore, if
   * the trajectory is used as implicit MPC control law, the value dt should
   * correspond to the controller period.
   *
   * @param t Time along the trajectory to be evaluated [s].
   * @param timeStep The timestep size for the finite differencing [s].
   * @return The body rates, expressed in the inertial frame [rad/s]
   */

  Vec3<Real>   GetOmega(Real t, Real timeStep) const
  {
    //Calculates the body rates necessary at time t, to rotate the normal vector.
    //The result is coordinated in the world frame, i.e. would have to be rotated into a
    //body frame.
    const Vec3<Real> n0 = GetNormalVector(t);
    const Vec3<Real> n1 = GetNormalVector(t + timeStep);

    const Vec3<Real> crossProd = n0.Cross(n1); //direction of omega, in inertial axes

    if(crossProd.GetNorm2() <= Real(1e-6)){
      // Cross product failed
      return Vec3<Real>(0, 0, 0);
    } else {
      errno = 0;
      Vec3<Real> ans = (tacos(n0.Dot(n1))/timeStep)*crossProd.GetUnitVector();
      if(errno){
        //either unit vector not defined, or numeric issues cause dot product > 1
        return Vec3<Real>(0, 0, 0);
      }
      return ans;
    }
  }

  //! Return the total cost of the trajectory.
  Real GetCost(void) const { return _axis[0].GetCost() + _axis[1].GetCost() + _axis[2].GetCost();};

  //! Return the parameter defining the trajectory.
	Real GetAxisParamAlpha(int i)          const{return _axis[i].GetParamAlpha();};
  //! Return the parameter defining the trajectory.
	Real GetAxisParamBeta(int i)           const{return _axis[i].GetParamBeta();};
  //! Return the parameter defining the trajectory.
	Real GetAxisParamGamma(int i)          const{return _axis[i].GetParamGamma();};

private:
	//! Test a section of the trajectory for input feasibility (recursion).
  InputFeasibilityResult CheckInputFeasibilitySection(Real fminAllowed, Real fmaxAllowed, Real wmaxAllowed, Real t1, Real t2, Real minTimeSection)
  {
    if (t2 - t1 < minTimeSection) return InputIndeterminable;
    //test the acceleration at the two limits:
    if (std::max(GetThrust(t1), GetThrust(t2)) > fmaxAllowed) return InputInfeasibleThrustHigh;
    if (std::min(GetThrust(t1), GetThrust(t2)) < fminAllowed) return InputInfeasibleThrustLow;

    Real fminSqr = 0;
    Real fmaxSqr = 0;
    Real jmaxSqr = 0;

    //Test the limits of the box we're putting around the trajectory:
    for (int i = 0; i < 3; i++)
    {
      Real amin, amax;
      _axis[i].GetMinMaxAcc(amin, amax, t1, t2);

      //distance from zero thrust point in this axis
      Real v1 = amin - _grav[i]; //left
      Real v2 = amax - _grav[i]; //right

      //definitely infeasible:
      if (std::max(tpow(v1, 2), tpow(v2, 2)) > tpow(fmaxAllowed, 2)) return InputInfeasibleThrustHigh;

      if (v1 * v2 < 0)
      {
        //sign of acceleration changes, so we've gone through zero
        fminSqr += 0;
      }
      else
      {
        fminSqr += tpow(std::min(tabs(v1), tabs(v2)), 2);
      }

      fmaxSqr += tpow(std::max(tabs(v1), tabs(v2)), 2);

      jmaxSqr += _axis[i].GetMaxJerkSquared(t1, t2);
    }

    Real fmin = tsqrt(fminSqr);
    Real fmax = tsqrt(fmaxSqr);
    Real wBound;
    if (fminSqr > Real(1e-6))
      wBound = tsqrt(jmaxSqr / fminSqr); //the 1e-6 is a divide-by-zero protection
    else
      wBound = std::numeric_limits<Real>::max();

    //definitely infeasible:
    if (fmax < fminAllowed)
      return InputInfeasibleThrustLow;
    if (fmin > fmaxAllowed)
      return InputInfeasibleThrustHigh;

    //possibly infeasible:
    if (fmin < fminAllowed || fmax > fmaxAllowed || wBound > wmaxAllowed) { //indeterminate: must check more closely:
      Real tHalf = (t1 + t2) / 2;
      InputFeasibilityResult r1 = CheckInputFeasibilitySection(fminAllowed,
          fmaxAllowed, wmaxAllowed, t1, tHalf, minTimeSection);

      if (r1 == InputFeasible) {
        //continue with second half
        return CheckInputFeasibilitySection(fminAllowed, fmaxAllowed,
            wmaxAllowed, tHalf, t2, minTimeSection);
      }

      //first section is already infeasible, or indeterminate:
      return r1;
    }

    //definitely feasible:
    return InputFeasible;
  }

  SingleAxisTrajectory<Real> _axis[3];//!<The axes along the single trajectories
  Vec3<Real> _grav;//!<gravity in the frame of the trajectory
  Real _tf; //!<trajectory end time [s]
};

template<>
inline float RapidTrajectoryGenerator<float>::tacos(float in) {
  return acosf(in);
}

template<>
inline double RapidTrajectoryGenerator<double>::tacos(double in) {
  return acos(in);
}

template<>
inline float RapidTrajectoryGenerator<float>::tpow(float base, float power){
  return powf(base, power);
}

template<>
inline double RapidTrajectoryGenerator<double>::tpow(double base, double power){
  return pow(base, power);
}
template<>
inline float RapidTrajectoryGenerator<float>::tabs(float in){
  return fabsf(in);
}
template<>
inline double RapidTrajectoryGenerator<double>::tabs(double in){
  return fabs(in);
}
template<>
inline float RapidTrajectoryGenerator<float>::tsqrt(float in){
  return sqrtf(in);
}
template<>
inline double RapidTrajectoryGenerator<double>::tsqrt(double in){
  return sqrt(in);
}


}//namespace


