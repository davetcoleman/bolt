/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK, The University of Tokyo.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK, The University of Tokyo nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Custom State Validity Checker based on PPM map - black is obstacle
*/

#ifndef BOLT_2D_VALIDITY_CHECKER_2D_H
#define BOLT_2D_VALIDITY_CHECKER_2D_H

// Boost
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

// OMPL
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// C++
#include <limits>

// Boost
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread.hpp>

namespace ob = ompl::base;

namespace bolt_2d
{
typedef boost::numeric::ublas::matrix<int> intMatrix;
typedef std::shared_ptr<intMatrix> intMatrixPtr;
}

namespace ompl
{
namespace base
{

/// @cond IGNORE
OMPL_CLASS_FORWARD(ValidityChecker2D);
/// @endcond

static const double MAX_COLOR = 255.0 * 3  - 2.0 * std::numeric_limits<double>::epsilon();

class ValidityChecker2D : public ob::StateValidityChecker
{
public:
  /** \brief Constructor */
  ValidityChecker2D(const ob::SpaceInformationPtr &si, ompl::PPM* ppm)
    : StateValidityChecker(si)
    , ppm_(ppm)
  {
    // Set default value
    clearanceSearchDistance_ = 1.0;

    // We can detect clearance
    specs_.clearanceComputationType = StateValidityCheckerSpecs::APPROXIMATE;
  }

  /** \brief Return true if the state \e state is valid. In addition, set \e dist to the distance to the
   * nearest invalid state. */
  virtual bool isValid(const State *state, double &dist) const
  {
    if (!isValid(state))
      return false;

    // Delay doing clearance check until we're sure its not in collision
    dist = clearance(state);
    return true;
  }

  /** \brief Return true if the state \e state is valid. Usually, this means at least collision checking. If it is
      possible that ompl::base::StateSpace::interpolate() or ompl::control::ControlSpace::propagate() return states that
      are outside of bounds, this function should also make a call to ompl::base::SpaceInformation::satisfiesBounds().
  */
  virtual bool isValid(const ob::State *state) const
  {
    if (!enabled_)
      return true;

    const double *coords = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    const ompl::PPM::Color& map_color = ppm_->getPixel(floor(coords[1]), floor(coords[0]));

    return (map_color.red + map_color.green + map_color.blue >= MAX_COLOR);

    /*
       // Debug visualization with multiple threads
       bool result = (map_color.red + map_color.green + map_color.blue >= MAX_COLOR);

    boost::lock_guard<boost::mutex> lock(vizMutex_);
    {
      // Visualize
      if (!result)
        visual_->viz2()->state(state, ompl::tools::SMALL, ompl::tools::RED, 0);
      else
        visual_->viz2()->state(state, ompl::tools::SMALL, ompl::tools::GREEN, 0);
      visual_->viz2()->trigger();
      usleep(0.001*1000000);
    }
    return result;
    */
  }

  /** \brief Report the distance to the nearest invalid state when starting from \e state. If the distance is
      negative, the value of clearance is the penetration depth.*/
  virtual double clearance(const ob::State *state) const
  {
    static const double DISCRETIZATION = 0.25;  // std::min(1.0, si_->getStateValidityCheckingResolution() * 10);

    // Copy the state so that we have the correct 3rd dimension if it exists
    base::State *work_state = si_->cloneState(state);

    // Find the nearest invalid state
    bool result = spiralSearchCollisionState(DISCRETIZATION, work_state);

    if (visual_ && false)
    {
      visual_->viz6()->trigger();
      usleep(0.1 * 1000000);
    }

    if (!result)
    {
      // No invalid state found within clearanceSearchDistance_
      si_->freeState(work_state);
      return std::numeric_limits<double>::infinity();  // indicates collision is very far away
    }

    const double dist = si_->distance(state, work_state);
    si_->freeState(work_state);

    return dist;
  }

  void setCheckingEnabled(bool collision_checking_enabled)
  {
    enabled_ = collision_checking_enabled;
  }

private:
  /**
   * \brief Search in spiral around starting state (xs, ys) for nearby state that is in collision
   * \param discretization - how often to check for nearby obstacle
   * \param state - seed state that will also be filled with the result
   * \return true if found invalid state, false if no state is invalid within clearanceSearchDistance_
   */
  bool spiralSearchCollisionState(double discretization, const ob::State *work_state) const
  {
    double *state_values = work_state->as<ob::RealVectorStateSpace::StateType>()->values;
    static const bool VISUALIZE_SPIRAL = true;

    if (visual_ && VISUALIZE_SPIRAL)
    {
      visual_->viz6()->state(work_state, tools::MEDIUM, tools::RED, 0);
    }

    double xs = state_values[0];
    double ys = state_values[1];

    // Loop around starting point
    for (double d = discretization; d < clearanceSearchDistance_; d += discretization)
    {
      bool foundStateThatSatisfiesBounds = false;

      for (double i = 0; i < d + discretization; i += discretization)
      {
        // Point 1
        state_values[0] = xs - d + i;
        state_values[1] = ys - i;

        if (visual_ && VISUALIZE_SPIRAL) // Debug
        {
          visual_->viz6()->state(work_state, tools::MEDIUM, tools::BLUE, 0);
        }

        if (si_->satisfiesBounds(work_state))  // Check bounds
        {
          foundStateThatSatisfiesBounds = true;
          if (!isValid(work_state))  // Check validity
            return true;             // found invalid state
        }

        // Point 2
        state_values[0] = xs + d - i;
        state_values[1] = ys + i;

        if (visual_ && VISUALIZE_SPIRAL) // Debug
          visual_->viz6()->state(work_state, tools::SMALL, tools::BLUE, 0);

        if (si_->satisfiesBounds(work_state))  // Check bounds
        {
          foundStateThatSatisfiesBounds = true;
          if (!isValid(work_state))  // Check validity
            return true;             // found invalid state
        }
      }

      for (double i = discretization; i < d; i += discretization)
      {
        // Point 3
        state_values[0] = xs - i;
        state_values[1] = ys + d - i;

        if (visual_ && VISUALIZE_SPIRAL) // Debug
          visual_->viz6()->state(work_state, tools::SMALL, tools::BLUE, 0);

        if (si_->satisfiesBounds(work_state))  // Check bounds
        {
          foundStateThatSatisfiesBounds = true;
          if (!isValid(work_state))  // Check validity
            return true;             // found invalid state
        }

        // Point 4
        state_values[0] = xs + d - i;
        state_values[1] = ys - i;

        if (visual_ && VISUALIZE_SPIRAL) // Debug
          visual_->viz6()->state(work_state, tools::SMALL, tools::BLUE, 0);

        if (si_->satisfiesBounds(work_state))  // Check bounds
        {
          foundStateThatSatisfiesBounds = true;
          if (!isValid(work_state))  // Check validity
            return true;             // found invalid state
        }
      }

      if (!foundStateThatSatisfiesBounds)
      {
        std::cout << "Quitting early clearance() search because did not find state that satisfies bounds " << std::endl;
        // break;
      }
    }  // for clearanceSearchDistance_

    return false;
  }

  ompl::PPM* ppm_;
  double max_threshold_;

  bool enabled_ = true;

  // mutable boost::mutex vizMutex_;

};  // class ValidityChecker2D

}  // namespace base
}  // namespace ompl

#endif
