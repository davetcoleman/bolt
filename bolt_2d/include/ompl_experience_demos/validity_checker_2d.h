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
   Desc:   Custom State Validity Checker with cost function
*/

#ifndef OMPL_EXPERIENCE_DEMOS_VALIDITY_CHECKER_2D_H
#define OMPL_EXPERIENCE_DEMOS_VALIDITY_CHECKER_2D_H

// Boost
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

// OMPL
#include <ompl/base/StateValidityChecker.h>
#include <ompl/tools/debug/Visualizer.h>

// C++
#include <limits>

// Boost
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread.hpp>

namespace ob = ompl::base;

namespace ompl_visual_tools
{
typedef boost::numeric::ublas::matrix<int> intMatrix;
typedef std::shared_ptr<intMatrix> intMatrixPtr;
}

namespace ompl
{
namespace base
{
// Nat_Rounding helper function to make readings from cost map more accurate
int nat_round(double x)
{
  return static_cast<int>(floor(x + 0.5f));
}

/// @cond IGNORE
OMPL_CLASS_FORWARD(ValidityChecker2D);
/// @endcond

class ValidityChecker2D : public ob::StateValidityChecker
{
public:
  /** \brief Constructor */
  ValidityChecker2D(const ob::SpaceInformationPtr &si, ompl_visual_tools::intMatrixPtr cost, double max_threshold)
    : StateValidityChecker(si)
  {
    cost_ = cost;
    max_threshold_ = max_threshold;

    // We can detect clearance
    specs_.clearanceComputationType = StateValidityCheckerSpecs::APPROXIMATE;
  }

  /** \brief Return true if the state \e state is valid. Usually, this means at least collision checking. If it is
      possible that ompl::base::StateSpace::interpolate() or ompl::control::ControlSpace::propagate() return states that
      are outside of bounds, this function should also make a call to ompl::base::SpaceInformation::satisfiesBounds().
     */
  virtual bool isValid(const ob::State *state) const
  {
    if (!enabled_)
      return true;

    return cost(state) < max_threshold_ && cost(state) > 1;

    /*
       // Debug visualization with multiple threads
    bool result = cost(state) < max_threshold_ && cost(state) > 1;


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

    // double cost = cost(state);
    // return cost < max_threshold_ && cost > 1;  // TODO(davetcoleman): why greater than 1?
  }

  /** \brief Report the distance to the nearest invalid state when starting from \e state. If the distance is
      negative, the value of clearance is the penetration depth.*/
  virtual double clearance(const ob::State *state) const
  {
    return clearance(state, NULL);
  }

  virtual double clearance(const ob::State *state, tools::VisualizerPtr visual) const
  {
    // Check if the starting point is in collision. If it is, just return that one
    if (!isValid(state))
    {
      return 0;  // TODO(davetcoleman): we do not yet calculate penetration depth
    }

    double discretization = 0.25;  // std::min(1.0, si_->getStateValidityCheckingResolution() * 10);

    // Copy the state so that we have the correct 3rd dimension if it exists
    // si_->copyState(/*destination*/ work_state_, /*source*/ state);
    base::State *work_state = si_->cloneState(state);

    // Find the nearest invalid state
    bool result = searchSpiralForState(discretization, work_state, visual);

    if (visual)
    {
      visual->viz6()->trigger();
      usleep(0.1 * 1000000);
    }

    if (!result)
    {
      // No invalid state found within clearanceSearchDistance_
      return std::numeric_limits<double>::infinity();  // indicates collision is very far away
    }

    return si_->distance(state, work_state);
  }

  void setCheckingEnabled(bool collision_checking_enabled)
  {
    enabled_ = collision_checking_enabled;
  }

  /** \brief Get class for managing various visualization features */
  ompl::tools::VisualizerPtr getVisual()
  {
    return visual_;
  }

  /** \brief Set class for managing various visualization features */
  void setVisual(ompl::tools::VisualizerPtr visual)
  {
    visual_ = visual;
  }

private:
  /**
   * \brief Search in spiral around starting state (xs, ys) for nearby state that is in collision
   * \param discretization - how often to check for nearby obstacle
   * \param state - seed state that will also be filled with the result
   * \return true if found invalid state, false if no state is invalid within clearanceSearchDistance_
   */
  bool searchSpiralForState(double discretization, const ob::State *work_state, tools::VisualizerPtr visual) const
  {
    double *state_values = work_state->as<ob::RealVectorStateSpace::StateType>()->values;
    const bool show_growth = true;
    if (visual)
    {
      visual->viz6()->state(work_state, tools::MEDIUM, tools::RED, 0);
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
        if (visual && show_growth)
          visual->viz6()->state(work_state, tools::SMALL, tools::BLUE, 0);

        if (si_->satisfiesBounds(work_state))  // Check bounds
        {
          foundStateThatSatisfiesBounds = true;
          if (!isValid(work_state))  // Check validity
            return true;             // found invalid state
        }

        // Point 2
        state_values[0] = xs + d - i;
        state_values[1] = ys + i;
        if (visual && show_growth)
          visual->viz6()->state(work_state, tools::SMALL, tools::BLUE, 0);

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
        if (visual && show_growth)
          visual->viz6()->state(work_state, tools::SMALL, tools::BLUE, 0);

        if (si_->satisfiesBounds(work_state))  // Check bounds
        {
          foundStateThatSatisfiesBounds = true;
          if (!isValid(work_state))  // Check validity
            return true;             // found invalid state
        }

        // Point 4
        state_values[0] = xs + d - i;
        state_values[1] = ys - i;
        if (visual && show_growth)
          visual->viz6()->state(work_state, tools::SMALL, tools::BLUE, 0);

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

  // Note: this cost function is not the one used for the optimization objective, it is only a helper function for
  // isValid
  double cost(const ob::State *state) const
  {
    const double *coords = state->as<ob::RealVectorStateSpace::StateType>()->values;

    // Return the cost from the matrix at the current dimensions
    //double cost = (*cost_)(nat_round(coords[1]), nat_round(coords[0]));
    double cost = (*cost_)(floor(coords[1]), floor(coords[0]));
    return cost;  // TODO(davetcoleman): make memory copy more efficient
  }

  ompl_visual_tools::intMatrixPtr cost_;
  double max_threshold_;

  bool enabled_ = true;

  /** \brief Class for managing various visualization features */
  ompl::tools::VisualizerPtr visual_;

  //mutable boost::mutex vizMutex_;

};  // class ValidityChecker2D

}  // namespace base
}  // namespace ompl

#endif
