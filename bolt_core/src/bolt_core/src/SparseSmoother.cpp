/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Smoothing tools for Sparse Graphs
*/

// OMPL
#include <ompl/util/Console.h>
#include <ompl/base/DiscreteMotionValidator.h>

// Bolt
#include <bolt_core/SparseSmoother.h>

namespace ompl
{
namespace tools
{
namespace bolt
{
SparseSmoother::SparseSmoother(base::SpaceInformationPtr si, VisualizerPtr visual) : si_(si), visual_(visual)
{
}

void SparseSmoother::setup()
{
  // Initialize path simplifier
  if (!pathSimplifier_)
  {
    pathSimplifier_.reset(new geometric::PathSimplifier(si_));
    pathSimplifier_->freeStates(true);
  }
}

bool SparseSmoother::smoothQualityPath(geometric::PathGeometric *path, double clearance, bool debug, std::size_t indent)
{
  BOLT_FUNC(indent, vSmooth_, "smoothQualityPath() clearance: " << clearance);

  // These vars are only used when compiled in debug mode
  base::State *startCopy;
  base::State *goalCopy;

#ifndef NDEBUG
  // debug code
  startCopy = si_->cloneState(path->getState(0));
  goalCopy = si_->cloneState(path->getState(path->getStateCount() - 1));
#endif

  // Visualize path
  if (visualizeQualityPathSmoothing_)
  {
    visual_->viz2()->deleteAllMarkers();
    visual_->viz3()->deleteAllMarkers();
    visual_->viz4()->deleteAllMarkers();
    visual_->viz6()->deleteAllMarkers();

    visual_->viz2()->path(path, tools::SMALL, tools::BLACK, tools::BLUE);
    visual_->viz2()->trigger();
    usleep(0.001 * 1000000);
  }

  // Ensure that the number of states always decreases
  std::size_t minStatesFound = path->getStateCount();
  BOLT_DEBUG(indent, visualizeQualityPathSmoothing_, "Original quality path has " << minStatesFound << " states");

  // if (vSmooth_)
  //   visual_->waitForUserFeedback("path simplification");

  // Set the motion validator to use clearance, this way isValid() checks clearance before confirming valid
  base::DiscreteMotionValidator *dmv = dynamic_cast<base::DiscreteMotionValidator *>(si_->getMotionValidator().get());
  dmv->setRequiredStateClearance(clearance);

  for (std::size_t i = 0; i < 3; ++i)
  {
    ompl::base::PlannerTerminationCondition neverTerminate = base::plannerNonTerminatingCondition();
    pathSimplifier_->simplify(*path, neverTerminate);

    if (vSmooth_)
      std::cout << "path->getStateCount(): " << path->getStateCount() << std::endl;

    if (visualizeQualityPathSmoothing_)
    {
      // visual_->viz3()->deleteAllMarkers();
      visual_->viz3()->path(path, tools::SMALL, tools::BLACK, tools::ORANGE);
      visual_->viz3()->trigger();
      usleep(0.1 * 1000000);

      // visual_->waitForUserFeedback("optimizing path");
    }

    pathSimplifier_->reduceVertices(*path, 1000, path->getStateCount() * 4);  // /*rangeRatio*/ 0.33, indent);

    if (visualizeQualityPathSmoothing_)
    {
      // visual_->viz4()->deleteAllMarkers();
      visual_->viz4()->path(path, tools::SMALL, tools::BLACK, tools::BLUE);
      visual_->viz4()->trigger();
      usleep(0.1 * 1000000);

      // visual_->waitForUserFeedback("optimizing path");
    }
  }

  // Turn off the clearance requirement - this is the default value that the DMV should remain in
  dmv->setRequiredStateClearance(0.0);

  pathSimplifier_->reduceVertices(*path, 1000, path->getStateCount() * 4);  //, /*rangeRatio*/ 0.33, indent);

  if (vSmooth_)
    std::cout << "path->getStateCount(): " << path->getStateCount() << std::endl;

  if (visualizeQualityPathSmoothing_)
  {
    visual_->viz6()->deleteAllMarkers();
    visual_->viz6()->path(path, tools::SMALL, tools::BLACK, tools::GREEN);
    visual_->viz6()->trigger();
    // visual_->waitForUserFeedback("finished quality path");
  }

  // TODO: very rarely a path is created that is out of bounds and can't be repaired - I don't know why but its super
  // rare and I think the bug is inside of OMPL
  std::pair<bool, bool> repairResult = path->checkAndRepair(100);

#ifndef NDEBUG
  // debug code
  BOLT_ASSERT(si_->equalStates(path->getState(0), startCopy), "Start state is no longer the same");
  BOLT_ASSERT(si_->equalStates(path->getState(path->getStateCount() - 1), goalCopy), "Goal state is not the same");
  si_->freeState(startCopy);
  si_->freeState(goalCopy);
#endif

  if (!repairResult.second)  // Repairing was not successful
  {
    if (!visualizeQualityPathSmoothing_)
    {
      visual_->viz6()->deleteAllMarkers();
      visual_->viz6()->path(path, tools::SMALL, tools::BLACK, tools::GREEN);
      visual_->viz6()->trigger();
    }

    BOLT_ERROR(indent, "Check and repair failed (v2)");
    usleep(1 * 1000000);
    return false;
  }

  // Everything was successful
  return true;
}

bool SparseSmoother::smoothMax(geometric::PathGeometric *path, std::size_t indent)
{
  BOLT_FUNC(indent, visualizeQualityPathSmoothing_ && false, "smoothMax()");

  // Two-point paths can't be optimized
  if (path->getStateCount() < 3)
    return true;

  // For testing that start and goal state do not move
  base::State *startCopy;
  base::State *goalCopy;
  BOLT_ASSERT(startCopy = si_->cloneState(path->getState(0)), "Only copy if in debug");
  BOLT_ASSERT(goalCopy = si_->cloneState(path->getState(path->getStateCount() - 1)), "Only copy if in debug");

  // Visualize path
  if (visualizeQualityPathSmoothing_)
  {
    // Clear all windows
    for (std::size_t i = 3; i <= 6; ++i)
    {
      visual_->viz(i)->deleteAllMarkers();
      visual_->viz(i)->trigger();
    }

    visual_->viz2()->deleteAllMarkers();
    visual_->viz2()->path(path, tools::MEDIUM, tools::BLACK, tools::BLUE);
    visual_->viz2()->trigger();
    usleep(0.001 * 1000000);
  }

  // Set the motion validator to use clearance, this way isValid() checks clearance before confirming valid
  base::DiscreteMotionValidator *dmv = dynamic_cast<base::DiscreteMotionValidator *>(si_->getMotionValidator().get());
  BOLT_ASSERT(dmv->getRequiredStateClearance() < 2 * std::numeric_limits<double>::epsilon(),
              "Discrete motion validator should have clearance = 0");

  double prevDistance = std::numeric_limits<double>::infinity();
  std::size_t origStateCount = path->getStateCount();
  std::size_t loops = 0;
  // while path is improving and the state count hasn't gotten larger than 5x its original size
  while (prevDistance > path->length() && path->getStateCount() < 5 * origStateCount && loops++ < 5)
  {
    prevDistance = path->length();

    // ------------------------------------------------------------------
    // interpolate, but only once
    if (loops == 1)
    {
      path->interpolate();

      if (visualizeQualityPathSmoothing_)
      {
        visual_->viz3()->deleteAllMarkers();
        visual_->viz3()->path(path, tools::MEDIUM, tools::BLACK, tools::ORANGE);
        visual_->viz3()->trigger();
        usleep(0.1 * 1000000);
        // BOLT_DEBUG(indent, true, "path->length() " << path->length() << " states: " << path->getStateCount());
        // visual_->waitForUserFeedback("interpolate");
      }
    }

    // ------------------------------------------------------------------
    // try a randomized step of connecting vertices

    bool tryMore = true;
    std::size_t times = 0;
    // while (tryMore && ++times <= 5)
    while (tryMore)
    {
      tryMore =
          pathSimplifier_->reduceVertices(*path, 1000, path->getStateCount() * 4);  // /*rangeRatio*/ 0.33, indent);

      if (visualizeQualityPathSmoothing_)
      {
        visual_->viz4()->deleteAllMarkers();
        visual_->viz4()->path(path, tools::MEDIUM, tools::BLACK, tools::ORANGE);
        visual_->viz4()->trigger();
        usleep(0.01 * 1000000);
        // visual_->waitForUserFeedback("reduce vertices");
      }
      // BOLT_DEBUG(indent, true, "reduce vert: length: " << path->length() << " states: " << path->getStateCount());

      if (path->getStateCount() < 3)  // Can't smooth if only two points
        break;
    }

    // ------------------------------------------------------------------
    // try to collapse close-by vertices
    // pathSimplifier_->collapseCloseVertices(*path);

    // if (visualizeQualityPathSmoothing_)
    // {
    //   visual_->viz4()->deleteAllMarkers();
    //   visual_->viz4()->path(path, tools::MEDIUM, tools::ORANGE);
    //   visual_->viz4()->trigger();
    //   usleep(0.01 * 1000000);
    //   BOLT_DEBUG(indent, true, "length: " << path->length() << " states: " << path->getStateCount());
    //   //visual_->waitForUserFeedback("collapseCloseVertices");
    // }

    // if (path->getStateCount() < 3) // Can't smooth if only two points
    //   break;

    // // ------------------------------------------------------------------
    // // split path segments, not just vertices
    // pathSimplifier_->shortcutPath(*path);

    // if (visualizeQualityPathSmoothing_)
    // {
    //   visual_->viz5()->deleteAllMarkers();
    //   visual_->viz5()->path(path, tools::MEDIUM, tools::ORANGE);
    //   visual_->viz5()->trigger();
    //   usleep(0.01 * 1000000);
    //   BOLT_DEBUG(indent, true, "length: " << path->length() << " states: " << path->getStateCount());
    //   //visual_->waitForUserFeedback("shortcutPath");
    // }

    // ------------------------------------------------------------------
    // smooth the path with BSpline interpolation
    pathSimplifier_->smoothBSpline(*path, 5, path->length() / 100.0);

    if (visualizeQualityPathSmoothing_)
    {
      visual_->viz5()->deleteAllMarkers();
      visual_->viz5()->path(path, tools::MEDIUM, tools::BLACK, tools::ORANGE);
      visual_->viz5()->trigger();
      usleep(0.01 * 1000000);

      // visual_->waitForUserFeedback("smoothBSpline");
    }
    // BOLT_DEBUG(indent, true, "smoothBSpline length: " << path->length() << " states: " << path->getStateCount());

    // Reduce vertices yet again
    tryMore = true;
    times = 0;
    // while (tryMore && ++times <= 5 && path->getStateCount() > 2)
    while (tryMore && path->getStateCount() > 2)
    {
      tryMore =
          pathSimplifier_->reduceVertices(*path, 1000, path->getStateCount() * 4);  // /*rangeRatio*/ 0.33, indent);

      if (visualizeQualityPathSmoothing_)
      {
        visual_->viz3()->deleteAllMarkers();
        visual_->viz3()->path(path, tools::MEDIUM, tools::BLACK, tools::ORANGE);
        visual_->viz3()->trigger();
        usleep(0.01 * 1000000);

        // visual_->waitForUserFeedback("reduce vertices");
      }
      // BOLT_DEBUG(indent, true, "reduce vert length: " << path->length() << " states: " << path->getStateCount());

      if (path->getStateCount() < 3)  // Can't smooth if only two points
        break;
    }
  }

  BOLT_ASSERT(si_->equalStates(path->getState(0), startCopy), "Start state is no longer same");
  BOLT_ASSERT(si_->equalStates(path->getState(path->getStateCount() - 1), goalCopy), "Goal state is no longer same");

  return true;
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
