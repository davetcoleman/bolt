/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include <moveit/ompl/detail/constrained_valid_state_sampler.h>
#include <moveit/ompl/model_based_planning_context.h>
#include <moveit/profiler/profiler.h>

moveit_ompl::ValidConstrainedSampler::ValidConstrainedSampler(
    const ModelBasedPlanningContext *pc, const kinematic_constraints::KinematicConstraintSetPtr &ks,
    const constraint_samplers::ConstraintSamplerPtr &cs, moveit_visual_tools::MoveItVisualToolsPtr visual_tools)
  : ob::ValidStateSampler(pc->getOMPLSimpleSetup()->getSpaceInformation().get())
  , planning_context_(pc)
  , kinematic_constraint_set_(ks)
  , constraint_sampler_(cs)
  , work_state_(pc->getCompleteInitialRobotState())
  , visual_tools_(visual_tools)
{
  // constraint_sampler_->setVerbose(true); // dave hack
  if (!constraint_sampler_)
    default_sampler_ = si_->allocStateSampler();
  inv_dim_ = si_->getStateSpace()->getDimension() > 0 ? 1.0 / (double)si_->getStateSpace()->getDimension() : 1.0;

  ROS_ERROR("Constructed a ValidConstrainedSampler instance at address %p", this);
}

bool moveit_ompl::ValidConstrainedSampler::project(ompl::base::State *state)
{
  std::cout << "moveit_ompl::ValidConstrainedSampler::project() with attempts "
            << planning_context_->getMaximumStateSamplingAttempts() << std::endl;

  if (constraint_sampler_)
  {
    planning_context_->getOMPLStateSpace()->copyToRobotState(work_state_, state);
    if (constraint_sampler_->project(work_state_, planning_context_->getMaximumStateSamplingAttempts()))
    {
      if (kinematic_constraint_set_->decide(work_state_).satisfied)
      {
        std::cout << " >> Kinematic constraint decided satisfied true " << std::endl;
        if (visual_tools_)
          visual_tools_->publishRobotState(work_state_, rviz_visual_tools::PURPLE);
        planning_context_->getOMPLStateSpace()->copyToOMPLState(state, work_state_);
        return true;
      }
    }
  }
  return false;
}

bool moveit_ompl::ValidConstrainedSampler::sample(ob::State *state)
{
  std::cout << "moveit_ompl::ValidConstrainedSampler::sample() with attempts "
            << planning_context_->getMaximumStateSamplingAttempts() << std::endl;

  if (constraint_sampler_)
  {
    if (constraint_sampler_->sample(work_state_, planning_context_->getCompleteInitialRobotState(),
                                    planning_context_->getMaximumStateSamplingAttempts()))
    {
      if (kinematic_constraint_set_->decide(work_state_).satisfied)
      {
        std::cout << " >> Kinematic constraint decided satisfied true " << std::endl;
        if (visual_tools_)
          visual_tools_->publishRobotState(work_state_, rviz_visual_tools::BROWN);
        planning_context_->getOMPLStateSpace()->copyToOMPLState(state, work_state_);
        return true;
      }
    }
  }
  else
  {
    default_sampler_->sampleUniform(state);
    planning_context_->getOMPLStateSpace()->copyToRobotState(work_state_, state);
    if (kinematic_constraint_set_->decide(work_state_).satisfied)
      return true;
  }

  return false;
}

bool moveit_ompl::ValidConstrainedSampler::sampleNear(ompl::base::State *state, const ompl::base::State *near,
                                                      const double distance)
{
  std::cout << "moveit_ompl::ValidConstrainedSampler::sampleNear() " << std::endl;

  if (!sample(state))
    return false;
  double total_d = si_->distance(state, near);
  if (total_d > distance)
  {
    double dist = pow(rng_.uniform01(), inv_dim_) * distance;
    si_->getStateSpace()->interpolate(near, state, dist / total_d, state);
    planning_context_->getOMPLStateSpace()->copyToRobotState(work_state_, state);
    if (!kinematic_constraint_set_->decide(work_state_).satisfied)
      return false;
  }
  return true;
}

bool moveit_ompl::ValidConstrainedSampler::createState(ob::State *state, std::vector<double> &values)
{
  std::cout << "moveit_ompl::ValidConstrainedSampler::createState()" << std::endl;

  // Copy vector to moveit::RobotState
  work_state_.setVariablePositions(values);

  if (visual_tools_)
    visual_tools_->publishRobotState(work_state_, rviz_visual_tools::PURPLE);
  ros::Duration(1).sleep();

  // Convert to OMPL
  planning_context_->getOMPLStateSpace()->copyToRobotState(work_state_, state);

  //if (kinematic_constraint_set_->decide(work_state_).satisfied)
  //return true;

  return false;
}
