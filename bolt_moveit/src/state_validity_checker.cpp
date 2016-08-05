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

/* Author: Dave Coleman, Ioan Sucan */

#include <curie_demos/state_validity_checker.h>
#include <ompl/base/SpaceInformation.h>
#include <ros/ros.h>
#include <moveit_ompl/detail/threadsafe_state_storage.h>

moveit_ompl::StateValidityChecker::StateValidityChecker(const std::string &group_name,
                                                        ompl::base::SpaceInformationPtr &si,
                                                        const moveit::core::RobotState &start_state,
                                                        const planning_scene::PlanningSceneConstPtr &planning_scene,
                                                        moveit_ompl::ModelBasedStateSpacePtr &mb_state_space)
  : ompl::base::StateValidityChecker(si)
  , group_name_(group_name)
  , tss_(start_state)
  , planning_scene_(planning_scene)
  , mb_state_space_(mb_state_space)
  , si_(si)
  , verbose_(false)
{
  specs_.clearanceComputationType = ompl::base::StateValidityCheckerSpecs::APPROXIMATE;
  specs_.hasValidDirectionComputation = false;

  collision_request_with_distance_.distance = true;
  collision_request_with_cost_.cost = true;

  collision_request_simple_.group_name = group_name_;
  collision_request_with_distance_.group_name = group_name_;
  collision_request_with_cost_.group_name = group_name_;

  collision_request_simple_verbose_ = collision_request_simple_;
  collision_request_simple_verbose_.verbose = true;

  collision_request_with_distance_verbose_ = collision_request_with_distance_;
  collision_request_with_distance_verbose_.verbose = true;

  setCheckingEnabled(true);
}

void moveit_ompl::StateValidityChecker::setVerbose(bool flag)
{
  verbose_ = flag;
}

bool moveit_ompl::StateValidityChecker::isValid(const ompl::base::State *state, bool verbose) const
{
  // check bounds
  if (!si_->satisfiesBounds(state))
  {
    if (verbose)
      ROS_INFO("State outside bounds");
    return false;
  }

  // Debugging mode that always says state is collision free
  if (!checking_enabled_)
  {
    return true;
  }

  // convert ompl state to moveit robot state
  robot_state::RobotState *robot_state = tss_.getStateStorage();
  mb_state_space_->copyToRobotState(*robot_state, state);

  // check path constraints
  // const kinematic_constraints::KinematicConstraintSetPtr &kset = planning_context_->getPathConstraints();
  // if (kset && !kset->decide(*robot_state, verbose).satisfied)
  //   return false;

  // check feasibility
  if (!planning_scene_->isStateFeasible(*robot_state, verbose))
    return false;

  // check collision avoidance
  collision_detection::CollisionResult res;
  if (verbose)
  {
    planning_scene_->checkCollision(collision_request_simple_verbose_, res, *robot_state);
  }
  else
  {
    planning_scene_->checkCollision(collision_request_simple_, res, *robot_state);
  }

  return res.collision == false;
}

bool moveit_ompl::StateValidityChecker::isValid(const ompl::base::State *state, double &dist, bool verbose) const
{
  if (!si_->satisfiesBounds(state))
  {
    if (verbose)
      ROS_INFO("State outside bounds");
    return false;
  }

  // Debugging mode that always says state is collision free
  if (!checking_enabled_)
  {
    dist = std::numeric_limits<double>::infinity();
    return true;
  }

  robot_state::RobotState *robot_state = tss_.getStateStorage();
  mb_state_space_->copyToRobotState(*robot_state, state);

  // check path constraints
  // const kinematic_constraints::KinematicConstraintSetPtr &kset = planning_context_->getPathConstraints();
  // if (kset)
  // {
  //   kinematic_constraints::ConstraintEvaluationResult cer = kset->decide(*robot_state, verbose);
  //   if (!cer.satisfied)
  //   {
  //     dist = cer.distance;
  //     return false;
  //   }
  // }

  // check feasibility
  if (!planning_scene_->isStateFeasible(*robot_state, verbose))
  {
    dist = 0.0;
    return false;
  }

  // check collision avoidance
  collision_detection::CollisionResult res;
  planning_scene_->checkCollision(verbose ? collision_request_with_distance_verbose_ : collision_request_with_distance_,
                                  res, *robot_state);
  dist = res.distance;

  // Visualize
  // if (res.collision)
  //   visual_->viz2()->state(state, ompl::tools::SMALL, ompl::tools::RED, 0);
  // else
  //   visual_->viz2()->state(state, ompl::tools::SMALL, ompl::tools::GREEN, 0);
  // visual_->viz2()->trigger();

  return res.collision == false;
}

double moveit_ompl::StateValidityChecker::cost(const ompl::base::State *state) const
{
  double cost = 0.0;

  robot_state::RobotState *robot_state = tss_.getStateStorage();
  mb_state_space_->copyToRobotState(*robot_state, state);

  // Calculates cost from a summation of distance to obstacles times the size of the obstacle
  collision_detection::CollisionResult res;
  planning_scene_->checkCollision(collision_request_with_cost_, res, *robot_state);

  for (std::set<collision_detection::CostSource>::const_iterator it = res.cost_sources.begin();
       it != res.cost_sources.end(); ++it)
    cost += it->cost * it->getVolume();

  return cost;
}

double moveit_ompl::StateValidityChecker::clearance(const ompl::base::State *state) const
{
  robot_state::RobotState *robot_state = tss_.getStateStorage();
  mb_state_space_->copyToRobotState(*robot_state, state);

  collision_detection::CollisionResult res;
  planning_scene_->checkCollision(collision_request_with_distance_, res, *robot_state);
  return res.collision ? 0.0 : (res.distance < 0.0 ? std::numeric_limits<double>::infinity() : res.distance);
}

void moveit_ompl::StateValidityChecker::setCheckingEnabled(const bool &checking_enabled)
{
  checking_enabled_ = checking_enabled;

  if (!checking_enabled_)
  {
    ROS_WARN_STREAM_NAMED(group_name_, "StateValidityChecker collision checking is DISABLED");
  }
  else
    ROS_INFO_STREAM_NAMED(group_name_, "StateValidityChecker collision checking is enabled");
}
