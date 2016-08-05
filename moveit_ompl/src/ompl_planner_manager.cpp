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

#include <moveit/ompl/ompl_planner_manager.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/ompl/detail/constrained_valid_state_sampler.h>
#include <moveit/profiler/profiler.h>
#include <fstream>

namespace moveit_ompl
{
OMPLPlannerManager::OMPLPlannerManager() : planning_interface::PlannerManager(), nh_("~"), simplify_solutions_(true)
{
}

bool OMPLPlannerManager::initialize(const robot_model::RobotModelConstPtr &robot_model, const std::string &ns)
{
  ROS_INFO("Initializing OMPL interface");

  // Load visualizer
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("/odom", "/moveit_ompl_markers", robot_model));
  visual_tools_->loadRobotStatePub("/moveit_ompl/display_robot_state");
  visual_tools_->hideRobot();

  // Save parameters
  robot_model_ = robot_model;
  if (!ns.empty())
    nh_ = ros::NodeHandle(ns);
  std::string ompl_ns = ns.empty() ? "ompl" : ns + "/ompl";

  // Load managers
  constraint_sampler_manager_.reset(new constraint_samplers::ConstraintSamplerManager());
  constraint_sampler_manager_loader_.reset(
      new constraint_sampler_manager_loader::ConstraintSamplerManagerLoader(constraint_sampler_manager_));

  context_manager_.reset(new PlanningContextManager(robot_model_, constraint_sampler_manager_));

  return true;
}

bool OMPLPlannerManager::canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
{
  return req.trajectory_constraints.constraints.empty();
}

void OMPLPlannerManager::getPlanningAlgorithms(std::vector<std::string> &algs) const
{

}

void OMPLPlannerManager::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pconfig)
{
  planning_interface::PlannerConfigurationMap pconfig2 = pconfig;

  // construct default configurations for planning groups that don't have configs already passed in
  const std::vector<const robot_model::JointModelGroup *> &groups = robot_model_->getJointModelGroups();
  for (std::size_t i = 0; i < groups.size(); ++i)
  {
    if (pconfig.find(groups[i]->getName()) == pconfig.end())
    {
      planning_interface::PlannerConfigurationSettings empty;
      empty.name = empty.group = groups[i]->getName();
      pconfig2[empty.name] = empty;
    }
  }

  context_manager_->setPlannerConfigurations(pconfig2);

  PlannerManager::setPlannerConfigurations(getPlannerConfigurations());
}

planning_interface::PlanningContextPtr OMPLPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr &planning_scene, const planning_interface::MotionPlanRequest &req,
    moveit_msgs::MoveItErrorCodes &error_code) const
{
  // Check if its already been loaded
  if (planning_context_)
    return planning_context_;

  context->setMaximumPlanningThreads(max_planning_threads_);
  context->setMaximumGoalSamples(max_goal_samples_);
  context->setMaximumStateSamplingAttempts(max_state_sampling_attempts_);
  context->setMaximumGoalSamplingAttempts(max_goal_sampling_attempts_);
  if (max_solution_segment_length_ <= std::numeric_limits<double>::epsilon())
    context->setMaximumSolutionSegmentLength(context->getOMPLSimpleSetup()->getStateSpace()->getMaximumExtent() /
                                             100.0);
  else
    context->setMaximumSolutionSegmentLength(max_solution_segment_length_);
  context->setMinimumWaypointCount(minimum_waypoint_count_);

}

}  // namespace
