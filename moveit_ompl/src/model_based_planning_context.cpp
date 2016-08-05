/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

#include <moveit/ompl/model_based_planning_context.h>
#include <moveit/ompl/detail/state_validity_checker.h>
#include <moveit/ompl/detail/constrained_sampler.h>
#include <moveit/ompl/detail/constrained_goal_sampler.h>
#include <moveit/ompl/detail/goal_union.h>
#include <moveit/ompl/detail/projection_evaluators.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/datastructures/PDF.h>

moveit_ompl::ModelBasedPlanningContext::ModelBasedPlanningContext(
    const std::string &name, const ModelBasedPlanningContextSpecification &spec,
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools)
  : planning_interface::PlanningContext(name, spec.state_space_->getJointModelGroup()->getName())
  , class_name_("model_based_planning_context")
  , spec_(spec)
  , complete_initial_robot_state_(spec.state_space_->getRobotModel())
  , shared_robot_state_(new moveit::core::RobotState(complete_initial_robot_state_))
  , ompl_simple_setup_(spec.ompl_simple_setup_)
  , ompl_benchmark_(*ompl_simple_setup_)
  , ompl_parallel_plan_(ompl_simple_setup_->getProblemDefinition())
  , ptc_(NULL)
  , last_plan_time_(0.0)
  , last_simplify_time_(0.0)
  , max_goal_samples_(0)
  , max_state_sampling_attempts_(0)
  , max_goal_sampling_attempts_(0)
  , max_planning_threads_(0)
  , max_solution_segment_length_(0.0)
  , minimum_waypoint_count_(0)
  , use_state_validity_cache_(true)
  , simplify_solutions_(true)
{
  ompl_simple_setup_->getStateSpace()->computeSignature(space_signature_);
  ompl_simple_setup_->getStateSpace()->setStateSamplerAllocator(
      boost::bind(&ModelBasedPlanningContext::allocPathConstrainedSampler, this, _1));
}

void moveit_ompl::ModelBasedPlanningContext::setProjectionEvaluator(const std::string &peval)
{
  if (!spec_.state_space_)
  {
    ROS_ERROR_STREAM_NAMED(class_name_, "No state space is configured yet");
    return;
  }
  ob::ProjectionEvaluatorPtr pe = getProjectionEvaluator(peval);
  if (pe)
    spec_.state_space_->registerDefaultProjection(pe);
}

ompl::base::ProjectionEvaluatorPtr
moveit_ompl::ModelBasedPlanningContext::getProjectionEvaluator(const std::string &peval) const
{
  if (peval.find_first_of("link(") == 0 && peval[peval.length() - 1] == ')')
  {
    std::string link_name = peval.substr(5, peval.length() - 6);
    if (getRobotModel()->hasLinkModel(link_name))
      return ob::ProjectionEvaluatorPtr(new ProjectionEvaluatorLinkPose(this, link_name));
    else
      ROS_ERROR_NAMED(class_name_, "Attempted to set projection evaluator with respect to position of link '%s', but that link is not "
                "known to the kinematic model.",
                             link_name.c_str());
  }
  else if (peval.find_first_of("joints(") == 0 && peval[peval.length() - 1] == ')')
  {
    std::string joints = peval.substr(7, peval.length() - 8);
    boost::replace_all(joints, ",", " ");
    std::vector<unsigned int> j;
    std::stringstream ss(joints);
    while (ss.good() && !ss.eof())
    {
      std::string v;
      ss >> v >> std::ws;
      if (getJointModelGroup()->hasJointModel(v))
      {
        unsigned int vc = getJointModelGroup()->getJointModel(v)->getVariableCount();
        if (vc > 0)
        {
          int idx = getJointModelGroup()->getVariableGroupIndex(v);
          for (int q = 0; q < vc; ++q)
            j.push_back(idx + q);
        }
        else
          ROS_WARN_NAMED(class_name_, "Ignoring joint '%s' in projection since it has 0 DOF", v.c_str());
      }
      else
        ROS_ERROR_NAMED(class_name_, "Attempted to set projection evaluator with respect to value of joint '%s', but that joint is "
                  "not known to the group '%s'.",
                  v.c_str(), getGroupName().c_str());
    }
    if (j.empty())
      ROS_ERROR_STREAM_NAMED(class_name_, "No valid joints specified for joint projection");
    else
      return ob::ProjectionEvaluatorPtr(new ProjectionEvaluatorJointValue(this, j));
  }
  else
    ROS_ERROR_NAMED(class_name_, "Unable to allocate projection evaluator based on description: '%s'", peval.c_str());
  return ob::ProjectionEvaluatorPtr();
}

ompl::base::StateSamplerPtr
moveit_ompl::ModelBasedPlanningContext::allocPathConstrainedSampler(const ompl::base::StateSpace *ss) const
{
  // std::cout << "ModelBasedPlanningContext::allocPathConstrainedSampler() " << std::endl;

  if (spec_.state_space_.get() != ss)
  {
    ROS_ERROR_STREAM_NAMED(class_name_, "Attempted to allocate a state sampler for an unknown state space");
    return ompl::base::StateSamplerPtr();
  }

  ROS_DEBUG_STREAM_NAMED(class_name_, "Allocating a new state sampler (attempts to use path constraints)");

  if (path_constraints_)
  {
    constraint_samplers::ConstraintSamplerPtr cs = spec_.constraint_sampler_manager_->selectSampler(
        getPlanningScene(), getGroupName(), path_constraints_->getAllConstraints());

    if (cs)
    {
      ROS_INFO_STREAM_NAMED(class_name_, "Allocating specialized, non-default, state sampler for state space");
      return ob::StateSamplerPtr(new ConstrainedSampler(this, cs));
    }
  }
  ROS_DEBUG_STREAM_NAMED(class_name_, "Allocating default state sampler for state space");
  return ss->allocDefaultStateSampler();
}

void moveit_ompl::ModelBasedPlanningContext::configure()
{
  // convert the input state to the corresponding OMPL state
  ompl::base::ScopedState<> ompl_start_state(spec_.state_space_);
  spec_.state_space_->copyToOMPLState(ompl_start_state.get(), getCompleteInitialRobotState());
  ompl_simple_setup_->setStartState(ompl_start_state);
  ompl_simple_setup_->setStateValidityChecker(ob::StateValidityCheckerPtr(new StateValidityChecker(this)));

  useConfig();
  if (ompl_simple_setup_->getGoal())
    ompl_simple_setup_->setup();

  // Set the visualization callbacks
  ompl_simple_setup_->setVizState(
      boost::bind(&moveit_ompl::ModelBasedPlanningContext::visualizationState, this, _1, _2, _3));
}

void moveit_ompl::ModelBasedPlanningContext::visualizationState(const ompl::base::State *state, std::size_t type,
                                                                        double neighborRadius)
{
  std::cout << "ModelBasedPlanningContext::visualizationState()" << std::endl;
  if (!visual_tools_)
    return;

  spec_.state_space_->copyToRobotState(*shared_robot_state_, state);

  switch (type)
  {
    case 1:
      visual_tools_->publishRobotState(shared_robot_state_, rviz_visual_tools::LIME_GREEN);
      break;
    case 2:
      visual_tools_->publishRobotState(shared_robot_state_, rviz_visual_tools::LIME_GREEN);
      break;
  }
  ros::spinOnce();

  // Debug data
  // std::cout << "Fixed link stability: " << shared_robot_state_->getFixedLinkStability() << std::endl;

  ros::Duration(5.0).sleep();
}

void moveit_ompl::ModelBasedPlanningContext::useConfig()
{
  const std::map<std::string, std::string> &config = spec_.config_;
  if (config.empty())
    return;
  std::map<std::string, std::string> cfg = config;

  // debug all configs
  for(std::map<std::string, std::string>::const_iterator it = cfg.begin();
      it != cfg.end(); ++it)
  {
    std::cout << it->first << " " << it->second << std::endl;
  }

  // set the projection evaluator
  std::map<std::string, std::string>::iterator it = cfg.find("projection_evaluator");
  if (it != cfg.end())
  {
    setProjectionEvaluator(boost::trim_copy(it->second));
    cfg.erase(it);
  }

  if (cfg.empty())
    return;

  // remove the 'type' parameter; the rest are parameters for the planner itself
  it = cfg.find("type");
  if (it == cfg.end())
  {
    if (name_ != getGroupName())
      ROS_WARN_STREAM_NAMED(class_name_, "Attribute 'type' not specified in planner configuration");
  }
  else
  {
    std::string type = it->second;
    cfg.erase(it);
    ompl_simple_setup_->setPlannerAllocator(
        boost::bind(spec_.planner_selector_(type), _1, name_ != getGroupName() ? name_ : "", spec_));
    ROS_INFO_NAMED(class_name_, "Planner configuration '%s' will use planner '%s'. Additional configuration parameters will be set when "
             "the planner is constructed.",
             type.c_str());
  }

  // call the setParams() after setup(), so we know what the params are
  ompl_simple_setup_->getSpaceInformation()->setup();
  ompl_simple_setup_->getSpaceInformation()->params().setParams(cfg, true);
  // call setup() again for possibly new param values
  ompl_simple_setup_->getSpaceInformation()->setup();
}

void moveit_ompl::ModelBasedPlanningContext::setPlanningVolume(const moveit_msgs::WorkspaceParameters &wparams)
{
  if (wparams.min_corner.x == wparams.max_corner.x && wparams.min_corner.x == 0.0 &&
      wparams.min_corner.y == wparams.max_corner.y && wparams.min_corner.y == 0.0 &&
      wparams.min_corner.z == wparams.max_corner.z && wparams.min_corner.z == 0.0)
    ROS_WARN_STREAM_NAMED(class_name_, "It looks like the planning volume was not specified.");

  // ROS_DEBUG_NAMED(class_name_, "Setting planning volume (affects SE2 & SE3 joints only) to x = [%f, %f], y = [%f, %f], z = [%f, %f]",
  //           wparams.min_corner.x, wparams.max_corner.x, wparams.min_corner.y, wparams.max_corner.y,
  //           wparams.min_corner.z, wparams.max_corner.z);

  spec_.state_space_->setPlanningVolume(wparams.min_corner.x, wparams.max_corner.x, wparams.min_corner.y,
                                        wparams.max_corner.y, wparams.min_corner.z, wparams.max_corner.z);
}

void moveit_ompl::ModelBasedPlanningContext::simplifySolution(double timeout)
{
  std::cout << "skipping simplification step 123 ==================================" << std::endl;
  ompl_simple_setup_->simplifySolution(timeout);
  last_simplify_time_ = ompl_simple_setup_->getLastSimplificationTime();
}

void moveit_ompl::ModelBasedPlanningContext::interpolateSolution()
{
  //std::cout << "skipping interpolation step ==================================" << std::endl;
  //return;

  if (ompl_simple_setup_->haveSolutionPath())
  {
    og::PathGeometric &path_geometric = ompl_simple_setup_->getSolutionPath();
    std::size_t prev_path_states = path_geometric.getStateCount();
    path_geometric.interpolate(std::max(
        (unsigned int)floor(0.5 + path_geometric.length() / max_solution_segment_length_), minimum_waypoint_count_));

    ROS_INFO_NAMED(class_name_, "Path states from interpolation increased from %d to %d", prev_path_states,
             path_geometric.getStateCount());
  }
}

void moveit_ompl::ModelBasedPlanningContext::convertPath(const ompl::geometric::PathGeometric &path_geometric,
                                                         robot_trajectory::RobotTrajectory &traj) const
{
  robot_state::RobotState ks = complete_initial_robot_state_;
  for (std::size_t i = 0; i < path_geometric.getStateCount(); ++i)
  {
    spec_.state_space_->copyToRobotState(ks, path_geometric.getState(i));
    traj.addSuffixWayPoint(ks, 0.0);
  }
}

bool moveit_ompl::ModelBasedPlanningContext::getSolutionPath(robot_trajectory::RobotTrajectory &traj) const
{
  traj.clear();
  if (!ompl_simple_setup_->haveSolutionPath())
    return false;
  convertPath(ompl_simple_setup_->getSolutionPath(), traj);
  return true;
}

void moveit_ompl::ModelBasedPlanningContext::setVerboseStateValidityChecks(bool flag)
{
  if (ompl_simple_setup_->getStateValidityChecker())
    static_cast<StateValidityChecker *>(ompl_simple_setup_->getStateValidityChecker().get())->setVerbose(flag);
}

ompl::base::GoalPtr moveit_ompl::ModelBasedPlanningContext::constructGoal()
{
  // ******************* set up the goal representation, based on goal constraints
  //ROS_ERROR_STREAM_NAMED(name_, "constructing goal with " << goal_constraints_.size() << " constraints");

  std::vector<ob::GoalPtr> goals;
  for (std::size_t i = 0; i < goal_constraints_.size(); ++i)
  {
    constraint_samplers::ConstraintSamplerPtr constraint_sampler;
    // Choose proper goal constraint sampler
    constraint_sampler = spec_.constraint_sampler_manager_->selectSampler(getPlanningScene(), getGroupName(),
                                                                          goal_constraints_[i]->getAllConstraints());

    if (constraint_sampler)
    {
      ob::GoalPtr g =
          ob::GoalPtr(new ConstrainedGoalSampler(this, goal_constraints_[i], constraint_sampler, visual_tools_));
      goals.push_back(g);
    }
  }

  if (goals.empty())
  {
    ROS_ERROR_STREAM_NAMED(class_name_, "Unable to construct goal representation");
    return ob::GoalPtr();
  }

  if (goals.size() == 1)
  {
    //ROS_INFO_STREAM_NAMED(name_, "Problem has only one goal");
    return goals[0];
  }

  // More than one goal
  ROS_INFO_STREAM_NAMED(name_, "Using goal sampleable region mux");
  return ompl::base::GoalPtr(new GoalSampleableRegionMux(goals));
}

void moveit_ompl::ModelBasedPlanningContext::setCompleteInitialState(
    const robot_state::RobotState &complete_initial_robot_state)
{
  complete_initial_robot_state_ = complete_initial_robot_state;
}

void moveit_ompl::ModelBasedPlanningContext::clear()
{
  ompl_simple_setup_->clear();
  ompl_simple_setup_->clearStartStates();
  ompl_simple_setup_->setGoal(ob::GoalPtr());
  ompl_simple_setup_->setStateValidityChecker(ob::StateValidityCheckerPtr());
  path_constraints_.reset();
  goal_constraints_.clear();
  getOMPLStateSpace()->setInterpolationFunction(InterpolationFunction());
}

bool moveit_ompl::ModelBasedPlanningContext::setPathConstraints(const moveit_msgs::Constraints &path_constraints,
                                                                moveit_msgs::MoveItErrorCodes *error)
{
  // ******************* set the path constraints to use
  path_constraints_.reset(new kinematic_constraints::KinematicConstraintSet(getRobotModel()));
  path_constraints_->add(path_constraints, getPlanningScene()->getTransforms());
  path_constraints_msg_ = path_constraints;

  return true;
}

bool moveit_ompl::ModelBasedPlanningContext::setGoalConstraints(
    const std::vector<moveit_msgs::Constraints> &goal_constraints, const moveit_msgs::Constraints &path_constraints,
    moveit_msgs::MoveItErrorCodes *error)
{
  // std::cout << "TEMP size: " << goal_constraints.size() << "\n " << goal_constraints[0] << std::endl;

  // ******************* check if the input is correct
  goal_constraints_.clear();
  for (std::size_t i = 0; i < goal_constraints.size(); ++i)
  {
    moveit_msgs::Constraints constr = kinematic_constraints::mergeConstraints(goal_constraints[i], path_constraints);
    kinematic_constraints::KinematicConstraintSetPtr kset(
        new kinematic_constraints::KinematicConstraintSet(getRobotModel()));
    kset->add(constr, getPlanningScene()->getTransforms());
    if (!kset->empty())
      goal_constraints_.push_back(kset);
  }

  if (goal_constraints_.empty())
  {
    ROS_WARN_STREAM_NAMED(class_name_, "No goal constraints specified. There is no problem to solve.");
    if (error)
      error->val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  ob::GoalPtr goal = constructGoal();
  ompl_simple_setup_->setGoal(goal);
  if (goal)
    return true;
  else
    return false;
}

bool moveit_ompl::ModelBasedPlanningContext::benchmark(double timeout, unsigned int count, const std::string &filename)
{
  ompl_benchmark_.clearPlanners();
  ompl_simple_setup_->setup();
  ompl_benchmark_.addPlanner(ompl_simple_setup_->getPlanner());
  ompl_benchmark_.setExperimentName(getRobotModel()->getName() + "_" + getGroupName() + "_" +
                                    getPlanningScene()->getName() + "_" + name_);

  ot::Benchmark::Request req;
  req.maxTime = timeout;
  req.runCount = count;
  req.displayProgress = true;
  req.saveConsoleOutput = false;
  ompl_benchmark_.benchmark(req);
  return filename.empty() ? ompl_benchmark_.saveResultsToFile() : ompl_benchmark_.saveResultsToFile(filename.c_str());
}

/*void moveit_ompl::ModelBasedPlanningContext::startSampling()
{
  std::cout << "ModelBasedPlanningContext::startSampling()" << std::endl;

  bool gls = ompl_simple_setup_->getGoal()->hasType(ob::GOAL_LAZY_SAMPLES);
  if (gls)
  {
    std::cout << "startSampling GoalLazySamples" << std::endl;
    static_cast<ob::GoalLazySamples *>(ompl_simple_setup_->getGoal().get())->startSampling();
  }
  else
    // we know this is a GoalSampleableMux by elimination
    static_cast<GoalSampleableRegionMux *>(ompl_simple_setup_->getGoal().get())->startSampling();
}

void moveit_ompl::ModelBasedPlanningContext::stopSampling()
{
  bool gls = ompl_simple_setup_->getGoal()->hasType(ob::GOAL_LAZY_SAMPLES);
  if (gls)
    static_cast<ob::GoalLazySamples *>(ompl_simple_setup_->getGoal().get())->stopSampling();
  else
    // we know this is a GoalSampleableMux by elimination
    static_cast<GoalSampleableRegionMux *>(ompl_simple_setup_->getGoal().get())->stopSampling();
}
*/

void moveit_ompl::ModelBasedPlanningContext::preSolve()
{
  // clear previously computed solutions
  ompl_simple_setup_->getProblemDefinition()->clearSolutionPaths();
  const ob::PlannerPtr planner = ompl_simple_setup_->getPlanner();
  if (planner)
    planner->clear();
  // startSampling();
  ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();
}

void moveit_ompl::ModelBasedPlanningContext::postSolve()
{
  //stopSampling();
  int v = ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getValidMotionCount();
  int iv = ompl_simple_setup_->getSpaceInformation()->getMotionValidator()->getInvalidMotionCount();
  ROS_DEBUG_NAMED(class_name_, "There were %d valid motions and %d invalid motions.", v, iv);

  if (ompl_simple_setup_->getProblemDefinition()->hasApproximateSolution())
    ROS_WARN_STREAM_NAMED(class_name_, "Computed solution is approximate");
}

bool moveit_ompl::ModelBasedPlanningContext::solve(planning_interface::MotionPlanResponse &res)
{
  if (solve(request_.allowed_planning_time, request_.num_planning_attempts))
  {
    double ptime = getLastPlanTime();

    bool disableSimplifyAndInterpolate = true;

    if (disableSimplifyAndInterpolate)
    {
      ROS_INFO_STREAM_NAMED(class_name_, "Disabled simplifySolution and interpolateSolution in model_based_planning_context.cpp");
    }
    else
    {
      if (simplify_solutions_ && ptime < request_.allowed_planning_time)
      {
        simplifySolution(request_.allowed_planning_time - ptime);
        ptime += getLastSimplifyTime();
      }
    }

    ROS_INFO_STREAM_NAMED(class_name_, "ModelBasedPlanningContext::solve() skipping interpolating solution");
    //ROS_INFO_STREAM_NAMED(class_name_, "ModelBasedPlanningContext::solve() Interpolating solution");
    //interpolateSolution();

    // fill the response
    ROS_INFO_NAMED(class_name_, "Returning successful solution with %lu states",
             getOMPLSimpleSetup()->getSolutionPath().getStateCount());

    res.trajectory_.reset(new robot_trajectory::RobotTrajectory(getRobotModel(), getGroupName()));
    getSolutionPath(*res.trajectory_);
    res.planning_time_ = ptime;
    return true;
  }
  else
  {
    ROS_INFO_STREAM_NAMED(class_name_, "Unable to solve the planning problem");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }
}

bool moveit_ompl::ModelBasedPlanningContext::solve(planning_interface::MotionPlanDetailedResponse &res)
{
  ROS_ERROR_STREAM_NAMED("temp", "Using detailed solve function");
  std::cout << "ModelBasedPlanningContext::solve - with detail " << std::endl;
  if (solve(request_.allowed_planning_time, request_.num_planning_attempts))
  {
    res.trajectory_.reserve(3);

    // add info about planned solution
    double ptime = getLastPlanTime();
    res.processing_time_.push_back(ptime);
    res.description_.push_back("plan");
    res.trajectory_.resize(res.trajectory_.size() + 1);
    res.trajectory_.back().reset(new robot_trajectory::RobotTrajectory(getRobotModel(), getGroupName()));
    getSolutionPath(*res.trajectory_.back());

    // simplify solution if time remains
    if (simplify_solutions_ && ptime < request_.allowed_planning_time)
    {
      //simplifySolution(request_.allowed_planning_time - ptime);
      res.processing_time_.push_back(getLastSimplifyTime());
      res.description_.push_back("simplify");
      res.trajectory_.resize(res.trajectory_.size() + 1);
      res.trajectory_.back().reset(new robot_trajectory::RobotTrajectory(getRobotModel(), getGroupName()));
      getSolutionPath(*res.trajectory_.back());
    }

    ompl::time::point start_interpolate = ompl::time::now();
    interpolateSolution();
    res.processing_time_.push_back(ompl::time::seconds(ompl::time::now() - start_interpolate));
    res.description_.push_back("interpolate");
    res.trajectory_.resize(res.trajectory_.size() + 1);
    res.trajectory_.back().reset(new robot_trajectory::RobotTrajectory(getRobotModel(), getGroupName()));
    getSolutionPath(*res.trajectory_.back());

    // fill the response
    ROS_DEBUG_NAMED(class_name_, "Returning successful solution with %lu states",
              getOMPLSimpleSetup()->getSolutionPath().getStateCount());
    return true;
  }
  else
  {
    ROS_INFO_STREAM_NAMED(class_name_, "Unable to solve the planning problem");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }
}

bool moveit_ompl::ModelBasedPlanningContext::solve(double timeout, unsigned int count)
{
  // Start goal sampling thread
  preSolve();

  bool result = false;
  if (count <= 1 || request_.use_experience)
  {
    result = solveOnce(timeout, count);
  }
  else
  {
    result = solveParallel(timeout, count);
  }

  postSolve();

  return result;
}

bool moveit_ompl::ModelBasedPlanningContext::solveOnce(double timeout, unsigned int count)
{
  bool result = false;

  ROS_DEBUG_STREAM_NAMED(class_name_, "Solving the planning problem once...");
  ompl::time::point start = ompl::time::now();

  ob::PlannerTerminationCondition ptc =
      ob::timedPlannerTerminationCondition(timeout - ompl::time::seconds(ompl::time::now() - start));
  registerTerminationCondition(ptc);

  // OMPL Simple Setup
  result = ompl_simple_setup_->solve(ptc) == ompl::base::PlannerStatus::EXACT_SOLUTION;

  last_plan_time_ = ompl_simple_setup_->getLastPlanComputationTime();
  unregisterTerminationCondition();

  return result;
}

bool moveit_ompl::ModelBasedPlanningContext::solveParallel(double timeout, unsigned int count)
{
  bool result = false;
  ompl::time::point start = ompl::time::now();

  ROS_DEBUG_NAMED(class_name_, "Solving the planning problem %u times...", count);
  ompl_parallel_plan_.clearHybridizationPaths();
  if (count <= max_planning_threads_)
  {
    ompl_parallel_plan_.clearPlanners();
    if (ompl_simple_setup_->getPlannerAllocator())
      for (unsigned int i = 0; i < count; ++i)
        ompl_parallel_plan_.addPlannerAllocator(ompl_simple_setup_->getPlannerAllocator());
    else
      for (unsigned int i = 0; i < count; ++i)
        ompl_parallel_plan_.addPlanner(ompl::tools::SelfConfig::getDefaultPlanner(ompl_simple_setup_->getGoal()));

    ob::PlannerTerminationCondition ptc =
        ob::timedPlannerTerminationCondition(timeout - ompl::time::seconds(ompl::time::now() - start));
    registerTerminationCondition(ptc);
    result = ompl_parallel_plan_.solve(ptc, 1, count, true) == ompl::base::PlannerStatus::EXACT_SOLUTION;
    last_plan_time_ = ompl::time::seconds(ompl::time::now() - start);
    unregisterTerminationCondition();
  }
  else
  {
    ob::PlannerTerminationCondition ptc =
        ob::timedPlannerTerminationCondition(timeout - ompl::time::seconds(ompl::time::now() - start));
    registerTerminationCondition(ptc);
    int n = count / max_planning_threads_;
    result = true;
    for (int i = 0; i < n && ptc() == false; ++i)
    {
      ompl_parallel_plan_.clearPlanners();
      if (ompl_simple_setup_->getPlannerAllocator())
        for (unsigned int i = 0; i < max_planning_threads_; ++i)
          ompl_parallel_plan_.addPlannerAllocator(ompl_simple_setup_->getPlannerAllocator());
      else
        for (unsigned int i = 0; i < max_planning_threads_; ++i)
          ompl_parallel_plan_.addPlanner(ompl::tools::SelfConfig::getDefaultPlanner(ompl_simple_setup_->getGoal()));
      bool r = ompl_parallel_plan_.solve(ptc, 1, count, true) == ompl::base::PlannerStatus::EXACT_SOLUTION;
      result = result && r;
    }
    n = count % max_planning_threads_;
    if (n && ptc() == false)
    {
      ompl_parallel_plan_.clearPlanners();
      if (ompl_simple_setup_->getPlannerAllocator())
        for (int i = 0; i < n; ++i)
          ompl_parallel_plan_.addPlannerAllocator(ompl_simple_setup_->getPlannerAllocator());
      else
        for (int i = 0; i < n; ++i)
          ompl_parallel_plan_.addPlanner(ompl::tools::SelfConfig::getDefaultPlanner(ompl_simple_setup_->getGoal()));
      bool r = ompl_parallel_plan_.solve(ptc, 1, count, true) == ompl::base::PlannerStatus::EXACT_SOLUTION;
      result = result && r;
    }
    last_plan_time_ = ompl::time::seconds(ompl::time::now() - start);
    unregisterTerminationCondition();
  }

  return result;
}

void moveit_ompl::ModelBasedPlanningContext::registerTerminationCondition(const ob::PlannerTerminationCondition &ptc)
{
  boost::mutex::scoped_lock slock(ptc_lock_);
  ptc_ = &ptc;
}

void moveit_ompl::ModelBasedPlanningContext::unregisterTerminationCondition()
{
  boost::mutex::scoped_lock slock(ptc_lock_);
  ptc_ = NULL;
}

bool moveit_ompl::ModelBasedPlanningContext::terminate()
{
  boost::mutex::scoped_lock slock(ptc_lock_);
  if (ptc_)
    ptc_->terminate();
  return true;
}
