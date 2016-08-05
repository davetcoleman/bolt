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

#include <ros/ros.h>

#include <moveit/ompl/planning_context_manager.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>

// OMPL
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>

#include <moveit/ompl/joint_space/model_based_state_space.h>

#include <ompl/tools/thunder/Thunder.h>
#include <ompl/tools/bolt/Bolt.h>

// C++
#include <algorithm>
#include <set>

// Boost
#include <boost/filesystem.hpp>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace mo = moveit_ompl;

namespace moveit_ompl
{
class PlanningContextManager::LastPlanningContext
{
public:
  ModelBasedPlanningContextPtr getContext()
  {
    boost::mutex::scoped_lock slock(lock_);
    return last_planning_context_solve_;
  }

  void setContext(const ModelBasedPlanningContextPtr &context)
  {
    boost::mutex::scoped_lock slock(lock_);
    last_planning_context_solve_ = context;
  }

  void clear()
  {
    boost::mutex::scoped_lock slock(lock_);
    last_planning_context_solve_.reset();
  }

private:
  /* The planning group for which solve() was called last */
  ModelBasedPlanningContextPtr last_planning_context_solve_;
  boost::mutex lock_;
};

struct PlanningContextManager::CachedContexts
{
  std::map<std::pair<std::string, std::string>, std::vector<ModelBasedPlanningContextPtr> > contexts_;
  boost::mutex lock_;
};

}  // namespace moveit_ompl

mo::PlanningContextManager::PlanningContextManager(const robot_model::RobotModelConstPtr &robot_model,
                                                   const constraint_samplers::ConstraintSamplerManagerPtr &csm)
  : name_("planning_context_manager")
  , robot_model_(robot_model)
  , constraint_sampler_manager_(csm)
  , max_goal_samples_(10)
  , max_state_sampling_attempts_(1000)  // 4
  , max_goal_sampling_attempts_(100)    // 1000),
  , max_planning_threads_(4)
  , max_solution_segment_length_(0.0)
  , minimum_waypoint_count_(5)
{
  last_planning_context_.reset(new LastPlanningContext());
  cached_contexts_.reset(new CachedContexts());
}

mo::PlanningContextManager::~PlanningContextManager()
{
}

namespace
{
using namespace moveit_ompl;

template <typename T>
static ompl::base::PlannerPtr allocatePlanner(const ob::SpaceInformationPtr &si, const std::string &new_name,
                                              const ModelBasedPlanningContextSpecification &spec)
{
  ompl::base::PlannerPtr planner(new T(si));
  if (!new_name.empty())
    planner->setName(new_name);
  planner->params().setParams(spec.config_, true);
  planner->setup();
  return planner;
}
}

mo::ConfiguredPlannerAllocator mo::PlanningContextManager::plannerSelector(const std::string &planner) const
{
  std::map<std::string, ConfiguredPlannerAllocator>::const_iterator it = known_planners_.find(planner);
  if (it != known_planners_.end())
    return it->second;
  else
  {
    ROS_ERROR("Unknown planner: '%s'", planner.c_str());
    return ConfiguredPlannerAllocator();
  }
}

mo::ConfiguredPlannerSelector mo::PlanningContextManager::getPlannerSelector() const
{
  return boost::bind(&PlanningContextManager::plannerSelector, this, _1);
}

void mo::PlanningContextManager::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pconfig)
{
  planner_configs_ = pconfig;
}

void mo::PlanningContextManager::createPlanningContext(ModelBasedPlanningContextPtr &context,
                                                       const planning_interface::PlannerConfigurationSettings &config,
                                                       const moveit_msgs::MotionPlanRequest &req,
                                                       const mo::ModelBasedStateSpaceFactoryPtr &factory,
                                                       moveit_visual_tools::MoveItVisualToolsPtr visual_tools) const
{
  ROS_INFO_STREAM_NAMED(name_, "Creating new planning context");

  ModelBasedStateSpaceSpecification space_spec(robot_model_, config.group);
  ModelBasedPlanningContextSpecification context_spec;

  context_spec.config_ = config.config;
  context_spec.planner_selector_ = getPlannerSelector();
  context_spec.constraint_sampler_manager_ = constraint_sampler_manager_;
  //context_spec.state_space_ = factory->getNewStateSpace(space_spec, visual_tools);

  enum SimpleSetupType
  {
    REGULAR,
    LIGHTNING,
    THUNDER,
    BOLT
  };

  // Choose simple setup type
  SimpleSetupType chosen_type;

  // Choose the correct simple setup type to load
  if (req.use_experience)
  {
    if (req.num_planning_attempts > 1)
      ROS_ERROR_STREAM_NAMED(name_, "Number of planning attempts is greater than one, which is not allowed for "
                                    "experienced-based planning. Reducing to 1");

    if (req.experience_method == "lightning")
      chosen_type = LIGHTNING;
    else if (req.experience_method == "thunder")
      chosen_type = THUNDER;
    else if (req.experience_method == "bolt")
      chosen_type = BOLT;
    else
    {
      ROS_ERROR_STREAM_NAMED(name_, "Invalid experience method specified " << req.experience_method);
      chosen_type = REGULAR;
    }
  }
  else
  {
    chosen_type = REGULAR;
  }

  // Load correct OMPL setup type
  switch (chosen_type)
  {
    case REGULAR:
    {
      ROS_DEBUG("planning_context_manager: Using regular framework for planning");
      context_spec.ompl_simple_setup_.reset(new ompl::geometric::SimpleSetup(context_spec.state_space_));
    }
    break;
    case LIGHTNING:
    {
      ROS_DEBUG("planning_context_manager: Using LIGHTNING Framework for planning");
      context_spec.ompl_simple_setup_.reset(new ompl::tools::Lightning(context_spec.state_space_));

      // Load the experience database
      ompl::tools::ExperienceSetup &experience_handle =
          static_cast<ompl::tools::ExperienceSetup &>(*context_spec.ompl_simple_setup_);

      // Choose the file location
      std::string file_path;
      if (!getFilePath(file_path,
                       "lightning_" + context_spec.state_space_->getJointModelGroup()->getName() + "_database",
                       "ros/ompl_storage"))
      {
        ROS_ERROR_STREAM_NAMED(name_, "Unable to find file path for experience framework");
      }

      experience_handle.setFilePath(file_path);

      if (!req.use_experience)
      {
        ROS_WARN("Lightning Framework is loaded but recall is disabled");
        experience_handle.enablePlanningFromRecall(false);
      }
    }
    break;
    case THUNDER:
    {
      ROS_DEBUG("planning_context_manager: Using THUNDER Framework for planning");
      context_spec.ompl_simple_setup_.reset(new ompl::tools::Thunder(context_spec.state_space_));

      // Load the experience database
      ompl::tools::Thunder &thunder_handle = static_cast<ompl::tools::Thunder &>(*context_spec.ompl_simple_setup_);

      // Choose the file location
      std::string file_path;
      if (!getFilePath(file_path, "thunder_" + context_spec.state_space_->getJointModelGroup()->getName() + "_database",
                       "ros/ompl_storage"))
      {
        ROS_ERROR_STREAM_NAMED(name_, "Unable to find file path for experience framework");
      }

      thunder_handle.setFilePath(file_path);

      // Set other parameters
      ros::NodeHandle nh("~");

      bool use_scratch;
      rosparam_shortcuts::get(name_, nh, "moveit_ompl/use_scratch", use_scratch);
      if (!use_scratch)
      {
        ROS_INFO_STREAM_NAMED(name_, "Planning from scratch disabled via rosparam server");
        thunder_handle.enablePlanningFromScratch(false);
      }

      bool saving_enabled;
      rosparam_shortcuts::get(name_, nh, "moveit_ompl/saving_enabled", saving_enabled);
      if (!saving_enabled)
      {
        ROS_INFO_STREAM_NAMED(name_, "Saving database disabled via rosparam server");
        thunder_handle.getExperienceDB()->setSavingEnabled(false);
      }

      bool use_experience;
      rosparam_shortcuts::get(name_, nh, "moveit_ompl/use_experience", use_experience);
      if (!use_experience)
      {
        ROS_WARN("Thunder Framework is loaded but planning from recall has been disabled via rosparam server by "
                 "user");
        thunder_handle.enablePlanningFromRecall(false);
      }

      double sparse_delta_fraction;
      rosparam_shortcuts::get(name_, nh, "moveit_ompl/sparse_delta_fraction", sparse_delta_fraction);
      // ROS_ERROR_STREAM_NAMED(name_,"Setting sparse delta fraction to " << sparse_delta_fraction);
      // thunder_handle.getExperienceDB()->getSPARSdb()->setSparseDeltaFraction( sparse_delta_fraction );
    }
    break;
    case BOLT:
    {
      ROS_DEBUG("planning_context_manager: Using BOLT Framework for planning");
      context_spec.ompl_simple_setup_.reset(new ompl::tools::Bolt(context_spec.state_space_));

      // Load the experience database
      ompl::tools::Bolt &bolt_handle = static_cast<ompl::tools::Bolt &>(*context_spec.ompl_simple_setup_);

      // Choose the file location
      std::string file_path;
      if (!getFilePath(file_path, "bolt_" + context_spec.state_space_->getJointModelGroup()->getName() + "_database",
                       "ros/ompl_storage"))
      {
        ROS_ERROR_STREAM_NAMED(name_, "Unable to find file path for experience framework");
      }

      bolt_handle.setFilePath(file_path);

      // Set other parameters
      ros::NodeHandle nh("~");

      bool saving_enabled;
      rosparam_shortcuts::get(name_, nh, "moveit_ompl/saving_enabled", saving_enabled);
      if (!saving_enabled)
      {
        ROS_INFO_STREAM_NAMED(name_, "Saving database disabled via rosparam server");
        bolt_handle.getExperienceDB()->setSavingEnabled(false);
      }
    }
    break;
    default:
      ROS_ERROR("planning_context_manager: No simple setup type found");
  }

  ROS_DEBUG("Creating new planning context");
  context.reset(new ModelBasedPlanningContext(config.name, context_spec, visual_tools));
  context->useStateValidityCache(true);

  // Add new context to cache
  {
    boost::mutex::scoped_lock slock(cached_contexts_->lock_);
    //cached_contexts_->contexts_[std::make_pair(config.name, factory->getType())].push_back(context);
  }
}

mo::ModelBasedPlanningContextPtr mo::PlanningContextManager::getPlanningContext(
    const planning_interface::PlannerConfigurationSettings &config,
    const moveit_msgs::MotionPlanRequest &req,
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools) const
{
  // Check for a cached planning context
  ModelBasedPlanningContextPtr context;

  {
    boost::mutex::scoped_lock slock(cached_contexts_->lock_);
    std::map<std::pair<std::string, std::string>, std::vector<ModelBasedPlanningContextPtr> >::const_iterator cc =
        cached_contexts_->contexts_.find(std::make_pair(config.name, factory->getType()));

    // Loop through the cached contextes
    if (cc != cached_contexts_->contexts_.end())
    {
      for (std::size_t i = 0; i < cc->second.size(); ++i)
      {
        // if (cc->second[i].unique()) // check if the context is being shared by anything else
        {
          ROS_DEBUG("Reusing cached planning context");
          context = cc->second[i];
          break;
        }
      }
    }
  }

  // Create a new planning context
  if (!context)
  {
    createPlanningContext(context, config, req, factory, visual_tools);
  }

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

  context->setSpecificationConfig(config.config);

  last_planning_context_->setContext(context);
  return context;
}

bool mo::PlanningContextManager::getFilePath(std::string &file_path, const std::string &database_name,
                                             const std::string &database_directory) const

{
  namespace fs = boost::filesystem;

  // Check that the directory exists, if not, create it
  fs::path rootPath;
  if (!std::string(getenv("HOME")).empty())
    rootPath = fs::path(getenv("HOME"));  // Support Linux/Mac
  else if (!std::string(getenv("HOMEPATH")).empty())
    rootPath = fs::path(getenv("HOMEPATH"));  // Support Windows
  else
  {
    ROS_WARN("Unable to find a home path for this computer");
    rootPath = fs::path("");
  }

  rootPath = rootPath / fs::path(database_directory);

  boost::system::error_code returnedError;
  fs::create_directories(rootPath, returnedError);

  if (returnedError)
  {
    // did not successfully create directories
    ROS_ERROR("Unable to create directory %s", database_directory.c_str());
    return false;
  }

  // directories successfully created, append the group name as the file name
  rootPath = rootPath / fs::path(database_name + ".ompl");
  file_path = rootPath.string();
  ROS_INFO_STREAM_NAMED(name_, "Setting database to " << file_path);

  return true;
}

const mo::ModelBasedStateSpaceFactoryPtr &mo::PlanningContextManager::getStateSpaceFactory(
    const std::string &group, const moveit_msgs::MotionPlanRequest &req) const
{
  // find the problem representation to use
  std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator best = state_space_factories_.end();
  int prev_priority = -1;
  for (std::map<std::string, ModelBasedStateSpaceFactoryPtr>::const_iterator it = state_space_factories_.begin();
       it != state_space_factories_.end(); ++it)
  {
    int priority = it->second->canRepresentProblem(group, req, robot_model_);
    if (priority > 0)
      if (best == state_space_factories_.end() || priority > prev_priority)
      {
        best = it;
        prev_priority = priority;
      }
  }

  if (best == state_space_factories_.end())
  {
    ROS_ERROR("There are no known state spaces that can represent the given planning problem");
    static const ModelBasedStateSpaceFactoryPtr empty;
    return empty;
  }
  else
  {
    ROS_DEBUG("Using '%s' parameterization for solving problem", best->first.c_str());
    return best->second;
  }
}

mo::ModelBasedPlanningContextPtr mo::PlanningContextManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr &planning_scene, const moveit_msgs::MotionPlanRequest &req,
    moveit_msgs::MoveItErrorCodes &error_code, moveit_visual_tools::MoveItVisualToolsPtr visual_tools) const
{
  // Error check
  if (req.group_name.empty())
  {
    ROS_ERROR("No group specified to plan for");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return ModelBasedPlanningContextPtr();
  }

  // Set default error value
  error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;

  // Error check
  if (!planning_scene)
  {
    ROS_ERROR("No planning scene supplied as input");
    return ModelBasedPlanningContextPtr();
  }

  // Identify the correct planning configuration
  planning_interface::PlannerConfigurationMap::const_iterator pc = planner_configs_.end();

  // User has requested specific planner, attempt to find it
  if (!req.planner_id.empty())
  {
    pc = planner_configs_.find(req.planner_id.find(req.group_name) == std::string::npos ?
                                   req.group_name + "[" + req.planner_id + "]" :
                                   req.planner_id);
    if (pc == planner_configs_.end())
      ROS_WARN("Cannot find planning configuration for group '%s' using planner '%s'. Will use defaults instead.",
               req.group_name.c_str(), req.planner_id.c_str());
  }

  // Has a specific planner been found yet?
  if (pc == planner_configs_.end())
  {
    // Just choose first planner config that works for our group
    pc = planner_configs_.find(req.group_name);
    if (pc == planner_configs_.end())
    {
      ROS_ERROR("Cannot find planning configuration for group '%s'", req.group_name.c_str());
      return ModelBasedPlanningContextPtr();
    }
  }

  // Choose best planning context
  ModelBasedPlanningContextPtr context = getPlanningContext(
      pc->second, boost::bind(&PlanningContextManager::getStateSpaceFactory, this, _1, req), req, visual_tools);

  // Error check
  if (!context)
  {
    ROS_ERROR("Cannot find planning context for group '%s'", req.group_name.c_str());
    return ModelBasedPlanningContextPtr();
  }

  context->clear();
  robot_state::RobotStatePtr start_state = planning_scene->getCurrentStateUpdated(req.start_state);

  // Setup the context
  context->setPlanningScene(planning_scene);
  context->setMotionPlanRequest(req);
  context->setCompleteInitialState(*start_state);

  context->setPlanningVolume(req.workspace_parameters);
  if (!context->setPathConstraints(req.path_constraints, &error_code))
    return ModelBasedPlanningContextPtr();

  // Create the goal
  if (!context->setGoalConstraints(req.goal_constraints, req.path_constraints, &error_code))
    return ModelBasedPlanningContextPtr();

  try
  {
    context->configure();
    ROS_DEBUG("%s: New planning context is set.", context->getName().c_str());
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  }
  catch (ompl::Exception &ex)
  {
    ROS_ERROR("OMPL encountered an error: %s", ex.what());
    context.reset();
  }

  return context;
}

mo::ModelBasedPlanningContextPtr mo::PlanningContextManager::getLastPlanningContext() const
{
  return last_planning_context_->getContext();
}
