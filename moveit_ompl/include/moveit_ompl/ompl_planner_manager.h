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

/* Author: Ioan Sucan, Dave Coleman */

#ifndef MOVEIT_OMPL_OMPL_PLANNER_MANAGER_
#define MOVEIT_OMPL_OMPL_PLANNER_MANAGER

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/constraint_sampler_manager_loader/constraint_sampler_manager_loader.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <ros/ros.h>
#include <class_loader/class_loader.h>
#include <moveit/macros/class_forward.h>

/** \brief The MoveIt interface to OMPL */
namespace moveit_ompl
{
MOVEIT_CLASS_FORWARD(OMPLPlannerManager);

/** @class OMPLPlannerManager
 *  This class defines the interface to the motion planners in OMPL*/
class OMPLPlannerManager : public planning_interface::PlannerManager
{
public:
  /** \brief Initialize OMPL-based planning for a particular robot model. ROS configuration is read from the specified
   * NodeHandle */
  OMPLPlannerManager();

  /// Initialize a planner. This function will be called after the construction of the plugin, before any other call is
  /// made.
  /// It is assumed that motion plans will be computed for the robot described by \e model and that any exposed ROS
  /// functionality
  /// or required ROS parameters are namespaced by \e ns
  virtual bool initialize(const robot_model::RobotModelConstPtr &robot_model, const std::string &ns);

  /// Get \brief a short string that identifies the planning interface
  virtual std::string getDescription() const
  {
    return "OMPL";
  }

  /// \brief Get the names of the known planning algorithms (values that can be filled as planner_id in the planning
  /// request)
  virtual void getPlanningAlgorithms(std::vector<std::string> &algs) const;

  /// \brief Determine whether this plugin instance is able to represent this planning request
  virtual bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const;

  /** @brief Specify configurations for the planners.
      @param pconfig Configurations for the different planners */
  virtual void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &pconfig);

  /** @brief Get the configurations for the planners that are already loaded
      @param pconfig Configurations for the different planners */
  const planning_interface::PlannerConfigurationMap &getPlannerConfigurations() const
  {
    //return context_manager_->getPlannerConfigurations();
  }

  /// \brief Calls the function above but ignores the error_code
  virtual planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr &planning_scene,
                     const planning_interface::MotionPlanRequest &req, moveit_msgs::MoveItErrorCodes &error_code) const;

protected:

  /** @brief Configure the planners*/
  void loadPlannerConfigurations();

  /** \brief Configure the OMPL planning context for a new planning request */
  ModelBasedPlanningContextPtr prepareForSolve(const planning_interface::MotionPlanRequest &req,
                                               const planning_scene::PlanningSceneConstPtr &planning_scene,
                                               moveit_msgs::MoveItErrorCodes *error_code, unsigned int *attempts,
                                               double *timeout) const;

  ros::NodeHandle nh_;  /// The ROS node handle

  /** \brief The kinematic model for which motion plans are computed */
  robot_model::RobotModelConstPtr robot_model_;

  // Create one planning context and stick to it the whole time (you can't change groups)
  ModelBasedPlanningContextPtr planning_context_;

  constraint_samplers::ConstraintSamplerManagerPtr constraint_sampler_manager_;

  // Visualize in Rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

private:
  constraint_sampler_manager_loader::ConstraintSamplerManagerLoaderPtr constraint_sampler_manager_loader_;
};

}  // ompl

CLASS_LOADER_REGISTER_CLASS(moveit_ompl::OMPLPlannerManager, planning_interface::PlannerManager);

#endif
