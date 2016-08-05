/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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
   Desc:   Base class for using MoveIt! in C++
*/

#ifndef CURIE_DEMOS_MOVEIT_BASE_H
#define CURIE_DEMOS_MOVEIT_BASE_H

// C++
#include <string>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/macros/console_colors.h>

// moveit_boilerplate
#include <moveit_boilerplate/namespaces.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace curie_demos
{
static const std::string ROBOT_DESCRIPTION = "robot_description";

class MoveItBase
{
public:
  /**
   * \brief Constructor
   */
  MoveItBase();

  /**
   * \brief Desctructor
   */
  virtual ~MoveItBase()
  {
  }

  /**
   * \brief Initialize all of the MoveIt! functionality
   */
  virtual bool init(ros::NodeHandle &nh);

  /**
   * \brief Connect to the MoveIt! planning scene messages
   *        Note: this is called within the init() function
   */
  bool loadPlanningSceneMonitor(const std::string &joint_state_topic);

  /** \brief Output to console the current state of the robot's joint limits */
  bool showJointLimits(JointModelGroup *jmg);

  /**
   * \brief Use the planning scene to get the robot's current state
   */
  moveit::core::RobotStatePtr getCurrentState();

  /**
   * \brief Get the published tf pose from two frames
   * \param from_frame e.g. 'world'
   * \param to_frame e.g. 'thing'
   * \param pose - the returned valie
   * \return false on missing transform, may just need to wait a little longer and retry
   */
  bool getTFTransform(const std::string &from_frame, const std::string &to_frame, Eigen::Affine3d &pose);

  /** \brief Getter for robot model */
  const robot_model::RobotModelPtr getRobotModel() const
  {
    return robot_model_;
  }

  /** \brief Getting for planning scene monitor */
  psm::PlanningSceneMonitorPtr getPlanningSceneMonitor()
  {
    return planning_scene_monitor_;
  }

protected:
  // A shared node handle
  ros::NodeHandle nh_;

  // Short name of this class
  std::string name_;

  // Settings
  std::string planning_scene_topic_;

  // Transform
  boost::shared_ptr<tf::TransformListener> tf_;

  // Core MoveIt components
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  psm::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Allocated memory for robot state
  moveit::core::RobotStatePtr current_state_;

  // Desired planning group to work with
  JointModelGroup *jmg_;
};  // end class

}  // namespace curie_demos

#endif  // CURIE_DEMOS_MOVEIT_BASE_H
