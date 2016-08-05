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

// C++
#include <string>
#include <vector>

// this package
#include <curie_demos/moveit_base.h>

// Conversions
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

namespace curie_demos
{
MoveItBase::MoveItBase() : name_("moveit_base")
{
}

bool MoveItBase::init(ros::NodeHandle& nh)
{
  nh_ = nh;
  std::string joint_state_topic;
  std::string joint_model_group;
  std::string execution_command_mode;

  // Load rosparams
  ros::NodeHandle rpnh(nh_, name_);
  int error = 0;
  error += !rosparam_shortcuts::get(name_, rpnh, "joint_state_topic", joint_state_topic);
  error += !rosparam_shortcuts::get(name_, rpnh, "joint_model_group", joint_model_group);
  error += !rosparam_shortcuts::get(name_, rpnh, "planning_scene_topic", planning_scene_topic_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // Load the loader
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));

  // Load the robot model
  robot_model_ = robot_model_loader_->getModel();  // Get a shared pointer to the robot

  // Choose planning group
  jmg_ = robot_model_->getJointModelGroup(joint_model_group);

  // Create the planning scene
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

  // Load planning scene monitor
  if (!loadPlanningSceneMonitor(joint_state_topic))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to load planning scene monitor");
  }

  // Create initial robot state
  {
    psm::LockedPlanningSceneRO scene(planning_scene_monitor_);  // Lock planning scene
    current_state_.reset(new moveit::core::RobotState(scene->getCurrentState()));
  }  // end scoped pointer of locked planning scene

  ROS_INFO_STREAM_NAMED(name_, "MoveItBase Ready.");

  return true;
}

bool MoveItBase::loadPlanningSceneMonitor(const std::string& joint_state_topic)
{
  // Create tf transformer
  tf_.reset(new tf::TransformListener(nh_));
  // TODO(davetcoleman): remove these lines, only an attempt to fix loadPlanningSceneMonitor bug
  ros::spinOnce();

  // Allows us to sycronize to Rviz and also publish collision objects to ourselves
  ROS_DEBUG_STREAM_NAMED(name_, "Loading Planning Scene Monitor");
  static const std::string PLANNING_SCENE_MONITOR_NAME = "MoveItBasePlanningScene";
  planning_scene_monitor_.reset(
      new psm::PlanningSceneMonitor(planning_scene_, robot_model_loader_, tf_, PLANNING_SCENE_MONITOR_NAME));
  ros::spinOnce();

  if (planning_scene_monitor_->getPlanningScene())
  {
    // Optional monitors to start:
    // planning_scene_monitor_->startStateMonitor(joint_state_topic, "");
    planning_scene_monitor_->getPlanningScene()->setName("curie_scene");
    planning_scene_monitor_->startPublishingPlanningScene(psm::PlanningSceneMonitor::UPDATE_NONE,
                                                          planning_scene_topic_);
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(name_, "Planning scene not configured");
    return false;
  }
  // ros::spinOnce();
  // ros::Duration(0.5).sleep();  // when at 0.1, i believe sometimes vjoint not properly loaded

  // Wait for complete state to be recieved
  bool wait_for_complete_state = false;
  // Break early
  if (!wait_for_complete_state)
    return true;

  std::vector<std::string> missing_joints;
  std::size_t counter = 0;
  while (!planning_scene_monitor_->getStateMonitor()->haveCompleteState() && ros::ok())
  {
    ROS_INFO_STREAM_THROTTLE_NAMED(1, name_, "Waiting for complete state from topic " << joint_state_topic);
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    // Show unpublished joints
    if (counter % 10 == 0)
    {
      planning_scene_monitor_->getStateMonitor()->haveCompleteState(missing_joints);
      for (std::size_t i = 0; i < missing_joints.size(); ++i)
        ROS_WARN_STREAM_NAMED(name_, "Unpublished joints: " << missing_joints[i]);
    }
    counter++;
  }
  ros::spinOnce();

  return true;
}

bool MoveItBase::showJointLimits(JointModelGroup* jmg)
{
  const std::vector<const moveit::core::JointModel*>& joints = jmg->getActiveJointModels();

  std::cout << std::endl;

  // Loop through joints
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    // Assume all joints have only one variable
    if (joints[i]->getVariableCount() > 1)
    {
      ROS_ERROR_STREAM_NAMED(name_, "Unable to handle joints with more than one var");
      return false;
    }
    getCurrentState();
    double current_value = current_state_->getVariablePosition(joints[i]->getName());

    // check if bad position
    bool out_of_bounds = !current_state_->satisfiesBounds(joints[i]);

    const moveit::core::VariableBounds& bound = joints[i]->getVariableBounds()[0];

    if (out_of_bounds)
      std::cout << MOVEIT_CONSOLE_COLOR_RED;

    std::cout << "   " << std::fixed << std::setprecision(5) << bound.min_position_ << "\t";
    double delta = bound.max_position_ - bound.min_position_;
    // std::cout << "delta: " << delta << " ";
    double step = delta / 20.0;

    bool marker_shown = false;
    for (double value = bound.min_position_; value < bound.max_position_; value += step)
    {
      // show marker of current value
      if (!marker_shown && current_value < value)
      {
        std::cout << "|";
        marker_shown = true;
      }
      else
        std::cout << "-";
    }
    // show max position
    std::cout << " \t" << std::fixed << std::setprecision(5) << bound.max_position_ << "  \t" << joints[i]->getName()
              << " current: " << std::fixed << std::setprecision(5) << current_value << std::endl;

    if (out_of_bounds)
      std::cout << MOVEIT_CONSOLE_COLOR_RESET;
  }

  return true;
}

moveit::core::RobotStatePtr MoveItBase::getCurrentState()
{
  // Get the real current state
  psm::LockedPlanningSceneRO scene(planning_scene_monitor_);  // Lock planning scene
  (*current_state_) = scene->getCurrentState();
  return current_state_;
}

bool MoveItBase::getTFTransform(const std::string& from_frame, const std::string& to_frame, Eigen::Affine3d& pose)
{
  tf::StampedTransform tf_transform;

  while (true)
  {
    try
    {
      tf_->lookupTransform(from_frame, to_frame, ros::Time(0), tf_transform);
      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_INFO_THROTTLE_NAMED(1, name_, "Waiting on TF: %s", ex.what());
    }
  }

  // Convert to eigen
  tf::transformTFToEigen(tf_transform, pose);
  return true;
}

}  // namespace curie_demos
