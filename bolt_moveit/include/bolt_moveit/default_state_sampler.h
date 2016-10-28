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

/* Author: Ioan Sucan, Dave Coleman */

#ifndef BOLT_MOVEIT_DETAILS_DEFAULT_STATE_SAMPLER_
#define BOLT_MOVEIT_DETAILS_DEFAULT_STATE_SAMPLER_

#include <bolt_moveit/model_based_state_space.h>

namespace bolt_moveit
{
template <typename T>
class DefaultStateSampler : public ompl::base::StateSampler
{
public:
  DefaultStateSampler(const ompl::base::StateSpace *space, ModelBasedStateSpaceSpecification spec)
    : ompl::base::StateSampler(space)
    , joint_model_group_(spec.joint_model_group_)
    , joint_bounds_(&spec.joint_bounds_)
    , robot_state_(spec.robot_model_)
    , near_robot_state_(spec.robot_model_)
    , visual_tools_(spec.visual_tools_)
  {
  }

  virtual void sampleUniform(ompl::base::State *state)
  {
    joint_model_group_->getVariableRandomPositions(moveit_rng_, state->as<T>()->values, *joint_bounds_);
  }

  virtual void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance)
  {
    ROS_INFO_STREAM("sampleUniformNear()");
    // Original method:
    // joint_model_group_->getVariableRandomPositionsNearBy(moveit_rng_, state->as<T>()->values, *joint_bounds_,
    //                                                      near->as<T>()->values, distance);

    // New method: use FK

    // Get near state end effectors
    space_->as<ModelBasedStateSpace>()->copyToRobotState(near_robot_state_, near);

    std::vector<const moveit::core::LinkModel *> tips;
    if (!joint_model_group_->getEndEffectorTips(tips))
    {
      ROS_ERROR_STREAM_NAMED(name_, "Unable to get end effector tips from jmg");
      exit(-1);
    }

    // Datastructures for holding pose
    EigenSTL::vector_Affine3d near_poses;

    // For each end effector save the pose
    for (const moveit::core::LinkModel *ee_parent_link : tips)
      near_poses.push_back(near_robot_state_.getGlobalLinkTransform(ee_parent_link));

    // Sample randomly until within desired distance
    std::size_t count = 0;
    while (ros::ok())
    {
      std::cout << "while loop " << count++ << std::endl;

      // Sample a random state
      robot_state_.setToRandomPositions(joint_model_group_, moveit_rng_);

      // Get the poses
      EigenSTL::vector_Affine3d state_poses;

      // For each end effector save the pose
      for (const moveit::core::LinkModel *ee_parent_link : tips)
        state_poses.push_back(near_robot_state_.getGlobalLinkTransform(ee_parent_link));

      // Get the total distance between poses of arms
      double dist = 0;
      for (std::size_t i = 0; i < tips.size(); ++i)
      {
        dist += getPoseDistance(state_poses[i], near_poses[i]);
        std::cout << "dist: " << dist << std::endl;
      }

      if (dist < distance)
        break;
    }

    std::cout << "found nearby pose " << std::endl;
    visual_tools_->publishRobotState(robot_state_);
    visual_tools_->prompt("found nearby pose");

    std::cout << "should prompt " << std::endl;
ros::Duration(10).sleep();



    // Convert to ompl robot state
    space_->as<ModelBasedStateSpace>()->copyToOMPLState(state, robot_state_);
  }

  virtual void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev)
  {
    sampleUniformNear(state, mean, rng_.gaussian(0.0, stdDev));
  }

  double getPoseDistance(const Eigen::Affine3d &from_pose, const Eigen::Affine3d &to_pose)
  {
    const double translation_dist = (from_pose.translation() - to_pose.translation()).norm();
    // const double distance_wrist_to_finger = 0.25; // meter

    const Eigen::Quaterniond from(from_pose.rotation());
    const Eigen::Quaterniond to(to_pose.rotation());

    // std::cout << "From: " << from.x() << ", " << from.y() << ", " << from.z() << ", " << from.w() << std::endl;
    // std::cout << "To: " << to.x() << ", " << to.y() << ", " << to.z() << ", " << to.w() << std::endl;

    double rotational_dist = arcLength(from, to);  // * distance_wrist_to_finger;

    std::cout << "  Translation_Dist: " << std::fixed << std::setprecision(4) << translation_dist
              << " rotational_dist: " << rotational_dist << std::endl;

    return rotational_dist + translation_dist;
  }

  double arcLength(const Eigen::Quaterniond &from, const Eigen::Quaterniond &to)
  {
    static const double MAX_QUATERNION_NORM_ERROR = 1e-9;
    double dq = fabs(from.x() * to.x() + from.y() * to.y() + from.z() * to.z() + from.w() * to.w());
    if (dq > 1.0 - MAX_QUATERNION_NORM_ERROR)
      return 0.0;
    else
      return acos(dq);
  }

protected:
  const std::string name_ = "default_state_sampler";

  random_numbers::RandomNumberGenerator moveit_rng_;
  const robot_model::JointModelGroup *joint_model_group_;
  const robot_model::JointBoundsVector *joint_bounds_;

  robot_state::RobotState robot_state_;
  robot_state::RobotState near_robot_state_;

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
};
}
#endif
