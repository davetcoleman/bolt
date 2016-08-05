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

#ifndef MOVEIT_OMPL_DETAILS_DEFAULT_STATE_SAMPLER_
#define MOVEIT_OMPL_DETAILS_DEFAULT_STATE_SAMPLER_

#include <moveit_ompl/model_based_state_space.h>

namespace moveit_ompl
{
class DefaultStateSampler : public ompl::base::StateSampler
{
public:
  DefaultStateSampler(const ompl::base::StateSpace *space, const robot_model::JointModelGroup *group,
                      const robot_model::JointBoundsVector *joint_bounds)
    : ompl::base::StateSampler(space)
    , joint_model_group_(group)
    , joint_bounds_(joint_bounds)
  {
  }

  virtual void sampleUniform(ompl::base::State *state)
  {
    // std::cout << "DefaultStateSampler::sampleUniform() " << std::endl;
    joint_model_group_->getVariableRandomPositions(moveit_rng_, state->as<ModelBasedStateSpace::StateType>()->values,
                                                   *joint_bounds_);
    //state->as<ModelBasedStateSpace::StateType>()->clearKnownInformation();
  }

  virtual void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance)
  {
    // std::cout << "DefaultStateSampler::sampleUniformNear() " << std::endl;
    joint_model_group_->getVariableRandomPositionsNearBy(
        moveit_rng_, state->as<ModelBasedStateSpace::StateType>()->values, *joint_bounds_,
        near->as<ModelBasedStateSpace::StateType>()->values, distance);
    //state->as<ModelBasedStateSpace::StateType>()->clearKnownInformation();
  }

  virtual void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev)
  {
    //std::cout << "DefaultStateSampler::sampleGaussian() " << std::endl;
    sampleUniformNear(state, mean, rng_.gaussian(0.0, stdDev));
  }

protected:
  random_numbers::RandomNumberGenerator moveit_rng_;
  const robot_model::JointModelGroup *joint_model_group_;
  const robot_model::JointBoundsVector *joint_bounds_;
};
}
#endif
