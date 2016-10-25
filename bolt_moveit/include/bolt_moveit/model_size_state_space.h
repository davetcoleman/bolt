/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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
   Desc:   Opitmized memory allocation for model based state space
*/

#ifndef BOLT_MOVEIT_MODEL_SIZE_STATE_SPACE_
#define BOLT_MOVEIT_MODEL_SIZE_STATE_SPACE_

#include <bolt_moveit/model_based_state_space.h>
#include <bolt_moveit/default_state_sampler.h>

namespace bolt_moveit
{
template <size_t N>
class ModelSizeStateSpace : public ModelBasedStateSpace
{
public:
  class StateType : public ompl::base::State
  {
  public:
    StateType() : ompl::base::State()
    {
    }

    double values[N];
  };

  ModelSizeStateSpace(const ModelBasedStateSpaceSpecification &spec) : ModelBasedStateSpace(spec)
  {
    if (variable_count_ != N)
    {
      OMPL_ERROR("Invalid ModelSizeStateSpace %u for variable count %u", N, variable_count_);
      exit(-1);
    }
  }

  virtual ob::State *allocState() const
  {
    return new StateType();
  }

  void freeState(ob::State *state) const
  {
    if (!state)
      BOLT_ERROR(0, "State already deleted " << state);

    delete state->as<StateType>();
  }

  /** \brief Allocate an array of states */
  void allocStates(std::size_t numStates, ob::State *states) const
  {
    StateType *states2 = new StateType[numStates];

    std::cout << "allocStates: " << std::endl;
    for (std::size_t i = 0; i < numStates; ++i)
    {
      std::cout << " - states[i]: " << &states2[i] << std::endl;
      for (std::size_t j = 0; j < N; ++j)
      {
        std::cout << "     - value " << j << ": " << states2[i].as<StateType>()->values[j] << std::endl;
      }
    }
    states = states2;
  }

  /** \brief Free an array of states */
  void freeStates(ob::State *states) const
  {
    delete[] states->as<StateType>();
  }

  void copyFromReals(ob::State *destination, const std::vector<double> &reals) const
  {
    assert(reals.size() == state_values_size_);
    memcpy(destination->as<StateType>()->values, &reals[0], state_values_size_);
  }

  void copyState(ob::State *destination, const ob::State *source) const
  {
    memcpy(destination->as<StateType>()->values, source->as<StateType>()->values, state_values_size_);
  }

  void serialize(void *serialization, const ob::State *state) const
  {
    memcpy(reinterpret_cast<char *>(serialization), state->as<StateType>()->values, state_values_size_);
  }

  void deserialize(ob::State *state, const void *serialization) const
  {
    memcpy(state->as<StateType>()->values, reinterpret_cast<const char *>(serialization), state_values_size_);
  }

  double distance(const ob::State *state1, const ob::State *state2) const
  {
    return spec_.joint_model_group_->distance(state1->as<StateType>()->values, state2->as<StateType>()->values);
  }

  bool equalStates(const ob::State *state1, const ob::State *state2) const
  {
    for (unsigned int i = 0; i < variable_count_; ++i)
      if (fabs(state1->as<StateType>()->values[i] - state2->as<StateType>()->values[i]) >
          std::numeric_limits<double>::epsilon())
        return false;

    return true;
  }

  void enforceBounds(ob::State *state) const
  {
    spec_.joint_model_group_->enforcePositionBounds(state->as<StateType>()->values, spec_.joint_bounds_);
  }

  bool satisfiesBounds(const ob::State *state) const
  {
    // TODO: this is too large an epsilon
    return spec_.joint_model_group_->satisfiesPositionBounds(state->as<StateType>()->values, spec_.joint_bounds_, 0.00001);
    // std::numeric_limits<double>::epsilon());
  }

  void interpolate(const ob::State *from, const ob::State *to, const double t, ob::State *state) const
  {
    if (!interpolation_function_ || !interpolation_function_(from, to, t, state))
    {
      // perform the actual interpolation
      spec_.joint_model_group_->interpolate(from->as<StateType>()->values, to->as<StateType>()->values, t,
                                            state->as<StateType>()->values);
    }
  }

  double *getValueAddressAtIndex(ob::State *state, const unsigned int index) const
  {
    if (index >= variable_count_)
      return NULL;
    return state->as<StateType>()->values + index;
  }

  ob::StateSamplerPtr allocDefaultStateSampler() const
  {
    return ob::StateSamplerPtr(
        static_cast<ob::StateSampler *>(new DefaultStateSampler<ModelSizeStateSpace<N>::StateType>(
            this, spec_.joint_model_group_, &spec_.joint_bounds_)));
  }

  void printState(const ob::State *state, std::ostream &out) const
  {
    for (std::size_t j = 0; j < joint_model_vector_.size(); ++j)
    {
      out << joint_model_vector_[j]->getName() << " = ";
      const int idx = spec_.joint_model_group_->getVariableGroupIndex(joint_model_vector_[j]->getName());
      const int vc = joint_model_vector_[j]->getVariableCount();
      for (int i = 0; i < vc; ++i)
        out << state->as<StateType>()->values[idx + i] << " ";
      out << std::endl;
    }
  }

  void copyToRobotState(robot_state::RobotState &rstate, const ob::State *state) const
  {
    rstate.setJointGroupPositions(spec_.joint_model_group_, state->as<StateType>()->values);

    rstate.update();
  }

  void copyToOMPLState(ob::State *state, const robot_state::RobotState &rstate) const
  {
    rstate.copyJointGroupPositions(spec_.joint_model_group_, state->as<StateType>()->values);
  }

  void copyJointToOMPLState(ob::State *state, const robot_state::RobotState &robot_state,
                            const moveit::core::JointModel *joint_model, int ompl_state_joint_index) const
  {
    // Copy one joint (multiple variables possibly)
    memcpy(getValueAddressAtIndex(state, ompl_state_joint_index),
           robot_state.getVariablePositions() + joint_model->getFirstVariableIndex() * sizeof(double),
           joint_model->getVariableCount() * sizeof(double));
  }
};  // class

ModelBasedStateSpacePtr chooseModelSizeStateSpace(const ModelBasedStateSpaceSpecification &spec)
{
  std::size_t dim = spec.joint_model_group_->getVariableCount();

  switch (dim)
  {
    case 2:
      return ModelBasedStateSpacePtr(new ModelSizeStateSpace<2>(spec));
    case 3:
      return ModelBasedStateSpacePtr(new ModelSizeStateSpace<3>(spec));
    case 4:
      return ModelBasedStateSpacePtr(new ModelSizeStateSpace<4>(spec));
    case 5:
      return ModelBasedStateSpacePtr(new ModelSizeStateSpace<5>(spec));
    case 6:
      return ModelBasedStateSpacePtr(new ModelSizeStateSpace<6>(spec));
    case 7:
      return ModelBasedStateSpacePtr(new ModelSizeStateSpace<7>(spec));
    case 12:
      return ModelBasedStateSpacePtr(new ModelSizeStateSpace<12>(spec));
    case 14:
      return ModelBasedStateSpacePtr(new ModelSizeStateSpace<14>(spec));
  }

  ROS_ERROR_STREAM("No model based state space setup currently for " << dim << " dimensions, TODO");
  exit(-1);
  return NULL;
}

// typedef ModelSizeStateSpace<6> ModelSize6StateSpace;
// typedef ModelSizeStateSpace<7> ModelSize7StateSpace;
// typedef ModelSizeStateSpace<12> ModelSize12StateSpace;
// typedef ModelSizeStateSpace<14> ModelSize14StateSpace;

}  // namespace bolt_moveit

#endif
