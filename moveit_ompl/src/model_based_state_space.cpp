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

#include <moveit_ompl/model_based_state_space.h>
#include <moveit_ompl/detail/default_state_sampler.h>
#include <boost/bind.hpp>

namespace mo = moveit_ompl;

mo::ModelBasedStateSpace::ModelBasedStateSpace(const ModelBasedStateSpaceSpecification &spec,
                                               moveit_visual_tools::MoveItVisualToolsPtr visual_tools)
  : ompl::base::StateSpace(), spec_(spec), visual_tools_(visual_tools)
{
  // set the state space name
  setName(spec_.joint_model_group_->getName());
  setName(getName() + "_JointModel");
  variable_count_ = spec_.joint_model_group_->getVariableCount();
  state_values_size_ = variable_count_ * sizeof(double);
  joint_model_vector_ = spec_.joint_model_group_->getActiveJointModels();

  // make sure we have bounds for every joint stored within the spec (use default bounds if not specified)
  if (!spec_.joint_bounds_.empty() && spec_.joint_bounds_.size() != joint_model_vector_.size())
  {
    ROS_ERROR("Joint group '%s' has incorrect bounds specified. Using the default bounds instead.",
              spec_.joint_model_group_->getName().c_str());
    spec_.joint_bounds_.clear();
  }

  // copy the default joint bounds if needed
  if (spec_.joint_bounds_.empty())
    spec_.joint_bounds_ = spec_.joint_model_group_->getActiveJointModelsBounds();

  // new perform a deep copy of the bounds, in case we need to modify them
  joint_bounds_storage_.resize(spec_.joint_bounds_.size());
  for (std::size_t i = 0; i < joint_bounds_storage_.size(); ++i)
  {
    joint_bounds_storage_[i] = *spec_.joint_bounds_[i];
    spec_.joint_bounds_[i] = &joint_bounds_storage_[i];
  }

  // Convert the bounds to an OMPL format
  // Resize OMPL's bound structure
  bounds_.resize(joint_bounds_storage_.size());
  for (std::size_t i = 0; i < joint_bounds_storage_.size(); ++i)
  {
    // Get the variable bounds - only 1 var supported currently
    const std::vector<moveit::core::VariableBounds> &var_bounds = joint_bounds_storage_[i];
    if (var_bounds.size() != 1)
    {
      ROS_ERROR_STREAM("JointModel does not have just one variable - feature not supported yet! " << var_bounds.size());
      exit(-1);
    }

    // Convert to OMPL format
    bounds_.setLow(i, var_bounds.front().min_position_);
    bounds_.setHigh(i, var_bounds.front().max_position_);

    // Debug
    // std::cout << "JointModel: " << spec_.joint_model_group_->getActiveJointModels()[i]->getName() << " has bounds ["
    //<< var_bounds.front().min_position_ << ", " << var_bounds.front().max_position_ << "]" << std::endl;
  }

  // default settings
  //setTagSnapToSegment(0.95);

  /// expose parameters
  //params_.declareParam<double>("tag_snap_to_segment", boost::bind(&ModelBasedStateSpace::setTagSnapToSegment, this, _1),
  //boost::bind(&ModelBasedStateSpace::getTagSnapToSegment, this));
}

mo::ModelBasedStateSpace::~ModelBasedStateSpace()
{
}

ompl::base::State *mo::ModelBasedStateSpace::allocState() const
{
  StateType *state = new StateType();
  state->values = new double[variable_count_];
  return state;
}

void mo::ModelBasedStateSpace::freeState(ompl::base::State *state) const
{
  delete[] state->as<StateType>()->values;
  delete state->as<StateType>();
}

bool mo::ModelBasedStateSpace::populateState(ompl::base::State *state, const std::vector<double> &values)
{
  // TODO(davetcoleman): make more efficient
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    state->as<ModelBasedStateSpace::StateType>()->values[i] = values[i];
  }
  // memcpy((void *) &values[0], state->as<ModelBasedStateSpace::StateType>()->values,
  // values.size() * sizeof(double));
  return true;
}

void mo::ModelBasedStateSpace::copyState(ompl::base::State *destination, const ompl::base::State *source) const
{
  memcpy(destination->as<StateType>()->values, source->as<StateType>()->values, state_values_size_);
  destination->as<StateType>()->level = source->as<StateType>()->level;
}

unsigned int mo::ModelBasedStateSpace::getSerializationLength() const
{
  return state_values_size_ + sizeof(int);
}

void mo::ModelBasedStateSpace::serialize(void *serialization, const ompl::base::State *state) const
{
  *reinterpret_cast<int *>(serialization) = state->as<StateType>()->level;
  memcpy(reinterpret_cast<char *>(serialization) + sizeof(int), state->as<StateType>()->values, state_values_size_);
}

void mo::ModelBasedStateSpace::deserialize(ompl::base::State *state, const void *serialization) const
{
  state->as<StateType>()->level = *reinterpret_cast<const int *>(serialization);
  memcpy(state->as<StateType>()->values, reinterpret_cast<const char *>(serialization) + sizeof(int),
         state_values_size_);
}

unsigned int mo::ModelBasedStateSpace::getDimension() const
{
  unsigned int d = 0;
  for (std::size_t i = 0; i < joint_model_vector_.size(); ++i)
    d += joint_model_vector_[i]->getStateSpaceDimension();
  return d;
}

double mo::ModelBasedStateSpace::getMaximumExtent() const
{
  return spec_.joint_model_group_->getMaximumExtent(spec_.joint_bounds_);
}

double mo::ModelBasedStateSpace::getMeasure() const
{
  double m = 1.0;
  for (std::size_t i = 0; i < spec_.joint_bounds_.size(); ++i)
  {
    const robot_model::JointModel::Bounds &bounds = *spec_.joint_bounds_[i];
    for (std::size_t j = 0; j < bounds.size(); ++j)
    {
      m *= bounds[j].max_position_ - bounds[j].min_position_;
    }
  }
  return m;
}

double mo::ModelBasedStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
  return spec_.joint_model_group_->distance(state1->as<StateType>()->values, state2->as<StateType>()->values);

  // if (distance_function_)
  //   return distance_function_(state1, state2);
  // else
  // return spec_.joint_model_group_->distance(state1->as<StateType>()->values, state2->as<StateType>()->values);
}

double mo::ModelBasedStateSpace::distance2(const ompl::base::State *state1, const ompl::base::State *state2) const
{
  if (distance_function_)
    return distance_function_(state1, state2);

  std::cout << "distance2 function missing! " << std::endl;
  return 0;
  // return spec_.joint_model_group_->distance(state1->as<StateType>()->values, state2->as<StateType>()->values);
}

bool mo::ModelBasedStateSpace::equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const
{
  for (unsigned int i = 0; i < variable_count_; ++i)
    if (fabs(state1->as<StateType>()->values[i] - state2->as<StateType>()->values[i]) >
        std::numeric_limits<double>::epsilon())
      return false;

  // Check level
  if(state1->as<StateType>()->level != state2->as<StateType>()->level)
    return false;

  return true;
}

void mo::ModelBasedStateSpace::enforceBounds(ompl::base::State *state) const
{
  spec_.joint_model_group_->enforcePositionBounds(state->as<StateType>()->values, spec_.joint_bounds_);
}

bool mo::ModelBasedStateSpace::satisfiesBounds(const ompl::base::State *state) const
{
  return spec_.joint_model_group_->satisfiesPositionBounds(state->as<StateType>()->values, spec_.joint_bounds_,
                                                           std::numeric_limits<double>::epsilon());
}

void mo::ModelBasedStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t,
                                           ompl::base::State *state) const
{
  // clear any cached info (such as validity known or not)
  //state->as<StateType>()->clearKnownInformation();

  if (!interpolation_function_ || !interpolation_function_(from, to, t, state))
  {
    // perform the actual interpolation
    spec_.joint_model_group_->interpolate(from->as<StateType>()->values, to->as<StateType>()->values, t,
                                          state->as<StateType>()->values);
  }
}

double *mo::ModelBasedStateSpace::getValueAddressAtIndex(ompl::base::State *state, const unsigned int index) const
{
  if (index >= variable_count_)
    return NULL;
  return state->as<StateType>()->values + index;
}

void mo::ModelBasedStateSpace::setPlanningVolume(double minX, double maxX, double minY, double maxY, double minZ,
                                                 double maxZ)
{
  for (std::size_t i = 0; i < joint_model_vector_.size(); ++i)
    if (joint_model_vector_[i]->getType() == robot_model::JointModel::PLANAR)
    {
      joint_bounds_storage_[i][0].min_position_ = minX;
      joint_bounds_storage_[i][0].max_position_ = maxX;
      joint_bounds_storage_[i][1].min_position_ = minY;
      joint_bounds_storage_[i][1].max_position_ = maxY;
    }
    else if (joint_model_vector_[i]->getType() == robot_model::JointModel::FLOATING)
    {
      joint_bounds_storage_[i][0].min_position_ = minX;
      joint_bounds_storage_[i][0].max_position_ = maxX;
      joint_bounds_storage_[i][1].min_position_ = minY;
      joint_bounds_storage_[i][1].max_position_ = maxY;
      joint_bounds_storage_[i][2].min_position_ = minZ;
      joint_bounds_storage_[i][2].max_position_ = maxZ;
    }
}

ompl::base::StateSamplerPtr mo::ModelBasedStateSpace::allocDefaultStateSampler() const
{
  return ompl::base::StateSamplerPtr(static_cast<ompl::base::StateSampler *>(
      new DefaultStateSampler(this, spec_.joint_model_group_, &spec_.joint_bounds_)));
}

void mo::ModelBasedStateSpace::printSettings(std::ostream &out) const
{
  out << "ModelBasedStateSpace '" << getName() << "' at " << this << std::endl;
}

void mo::ModelBasedStateSpace::printState(const ompl::base::State *state, std::ostream &out) const
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

  out << "Level: " << state->as<StateType>()->level << std::endl;
}

void mo::ModelBasedStateSpace::copyToRobotState(robot_state::RobotState &rstate, const ompl::base::State *state) const
{
  rstate.setJointGroupPositions(spec_.joint_model_group_, state->as<StateType>()->values);

  rstate.update();
}

void mo::ModelBasedStateSpace::copyToOMPLState(ompl::base::State *state, const robot_state::RobotState &rstate) const
{
  rstate.copyJointGroupPositions(spec_.joint_model_group_, state->as<StateType>()->values);
}

void mo::ModelBasedStateSpace::copyJointToOMPLState(ompl::base::State *state,
                                                    const robot_state::RobotState &robot_state,
                                                    const moveit::core::JointModel *joint_model,
                                                    int ompl_state_joint_index) const
{
  // Copy one joint (multiple variables possibly)
  memcpy(getValueAddressAtIndex(state, ompl_state_joint_index),
         robot_state.getVariablePositions() + joint_model->getFirstVariableIndex() * sizeof(double),
         joint_model->getVariableCount() * sizeof(double));
}

/** \brief Get the mode (for hybrid task planning) of this state */
int mo::ModelBasedStateSpace::getLevel(const ompl::base::State *state) const
{
  return state->as<StateType>()->level;
}

/** \brief Set the mode (for hybrid task planning) of this state */
void mo::ModelBasedStateSpace::setLevel(ompl::base::State *state, int level)
{
  state->as<StateType>()->level = level;
}
