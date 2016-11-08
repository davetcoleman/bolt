/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, PickNik LLC
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

/* Author: Dave Coleman
   Desc:   Testing for package
*/

/** EXAMPLES:
    EXPECT_FALSE(robot_state.hasFixedLinks());
    EXPECT_EQ(robot_state.getFixedLinksCount(), 0);
    EXPECT_TRUE(robot_state.getPrimaryFixedLink() == NULL);
    EXPECT_GT(robot_state.getFixedLinksMode(), 0);
    EXPECT_LT( fabs(vars[0] - 0), EPSILON) << "Virtual joint in wrong position " << vars[0];
*/

// C++
#include <string>

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <bolt_baxter/model_based_state_space.h>

// OMPL
#include <bolt_core/Bolt.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>

/* Class to hold general test data ------------------------------------------------------ */
class TestingBase
{
public:
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  moveit::core::RobotStatePtr robot_state_;
  moveit::core::JointModelGroup *jmg_;

  bool initialize()
  {
    std::vector<std::string> xacro_args;

    static const std::string URDF_PACKAGE = "baxter_description";
    static const std::string URDF_PATH = "urdf/baxter.urdf.xacro";
    std::string urdf_string;
    if (!rdf_loader::RDFLoader::loadPkgFileToString(urdf_string, URDF_PACKAGE, URDF_PATH, xacro_args))
    {
      ROS_ERROR_STREAM("Could not load URDF from file");
      return false;
    }

    static const std::string SRDF_PACKAGE = "baxter_moveit_config";
    static const std::string SRDF_PATH = "config/baxter.srdf";
    std::string srdf_string;
    if (!rdf_loader::RDFLoader::loadPkgFileToString(srdf_string, SRDF_PACKAGE, SRDF_PATH, xacro_args))
    {
      ROS_ERROR_STREAM("Could not load SRDF from file");
      return false;
    }

    // Robot model loader
    robot_model_loader::RobotModelLoader::Options options(urdf_string, srdf_string);
    options.load_kinematics_solvers_ = false;
    robot_model_loader_std::make_shared<robot_model_loader::RobotModelLoader(options));

    // Load the robot model
    robot_model_ = robot_model_loader_->getModel();  // Get a shared pointer to the robot

    // Choose planning group
    jmg_ = robot_model_->getJointModelGroup("right_arm");

    // Create the planning scene
    planning_scene_std::make_shared<planning_scene::PlanningScene(robot_model_));

    // Create initial robot state
    robot_state_std::make_shared<moveit::core::RobotState(planning_scene_->getCurrentState()));

    ROS_INFO_STREAM_NAMED("test", "Done initializing MoveIt!");
    return true;
  }
};  // class

/* Create instance of test class ---------------------------------------------------------- */
TestingBase base;

/* Run tests ------------------------------------------------------------------------------ */

// Initialize
TEST(TestingBase, initialize)
{
  ASSERT_TRUE(base.initialize());
  ASSERT_TRUE(base.robot_model_ != NULL);
  ASSERT_TRUE(base.robot_model_->getName() == "baxter");
  ASSERT_TRUE(base.planning_scene_ != NULL);
  ASSERT_TRUE(base.robot_state_ != NULL);
  ASSERT_TRUE(base.jmg_ != NULL);
}

TEST(TestingBase, get_7d_state_by_vector)
{
  namespace ob = ompl::base;
  namespace ot = ompl::tools;
  namespace otb = ompl::tools::bolt;

  // Setup space
  bolt_baxter::ModelBasedStateSpaceSpecification mbss_spec(base.robot_model_, base.jmg_);

  // Construct the state space we are planning in
  ob::StateSpacePtr space_ = ob::StateSpacePtr(new bolt_baxter::ModelBasedStateSpace(mbss_spec));
  EXPECT_TRUE(space_ != NULL);

  // SpaceInfo
  // ob::SpaceInformationPtr si_ = bolt_->getSpaceInformation();
  ob::SpaceInformationPtr si_ = std::make_shared<ob::SpaceInformation>(space_);
  EXPECT_TRUE(si_ != NULL);
  si_->setup();
  EXPECT_TRUE(si_->isSetup());

  // Example data
  EXPECT_TRUE(space_->getDimension() == 7);
  std::vector<double> values(space_->getDimension(), /*default value*/ 0);
  values[0] = 0.1;
  values[1] = 0.2;
  values[2] = 0.3;
  values[3] = 0.4;
  values[4] = 0.5;
  values[5] = 0.6;
  values[6] = 0.7;
  EXPECT_TRUE(values.size() == 7);

  // Create state
  ob::State *candidateState = space_->allocState();
  EXPECT_TRUE(candidateState != NULL);

  // Populate state
  space_->copyFromReals(candidateState, values);
  EXPECT_TRUE(candidateState != NULL);

  // Convert to real vector
  ob::RealVectorStateSpace::StateType *real_state = static_cast<ob::RealVectorStateSpace::StateType *>(candidateState);
  EXPECT_TRUE(real_state != NULL);
  EXPECT_TRUE(real_state->values[0]);
  EXPECT_TRUE(real_state->values[1]);
  EXPECT_TRUE(real_state->values[0] == 0.1);
  EXPECT_TRUE(real_state->values[1] == 0.2);
  EXPECT_TRUE(real_state->values[6] == 0.7);

  // Get value without real vector
  EXPECT_TRUE((*space_->getValueAddressAtIndex(candidateState, 0)) == 0.1);
  EXPECT_TRUE((*space_->getValueAddressAtIndex(candidateState, 1)) == 0.2);
  EXPECT_TRUE((*space_->getValueAddressAtIndex(candidateState, 6)) == 0.7);

  // Get values into a vector again
  std::vector<double> output_values;
  space_->copyToReals(output_values, candidateState);
  EXPECT_TRUE(output_values.size() == 7);
  EXPECT_TRUE(output_values[0] == 0.1);
  EXPECT_TRUE(output_values[1] == 0.2);
  EXPECT_TRUE(output_values[2] == 0.3);
  EXPECT_TRUE(output_values[3] == 0.4);
  EXPECT_TRUE(output_values[4] == 0.5);
  EXPECT_TRUE(output_values[5] == 0.6);
  EXPECT_TRUE(output_values[6] == 0.7);
}

TEST(TestingBase, compound_state)
{
  namespace ob = ompl::base;
  namespace ot = ompl::tools;
  namespace otb = ompl::tools::bolt;

  // ------------------------------------------------------------------------
  // Create model based state space
  bolt_baxter::ModelBasedStateSpaceSpecification mbss_spec(base.robot_model_, base.jmg_);

  // Construct the state space we are planning in
  ob::StateSpacePtr model_based_space = std::make_shared<bolt_baxter::ModelBasedStateSpace>(mbss_spec);
  EXPECT_TRUE(model_based_space != NULL);
  model_based_space->setup();
  EXPECT_TRUE(model_based_space->getDimension() == 7);

  // Example data
  std::vector<double> values(model_based_space->getDimension(), /*default value*/ 0);
  values[0] = 0.1;
  values[1] = 0.2;
  values[2] = 0.3;
  values[3] = 0.4;
  values[4] = 0.5;
  values[5] = 0.6;
  values[6] = 0.7;
  EXPECT_TRUE(values.size() == 7);

  // Create state
  ob::State *model_state = model_based_space->allocState();
  EXPECT_TRUE(model_state != NULL);

  // Populate state
  model_based_space->copyFromReals(model_state, values);
  EXPECT_TRUE(model_state != NULL);

  // ------------------------------------------------------------------------
  // Create discrete state space
  const int lowerBound = 0;
  const int upperBound = 2;
  ob::StateSpacePtr discrete_space = std::make_shared<ob::DiscreteStateSpace>(lowerBound, upperBound);
  EXPECT_TRUE(discrete_space != NULL);
  EXPECT_TRUE(discrete_space->getDimension() == 1);

  // Create state
  ob::State *discrete_state = discrete_space->allocState();
  EXPECT_TRUE(discrete_state != NULL);

  // Populate state
  const_cast<ob::State *>(discrete_state)->as<ob::DiscreteStateSpace::StateType>()->value = 1;
  EXPECT_TRUE(const_cast<ob::State *>(discrete_state)->as<ob::DiscreteStateSpace::StateType>()->value == 1);

  // ------------------------------------------------------------------------
  // Create compound state space
  ob::CompoundStateSpacePtr compound_space = std::make_shared<ob::CompoundStateSpace>();
  compound_space->addSubspace(model_based_space, 1.0);  // 100% weight
  compound_space->addSubspace(discrete_space, 0.0);     // 0% weight

  // Create default state
  ob::State *default_state = compound_space->allocState();
  EXPECT_TRUE(default_state != NULL);

  // Create manual state
  ob::State *compound_state = new ob::CompoundState();
  EXPECT_TRUE(compound_space->getSubspaceCount() == 2);
  compound_state->as<ob::CompoundState>()->components = new ob::State *[compound_space->getSubspaceCount()];
  for (unsigned int i = 0; i < compound_space->getSubspaceCount(); ++i)
  {
    // Hardcoded - ordering of subspaces TODO
    if (i == 0)
    {
      compound_state->as<ob::CompoundState>()->components[i] = model_state;
    }
    else
    {
      compound_state->as<ob::CompoundState>()->components[i] = discrete_state;
    }
  }

  // Check that the components are the types expected
  EXPECT_TRUE(compound_space->isCompound());
  EXPECT_TRUE(compound_space->getSubspace(0)->getType() == ob::STATE_SPACE_UNKNOWN);  // model_based_state_space
  EXPECT_TRUE(compound_space->getSubspace(0)->getDimension() == 7);
  EXPECT_TRUE(compound_space->getSubspace(1)->getType() == ob::STATE_SPACE_DISCRETE);  // discrete
  EXPECT_TRUE(compound_space->getSubspace(1)->getDimension() == 1);
  EXPECT_TRUE(compound_space->as<ob::DiscreteStateSpace>(1)->getLowerBound() == 0);
  EXPECT_TRUE(compound_space->as<ob::DiscreteStateSpace>(1)->getUpperBound() == 2);

  // Check that the components have the values expected
  EXPECT_TRUE(compound_state->as<ob::CompoundState>()->as<ob::DiscreteStateSpace::StateType>(1)->value ==
              1);  // discrete

  std::vector<double> output_values;  // Get values into a vector again
  model_based_space->copyToReals(
      output_values, compound_state->as<ob::CompoundState>()->as<bolt_baxter::ModelBasedStateSpace::StateType>(0));
  EXPECT_TRUE(output_values.size() == 7);
  EXPECT_TRUE(output_values[0] == 0.1);
  EXPECT_TRUE(output_values[1] == 0.2);
  EXPECT_TRUE(output_values[2] == 0.3);
  EXPECT_TRUE(output_values[3] == 0.4);
  EXPECT_TRUE(output_values[4] == 0.5);
  EXPECT_TRUE(output_values[5] == 0.6);
  EXPECT_TRUE(output_values[6] == 0.7);

  // --------------------------------------------------------------------------
  // Free all memory
  model_based_space->freeState(model_state);
  discrete_space->freeState(discrete_state);
  compound_space->freeState(default_state);
}

/* Main  ------------------------------------------------------------------------------------- */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "moveit_bolt_test");
  return RUN_ALL_TESTS();
}
