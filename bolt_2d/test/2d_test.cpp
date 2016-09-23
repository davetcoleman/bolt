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
   Desc:   Testing for package bolt_2d
*/

/** TEMPLATE NOTES

    EXAMPLES:
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

class TestingBase
{
public:
  // A shared node handle
  // ros::NodeHandle nh_;

  // For visualizing things in rviz
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

  bool initialize()
  {
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("base", "/rviz_visual_tools"));

    // Allow time to publish messages
    ROS_INFO_STREAM_NAMED("test", "Waiting 4 seconds to start test...");
    return true;
  }
};  // class

/* Create instance of test class ---------------------------------------------------------- */
TestingBase base;

/* Run tests ------------------------------------------------------------------------------ */
TEST(TestingBase, initialize)
{
  ASSERT_TRUE(base.initialize());
}

TEST(TestingBase, name_of_this_test)
{
  base.visual_tools_->convertToXYZRPY(expected_affine, xyzrpy);
}

/* Main  ------------------------------------------------------------------------------------- */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "2d_test");
  return RUN_ALL_TESTS();
}
