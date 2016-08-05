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
   Desc:
*/

#ifndef CURIE_DEMOS_TEST_POSE_DISTANCE_H
#define CURIE_DEMOS_TEST_POSE_DISTANCE_H

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Conversions
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// Visualize
#include <rviz_visual_tools/rviz_visual_tools.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// Boost
#include <boost/lexical_cast.hpp>

namespace curie_demos
{
class TestPoseDistance
{
public:
  /**
   * \brief Constructor
   */
  TestPoseDistance() : name_("test_pose_distance"), nh_("~")
  {
    // Load rosparams
    // ros::NodeHandle rpnh(nh_, name_);
    // std::size_t error = 0;
    // error += !rosparam_shortcuts::get(name_, rpnh, "control_rate", control_rate_);
    // add more parameters here to load if desired
    // rosparam_shortcuts::shutdownIfError(name_, error);

    rviz_visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world", "/hilgendorf/markers"));
    rviz_visual_tools_->deleteAllMarkers();

    ROS_INFO_STREAM_NAMED(name_, "TestPoseDistance Ready.");
  }

  void runLoop()
  {
    tf::TransformListener listener;

    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 0.5;

    ros::Rate rate(10.0);
    while (ros::ok())
    {
      tf::StampedTransform tf_transform;
      Eigen::Affine3d transform;
      try
      {
        listener.lookupTransform("/world", "/thing", ros::Time(0), tf_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }

      // Convert to eigen
      tf::transformTFToEigen(tf_transform, transform);

      // std::cout << "transform: \n"
      //<< transform.rotation() << std::endl;

      double distance = getPoseDistance(transform, Eigen::Affine3d::Identity());

      rviz_visual_tools_->publishText(text_pose, boost::lexical_cast<std::string>(distance));

      rate.sleep();
    }
  }

  double getPoseDistance(const Eigen::Affine3d &from_pose, const Eigen::Affine3d &to_pose)
  {
    std::cout << std::endl;

    const double translation_dist = (from_pose.translation() - to_pose.translation()).norm();

    const Eigen::Quaterniond from(from_pose.rotation());
    const Eigen::Quaterniond to(to_pose.rotation());

    std::cout << "From: " << from.x() << ", " << from.y() << ", " << from.z() << ", " << from.w() << std::endl;
    std::cout << "To: " << to.x() << ", " << to.y() << ", " << to.z() << ", " << to.w() << std::endl;

    double rotational_dist = arcLength(from, to);

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

private:
  // --------------------------------------------------------
  // The short name of this class
  std::string name_;

  // A shared node handle
  ros::NodeHandle nh_;

  ros::Subscriber tf_subscriber_;

  rviz_visual_tools::RvizVisualToolsPtr rviz_visual_tools_;
};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<TestPoseDistance> TestPoseDistancePtr;
typedef boost::shared_ptr<const TestPoseDistance> TestPoseDistanceConstPtr;

}  // namespace curie_demos
#endif  // CURIE_DEMOS_TEST_POSE_DISTANCE_H

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "test_pose_distance");
  ROS_INFO_STREAM_NAMED("main", "Starting TestPoseDistance...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Initialize main class
  curie_demos::TestPoseDistance server;
  server.runLoop();

  // Shutdown
  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}
