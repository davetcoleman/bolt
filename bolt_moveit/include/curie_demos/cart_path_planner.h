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
   Desc:   Creates a cartesian path to be inserted into a planning roadmap
*/

#ifndef CURIE_DEMOS_CART_PATH_PLANNER_H
#define CURIE_DEMOS_CART_PATH_PLANNER_H

// MoveIt
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// ROS
#include <ros/ros.h>

// Descartes
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/sparse_planner.h>
#include <ur5_demo_descartes/ur5_robot_model.h>

// Eigen
#include <eigen_conversions/eigen_msg.h>

// OMPL
#include <ompl/tools/bolt/TaskGraph.h>

// this package
#include <moveit_visual_tools/imarker_robot_state.h>
#include <curie_demos/tolerances.h>

namespace curie_demos
{
class CurieDemos;

class CartPathPlanner
{
public:
  /**
   * \brief Constructor
   */
  CartPathPlanner(CurieDemos* parent);

  void initDescartes();

  void processIMarkerPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback,
                          const Eigen::Affine3d& feedback_pose);

  bool generateExactPoses(bool debug = false);
  bool generateExactPoses(const Eigen::Affine3d& start_pose, bool debug = false);

  bool debugShowAllIKSolutions();
  bool computeAllPoses(const Eigen::Affine3d& pose, const OrientationTol& orientation_tol,
                       EigenSTL::vector_Affine3d& candidate_poses);
  bool rotateOnAxis(const Eigen::Affine3d& pose, const OrientationTol& orientation_tol, const Axis axis,
                    EigenSTL::vector_Affine3d& candidate_poses);
  bool transform2DPath(const Eigen::Affine3d& starting_pose, EigenSTL::vector_Affine3d& poses);
  bool populateBoltGraph(ompl::tools::bolt::TaskGraphPtr task_graph);
  bool addCartPointToBoltGraph(const std::vector<std::vector<double>>& joint_poses,
                               std::vector<ompl::tools::bolt::TaskVertex>& point_vertices,
                               moveit::core::RobotStatePtr moveit_robot_state);
  bool addEdgesToBoltGraph(const TrajectoryGraph& graph_vertices, ompl::tools::bolt::TaskVertex startingVertex,
                           ompl::tools::bolt::TaskVertex endingVertex);
  bool connectTrajectoryEndPoints(const TrajectoryGraph& graph_vertices, double& shortest_path_across_cart);
  bool getAllJointPosesForCartPoint(const Eigen::Affine3d& pose, std::vector<std::vector<double>>& joint_poses);
  void visualizeAllJointPoses(const std::vector<std::vector<double>>& joint_poses);

private:
  // --------------------------------------------------------

  // The short name of this class
  std::string name_;

  // A shared node handle
  ros::NodeHandle nh_;

  // Parent class
  CurieDemos* parent_;

  // The main graph
  ompl::tools::bolt::TaskGraphPtr task_graph_;

  // State
  moveit::core::RobotStatePtr imarker_state_;

  // Rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Interactive markers
  moveit_visual_tools::IMarkerRobotStatePtr imarker_cartesian_;

  // The planning group to work on
  const moveit::core::JointModelGroup* jmg_;

  // Performs tasks specific to the Robot such IK, FK and collision detection
  ur5_demo_descartes::UR5RobotModelPtr ur5_robot_model_;

  // The exact trajectory to follow
  EigenSTL::vector_Affine3d exact_poses_;
  // The trajectory's associated tolernaces
  OrientationTol orientation_tol_;
  // Timing between each pose in exact_poses
  double timing_;

  // User settings
  bool descartes_check_collisions_;

  double orientation_increment_ = 0.5;

  // Robot settings
  std::string group_name_;
  std::string tip_link_;
  std::string base_link_;
  std::string world_frame_;
  double trajectory_discretization_;

  // Desired path to draw
  EigenSTL::vector_Affine3d path_;

};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<CartPathPlanner> CartPathPlannerPtr;
typedef boost::shared_ptr<const CartPathPlanner> CartPathPlannerConstPtr;

}  // namespace curie_demos
#endif  // CURIE_DEMOS_CART_PATH_PLANNER_H
