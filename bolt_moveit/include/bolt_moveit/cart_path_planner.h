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

#ifndef BOLT_MOVEIT_CART_PATH_PLANNER_H
#define BOLT_MOVEIT_CART_PATH_PLANNER_H

// MoveIt
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// ROS
#include <ros/ros.h>

// Eigen
#include <eigen_conversions/eigen_msg.h>

// OMPL
#include <bolt_core/TaskGraph.h>

// this package
#include <moveit_visual_tools/imarker_robot_state.h>
#include <bolt_moveit/tolerances.h>

namespace bolt_moveit
{

typedef std::vector<std::vector<double>> JointPoses;
typedef std::vector<std::vector<ompl::tools::bolt::TaskVertex>> TaskVertexMatrix;

class BoltMoveIt;

class CartPathPlanner
{
public:
  /**
   * \brief Constructor
   */
  CartPathPlanner(BoltMoveIt* parent);

  void processIMarkerPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback,
                          const Eigen::Affine3d& feedback_pose);

  bool generateExactPoses(std::size_t indent);
  bool generateExactPoses(const Eigen::Affine3d& start_pose, std::size_t indent);

  bool debugShowAllIKSolutions(std::size_t indent);
  bool computeRedundantPosesForCartPoint(const Eigen::Affine3d& pose, const OrientationTol& orientation_tol,
                                       EigenSTL::vector_Affine3d& candidate_poses, std::size_t indent);
  bool rotateOnAxis(const Eigen::Affine3d& pose, const OrientationTol& orientation_tol, const Axis axis,
                    EigenSTL::vector_Affine3d& candidate_poses, std::size_t indent);
  bool transform2DPath(const Eigen::Affine3d& starting_pose, EigenSTL::vector_Affine3d& poses, std::size_t indent);
  bool populateBoltGraph(ompl::tools::bolt::TaskGraphPtr task_graph, std::size_t indent);
  bool addCartPointToBoltGraph(const std::vector<std::vector<double>>& joint_poses,
                               std::vector<ompl::tools::bolt::TaskVertex>& point_vertices,
                               moveit::core::RobotStatePtr moveit_robot_state, std::size_t indent);
  bool addEdgesToBoltGraph(const TaskVertexMatrix& graph_vertices, ompl::tools::bolt::TaskVertex startingVertex,
                           ompl::tools::bolt::TaskVertex endingVertex, std::size_t indent);
  bool connectTrajectoryEndPoints(const TaskVertexMatrix& graph_vertices, double& shortest_path_across_cart, std::size_t indent);
  bool getRedundantJointPosesForCartPoint(const Eigen::Affine3d& pose, std::vector<std::vector<double>>& joint_poses,
                                          const moveit::core::LinkModel* ee_link,
                                          std::size_t indent);
  void visualizeAllJointPoses(const std::vector<std::vector<double>>& joint_poses, std::size_t indent);

private:
  // --------------------------------------------------------

  // The short name of this class
  std::string name_ = "cart_path_planner";

  // A shared node handle
  ros::NodeHandle nh_;

  // Parent class
  BoltMoveIt* parent_;

  // The main graph
  ompl::tools::bolt::TaskGraphPtr task_graph_;

  // State used for solving IK
  moveit::core::RobotStatePtr ik_state_;

  // Rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Interactive markers
  moveit_visual_tools::IMarkerRobotStatePtr imarker_cartesian_;

  // The planning group to work on
  //moveit::core::JointModelGroup* jmg_;

  // Performs tasks specific to the Robot such IK, FK and collision detection
  // bolt_ur5::UR5RobotModelPtr ur5_robot_model_;

  // The exact trajectory to follow
  EigenSTL::vector_Affine3d exact_poses_;
  // The trajectory's associated tolernaces
  OrientationTol orientation_tol_;
  // Timing between each pose in exact_poses
  double timing_;

  double ik_discretization_ = M_PI / 4;

  // Robot settings
  std::string group_name_;
  std::string tip_link_;
  std::string base_link_;
  std::string world_frame_;
  double trajectory_discretization_;

  // Desired path to draw
  EigenSTL::vector_Affine3d path_from_file_;

  bool verbose_ = false;
  bool visualize_show_all_solutions_ = false;
  double visualize_show_all_solutions_sleep_ = 0.001;
  bool visualize_show_all_cart_poses_ = false;

  double tolerance_increment_ = 0.5;
  double tolerance_roll_;
  double tolerance_pitch_;
  double tolerance_yaw_;
};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<CartPathPlanner> CartPathPlannerPtr;
typedef boost::shared_ptr<const CartPathPlanner> CartPathPlannerConstPtr;

}  // namespace bolt_moveit
#endif  // BOLT_MOVEIT_CART_PATH_PLANNER_H
