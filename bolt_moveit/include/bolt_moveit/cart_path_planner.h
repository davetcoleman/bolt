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
           Note: all ordering of multiple end effectors corresponds to order of eefs in parent_->ee_links_
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
typedef std::vector<double> JointSpacePoint;

typedef std::vector<JointSpacePoint> RedunJointPoses;
typedef std::vector<RedunJointPoses> RedunJointTrajectory;

typedef std::vector<JointSpacePoint> BothArmsJointPose;
typedef std::vector<BothArmsJointPose> CombinedPoints;
typedef std::vector<CombinedPoints> CombinedTrajectory;

typedef std::vector<ompl::tools::bolt::TaskVertex> TaskVertexPoint;
typedef std::vector<TaskVertexPoint> TaskVertexMatrix;

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

  void generateExactPoses(std::size_t indent);

  void generateExactPoses(const Eigen::Affine3d& start_pose, std::size_t indent);

  bool debugShowAllIKSolutions(std::size_t indent);

  bool computeRedunPosesForCartPoint(const Eigen::Affine3d& pose, const OrientationTol& orientation_tol,
                                     EigenSTL::vector_Affine3d& candidate_poses, std::size_t indent);

  bool rotateOnAxis(const Eigen::Affine3d& pose, const OrientationTol& orientation_tol, const Axis axis,
                    EigenSTL::vector_Affine3d& candidate_poses, std::size_t indent);

  bool transform2DPaths(const Eigen::Affine3d& starting_pose, std::vector<EigenSTL::vector_Affine3d>& exact_poses,
                        std::size_t indent);

  bool transform2DPath(const Eigen::Affine3d& starting_pose, EigenSTL::vector_Affine3d& path_from_file,
                       EigenSTL::vector_Affine3d& exact_poses, std::size_t indent);

  bool populateBoltGraph(ompl::tools::bolt::TaskGraphPtr task_graph, std::size_t indent);

  bool createSingleDimTrajectory(const EigenSTL::vector_Affine3d& exact_poses, RedunJointTrajectory& redun_poses,
                                 const moveit::core::LinkModel* ee_link, moveit::core::JointModelGroup* jmg,
                                 std::size_t indent);

  bool combineEETrajectories(const std::vector<RedunJointTrajectory>& redun_traj_per_eef,
                             CombinedTrajectory& combined_traj_points, std::size_t indent);

  bool addCartPointToBoltGraph(const CombinedPoints& combined_points, TaskVertexPoint& point_vertices,
                               moveit::core::RobotStatePtr moveit_robot_state, std::size_t indent);

  bool addEdgesToBoltGraph(const TaskVertexMatrix& graphVertices, ompl::tools::bolt::TaskVertex startingVertex,
                           ompl::tools::bolt::TaskVertex endingVertex, std::size_t indent);

  bool connectTrajectoryEndPoints(const TaskVertexMatrix& graphVertices, double& shortest_path_across_cart,
                                  std::size_t indent);

  bool getRedunJointPosesForCartPoint(const Eigen::Affine3d& pose, RedunJointPoses& joint_poses,
                                      const moveit::core::LinkModel* ee_link, moveit::core::JointModelGroup* jmg,
                                      std::size_t indent);

  void visualizeAllJointPoses(const RedunJointPoses& joint_poses, const moveit::core::JointModelGroup* jmg,
                              std::size_t indent);

  void visualizeAllPointVertices(const TaskVertexMatrix& point_vertices, std::size_t indent);

private:
  // --------------------------------------------------------

  // The short name of this class
  std::string name_ = "cart_path_planner";

  // A shared node handle
  ros::NodeHandle nh_;

  // Parent class
  BoltMoveIt* parent_;

  // Data about the arms
  std::vector<moveit_visual_tools::ArmData> arm_datas_;

  // The main graph
  ompl::tools::bolt::TaskGraphPtr task_graph_;

  // State used for solving IK
  moveit::core::RobotStatePtr shared_robot_state0_;
  moveit::core::RobotStatePtr shared_robot_state1_;

  // Rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools2_;

  // Interactive markers
  moveit_visual_tools::IMarkerRobotStatePtr imarker_cartesian_;

  // The exact trajectory to follow
  std::vector<EigenSTL::vector_Affine3d> exact_poses_;
  // The trajectory's associated tolernaces
  OrientationTol orientation_tol_;
  // Timing between each pose in exact_poses
  double timing_;

  double ik_discretization_ = M_PI / 4;

  // Trajectory settings
  double trajectory_discretization_;

  // Desired path to draw
  std::vector<EigenSTL::vector_Affine3d> path_from_file_;

  // Verbose
  bool verbose_ = false;
  bool verbose_collision_check_ = false;

  // Visualize
  bool visualize_all_solutions_ = false;
  double visualize_all_solutions_sleep_ = 0.001;
  bool visualize_all_cart_poses_ = false;
  bool visualize_all_cart_poses_individual_ = false;
  bool visualize_combined_solutions_ = false;
  bool visualize_rejected_states_ = false;
  bool visualize_rejected_edges_due_to_timing_ = false;

  double tolerance_increment_ = 0.5;
  double tolerance_roll_;
  double tolerance_pitch_;
  double tolerance_yaw_;
};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<CartPathPlanner> CartPathPlannerPtr;
typedef boost::shared_ptr<const CartPathPlanner> CartPathPlannerConstPtr;

}  // namespace bolt_moveit

namespace
{
/** \brief Collision checking handle for IK solvers */
bool isStateValid(const planning_scene::PlanningScene *planning_scene, bool verbose, bool only_check_self_collision,
                  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_, robot_state::RobotState *state,
                  const robot_state::JointModelGroup *group, const double *ik_solution);
}

#endif  // BOLT_MOVEIT_CART_PATH_PLANNER_H
