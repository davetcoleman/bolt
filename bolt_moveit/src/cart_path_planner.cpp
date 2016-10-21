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

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// this package
#include <bolt_moveit/cart_path_planner.h>
#include <bolt_moveit/bolt_moveit.h>
#include <bolt_moveit/path_loader.h>

// moveit_boilerplate
#include <moveit_boilerplate/namespaces.h>

// visual tools
#include <moveit_visual_tools/imarker_end_effector.h>

namespace bolt_moveit
{
CartPathPlanner::CartPathPlanner(BoltMoveIt* parent)
  : nh_("~"), parent_(parent), arm_datas_(parent_->arm_datas_), visual_tools2_(parent_->viz6_->getVisualTools())
{
  std::size_t indent = 0;
  // jmg_ = parent_->jmg_;

  double tolerance_pi_fraction_increment;

  // loading parameters
  {
    using namespace rosparam_shortcuts;
    ros::NodeHandle rpnh(nh_, name_);
    std::size_t error = 0;
    error += !get(name_, rpnh, "ik_solver/discretization", ik_solver_discretization_);
    error += !get(name_, rpnh, "trajectory/discretization", trajectory_discretization_);
    error += !get(name_, rpnh, "trajectory/timing", timing_);
    error += !get(name_, rpnh, "tolerance/pi_fraction_increment", tolerance_pi_fraction_increment);
    error += !get(name_, rpnh, "tolerance/roll", tolerance_roll_);
    error += !get(name_, rpnh, "tolerance/pitch", tolerance_pitch_);
    error += !get(name_, rpnh, "tolerance/yaw", tolerance_yaw_);
    error += !get(name_, rpnh, "hybrid_rand_factor", hybrid_rand_factor_);
    error += !get(name_, rpnh, "verbose", verbose_);
    error += !get(name_, rpnh, "visualize/all_solutions", visualize_all_solutions_);
    error += !get(name_, rpnh, "visualize/all_solutions_sleep", visualize_all_solutions_sleep_);
    error += !get(name_, rpnh, "visualize/all_cart_poses", visualize_all_cart_poses_);
    error += !get(name_, rpnh, "visualize/all_cart_poses_individual", visualize_all_cart_poses_individual_);
    error += !get(name_, rpnh, "visualize/combined_solutions", visualize_combined_solutions_);
    error += !get(name_, rpnh, "visualize/combined_solutions_sleep", combined_solutions_sleep_);
    error += !get(name_, rpnh, "visualize/rejected_states", visualize_rejected_states_);
    error += !get(name_, rpnh, "visualize/rejected_edges_due_to_timing", visualize_rejected_edges_due_to_timing_);
    shutdownIfError(name_, error);
  }

  // Load planning state
  shared_robot_state0_.reset(new moveit::core::RobotState(*parent_->moveit_start_));
  shared_robot_state1_.reset(new moveit::core::RobotState(*parent_->moveit_start_));

  // Create cartesian start pose interactive marker
  imarker_cartesian_.reset(new mvt::IMarkerRobotState(parent_->getPlanningSceneMonitor(), "cart", arm_datas_, rvt::BLUE,
                                                      parent_->package_path_));
  imarker_cartesian_->setIMarkerCallback(
      std::bind(&CartPathPlanner::processIMarkerPose, this, std::placeholders::_1, std::placeholders::_2));

  // Set visual tools
  visual_tools_ = imarker_cartesian_->getVisualTools();

  if (!visual_tools_)
    BOLT_ERROR(indent, "Not loaded!");

  visual_tools_->setMarkerTopic(nh_.getNamespace() + "/cartesian_trajectory");
  visual_tools_->loadMarkerPub(true);
  visual_tools_->deleteAllMarkers();
  visual_tools_->setManualSceneUpdating(true);
  visual_tools_->loadTrajectoryPub(nh_.getNamespace() + "/display_trajectory");

  // Load desired path
  PathLoader path_loader(parent_->package_path_);
  const bool debug = false;
  if (!path_loader.get2DPath(path_from_file_, debug))
    exit(0);

  // Specify tolerance for new exact_poses
  tolerance_increment_ = tolerance_pi_fraction_increment;
  orientation_tol_ = OrientationTol(tolerance_roll_, tolerance_pitch_, tolerance_yaw_);

  // Debug tolerances
  // EigenSTL::vector_Affine3d candidate_poses;
  // Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  // computeRedunPosesForCartPoint(pose, orientation_tol_, candidate_poses, true, 0);

  // Trigger the first path viz
  generateExactPoses(indent);

  BOLT_INFO(indent, true, "CartPathPlanner Ready.");
}

void CartPathPlanner::processIMarkerPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback,
                                         const Eigen::Affine3d& feedback_pose)
{
  std::size_t indent = 2;

  // For now both the Cartesian paths are driven only by the right arm
  if (feedback->marker_name == "cart_left")
    return;

  moveit::core::RobotStatePtr imarker_state = imarker_cartesian_->getRobotState();
  const moveit::core::LinkModel* ee_link = imarker_cartesian_->getEEF(feedback->marker_name)->getEELink();
  Eigen::Affine3d start_pose = imarker_state->getGlobalLinkTransform(ee_link);

  generateExactPoses(start_pose, indent);
}

void CartPathPlanner::generateExactPoses(std::size_t indent)
{
  // Generate exact poses
  moveit::core::RobotStatePtr imarker_state = imarker_cartesian_->getRobotState();

  // For now only use right arm
  generateExactPoses(imarker_state->getGlobalLinkTransform(arm_datas_[0].ee_link_), indent);

  // std::vector<Eigen::Affine3d> start_poses;
  // for (std::size_t i = 0; i < arm_datas_->ee_links_.size(); ++i)
  // {
  //   start_poses.push_back(imarker_state->getGlobalLinkTransform(arm_datas_->ee_links_[i]));
  //   generateExactPoses(start_poses[i], indent);
  // }
}

void CartPathPlanner::generateExactPoses(const Eigen::Affine3d& start_pose, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "generateExactPoses()");

  if (!transform2DPaths(start_pose, exact_poses_, indent))
  {
    BOLT_ERROR(indent, "Trajectory generation failed");
    exit(-1);
  }

  BOLT_DEBUG(indent, verbose_, "Generated exact Cartesian traj with " << exact_poses_.front().size() << " points for "
                                                                  << arm_datas_.size() << " dimensions");

  // Publish trajectory poses for visualization
  visual_tools_->deleteAllMarkers();
  for (std::size_t i = 0; i < exact_poses_.size(); ++i)
  {
    visual_tools_->publishPath(exact_poses_[i], rvt::ORANGE, rvt::XXSMALL);
    // visual_tools_->publishAxisPath(exact_poses_[i], rvt::XXXSMALL);
  }
  visual_tools_->trigger();

  if (visualize_all_solutions_)
    debugShowAllIKSolutions(indent);
}

bool CartPathPlanner::debugShowAllIKSolutions(std::size_t indent)
{
  BOLT_FUNC(indent, true, "debugShowAllIKSolutions()");
  std::size_t total_redun_joint_poses = 0;

  // For each dimension
  for (std::size_t j = 0; j < arm_datas_.size(); ++j)
  {
    const moveit::core::LinkModel* ee_link = arm_datas_[j].ee_link_;
    moveit::core::JointModelGroup* jmg = arm_datas_[j].jmg_;

    // Enumerate the potential poses within tolerance
    for (std::size_t i = 0; i < exact_poses_[j].size(); ++i)
    {
      const Eigen::Affine3d& pose = exact_poses_[j][i];

      RedunJointPoses local_joint_poses;
      if (!getRedunJointPosesForCartPoint(pose, local_joint_poses, ee_link, jmg, indent))
      {
        BOLT_ERROR(indent, "Error when getting joint poses for cartesian point");
        continue;
      }

      // Handle error: no IK solutions found
      if (local_joint_poses.empty())
      {
        BOLT_ERROR(indent, "No joint solutions found for cartesian pose " << i);

        visual_tools_->publishAxis(pose, rvt::XXSMALL);
        visual_tools_->trigger();

        return false;
      }

      // Show all possible configurations
      visualizeAllJointPoses(local_joint_poses, jmg, indent);

      total_redun_joint_poses += local_joint_poses.size();
    }
  }
  BOLT_INFO(indent, true, "Found " << total_redun_joint_poses << " total redun joint poses");

  return true;
}

bool CartPathPlanner::computeRedunPosesForCartPoint(const Eigen::Affine3d& pose, const OrientationTol& orientation_tol,
                                                    EigenSTL::vector_Affine3d& candidate_poses, bool verbose,
                                                    std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "computeRedunPosesForCartPoint()");
  BOOST_ASSERT_MSG(tolerance_increment_ > std::numeric_limits<double>::epsilon(), "Divide by zero using orientation "
                                                                                  "increment");

  // override input TODO
  verbose = visualize_all_cart_poses_individual_;

  const std::size_t num_axis = 3;

  BOLT_INFO(indent, verbose, "----------------------------------");
  BOLT_INFO(indent, verbose, "Poses per Point");
  BOLT_INFO(indent, verbose, "  IK Solver Discretization: " << ik_solver_discretization_);
  BOLT_INFO(indent, verbose, "  Tolerance Increment:      " << tolerance_increment_);

  // Reserve vector size
  std::vector<size_t> num_steps_per_axis(num_axis, 0 /* value */);
  for (std::size_t i = 0; i < num_axis; ++i)
  {
    double range = 2 * orientation_tol.axis_dist_from_center_[i];
    num_steps_per_axis[i] = std::max(1.0, ceil(range / tolerance_increment_));
    // std::cout << "range: " << range << std::endl;
    // std::cout << "num_steps_per_axis[i]: " << num_steps_per_axis[i] << std::endl;

    if (i == 0)
      BOLT_INFO(indent, verbose, "  Roll:");
    else if (i == 1)
      BOLT_INFO(indent, verbose, "  Pitch:");
    else if (i == 2)
      BOLT_INFO(indent, verbose, "  Yaw:");
    BOLT_INFO(indent, verbose, "    Tolerance:          " << orientation_tol.axis_dist_from_center_[i]);
    BOLT_INFO(indent, verbose, "    Range (x2):         " << range);
    BOLT_INFO(indent, verbose, "    Steps:              " << num_steps_per_axis[i]);
  }
  double total_num_steps = num_steps_per_axis[0] * num_steps_per_axis[1] * num_steps_per_axis[2];

  BOLT_INFO(indent, verbose, "  Total Poses:          " << total_num_steps);
  BOLT_INFO(indent, verbose, "----------------------------------");

  candidate_poses.reserve(total_num_steps);

  // Start recursion
  bool result = rotateOnAxis(pose, orientation_tol, X_AXIS, candidate_poses, indent);

  if (visualize_all_cart_poses_ || visualize_all_cart_poses_individual_)
  {
    for (const Eigen::Affine3d& candidate_pose : candidate_poses)
    {
      if (visualize_all_cart_poses_individual_)
      {
        visual_tools_->deleteAllMarkers();
        visual_tools_->publishAxis(candidate_pose);
        // visual_tools_->publishZArrow(candidate_pose, rvt::BLUE, rvt::SMALL);
        visual_tools_->trigger();
        parent_->waitForNextStep("next pose");
      }
      else
      {
        visual_tools_->publishAxis(candidate_pose);
      }
    }
    visual_tools_->trigger();
  }

  BOOST_ASSERT_MSG(candidate_poses.size() == total_num_steps, "Estimated poses should be same as actual poses");

  return result;
}

bool CartPathPlanner::rotateOnAxis(const Eigen::Affine3d& pose, const OrientationTol& orientation_tol, const Axis axis,
                                   EigenSTL::vector_Affine3d& candidate_poses, std::size_t indent)
{
  BOLT_FUNC(indent, false, "rotateOnAxis()");

  const bool verbose = false;
  double range = 2 * orientation_tol.axis_dist_from_center_[axis];
  std::size_t num_steps = std::max(1.0, ceil(range / tolerance_increment_));
  Eigen::Affine3d new_pose;

  // Rotate all around one axis
  for (int i = num_steps * -0.5; i < num_steps * 0.5; ++i)
  {
    double rotation_amount = i * tolerance_increment_;
    BOLT_DEBUG(indent, verbose, std::string(axis * 2, ' ') << "axis: " << axis << " i: " << i << " rotation_amount: "
                                                           << rotation_amount << " num_steps: " << num_steps);

    // clang-format off
    switch (axis)
    {
      case X_AXIS: new_pose = pose * Eigen::AngleAxisd(rotation_amount, Eigen::Vector3d::UnitX()); break;
      case Y_AXIS: new_pose = pose * Eigen::AngleAxisd(rotation_amount, Eigen::Vector3d::UnitY()); break;
      case Z_AXIS: new_pose = pose * Eigen::AngleAxisd(rotation_amount, Eigen::Vector3d::UnitZ()); break;
      default: BOLT_ERROR(indent, "Unknown axis");
    }
    // clang-format on

    // Recursively rotate
    if (axis < Z_AXIS)
    {
      rotateOnAxis(new_pose, orientation_tol, static_cast<Axis>(axis + 1), candidate_poses, indent);
    }
    else
    {
      candidate_poses.push_back(new_pose);
      // visual_tools_->publishZArrow(new_pose, rvt::BLUE, rvt::XXXSMALL);
      // visual_tools_->publishAxis(new_pose, rvt::XXXSMALL);
    }
  }
  return true;
}

bool CartPathPlanner::transform2DPaths(const Eigen::Affine3d& starting_pose,
                                       std::vector<EigenSTL::vector_Affine3d>& exact_poses, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "transform2DPaths()");

  if (path_from_file_.empty())
  {
    BOLT_ERROR(indent, "Unable to create drawing: no paths loaded from file");
    return false;
  }

  // Remove the roll and pitch
  // std::vector<double> xyzrpy;
  // rviz_visual_tools::RvizVisualTools::convertToXYZRPY(starting_pose, xyzrpy);
  // xyzrpy[3] = 0;
  // xyzrpy[4] = 0;
  // Eigen::Affine3d starting_pose_aligned = rviz_visual_tools::RvizVisualTools::convertFromXYZRPY(xyzrpy);

  exact_poses.clear();
  exact_poses.resize(path_from_file_.size());

  for (std::size_t i = 0; i < path_from_file_.size(); ++i)
  {
    transform2DPath(starting_pose, path_from_file_[i], exact_poses[i], indent);
  }

  return true;
}

bool CartPathPlanner::transform2DPath(const Eigen::Affine3d& starting_pose, EigenSTL::vector_Affine3d& path_from_file,
                                      EigenSTL::vector_Affine3d& exact_poses, std::size_t indent)
{
  if (path_from_file.empty())
  {
    BOLT_ERROR(indent, "Unable to create drawing: no path loaded from file");
    return false;
  }

  if (path_from_file.size() == 1)
  {
    BOLT_ERROR(indent, "Unable to create drawing: path only has 1 point");
    return false;
  }

  // Transform each point read from file
  EigenSTL::vector_Affine3d transformed_poses;
  for (std::size_t i = 0; i < path_from_file.size(); ++i)
  {
    Eigen::Affine3d point = starting_pose * path_from_file[i];

    // Rotate 90 so that the x axis points down
    // point = point * Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY());

    // Add start pose
    transformed_poses.push_back(point);
  }

  // Discretize trajectory
  for (std::size_t i = 1; i < transformed_poses.size(); ++i)
  {
    const Eigen::Affine3d& p1 = transformed_poses[i - 1];
    const Eigen::Affine3d& p2 = transformed_poses[i];

    // decide how many steps we will need for this trajectory
    const double distance = (p2.translation() - p1.translation()).norm();
    const std::size_t steps = ceil(distance / static_cast<double>(trajectory_discretization_));

    Eigen::Quaterniond p1_quaternion(p1.rotation());
    Eigen::Quaterniond p2_quaternion(p2.rotation());
    for (std::size_t j = 0; j < steps; ++j)
    {
      double percentage = (double)j / (double)steps;

      // Create new intermediate pose
      Eigen::Affine3d temp_pose(p1_quaternion.slerp(percentage, p2_quaternion));
      temp_pose.translation() = percentage * p2.translation() + (1 - percentage) * p1.translation();

      // Add to trajectory
      exact_poses.push_back(temp_pose);
    }
  }

  return true;
}

bool CartPathPlanner::populateBoltGraph(ompl::tools::bolt::TaskGraphPtr task_graph, std::size_t indent)
{
  BOLT_FUNC(indent, true, "populateBoltGraph()");
  ros::Time start_time = ros::Time::now();

  task_graph_ = task_graph;  // copy pointer into this class to share among all functions

  // Make sure exact poses are available
  if (exact_poses_.empty())
  {
    BOLT_WARN(indent, true, "No exact poses had been previously generated. Creating.");

    // Generating trajectory
    generateExactPoses(indent);
  }

  if (arm_datas_.size() < 2)
  {
    BOLT_ERROR(indent, "Must have at least 2 poses in trajectory");
    return false;
  }

  // For converting to MoveIt! format
  moveit::core::RobotStatePtr moveit_robot_state(new moveit::core::RobotState(*visual_tools_->getSharedRobotState()));

  // Remove any previous Cartesian vertices/edges by simply re-creating the whole task graph
  task_graph_->generateTaskSpace(indent);

  // std::cout << "cart_path_planner ending early " << std::endl;
  // return true;

  // For assertion testing
  ompl::tools::bolt::TaskVertex startingVertex = task_graph_->getNumVertices() - 1;

  // -------------------------------------------------------------------------
  // Solve each dimension redunant joint poses independently, the combine into full state space
  std::vector<RedunJointTrajectory> redun_traj_per_eef(arm_datas_.size());

  // For each dimension
  for (std::size_t i = 0; i < arm_datas_.size(); ++i)
  {
    const moveit::core::LinkModel* ee_link = arm_datas_[i].ee_link_;
    moveit::core::JointModelGroup* arm_jmg = arm_datas_[i].jmg_;
    BOLT_DEBUG(indent, true, "Creating redundant poses for end effector \"" << ee_link->getName() << "\"");
    if (!createSingleDimTrajectory(exact_poses_[i], redun_traj_per_eef[i], ee_link, arm_jmg, indent))
    {
      BOLT_ERROR(indent, "Error creating single dim trajectory");
      return false;
    }
  }

  // Re-arrange multiple trajectories so they are matched on each cartesian point (and multiplied together)
  CombinedTrajectory combined_traj_points;
  if (!combineEETrajectories(redun_traj_per_eef, combined_traj_points, indent))
    return false;

  // Track all created vertices - for each pt in the cartesian trajectory, contains multiple vertices
  TaskVertexMatrix point_vertices;
  point_vertices.resize(combined_traj_points.size());

  BOLT_DEBUG(indent, true, "Converting combined trajectories into TaskGraph vertices");

  std::size_t total_vertices = 0;
  for (std::size_t traj_id = 0; traj_id < combined_traj_points.size(); ++traj_id)
  {
    const CombinedPoints& combined_points = combined_traj_points[traj_id];

    // Stats
    total_vertices += combined_points.size();

    // Convert all possible configurations into the Bolt graph
    if (!addCartPointToBoltGraph(combined_points, point_vertices[traj_id], moveit_robot_state, indent))
    {
      BOLT_ERROR(indent, "Failed to add all joint configurations to Bolt graph");
      return false;
    }

    if (!ros::ok())
      break;
  }
  BOLT_DEBUG(indent, true, "Added " << total_vertices << " total vertices to the task graph");

  // For assertion testing
  ompl::tools::bolt::TaskVertex endingVertex = task_graph_->getNumVertices() - 1;

  // ---------------------------------------------------------------
  // Add edges
  if (!addEdgesToBoltGraph(point_vertices, startingVertex, endingVertex, indent))
  {
    BOLT_ERROR(indent, "Error creating edges");
    return false;
  }

  // ---------------------------------------------------------------

  // Track the shortest straight-line cost across any pair of start/goal points
  double shortest_path_across_cart = std::numeric_limits<double>::infinity();
  if (!connectTrajectoryEndPoints(point_vertices, shortest_path_across_cart, indent))
  {
    BOLT_ERROR(indent, "Unable to connect trajectory end points!");
    return false;
  }

  // Set the shortest path across cartesian graph in the TaskGraph
  task_graph_->setShortestDistAcrossCart(shortest_path_across_cart);

  // Tell the planner to require task planning
  task_graph_->setTaskPlanningEnabled();

  // Benchmark runtime
  double duration = (ros::Time::now() - start_time).toSec();
  task_graph_->printGraphStats(duration, indent);

  return true;
}

bool CartPathPlanner::createSingleDimTrajectory(const EigenSTL::vector_Affine3d& exact_poses,
                                                RedunJointTrajectory& redun_poses,
                                                const moveit::core::LinkModel* ee_link,
                                                moveit::core::JointModelGroup* jmg, std::size_t indent)
{
  BOLT_FUNC(indent, true, "createSingleDimTrajectory() for " << exact_poses.size() << " trajectory points");

  // Enumerate the potential cartesian poses within tolerance
  for (std::size_t traj_id = 0; traj_id < exact_poses.size(); ++traj_id)
  {
    const Eigen::Affine3d& exact_pose = exact_poses[traj_id];

    // Calculate all possible joint solutions
    RedunJointPoses joint_poses;
    getRedunJointPosesForCartPoint(exact_pose, joint_poses, ee_link, jmg, indent);

    // Handle error: no IK solutions found
    if (joint_poses.empty())
    {
      BOLT_ERROR(indent, "No joint solutions found for pose " << traj_id);

      visual_tools_->publishAxis(exact_pose, rvt::SMALL);
      visual_tools_->trigger();

      // Show last valid pose if possible
      if (traj_id > 0)
      {
        joint_poses.clear();  // reset vector

        // Get the joint poses from the last cartesian point
        getRedunJointPosesForCartPoint(exact_poses[traj_id - 1], joint_poses, ee_link, jmg, indent);
        BOOST_ASSERT_MSG(!joint_poses.empty(), "Should not happen - no joint poses found for previous cartesian point");
        visual_tools_->publishRobotState(joint_poses.front(), jmg, rvt::RED);
      }
      else
        BOLT_WARN(indent, true, "First pose is invalid, unable to visualize last pose");

      return false;
    }

    // Copy to trajectory
    redun_poses.push_back(std::move(joint_poses));

    // Debug:: Show all possible configurations
    if (false)
      visualizeAllJointPoses(joint_poses, jmg, indent);
  }

  BOLT_DEBUG(indent, true, "Trajectory " << ee_link->getName() << " has redun_poses: " << redun_poses.size());

  return true;
}

bool CartPathPlanner::combineEETrajectories(const std::vector<RedunJointTrajectory>& redun_traj_per_eef,
                                            CombinedTrajectory& combined_traj_points, std::size_t indent)
{
  BOLT_ASSERT(redun_traj_per_eef.size() == 2, "cart_path_planner: combineEETrajectories() is hard coded for only "
                                              "two eefs currently");

  // Find longest trajectory
  std::size_t longest_trajectory = 0;
  std::size_t longest_trajectory_length = 0;
  for (std::size_t i = 0; i < redun_traj_per_eef.size(); ++i)
  {
    if (redun_traj_per_eef[i].size() > longest_trajectory_length)
    {
      longest_trajectory_length = redun_traj_per_eef[i].size();
      longest_trajectory = i;
    }
  }

  // Loop through trajectory points in longest trajectory
  for (std::size_t i = 0; i < redun_traj_per_eef[longest_trajectory].size(); ++i)
  {
    // Get trajectory for each eef
    // Assume only two end effectors. TODO: remove this assumption
    const RedunJointTrajectory& trajectory_eef0 = redun_traj_per_eef[0];
    const RedunJointTrajectory& trajectory_eef1 = redun_traj_per_eef[1];

    const RedunJointPoses* poses0;
    const RedunJointPoses* poses1;

    // Check that both have a poses for this trajectory point
    // it is possible they would not because one trajectory is longer than the other
    if (trajectory_eef0.size() <= i)
    {
      // just use the last set of redun poses in the shorter trajectory
      // BOLT_INFO(indent, true, "eef0: using last set of redundant poses in shorter trajectory");
      poses0 = &trajectory_eef0.back();
    }
    else
      poses0 = &redun_traj_per_eef[0][i];

    if (trajectory_eef1.size() <= i)
    {
      // just use the last set of redun poses in the shorter trajectory
      // BOLT_INFO(indent, true, "eef1: using last set of redundant poses in shorter trajectory");
      poses1 = &trajectory_eef1.back();
    }
    else
      poses1 = &redun_traj_per_eef[1][i];

    // std::cout << "eef0.size(): " << trajectory_eef0.size() << ", eef1.size(): " << trajectory_eef1.size()
    //           << ", i: " << i << std::endl;

    BOLT_ASSERT(poses0, "Poses0 null");
    BOLT_ASSERT(poses1, "Poses1 null");
    BOLT_ASSERT(!poses0->empty(), "No redun poses for eef 0");
    BOLT_ASSERT(!poses1->empty(), "No redun poses for eef 1");

    // Create a combined trajectory point
    CombinedPoints combined_points;

    // Loop through each pair of poses across end effectors
    for (std::size_t pose0_id = 0; pose0_id < poses0->size(); ++pose0_id)
    {
      bool pairEveryPoseWithEveryPose = (hybrid_rand_factor_ < std::numeric_limits<double>::epsilon());

      // Enumerate every possible pair of poses
      if (pairEveryPoseWithEveryPose)
      {
        // std::cout << "pose0_id: " << pose0_id << " size: " << poses0->size() << std::endl;
        for (std::size_t pose1_id = 0; pose1_id < poses1->size(); ++pose1_id)
        {
          if (poses0->size() < pose0_id)
            BOLT_ERROR(indent, "Pose 0 is null at id " << pose0_id);
          if (poses1->size() < pose1_id)
            BOLT_ERROR(indent, "Pose 1 is null at id " << pose1_id);

          // Create new trajectory point
          BothArmsJointPose point;
          point.push_back((*poses0)[pose0_id]);
          point.push_back((*poses1)[pose1_id]);

          combined_points.push_back(point);
        }
      }
      else  // randomly choose a pose from other trajectory
      {
        // Add multiple random poses
        std::size_t num_rand_poses = std::min(std::size_t(hybrid_rand_factor_), poses1->size());
        for (std::size_t i = 0; i < num_rand_poses; ++i)
        {
          std::size_t pose1_id;

          if (i == 0)  // not random
            pose1_id = (pose0_id < poses1->size()) ? pose0_id : poses1->size() - 1;
          else  // random
            pose1_id = visual_tools_->iRand(0, poses1->size() - 1);

          // std::cout << "pose0_id: " << pose0_id << " pose1_id: " << pose1_id << std::endl;

          // Create new trajectory point
          BothArmsJointPose point;
          point.push_back((*poses0)[pose0_id]);
          point.push_back((*poses1)[pose1_id]);

          combined_points.push_back(point);
        }
      }
    }
    BOLT_DEBUG(indent, verbose_, "Total combined points for this trajectory point: " << combined_points.size());

    if (combined_points.size() == 0)
    {
      BOLT_DEBUG(indent, !verbose_, "Total combined points for this trajectory point: " << combined_points.size());
      BOLT_ERROR(indent, "Trajectory point has no point!");
      return false;
    }

    // Add set of points to trajectory
    combined_traj_points.push_back(combined_points);
  }

  BOLT_DEBUG(indent, verbose_, "Total size of combined traj points vector: " << combined_traj_points.size());
  return true;
}

bool CartPathPlanner::addCartPointToBoltGraph(const CombinedPoints& combined_points, TaskVertexPoint& point_vertices,
                                              moveit::core::RobotStatePtr moveit_robot_state, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "addCartPointToBoltGraph() - total points: " << combined_points.size());

  moveit_ompl::ModelBasedStateSpacePtr space = parent_->space_;

  std::size_t new_vertex_count = 0;
  const ompl::tools::bolt::VertexType vertex_type = ompl::tools::bolt::CARTESIAN;
  const ompl::tools::bolt::VertexLevel level1 = 1;  // middle layer

  point_vertices.reserve(combined_points.size());
  std::size_t states_rejected = 0;

  // Lock planning scene
  std::shared_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls =
    std::make_shared<planning_scene_monitor::LockedPlanningSceneRO>(parent_->getPlanningSceneMonitor());
  const planning_scene::PlanningScene* planning_scene =
    static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get();

  for (std::size_t i = 0; i < combined_points.size(); ++i)
  {
    const BothArmsJointPose& joints_pose = combined_points[i];
    BOLT_ASSERT(joints_pose.size() == 2, "TODO: invalid number of arm joint poses, currently can only be 2");

    // Copy vector into moveit format
    // TODO: this is hard coded for 2 arms currently
    moveit_robot_state->setJointGroupPositions(arm_datas_[0].jmg_, joints_pose[0]);
    moveit_robot_state->setJointGroupPositions(arm_datas_[1].jmg_, joints_pose[1]);

    // Check for validity
    moveit_robot_state->update();

    if (!planning_scene->isStateValid(*moveit_robot_state, parent_->planning_jmg_->getName(), verbose_collision_check_))
    {
      BOLT_DEBUG(indent, verbose_collision_check_, "Invalid robot state");

      if (visualize_rejected_states_)
      {
        visual_tools_->publishRobotState(moveit_robot_state, rvt::BROWN);
        parent_->waitForNextStep("invalid");
      }

      states_rejected++;
      continue;
    }

    // Create new OMPL state
    ompl::base::State* ompl_state = space->allocState();

    // Convert the MoveIt! robot state into an OMPL state
    space->copyToOMPLState(ompl_state, *moveit_robot_state);

    // Add vertex to task graph
    point_vertices.push_back(task_graph_->addVertexWithLevel(ompl_state, level1, indent));

    if (visualize_combined_solutions_)
    {
      visual_tools_->publishRobotState(moveit_robot_state);

      if (combined_solutions_sleep_ < std::numeric_limits<double>::epsilon())
        parent_->waitForNextStep("added vertex");
      else
        ros::Duration(combined_solutions_sleep_).sleep();
    }

    new_vertex_count++;
  } // end for

  const double percent = states_rejected / double(combined_points.size()) * 100.0;
  BOLT_DEBUG(indent, verbose_, "Added " << new_vertex_count << " new vertices to TaskGraph, rejected by collision: "
                                        << states_rejected << "  (" << percent << "%)");

  return true;
}

bool CartPathPlanner::addEdgesToBoltGraph(const TaskVertexMatrix& point_vertices,
                                          ompl::tools::bolt::TaskVertex startingVertex,
                                          ompl::tools::bolt::TaskVertex endingVertex, std::size_t indent)
{
  BOLT_FUNC(indent, true, "addEdgesToBoltGraph()");

  moveit_ompl::ModelBasedStateSpacePtr space = parent_->space_;

  // Iterate to create edges
  std::size_t new_edge_count = 0;
  std::size_t edges_skipped_count = 0;
  const ompl::tools::bolt::EdgeType edge_type = ompl::tools::bolt::eCARTESIAN;

  // Debug
  if (false)
    visualizeAllPointVertices(point_vertices, indent);

  // Step through each cartesian point, starting at second point
  std::size_t feedbackFrequency = std::max(10.0, point_vertices.size() / 10.0);
  std::cout << "          Adding edges at traj pt: ";
  for (std::size_t traj_id = 1; traj_id < point_vertices.size(); ++traj_id)
  {
    // Get all vertices at previous cartesian point
    const TaskVertexPoint& point_vertices0 = point_vertices[traj_id - 1];
    // Get all vertices at this cartesian point
    const TaskVertexPoint& point_vertices1 = point_vertices[traj_id];

    // Step through each vertex in a cartesian point
    for (std::size_t vertex1_id = 0; vertex1_id < point_vertices1.size(); ++vertex1_id)
    {
      ompl::tools::bolt::TaskVertex v1 = point_vertices1[vertex1_id];
      space->copyToRobotState(*shared_robot_state1_, task_graph_->getModelBasedState(v1));

      BOLT_ASSERT(v1 > startingVertex && v1 <= endingVertex, "Attempting to create edge with out of range vertex");

      // Visualize
      // visual_tools2_->publishRobotState(shared_robot_state1_, rvt::ORANGE);

      // Connect to every vertex in *previous* cartesian point
      for (std::size_t vertex0_id = 0; vertex0_id < point_vertices0.size(); ++vertex0_id)
      {
        ompl::tools::bolt::TaskVertex v0 = point_vertices0[vertex0_id];
        BOLT_ASSERT(v0 > startingVertex && v0 <= endingVertex, "Attempting to create edge with out of range vertex");

        // Convert OMPL state to MoveIt! state
        space->copyToRobotState(*shared_robot_state0_, task_graph_->getModelBasedState(v0));

        if (!shared_robot_state0_->isValidVelocityMove(*shared_robot_state1_, parent_->planning_jmg_, timing_))
        {
          // Visualize
          if (visualize_rejected_edges_due_to_timing_)
          {
            visual_tools_->publishRobotState(shared_robot_state0_, rvt::GREEN);
            usleep(1 * 1000000);
            visual_tools_->publishRobotState(shared_robot_state1_, rvt::LIME_GREEN);
            parent_->waitForNextStep("showing rejected edge due to timing");
          }

          // BOLT_DEBUG(indent, true, "Skipping edge, total skipped: " << edges_skipped_count);
          edges_skipped_count++;

          continue;
        }

        // BOLT_DEBUG(indent, true, "Adding edge " << v0 << " to " << v1);
        task_graph_->addEdge(v0, v1, indent);

        new_edge_count++;
      }  // for

      if (!ros::ok())
        exit(0);
    }  // for

    // BOLT_DEBUG(indent, true, "Point " << traj_id << " current edge count: " << new_edge_count);

    if (!ros::ok())
      exit(0);

    // Feedback
    if ((traj_id + 1) % feedbackFrequency == 0)
      std::cout << static_cast<int>(ceil(traj_id / static_cast<double>(point_vertices.size()) * 100.0)) << "% "
                << std::flush;

  }  // for
  std::cout << std::endl;

  BOLT_INFO(indent, true, "Added " << new_edge_count << " new edges, rejected " << edges_skipped_count
                                   << " because of velocity constraint (rejected "
                                   << edges_skipped_count / (new_edge_count + double(edges_skipped_count)) * 100.0
                                   << "%)");

  std::size_t warning_factor = 4;
  if (edges_skipped_count * warning_factor > new_edge_count)
    BOLT_INFO(indent, true, "More than " << warning_factor << " times as many edges were rejected than accepted "
                                                              "because of motion timing/velocity contraints.");

  return true;
}

bool CartPathPlanner::connectTrajectoryEndPoints(const TaskVertexMatrix& point_vertices,
                                                 double& shortest_path_across_cart, std::size_t indent)
{
  BOLT_FUNC(indent, true, "connectTrajectoryEndPoints()");

  // force visualization
  // task_graph_->visualizeTaskGraph_ = true;

  // Get all vertices at this starting and ending points
  const TaskVertexPoint& start_vertices = point_vertices.front();
  const TaskVertexPoint& goal_vertices = point_vertices.back();

  // Loop through all start points
  BOLT_INFO(indent, true, "Connecting Cartesian start points to TaskGraph");
  for (const ompl::tools::bolt::TaskVertex& start_vertex : start_vertices)
  {
    // Connect to TaskGraph
    const ompl::tools::bolt::VertexLevel level0 = 0;
    bool isStart = true;
    if (!task_graph_->connectVertexToNeighborsAtLevel(start_vertex, level0, isStart, indent))
    {
      BOLT_WARN(indent, true, "Failed to connect start vertex");
      return false;  // TODO(davetcoleman): return here?
    }

    // Calculate the shortest straight-line distance across graph
    // Record min cost for cost-to-go heurstic distance function later
    // Check if this start vertex has has the shortest path across the Cartesian graph
    for (const ompl::tools::bolt::TaskVertex& goal_vertex : goal_vertices)
    {
      double distance_across_graph = task_graph_->distanceVertex(start_vertex, goal_vertex);

      if (distance_across_graph < shortest_path_across_cart)
      {
        shortest_path_across_cart = distance_across_graph;
      }
    }

    if (!ros::ok())
      exit(-1);
  }

  // Loop through all goal points
  BOLT_INFO(indent, true, "Connecting Cartesian end points to TaskGraph");
  for (const ompl::tools::bolt::TaskVertex& goal_vertex : goal_vertices)
  {
    // Connect to TaskGraph
    const ompl::tools::bolt::VertexLevel level2 = 2;
    bool isStart = false;
    if (!task_graph_->connectVertexToNeighborsAtLevel(goal_vertex, level2, isStart, indent))
    {
      BOLT_WARN(indent, true, "Failed to connect goal vertex");
      return false;  // TODO(davetcoleman): return here?
    }

    if (!ros::ok())
      exit(-1);
  }
  BOLT_INFO(indent, true, "Finished connecting Cartesian end points to TaskGraph");

  return true;
}

bool CartPathPlanner::getRedunJointPosesForCartPoint(const Eigen::Affine3d& pose, RedunJointPoses& joint_poses,
                                                     const moveit::core::LinkModel* ee_link,
                                                     moveit::core::JointModelGroup* jmg, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "getRedunJointPosesForCartPoint()");

  EigenSTL::vector_Affine3d candidate_poses;
  if (!computeRedunPosesForCartPoint(pose, orientation_tol_, candidate_poses, true, indent))
    return false;

  // Get the IK solver for a given planning group
  const kinematics::KinematicsBasePtr& solver = jmg->getSolverInstance();

  // Discretization setting
  kinematics::KinematicsQueryOptions options;
  options.discretization_method = kinematics::DiscretizationMethods::DiscretizationMethod::ALL_DISCRETIZED;
  // options.discretization_method = kinematics::DiscretizationMethods::DiscretizationMethod::NO_DISCRETIZATION;

  // Enumerate solvable joint poses for each candidate_pose
  for (const Eigen::Affine3d& candidate_pose : candidate_poses)
  {
    RedunJointPoses local_joint_poses;

    // Bring the pose to the frame of the IK solver
    Eigen::Affine3d ik_query = candidate_pose;
    shared_robot_state0_->setToIKSolverFrame(ik_query, solver);

    // Convert pose to tip of ik solver
    // TODO: make this a helper function in robot_state
    const std::vector<std::string>& solver_tip_frames = solver->getTipFrames();
    BOOST_ASSERT_MSG(solver_tip_frames.size() == 1, "Currently only supports solvers for one tip");
    const std::string solver_tip_frame = solver_tip_frames.front();
    std::string query_frame = ee_link->getName();

    if (query_frame != solver_tip_frame)
    {
      const robot_model::LinkModel* lm = shared_robot_state0_->getLinkModel(query_frame);
      BOOST_ASSERT_MSG(lm, "No link model found for query frame (ee_link name)");

      const robot_model::LinkTransformMap& fixed_links = lm->getAssociatedFixedTransforms();
      for (robot_model::LinkTransformMap::const_iterator it = fixed_links.begin(); it != fixed_links.end(); ++it)
      {
        if (moveit::core::Transforms::sameFrame(it->first->getName(), solver_tip_frame))
        {
          query_frame = solver_tip_frame;
          ik_query = ik_query * it->second;
          break;
        }
      }
    }

    // Convert to msg
    geometry_msgs::Pose ik_query_msg;
    tf::poseEigenToMsg(ik_query, ik_query_msg);

    // Add to vector
    std::vector<geometry_msgs::Pose> ik_queries;
    ik_queries.push_back(ik_query_msg);

    // Create seed state
    JointSpacePoint ik_seed_state;
    JointSpacePoint initial_values;
    shared_robot_state0_->copyJointGroupPositions(jmg, initial_values);
    ik_seed_state.resize(initial_values.size());

    const std::vector<unsigned int>& bij = jmg->getKinematicsSolverJointBijection();

    for (std::size_t i = 0; i < bij.size(); ++i)
    {
      ik_seed_state[i] = initial_values[bij[i]];
    }

    // Solution
    RedunJointPoses solutions;
    kinematics::KinematicsResult kin_result;

    // Set discretization of redun joints
    solver->setSearchDiscretization(ik_solver_discretization_);

    // Solve
    if (!solver->getPositionIK(ik_queries, ik_seed_state, solutions, kin_result, options))
    {
      BOLT_DEBUG(indent, verbose_, "Failed to find a solution for a pose");

      // visual_tools_->publishZArrow(pose, rvt::YELLOW, rvt::MEDIUM);
      // visual_tools_->trigger();

      continue;
    }

    // Lock planning scene
    std::shared_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls =
      std::make_shared<planning_scene_monitor::LockedPlanningSceneRO>(parent_->getPlanningSceneMonitor());
    const planning_scene::PlanningScene* planning_scene =
      static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get();

    // Collision check each potential solution
    std::size_t states_rejected = 0;
    joint_poses.reserve(solutions.size());
    for (std::size_t i = 0; i < solutions.size(); ++i)
    {
      const JointSpacePoint& joint_state = solutions[i];

      // Copy joint values to a moveit robot state
      shared_robot_state1_->setJointGroupPositions(jmg, joint_state);

      // Check for validity
      shared_robot_state1_->update();

      if (!planning_scene->isStateValid(*shared_robot_state1_, jmg->getName(), verbose_collision_check_))
      {
        // BOLT_DEBUG(indent, true, "Invalid robot state");
        if (visualize_rejected_states_)
        {
          visual_tools_->publishRobotState(shared_robot_state1_, rvt::RED);
          parent_->waitForNextStep("invalid");
        }

        states_rejected++;
      }
      else
      {
        joint_poses.push_back(joint_state);
      }
    }  // for each solution

    const double percent = states_rejected / double(solutions.size()) * 100.0;
    BOLT_DEBUG(indent, verbose_, "Found " << joint_poses.size() << " redun poses for Cartesian point, rejected "
                                          << states_rejected << "  (" << percent << "%)");
  }
  return true;
}

void CartPathPlanner::visualizeAllJointPoses(const RedunJointPoses& joint_poses,
                                             const moveit::core::JointModelGroup* jmg, std::size_t indent)
{
  BOLT_FUNC(indent, true, "visualizeAllJointPoses() found " << joint_poses.size() << " joint_poses for this cartesian "
                                                                                     "point");

  for (const JointSpacePoint& joint_pose : joint_poses)
  {
    visual_tools_->publishRobotState(joint_pose, jmg);
    ros::Duration(visualize_all_solutions_sleep_).sleep();

    if (!ros::ok())
      exit(0);
  }
}

void CartPathPlanner::visualizeAllPointVertices(const TaskVertexMatrix& point_vertices, std::size_t indent)
{
  moveit_ompl::ModelBasedStateSpacePtr space = parent_->space_;

  for (std::size_t traj_id = 0; traj_id < point_vertices.size(); ++traj_id)
  {
    BOLT_DEBUG(indent, true, "traj_id: " << traj_id);

    // Get all vertices at this cartesian point
    const TaskVertexPoint& point_vertices1 = point_vertices[traj_id];

    // Step through each vertex in a cartesian point
    for (std::size_t vertex1_id = 0; vertex1_id < point_vertices1.size(); ++vertex1_id)
    {
      ompl::tools::bolt::TaskVertex v0 = point_vertices1[vertex1_id];
      std::cout << "vertex1_id: " << vertex1_id << " v0: " << v0 << std::endl;

      space->copyToRobotState(*shared_robot_state0_, task_graph_->getModelBasedState(v0));
      visual_tools2_->publishRobotState(shared_robot_state0_, rvt::RAND);
      // usleep(0.0001*1000000);
    }
  }
}

}  // namespace bolt_moveit
