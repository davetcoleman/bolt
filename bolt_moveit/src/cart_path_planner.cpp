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

namespace bolt_moveit
{
CartPathPlanner::CartPathPlanner(BoltMoveIt* parent) : nh_("~"), parent_(parent)
{
  std::size_t indent = 0;
  jmg_ = parent_->jmg_;

  // loading parameters
  {
    using namespace rosparam_shortcuts;
    ros::NodeHandle rpnh(nh_, name_);
    std::size_t error = 0;
    error += !get(name_, rpnh, "group_name", group_name_);
    //error += !get(name_, rpnh, "tip_link", tip_link_);
    error += !get(name_, rpnh, "base_link", base_link_);
    error += !get(name_, rpnh, "world_frame", world_frame_);
    error += !get(name_, rpnh, "trajectory_discretization", trajectory_discretization_);
    error += !get(name_, rpnh, "timing", timing_);
    error += !get(name_, rpnh, "tolerance_increment", tolerance_increment_);
    error += !get(name_, rpnh, "tolerance_roll", tolerance_roll_);
    error += !get(name_, rpnh, "tolerance_pitch", tolerance_pitch_);
    error += !get(name_, rpnh, "tolerance_yaw", tolerance_yaw_);
    error += !get(name_, rpnh, "ik_discretization", ik_discretization_);
    error += !get(name_, rpnh, "verbose", verbose_);
    error += !get(name_, rpnh, "visualize/show_all_solutions", visualize_show_all_solutions_);
    error += !get(name_, rpnh, "visualize/show_all_solutions_sleep", visualize_show_all_solutions_sleep_);
    error += !get(name_, rpnh, "visualize/show_all_cart_poses", visualize_show_all_cart_poses_);
    shutdownIfError(name_, error);
  }

  // Load planning state
  ik_state_.reset(new moveit::core::RobotState(*parent_->moveit_start_));

  // Get a pointer to the ee link
  ee_link_ = parent_->ee_link_; //ik_state_->getRobotModel()->getJointModelGroup(tip_link_);
  // if (!ee_link_)
  // {
  //   BOLT_ERROR(indent, true, "No joint model group found for " << tip_link_);
  //   exit(0);
  // }

  // Create cartesian start pose interactive marker
  imarker_cartesian_.reset(new mvt::IMarkerRobotState(parent_->getPlanningSceneMonitor(), "cart", jmg_,
                                                      ee_link_, rvt::BLUE, parent_->package_path_));
  imarker_cartesian_->setIMarkerCallback(
      std::bind(&CartPathPlanner::processIMarkerPose, this, std::placeholders::_1, std::placeholders::_2));

  // Set visual tools
  visual_tools_ = imarker_cartesian_->getVisualTools();
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

  // Trigger the first path viz
  generateExactPoses(indent);

  BOLT_INFO(indent, true, "CartPathPlanner Ready.");
}

void CartPathPlanner::processIMarkerPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback,
                                         const Eigen::Affine3d& feedback_pose)
{
  std::size_t indent = 2;
  moveit::core::RobotStatePtr imarker_state = imarker_cartesian_->getRobotState();
  Eigen::Affine3d start_pose = imarker_state->getGlobalLinkTransform(ee_link_);

  generateExactPoses(start_pose, indent);
}

bool CartPathPlanner::generateExactPoses(std::size_t indent)
{
  // Generate exact poses
  moveit::core::RobotStatePtr imarker_state = imarker_cartesian_->getRobotState();
  Eigen::Affine3d start_pose = imarker_state->getGlobalLinkTransform(ee_link_);

  generateExactPoses(start_pose, indent);
}

bool CartPathPlanner::generateExactPoses(const Eigen::Affine3d& start_pose, std::size_t indent)
{
  BOLT_FUNC(indent, true, "generateExactPoses()");

  if (!transform2DPath(start_pose, exact_poses_, indent))
  {
    BOLT_ERROR(indent, true, "Trajectory generation failed");
    exit(-1);
  }

  BOLT_DEBUG(indent, true, "Generated exact Cartesian traj with " << exact_poses_.size() << " points");

  // Publish trajectory poses for visualization
  visual_tools_->deleteAllMarkers();
  visual_tools_->publishPath(exact_poses_, rvt::ORANGE, rvt::XXSMALL);
  visual_tools_->publishAxisPath(exact_poses_, rvt::XXXSMALL);
  visual_tools_->trigger();

  // Specify tolerance for new exact_poses
  orientation_tol_ = OrientationTol(tolerance_roll_, tolerance_pitch_, tolerance_yaw_);
  // orientation_tol_ = OrientationTol(M_PI / 5.0, M_PI / 5.0, M_PI);

  if (visualize_show_all_solutions_)
    debugShowAllIKSolutions(indent);

  return true;
}

bool CartPathPlanner::debugShowAllIKSolutions(std::size_t indent)
{
  BOLT_FUNC(indent, true, "debugShowAllIKSolutions()");
  std::size_t total_redundant_joint_poses = 0;

  // Enumerate the potential poses within tolerance
  for (std::size_t i = 0; i < exact_poses_.size(); ++i)
  {
    const Eigen::Affine3d& pose = exact_poses_[i];

    JointPoses local_joint_poses;
    if (!getRedundantJointPosesForCartPoint(pose, local_joint_poses, indent))
    {
      BOLT_ERROR(indent, true, "Error when getting joint poses for cartesian point");
      continue;
    }

    // Handle error: no IK solutions found
    if (local_joint_poses.empty())
    {
      BOLT_ERROR(indent, true, "No joint solutions found for cartesian pose " << i);

      visual_tools_->publishAxis(pose, rvt::XXSMALL);
      visual_tools_->trigger();

      return false;
    }

    // Show all possible configurations
    visualizeAllJointPoses(local_joint_poses, indent);

    total_redundant_joint_poses += local_joint_poses.size();
  }
  BOLT_INFO(indent, true, "Found " << total_redundant_joint_poses << " total redundant joint poses");

  return true;
}

bool CartPathPlanner::computeRedundantPosesForCartPoint(const Eigen::Affine3d& pose,
                                                        const OrientationTol& orientation_tol,
                                                        EigenSTL::vector_Affine3d& candidate_poses, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "computeRedundantPosesForCartPoint()");
  BOOST_ASSERT_MSG(tolerance_increment_ != 0, "Divide by zero using orientation increment");

  const std::size_t num_axis = 3;

  // Reserve vector size
  std::vector<size_t> num_steps_per_axis(num_axis, 0 /* value */);
  for (std::size_t i = 0; i < num_axis; ++i)
  {
    double range = 2 * orientation_tol.axis_dist_from_center_[i];
    num_steps_per_axis[i] = std::max(1.0, ceil(range / tolerance_increment_));
  }
  double total_num_steps = num_steps_per_axis[0] * num_steps_per_axis[1] * num_steps_per_axis[2];
  BOLT_DEBUG(indent, verbose_, "Generating " << total_num_steps << " under-constrained poses for this "
                                                               "cartesian point");
  candidate_poses.reserve(total_num_steps);

  // Start recursion
  bool result = rotateOnAxis(pose, orientation_tol, X_AXIS, candidate_poses, indent);

  if (visualize_show_all_cart_poses_)
  {
    for (const Eigen::Affine3d& candidate_pose : candidate_poses)
    {
      visual_tools_->publishZArrow(candidate_pose, rvt::BLUE, rvt::SMALL);
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
    BOLT_DEBUG(indent, verbose, std::string(axis * 2, ' ') << "axis: " << axis << " i: " << i
               << " rotation_amount: " << rotation_amount << " num_steps: " << num_steps);

    // clang-format off
    switch (axis)
    {
      case X_AXIS: new_pose = pose * Eigen::AngleAxisd(rotation_amount, Eigen::Vector3d::UnitX()); break;
      case Y_AXIS: new_pose = pose * Eigen::AngleAxisd(rotation_amount, Eigen::Vector3d::UnitY()); break;
      case Z_AXIS: new_pose = pose * Eigen::AngleAxisd(rotation_amount, Eigen::Vector3d::UnitZ()); break;
      default: BOLT_ERROR(indent, true, "Unknown axis");
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

bool CartPathPlanner::transform2DPath(const Eigen::Affine3d& starting_pose, EigenSTL::vector_Affine3d& exact_poses,
                                      std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "transform2DPath()");
  exact_poses.clear();

  if (path_from_file_.empty())
  {
    BOLT_ERROR(indent, true, "Unable to create drawing: no path loaded from file");
    return false;
  }
  if (path_from_file_.size() == 1)
  {
    BOLT_ERROR(indent, true, "Unable to create drawing: path only has 1 point");
    return false;
  }

  // Transform each point read from file
  EigenSTL::vector_Affine3d transformed_poses;
  for (std::size_t i = 0; i < path_from_file_.size(); ++i)
  {
    Eigen::Affine3d point = starting_pose * path_from_file_[i];

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
  // Benchmark runtime
  ros::Time start_time = ros::Time::now();

  task_graph_ = task_graph;  // copy pointer into this class to share among all functions

  // Make sure exact poses are available
  if (exact_poses_.empty())
  {
    BOLT_WARN(indent, true, "No exact poses had been previously generated. Creating.");

    // Generating trajectory
    if (!generateExactPoses(indent))
      return false;
  }

  if (exact_poses_.size() < 2)
  {
    BOLT_ERROR(indent, true, "Must have at least 2 poses in trajectory");
    return false;
  }

  // Remove any previous Cartesian vertices/edges by simply re-creating the whole task graph
  task_graph_->generateTaskSpace(indent);

  // Track all created vertices - for each pt in the cartesian trajectory, contains multiple vertices
  TaskVertexMatrix graph_vertices;
  graph_vertices.resize(exact_poses_.size());

  ompl::tools::bolt::TaskVertex startingVertex = task_graph_->getNumVertices() - 1;

  // For converting to MoveIt! format
  moveit::core::RobotStatePtr moveit_robot_state(new moveit::core::RobotState(*visual_tools_->getSharedRobotState()));

  // Enumerate the potential cartesian poses within tolerance
  std::size_t total_vertices = 0;
  for (std::size_t traj_id = 0; traj_id < exact_poses_.size(); ++traj_id)
  {
    const Eigen::Affine3d& pose = exact_poses_[traj_id];

    // Calculate all possible joint solutions
    JointPoses joint_poses;
    getRedundantJointPosesForCartPoint(pose, joint_poses, indent);

    // Handle error: no IK solutions found
    if (joint_poses.empty())
    {
      BOLT_ERROR(indent, true, "No joint solutions found for pose " << traj_id);

      visual_tools_->publishAxis(pose, rvt::XXSMALL);
      visual_tools_->trigger();

      // Show last valid pose if possible
      if (traj_id > 0)
      {
        joint_poses.clear();  // reset vector

        // Get the joint poses from the last cartesian point
        getRedundantJointPosesForCartPoint(exact_poses_[traj_id - 1], joint_poses, indent);
        BOOST_ASSERT_MSG(!joint_poses.empty(), "Should not happen - no joint poses found for previous cartesian point");
        visual_tools_->publishRobotState(joint_poses.front(), jmg_, rvt::RED);
      }

      return false;
    }

    // Debug:: Show all possible configurations
    if (false)
      visualizeAllJointPoses(joint_poses, indent);

    // Stats
    total_vertices += graph_vertices[traj_id].size();

    // Convert all possible configurations into the Bolt graph
    if (!addCartPointToBoltGraph(joint_poses, graph_vertices[traj_id], moveit_robot_state, indent))
    {
      BOLT_ERROR(indent, true, "Failed to add all joint configurations to Bolt graph");
      return false;
    }
  }
  BOLT_DEBUG(indent, true, "Added " << total_vertices << " total vertices to the task graph");

  ompl::tools::bolt::TaskVertex endingVertex = task_graph_->getNumVertices() - 1;
  //(void)endingVertex;  // prevent unused variable warning

  // ---------------------------------------------------------------
  // Add edges
  if (!addEdgesToBoltGraph(graph_vertices, startingVertex, endingVertex, indent))
  {
    BOLT_ERROR(indent, true, "Error creating edges");
    return false;
  }

  // ---------------------------------------------------------------

  // Track the shortest straight-line cost across any pair of start/goal points
  double shortest_path_across_cart = std::numeric_limits<double>::infinity();
  if (!connectTrajectoryEndPoints(graph_vertices, shortest_path_across_cart, indent))
  {
    BOLT_ERROR(indent, true, "Unable to connect trajectory end points!");
    return false;
  }

  // Set the shortest path across cartesian graph in the TaskGraph
  task_graph_->setShortestDistAcrossCart(shortest_path_across_cart);

  // Tell the planner to require task planning
  task_graph_->setTaskPlanningEnabled();

  // Benchmark runtime
  double duration = (ros::Time::now() - start_time).toSec();
  task_graph_->printGraphStats(duration);

  return true;
}

bool CartPathPlanner::addCartPointToBoltGraph(const JointPoses& joint_poses,
                                              std::vector<ompl::tools::bolt::TaskVertex>& point_vertices,
                                              moveit::core::RobotStatePtr moveit_robot_state, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "addCartPointToBoltGraph()");

  moveit_ompl::ModelBasedStateSpacePtr space = parent_->space_;

  std::size_t new_vertex_count = 0;
  const ompl::tools::bolt::VertexType vertex_type = ompl::tools::bolt::CARTESIAN;
  const ompl::tools::bolt::VertexLevel level = 1;  // middle layer

  point_vertices.resize(joint_poses.size());
  for (std::size_t i = 0; i < joint_poses.size(); ++i)
  {
    const std::vector<double>& joints_pose = joint_poses[i];

    // Copy vector into moveit format
    moveit_robot_state->setJointGroupPositions(jmg_, joints_pose);

    // Create new OMPL state
    ompl::base::State* ompl_state = space->allocState();

    // Convert the MoveIt! robot state into an OMPL state
    space->copyToOMPLState(ompl_state, *moveit_robot_state);

    // Add vertex to task graph
    point_vertices[i] = task_graph_->addVertex(ompl_state, vertex_type, level, indent);

    new_vertex_count++;
  }

  BOLT_DEBUG(indent, verbose_, "Added " << new_vertex_count << " new vertices to TaskGraph");

  return true;
}

bool CartPathPlanner::addEdgesToBoltGraph(const TaskVertexMatrix& graph_vertices,
                                          ompl::tools::bolt::TaskVertex startingVertex,
                                          ompl::tools::bolt::TaskVertex endingVertex, std::size_t indent)
{
  BOLT_FUNC(indent, true, "addEdgesToBoltGraph()");

  moveit_ompl::ModelBasedStateSpacePtr space = parent_->space_;

  // Iterate to create edges
  std::size_t new_edge_count = 0;
  std::size_t edges_skipped_count = 0;
  const ompl::tools::bolt::EdgeType edge_type = ompl::tools::bolt::eCARTESIAN;

  // Step through each cartesian point, starting at second point
  for (std::size_t traj_id = 1; traj_id < exact_poses_.size(); ++traj_id)
  {
    // Get all vertices at this cartesian point
    const std::vector<ompl::tools::bolt::TaskVertex>& point_vertices1 = graph_vertices[traj_id];

    // Step through each vertex in a cartesian point
    for (std::size_t vertex1_id = 0; vertex1_id < point_vertices1.size(); ++vertex1_id)
    {
      ompl::tools::bolt::TaskVertex v1 = point_vertices1[vertex1_id];

      // Get all vertices at previous cartesian point
      const std::vector<ompl::tools::bolt::TaskVertex>& point_vertices0 = graph_vertices[traj_id - 1];

      // Connect to every vertex in *previous* cartesian point
      for (std::size_t vertex0_id = 0; vertex0_id < point_vertices0.size(); ++vertex0_id)
      {
        ompl::tools::bolt::TaskVertex v0 = point_vertices0[vertex0_id];

        // Create edge
        BOOST_ASSERT_MSG(v1 > startingVertex && v1 <= endingVertex, "Attempting to create edge with out of range "
                                                                    "vertex");
        BOOST_ASSERT_MSG(v0 > startingVertex && v0 <= endingVertex, "Attempting to create edge with out of range "
                                                                    "vertex");

        // Convert OMPL state to std::vector
        const double* start_array =
            task_graph_->getState(v0)->as<moveit_ompl::ModelBasedStateSpace::StateType>()->values;
        const double* end_array = task_graph_->getState(v1)->as<moveit_ompl::ModelBasedStateSpace::StateType>()->values;

        // Attempt to eliminate edge based on the ability to achieve joint motion is possible in the window provided
        // if (!ur5_robot_model_->isValidMove(start_array, end_array, space->getDimension(), timing_))
        // {
        //   edges_skipped_count++;
        //   continue;
        // }
        if (!ik_state_->isValidVelocityMove(jmg_, start_array, end_array, space->getDimension(), timing_))
        {
          edges_skipped_count++;
          continue;
        }

        // BOLT_DEBUG(indent, true, "Adding edge " << v0 << " to " << v1);
        task_graph_->addEdge(v0, v1, edge_type, indent);

        new_edge_count++;
      }  // for
    }    // for
    // BOLT_DEBUG(indent, true, "Point " << traj_id << " current edge count: " << new_edge_count);
    if (!ros::ok())
      exit(0);
  }  // for
  BOLT_DEBUG(indent, true, "Added " << new_edge_count << " new edges, rejected " << edges_skipped_count << " (rejected "
             << edges_skipped_count / (new_edge_count + double(edges_skipped_count)) * 100.0 << "%)");

  std::size_t warning_factor = 4;
  if (edges_skipped_count * warning_factor > new_edge_count)
    BOLT_WARN(indent, true, "More than " << warning_factor << " times as many edges were rejected than accepted "
                                                              "because of motion timing/velocity contraints. "
                                                              "Consider tweaking.");

  return true;
}

bool CartPathPlanner::connectTrajectoryEndPoints(const TaskVertexMatrix& graph_vertices,
                                                 double& shortest_path_across_cart, std::size_t indent)
{
  BOLT_FUNC(indent, true, "connectTrajectoryEndPoints()");

  // force visualization
  // task_graph_->visualizeTaskGraph_ = true;

  // Get all vertices at this starting and ending points
  const std::vector<ompl::tools::bolt::TaskVertex>& start_vertices = graph_vertices.front();
  const std::vector<ompl::tools::bolt::TaskVertex>& goal_vertices = graph_vertices.back();

  // Loop through all start points
  BOLT_INFO(indent, true, "Connecting Cartesian start points to TaskGraph");
  for (const ompl::tools::bolt::TaskVertex& start_vertex : start_vertices)
  {
    // Connect to TaskGraph
    const ompl::tools::bolt::VertexLevel level0 = 0;
    bool isStart = true;
    if (!task_graph_->connectVertexToNeighborsAtLevel(start_vertex, level0, isStart, indent))
    {
      OMPL_WARN("Failed to connect start vertex");
      return false;  // TODO(davetcoleman): return here?
    }

    // Calculate the shortest straight-line distance across graph
    // Record min cost for cost-to-go heurstic distance function later
    // Check if this start vertex has has the shortest path across the Cartesian graph
    for (const ompl::tools::bolt::TaskVertex& goal_vertex : goal_vertices)
    {
      double distance_across_graph = task_graph_->distanceFunction(start_vertex, goal_vertex);

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
      OMPL_WARN("Failed to connect goal vertex");
      return false;  // TODO(davetcoleman): return here?
    }

    if (!ros::ok())
      exit(-1);
  }
  BOLT_INFO(indent, true, "Finished connecting Cartesian end points to TaskGraph");

  return true;
}

bool CartPathPlanner::getRedundantJointPosesForCartPoint(const Eigen::Affine3d& pose, JointPoses& joint_poses,
                                                         std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "getRedundantJointPosesForCartPoint()");

  EigenSTL::vector_Affine3d candidate_poses;
  if (!computeRedundantPosesForCartPoint(pose, orientation_tol_, candidate_poses, indent))
    return false;

  // Get the IK solver for a given planning group
  const kinematics::KinematicsBasePtr& solver = jmg_->getSolverInstance();

  // Enumerate solvable joint poses for each candidate_pose
  for (const Eigen::Affine3d& candidate_pose : candidate_poses)
  {
    JointPoses local_joint_poses;

    // Bring the pose to the frame of the IK solver
    Eigen::Affine3d ik_query = candidate_pose;
    ik_state_->setToIKSolverFrame(ik_query, solver);

    // Convert pose to tip of ik solver
    // TODO: make this a helper function in robot_state
    const std::vector<std::string> &solver_tip_frames = solver->getTipFrames();
    BOOST_ASSERT_MSG(solver_tip_frames.size() == 1, "Currently only supports solvers for one tip");
    const std::string solver_tip_frame = solver_tip_frames.front();
    std::string query_frame = ee_link_->getName();

    if (query_frame != solver_tip_frame)
    {
      const robot_model::LinkModel *lm = ik_state_->getLinkModel(query_frame);
      BOOST_ASSERT_MSG(lm, "No link model found for query frame (ee_link name)");

      const robot_model::LinkTransformMap &fixed_links = lm->getAssociatedFixedTransforms();
      for (robot_model::LinkTransformMap::const_iterator it = fixed_links.begin() ; it != fixed_links.end() ; ++it)
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
    std::vector<double> ik_seed_state;
    std::vector<double> initial_values;
    ik_state_->copyJointGroupPositions(jmg_, initial_values);
    ik_seed_state.resize(initial_values.size());

    const std::vector<unsigned int>& bij = jmg_->getKinematicsSolverJointBijection();

    for (std::size_t i = 0; i < bij.size(); ++i)
    {
      ik_seed_state[i] = initial_values[bij[i]];
    }

    // Discretization setting
    kinematics::KinematicsQueryOptions options;
    options.discretization_method = kinematics::DiscretizationMethods::DiscretizationMethod::ALL_DISCRETIZED;

    // Solution
    JointPoses solutions;
    kinematics::KinematicsResult kin_result;

    // Set discretization of redundant joints
    solver->setSearchDiscretization(ik_discretization_);

    // Solve
    bool result = solver->getPositionIK(ik_queries, ik_seed_state, solutions, kin_result, options);
    if (result)
    {
      BOLT_DEBUG(indent, verbose_, "Found " << solutions.size() << " redundant poses for Cartesian point");
      joint_poses.insert(joint_poses.end(), solutions.begin(), solutions.end());
    }
    else if (false)  // debug mode
    {
      BOLT_WARN(indent, true, "Failed to find a solution for a pose");

      visual_tools_->publishZArrow(ik_queries.front(), rvt::RED, rvt::MEDIUM);
      visual_tools_->trigger();
    }
  }
  return true;
}

void CartPathPlanner::visualizeAllJointPoses(const JointPoses& joint_poses, std::size_t indent)
{
  BOLT_FUNC(indent, true, "visualizeAllJointPoses()");

  for (std::vector<double> joint_pose : joint_poses)
  {
    visual_tools_->publishRobotState(joint_pose, jmg_);
    ros::Duration(visualize_show_all_solutions_sleep_).sleep();

    if (!ros::ok())
      exit(0);
  }
}

}  // namespace bolt_moveit
