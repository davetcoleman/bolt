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
#include <curie_demos/cart_path_planner.h>
#include <curie_demos/curie_demos.h>
#include <curie_demos/path_loader.h>

// moveit_boilerplate
#include <moveit_boilerplate/namespaces.h>

namespace curie_demos
{
CartPathPlanner::CartPathPlanner(CurieDemos* parent) : name_("cart_path_planner"), nh_("~"), parent_(parent)
{
  jmg_ = parent_->jmg_;

  // Load planning state
  imarker_state_.reset(new moveit::core::RobotState(*parent_->moveit_start_));

  // Create cartesian start pose interactive marker
  imarker_cartesian_.reset(new mvt::IMarkerRobotState(parent_->getPlanningSceneMonitor(), "cart", jmg_,
                                                      parent_->ee_link_, rvt::BLUE, parent_->package_path_));
  imarker_cartesian_->setIMarkerCallback(
      std::bind(&CartPathPlanner::processIMarkerPose, this, std::placeholders::_1, std::placeholders::_2));

  // Set visual tools
  visual_tools_ = imarker_cartesian_->getVisualTools();
  visual_tools_->setMarkerTopic(nh_.getNamespace() + "/cartesian_trajectory");
  visual_tools_->loadMarkerPub(true);
  visual_tools_->deleteAllMarkers();
  visual_tools_->setManualSceneUpdating(true);
  visual_tools_->loadTrajectoryPub(nh_.getNamespace() + "/display_trajectory");

  // Load Descartes ------------------------------------------------

  // loading parameters
  ros::NodeHandle rpnh(nh_, name_);
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, rpnh, "group_name", group_name_);
  error += !rosparam_shortcuts::get(name_, rpnh, "tip_link", tip_link_);
  error += !rosparam_shortcuts::get(name_, rpnh, "base_link", base_link_);
  error += !rosparam_shortcuts::get(name_, rpnh, "world_frame", world_frame_);
  error += !rosparam_shortcuts::get(name_, rpnh, "descartes_check_collisions", descartes_check_collisions_);
  error += !rosparam_shortcuts::get(name_, rpnh, "orientation_increment", orientation_increment_);
  error += !rosparam_shortcuts::get(name_, rpnh, "trajectory_discretization", trajectory_discretization_);
  error += !rosparam_shortcuts::get(name_, rpnh, "timing", timing_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // initializing descartes
  initDescartes();

  // Load desired path
  PathLoader path_loader(parent_->package_path_);
  const bool debug = false;
  if (!path_loader.get2DPath(path_, debug))
    exit(0);

  // Trigger the first path viz
  generateExactPoses(/*debug*/ false);

  ROS_INFO_STREAM_NAMED(name_, "CartPathPlanner Ready.");
}

void CartPathPlanner::initDescartes()
{
  ROS_INFO_STREAM_NAMED(name_, "initDescartes()");

  // Instantiating a robot model
  const std::string prefix = "right_";
  ur5_robot_model_.reset(new ur5_demo_descartes::UR5RobotModel(prefix));

  // Initialize
  if (!ur5_robot_model_->initialize(visual_tools_->getSharedRobotState()->getRobotModel(), group_name_, world_frame_,
                                    tip_link_))
  {
    ROS_ERROR_STREAM("Failed to initialize Robot Model");
    exit(-1);
  }
  ROS_INFO_STREAM("Descartes Robot Model initialized");

  // Set collision checking.
  ur5_robot_model_->setCheckCollisions(descartes_check_collisions_);
  if (!descartes_check_collisions_)
    ROS_INFO_STREAM_NAMED(name_, "Descartes collision checking disabled");

  // if (!planner_.initialize(ur5_robot_model_))
  // {
  //   ROS_ERROR_STREAM("Failed to initialize Dense Planner");
  //   exit(-1);
  // }

  ROS_INFO_STREAM("Descartes Dense Planner initialized");
}

void CartPathPlanner::processIMarkerPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback,
                                         const Eigen::Affine3d& feedback_pose)
{
  imarker_state_ = imarker_cartesian_->getRobotState();
  Eigen::Affine3d start_pose = imarker_state_->getGlobalLinkTransform(parent_->ee_link_);

  generateExactPoses(start_pose);
}

bool CartPathPlanner::generateExactPoses(bool debug)
{
  // Generate exact poses
  imarker_state_ = imarker_cartesian_->getRobotState();
  Eigen::Affine3d start_pose = imarker_state_->getGlobalLinkTransform(parent_->ee_link_);

  generateExactPoses(start_pose, debug);
}

bool CartPathPlanner::generateExactPoses(const Eigen::Affine3d& start_pose, bool debug)
{
  // ROS_DEBUG_STREAM_NAMED(name_, "generateExactPoses()");
  if (debug)
    ROS_WARN_STREAM_NAMED(name_, "Running generateExactPoses() in debug mode");

  if (!transform2DPath(start_pose, exact_poses_))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Trajectory generation failed");
    exit(-1);
  }

  ROS_DEBUG_STREAM_NAMED(name_ + ".generation", "Generated exact Cartesian traj with " << exact_poses_.size() << " poin"
                                                                                                                 "ts");

  // Publish trajectory poses for visualization
  visual_tools_->deleteAllMarkers();
  visual_tools_->publishPath(exact_poses_, rvt::ORANGE, rvt::XXSMALL);
  visual_tools_->publishAxisPath(exact_poses_, rvt::XXXSMALL);
  visual_tools_->triggerBatchPublish();

  // Specify tolerance for new exact_poses
  // orientation_tol_ = OrientationTol(M_PI, 0, 0);
  orientation_tol_ = OrientationTol(M_PI, M_PI / 5, M_PI / 5);
  // rosparam timing_ = 0.1;

  if (debug)
    debugShowAllIKSolutions();

  return true;
}

bool CartPathPlanner::debugShowAllIKSolutions()
{
  // Enumerate the potential poses within tolerance
  for (std::size_t i = 0; i < exact_poses_.size(); ++i)
  {
    const Eigen::Affine3d& pose = exact_poses_[i];

    EigenSTL::vector_Affine3d candidate_poses;
    computeAllPoses(pose, orientation_tol_, candidate_poses);

    std::vector<std::vector<double>> joint_poses;
    for (const Eigen::Affine3d& candidate_pose : candidate_poses)
    {
      std::vector<std::vector<double>> local_joint_poses;
      if (ur5_robot_model_->getAllIK(candidate_pose, local_joint_poses))
      {
        joint_poses.insert(joint_poses.end(), local_joint_poses.begin(), local_joint_poses.end());
      }
    }

    // Handle error: no IK solutions found
    if (joint_poses.empty())
    {
      ROS_ERROR_STREAM_NAMED(name_, "No joint solutions found for pose " << i);

      visual_tools_->publishAxis(pose, rvt::XXSMALL);
      visual_tools_->triggerBatchPublish();

      return false;
    }

    // Show all possible configurations
    visualizeAllJointPoses(joint_poses);
  }

  return true;
}

bool CartPathPlanner::computeAllPoses(const Eigen::Affine3d& pose, const OrientationTol& orientation_tol,
                                      EigenSTL::vector_Affine3d& candidate_poses)
{
  BOOST_ASSERT_MSG(orientation_increment_ != 0, "Divide by zero using orientation increment");

  const std::size_t num_axis = 3;

  // Reserve vector size
  std::vector<size_t> num_steps_per_axis(num_axis, 0 /* value */);
  for (std::size_t i = 0; i < num_axis; ++i)
  {
    double range = 2 * orientation_tol.axis_dist_from_center_[i];
    num_steps_per_axis[i] = std::max(1.0, ceil(range / orientation_increment_));
  }
  double total_num_steps = num_steps_per_axis[0] * num_steps_per_axis[1] * num_steps_per_axis[2];
  ROS_DEBUG_STREAM_NAMED(name_ + ".generation", "Generating " << total_num_steps << " under-constrained poses for this "
                                                                                    "cartesian point");
  candidate_poses.reserve(total_num_steps);

  // Start recursion
  bool result = rotateOnAxis(pose, orientation_tol, X_AXIS, candidate_poses);

  BOOST_ASSERT_MSG(candidate_poses.size() == total_num_steps, "Estimated poses should be same as actual poses");
  return result;
}

bool CartPathPlanner::rotateOnAxis(const Eigen::Affine3d& pose, const OrientationTol& orientation_tol, const Axis axis,
                                   EigenSTL::vector_Affine3d& candidate_poses)
{
  const bool verbose = false;
  double range = 2 * orientation_tol.axis_dist_from_center_[axis];
  std::size_t num_steps = std::max(1.0, ceil(range / orientation_increment_));
  Eigen::Affine3d new_pose;

  // Rotate all around one axis
  for (int i = num_steps * -0.5; i < num_steps * 0.5; ++i)
  {
    double rotation_amount = i * orientation_increment_;
    if (verbose)
      std::cout << std::string(axis * 2, ' ') << "axis: " << axis << " i: " << i
                << " rotation_amount: " << rotation_amount << " num_steps: " << num_steps << std::endl;

    // clang-format off
    switch (axis)
    {
      case X_AXIS: new_pose = pose * Eigen::AngleAxisd(rotation_amount, Eigen::Vector3d::UnitX()); break;
      case Y_AXIS: new_pose = pose * Eigen::AngleAxisd(rotation_amount, Eigen::Vector3d::UnitY()); break;
      case Z_AXIS: new_pose = pose * Eigen::AngleAxisd(rotation_amount, Eigen::Vector3d::UnitZ()); break;
      default: ROS_ERROR_STREAM_NAMED(name_, "Unknown axis");
    }
    // clang-format on

    // Recursively rotate
    if (axis < Z_AXIS)
    {
      rotateOnAxis(new_pose, orientation_tol, static_cast<Axis>(axis + 1), candidate_poses);
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

bool CartPathPlanner::transform2DPath(const Eigen::Affine3d& starting_pose, EigenSTL::vector_Affine3d& poses)
{
  poses.clear();

  if (path_.empty())
  {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to create drawing: no path loaded from file");
    return false;
  }
  if (path_.size() == 1)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to create drawing: path only has 1 point");
    return false;
  }

  // Transform each point read from file
  EigenSTL::vector_Affine3d transformed_poses;
  for (std::size_t i = 0; i < path_.size(); ++i)
  {
    Eigen::Affine3d point = starting_pose * path_[i];

    // Rotate 90 so that the x axis points down
    //  point = point * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());

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
      poses.push_back(temp_pose);
    }
  }

  return true;
}

bool CartPathPlanner::populateBoltGraph(ompl::tools::bolt::TaskGraphPtr task_graph)
{
  std::size_t indent = 0;
  task_graph_ = task_graph;  // copy into this class to share among all functions

  // Make sure exact poses are available
  if (exact_poses_.empty())
  {
    ROS_WARN_STREAM_NAMED(name_, "No exact poses had been previously generated. Creating.");

    // Generating trajectory
    bool debug = false;
    if (!generateExactPoses(debug))
      return false;
  }

  if (exact_poses_.size() < 2)
  {
    ROS_ERROR_STREAM_NAMED(name_, "Must have at least 2 poses in trajectory");
    return false;
  }

  // Remove any previous Cartesian vertices/edges by simply re-creating the whole task graph
  task_graph_->generateTaskSpace(indent);

  // Track all created vertices - for each pt in the cartesian trajectory, contains multiple vertices
  TrajectoryGraph graph_vertices;
  graph_vertices.resize(exact_poses_.size());

  ompl::tools::bolt::TaskVertex startingVertex = task_graph_->getNumVertices() - 1;
  //(void)startingVertex;  // prevent unused variable warning

  // For converting to MoveIt! format
  moveit::core::RobotStatePtr moveit_robot_state(new moveit::core::RobotState(*visual_tools_->getSharedRobotState()));

  // Enumerate the potential cartesian poses within tolerance
  std::size_t total_vertices = 0;
  for (std::size_t traj_id = 0; traj_id < exact_poses_.size(); ++traj_id)
  {
    const Eigen::Affine3d& pose = exact_poses_[traj_id];

    // Calculate all possible joint solutions
    std::vector<std::vector<double>> joint_poses;
    getAllJointPosesForCartPoint(pose, joint_poses);

    // Handle error: no IK solutions found
    if (joint_poses.empty())
    {
      ROS_ERROR_STREAM_NAMED(name_, "No joint solutions found for pose " << traj_id);

      visual_tools_->publishAxis(pose, rvt::XXSMALL);
      visual_tools_->triggerBatchPublish();

      // Show last valid pose if possible
      if (traj_id > 0)
      {
        joint_poses.clear();  // reset vector

        // Get the joint poses from the last cartesian point
        getAllJointPosesForCartPoint(exact_poses_[traj_id - 1], joint_poses);
        BOOST_ASSERT_MSG(!joint_poses.empty(), "Should not happen - no joint poses found for previous cartesian point");
        visual_tools_->publishRobotState(joint_poses.front(), jmg_, rvt::RED);
      }

      return false;
    }

    // Debug:: Show all possible configurations
    if (false)
      visualizeAllJointPoses(joint_poses);

    // Convert all possible configurations into the Bolt graph
    if (!addCartPointToBoltGraph(joint_poses, graph_vertices[traj_id], moveit_robot_state))
    {
      ROS_ERROR_STREAM_NAMED(name_, "Failed to add all joint configurations to Bolt graph");
      return false;
    }

    total_vertices += graph_vertices[traj_id].size();
  }
  ROS_DEBUG_STREAM_NAMED(name_, "Generated " << total_vertices << " total vertices in graph");

  ompl::tools::bolt::TaskVertex endingVertex = task_graph_->getNumVertices() - 1;
  //(void)endingVertex;  // prevent unused variable warning

  // ---------------------------------------------------------------
  // Add edges
  if (!addEdgesToBoltGraph(graph_vertices, startingVertex, endingVertex))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Error creating edges");
    return false;
  }

  // ---------------------------------------------------------------
  // Connect Descartes graph to Bolt graph

  // Track the shortest straight-line cost across any pair of start/goal points
  double shortest_path_across_cart = std::numeric_limits<double>::infinity();
  if (!connectTrajectoryEndPoints(graph_vertices, shortest_path_across_cart))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to connect trajectory end points!");
    return false;
  }

  // Set the shortest path across cartesian graph in the TaskGraph
  task_graph_->setShortestDistAcrossCart(shortest_path_across_cart);

  // Tell the planner to require task planning
  task_graph_->setTaskPlanningEnabled();

  task_graph_->printGraphStats();

  return true;
}

bool CartPathPlanner::addCartPointToBoltGraph(const std::vector<std::vector<double>>& joint_poses,
                                              std::vector<ompl::tools::bolt::TaskVertex>& point_vertices,
                                              moveit::core::RobotStatePtr moveit_robot_state)
{
  std::size_t indent = 0;
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

  ROS_DEBUG_STREAM_NAMED(name_ + ".generation", "Added " << new_vertex_count << " new vertices to TaskGraph");

  return true;
}

bool CartPathPlanner::addEdgesToBoltGraph(const TrajectoryGraph& graph_vertices,
                                          ompl::tools::bolt::TaskVertex startingVertex,
                                          ompl::tools::bolt::TaskVertex endingVertex)
{
  ROS_INFO_STREAM_NAMED(name_, "addEdgesToBoltGraph()");
  std::size_t indent = 0;

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
        if (!ur5_robot_model_->isValidMove(start_array, end_array, space->getDimension(), timing_))
        {
          edges_skipped_count++;
          continue;
        }

        // ROS_DEBUG_STREAM_NAMED(name_, "Adding edge " << v0 << " to " << v1);
        task_graph_->addEdge(v0, v1, edge_type, indent);

        new_edge_count++;
      }  // for
    }    // for
    ROS_DEBUG_STREAM_NAMED(name_ + ".generation", "Point " << traj_id << " current edge count: " << new_edge_count);
    if (!ros::ok())
      exit(0);
  }  // for
  ROS_DEBUG_STREAM_NAMED(name_, "Added " << new_edge_count << " new edges, rejected " << edges_skipped_count);

  std::size_t warning_factor = 4;
  if (edges_skipped_count * warning_factor > new_edge_count)
    ROS_WARN_STREAM_NAMED(name_, "More than " << warning_factor << " times as many edges were rejected than accepted "
                                                                   "because of motion timing/velocity contraints. "
                                                                   "Consider tweaking.");

  return true;
}

bool CartPathPlanner::connectTrajectoryEndPoints(const TrajectoryGraph& graph_vertices,
                                                 double& shortest_path_across_cart)
{
  std::size_t indent = 0;
  // force visualization
  // task_graph_->visualizeTaskGraph_ = true;

  // Get all vertices at this starting and ending points
  const std::vector<ompl::tools::bolt::TaskVertex>& start_vertices = graph_vertices.front();
  const std::vector<ompl::tools::bolt::TaskVertex>& goal_vertices = graph_vertices.back();

  // Loop through all start points
  ROS_INFO_STREAM_NAMED(name_, "Connecting Cartesian start points to TaskGraph");
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

    // Calculate the shortest straight-line distance across Descartes graph
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
  ROS_INFO_STREAM_NAMED(name_, "Connecting Cartesian end points to TaskGraph");
  for (const ompl::tools::bolt::TaskVertex& goal_vertex : goal_vertices)
  {
    // Connect to TaskGraph
    const ompl::tools::bolt::VertexLevel level2 = 2;
    bool isStart = false;
    if (!task_graph_->connectVertexToNeighborsAtLevel(goal_vertex, level2, isStart, indent))
    {
      OMPL_WARN("Failed to connect Descartes goal vertex");
      return false;  // TODO(davetcoleman): return here?
    }

    if (!ros::ok())
      exit(-1);
  }
  ROS_INFO_STREAM_NAMED(name_, "Finished connecting Cartesian end points to TaskGraph");

  return true;
}

bool CartPathPlanner::getAllJointPosesForCartPoint(const Eigen::Affine3d& pose,
                                                   std::vector<std::vector<double>>& joint_poses)
{
  EigenSTL::vector_Affine3d candidate_poses;
  if (!computeAllPoses(pose, orientation_tol_, candidate_poses))
    return false;

  // Enumerate solvable joint poses for each candidate_pose
  for (const Eigen::Affine3d& candidate_pose : candidate_poses)
  {
    std::vector<std::vector<double>> local_joint_poses;
    if (ur5_robot_model_->getAllIK(candidate_pose, local_joint_poses))
    {
      joint_poses.insert(joint_poses.end(), local_joint_poses.begin(), local_joint_poses.end());
    }
  }
  return true;
}

void CartPathPlanner::visualizeAllJointPoses(const std::vector<std::vector<double>>& joint_poses)
{
  for (std::vector<double> joint_pose : joint_poses)
  {
    visual_tools_->publishRobotState(joint_pose, jmg_);
    // ros::Duration(0.01).sleep();

    if (!ros::ok())
      exit(0);
  }
}

}  // namespace curie_demos
