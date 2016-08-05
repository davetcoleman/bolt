bool useDescartesToGetPoses(EigenSTL::vector_Affine3d exact_poses, bool debug);
bool generateCartGraph();
bool convertDescartesGraphToBolt(ompl::tools::bolt::TaskGraphPtr task_graph);

bool CartPathPlanner::useDescartesToGetPoses(EigenSTL::vector_Affine3d exact_poses, bool debug)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  // Create descartes trajectory points
  cart_traj_.clear();
  cart_traj_.reserve(exact_poses.size());
  for (unsigned int i = 0; i < exact_poses.size(); i++)
  {
    // Use AxialSymetricPt objects to allow a trajectory cartesian point with rotational freedom about the tool's z axis
    using namespace descartes_core;
    cart_traj_.push_back(TrajectoryPtPtr(new descartes_trajectory::AxialSymmetricPt(
        exact_poses[i], orientation_increment_, descartes_trajectory::AxialSymmetricPt::FreeAxis::Z_AXIS,
        TimingConstraint(0.5))));

    // X_AXIS - rotates around orange line
    // Y_AXIS - rotates long ways around orange line
    // Z_AXIS - rotates around orange line
  }

  if (debug)
  {
    // Enumerate the potential points
    // for (descartes_core::TrajectoryPtPtr cart_point : cart_traj_)
    for (std::size_t i = 0; i < cart_traj_.size(); ++i)
    {
      descartes_core::TrajectoryPtPtr cart_point = cart_traj_[i];
      Eigen::Affine3d& pose = exact_poses[i];

      // Compute all poses for each pose
      EigenSTL::vector_Affine3d poses;
      if (!dynamic_cast<CartTrajectoryPt*>(cart_point.get())->computeCartesianPoses(poses))
      {
        ROS_ERROR("Failed for find ANY cartesian poses");
        return false;
      }

      for (const auto& pose : poses)
      {
        visual_tools_->publishAxis(pose);
      }

      std::vector<std::vector<double>> joint_poses;
      cart_point->getJointPoses(*ur5_robot_model_, joint_poses);

      if (joint_poses.empty())
      {
        ROS_ERROR_STREAM_NAMED(name_, "No joint solutions found for pose " << i);

        visual_tools_->publishAxis(pose, rvt::XXSMALL);
        visual_tools_->triggerBatchPublish();

        // Show previous joint poses
        if (i > 0)
        {
          ROS_INFO_STREAM_NAMED(name_, "Showing last valid robot state in red");
          descartes_core::TrajectoryPtPtr cart_point = cart_traj_[i - 1];

          std::vector<std::vector<double>> joint_poses;
          cart_point->getJointPoses(*ur5_robot_model_, joint_poses);
          visual_tools_->publishRobotState(joint_poses.front(), jmg_, rvt::RED);
        }

        return false;
      }

      // Show first configuration
      if (false)
      {
        visual_tools_->publishRobotState(joint_poses.front(), jmg_);
        ros::Duration(0.1).sleep();
      }

      // Show all possible configurations
      if (true)
      {
        for (std::vector<double> pose : joint_poses)
        {
          visual_tools_->publishRobotState(pose, jmg_);
          ros::Duration(0.1).sleep();

          if (!ros::ok())
            exit(0);
        }
      }

      visual_tools_->publishAxis(pose, rvt::XXXXSMALL);
      visual_tools_->triggerBatchPublish();
    }
  }

  // ROS_INFO_STREAM_NAMED(name_, "getAllIK found all solutions for trajectory");
  return true;
}

bool CartPathPlanner::generateCartGraph()
{
  imarker_state_ = imarker_cartesian_->getRobotState();
  Eigen::Affine3d start_pose = imarker_state_->getGlobalLinkTransform(parent_->ee_link_);

  // Generating trajectory
  bool debug = false;
  if (!generateCartTrajectory(start_pose, debug))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Failed to generate full cart trajectory at current position");
    return false;
  }

  // Benchmark runtime
  ros::Time start_time = ros::Time::now();

  const descartes_planner::PlanningGraph& pg = planner_.getPlanningGraph();

  // ------------------------------------------------------------------
  // Create the Descartes planning graph
  if (!planner_.insertGraph(cart_traj_))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Failed to create Descarrtes graph for trajectory");

    // Run again in debug mode (visualizes poses)
    bool debug = true;
    if (!generateCartTrajectory(start_pose, debug))
      return false;
  }

  double duration = (ros::Time::now() - start_time).toSec();
  ROS_INFO_STREAM_NAMED(name_, "Descartes graph generated in " << duration << " seconds with "
                                                               << boost::num_vertices(pg.getGraph()) << " vertices");
  std::cout << std::endl;

  return true;
}

bool CartPathPlanner::convertDescartesGraphToBolt(ompl::tools::bolt::TaskGraphPtr task_graph)
{
  std::size_t indent = 0;

  // Convert graph formats from Descartes to Bolt
  using namespace descartes_planner;
  using namespace descartes_core;
  using namespace descartes_trajectory;

  // Remove any previous Cartesian vertices/edges
  task_graph->generateTaskSpace(indent);

  ROS_INFO_STREAM_NAMED(name_, "Converting Descartes graph to Bolt graph");

  // For converting to MoveIt! format
  moveit::core::RobotStatePtr moveit_robot_state(new moveit::core::RobotState(*visual_tools_->getSharedRobotState()));
  std::vector<double> empty_seed;  // seed is required but not used by getNominalJointPose()
  moveit_ompl::ModelBasedStateSpacePtr space = parent_->space_;

  // Map from a Descartes vertex to a Bolt vertex
  std::map<JointGraph::vertex_descriptor, ompl::tools::bolt::TaskVertex> descarteToBoltVertex;

  ompl::tools::bolt::TaskVertex startingVertex = task_graph->getNumVertices() - 1;
  (void)startingVertex;  // prevent unused variable warning
  std::size_t new_vertex_count = 0;
  const ompl::tools::bolt::VertexType vertex_type = ompl::tools::bolt::CARTESIAN;
  const ompl::tools::bolt::VertexLevel level = 1;  // middle layer
  const PlanningGraph& pg = planner_.getPlanningGraph();

  // Iterate through vertices
  std::pair<VertexIterator, VertexIterator> vi = vertices(pg.getGraph());
  for (VertexIterator vert_iter = vi.first; vert_iter != vi.second; ++vert_iter)
  {
    JointGraph::vertex_descriptor jv = *vert_iter;

    // Get the joint values for this vertex and convert to a MoveIt! robot state
    TrajectoryPt::ID tajectory_id = pg.getGraph()[jv].id;
    const JointMap& joint_map = pg.getJointMap();
    std::vector<double> joints_pose;
    try
    {
      const JointTrajectoryPt& pt = joint_map.at(tajectory_id);
      pt.getNominalJointPose(empty_seed, *ur5_robot_model_, joints_pose);
    }
    catch (std::out_of_range)
    {
      ROS_WARN_STREAM_NAMED(name_, "Unable to find JointTrajectoryPtr in JointMap");
      return false;
    }

    // Copy vector into moveit format
    moveit_robot_state->setJointGroupPositions(jmg_, joints_pose);

    // Create new OMPL state
    ompl::base::State* ompl_state = space->allocState();

    // Convert the MoveIt! robot state into an OMPL state
    space->copyToOMPLState(ompl_state, *moveit_robot_state);

    // Add vertex to task graph
    descarteToBoltVertex[jv] = task_graph->addVertex(ompl_state, vertex_type, level, indent);

    new_vertex_count++;
  }
  ompl::tools::bolt::TaskVertex endingVertex = task_graph->getNumVertices() - 1;
  (void)endingVertex;  // prevent unused variable warning
  ROS_INFO_STREAM_NAMED(name_, "Added " << new_vertex_count << " new vertices");

  // Iterate through edges
  std::size_t new_edge_count = 0;
  std::pair<EdgeIterator, EdgeIterator> ei = edges(pg.getGraph());
  const ompl::tools::bolt::EdgeType edge_type = ompl::tools::bolt::eCARTESIAN;
  for (EdgeIterator edge_iter = ei.first; edge_iter != ei.second; ++edge_iter)
  {
    JointGraph::vertex_descriptor jv1 = source(*edge_iter, pg.getGraph());
    JointGraph::vertex_descriptor jv2 = target(*edge_iter, pg.getGraph());

    const ompl::tools::bolt::TaskVertex v1 = descarteToBoltVertex.at(jv1);
    const ompl::tools::bolt::TaskVertex v2 = descarteToBoltVertex.at(jv2);
    BOOST_ASSERT_MSG(v1 > startingVertex && v1 <= endingVertex, "Attempting to create edge with out of range vertex");
    BOOST_ASSERT_MSG(v2 > startingVertex && v2 <= endingVertex, "Attempting to create edge with out of range vertex");

    task_graph->addEdge(v1, v2, edge_type, indent);

    new_edge_count++;
  }
  ROS_DEBUG_STREAM_NAMED(name_, "Added " << new_edge_count << " new edges");

  // Connect Descartes graph to Bolt graph

  // Enumerate the start & end vertice descriptors
  std::vector<JointGraph::vertex_descriptor> start_points;
  pg.findStartVertices(start_points);
  std::vector<JointGraph::vertex_descriptor> goal_points;
  pg.findEndVertices(goal_points);

  // Track the shortest straight-line cost across any pair of start/goal points
  double shortest_path_across_cart = std::numeric_limits<double>::infinity();

  // force visualization
  // task_graph->visualizeTaskGraph_ = true;

  // Loop through all start points
  ROS_INFO_STREAM_NAMED(name_, "Connecting Descartes start points to TaskGraph");
  for (JointGraph::vertex_descriptor& jv : start_points)
  {
    // Get the corresponding BoltGraph vertex
    const ompl::tools::bolt::TaskVertex start_vertex = descarteToBoltVertex.at(jv);

    // Connect to TaskGraph
    const ompl::tools::bolt::VertexLevel level0 = 0;
    bool isStart = true;
    if (!task_graph->connectVertexToNeighborsAtLevel(start_vertex, level0, isStart, indent))
    {
      OMPL_WARN("Failed to connect Descartes start vertex %u", jv);
    }

    // Calculate the shortest straight-line distance across Descartes graph
    // Record min cost for cost-to-go heurstic distance function later
    // Check if this start vertex has has the shortest path across the Cartesian graph
    for (JointGraph::vertex_descriptor& goal_v : goal_points)
    {
      // Get the corresponding BoltGraph vertex
      const ompl::tools::bolt::TaskVertex goal_vertex = descarteToBoltVertex.at(goal_v);

      double distance_across_graph = task_graph->distanceFunction(start_vertex, goal_vertex);

      if (distance_across_graph < shortest_path_across_cart)
      {
        shortest_path_across_cart = distance_across_graph;
      }
    }

    if (!ros::ok())
      exit(-1);
  }

  // Loop through all goal points
  ROS_INFO_STREAM_NAMED(name_, "Connecting Descartes end points to TaskGraph");
  for (JointGraph::vertex_descriptor& jv : goal_points)
  {
    // Get the corresponding BoltGraph vertex
    const ompl::tools::bolt::TaskVertex goal_vertex = descarteToBoltVertex.at(jv);

    // Connect to TaskGraph
    const ompl::tools::bolt::VertexLevel level2 = 2;
    bool isStart = false;
    if (!task_graph->connectVertexToNeighborsAtLevel(goal_vertex, level2, isStart, indent))
    {
      OMPL_WARN("Failed to connect Descartes goal vertex %u", jv);
    }

    if (!ros::ok())
      exit(-1);
  }
  ROS_INFO_STREAM_NAMED(name_, "Finished connecting Descartes end points to TaskGraph");

  // Set the shortest path across cartesian graph in the TaskGraph
  task_graph->setShortestDistAcrossCart(shortest_path_across_cart);

  // Tell the planner to require task planning
  task_graph->setTaskPlanningEnabled();

  task_graph->printGraphStats();

  return true;
}
