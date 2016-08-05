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
   Desc:   Demo dual arm manipulation
*/

// Interface for loading rosparam settings into OMPL
#include <moveit_ompl/ompl_rosparam.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <ompl_visual_tools/projection_viz_window.h>

// this package
#include <curie_demos/curie_demos.h>

// Profiling
#include <valgrind/callgrind.h>

namespace ob = ompl::base;
namespace ot = ompl::tools;
namespace otb = ompl::tools::bolt;
namespace og = ompl::geometric;
namespace rvt = rviz_visual_tools;

namespace curie_demos
{
CurieDemos::CurieDemos(const std::string &hostname, const std::string &package_path)
  : MoveItBase(), nh_("~"), remote_control_(nh_), package_path_(package_path)
{
  // Profiler
  CALLGRIND_TOGGLE_COLLECT;

  bool seed_random;
  // Load rosparams
  ros::NodeHandle rpnh(nh_, name_);
  std::size_t error = 0;
  // run mode
  error += !rosparam_shortcuts::get(name_, rpnh, "run_problems", run_problems_);
  error += !rosparam_shortcuts::get(name_, rpnh, "create_spars", create_spars_);
  error += !rosparam_shortcuts::get(name_, rpnh, "load_spars", load_spars_);
  error += !rosparam_shortcuts::get(name_, rpnh, "continue_spars", continue_spars_);
  error += !rosparam_shortcuts::get(name_, rpnh, "eliminate_dense_disjoint_sets", eliminate_dense_disjoint_sets_);
  error += !rosparam_shortcuts::get(name_, rpnh, "check_valid_vertices", check_valid_vertices_);
  error += !rosparam_shortcuts::get(name_, rpnh, "display_disjoint_sets", display_disjoint_sets_);
  error += !rosparam_shortcuts::get(name_, rpnh, "benchmark_performance", benchmark_performance_);

  // run type
  error += !rosparam_shortcuts::get(name_, rpnh, "auto_run", auto_run_);
  error += !rosparam_shortcuts::get(name_, rpnh, "experience_planner", experience_planner_);
  error += !rosparam_shortcuts::get(name_, rpnh, "planning_runs", planning_runs_);
  error += !rosparam_shortcuts::get(name_, rpnh, "headless", headless_);
  error += !rosparam_shortcuts::get(name_, rpnh, "problem_type", problem_type_);
  error += !rosparam_shortcuts::get(name_, rpnh, "use_task_planning", use_task_planning_);
  error += !rosparam_shortcuts::get(name_, rpnh, "planning_group_name", planning_group_name_);
  error += !rosparam_shortcuts::get(name_, rpnh, "ee_tip_link", ee_tip_link_);
  error += !rosparam_shortcuts::get(name_, rpnh, "seed_random", seed_random);
  error += !rosparam_shortcuts::get(name_, rpnh, "post_processing", post_processing_);
  error += !rosparam_shortcuts::get(name_, rpnh, "post_processing_interval", post_processing_interval_);
  error += !rosparam_shortcuts::get(name_, rpnh, "use_logging", use_logging_);
  error += !rosparam_shortcuts::get(name_, rpnh, "collision_checking_enabled", collision_checking_enabled_);
  // Visualize
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/display_database", visualize_display_database_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/interpolated_traj", visualize_interpolated_traj_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/start_goal_states", visualize_start_goal_states_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/time_between_plans", visualize_time_between_plans_);
  error += !rosparam_shortcuts::get(name_, rpnh, "visualize/database_every_plan", visualize_database_every_plan_);
  // Debug
  error += !rosparam_shortcuts::get(name_, rpnh, "verbose/print_trajectory", debug_print_trajectory_);
  rosparam_shortcuts::shutdownIfError(name_, error);

  // Auto-set headless if not on developer PC, assume we are on server
  if (hostname != "ros-monster")
  {
    OMPL_WARN("Auto-setting to headless mode because hostname is %s", hostname.c_str());
    headless_ = true;
  }
  if (headless_)
    OMPL_WARN("Running in headless mode");

  // Seed random
  if (seed_random)
    srand(time(NULL));

  // Initialize MoveIt base
  MoveItBase::init(nh_);

  // Load 2 more robot states
  moveit_start_.reset(new moveit::core::RobotState(*current_state_));
  moveit_goal_.reset(new moveit::core::RobotState(*current_state_));

  // Get the two arms jmg
  jmg_ = robot_model_->getJointModelGroup(planning_group_name_);
  ee_link_ = robot_model_->getLinkModel(ee_tip_link_);

  // Load planning
  if (!loadOMPL())
  {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to load planning context");
    exit(-1);
  }

  // Load more visual tool objects
  loadVisualTools();

  // Add a collision objects
  visual_moveit_start_->publishCollisionFloor(0.001, "floor", rvt::TRANSLUCENT_DARK);
  visual_moveit_start_->publishCollisionWall(-0.3, 0.0, 0, 2, 1.5, "wall", rvt::BLACK);
  visual_moveit_start_->triggerPlanningSceneUpdate();
  ros::spinOnce();

  // if (track_memory_consumption_)
  // {
  //   double vm1, rss1;
  //   processMemUsage(vm1, rss1);
  //   ROS_INFO_STREAM_NAMED(name_, "Current memory consumption - VM: " << vm1 << " MB | RSS: " << rss1 << " MB");
  // }

  // Create start/goal state imarker
  if (!headless_)
  {
    // Create cartesian planner
    cart_path_planner_.reset(new CartPathPlanner(this));

    imarker_start_.reset(
        new mvt::IMarkerRobotState(planning_scene_monitor_, "start", jmg_, ee_link_, rvt::GREEN, package_path_));
    imarker_goal_.reset(
        new mvt::IMarkerRobotState(planning_scene_monitor_, "goal", jmg_, ee_link_, rvt::ORANGE, package_path_));
  }

  // Set remote_control
  remote_control_.setDisplayWaitingState(boost::bind(&CurieDemos::displayWaitingState, this, _1));

  // Wait until user does something
  if (!auto_run_)
    waitForNextStep("run first problem");

  // Run application
  run();

  // Profiler
  CALLGRIND_TOGGLE_COLLECT;
  CALLGRIND_DUMP_STATS;
}

CurieDemos::~CurieDemos()
{
  // Free start and goal states
  space_->freeState(ompl_start_);
  space_->freeState(ompl_goal_);
}

bool CurieDemos::loadOMPL()
{
  moveit_ompl::ModelBasedStateSpaceSpecification mbss_spec(robot_model_, jmg_);

  // Construct the state space we are planning in
  space_.reset(new moveit_ompl::ModelBasedStateSpace(mbss_spec));

  // Create SimpleSetup
  if (experience_planner_ == "bolt")
  {
    bolt_ = otb::BoltPtr(new otb::Bolt(space_));
    experience_setup_ = bolt_;
    is_bolt_ = true;
  }
  else if (experience_planner_ == "thunder")
  {
    experience_setup_ = ot::ThunderPtr(new ot::Thunder(space_));
    is_thunder_ = true;
  }

  // Get Space Info
  si_ = experience_setup_->getSpaceInformation();

  // Run interface for loading rosparam settings into OMPL
  moveit_ompl::loadOMPLParameters(nh_, name_, bolt_);

  // Load collision checker
  loadCollisionChecker();

  // Setup base OMPL stuff. Do this before choosing filename so sparseDeltaFraction is ready
  ROS_INFO_STREAM_NAMED(name_, "Setting up Bolt");
  experience_setup_->setup();
  assert(si_->isSetup());

  // Set the database file location
  std::string file_path = "";
  std::string file_name;
  if (benchmark_performance_)
    file_name = "benchmark_";
  if (is_bolt_)
    file_name = file_name + "bolt_" + planning_group_name_ + "_" +
                std::to_string(bolt_->getSparseCriteria()->sparseDeltaFraction_) + "_database";
  else
    file_name = file_name + " thunder_" + planning_group_name_ + "_database";
  moveit_ompl::getFilePath(file_path, file_name, "ros/ompl_storage");
  experience_setup_->setFilePath(file_path);  // this is here because its how we do it in moveit_ompl

  // Create start and goal states
  ompl_start_ = space_->allocState();
  ompl_goal_ = space_->allocState();

  return true;
}

bool CurieDemos::loadData()
{
  // double vm1, rss1;
  // if (track_memory_consumption_)  // Track memory usage
  // {
  //   processMemUsage(vm1, rss1);
  //   // ROS_INFO_STREAM_NAMED(name_, "Current memory consumption - VM: " << vm1 << " MB | RSS: " << rss1 << " MB");
  // }

  // Load database or generate new roadmap
  ROS_INFO_STREAM_NAMED(name_, "Loading or generating roadmap");
  if (is_bolt_)
  {
    if (!bolt_->load())
    {
      ROS_INFO_STREAM_NAMED(name_, "Unable to load sparse graph from file");
      return false;
    }
  }

  // if (track_memory_consumption_)  // Track memory usage
  // {
  //   double vm2, rss2;
  //   processMemUsage(vm2, rss2);
  //   // ROS_INFO_STREAM_NAMED(name_, "Current memory consumption - VM: " << vm2 << " MB | RSS: " << rss2 << " MB");
  //   ROS_INFO_STREAM_NAMED(name_, "RAM usage diff - VM: " << vm2 - vm1 << " MB | RSS: " << rss2 - rss1 << " MB");
  // }

  return true;
}

void CurieDemos::run()
{
  deleteAllMarkers();  // again, cause it seems broken

  // Benchmark performance
  if (benchmark_performance_)
  {
    // testMotionValidator();
    // bolt_->benchmarkSparseGraphGeneration();
    bolt_->benchmarkRandValidSampling();
    ROS_INFO_STREAM_NAMED(name_, "Finished benchmarking");
    exit(0);
  }

  // Load from file
  bool loaded = false;
  if (load_spars_)
  {
    loaded = loadData();
  }

  // Create SPARS
  if (create_spars_ && (!loaded || continue_spars_))
  {
    bolt_->getSparseGenerator()->createSPARS();
    loaded = true;
  }

  if (!loaded)
    ROS_WARN_STREAM_NAMED(name_, "Creating AND loading sparse graph disabled, no contents in graph");

  // Display disconnected components
  if (display_disjoint_sets_ && is_bolt_)
  {
    std::cout << std::endl;
    ROS_INFO_STREAM_NAMED(name_, "Displaying disjoint sets ----------- ");
    ompl::tools::bolt::SparseDisjointSetsMap disjointSets;
    bolt_->getSparseGraph()->getDisjointSets(disjointSets);
    bolt_->getSparseGraph()->printDisjointSets(disjointSets);
    bolt_->getSparseGraph()->visualizeDisjointSets(disjointSets);
  }

  // Repair missing coverage in the dense graph
  // if (eliminate_dense_disjoint_sets_)
  // {
  //   experience_setup_->getSparseGraph()->getDiscretizer()->eliminateDisjointSets();
  // }

  // Remove verticies that are somehow in collision
  // if (check_valid_vertices_)
  // {
  //   experience_setup_->getSparseGraph()->removeInvalidVertices();
  //   experience_setup_->getSparseGraph()->saveIfChanged();
  // }

  // Run the demo
  if (!run_problems_)
    ROS_INFO_STREAM("Solving requested to be skipped by config file");
  else
  {
    runProblems();
    // runPopularityExperiement();
    // runSparseFactorExperiment();
  }
  // testConnectionToGraphOfRandStates();

  bolt_->saveIfChanged();
}

bool CurieDemos::runProblems()
{
  // Logging
  std::ofstream logging_file;  // open to append
  if (use_logging_)
  {
    std::string file_path;
    moveit_ompl::getFilePath(file_path, "bolt_2d_world_logging.csv", "ros/ompl_storage");
    logging_file.open(file_path.c_str(), std::ios::out);  // no append | std::ios::app);
  }

  // Run the demo the desired number of times
  for (std::size_t run_id = 0; run_id < planning_runs_; ++run_id)
  {
    if (!ros::ok())  // Check if user wants to shutdown
      break;

    std::cout << std::endl;
    std::cout << "------------------------------------------------------------------------" << std::endl;
    ROS_INFO_STREAM_NAMED("plan", "Planning " << run_id + 1 << " out of " << planning_runs_);
    std::cout << "------------------------------------------------------------------------" << std::endl;

    if (headless_)
      ROS_WARN_STREAM_NAMED(name_, "imarker start/goal not loaded");

    // Generate start/goal pair
    if (problem_type_ == 0)
    {
      imarker_start_->setToRandomState();
      imarker_goal_->setToRandomState();
    }
    moveit_start_ = imarker_start_->getRobotState();
    moveit_goal_ = imarker_goal_->getRobotState();

    // Visualize
    if (visualize_start_goal_states_)
      visualizeStartGoal();

    // Optionally create cartesian path, if this is a task plan
    if (use_task_planning_)
    {
      if (!generateCartGraph())
      {
        ROS_ERROR_STREAM_NAMED(name_, "Unable to create cart path");
        exit(-1);
      }
    }

    // Do one plan
    plan();

    // Console display
    experience_setup_->printLogs();

    // Logging
    if (use_logging_)
    {
      experience_setup_->saveDataLog(logging_file);
      logging_file.flush();
    }

    // Regenerate Sparse Graph
    if (post_processing_ && run_id % post_processing_interval_ == 0 && run_id > 0)  // every x runs
    {
      ROS_INFO_STREAM_NAMED(name_, "Performing post processing every " << post_processing_interval_ << " intervals");
      experience_setup_->doPostProcessing();
    }

    if (visualize_wait_between_plans_ && run_id < planning_runs_ - 1)
      waitForNextStep("run next problem");
    else  // Main pause between planning instances - allows user to analyze
      ros::Duration(visualize_time_between_plans_).sleep();

    if (!ros::ok())  // Check if user wants to shutdown
      break;

    // Reset marker if this is not our last run
    if (run_id < planning_runs_ - 1)
      deleteAllMarkers(false);
  }  // for each run

  // Save experience
  if (post_processing_)
    experience_setup_->doPostProcessing();

  // Finishing up
  ROS_INFO_STREAM_NAMED(name_, "Saving experience db...");
  experience_setup_->saveIfChanged();

  // Stats
  if (total_runs_ > 0)
    ROS_INFO_STREAM_NAMED(name_, "Average solving time: " << total_duration_ / total_runs_);

  return true;
}

bool CurieDemos::plan()
{
  // Setup -----------------------------------------------------------

  // Clear all planning data. This only includes data generated by motion plan computation.
  // Planner settings, start & goal states are not affected.
  experience_setup_->clear();

  // Convert MoveIt state to OMPL state
  space_->copyToOMPLState(ompl_start_, *moveit_start_);
  space_->copyToOMPLState(ompl_goal_, *moveit_goal_);

  // Convert the goal state to level 2
  if (use_task_planning_)
  {
    const int level = 2;
    space_->setLevel(ompl_goal_, level);
  }

  // Set the start and goal states
  experience_setup_->setStartAndGoalStates(ompl_start_, ompl_goal_);

  // Solve -----------------------------------------------------------

  // Create the termination condition
  double seconds = 600;
  ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(seconds, 0.1);

  // Benchmark runtime
  ros::Time start_time = ros::Time::now();

  // Attempt to solve the problem within x seconds of planning time
  ob::PlannerStatus solved = experience_setup_->solve(ptc);

  // Benchmark runtime
  total_duration_ = (ros::Time::now() - start_time).toSec();

  // Check for error
  if (!solved)
  {
    ROS_ERROR_STREAM_NAMED(name_, "No solution found");
    exit(-1);
    return false;
  }

  // Get solution
  og::PathGeometric path = experience_setup_->getSolutionPath();

  // Add start to solution
  // path.prepend(ompl_start_);  // necessary?

  // Check/test the solution for errors
  if (use_task_planning_)
  {
    bolt_->getTaskGraph()->checkTaskPathSolution(path, ompl_start_, ompl_goal_);
  }

  // Add more states between waypoints
  // state_count = path.getStateCount();
  // path.interpolate();
  // ROS_INFO_STREAM_NAMED(name_, "Interpolation added: " << path.getStateCount() - state_count << " states");

  // Convert trajectory
  robot_trajectory::RobotTrajectoryPtr traj;
  const double speed = 0.025;
  viz3_->convertPath(path, jmg_, traj, speed);

  // Check/test the solution for errors
  checkMoveItPathSolution(traj);

  // Visualize the trajectory
  // if (visualize_interpolated_traj_)
  // {
  //   ROS_INFO("Visualizing the interpolated trajectory");

  //   // Show trajectory line
  //   mvt::MoveItVisualToolsPtr visual_moveit3 = boost::dynamic_pointer_cast<mvt::MoveItVisualTools>(viz3_);
  //   visual_moveit3->publishTrajectoryLine(traj, ee_link_, rvt::RED);

  //   // Show trajectory
  //   const bool wait_for_trajectory = true;
  //   visual_moveit3->publishTrajectoryPath(traj, wait_for_trajectory);

  //   ros::Duration(1).sleep();
  // }

  // Visualize the doneness
  std::cout << std::endl;

  return true;
}

void CurieDemos::loadCollisionChecker()
{
  // Create state validity checking for this space
  validity_checker_ =
      new moveit_ompl::StateValidityChecker(planning_group_name_, si_, *current_state_, planning_scene_, space_);
  validity_checker_->setCheckingEnabled(collision_checking_enabled_);

  // Set checker
  experience_setup_->setStateValidityChecker(ob::StateValidityCheckerPtr(validity_checker_));

  // The interval in which obstacles are checked for between states
  // seems that it default to 0.01 but doesn't do a good job at that level
  si_->setStateValidityCheckingResolution(0.005);
}

void CurieDemos::deleteAllMarkers(bool clearDatabase)
{
  if (headless_)
    return;

  // Reset rviz markers
  if (clearDatabase)
  {
    viz1_->deleteAllMarkers();
    viz2_->deleteAllMarkers();
    viz3_->deleteAllMarkers();
  }
  viz4_->deleteAllMarkers();
  viz5_->deleteAllMarkers();
  viz6_->deleteAllMarkers();

  // Publish
  viz1_->trigger();
  viz2_->trigger();
  viz3_->trigger();
  viz4_->trigger();
  viz5_->trigger();
  viz6_->trigger();
}

void CurieDemos::loadVisualTools()
{
  using namespace ompl_visual_tools;
  using namespace moveit_visual_tools;

  Eigen::Affine3d offset;
  std::string namesp = nh_.getNamespace();
  moveit_start_->setToDefaultValues();

  const std::size_t NUM_VISUALS = 6;
  for (std::size_t i = 1; i <= NUM_VISUALS; ++i)
  {
    MoveItVisualToolsPtr moveit_visual = MoveItVisualToolsPtr(new MoveItVisualTools(
        "/world_visual" + std::to_string(i), namesp + "/ompl_visual" + std::to_string(i), robot_model_));
    moveit_visual->loadMarkerPub(false);
    moveit_visual->setPlanningSceneMonitor(planning_scene_monitor_);
    moveit_visual->setManualSceneUpdating(true);
    moveit_visual->setGlobalScale(0.8);
    moveit_visual->enableBatchPublishing();

    MoveItVizWindowPtr viz = MoveItVizWindowPtr(new MoveItVizWindow(moveit_visual, si_));
    viz->setJointModelGroup(jmg_);

    bool blocking = false;
    if (!headless_)
    {
      // Load publishers
      moveit_visual->loadRobotStatePub(namesp + "/robot_state" + std::to_string(i), blocking);

      // Load trajectory publisher - ONLY for viz6
      if (i == 6)
        moveit_visual->loadTrajectoryPub("/hilgendorf/display_trajectory", blocking);
    }

    // Calibrate the color scale for visualization
    const bool invert_colors = true;
    viz->setMinMaxEdgeCost(0, 110, invert_colors);
    viz->setMinMaxEdgeRadius(0.001, 0.004);
    viz->setMinMaxStateRadius(0.5, 5);

    // Copy pointers over
    // clang-format off
    switch (i)
    {
      case 1: viz1_ = viz; break;
      case 2: viz2_ = viz; break;
      case 3: viz3_ = viz; break;
      case 4: viz4_ = viz; break;
      case 5: viz5_ = viz; break;
      case 6: viz6_ = viz; break;
    }
    // clang-format on

    // Index the visualizers
    vizs_.push_back(viz);
  }  // for reach visualizer

  ros::spinOnce();

  // Secondary loop to give time for all the publishers to load up
  if (!headless_)
  {
    for (std::size_t i = 1; i <= NUM_VISUALS; ++i)
    {
      MoveItVisualToolsPtr moveit_visual = vizs_[i - 1]->getVisualTools();
      // Get TF
      getTFTransform("world", "world_visual" + std::to_string(i), offset);
      moveit_visual->enableRobotStateRootOffet(offset);
    }
  }

  viz6_->getVisualTools()->setBaseFrame("world");
  visual_moveit_start_ = viz6_->getVisualTools();
  visual_moveit_goal_ = viz5_->getVisualTools();

  ros::spinOnce();

  // Block until all visualizers are finished loading
  if (!headless_)
  {
    for (std::size_t i = 1; i <= NUM_VISUALS; ++i)
    {
      vizs_[i - 1]->getVisualTools()->waitForMarkerPub();

      // Show the initial robot state
      MoveItVisualToolsPtr moveit_visual = vizs_[i - 1]->getVisualTools();
      moveit_visual->publishRobotState(moveit_start_);
    }
  }

  deleteAllMarkers();

  // Set Rviz visuals in OMPL planner
  ompl::tools::VisualizerPtr visual = experience_setup_->getVisual();

  visual->setVizWindow(1, viz1_);
  visual->setVizWindow(2, viz2_);
  visual->setVizWindow(3, viz3_);
  visual->setVizWindow(4, viz4_);
  visual->setVizWindow(5, viz5_);
  visual->setVizWindow(6, viz6_);

  // Project\ion viewer - mirrors MoveItVisualTools 6
  {
    viz6_->getVisualTools()->setGlobalScale(1.0);

    ProjectionVizWindowPtr viz = ProjectionVizWindowPtr(new ProjectionVizWindow(viz2_->getVisualTools(), si_));
    // Calibrate the color scale for visualization
    const bool invert_colors = true;
    viz->setMinMaxEdgeCost(0, 110, invert_colors);
    viz->setMinMaxEdgeRadius(0.001, 0.004);
    viz->setMinMaxStateRadius(1, 4);

    visual->setVizWindow(7, viz);
  }

  // Set other hooks
  visual->setWaitForUserFeedback(boost::bind(&CurieDemos::waitForNextStep, this, _1));
}

void CurieDemos::visualizeStartGoal()
{
  visual_moveit_start_->publishRobotState(moveit_start_, rvt::GREEN);
  visual_moveit_goal_->publishRobotState(moveit_goal_, rvt::ORANGE);

  // Show values and limits
  // std::cout << "Start: " << std::endl;
  // visual_moveit_start_->showJointLimits(moveit_start_);
  // std::cout << "Goal: " << std::endl;
  // visual_moveit_start_->showJointLimits(moveit_goal_);
}

void CurieDemos::displayWaitingState(bool waiting)
{
  // std::cout << " TODO display waiting state " << std::endl;
  // if (waiting)
  //   publishViewFinderFrame(rvt::REGULAR);
  // else
  //   publishViewFinderFrame(rvt::XSMALL);

  // viz_bg_->trigger();
}

void CurieDemos::waitForNextStep(const std::string &msg)
{
  remote_control_.waitForNextStep(msg);
}

void CurieDemos::testConnectionToGraphOfRandStates()
{
  ompl::base::State *random_state = space_->allocState();

  std::size_t successful_connections = 0;
  for (std::size_t run_id = 0; run_id < planning_runs_; ++run_id)
  {
    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    ROS_INFO_STREAM_NAMED(name_, "Testing random state " << run_id);

    // Generate random state
    getRandomState(moveit_start_);

    // Visualize
    visual_moveit_start_->publishRobotState(moveit_start_, rvt::GREEN);

    // Convert to ompl
    space_->copyToOMPLState(random_state, *moveit_start_);

    // Test
    const ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(60.0);
    std::size_t indent = 0;
    bool result = bolt_->getBoltPlanner()->canConnect(random_state, ptc, indent);
    if (result)
      successful_connections++;

    ROS_ERROR_STREAM_NAMED(name_, "Percent connnected: " << successful_connections / double(run_id + 1) * 100.0);
  }

  space_->freeState(random_state);
}

void CurieDemos::visualizeRawTrajectory(og::PathGeometric &path)
{
  ROS_INFO("Visualizing non-interpolated trajectory");

  // Convert trajectory
  robot_trajectory::RobotTrajectoryPtr traj;
  const double speed = 0.05;
  viz3_->convertPath(path, jmg_, traj, speed);

  // Show trajectory line
  viz3_->getVisualTools()->publishTrajectoryLine(traj, ee_link_, rvt::GREY);
  viz3_->trigger();
}

bool CurieDemos::generateCartGraph()
{
  // Generate the Descartes graph - if it fails let user adjust interactive marker
  while (true)
  {
    if (!cart_path_planner_->populateBoltGraph(bolt_->getTaskGraph()))
    {
      ROS_INFO_STREAM_NAMED(name_, "Unable to populate Bolt graph - try moving the start location");
      waitForNextStep("attempt Bolt graph generation again");
      if (!ros::ok())
        exit(0);
    }
    else
      break;
  }

  return true;
}

bool CurieDemos::checkMoveItPathSolution(robot_trajectory::RobotTrajectoryPtr traj)
{
  std::size_t state_count = traj->getWayPointCount();
  if (state_count < 3)
    ROS_WARN_STREAM_NAMED(name_, "checkMoveItPathSolution: Solution path has only " << state_count << " states");
  else
    ROS_INFO_STREAM_NAMED(name_, "checkMoveItPathSolution: Solution path has " << state_count << " states");

  std::vector<std::size_t> index;
  const bool verbose = true;
  if (!planning_scene_->isPathValid(*traj, "", verbose, &index))
  {
    if (index.size() == 1 && index[0] == 0)  // ignore cases when the robot starts at invalid location
      ROS_DEBUG("It appears the robot is starting at an invalid state, but that is ok.");
    else
    {
      // display error messages
      std::stringstream ss;
      for (std::size_t i = 0; i < index.size(); ++i)
        ss << index[i] << " ";
      ROS_ERROR_STREAM_NAMED(
          name_, "checkMoveItPathSolution: Computed path is not valid. Invalid states at index locations: [ "
                     << ss.str() << "] out of " << state_count << ". Explanations follow in command line.");

      // Call validity checks in verbose mode for the problematic states
      visualization_msgs::MarkerArray arr;
      for (std::size_t i = 0; i < index.size(); ++i)
      {
        /*
        // check validity with verbose on
        const robot_state::RobotState &robot_state = traj->getWayPoint(index[i]);
        planning_scene_->isStateValid(robot_state, request.path_constraints, request.group_name, true);

        // compute the contacts if any
        collision_detection::CollisionRequest c_req;
        collision_detection::CollisionResult c_res;
        c_req.contacts = true;
        c_req.max_contacts = 10;
        c_req.max_contacts_per_pair = 3;
        c_req.verbose = false;
        planning_scene_->checkCollision(c_req, c_res, robot_state);
        */
        ROS_ERROR_STREAM_NAMED(name_, "checkMoveItPathSolution: TODO: show collision states in code " << i);
        /*
          if (c_res.contact_count > 0)
          {
          visualization_msgs::MarkerArray arr_i;
          collision_detection::getCollisionMarkersFromContacts(arr_i, planning_scene_->getPlanningFrame(),
          c_res.contacts);
          arr.markers.insert(arr.markers.end(), arr_i.markers.begin(), arr_i.markers.end());
          }
        */
      }
      ROS_ERROR_STREAM_NAMED(name_, "checkMoveItPathSolution: Completed listing of explanations for invalid states.");
    }
  }
  return true;
}

bool CurieDemos::getRandomState(moveit::core::RobotStatePtr &robot_state)
{
  static const std::size_t MAX_ATTEMPTS = 1000;
  for (std::size_t i = 0; i < MAX_ATTEMPTS; ++i)
  {
    robot_state->setToRandomPositions(jmg_);
    robot_state->update();

    // Error check
    bool check_verbose = false;
    if (planning_scene_->isStateValid(*robot_state, "", check_verbose))  // second argument is what planning group to
                                                                         // collision check, "" is everything
    {
      // ROS_DEBUG_STREAM_NAMED(name_, "Found valid random robot state after " << i << " attempts");
      return true;
    }

    if (i == 100)
      ROS_WARN_STREAM_NAMED(name_, "Taking long time to find valid random state");
  }

  ROS_ERROR_STREAM_NAMED(name_, "Unable to find valid random robot state");
  exit(-1);
  return false;
}

void CurieDemos::testMotionValidator()
{
  // THIS FUNCTION BROKEN BECAUSE moveit_core SAYS "FCL continuous collision checking not yet implemented"

  // moveit::core::RobotStatePtr start = moveit::core::RobotStatePtr(new moveit::core::RobotState(*current_state_));
  // moveit::core::RobotStatePtr goal = moveit::core::RobotStatePtr(new moveit::core::RobotState(*current_state_));
  moveit_start_->setToRandomPositions(jmg_);
  moveit_goal_->setToRandomPositions(jmg_);

  // visual_moveit_start_->publishRobotState(moveit_start_, rvt::GREEN);
  visual_moveit_goal_->publishRobotState(moveit_goal_, rvt::ORANGE);

  // Check for collision between to states
  bool verbose = true;
  collision_detection::CollisionResult res;
  planning_scene_->checkCollision(validity_checker_->collision_request_with_distance_verbose_, res, *moveit_start_);
  std::cout << "start state in collision: " << res.collision << std::endl;

  collision_detection::CollisionRequest req;
  req.group_name = planning_group_name_;
  req.verbose = true;

  // Check motion
  planning_scene_->getCollisionWorld()->checkCollision(req, res, *planning_scene_->getCollisionRobot(), *moveit_start_,
                                                       *moveit_goal_);

  std::cout << "motion in collision: " << res.collision << std::endl;
}

}  // namespace curie_demos
