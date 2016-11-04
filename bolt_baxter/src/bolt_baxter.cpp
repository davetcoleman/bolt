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

// ROS parameter helpers
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <bolt_ros/ompl_rosparam.h>

// bolt_core
#include <bolt_core/SparseMirror.h>
#include <bolt_core/SparseCriteria.h>
#include <bolt_core/SparseFormula.h>
#include <bolt_core/BoltPlanner.h>

// bolt_moveit
#include <bolt_moveit/process_mem_usage.h>
#include <bolt_moveit/projection_viz_window.h>
#include <bolt_moveit/model_size_state_space.h>

// OMPL
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>

// this package
#include <bolt_baxter/bolt_baxter.h>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>

// MoveIt
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>

// Profiling
#include <valgrind/callgrind.h>

// C++
#include <algorithm>  // for lower case

namespace ob = ompl::base;
namespace ot = ompl::tools;
namespace otb = ompl::tools::bolt;
namespace og = ompl::geometric;
namespace rvt = rviz_visual_tools;

namespace bolt_baxter
{
BoltBaxter::BoltBaxter(const std::string &hostname, const std::string &package_path)
  : MoveItBase(), nh_("~"), package_path_(package_path)
{
  std::size_t indent = 0;

  std::vector<std::string> ee_tip_links;
  std::vector<std::string> arm_jmgs;

  bool seed_random;
  // Load rosparams
  ros::NodeHandle rpnh(nh_, name_);
  std::size_t error = 0;
  // run mode
  error += !rosparam_shortcuts::get(name_, rpnh, "run_problems", run_problems_);
  error += !rosparam_shortcuts::get(name_, rpnh, "create_spars", create_spars_);
  error += !rosparam_shortcuts::get(name_, rpnh, "load_spars", load_spars_);
  error += !rosparam_shortcuts::get(name_, rpnh, "load_database_version", load_database_version_);
  error += !rosparam_shortcuts::get(name_, rpnh, "continue_spars", continue_spars_);
  error += !rosparam_shortcuts::get(name_, rpnh, "eliminate_dense_disjoint_sets", eliminate_dense_disjoint_sets_);
  error += !rosparam_shortcuts::get(name_, rpnh, "check_valid_vertices", check_valid_vertices_);
  error += !rosparam_shortcuts::get(name_, rpnh, "display_disjoint_sets", display_disjoint_sets_);
  error += !rosparam_shortcuts::get(name_, rpnh, "benchmark_performance", benchmark_performance_);
  error += !rosparam_shortcuts::get(name_, rpnh, "save_imarkers_to_file", save_imarkers_to_file_);
  error += !rosparam_shortcuts::get(name_, rpnh, "view_imarkers_from_file", view_imarkers_from_file_);

  // mirror
  error += !rosparam_shortcuts::get(name_, rpnh, "mirror_graph", mirror_graph_);
  error += !rosparam_shortcuts::get(name_, rpnh, "opposite_arm_name", opposite_arm_name_);
  error += !rosparam_shortcuts::get(name_, rpnh, "both_arms_group_name", both_arms_group_name_);

  // fill in last dimension
  // error += !rosparam_shortcuts::get(name_, rpnh, "fill_in_dim", fill_in_dim_);
  // error += !rosparam_shortcuts::get(name_, rpnh, "full_arm_name", full_arm_name_);

  // run type
  error += !rosparam_shortcuts::get(name_, rpnh, "auto_run", auto_run_);
  error += !rosparam_shortcuts::get(name_, rpnh, "planners", planners_);
  error += !rosparam_shortcuts::get(name_, rpnh, "planning_time", planning_time_);
  error += !rosparam_shortcuts::get(name_, rpnh, "num_problems", num_problems_);
  error += !rosparam_shortcuts::get(name_, rpnh, "headless", headless_);
  error += !rosparam_shortcuts::get(name_, rpnh, "problem_type", problem_type_);
  error += !rosparam_shortcuts::get(name_, rpnh, "imarker_goal_list", imarker_goal_list_);
  error += !rosparam_shortcuts::get(name_, rpnh, "use_task_planning", use_task_planning_);
  error += !rosparam_shortcuts::get(name_, rpnh, "planning_group_name", planning_group_name_);
  error += !rosparam_shortcuts::get(name_, rpnh, "arm_jmgs", arm_jmgs);
  error += !rosparam_shortcuts::get(name_, rpnh, "ee_tip_links", ee_tip_links);
  error += !rosparam_shortcuts::get(name_, rpnh, "seed_random", seed_random);
  error += !rosparam_shortcuts::get(name_, rpnh, "post_processing", post_processing_);
  error += !rosparam_shortcuts::get(name_, rpnh, "post_processing_interval", post_processing_interval_);
  error += !rosparam_shortcuts::get(name_, rpnh, "use_logging", use_logging_);
  error += !rosparam_shortcuts::get(name_, rpnh, "collision_checking_enabled", collision_checking_enabled_);
  // collision
  error += !rosparam_shortcuts::get(name_, rpnh, "scene_type", scene_type_);
  error += !rosparam_shortcuts::get(name_, rpnh, "distance_to_shelf", distance_to_shelf_);
  // execution
  error += !rosparam_shortcuts::get(name_, rpnh, "connect_to_hardware", connect_to_hardware_);
  error += !rosparam_shortcuts::get(name_, rpnh, "velocity_scaling_factor", velocity_scaling_factor_);
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

  // Set custom joint weights
  robot_model_->getJointModel("left_s1")->setDistanceFactor(1.0);
  robot_model_->getJointModel("left_e0")->setDistanceFactor(0.9);
  robot_model_->getJointModel("left_e1")->setDistanceFactor(0.8);
  robot_model_->getJointModel("left_w0")->setDistanceFactor(0.7);
  robot_model_->getJointModel("left_w1")->setDistanceFactor(0.6);
  robot_model_->getJointModel("left_w2")->setDistanceFactor(0.0);

  robot_model_->getJointModel("right_s1")->setDistanceFactor(1.0);
  robot_model_->getJointModel("right_e0")->setDistanceFactor(0.9);
  robot_model_->getJointModel("right_e1")->setDistanceFactor(0.8);
  robot_model_->getJointModel("right_w0")->setDistanceFactor(0.7);
  robot_model_->getJointModel("right_w1")->setDistanceFactor(0.6);
  robot_model_->getJointModel("right_w2")->setDistanceFactor(0.0);

  // Load more robot states
  moveit_start_ = std::make_shared<moveit::core::RobotState>(*current_state_);
  moveit_goal_ = std::make_shared<moveit::core::RobotState>(*current_state_);

  // State for copying one arm to another (mirroring)
  mirror_state_ = std::make_shared<moveit::core::RobotState>(*current_state_);
  // set default wrist position (and all other joints)
  mirror_state_->setToDefaultValues();

  // Get the two arms jmg
  planning_jmg_ = robot_model_->getJointModelGroup(planning_group_name_);

  if (arm_jmgs.size() != ee_tip_links.size())
  {
    BOLT_ERROR("Joint model groups array must match size of EEF tip links array");
    exit(-1);
  }

  for (std::size_t i = 0; i < arm_jmgs.size(); ++i)
  {
    arm_datas_.push_back(
        mvt::ArmData(robot_model_->getJointModelGroup(arm_jmgs[i]), robot_model_->getLinkModel(ee_tip_links[i])));
    if (!arm_datas_.back().jmg_)
    {
      BOLT_ERROR("No joint model group found for jmg name " << arm_jmgs[i]);
      exit(-1);
    }
    if (!arm_datas_.back().ee_link_)
    {
      BOLT_ERROR("No link model found for link name " << ee_tip_links[i]);
      exit(-1);
    }
  }

  // Load more visual tool objects
  loadVisualTools(indent);

  // Add a collision objects
  loadScene(indent);

  // Create start/goal state imarker
  if (!headless_)
    loadIMarkers(indent);

  // Connect to physical hardware
  if (connect_to_hardware_)
  {
    execution_interface_ = std::make_shared<moveit_boilerplate::ExecutionInterface>(psm_, visual_tools_[6]);
  }
  planning_interface_ = std::make_shared<moveit_boilerplate::PlanningInterface>(psm_, visual_tools_[6], planning_jmg_,
                                                                                execution_interface_);

  // Wait until user does something
  if (!auto_run_)
    visual_->viz1()->prompt("run first problem");

  // Run application
  eachPlanner(indent);
}

BoltBaxter::~BoltBaxter()
{
}

void BoltBaxter::reset(std::size_t indent)
{
  is_bolt_ = false;
  is_thunder_ = false;
  is_simple_setup_ = false;

  // Free start and goal states
  if (space_)
  {
    space_->freeState(ompl_start_);
    space_->freeState(ompl_goal_);
  }

  // Clear classes
  bolt_.reset();
  thunder_.reset();
  spars2_.reset();
  simple_setup_.reset();
  si_.reset();
  space_.reset();
  visual_.reset();
}

void BoltBaxter::loadVisualTools(std::size_t indent)
{
  // Load the ROS part, but not the OMPL part until loadOMPL has occured
  using namespace moveit_visual_tools;

  std::string namesp = nh_.getNamespace();

  // NOTE that all visuals start at index 1, not 0
  visual_tools_.resize(NUM_VISUALS + 1);
  for (std::size_t i = 1; i <= NUM_VISUALS; ++i)
  {
    MoveItVisualToolsPtr moveit_visual = std::make_shared<MoveItVisualTools>(
        "/world_visual" + std::to_string(i), namesp + "/ompl_visual" + std::to_string(i), robot_model_);
    moveit_visual->loadMarkerPub(false);
    moveit_visual->setPlanningSceneMonitor(psm_);
    moveit_visual->setManualSceneUpdating(true);
    moveit_visual->setGlobalScale(0.8);
    moveit_visual->enableBatchPublishing();
    moveit_visual->loadRemoteControl();
    visual_tools_[i] = moveit_visual;
  }

  ros::spinOnce();
  moveit_start_->setToDefaultValues();

  // Secondary loop to give time for all the publishers to load up
  Eigen::Affine3d offset;
  if (!headless_)
  {
    for (std::size_t i = 1; i <= NUM_VISUALS; ++i)
    {
      // Get TF
      getTFTransform("world", "world_visual" + std::to_string(i), offset);
      visual_tools_[i]->enableRobotStateRootOffet(offset);
    }
  }

  visual_tools_[6]->setBaseFrame("world");
  visual_moveit_start_ = visual_tools_[4];
  visual_moveit_goal_ = visual_tools_[5];

  ros::spinOnce();

  // Block until all visualizers are finished loading
  if (!headless_)
  {
    const double wait_time = 0.2;
    for (std::size_t i = 1; i <= NUM_VISUALS; ++i)
    {
      visual_tools_[i]->waitForMarkerPub(wait_time);

      // Load publishers
      bool blocking = false;
      visual_tools_[i]->loadRobotStatePub(namesp + "/robot_state" + std::to_string(i), blocking);

      // Show the initial robot state
      usleep(0.001 * 1000000);
      visual_tools_[i]->publishRobotState(moveit_start_);

      // Load trajectory publisher - ONLY for viz6
      if (i == 6)
        visual_tools_[i]->loadTrajectoryPub("/baxter/display_trajectory", blocking);
    }
  }

  deleteAllMarkers(indent);
}

bool BoltBaxter::loadOMPL(std::size_t indent)
{
  std::size_t visual_id = 6;  // use 6th for sampler, which is loaded inside ModelBasedStateSpace
  bolt_moveit::ModelBasedStateSpaceSpecification mbss_spec(robot_model_, planning_jmg_, visual_tools_[visual_id]);

  // Construct the state space we are planning in
  space_ = bolt_moveit::chooseModelSizeStateSpace(mbss_spec);
  si_ = std::make_shared<ob::SpaceInformation>(space_);

  // Create SimpleSetup
  if (planner_ == "Bolt")
  {
    bolt_ = std::make_shared<otb::Bolt>(si_);
    simple_setup_ = bolt_;
    is_bolt_ = true;

    // The visual pointer was already created and populated throughout the Bolt framework
    visual_ = bolt_->getVisual();
  }
  else if (planner_ == "Thunder")
  {
    thunder_ = std::make_shared<ot::Thunder>(si_);
    simple_setup_ = thunder_;
    is_thunder_ = true;

    // The visual pointer was already created and populated throughout the Bolt framework
    visual_ = thunder_->getVisual();
  }
  else  // Assume simple setup
  {
    simple_setup_ = std::make_shared<og::SimpleSetup>(si_);
    is_simple_setup_ = true;

    visual_ = std::make_shared<ot::Visualizer>();

    if (planner_ == "RRTConnect")
    {
      simple_setup_->setPlanner(std::make_shared<og::RRTConnect>(si_));
    }
    else if (planner_ == "LazyRRT")
    {
      simple_setup_->setPlanner(std::make_shared<og::LazyRRT>(si_));
    }
    else if (planner_ == "LazyPRM")
    {
      simple_setup_->setPlanner(std::make_shared<og::LazyPRM>(si_));
    }
    else if (planner_ == "SPARStwo")
    {
      spars2_ = std::make_shared<og::SPARStwo>(si_);
      simple_setup_->setPlanner(spars2_);
    }
    else
    {
      BOLT_ERROR("Unknown planner: " << planner_);
      exit(-1);
    }
  }
  // Run interface for loading rosparam settings into OMPL
  if (is_bolt_)
  {
    bolt_moveit::loadOMPLParameters(nh_, name_, bolt_);
  }

  // Load collision checker
  loadCollisionChecker(indent);

  // Add moveit_visual_tools to visual_ class now that OMPL is finished loading
  loadOMPLVisualTools(indent);

  // Setup base OMPL stuff. Do this before choosing filename so sparseDeltaFraction is ready
  BOLT_INFO(true, "Setting up SimpleSetup");
  simple_setup_->setup();
  assert(si_->isSetup());

  // Stuff that must be run after setup()
  if (is_bolt_)
  {
    bolt_->setFilePath(getPlannerFilePath(planning_group_name_, indent));
  }
  else if (is_thunder_)
  {
    thunder_->setFilePath(getPlannerFilePath(planning_group_name_, indent) + ".ompl");
    thunder_->getExperienceDB()->getSPARSdb()->setSparseDeltaFraction(0.1);  // TODO do not hardcode
  }

  // Create start and goal states
  ompl_start_ = space_->allocState();
  ompl_goal_ = space_->allocState();

  return true;
}

// Set the OMPL planner / SimpleSetup with proper visualizer
void BoltBaxter::loadOMPLVisualTools(std::size_t indent)
{
  for (std::size_t i = 1; i <= NUM_VISUALS; ++i)
  {
    bolt_moveit::MoveItVizWindowPtr viz = std::make_shared<bolt_moveit::MoveItVizWindow>(visual_tools_[i], si_);
    viz->setJointModelGroup(planning_jmg_);
    for (std::size_t i = 0; i < arm_datas_.size(); ++i)
    {
      viz->setEEFLink(arm_datas_[i].ee_link_);
    }

    // Index the visualizers
    visual_->setVizWindow(i, viz);
  }
}

bool BoltBaxter::loadData(std::size_t indent)
{
  BOLT_FUNC(true, "loadData()");

  double vm1, rss1;
  if (track_memory_consumption_)  // Track memory usage
    processMemUsage(vm1, rss1);

  // Load database or generate new roadmap
  BOLT_INFO(true, "Loading or generating roadmap");
  if (is_bolt_)
  {
    if (!bolt_->load(indent))
    {
      BOLT_INFO(true, "Unable to load sparse graph from file");
      return false;
    }
  }

  if (track_memory_consumption_)  // Track memory usage
  {
    double vm2, rss2;
    processMemUsage(vm2, rss2);
    BOLT_INFO(true, "RAM usage diff - VM: " << vm2 - vm1 << " MB | RSS: " << rss2 - rss1 << " MB");
  }

  return true;
}

// Loop through each planner to benchmark
void BoltBaxter::eachPlanner(std::size_t indent)
{
  // Logging
  if (use_logging_)
  {
    std::string file_path;
    bolt_moveit::getFilePath(file_path, "bolt_baxter_logging.csv", "ros/ompl_storage");
    logging_file_.open(file_path.c_str(), std::ios::out | std::ios::app);
    // Header of CSV file
    logging_file_ << "planner, solved, planTime, pathLength, verticesAdded, edgesAdded" << std::endl;
  }

  for (std::size_t i = 0; i < planners_.size(); ++i)
  {
    planner_ = planners_[i];

    // Feedback
    std::cout << std::endl;
    std::cout << "###################################################################### " << std::endl;
    BOLT_INFO(true, "Testing with planner '" << planner_ << "'");
    std::cout << "###################################################################### " << std::endl;

    // Load planning
    if (!loadOMPL(indent))
    {
      BOLT_ERROR("Unable to load planning context");
      exit(-1);
    }

    // --------------------------------------
    run(indent);
    // --------------------------------------

    // Wait for user
    if (i + 1 < planners_.size() - 1)  // if there are more planners to test
    {
      BOLT_INFO(true, "Next planner " << planners_[i + 1]);
      visual_->prompt("Next planner");  // Must do this before calling reset()
    }

    // Clear previous planner
    reset(indent);
  }

  logging_file_.close();
}

void BoltBaxter::run(std::size_t indent)
{
  // Benchmark performance
  if (benchmark_performance_)
  {
    benchmarkMemoryAllocation(indent);
    // testMotionValidator();
    // bolt_->getSparseGenerator()->benchmarkSparseGraphGeneration();
    // bolt_->getSparseGenerator()->benchmarkValidClearanceSampler();
    // bolt_->getSparseGenerator()->benchmarkRandValidSampling();
    // bolt_->getSparseGenerator()->benchmarkVisualizeSampling();
    // bolt_->getSparseGenerator()->benchmarkMemoryAllocation();
    BOLT_INFO(true, "Finished benchmarking");
    exit(0);
  }

  // Create list of goal states
  if (save_imarkers_to_file_)
    saveIMarkersToFile(indent);

  // View list of goal states
  if (view_imarkers_from_file_)
    viewIMarkersFromFile(indent);

  // Load from file
  bool loaded = false;
  if (is_bolt_)
  {
    if (load_spars_)
    {
      loaded = loadData(indent);
    }

    // Create SPARS
    if (create_spars_ && (!loaded || continue_spars_))
    {
      // bolt_->getSparseGenerator()->createSPARS();
      bolt_->getSparseGenerator()->createSPARS2(indent);
      loaded = true;
    }
    if (!loaded)
      BOLT_WARN(true, "Creating AND loading sparse graph disabled, no contents in graph");
  }

  // Display disconnected components
  if (display_disjoint_sets_ && is_bolt_)
  {
    displayDisjointSets(indent);
    exit(0);
  }

  // Repair missing coverage in the dense graph
  // if (eliminate_dense_disjoint_sets_)
  // {
  //   bolt_->getSparseGraph()->getDiscretizer()->eliminateDisjointSets();
  // }

  // Check for verticies that are somehow in collision
  if (check_valid_vertices_)
  {
    bolt_->getSparseGraph()->verifyGraph(indent);
    exit(0);
  }

  if (mirror_graph_)
  {
    mirrorGraph(indent);
    exit(0);
  }

  // Run the demo
  if (!run_problems_)
  {
    BOLT_INFO(true, "Solving requested to be skipped by config file");
    return;
  }

  // -----------------------------------------------
  // -----------------------------------------------
  runProblems(indent);
  // -----------------------------------------------
  // -----------------------------------------------

  if (is_bolt_)
  {
    bolt_->saveIfChanged(indent);
  }
  else if (is_thunder_)
    thunder_->saveIfChanged();
}

bool BoltBaxter::runProblems(std::size_t indent)
{
  // Run the demo the desired number of times
  for (std::size_t run_id = 0; run_id < num_problems_; ++run_id)
  {
    if (!ros::ok())  // Check if user wants to shutdown
      break;

    std::cout << std::endl;
    std::cout << "***************************************************************" << std::endl;
    BOLT_INFO(true, "Planning " << run_id + 1 << " out of " << num_problems_);
    std::cout << "***************************************************************" << std::endl;

    if (headless_)
      BOLT_WARN(true, "IMarker start/goal not loaded, should not be planning in headless mode");

    // Generate start/goal pair
    chooseStartGoal(run_id, indent);

    // Track memory usage
    double vm1, rss1;
    if (track_memory_consumption_)
      processMemUsage(vm1, rss1);

    // Populate TaskGraph, even for non-task planning
    if (is_bolt_)
    {
      if (use_task_planning_)
      {
        if (!generateCartGraph(indent))
        {
          BOLT_ERROR("Unable to create cart path");
          exit(-1);
        }
      }
      else
      {
        bolt_->getTaskGraph()->generateMonoLevelTaskSpace(indent);
      }
    }

    // Track memory usage
    if (track_memory_consumption_)
    {
      double vm2, rss2;
      processMemUsage(vm2, rss2);
      BOLT_INFO(true, "RAM usage diff (VM, RSS) MB:\n" << vm2 - vm1 << ", " << rss2 - rss1);
    }

    // -----------------------------------------------------
    // -----------------------------------------------------
    bool solved = plan(indent);
    // -----------------------------------------------------
    // -----------------------------------------------------

    if (is_bolt_)
    {
      bolt_->processResults(indent);
      bolt_->printLogs();
    }

    // Post Proccess
    if (post_processing_ && run_id % post_processing_interval_ == 0)  // every x runs
    {
      BOLT_INFO(true, "Performing post processing every " << post_processing_interval_ << " plans");
      doPostProcessing(indent);
    }

    // Logging
    if (use_logging_)
      log(solved, indent);

    // Prompt user
    if (visualize_wait_between_plans_ && run_id < num_problems_ - 1)
      visual_->viz1()->prompt("run next problem");
    else  // Main pause between planning instances - allows user to analyze
      ros::Duration(visualize_time_between_plans_).sleep();
    if (!ros::ok())  // Check if user wants to shutdown
      break;

    // Reset marker if this is not our last run
    if (run_id < num_problems_ - 1)
      deleteAllMarkers(indent);
  }  // for each run

  // Save experience
  if (post_processing_)
    doPostProcessing(indent);

  // Finishing up
  if (is_bolt_)
  {
    BOLT_INFO(true, "Saving experience db...");
    bolt_->saveIfChanged(indent);
  }

  return true;
}

bool BoltBaxter::plan(std::size_t indent)
{
  BOLT_FUNC(true, "plan()");

  // Setup -----------------------------------------------------------

  // Clear all planning data. This only includes data generated by motion plan computation.
  // Planner settings, start & goal states are not affected.
  if (is_bolt_)
    bolt_->clearForNextPlan(indent);
  else
    simple_setup_->clear();

  // Convert MoveIt state to OMPL state
  space_->copyToOMPLState(ompl_start_, *moveit_start_);
  space_->copyToOMPLState(ompl_goal_, *moveit_goal_);

  // Set the start and goal states
  simple_setup_->setStartAndGoalStates(ompl_start_, ompl_goal_);

  // Solve -----------------------------------------------------------

  // Create the termination condition
  ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(planning_time_, 0.1);

  // Profiler
  CALLGRIND_TOGGLE_COLLECT;

  // Attempt to solve the problem within x seconds of planning time
  if (!simple_setup_->solve(ptc))
  {
    BOLT_ERROR("No solution found");
    return false;
  }

  // Profiler
  CALLGRIND_TOGGLE_COLLECT;
  CALLGRIND_DUMP_STATS;

  // Simplify
  if (!is_bolt_)
  {
    simple_setup_->simplifySolution();
  }

  // Interpolate, parameterize, and execute/visualize
  processAndExecute(indent);

  return true;
}

void BoltBaxter::loadCollisionChecker(std::size_t indent)
{
  // Create state validity checking for this space
  validity_checker_ = std::make_shared<bolt_moveit::StateValidityChecker>(planning_group_name_, si_, *current_state_,
                                                                          planning_scene_, space_);
  validity_checker_->setCheckingEnabled(collision_checking_enabled_);

  // Set checker
  si_->setStateValidityChecker(validity_checker_);

  // The interval in which obstacles are checked for between states
  // seems that it default to 0.01 but doesn't do a good job at that level
  // si_->setStateValidityCheckingResolution(0.005);
  si_->setStateValidityCheckingResolution(0.001);

  // Allow collision checker to visualize
  validity_checker_->setVisual(visual_);
}

void BoltBaxter::deleteAllMarkers(std::size_t indent)
{
  if (headless_)
    return;

  // Reset rviz markers
  for (std::size_t i = 1; i < NUM_VISUALS; ++i)
  {
    visual_tools_[i]->deleteAllMarkers();
    visual_tools_[i]->trigger();
  }
}

void BoltBaxter::visualizeStartGoal(std::size_t indent)
{
  visual_moveit_start_->publishRobotState(moveit_start_, rvt::GREEN);
  visual_moveit_goal_->publishRobotState(moveit_goal_, rvt::ORANGE);

  // Show values and limits
  // std::cout << "Start: " << std::endl;
  // visual_moveit_start_->showJointLimits(moveit_start_);
  // std::cout << "Goal: " << std::endl;
  // visual_moveit_start_->showJointLimits(moveit_goal_);
}

void BoltBaxter::displayWaitingState(bool waiting)
{
  // std::cout << " TODO display waiting state " << std::endl;
  // if (waiting)
  //   publishViewFinderFrame(rvt::MEDIUM);
  // else
  //   publishViewFinderFrame(rvt::XSMALL);

  // viz_bg_->trigger();
}

void BoltBaxter::testConnectionToGraphOfRandStates(std::size_t indent)
{
  ob::State *random_state = space_->allocState();

  std::size_t successful_connections = 0;
  for (std::size_t run_id = 0; run_id < num_problems_; ++run_id)
  {
    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    BOLT_INFO(true, "Testing random state " << run_id);

    // Generate random state
    getRandomState(moveit_start_);

    // Visualize
    visual_moveit_start_->publishRobotState(moveit_start_, rvt::GREEN);

    // Convert to ompl
    space_->copyToOMPLState(random_state, *moveit_start_);

    // Test
    const ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(60.0);
    std::size_t indent = 0;

    BOLT_ERROR("bolt_baxter: not implemented");
    // bool result = bolt_->getBoltPlanner()->canConnect(random_state, ptc, indent);
    // if (result)
    //   successful_connections++;

    BOLT_ERROR("Percent connnected: " << successful_connections / double(run_id + 1) * 100.0);
  }

  space_->freeState(random_state);
}

void BoltBaxter::visualizeRawTrajectory(og::PathGeometric &path, std::size_t indent)
{
  BOLT_INFO(true, "Visualizing non-interpolated trajectory");

  // Convert trajectory
  robot_trajectory::RobotTrajectoryPtr traj;
  const double speed = 0.05;
  space_->convertPathToRobotState(path, planning_jmg_, traj, speed);

  // Show trajectory line
  visual_tools_[3]->publishTrajectoryLine(traj, arm_datas_[0].ee_link_, rvt::GREY);  // TODO multiple EEs
  visual_->viz3()->trigger();
}

bool BoltBaxter::generateCartGraph(std::size_t indent)
{
  // Generate the Descartes graph - if it fails let user adjust interactive marker
  while (true)
  {
    if (!cart_path_planner_->populateBoltGraph(bolt_->getTaskGraph(), indent))
    {
      BOLT_INFO(true, "Unable to populate Bolt graph - try moving the start location");
      visual_->viz1()->prompt("attempt Bolt graph generation again");
      if (!ros::ok())
        exit(0);
    }
    else
      break;
  }

  return true;
}

bool BoltBaxter::checkMoveItPathSolution(robot_trajectory::RobotTrajectoryPtr traj, std::size_t indent)
{
  std::size_t state_count = traj->getWayPointCount();
  if (state_count < 3)
  {
    BOLT_WARN(true, "checkMoveItPathSolution: Solution path has only " << state_count << " states");
  }
  else
  {
    BOLT_INFO(true, "checkMoveItPathSolution: Solution path has " << state_count << " states");
  }

  std::vector<std::size_t> index;
  const bool verbose = true;
  std::cout << "before isPathValid " << std::endl;
  if (planning_scene_->isPathValid(*traj, "", verbose, &index))
  {
    return true;
  }

  std::cout << "after isPathValid " << std::endl;
  // if (index.size() == 1 && index[0] == 0)  // ignore cases when the robot starts at invalid location
  //   ROS_DEBUG("It appears the robot is starting at an invalid state, but that is ok.");
  // else
  {
    // display error messages
    std::stringstream ss;
    for (std::size_t i = 0; i < index.size(); ++i)
      ss << index[i] << " ";
    BOLT_ERROR("checkMoveItPathSolution: Computed path is not valid. Invalid states at index locations: [ "
               << ss.str() << "] out of " << state_count << ". Explanations follow in command line.");

    // Call validity checks in verbose mode for the problematic states
    visualization_msgs::MarkerArray combined_array;
    for (std::size_t i = 0; i < index.size(); ++i)
    {
      // check validity with verbose on
      const robot_state::RobotState &robot_state = traj->getWayPoint(index[i]);
      bool check_verbose = true;
      planning_scene_->isStateValid(robot_state, planning_group_name_, check_verbose);

      // compute the contacts if any
      collision_detection::CollisionRequest c_req;
      collision_detection::CollisionResult c_res;
      c_req.contacts = true;
      c_req.max_contacts = 10;
      c_req.max_contacts_per_pair = 3;
      c_req.verbose = false;
      planning_scene_->checkCollision(c_req, c_res, robot_state);

      if (c_res.contact_count > 0)
      {
        visualization_msgs::MarkerArray single_array;
        collision_detection::getCollisionMarkersFromContacts(single_array, planning_scene_->getPlanningFrame(),
                                                             c_res.contacts);
        combined_array.markers.insert(combined_array.markers.end(), single_array.markers.begin(),
                                      single_array.markers.end());
      }
    }  // end for
    // publish marker array
    visual_tools_[6]->publishMarkers(combined_array);

    BOLT_ERROR("checkMoveItPathSolution: Completed listing of explanations for invalid states.");
  }

  return false;
}

bool BoltBaxter::getRandomState(moveit::core::RobotStatePtr &robot_state)
{
  std::size_t indent = 0;
  static const std::size_t MAX_ATTEMPTS = 1000;
  for (std::size_t i = 0; i < MAX_ATTEMPTS; ++i)
  {
    robot_state->setToRandomPositions(planning_jmg_);
    robot_state->update();

    // Error check
    bool check_verbose = false;
    if (planning_scene_->isStateValid(*robot_state, "", check_verbose))  // second argument is what planning group to
                                                                         // collision check, "" is everything
    {
      return true;
    }

    if (i == 100)
      BOLT_WARN(true, "Taking long time to find valid random state");
  }

  BOLT_ERROR("Unable to find valid random robot state");
  exit(-1);
  return false;
}

void BoltBaxter::testMotionValidator(std::size_t indent)
{
  // THIS FUNCTION BROKEN BECAUSE moveit_core SAYS "FCL continuous collision checking not yet implemented"

  // moveit::core::RobotStatePtr start = moveit::core::RobotStatePtr(new moveit::core::RobotState(*current_state_));
  // moveit::core::RobotStatePtr goal = moveit::core::RobotStatePtr(new moveit::core::RobotState(*current_state_));
  moveit_start_->setToRandomPositions(planning_jmg_);
  moveit_goal_->setToRandomPositions(planning_jmg_);

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

void BoltBaxter::mirrorGraph(std::size_t indent)
{
  // Choose planning group
  both_arms_jmg_ = robot_model_->getJointModelGroup(both_arms_group_name_);
  left_arm_jmg_ = robot_model_->getJointModelGroup(opposite_arm_name_);

  // Setup space
  bolt_moveit::ModelBasedStateSpaceSpecification both_arms_mbss_spec(robot_model_, both_arms_jmg_);
  bolt_moveit::ModelBasedStateSpaceSpecification left_arm_mbss_spec(robot_model_, left_arm_jmg_);

  // Construct the state space we are planning in
  both_arms_state_space_ = bolt_moveit::chooseModelSizeStateSpace(both_arms_mbss_spec);
  left_arm_state_space_ = bolt_moveit::chooseModelSizeStateSpace(left_arm_mbss_spec);

  both_arms_state_space_->setup();
  both_arms_state_space_->setName(both_arms_group_name_);
  left_arm_state_space_->setup();
  left_arm_state_space_->setName(opposite_arm_name_);

  // SpaceInfo
  ob::SpaceInformationPtr both_arms_space_info = std::make_shared<ob::SpaceInformation>(both_arms_state_space_);
  ob::SpaceInformationPtr left_arm_space_info = std::make_shared<ob::SpaceInformation>(left_arm_state_space_);
  both_arms_space_info->setup();
  left_arm_space_info->setup();

  // Create state validity checking for both arms
  bolt_moveit::StateValidityCheckerPtr both_arms_validity_checker = std::make_shared<bolt_moveit::StateValidityChecker>(
      both_arms_group_name_, both_arms_space_info, *current_state_, planning_scene_, both_arms_state_space_);
  both_arms_space_info->setStateValidityChecker(both_arms_validity_checker);

  // Create state validity checking for left arm
  bolt_moveit::StateValidityCheckerPtr left_arm_validity_checker = std::make_shared<bolt_moveit::StateValidityChecker>(
      opposite_arm_name_, left_arm_space_info, *current_state_, planning_scene_, left_arm_state_space_);
  left_arm_space_info->setStateValidityChecker(left_arm_validity_checker);

  // Set the database file location
  const std::string file_path = getPlannerFilePath(both_arms_group_name_, indent);

  // Test all verticies
  if (false)
  {
    BOLT_INFO(true, "TESTING ALL VERTICES ON OTHER ARM");
    bolt_->getSparseMirror()->checkValidityOfArmMirror(both_arms_space_info, left_arm_space_info, indent);
    std::cout << "success " << std::endl;
    exit(0);
  }

  // Set callback for how to combine two arms into one state
  bolt_->getSparseMirror()->setCombineStatesCallback(boost::bind(&BoltBaxter::combineStates, this, _1, _2));

  // Mirror graph
  bolt_->getSparseMirror()->mirrorGraphDualArm(both_arms_space_info, left_arm_space_info, file_path, indent);
  BOLT_INFO(true, "Done mirroring graph!");
}

ob::State *BoltBaxter::combineStates(const ob::State *state1, const ob::State *state2)
{
  /* Notes:
     state1
     state space: space_
     jmg: planning_jmg_
     state2
     state space: left_arm_state_space_
     jmg: left_arm_jmg_
     return state
     state space: both_arms_state_space_
     jmg: both_arms_jmg_
  */

  ob::State *both_arms_state = both_arms_state_space_->allocState();

  // Get the values of the individual states
  std::vector<double> values1, values2;
  si_->getStateSpace()->copyToReals(values1, state1);
  si_->getStateSpace()->copyToReals(values2, state2);

  // Set the vectors for each joint model group
  // TODO: its possible the vectors do not align correctly for some robots, but I'm not sure
  mirror_state_->setJointGroupPositions(planning_jmg_, values1);
  mirror_state_->setJointGroupPositions(left_arm_jmg_, values2);

  // Fill the state with current values
  both_arms_state_space_->copyToOMPLState(both_arms_state, *mirror_state_);

  if (false)
  {
    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    space_->printState(state1);

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    left_arm_state_space_->printState(state2);

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    both_arms_state_space_->printState(both_arms_state);

    visual_->viz1()->prompt("compare combination");
  }

  return both_arms_state;
}

void BoltBaxter::benchmarkMemoryAllocation(std::size_t indent)
{
  std::cout << "-------------------------------------------------------" << std::endl;
  OMPL_INFORM("BoltBaxter: Running memory allocation benchmark");

  // std::size_t numStates = 10000000;
  std::size_t numStates = 2;
  std::size_t dim = 14;
  std::size_t tests = 2;

  bolt_moveit::ModelBasedStateSpaceSpecification mbss_spec(robot_model_, planning_jmg_);
  bolt_moveit::ModelBasedStateSpace space_old(mbss_spec);
  // bolt_moveit::ModelBasedStateSpacePtr space = bolt_moveit::chooseModelSizeStateSpace(mbss_spec);

  // METHOD 1
  ros::Time start_time = ros::Time::now();  // Benchmark runtime
  for (std::size_t test = 0; test < tests; ++test)
  {
    // Allocate
    std::vector<ob::State *> states;
    for (std::size_t i = 0; i < numStates; ++i)
      states.push_back(space_old.allocState());

    // Free
    for (std::size_t i = 0; i < numStates; ++i)
      space_old.freeState(states[i]);
  }
  BOLT_INFO(true, "Old state - Total time: " << (ros::Time::now() - start_time).toSec() << " seconds");

  // METHOD 2
  // ros::Time start_time2 = ros::Time::now(); // Benchmark runtime
  // for (std::size_t test = 0; test < tests; ++test)
  // {
  //   // Allocate
  //   std::vector<ob::State*> states;
  //   for (std::size_t i = 0; i < numStates; ++i)
  //     states.push_back(space->allocState());

  //   // Free
  //   for (std::size_t i = 0; i < numStates; ++i)
  //     space->freeState(states[i]);
  // }
  // BOLT_INFO(true, "New state - Total time: " << (ros::Time::now() - start_time2).toSec() << " seconds");

  // METHOD 3
  /*
    ros::Time start_time0 = ros::Time::now();  // Benchmark runtime
    for (std::size_t test = 0; test < tests; ++test)
    {
    using namespace bolt_moveit;

    // Allocate
    // ompl::base::State* states;
    // space->allocStates(numStates, states);
    ModelSize14StateSpace::StateType *states = new ModelSize14StateSpace::StateType[numStates];

    std::cout << "allocStates: " << std::endl;
    for (std::size_t i = 0; i < numStates; ++i)
    {
    std::cout << " - states[i]: " << &states[i] << std::endl;
    for (std::size_t j = 0; j < 14; ++j)
    {
    std::cout << "     - value " << j << ": " << states[i].as<ModelSize14StateSpace::StateType>()->values[j] <<
    std::endl;
    }
    }

    std::cout << "allocated states " << std::endl;

    // Free
    // space->freeStates(states);
    for (std::size_t i = 0; i < numStates; ++i)
    {
    std::cout << "i: " << i << std::endl;
    // std::cout << "states[i]: " << states[i] << std::endl;
    std::cout << &states[i] << " &states[i]" << std::endl;
    std::cout << (&states[i])->as<ModelSize14StateSpace::StateType>() << "
    (&states[i])->as<ModelSize14StateSpace::StateType>()" << std::endl;
    std::cout << (&states[i])->as<ModelSize14StateSpace::StateType>()->values << "
    (&states[i])->as<ModelSize14StateSpace::StateType>()->values" << std::endl;

    for (std::size_t j = 0; j < 14; ++j)
    {
    std::cout << " - " << (&states[i])->as<ModelSize14StateSpace::StateType>()->values[j] << std::endl;
    }

    std::cout << "1 " << std::endl;
    (&states[i])->as<ModelSize14StateSpace::StateType>()->values[0] = 0;
    std::cout << "2 " << std::endl;
    (&states[i])->as<ModelSize14StateSpace::StateType>()->values[7] = 7;
    (&states[i])->as<ModelSize14StateSpace::StateType>()->values[13] = 13;

    std::cout << "delete: " << std::endl;
    delete[] states;
    // space->freeState(&states[i]);
    }
    }
    BOLT_INFO(true, "Array Total time: " << (ros::Time::now() - start_time0).toSec() << " seconds");
  */
  visual_->viz1()->prompt("finished running");

  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << std::endl;
}

void BoltBaxter::loadScene(std::size_t indent)
{
  switch (scene_type_)
  {
    case 1:
      loadAmazonScene(indent);
    // break;
    case 0:
      loadOfficeScene(indent);
      break;
  }

  visual_moveit_start_->triggerPlanningSceneUpdate();

  // Append to allowed collision matrix
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(psm_);  // Lock planning
    collision_detection::AllowedCollisionMatrix &collision_matrix = scene->getAllowedCollisionMatrixNonConst();
    collision_matrix.setEntry("wall", "pedestal", true);
  }
}

void BoltBaxter::loadOfficeScene(std::size_t indent)
{
  BOLT_FUNC(true, "loadOfficeScene()");
  // psm_->updateFrameTransforms();

  // const double table_height = -0.77 * baxter_torso_height_;
  const double table_height = -0.75 * baxter_torso_height_;
  visual_moveit_start_->publishCollisionFloor(baxter_torso_height_ + 0.001, "floor", rvt::TRANSLUCENT_DARK);
  visual_moveit_start_->publishCollisionWall(/*x*/ -1.0, /*y*/ 0.0, /*z*/ baxter_torso_height_, /*angle*/ 0,
                                             /*width*/ 2, /*height*/ 2.0, "wall1", rvt::YELLOW);
  visual_moveit_start_->publishCollisionWall(/*x*/ 0.0, /*y*/ -1.075, /*z*/ baxter_torso_height_, /*angle*/ M_PI / 2.0,
                                             /*width*/ 2, /*height*/ 2.0, "wall2", rvt::YELLOW);
  visual_moveit_start_->publishCollisionWall(/*x*/ 0.0, /*y*/ 1.075, /*z*/ baxter_torso_height_, /*angle*/ M_PI / 2.0,
                                             /*width*/ 2, /*height*/ 2.0, "wall3", rvt::YELLOW);
  visual_moveit_start_->publishCollisionTable(/*x*/ 0.85, /*y*/ 0.0, /*z*/ baxter_torso_height_, /*angle*/ 0,
                                              /*width*/ 2.0, /*height*/ table_height, /*depth*/ 0.8, "table",
                                              rvt::DARK_GREY);
}

void BoltBaxter::loadAmazonScene(std::size_t indent)
{
  BOLT_FUNC(true, "loadAmazonScene()");

  // Load mesh file name
  std::string collision_mesh_path = "file://" + package_path_ + "/meshes/kiva_pod/meshes/pod_lowres.stl";

  BOLT_INFO(true, "Loading mesh from " << collision_mesh_path);

  Eigen::Affine3d mesh_centroid = Eigen::Affine3d::Identity();

  // Calculate offset
  mesh_centroid = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  mesh_centroid.translation().x() = distance_to_shelf_;
  mesh_centroid.translation().y() = 0;
  mesh_centroid.translation().z() = baxter_torso_height_;

  const std::string collision_object_name = "shelf";

  shapes::Shape *mesh = shapes::createMeshFromResource(collision_mesh_path);  // make sure its prepended by file://
  shapes::ShapeMsg shape_msg;  // this is a boost::variant type from shape_messages.h
  if (!mesh || !shapes::constructMsgFromShape(mesh, shape_msg))
  {
    BOLT_ERROR("Unable to create mesh shape message from resource " << collision_mesh_path);
    return;
  }

  shape_msgs::Mesh mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);

  // Add mesh to scene
  visual_tools_[6]->publishCollisionMesh(mesh_centroid, collision_object_name, mesh_msg, rvt::BROWN);
}

void BoltBaxter::saveIMarkersToFile(std::size_t indent)
{
  std::string file_path;
  bolt_moveit::getFilePath(file_path, imarker_goal_list_, "ros/ompl_storage");
  BOLT_INFO(true, "Saving goal states to " << file_path);

  std::ofstream output_file;
  output_file.open(file_path, std::ofstream::out | std::ofstream::app);  // append

  while (ros::ok())
  {
    visual_->viz1()->prompt("save state to file...");

    if (!imarker_goal_->isStateValid())
    {
      BOLT_WARN(true, "IMarker goal state is in collision, try again");
      continue;
    }

    moveit::core::robotStateToStream(*imarker_goal_->getRobotState(), output_file, false);
  }

  output_file.close();
}

void BoltBaxter::viewIMarkersFromFile(std::size_t indent)
{
  BOLT_FUNC(true, "viewIMarkersFromFile()");
  std::vector<moveit::core::RobotStatePtr> robot_states;

  loadIMarkersFromFile(robot_states, indent);

  std::size_t count = 0;
  for (moveit::core::RobotStatePtr state : robot_states)
  {
    BOLT_INFO(true, "Viewing state " << count++);
    visual_tools_[6]->publishRobotState(state);
    ros::Duration(1.0).sleep();
  }

  BOLT_INFO(true, "Done, shutting down.");
  exit(0);
}

void BoltBaxter::loadIMarkersFromFile(std::vector<moveit::core::RobotStatePtr> &robot_states, std::size_t indent)
{
  BOLT_FUNC(true, "loadIMarkersFromFile()");

  std::string file_path;
  bolt_moveit::getFilePath(file_path, imarker_goal_list_, "ros/ompl_storage");
  BOLT_INFO(true, "Loading goal states from " << file_path);

  // Error check
  if (!boost::filesystem::exists(file_path))
  {
    BOLT_WARN(true, "File not found: " << file_path);
    return;
  }

  std::ifstream input_file(file_path);

  std::string line;
  std::size_t count = 0;
  while (std::getline(input_file, line))
  {
    moveit::core::streamToRobotState(*imarker_goal_->getRobotState(), line);
    robot_states.push_back(std::make_shared<moveit::core::RobotState>(*imarker_goal_->getRobotState()));
  }

  input_file.close();
}

void BoltBaxter::loadIMarkers(std::size_t indent)
{
  // Create cartesian planner
  if (use_task_planning_)
  {
    cart_path_planner_ = std::make_shared<bolt_moveit::CartPathPlanner>(arm_datas_, visual_tools_[6], moveit_start_,
                                                                        psm_, package_path_, space_, planning_jmg_);
  }

  // Create start/goal imarkers
  if (!connect_to_hardware_)  // if running on hardware, these imarkers are not needed
    imarker_start_ = std::make_shared<mvt::IMarkerRobotState>(psm_, "start", arm_datas_, rvt::GREEN, package_path_);
  imarker_goal_ = std::make_shared<mvt::IMarkerRobotState>(psm_, "goal", arm_datas_, rvt::ORANGE, package_path_);

  // Error message until current state is valid
  if (connect_to_hardware_)
  {
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    bool check_verbose = true;
    while (!planning_scene_->isStateValid(*getCurrentState(), "", check_verbose) && ros::ok())
    {
      BOLT_ERROR("START STATE INVALID " << ros::Time::now());
      visual_tools_[5]->publishRobotState(getCurrentState(), rvt::RED);
      ros::spinOnce();
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }
  }
}

robot_trajectory::RobotTrajectoryPtr BoltBaxter::processSimpleSolution(std::size_t indent)
{
  BOLT_FUNC(true, "processSimpleSolution()");

  og::PathGeometric &path = static_cast<og::PathGeometric &>(*simple_setup_->getProblemDefinition()->getSolutionPath());

  last_plan_path_length_ = path.length();

  // Have additional visualizations that mimmic those in BoltPlanner
  visual_->viz6()->deleteAllMarkers();
  visual_->viz6()->path(&path, ot::MEDIUM, ot::BLACK, ot::BLUE);
  visual_->viz6()->trigger();

  // Convert trajectory from OMPL to MoveIt! format
  robot_trajectory::RobotTrajectoryPtr trajectory;
  const double speed = 0.025;
  if (!space_->convertPathToRobotState(path, planning_jmg_, trajectory, speed))
  {
    BOLT_ERROR("Unable to convert path");
    return false;
  }

  double velocity_scaling_factor = velocity_scaling_factor_;

  // Interpolate and parameterize
  const bool use_interpolation = false;
  planning_interface_->convertRobotStatesToTraj(trajectory, planning_jmg_, velocity_scaling_factor, use_interpolation);

  return trajectory;
}

robot_trajectory::RobotTrajectoryPtr BoltBaxter::processSegments(std::size_t indent)
{
  BOLT_FUNC(true, "processSegments()");

  // Visualize if not already done so within BoltPlanner
  if (!bolt_->getBoltPlanner()->visualizeRawTrajectory_)
    bolt_->getBoltPlanner()->visualizeRaw(indent);
  if (!bolt_->getBoltPlanner()->visualizeSmoothTrajectory_)
    bolt_->getBoltPlanner()->visualizeSmoothed(indent);

  // Get solution segments
  std::vector<og::PathGeometricPtr> model_sol_segments = bolt_->getBoltPlanner()->getModelSolSegments();
  robot_trajectory::RobotTrajectoryPtr combined_traj =
      std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, planning_jmg_);

  // For each segment of trajectory
  for (std::size_t i = 0; i < model_sol_segments.size(); ++i)
  {
    og::PathGeometricPtr path_segment = model_sol_segments[i];

    // Convert trajectory from OMPL to MoveIt! format
    robot_trajectory::RobotTrajectoryPtr traj_segment;
    const double speed = 0.025;
    if (!space_->convertPathToRobotState(*path_segment, planning_jmg_, traj_segment, speed))
    {
      BOLT_ERROR("Unable to convert path");
      return false;
    }

    // Check/test the solution for errors
    // if (!checkMoveItPathSolution(traj_segment))
    // {
    //   BOLT_WARN(true, "Invalid path");
    // }

    // Loop through each state in subtrajectory
    if (false)
      for (std::size_t i = 0; i < traj_segment->getWayPointCount(); ++i)
      {
        std::cout << "i: " << i << std::endl;
        visual_tools_[6]->publishRobotState(traj_segment->getWayPoint(i), rvt::BLUE);

        // traj_segment->getWayPoint(i).printStateInfo();
        visual_->viz1()->prompt("next step");
      }

    // For the cartesian path, go real slow
    double velocity_scaling_factor = velocity_scaling_factor_;
    if (i == 1)
      velocity_scaling_factor = 0.1;

    // Interpolate and parameterize
    const bool use_interpolation = false;
    planning_interface_->convertRobotStatesToTraj(traj_segment, planning_jmg_, velocity_scaling_factor,
                                                  use_interpolation);

    // Add to combined traj
    const double dt = i == 0 ? 0.0 : 1.0;  // Quick pause between segments except first one
    combined_traj->append(*traj_segment, dt);
  }

  // Get the resulting length
  last_plan_path_length_ = bolt_->getBoltPlanner()->getSmoothedModelSolPath()->length();

  return combined_traj;
}

void BoltBaxter::chooseStartGoal(std::size_t run_id, std::size_t indent)
{
  BOLT_FUNC(true, "chooseStartGoal()");

  switch (problem_type_)
  {
    case 0:                       // random
      if (!connect_to_hardware_)  // if running on hardware, these markers are not needed
        imarker_start_->setToRandomState();
      imarker_goal_->setToRandomState();
      break;
    case 1:  // imarkers
      // do nothing
      break;
    case 2:  // imarker goal list
    {
      // start is always the same
      imarker_start_->getRobotState()->setToDefaultValues(planning_jmg_, "both_ready");
      imarker_start_->publishState();

      // load goals
      std::vector<moveit::core::RobotStatePtr> robot_states;
      loadIMarkersFromFile(robot_states, indent);

      // choose goal
      std::size_t state_id = run_id % robot_states.size();
      std::cout << "total states: " << robot_states.size() << " run_id: " << run_id << " state_id: " << state_id
                << std::endl;

      imarker_goal_->getRobotState() = robot_states[state_id];
      imarker_goal_->publishState();
    }
    break;
    default:
      BOLT_ERROR("Unknown problem type");
  }

  // Get start
  if (!connect_to_hardware_)  // if running on hardware, these markers are not needed
    moveit_start_ = imarker_start_->getRobotState();
  else
    moveit_start_ = getCurrentState();

  // Get goal
  moveit_goal_ = imarker_goal_->getRobotState();

  // Visualize
  if (visualize_start_goal_states_)
    visualizeStartGoal(indent);
}

void BoltBaxter::displayDisjointSets(std::size_t indent)
{
  std::cout << std::endl;
  BOLT_INFO(true, "Displaying disjoint sets ----------- ");
  ot::bolt::SparseDisjointSetsMap disjointSets;
  bolt_->getSparseGraph()->getDisjointSets(disjointSets, indent);
  bolt_->getSparseGraph()->printDisjointSets(disjointSets, indent);
  bolt_->getSparseGraph()->visualizeDisjointSets(disjointSets, indent);
}

std::string BoltBaxter::getPlannerFilePath(const std::string &planning_group_name, std::size_t indent)
{
  // Set the database file location

  std::string planner_lower_ = planner_;
  std::transform(planner_lower_.begin(), planner_lower_.end(), planner_lower_.begin(), ::tolower);

  std::string file_name;
  if (is_bolt_)
  {
    file_name = file_name + planner_lower_ + "_" + planning_group_name + "_" +
                std::to_string(bolt_->getSparseCriteria()->sparseDeltaFraction_) + "_database_v" +
                std::to_string(load_database_version_);
  }
  else
  {
    file_name = file_name + planner_lower_ + "_" + planning_group_name + "_database";
  }

  std::string file_path;
  bolt_moveit::getFilePath(file_path, file_name, "ros/ompl_storage");
  return file_path;
}

void BoltBaxter::doPostProcessing(std::size_t indent)
{
  if (is_bolt_)
  {
    bolt_->doPostProcessing(indent);
  }
  else if (is_thunder_)
  {
    thunder_->doPostProcessing();
  }
}

void BoltBaxter::loadSPARS2Data(std::size_t indent)
{
  using namespace rosparam_shortcuts;
  std::size_t error = 0;

  double sparseDeltaFraction_;
  double denseDeltaFraction_;
  double nearSamplePointsMultiple_;
  double stretchFactor_;
  double penetrationOverlapFraction_;
  bool useL2Norm_;
  {
    ros::NodeHandle rpnh(nh_, "sparse_criteria");
    error += !get(name_, rpnh, "sparse_delta_fraction", sparseDeltaFraction_);
    error += !get(name_, rpnh, "dense_delta_fraction", denseDeltaFraction_);
    error += !get(name_, rpnh, "near_sample_points_multiple", nearSamplePointsMultiple_);
    error += !get(name_, rpnh, "stretch_factor", stretchFactor_);
    error += !get(name_, rpnh, "penetration_overlap_fraction", penetrationOverlapFraction_);
    error += !get(name_, rpnh, "use_l2_norm", useL2Norm_);
  }

  // Mimmic bolt method for calculating
  ompl::tools::bolt::SparseFormula formulas;
  formulas.calc(si_, stretchFactor_, sparseDeltaFraction_, penetrationOverlapFraction_, nearSamplePointsMultiple_,
                useL2Norm_, indent);

  spars2_->setSparseDeltaFraction(sparseDeltaFraction_);
  spars2_->setDenseDeltaFraction(denseDeltaFraction_);
  spars2_->setStretchFactor(formulas.stretchFactor_);  // uses same method as Bolt to calculate

  {
    ros::NodeHandle rpnh(nh_, "sparse_generator");
    std::size_t temp, total_failures;
    error += !get(name_, rpnh, "terminate_after_failures", temp);
    total_failures = temp;
    error += !get(name_, rpnh, "fourth_criteria_after_failures", temp);
    total_failures += temp;
    // total_failures *= 2; // Because SPARS2 keeps failing the optimiality test with "no neighbors found"
    std::cout << " Found total failures: " << total_failures << std::endl;
    spars2_->setMaxFailures(total_failures);
  }

  // Clearance
  // {
  //   ros::NodeHandle rpnh(nh_, "sparse_graph");
  //   double clearance;
  //   error += !get(name_, rpnh, "obstacle_clearance", clearance);
  //   spars2_->setClearance(clearance);
  // }

  shutdownIfError(name_, error);
}

void BoltBaxter::log(bool solved, std::size_t indent)
{
  std::size_t numVerticesAdded = 0;
  std::size_t numEdgesAdded = 0;
  double planTime;
  if (is_bolt_)
  {
    planTime = bolt_->getPlanTime();

    // Get a copy of the stats and clear them from the Bolt setup
    otb::ExperiencePathStats stats = bolt_->getPostProcessingResultsAndReset();
    numVerticesAdded = stats.numVerticesAdded_;
    numEdgesAdded = stats.numEdgesAdded_;
  }
  else
  {
    planTime = simple_setup_->getLastSimplificationTime() + simple_setup_->getLastPlanComputationTime();
  }

  logging_file_ << planner_ << ", "                    // bolt, etc
                << solved << ", " << planTime << ", "  // smoothing + planning
                << last_plan_path_length_ << ", "      // basic planning stats
                << numVerticesAdded << ", "            // numVerticesAdded
                << numEdgesAdded                       // numEdgesAdded
                << std::endl;
  logging_file_.flush();
}

void BoltBaxter::processAndExecute(std::size_t indent)
{
  robot_trajectory::RobotTrajectoryPtr execution_traj;

  // Interpolate and parameterize
  if (is_bolt_)
  {
    execution_traj = processSegments(indent);
  }
  else  // RRTConnect, etc
  {
    execution_traj = processSimpleSolution(indent);
  }

  // Execute
  if (connect_to_hardware_)
  {
    bool wait_for_execution = true;
    execution_interface_->executeTrajectory(execution_traj, planning_jmg_, wait_for_execution);
  }
  else  // Simulation
  {
    // visual_tools_[6]->deleteAllMarkers();
    // visual_tools_[6]->publishTrajectoryLine(execution_traj, planning_jmg_, rvt::LIME_GREEN);
    // visual_tools_[6]->trigger();
    bool blocking = false;
    visual_tools_[6]->publishTrajectoryPath(execution_traj, blocking);
  }
}

}  // namespace bolt_baxter
