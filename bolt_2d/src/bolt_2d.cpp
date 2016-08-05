/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/*
  Author: Dave Coleman <dave@dav.ee>
  Desc:   Visualize experience planning with OMPL in Rviz
*/

// ROS
#include <ros/ros.h>
#include <ros/package.h>  // for getting file path of package names

// Display in Rviz tool
#include <ompl_visual_tools/ros_viz_window.h>
#include <ompl/tools/debug/VizWindow.h>
#include <bolt_2d/validity_checker_2d.h>

// OMPL
#include <ompl/tools/lightning/Lightning.h>
#include <ompl/tools/thunder/Thunder.h>
#include <ompl/tools/bolt/Bolt.h>
//#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>

// Interface for loading rosparam settings into OMPL
#include <moveit_ompl/ompl_rosparam.h>
#include <moveit_ompl/remote_control.h>

namespace ob = ompl::base;
namespace ot = ompl::tools;
namespace otb = ompl::tools::bolt;
namespace og = ompl::geometric;
namespace rvt = rviz_visual_tools;

namespace bolt_2d
{
static const std::string BASE_FRAME = "/world";

enum PlannerName
{
  BOLT,
  THUNDER,
  SPARS,
  SPARS2,
  PRM
};

/**
 * \brief Experience Planning Class
 */
class ExperienceDemos
{
public:
  /**
   * \brief Constructor
   */
  ExperienceDemos() : nh_("~"), remote_control_(nh_), psychedelic_mode_(true)
  {
    // Load rosparams
    ros::NodeHandle rpnh(nh_, name_);
    std::size_t error = 0;
    // run mode
    error += !rosparam_shortcuts::get(name_, rpnh, "run_problems", run_problems_);
    error += !rosparam_shortcuts::get(name_, rpnh, "create_spars", create_spars_);
    error += !rosparam_shortcuts::get(name_, rpnh, "eliminate_dense_disjoint_sets", eliminate_dense_disjoint_sets_);
    error += !rosparam_shortcuts::get(name_, rpnh, "check_valid_vertices", check_valid_vertices_);
    error += !rosparam_shortcuts::get(name_, rpnh, "display_disjoint_sets", display_disjoint_sets_);
    error += !rosparam_shortcuts::get(name_, rpnh, "benchmark_performance", benchmark_performance_);

    // run type
    error += !rosparam_shortcuts::get(name_, rpnh, "planning_runs", planning_runs_);
    error += !rosparam_shortcuts::get(name_, rpnh, "experience_planner", experience_planner_);
    error += !rosparam_shortcuts::get(name_, rpnh, "problem_type", problem_type_);
    error += !rosparam_shortcuts::get(name_, rpnh, "problem_id", problem_id_);
    error += !rosparam_shortcuts::get(name_, rpnh, "post_processing_interval", post_processing_interval_);
    error += !rosparam_shortcuts::get(name_, rpnh, "seed_random", seed_random_);
    error += !rosparam_shortcuts::get(name_, rpnh, "image_id", image_id_);
    error += !rosparam_shortcuts::get(name_, rpnh, "dimensions", dimensions_);
    error += !rosparam_shortcuts::get(name_, rpnh, "use_task_planning", use_task_planning_);
    error += !rosparam_shortcuts::get(name_, rpnh, "collision_checking_enabled", collision_checking_enabled_);
    // Debug
    error += !rosparam_shortcuts::get(name_, rpnh, "verbose/verbose", verbose_);
    // Visualize
    error += !rosparam_shortcuts::get(name_, rpnh, "visualize/time_between_plans", visualize_time_between_plans_);
    error += !rosparam_shortcuts::get(name_, rpnh, "visualize/start_goal_states", visualize_start_goal_states_);
    error += !rosparam_shortcuts::get(name_, rpnh, "visualize/database_every_plan", visualize_database_every_plan_);
    error += !rosparam_shortcuts::get(name_, rpnh, "visualize/database_on_load", visualize_database_on_load_);
    error += !rosparam_shortcuts::get(name_, rpnh, "visualize/wait_between_plans", visualize_wait_between_plans_);
    rosparam_shortcuts::shutdownIfError(name_, error);

    // Error check
    BOOST_ASSERT_MSG(dimensions_ >= 2, "Must have at least 2 dimensions");

    // Seed random
    if (seed_random_)
      srand(time(NULL));

    // Set remote_control
    remote_control_.setDisplayWaitingState(boost::bind(&ExperienceDemos::displayWaitingState, this, _1));

    // Load planning
    if (!loadOMPL())
    {
      ROS_ERROR_STREAM_NAMED(name_, "Unable to load planning context");
      exit(-1);
    }

    // Run application
    run();
  }

  ~ExperienceDemos()
  {
    // Free start and goal states
    space_->freeState(ompl_start_);
    space_->freeState(ompl_goal_);
  }

  bool loadOMPL()
  {
    // Construct the state space we are planning in
    space_.reset(new ob::RealVectorStateSpace(dimensions_));

    // Create SimpleSetup
    if (experience_planner_ == "bolt")
    {
      bolt_ = otb::BoltPtr(new otb::Bolt(space_));
      experience_setup_ = bolt_;
      simple_setup_ = experience_setup_;
      planner_name_ = BOLT;
    }
    else if (experience_planner_ == "thunder")
    {
      experience_setup_ = ot::ThunderPtr(new ot::Thunder(space_));
      simple_setup_ = experience_setup_;
      planner_name_ = THUNDER;
    }
    else if (experience_planner_ == "spars2")
    {
      std::cout << "SPARS2 " << std::endl;
      simple_setup_ = og::SimpleSetupPtr(new og::SimpleSetup(space_));
      planner_name_ = SPARS2;
    }

    // Get Space Info
    si_ = simple_setup_->getSpaceInformation();

    if (planner_name_ == SPARS2)
    {
      std::cout << "spars2 2 " << std::endl;
      sparse_two_ = og::SPARStwoPtr(new og::SPARStwo(si_));
      simple_setup_->setPlanner(sparse_two_);
    }

    // Set the database file location
    if (experience_planner_ != "spars2")
    {
      std::string file_path;
      std::string file_name;
      if (planner_name_ == BOLT)
        file_name = "2d_world_" + std::to_string(bolt_->getSparseCriteria()->sparseDeltaFraction_) + "_database";
      else
        file_name = "2d_world_database";
      moveit_ompl::getFilePath(file_path, file_name, "ros/ompl_storage");
      experience_setup_->setFilePath(file_path);  // this is here because its how we do it in moveit_ompl
    }

    // Load visual tool objects
    loadVisualTools();

    // Load collision checker
    loadCollisionChecker(0.0);

    // Run interface for loading rosparam settings into OMPL
    if (planner_name_ == BOLT)
      moveit_ompl::loadOMPLParameters(nh_, name_, bolt_);
    if (planner_name_ == SPARS2)
      loadSPARS2Data();

    // Setup base OMPL stuff
    ROS_INFO_STREAM_NAMED(name_, "Setting up OMPL experience");
    simple_setup_->setup();
    assert(si_->isSetup());

    // Visualize denseDelta and sparseDelta
    visualizeGUI();

    // Create start and goal states
    ompl_start_ = space_->allocState();
    ompl_goal_ = space_->allocState();

    // Set Rviz visuals in OMPL planner
    ot::VisualizerPtr visual;
    if (planner_name_ == BOLT)
    {
      visual = experience_setup_->getVisual();
    }
    else if (planner_name_ == SPARS2)
    {
      visual = ot::VisualizerPtr(new ot::Visualizer());
      sparse_two_->setVisual(visual);
    }
    validity_checker_->setVisual(visual);

    visual->setVizWindow(1, viz1_);
    visual->setVizWindow(2, viz2_);
    visual->setVizWindow(3, viz3_);
    visual->setVizWindow(4, viz4_);
    visual->setVizWindow(5, viz5_);
    visual->setVizWindow(6, viz6_);
    visual->setVizWindow(7, viz6_);  // TODO: this is really hacky

    // Set other hooks
    visual->setWaitForUserFeedback(boost::bind(&ExperienceDemos::waitForNextStep, this, _1));
    visual->setVizVoronoiDiagram(boost::bind(&ExperienceDemos::voronoiDiagram, this));

    return true;
  }

  void loadSPARS2Data()
  {
    using namespace rosparam_shortcuts;
    std::size_t error = 0;

    ros::NodeHandle rpnh(nh_, "sparse_criteria");
    double temp;
    error += !get(name_, rpnh, "sparse_delta_fraction", temp);
    sparse_two_->setSparseDeltaFraction(temp);

    error += !get(name_, rpnh, "dense_delta_fraction", temp);
    sparse_two_->setDenseDeltaFraction(temp);

    // error += !get(name_, rpnh2, "stretch_factor", temp);
    sparse_two_->setStretchFactor(2.82843);  // manually copied from auto calculation for 2D world

    ros::NodeHandle rpnh2(nh_, "sparse_generator");
    std::size_t tempt, total_failures;
    error += !get(name_, rpnh2, "terminate_after_failures", tempt);
    total_failures = tempt;
    error += !get(name_, rpnh2, "fourth_criteria_after_failures", tempt);
    total_failures += tempt;
    std::cout << "total_failures " << total_failures << std::endl;
    sparse_two_->setMaxFailures(total_failures);

    shutdownIfError(name_, error);
  }

  bool loadData()
  {
    // Load database or generate new grid
    ROS_INFO_STREAM_NAMED(name_, "Loading or generating graph");
    if (planner_name_ == BOLT)
    {
      if (!bolt_->load())
      {
        ROS_INFO_STREAM_NAMED(name_, "Unable to load sparse graph from file");
        return false;
      }
    }
    return false;
  }

  void run()
  {
    // deleteAllMarkers();

    // Benchmark performance
    if (benchmark_performance_ && planner_name_ == BOLT)
    {
      bolt_->benchmarkVisualizeSampling();
    }

    // Load from file or generate graph
    if (!loadData())
    {
      // Create SPARs graph
      switch (planner_name_)
      {
        case BOLT:
          if (create_spars_)
          {
            bolt_->getSparseGenerator()->createSPARS();
          }
          else
            ROS_WARN_STREAM_NAMED(name_, "Creating sparse graph disabled, but no file loaded");
          break;
        case THUNDER:
          break;
        case SPARS2:
        {
          ompl::base::PlannerTerminationCondition ptc = ompl::base::plannerNonTerminatingCondition();
          ROS_INFO_STREAM_NAMED(name_, "Constructing SPARS2 roadmap");
          bool stopOnMaxFail = true;
          sparse_two_->constructRoadmap(ptc, stopOnMaxFail);

          // temp
          std::cout << "done with roadmap " << std::endl;
          exit(0);
        }
        break;
      }
    }

    // Display disconnected components
    if (display_disjoint_sets_ && planner_name_ == BOLT)
    {
      std::cout << std::endl;
      ROS_INFO_STREAM_NAMED(name_, "Displaying disjoint sets ----------- ");
      otb::SparseDisjointSetsMap disjointSets;
      bolt_->getSparseGraph()->getDisjointSets(disjointSets);
      bolt_->getSparseGraph()->printDisjointSets(disjointSets);
      bolt_->getSparseGraph()->visualizeDisjointSets(disjointSets);
    }

    // Remove verticies that are somehow in collision
    // if (check_valid_vertices_ && planner_name_ == BOLT)
    // {
    //   bolt_->getSparseGraph()->removeInvalidVertices();
    //   bolt_->getSparseGraph()->saveIfChanged();
    // }

    // Repair missing coverage in the dense graph
    // if (eliminate_dense_disjoint_sets_ && planner_name_ == BOLT)
    // {
    //  bolt_->getSparseGraph()->getDiscretizer()->eliminateDisjointSets();
    // }

    // Run the demo
    if (!run_problems_)
      ROS_INFO_STREAM("Solving requested to be skipped by config file");
    else
    {
      runProblems();
    }
    // testConnectionToGraphOfRandStates();

    if (planner_name_ == BOLT)
    {
      bolt_->saveIfChanged();
    }
  }

  /** \brief Plan repeatidly */
  bool runProblems()
  {
    // Logging
    std::string file_path;
    moveit_ompl::getFilePath(file_path, "bolt_2d_world_logging.csv", "ros/ompl_storage");

    std::ofstream logging_file;                           // open to append
    logging_file.open(file_path.c_str(), std::ios::out);  // no append | std::ios::app);

    // Run the demo the desired number of times
    for (std::size_t run_id = 0; run_id < planning_runs_; ++run_id)
    {
      // Check if user wants to shutdown
      if (!ros::ok())
        break;

      std::cout << std::endl;
      std::cout << "------------------------------------------------------------------------" << std::endl;
      ROS_INFO_STREAM_NAMED("plan", "Planning " << run_id + 1 << " out of " << planning_runs_);
      std::cout << "------------------------------------------------------------------------" << std::endl;

      // Create start and goal space
      chooseStartGoal(ompl_start_, ompl_goal_);

      // Visualize
      if (visualize_start_goal_states_)
        visualizeStartGoal();

      // Do one plan
      plan();

      // Console display
      if (experience_setup_)
        experience_setup_->printLogs();

      // Logging
      if (experience_setup_)
        experience_setup_->saveDataLog(logging_file);
      logging_file.flush();

      // Regenerate Sparse Graph
      if (post_processing_ && run_id % post_processing_interval_ == 0 && run_id > 0)  // every x runs
      {
        ROS_INFO_STREAM_NAMED(name_, "Performing post processing every " << post_processing_interval_ << " intervals");
        experience_setup_->doPostProcessing();
      }

      if (visualize_wait_between_plans_)
        waitForNextStep("run next problem");
      else  // Main pause between planning instances - allows user to analyze
        ros::Duration(visualize_time_between_plans_).sleep();

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

  bool plan()
  {
    // Setup -----------------------------------------------------------

    // Clear all planning data. This only includes data generated by motion plan computation.
    // Planner settings, start & goal states are not affected.
    simple_setup_->clear();

    // Set the start and goal states
    simple_setup_->setStartAndGoalStates(ompl_start_, ompl_goal_);

    // Cartesian -----------------------------------------------------------

    // Optionally create cartesian path
    if (use_task_planning_)
    {
      generateRandCartesianPath();
    }

    // Solve -----------------------------------------------------------

    // Create the termination condition
    double seconds = 600;
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(seconds, 0.1);

    // Benchmark runtime
    ros::Time start_time = ros::Time::now();

    // Attempt to solve the problem within x seconds of planning time
    ob::PlannerStatus solved = simple_setup_->solve(ptc);

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
    og::PathGeometric path = simple_setup_->getSolutionPath();

    // Check/test the solution for errors
    if (use_task_planning_)
    {
      bolt_->getTaskGraph()->checkTaskPathSolution(path, ompl_start_, ompl_goal_);
    }

    // Visualize the doneness
    std::cout << std::endl;

    return true;
  }

  /**
   * \brief Load cost map from file
   * \param file path
   * \param how much of the peaks of the mountains are considered obstacles
   */
  void loadCollisionChecker(double max_cost_threshold_percent = 0.4)
  {
    // Get image path based on package name
    std::string image_path = ros::package::getPath("bolt_2d");

    if (image_path.empty())
    {
      ROS_ERROR("Unable to get OMPL Visual Tools package path ");
      return;
    }
    //   image_num = ompl_visual_tools::OmplVisualTools::iRand(0, 4);

    switch (image_id_)
    {
      case 0:
        image_path.append("/resources/hard0.ppm");
        break;
      case 1:
        image_path.append("/resources/hard1.ppm");
        break;
      case 2:
        image_path.append("/resources/hard2.ppm");
        break;
      case 3:
        image_path.append("/resources/hard3.ppm");
        break;
      case 4:
        image_path.append("/resources/hard4.ppm");
        break;
      case 5:
        image_path.append("/resources/blank.ppm");
        break;
      case 6:
        image_path.append("/resources/sparse.ppm");
        break;
      case 7:
        image_path.append("/resources/narrow.ppm");
        break;
      default:
        ROS_ERROR_STREAM_NAMED("main", "Random has no case " << image_id_);
        break;
    }

    // Load the cost map
    cost_map_.reset(new ompl::base::CostMap2DOptimizationObjective(si_));
    cost_map_->max_cost_threshold_percent_ = max_cost_threshold_percent;
    cost_map_->loadImage(image_path);

    // Set the bounds for the R^2
    ob::RealVectorBounds bounds(dimensions_);
    bounds.setLow(0);                             // both dimensions start at 0
    bounds.setHigh(0, cost_map_->image_->x - 1);  // allow for non-square images
    bounds.setHigh(1, cost_map_->image_->y - 1);  // allow for non-square images
    if (dimensions_ == 3)
    {
      bounds.setHigh(2, cost_map_->image_->y - 1);  // third dimension is now possible
      // bounds.setHigh(2, 2);                         // third dimension has three steps
    }
    space_->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    space_->setup();

    // Set state validity checking for this space
    validity_checker_.reset(new ob::ValidityChecker2D(si_, cost_map_->cost_, cost_map_->max_cost_threshold_));
    validity_checker_->setCheckingEnabled(collision_checking_enabled_);

    simple_setup_->setStateValidityChecker(validity_checker_);

    // The interval in which obstacles are checked for between states
    // seems that it default to 0.01 but doesn't do a good job at that level
    si_->setStateValidityCheckingResolution(0.005);

    // Setup the optimization objective to use the 2d cost map
    simple_setup_->setOptimizationObjective(cost_map_);

    // Pass cost to visualizer
    viz_bg_->setCostMap(cost_map_->cost_);

    // Align the text with the map
    text_pose_.position.x = cost_map_->cost_->size1() / 2.0;
    text_pose_.position.y = cost_map_->cost_->size1() / -20.0 * 0.9;  // scale to offset from base layer
    text_pose_.position.z = cost_map_->cost_->size1() / 10.0;
    sub_text_pose_ = text_pose_;

    sub_text_pose_.position.x -= 10;  // move right
    sub_text_pose_.position.y -= 9;   // move up

    publishCostMapImage();
  }

  /**
   * \brief Clear all markers displayed in Rviz
   */
  void deleteAllMarkers(bool clearDatabase = true)
  {
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

  void loadVisualTools()
  {
    using namespace ompl_visual_tools;

    const std::size_t NUM_VISUALS = 6;
    for (std::size_t i = 1; i <= NUM_VISUALS; ++i)
    {
      rviz_visual_tools::RvizVisualToolsPtr rviz_visual =
          rviz_visual_tools::RvizVisualToolsPtr(new rviz_visual_tools::RvizVisualTools(
              "/world_visual" + std::to_string(i), "/ompl_visual" + std::to_string(i)));
      rviz_visual->loadMarkerPub();
      rviz_visual->enableBatchPublishing();
      ros::spinOnce();

      ROSVizWindowPtr viz = ROSVizWindowPtr(new ROSVizWindow(rviz_visual, si_));
      viz->getVisualTools()->setGlobalScale(global_scale_);

      // Calibrate the color scale for visualization
      const bool invert_colors = true;
      viz->setMinMaxEdgeCost(0, 110, invert_colors);
      viz->setMinMaxEdgeRadius(0.1, 0.6);
      viz->setMinMaxStateRadius(0.2, 1.4);

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
    }

    // Load background visualizer
    rviz_visual_tools::RvizVisualToolsPtr rviz_visual =
        rviz_visual_tools::RvizVisualToolsPtr(new rviz_visual_tools::RvizVisualTools("/world", "/ompl_background"));
    rviz_visual->loadMarkerPub();
    rviz_visual->enableBatchPublishing();
    ros::spinOnce();

    viz_bg_.reset(new ROSVizWindow(rviz_visual, si_));
    viz_bg_->getVisualTools()->setGlobalScale(global_scale_);
    viz_bg_->deleteAllMarkers();

    // Clear Rviz
    deleteAllMarkers();
  }

  void visualizeStartGoal()
  {
    // // Increase the height of zero-based start states by a little
    // ob::ScopedState<> start(space_);
    // space_->copyState(start.get(), ompl_start_);
    // if (dimensions_ > 2)
    //   start[2] = 0.05;  // adjust the z dimension for visualization

    // ob::ScopedState<> goal(space_);
    // space_->copyState(goal.get(), ompl_goal_);
    // if (dimensions_ > 2)
    // {
    //   if (goal[2] < std::numeric_limits<double>::epsilon())
    //     goal[2] = 0.05;  // adjust the z dimension for visualization
    // }

    // Show start and goal
    double start_goal_size = 1.2;

    // Astar
    viz4_->publishState(ompl_start_, rvt::GREEN, start_goal_size, "plan_start_goal");
    viz4_->publishState(ompl_goal_, rvt::RED, start_goal_size, "plan_start_goal");
    viz4_->trigger();

    // Raw
    viz5_->publishState(ompl_start_, rvt::GREEN, start_goal_size, "plan_start_goal");
    viz5_->publishState(ompl_goal_, rvt::RED, start_goal_size, "plan_start_goal");
    viz5_->trigger();

    // Final
    viz6_->publishState(ompl_start_, rvt::GREEN, start_goal_size, "plan_start_goal");
    viz6_->publishState(ompl_goal_, rvt::RED, start_goal_size, "plan_start_goal");
    viz6_->trigger();
  }

  void displayWaitingState(bool waiting)
  {
    if (waiting)
      publishViewFinderFrame(rvt::LARGE);
    else
      publishViewFinderFrame(rvt::MEDIUM);

    viz_bg_->trigger();
  }

  void waitForNextStep(const std::string &msg)
  {
    remote_control_.waitForNextStep(msg);
  }

  /** \brief Show 2d world costs */
  void publishCostMapImage()
  {
    ROS_INFO_STREAM_NAMED("publishCostMapImage", "Publishing cost map image...");
    const bool use_labels = false;

    viz_bg_->getVisualTools()->setBaseFrame("world_visual1");
    if (use_labels)
      viz_bg_->getVisualTools()->publishText(text_pose_, std::string("1 Sparse Graph"), rvt::BLACK, rvt::MEDIUM, false);
    viz_bg_->publishCostMap(cost_map_->image_, false);
    viz_bg_->getVisualTools()->setBaseFrame("world_visual2");
    if (use_labels)
      viz_bg_->getVisualTools()->publishText(text_pose_, std::string("2 "), rvt::BLACK, rvt::MEDIUM, false);
    viz_bg_->publishCostMap(cost_map_->image_, false);
    std::string message =
        "Nodes - black: coverage, orange: connectivity, pink: interface, blue: quality, green: discretized\n"
        "Edges - green: connectivity, yellow: interface, red: quality";
    if (use_labels)
      viz_bg_->getVisualTools()->publishText(sub_text_pose_, message, rvt::BLACK, rvt::SMALL, false);

    viz_bg_->getVisualTools()->setBaseFrame("world_visual3");
    if (use_labels)
      viz_bg_->getVisualTools()->publishText(text_pose_, std::string("3 "), rvt::BLACK, rvt::MEDIUM, false);
    viz_bg_->publishCostMap(cost_map_->image_, false);

    viz_bg_->getVisualTools()->setBaseFrame("world_visual4");
    if (use_labels)
      viz_bg_->getVisualTools()->publishText(text_pose_, std::string("4 AStar"), rvt::BLACK, rvt::MEDIUM, false);
    viz_bg_->publishCostMap(cost_map_->image_, false);

    viz_bg_->getVisualTools()->setBaseFrame("world_visual5");
    if (use_labels)
      viz_bg_->getVisualTools()->publishText(text_pose_, std::string("5 Raw Solution"), rvt::BLACK, rvt::MEDIUM, false);
    viz_bg_->publishCostMap(cost_map_->image_, false);

    viz_bg_->getVisualTools()->setBaseFrame("world_visual6");
    if (use_labels)
      viz_bg_->getVisualTools()->publishText(text_pose_, std::string("6 Solution"), rvt::BLACK, rvt::MEDIUM, false);
    viz_bg_->publishCostMap(cost_map_->image_, false);

    // Publish all
    viz_bg_->trigger();

    ros::Duration(0.001).sleep();
    ros::spinOnce();

    // Show outlines
    displayWaitingState(false);
  }

  void publishViewFinderFrame(const rvt::scales &scale)
  {
    // Cached pose for where to put the outlineing visual frame
    Eigen::Affine3d frame_pose;

    double margin = 0.0;  // rvt::XSMALL
    if (scale == rvt::MEDIUM)
      margin = 1.0;

    // Create frame
    frame_pose = Eigen::Affine3d::Identity();
    frame_pose.translation().x() = cost_map_->image_->x / 2.0 - 1;  // move frame slightly right for frame thickness
    frame_pose.translation().y() = cost_map_->image_->y / 2.0;

    double width = cost_map_->image_->x + 2 * margin;
    double height = cost_map_->image_->y + 2 * margin;
    for (std::size_t i = 1; i < 7; ++i)
    {
      viz_bg_->getVisualTools()->setBaseFrame("world_visual" + std::to_string(i));
      viz_bg_->getVisualTools()->publishWireframeRectangle(frame_pose, width, height, rvt::BLACK, scale, i);
    }
  }

  bool save()
  {
    return experience_setup_->saveIfChanged();
  }

  void chooseStartGoal(ob::State *start, ob::State *goal, bool verbose = true)
  {
    switch (problem_type_)
    {
      case 0:  // random
      {
        findRandValidState(start);
        findRandValidState(goal);

        // Debug
        if (verbose)
        {
          std::cout << "Random Problem " << std::setprecision(5) << std::endl;
          std::cout << "  Start: ";
          si_->printState(start, std::cout);
          std::cout << "  Goal:  ";
          si_->printState(goal, std::cout);
        }
      }
      break;
      case 1:  // static
      {
        ob::RealVectorStateSpace::StateType *real_start = static_cast<ob::RealVectorStateSpace::StateType *>(start);
        ob::RealVectorStateSpace::StateType *real_goal = static_cast<ob::RealVectorStateSpace::StateType *>(goal);
        getStaticStartGoal(problem_id_, real_start, real_goal);
      }
      break;
      case 2:  // Randomly sample around two states
      {
        ROS_INFO_STREAM_NAMED("experience_setup", "Sampling start and goal around two center points");

        // Create new states to be the 'near' states
        ob::ScopedState<> near_start(space_);
        ob::ScopedState<> near_goal(space_);
        ob::RealVectorStateSpace::StateType *real_near_start =
            static_cast<ob::RealVectorStateSpace::StateType *>(near_start.get());
        ob::RealVectorStateSpace::StateType *real_near_goal =
            static_cast<ob::RealVectorStateSpace::StateType *>(near_goal.get());
        getStaticStartGoal(problem_id_, real_near_start, real_near_goal);

        // Check these hard coded values against varying image sizes
        if (!space_->satisfiesBounds(real_near_start) || !space_->satisfiesBounds(real_near_goal))
        {
          ROS_ERROR_STREAM_NAMED("chooseStartGoal:", "State does not satisfy bounds");

          // Debug
          if (verbose)
          {
            std::cout << "Start: " << std::setprecision(5);
            si_->printState(near_start.get(), std::cout);
            std::cout << "Goal:  ";
            si_->printState(near_goal.get(), std::cout);
          }

          exit(-1);
          return;
        }

        // Choose the distance to sample around
        double maxExtent = si_->getMaximumExtent();
        double distance = maxExtent * 0.1;
        ROS_INFO_STREAM_NAMED("experience_setup", "Distance is " << distance << " from max extent " << maxExtent);

        // Sample near the target states
        findRandValidState(start, real_near_start, distance);
        findRandValidState(goal, real_near_goal, distance);

        // Debug
        if (verbose)
        {
          std::cout << "Start: " << std::setprecision(5);
          si_->printState(start, std::cout);
          std::cout << "Goal: ";
          si_->printState(goal, std::cout);
        }
        // Show the sample regions
        if (false)
        {
          viz5_->publishSampleRegion(near_start, distance);
          viz5_->publishSampleRegion(near_goal, distance);
        }
      }
      break;
      default:
        ROS_ERROR_STREAM_NAMED(name_, "Invalid problem type");
    }

    // Set task dimension if necessary
    if (use_task_planning_)
    {
      space_->setLevel(goal, 2);  // set goal on 3rd (last) level
    }
  }

  void getStaticStartGoal(int problem_id, ob::RealVectorStateSpace::StateType *real_start,
                          ob::RealVectorStateSpace::StateType *real_goal)
  {
    switch (problem_id)
    {
      case 0:
        // Along right side
        real_start->values[0] = 3;
        real_start->values[1] = 4;
        real_goal->values[0] = 3;
        real_goal->values[1] = 45;
        break;
      case 1:
        // Along bottom
        real_start->values[0] = 5;
        real_start->values[1] = 47.7005;
        real_goal->values[0] = 40;
        real_goal->values[1] = 47.7005;
        break;
      case 2:
        // Bad state - does not connect to graph
        real_start->values[0] = 29;
        real_start->values[1] = 13;
        real_goal->values[0] = 2;
        real_goal->values[1] = 9;
        break;
      case 3:
        // Middle to slightly below and left
        real_start->values[0] = 17.2002;
        real_start->values[1] = 23.6594;
        real_goal->values[0] = 26.9576;
        real_goal->values[1] = 39.8155;
        break;
      case 4:
        // Middle to slightly below and left
        real_start->values[0] = 15.2002;
        real_start->values[1] = 23.6594;
        real_goal->values[0] = 26.9576;
        real_goal->values[1] = 41.8155;
        break;
      case 5:
        // Along bottom
        real_start->values[0] = 5;   // x - the lower the more right
        real_start->values[1] = 40;  // y - the higher the more bottom
        real_goal->values[0] = 45;
        real_goal->values[1] = 45;
        break;
      case 6:
        // From middle corner (invalid) to top right
        real_start->values[0] = 28.97358;  // x - the lower the more right
        real_start->values[1] = 17.17777;  // y - the higher the more bottom
        real_goal->values[0] = 1.21452;
        real_goal->values[1] = 7.77564;
        break;
      default:
        OMPL_ERROR("Invalid problem_id %u", problem_id);
    }

    // Set task
    real_start->values[2] = 0;
    real_goal->values[2] = 0;
  }

  void findRandValidState(ob::State *state)
  {
    // Create sampler
    ob::StateSamplerPtr sampler = si_->allocStateSampler();

    std::size_t rounds = 0;
    while (rounds < 100)
    {
      sampler->sampleUniform(state);

      // Check if the sampled points are valid
      if (si_->isValid(state))
      {
        return;
      }
      ++rounds;
    }
    ROS_ERROR_STREAM_NAMED("findRandValidState", "Unable to find valid start/goal state after " << rounds << " rounds");
  }

  void findRandValidState(ob::State *state, const ob::State *near, const double distance)
  {
    // Create sampler
    ob::StateSamplerPtr sampler = si_->allocStateSampler();

    while (true)
    {
      sampler->sampleUniformNear(state, near, distance);  // samples (near + distance, near - distance)

      // Check if the sampled points are valid
      if (si_->isValid(state))
      {
        return;
      }
      // else
      // ROS_INFO_STREAM_NAMED("experience_setup", "Searching for valid start/goal state");
    }
  }

  /** \brief Create example cartesian paths */
  void generateRandCartesianPath()
  {
    // First cleanup previous cartesian paths
    // bolt_->getSparseGraph()->cleanupTemporaryVerticies();

    ROS_INFO_STREAM_NAMED(name_, "Adding cart paths");
    ob::State *start = space_->allocState();
    ob::State *goal = space_->allocState();

    // Find valid motion across space
    std::size_t count = 0;
    while (true)
    {
      // Create random start and goal space
      findRandValidState(start);
      findRandValidState(goal);

      if (si_->checkMotion(start, goal))
      {
        break;
      }
      if (count++ > 100)
      {
        ROS_ERROR_STREAM_NAMED(name_, "Unable to find random valid cartesian path through space");
        exit(-1);
      }
    }

    // Set the mid layer level
    // space_->setLevel(start, 1);
    // space_->setLevel(goal, 1);

    // assert(space_->getLevel(start) == 1);
    // assert(space_->getLevel(goal) == 1);

    // Create OMPL path
    std::vector<ompl::base::State *> ompl_path;
    ompl_path.push_back(start);
    ompl_path.push_back(goal);

    // Insert into graph
    std::cout << "adding path --------------------- " << std::endl;
    std::size_t indent = 0;
    if (!bolt_->getTaskGraph()->addCartPath(ompl_path, indent))
    {
      ROS_ERROR_STREAM_NAMED(name_, "Unable to add cartesian path");
      exit(-1);
    }

    // assert(space_->getLevel(start) == 1);
    // assert(space_->getLevel(goal) == 1);

    // // Run check that all states are the correct type
    // ROS_INFO_STREAM_NAMED(name_, "Checking state types for all vertices");
    // bolt_->getSparseGraph()->checkStateType();
  }

  /** \brief Allow access to thunder framework */
  ot::ExperienceSetupPtr getExperienceSetup()
  {
    return experience_setup_;
  }

  void voronoiDiagram()
  {
    // ROS_INFO_STREAM_NAMED(name_, "Publishing voronoi diagram");

    ob::RealVectorBounds bounds = space_->as<ob::RealVectorStateSpace>()->getBounds();

    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    marker.ns = "voronoi_diagram";

    // Set the marker type.
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = -0.25;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Create temporary state
    temp_state_ = si_->allocState();
    temp_values_.resize(3);

    double discretization = 0.5;
    std::size_t xsize = (bounds.high[0] - bounds.low[0]) * discretization;
    std::size_t ysize = (bounds.high[1] - bounds.low[1]) * discretization;
    std::size_t num_points = xsize * ysize * 6;
    marker.points.reserve(num_points);
    marker.colors.reserve(num_points);
    xyToVertex_.clear();
    vertex_reuse_ = 0;

    for (double x = bounds.low[0]; x < bounds.high[0]; x += discretization)
    {
      for (double y = bounds.low[1]; y < bounds.high[1]; y += discretization)
      {
        // Make right and down triangle
        // Check that we are not on the far right or bottom
        if (x + discretization < bounds.high[0] && y + discretization < bounds.high[1])
        {
          publishTriangle(x, y, marker);
          publishTriangle(x + discretization, y, marker);
          publishTriangle(x, y + discretization, marker);
        }

        // Make back and down triangle
        // Check that we are not on the far left or bottom
        if (x - discretization >= 0 && y + discretization < bounds.high[1])
        {
          publishTriangle(x, y, marker);
          publishTriangle(x, y + discretization, marker);
          publishTriangle(x - discretization, y + discretization, marker);
        }
      }
    }

    // Publish to window 2
    // marker.header.frame_id = viz2_->getVisualTools()->getBaseFrame();
    // marker.id = 2; // viz2
    // viz_bg_->getVisualTools()->publishMarker(marker);
    // viz_bg_->trigger();
    // usleep(0.01*1000000);

    // Publish to window 3
    marker.header.frame_id = viz3_->getVisualTools()->getBaseFrame();
    marker.id = 3;
    viz_bg_->getVisualTools()->publishMarker(marker);
    viz_bg_->trigger();

    // Publish to window 5
    marker.header.frame_id = viz5_->getVisualTools()->getBaseFrame();
    marker.id = 5;
    viz_bg_->getVisualTools()->publishMarker(marker);
    viz_bg_->trigger();

    // Cleanup
    si_->freeState(temp_state_);
  }

  void publishTriangle(double x, double y, visualization_msgs::Marker &marker)
  {
    temp_point_.x = x;
    temp_point_.y = y;
    temp_point_.z = 0;
    marker.points.push_back(temp_point_);

    // Get vertex rep --------------------------------------------
    otb::SparseVertex rep;
    std::pair<double, double> coordinates(x, y);
    // Search map
    if (xyToVertex_.find(coordinates) == xyToVertex_.end())
    {
      // Create temporary state
      temp_values_[0] = x;
      temp_values_[1] = y;
      temp_values_[2] = 0.0;
      si_->getStateSpace()->populateState(temp_state_, temp_values_);

      // Find state's representative
      rep = bolt_->getSparseGraph()->getSparseRepresentative(temp_state_);

      xyToVertex_[coordinates] = rep;
    }
    else
    {
      vertex_reuse_++;
      rep = xyToVertex_[coordinates];
    }

    // Search map
    if (vertexToColor_.find(rep) == vertexToColor_.end())
      vertexToColor_[rep] = viz_bg_->getVisualTools()->createRandColor();

    // Assign color
    marker.colors.push_back(vertexToColor_[rep]);
  }

  void visualizeGUI()
  {
    return;  // TODO fix this

    geometry_msgs::Pose demo_pose;
    demo_pose.position.x = cost_map_->cost_->size1() * 0.95;  // move to far left
    demo_pose.position.y = -10;                               //-12;
    demo_pose.position.z = 0;
    demo_pose.orientation.x = 0;
    demo_pose.orientation.y = 0;
    demo_pose.orientation.z = 0;
    demo_pose.orientation.w = 1;
    viz_bg_->getVisualTools()->setBaseFrame("world_visual1");
    viz_bg_->getVisualTools()->setGlobalScale(1);  // temporary change the global scale
    viz_bg_->getVisualTools()->publishSphere(demo_pose, rvt::TRANSLUCENT, bolt_->getSparseCriteria()->getDenseDelta(),
                                             "denseDelta");
    viz_bg_->getVisualTools()->publishSphere(demo_pose, rvt::TRANSLUCENT_LIGHT,
                                             bolt_->getSparseCriteria()->getSparseDelta(), "sparseDelta");
    viz_bg_->trigger();

    // Visualize name of algorithm
    demo_pose.position.y = -10.0;
    demo_pose.position.x = cost_map_->cost_->size1() * 0.6;    // move to left
    viz_bg_->getVisualTools()->setGlobalScale(global_scale_);  // change back to normal value
    viz_bg_->getVisualTools()->publishText(demo_pose, "BOLT", rvt::BLACK, rvt::XXXLARGE, false);
    viz_bg_->trigger();
  }

private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Name of this class
  std::string name_ = "bolt_2d";

  // Recieve input from Rviz
  moveit_ompl::RemoteControl remote_control_;

  // Save the experience setup until the program ends so that the planner data is not lost
  ot::ExperienceSetupPtr experience_setup_;
  og::SimpleSetupPtr simple_setup_;
  otb::BoltPtr bolt_;
  og::SPARStwoPtr sparse_two_;

  // Configuration space
  ob::StateSpacePtr space_;
  ob::SpaceInformationPtr si_;
  ob::ValidityChecker2DPtr validity_checker_;

  // Cost in 2D
  ompl::base::CostMap2DOptimizationObjectivePtr cost_map_;

  // The visual tools for interfacing with Rviz
  ompl_visual_tools::ROSVizWindowPtr viz1_;
  ompl_visual_tools::ROSVizWindowPtr viz2_;
  ompl_visual_tools::ROSVizWindowPtr viz3_;
  ompl_visual_tools::ROSVizWindowPtr viz4_;
  ompl_visual_tools::ROSVizWindowPtr viz5_;
  ompl_visual_tools::ROSVizWindowPtr viz6_;
  ompl_visual_tools::ROSVizWindowPtr viz_bg_;

  // The number of dimensions - always 2 for images
  std::size_t dimensions_ = 2;

  // Robot states
  ob::State *ompl_start_;
  ob::State *ompl_goal_;

  // Modes
  bool run_problems_;
  bool create_spars_;
  bool eliminate_dense_disjoint_sets_;
  bool check_valid_vertices_;
  bool display_disjoint_sets_;
  bool benchmark_performance_;
  bool post_processing_;
  int post_processing_interval_;

  // Type of planner
  std::string experience_planner_;
  PlannerName planner_name_;

  // Operation settings
  std::size_t planning_runs_;
  double discretization_;
  int problem_type_;
  int problem_id_;
  bool seed_random_;  // whether to use predictable random numbers
  bool use_task_planning_;
  int image_id_;  // which image to load
  bool collision_checking_enabled_ = true;

  // Verbosity levels
  bool verbose_;  // Flag for determining amount of debug output to show

  // Display preferences
  bool visualize_start_goal_states_;
  bool visualize_database_every_plan_;
  bool visualize_database_on_load_;
  double visualize_time_between_plans_;
  bool visualize_wait_between_plans_;
  bool psychedelic_mode_;
  double global_scale_ = 50;  // this is the width of the cost_map

  // Average planning time
  double total_duration_ = 0;
  std::size_t total_runs_ = 0;
  std::size_t total_failures_ = 0;

  // For publishing text
  geometry_msgs::Pose text_pose_;
  geometry_msgs::Pose sub_text_pose_;

  // Prevent from publishing experience database except when changes
  std::size_t last_experiences_count_;

  // Fast voronoi
  geometry_msgs::Point temp_point_;
  ompl::base::State *temp_state_;
  std::vector<double> temp_values_;
  std::map<otb::SparseVertex, std_msgs::ColorRGBA> vertexToColor_;
  std::map<std::pair<double, double>, otb::SparseVertex> xyToVertex_;
  std::size_t vertex_reuse_;

};  // end of class

}  // namespace

// *********************************************************************************************************
// Main
// *********************************************************************************************************
int main(int argc, char **argv)
{
  // google::SetVersionString("0.0.1");
  // google::SetUsageMessage("Demonstrate usage of OMPL exerperience databases");
  // google::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "bolt_2d");
  ROS_INFO("OMPL Visual Tools with the Experience Framework ----------------------------------------- ");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Create the demo
  bolt_2d::ExperienceDemos demo;

  // Wait to let anything still being published finish
  ros::spinOnce();
  ros::Duration(0.1).sleep();

  ROS_INFO_STREAM("Shutting down...");

  return 0;
}
