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
#include <geometry_msgs/PointStamped.h>

// Display in Rviz tool
#include <bolt_2d/two_dim_viz_window.h>
#include <ompl/tools/debug/VizWindow.h>
#include <bolt_2d/validity_checker_2d.h>

// OMPL
#include <ompl/tools/lightning/Lightning.h>
#include <ompl/tools/thunder/Thunder.h>
#include <bolt_core/Bolt.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <spars2/SPARS2.h>
#include <ompl/util/PPM.h> // For reading image files
#include <bolt_core/SparseFormula.h>

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
class Bolt2D
{
public:

  /**
   * \brief Constructor
   */
  Bolt2D() : nh_("~"), remote_control_(nh_), psychedelic_mode_(true)
  {
    // Load rosparams
    ros::NodeHandle rpnh(nh_, name_);
    std::size_t error = 0;
    // run mode
    error += !rosparam_shortcuts::get(name_, rpnh, "run_problems", run_problems_);
    error += !rosparam_shortcuts::get(name_, rpnh, "create_spars", create_spars_);
    error += !rosparam_shortcuts::get(name_, rpnh, "create_spars_count", create_spars_count_);
    error += !rosparam_shortcuts::get(name_, rpnh, "load_spars", load_spars_);
    error += !rosparam_shortcuts::get(name_, rpnh, "continue_spars", continue_spars_);
    error += !rosparam_shortcuts::get(name_, rpnh, "eliminate_dense_disjoint_sets", eliminate_dense_disjoint_sets_);
    error += !rosparam_shortcuts::get(name_, rpnh, "check_valid_vertices", check_valid_vertices_);
    error += !rosparam_shortcuts::get(name_, rpnh, "display_disjoint_sets", display_disjoint_sets_);
    error += !rosparam_shortcuts::get(name_, rpnh, "benchmark_performance", benchmark_performance_);
    error += !rosparam_shortcuts::get(name_, rpnh, "sweep_spars_maps", sweep_spars_maps_);
    error += !rosparam_shortcuts::get(name_, rpnh, "sweep_map_start", sweep_map_start_);
    error += !rosparam_shortcuts::get(name_, rpnh, "sweep_map_end", sweep_map_end_);

    // run type
    error += !rosparam_shortcuts::get(name_, rpnh, "planning_runs", planning_runs_);
    error += !rosparam_shortcuts::get(name_, rpnh, "experience_planner", experience_planner_);
    error += !rosparam_shortcuts::get(name_, rpnh, "problem_type", problem_type_);
    error += !rosparam_shortcuts::get(name_, rpnh, "problem_id", problem_id_);
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
    BOLT_ASSERT(dimensions_ >= 2, "Must have at least 2 dimensions");

    // Load potential maps
    maps_.push_back("/resources/hard0.ppm");
    maps_.push_back("/resources/hard1.ppm");
    maps_.push_back("/resources/hard2.ppm");
    maps_.push_back("/resources/hard3.ppm");
    maps_.push_back("/resources/hard4.ppm");
    maps_.push_back("/resources/blank.ppm");
    maps_.push_back("/resources/sparse.ppm");
    maps_.push_back("/resources/narrow.ppm");

    // Get image path based on package name
    package_path_ = ros::package::getPath("bolt_2d");
    if (package_path_.empty())
    {
      ROS_ERROR("Unable to get OMPL Visual Tools package path ");
      exit(-1);
    }

    // Seed random
    if (seed_random_)
      srand(time(NULL));

    // Set remote_control
    remote_control_.setDisplayWaitingState(boost::bind(&Bolt2D::displayWaitingState, this, _1));

    // Load planning
    if (!loadOMPL())
    {
      ROS_ERROR_STREAM_NAMED(name_, "Unable to load planning context");
      exit(-1);
    }

    // Load subscriber from Rviz
    clicked_point_sub_ = nh_.subscribe("/clicked_point", 1000, &Bolt2D::clickedCallback, this);

    // Run application
    run();
  }

  ~Bolt2D()
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
      sparse_two_ = og::SPARS2Ptr(new og::SPARS2(si_));
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
    loadMapByID();

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
    visual->setWaitForUserFeedback(boost::bind(&Bolt2D::waitForNextStep, this, _1));
    visual->setVizVoronoiDiagram(boost::bind(&Bolt2D::voronoiDiagram, this));

    return true;
  }

  void loadSPARS2Data()
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
                  useL2Norm_);

    sparse_two_->setSparseDeltaFraction(sparseDeltaFraction_);
    sparse_two_->setDenseDeltaFraction(denseDeltaFraction_);
    sparse_two_->setStretchFactor(formulas.stretchFactor_); // uses same method as Bolt to calculate

    {
      ros::NodeHandle rpnh(nh_, "sparse_generator");
      std::size_t temp, total_failures;
      error += !get(name_, rpnh, "terminate_after_failures", temp);
      total_failures = temp;
      error += !get(name_, rpnh, "fourth_criteria_after_failures", temp);
      total_failures += temp;
      //total_failures *= 2; // Because SPARS2 keeps failing the optimiality test with "no neighbors found"
      std::cout << " Found total failures: " << total_failures << std::endl;
      sparse_two_->setMaxFailures(total_failures);
    }

    // Clearance
    {
      ros::NodeHandle rpnh(nh_, "sparse_graph");
      double clearance;
      error += !get(name_, rpnh, "obstacle_clearance", clearance);
      sparse_two_->setClearance(clearance);
    }

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
    return true;
  }

  void run()
  {
    std::size_t indent = 0;

    // Benchmark performance
    if (benchmark_performance_ && planner_name_ == BOLT)
    {
      bolt_->getSparseGenerator()->benchmarkVisualizeSampling();
      ROS_INFO_STREAM_NAMED(name_, "Finished benchmarking");
      exit(0);
    }

    // Load from file
    bool loaded = false;
    if (load_spars_)
    {
      loaded = loadData();
    }

    // Sweet maps
    if (sweep_spars_maps_)
    {
      createBoltSweepMaps(indent);
      std::cout << "Done sweeping spars maps " << std::endl;
      exit(0);
    }

    // Create SPARS
    if (create_spars_ && (!loaded || continue_spars_))
    {
      // Create SPARs graph
      switch (planner_name_)
      {
        case BOLT:
          createBolt();
          break;
        case THUNDER:
          break;
        case SPARS2:
          {
            ompl::base::PlannerTerminationCondition ptc = ompl::base::plannerNonTerminatingCondition();
            ROS_INFO_STREAM_NAMED(name_, "Constructing SPARS2 roadmap");
            bool stopOnMaxFail = true;
            sparse_two_->constructRoadmap(ptc, stopOnMaxFail);
          }
          break;
      }
      loaded = true;
    }

    if (!loaded)
    {
      ROS_WARN_STREAM_NAMED(name_, "Creating AND loading sparse graph disabled, no contents in graph");
    }
    // else  // graph loaded fine and will not be modified
    // {
    //   ROS_INFO_STREAM_NAMED(name_, "Checking loaded graph for optimality");
    //   bolt_->getSparseGenerator()->checkGraphOptimality(indent);
    // }

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

  /** \brief Create spars graph once */
  void createBolt()
  {
    // Ensure it is created at least once
    BOLT_ASSERT(create_spars_count_ == 1, "Create spars count is not 1");

    // Create spars
    bolt_->getSparseGenerator()->createSPARS();
  }

  /** \brief Create spars graph repeatidly */
  void createBoltSweepMaps(std::size_t indent)
  {
    BOLT_FUNC(indent, true, "createBoltSweepMaps()");

    std::vector<std::string> trial_maps;
    trial_maps.push_back(std::move("level1"));
    trial_maps.push_back(std::move("level2"));
    trial_maps.push_back(std::move("level3"));
    trial_maps.push_back(std::move("level4"));
    trial_maps.push_back(std::move("level5"));
    BOLT_ASSERT(sweep_map_start_ > 0, "Invalid start");
    BOLT_ASSERT(sweep_map_end_ <= trial_maps.size(), "Invalid end");

    // Config
    const std::size_t TRIALS_PER_MAP = 10;
    bool showGrownLive = false;
    if (planner_name_ == BOLT)
    {
      if (bolt_->getSparseGraph()->visualizeSparseGraph_ &&
          bolt_->getSparseGraph()->visualizeSparseGraphSpeed_ > std::numeric_limits<double>::epsilon())
      {
        showGrownLive = true; // visualization will occur as it grows
      }
    }

    std::vector<std::string> high_level_log;

    // For each map
    //for (std::size_t map_id = 0; map_id < trial_maps.size(); ++map_id)
    for (std::size_t map_id = sweep_map_start_ - 1; map_id < sweep_map_end_; ++map_id)
    {
      // Begin statistics
      std::vector<double> total_edges;
      std::vector<double> total_vertices;
      std::vector<double> total_gen_times;
      std::vector<double> total_avg_plan_times;
      std::vector<double> total_avg_path_quality;

      // For each trial
      for (std::size_t trial_id = 0; trial_id < TRIALS_PER_MAP; ++trial_id)
      {
        // Debug
        BOLT_DEBUG(indent + 2, true, "----------------------------------------------------------------------------");
        BOLT_DEBUG(indent + 2, true, "Creating spars graph, trial " << trial_id+1 << " out of " << TRIALS_PER_MAP << " for map " << trial_maps[map_id] << ".ppm");
        BOLT_DEBUG(indent + 2, true, "----------------------------------------------------------------------------");

        // Clear spars graph
        viz_bg_->deleteAllMarkers();
        viz_bg_->trigger();
        simple_setup_->clear();
        if (showGrownLive)
          deleteAllMarkers(true); // trigger now
        else
          viz1_->deleteAllMarkers(); // delete them, but do not trigger

        // if (trial_id > 0 || map_id > 0)
        //   waitForNextStep("cleared data - memory should be low");

        // Load the map
        std::string image_path = package_path_ + "/resources/trial_set/";
        image_path.append(trial_maps[map_id] + ".ppm");
        if (planner_name_ == BOLT)
          bolt_->getSparseGenerator()->setMapName(trial_maps[map_id]);
        else
          sparse_two_->setMapName(trial_maps[map_id]);

        loadMapAndCollisionChecker(image_path);

        // Create spars
        if (planner_name_ == BOLT)
          bolt_->getSparseGenerator()->createSPARS();
        else
        {
          ompl::base::PlannerTerminationCondition ptc = ompl::base::plannerNonTerminatingCondition();
          ROS_INFO_STREAM_NAMED(name_, "Constructing SPARS2 roadmap");
          bool stopOnMaxFail = true;
          sparse_two_->constructRoadmap(ptc, stopOnMaxFail);

          if (!showGrownLive)
          {
            viz1_->trigger();
            usleep(0.1*1000000);
          }

          if (!sparse_two_->checkGraphOptimality())
            OMPL_ERROR("SPARS2 failed optimality check");
        }

        // Collect stats
        if (planner_name_ == BOLT)
        {
          total_edges.push_back(bolt_->getSparseGraph()->getNumEdges());
          total_vertices.push_back(bolt_->getSparseGraph()->getNumRealVertices());
          total_gen_times.push_back(bolt_->getSparseGenerator()->getLastGraphGenerationTime());
          total_avg_plan_times.push_back(bolt_->getSparseGenerator()->getAvgPlanTime());
          total_avg_path_quality.push_back(bolt_->getSparseGenerator()->getAvgPathQuality());
        }
        else // SPARS2
        {
          total_edges.push_back(sparse_two_->getNumEdges());
          total_vertices.push_back(sparse_two_->milestoneCount());
          total_gen_times.push_back(sparse_two_->getLastGraphGenerationTime());
          total_avg_plan_times.push_back(sparse_two_->getAvgPlanTime());
          total_avg_path_quality.push_back(sparse_two_->getAvgPathQuality());
        }

        if (!ros::ok())
          break;
      } // for each trial

        // Output log
      if (planner_name_ == BOLT)
        bolt_->getSparseGenerator()->dumpLog();
      else
        sparse_two_->dumpLog();
      //waitForNextStep("copy data");

      // Create high level log entry
      std::stringstream line;

      std::pair<double,double> edge_data = getMeanStdDev(total_edges);
      std::pair<double,double> vertex_data = getMeanStdDev(total_vertices);
      std::pair<double,double> gen_time_data = getMeanStdDev(total_gen_times);
      std::pair<double,double> avg_plan_time_data = getMeanStdDev(total_avg_plan_times);
      std::pair<double,double> avg_path_quality_data = getMeanStdDev(total_avg_path_quality);

      std::string planner_name = "Spars2";
      if (planner_name_ == BOLT)
        planner_name = "Bolt";

      // clang-format off
      line << "=SPLIT(\"" << planner_name << ", "
           << trial_maps[map_id] << ", "
           << edge_data.first << ", "
           << edge_data.second << ", "
           << vertex_data.first << ", "
           << vertex_data.second << ", "
           << gen_time_data.first << ", "
           << gen_time_data.second << ", "
           << avg_plan_time_data.first << ", "
           << avg_plan_time_data.second << ", "
           << avg_path_quality_data.first << ", "
           << avg_path_quality_data.second << "\", \",\")";
      // clang-format on

      // Save log
      high_level_log.push_back(line.str());

      // Output to console
      BOLT_CYAN(0, true, "----------------------------------------------------------------------------");
      BOLT_CYAN(0, true, "----------------------------------------------------------------------------");
      BOLT_CYAN(0, true, "High Level Log:");
      BOLT_CYAN(0, true, "----------------------------------------------------------------------------");
      BOLT_CYAN(0, true, "----------------------------------------------------------------------------");
      std::cout << high_level_log.back() << std::endl;
      BOLT_CYAN(0, true, "----------------------------------------------------------------------------");
      BOLT_CYAN(0, true, "----------------------------------------------------------------------------");
      BOLT_CYAN(0, true, "----------------------------------------------------------------------------");
      BOLT_CYAN(0, true, "----------------------------------------------------------------------------");

      if (!ros::ok())
        break;

    } // for each map

    // Done experiment... dump to console
    for (auto line : high_level_log)
      std::cout << line << std::endl;
  }

  std::pair<double,double> getMeanStdDev(const std::vector<double>& data)
  {
    double sum = std::accumulate(data.begin(), data.end(), 0.0);
    double mean = sum / data.size();

    std::vector<double> diff(data.size());
    std::transform(data.begin(), data.end(), diff.begin(), [mean](double x) { return x - mean; });
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / data.size());

    return std::pair<double,double>(mean, stdev);
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

      if (visualize_wait_between_plans_)
        waitForNextStep("run next problem");
      else  // Main pause between planning instances - allows user to analyze
        ros::Duration(visualize_time_between_plans_).sleep();

      // Reset marker if this is not our last run
      if (run_id < planning_runs_ - 1)
        deleteAllMarkers(false);
    }  // for each run

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
   * \brief Load ppm image from file
   */
  void loadMapByID()
  {
    // Get image path based on package name
    std::string image_path = package_path_;

    if (image_id_ > maps_.size() - 1)
    {
      ROS_ERROR_STREAM_NAMED("main", "No image map for ID " << image_id_);
      exit(0);
    }
    image_path.append(maps_[image_id_]);

    loadMapAndCollisionChecker(image_path);
  }

  void loadMapAndCollisionChecker(const std::string& image_path)
  {
    loadImage(image_path);

    // Set the bounds for the R^2
    ob::RealVectorBounds bounds(dimensions_);
    bounds.setLow(0);                             // both dimensions start at 0
    bounds.setHigh(0, ppm_.getWidth() - 1);  // allow for non-square images
    // Allow arbitrary number of dimensions
    for (std::size_t i = 1; i < dimensions_; ++i)
    {
      bounds.setHigh(i, ppm_.getHeight() - 1);  // allow for non-square images
    }
    space_->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // Change the discretization level for collision checking
    //space_->setLongestValidSegmentFraction(0.005);
    //space_->setLongestValidSegmentFraction(0.01); // this is the default value

    space_->setup();

    // Set state validity checking for this space
    validity_checker_.reset(new ob::ValidityChecker2D(si_, &ppm_));
    validity_checker_->setCheckingEnabled(collision_checking_enabled_);
    simple_setup_->setStateValidityChecker(validity_checker_);

    // The interval in which obstacles are checked for between states
    // seems that it defaults to 0.01 but doesn't do a good job at that level
    si_->setStateValidityCheckingResolution(0.005);

    // Align the text with the map
    text_pose_.position.x = ppm_.getHeight() / 2.0;
    text_pose_.position.y = ppm_.getHeight() / -20.0 * 0.9;  // scale to offset from base layer
    text_pose_.position.z = ppm_.getHeight() / 10.0;
    sub_text_pose_ = text_pose_;

    sub_text_pose_.position.x -= 10;  // move right
    sub_text_pose_.position.y -= 9;   // move up

    publishPPMImage();
  }

  void loadImage(std::string image_path)
  {
    bool ok = false;
    try
    {
      ppm_.loadFile(image_path.c_str());
      ok = true;
    }
    catch(ompl::Exception &ex)
    {
      ROS_ERROR_STREAM("Unable to load " << image_path);
      return;
    }

    // Disallow non-square
    if (ppm_.getWidth() != ppm_.getHeight())
    {
      ROS_ERROR("Does not currently support non-square images because of some weird bug. Feel free to fork and fix!");
      return;
    }

    ROS_INFO_STREAM("Map Height: " << ppm_.getHeight() << " Map Width: " << ppm_.getWidth());
  };

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
    using namespace bolt_2d;

    const std::size_t NUM_VISUALS = 6;
    for (std::size_t i = 1; i <= NUM_VISUALS; ++i)
    {
      rviz_visual_tools::RvizVisualToolsPtr rviz_visual =
        rviz_visual_tools::RvizVisualToolsPtr(new rviz_visual_tools::RvizVisualTools(
                                                                                     "/world_visual" + std::to_string(i), "/ompl_visual" + std::to_string(i)));
      rviz_visual->loadMarkerPub();
      rviz_visual->enableBatchPublishing();
      ros::spinOnce();

      TwoDimVizWindowPtr viz = TwoDimVizWindowPtr(new TwoDimVizWindow(rviz_visual, si_));
      viz->getVisualTools()->setGlobalScale(global_scale_);

      // Calibrate the color scale for visualization
      const bool invert_colors = true;
      viz->setMinMaxEdgeCost(0, 110, invert_colors);
      viz->setMinMaxEdgeRadius(0.15, 0.6);
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

    viz_bg_.reset(new TwoDimVizWindow(rviz_visual, si_));
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
      publishViewFinderFrame(rvt::XXLARGE);
    else
      publishViewFinderFrame(rvt::MEDIUM);

    viz_bg_->trigger();
  }

  void waitForNextStep(const std::string &msg)
  {
    remote_control_.waitForNextStep(msg);
  }

  /** \brief Show 2d world image */
  void publishPPMImage()
  {
    ROS_INFO_STREAM_NAMED("publishPPMImage", "Publishing background image...");
    const bool use_labels = false;

    viz_bg_->getVisualTools()->setBaseFrame("world_visual1");
    if (use_labels)
      viz_bg_->getVisualTools()->publishText(text_pose_, std::string("1 Sparse Graph"), rvt::BLACK, rvt::MEDIUM, false);
    viz_bg_->publishPPMImage(ppm_, false);
    viz_bg_->getVisualTools()->setBaseFrame("world_visual2");
    if (use_labels)
      viz_bg_->getVisualTools()->publishText(text_pose_, std::string("2 "), rvt::BLACK, rvt::MEDIUM, false);
    viz_bg_->publishPPMImage(ppm_, false);
    std::string message =
      "Nodes - black: coverage, orange: connectivity, pink: interface, blue: quality, green: discretized\n"
      "Edges - green: connectivity, yellow: interface, red: quality";
    if (use_labels)
      viz_bg_->getVisualTools()->publishText(sub_text_pose_, message, rvt::BLACK, rvt::SMALL, false);

    viz_bg_->getVisualTools()->setBaseFrame("world_visual3");
    if (use_labels)
      viz_bg_->getVisualTools()->publishText(text_pose_, std::string("3 "), rvt::BLACK, rvt::MEDIUM, false);
    viz_bg_->publishPPMImage(ppm_, false);

    viz_bg_->getVisualTools()->setBaseFrame("world_visual4");
    if (use_labels)
      viz_bg_->getVisualTools()->publishText(text_pose_, std::string("4 AStar"), rvt::BLACK, rvt::MEDIUM, false);
    viz_bg_->publishPPMImage(ppm_, false);

    viz_bg_->getVisualTools()->setBaseFrame("world_visual5");
    if (use_labels)
      viz_bg_->getVisualTools()->publishText(text_pose_, std::string("5 Raw Solution"), rvt::BLACK, rvt::MEDIUM, false);
    viz_bg_->publishPPMImage(ppm_, false);

    viz_bg_->getVisualTools()->setBaseFrame("world_visual6");
    if (use_labels)
      viz_bg_->getVisualTools()->publishText(text_pose_, std::string("6 Solution"), rvt::BLACK, rvt::MEDIUM, false);
    viz_bg_->publishPPMImage(ppm_, false);

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
    frame_pose.translation().x() = ppm_.getWidth() / 2.0 - 1;  // move frame slightly right for frame thickness
    frame_pose.translation().y() = ppm_.getHeight() / 2.0;

    double width = ppm_.getWidth() + 2 * margin;
    double height = ppm_.getHeight() + 2 * margin;
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
    xy_to_vertex_.clear();
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
    if (xy_to_vertex_.find(coordinates) == xy_to_vertex_.end())
    {
      // Create temporary state
      temp_values_[0] = x;
      temp_values_[1] = y;
      temp_values_[2] = 0.0;
      si_->getStateSpace()->populateState(temp_state_, temp_values_);

      // Find state's representative
      rep = bolt_->getSparseGraph()->getSparseRepresentative(temp_state_);

      xy_to_vertex_[coordinates] = rep;
    }
    else
    {
      vertex_reuse_++;
      rep = xy_to_vertex_[coordinates];
    }

    // Search map
    if (vertex_to_color_.find(rep) == vertex_to_color_.end())
      vertex_to_color_[rep] = viz_bg_->getVisualTools()->createRandColor();

    // Assign color
    marker.colors.push_back(vertex_to_color_[rep]);
  }

  void visualizeGUI()
  {
    return;  // TODO fix this

    geometry_msgs::Pose demo_pose;
    demo_pose.position.x = ppm_.getHeight() * 0.95;  // move to far left
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
    demo_pose.position.x = ppm_.getHeight() * 0.6;    // move to left
    viz_bg_->getVisualTools()->setGlobalScale(global_scale_);  // change back to normal value
    viz_bg_->getVisualTools()->publishText(demo_pose, "BOLT", rvt::BLACK, rvt::XXXLARGE, false);
    viz_bg_->trigger();
  }

  void clickedCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    //ROS_INFO_STREAM_NAMED(name_, "Clicked point:\n" << msg->point);
    ROS_INFO_STREAM_NAMED(name_, "Clicked point");

    ob::State* temp_state = space_->allocState();
    ob::RealVectorStateSpace::StateType* real_state = static_cast<ob::RealVectorStateSpace::StateType*>(temp_state);
    real_state->values[0] = msg->point.x;
    real_state->values[1] = msg->point.y;
    real_state->values[2] = msg->point.z;

    // Show point
    if (false)
    {
      experience_setup_->getVisual()->viz1()->state(temp_state, ompl::tools::MEDIUM, ompl::tools::BLUE, 0);
      experience_setup_->getVisual()->viz1()->trigger();
      usleep(0.001*1000000);
    }

    // Make SparseCriteria and SparseGraph verbose
    bolt_->getSparseCriteria()->visualizeAttemptedStates_ = true;
    bolt_->getSparseCriteria()->visualizeConnectivity_ = true;
    bolt_->getSparseCriteria()->visualizeQualityCriteria_ = true;
    bolt_->getSparseCriteria()->vCriteria_ = true;
    bolt_->getSparseCriteria()->vQuality_ = true;
    bolt_->getSparseCriteria()->vQualityMaxSpanner_ = true;
    bolt_->getSparseCriteria()->vAddedReason_ = true;
    bolt_->getSparseCriteria()->vRemoveClose_ = true;
    bolt_->getSparseGraph()->visualizeSparseGraphSpeed_ = 0.0001;
    bolt_->getSparseGraph()->visualizeSparseGraph_ = true;

    // Add point to graph
    const std::size_t threadID = 0;
    bool usedState;
    const std::size_t indent = 0;
    bolt_->getSparseGenerator()->addSample(temp_state, threadID, usedState, indent);
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
  og::SPARS2Ptr sparse_two_;

  // Configuration space
  ob::StateSpacePtr space_;
  ob::SpaceInformationPtr si_;
  ob::ValidityChecker2DPtr validity_checker_;

  // The visual tools for interfacing with Rviz
  bolt_2d::TwoDimVizWindowPtr viz1_;
  bolt_2d::TwoDimVizWindowPtr viz2_;
  bolt_2d::TwoDimVizWindowPtr viz3_;
  bolt_2d::TwoDimVizWindowPtr viz4_;
  bolt_2d::TwoDimVizWindowPtr viz5_;
  bolt_2d::TwoDimVizWindowPtr viz6_;
  bolt_2d::TwoDimVizWindowPtr viz_bg_;

  // The number of dimensions - always 2 for images
  std::size_t dimensions_ = 2;

  // Robot states
  ob::State *ompl_start_;
  ob::State *ompl_goal_;

  // Modes
  bool run_problems_;
  bool create_spars_;
  std::size_t create_spars_count_;
  bool load_spars_;
  bool continue_spars_;
  bool eliminate_dense_disjoint_sets_;
  bool check_valid_vertices_;
  bool display_disjoint_sets_;
  bool benchmark_performance_;
  bool sweep_spars_maps_;
  std::size_t sweep_map_start_;
  std::size_t sweep_map_end_;

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
  double global_scale_ = 50;  // this is the width of the ppm image

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
  std::map<otb::SparseVertex, std_msgs::ColorRGBA> vertex_to_color_;
  std::map<std::pair<double, double>, otb::SparseVertex> xy_to_vertex_;
  std::size_t vertex_reuse_;

  // The RGB image data
  ompl::PPM ppm_;
  std::vector<std::string> maps_;
  std::string package_path_;

  // Getting input from Rviz
  ros::Subscriber clicked_point_sub_;

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
  bolt_2d::Bolt2D application;

  // Wait to let anything still being published finish
  ros::spinOnce();
  ros::Duration(0.1).sleep();

  ROS_INFO_STREAM("Shutting down...");

  return 0;
}
