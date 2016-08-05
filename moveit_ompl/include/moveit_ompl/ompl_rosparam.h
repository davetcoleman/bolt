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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Helper function for loading rosparameter settings into the
           OMPL Bolt algorithm from different applications
*/

#ifndef MOVEIT_OMPL_OMPL_ROSPARAM_H_
#define MOVEIT_OMPL_OMPL_ROSPARAM_H_

// ROS
#include <ros/ros.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// OMPL
#include <ompl/tools/bolt/Bolt.h>
#include <ompl/tools/bolt/SparseCriteria.h>
#include <ompl/tools/bolt/SparseGenerator.h>

// Boost
#include <boost/filesystem.hpp>

namespace moveit_ompl
{
void loadOMPLParameters(ros::NodeHandle nh, const std::string &name, ompl::tools::bolt::BoltPtr bolt)
{
  using namespace rosparam_shortcuts;
  std::size_t error = 0;
  ompl::tools::bolt::SparseGraphPtr sparseGraph = bolt->getSparseGraph();
  ompl::tools::bolt::TaskGraphPtr taskGraph = bolt->getTaskGraph();
  ompl::tools::bolt::SparseCriteriaPtr sparseCriteria = bolt->getSparseCriteria();
  ompl::tools::bolt::SparseGeneratorPtr sparseGenerator = bolt->getSparseGenerator();
  ompl::tools::bolt::BoltPlannerPtr boltPlanner = bolt->getBoltPlanner();
  ompl::tools::bolt::VertexDiscretizerPtr vertexDiscret = sparseGenerator->getVertexDiscretizer();
  //ompl::tools::bolt::DenseCachePtr denseCache = sparseGraph->getDenseCache();

  // Bolt
  {
    ros::NodeHandle rpnh(nh, "bolt");
    error += !get(name, rpnh, "visualize/raw_trajectory", bolt->visualizeRawTrajectory_);
    error += !get(name, rpnh, "visualize/smooth_trajectory", bolt->visualizeSmoothTrajectory_);
    error += !get(name, rpnh, "visualize/robot_trajectory", bolt->visualizeRobotTrajectory_);
  }

  // Vertex Discretizer
  {
    ros::NodeHandle rpnh(nh, "vertex_discretizer");
    error += !get(name, rpnh, "verbose/verbose", vertexDiscret->verbose_);
    error += !get(name, rpnh, "verbose/thread", vertexDiscret->vThread_);
    error += !get(name, rpnh, "visualize/grid_generation", vertexDiscret->visualizeGridGeneration_);
    error += !get(name, rpnh, "visualize/grid_generation_wait", vertexDiscret->visualizeGridGenerationWait_);
  }

  // SparseGraph
  {
    ros::NodeHandle rpnh(nh, "sparse_graph");
    error += !get(name, rpnh, "obstacle_clearance", sparseGraph->obstacleClearance_);
    error += !get(name, rpnh, "save_enabled", sparseGraph->savingEnabled_);
    error += !get(name, rpnh, "super_debug", sparseGraph->superDebug_);
    error += !get(name, rpnh, "verbose/add", sparseGraph->vAdd_);
    error += !get(name, rpnh, "visualize/spars_graph", sparseGraph->visualizeSparseGraph_);
    error += !get(name, rpnh, "visualize/spars_graph_speed", sparseGraph->visualizeSparseGraphSpeed_);
    error += !get(name, rpnh, "visualize/database_vertices", sparseGraph->visualizeDatabaseVertices_);
    error += !get(name, rpnh, "visualize/database_edges", sparseGraph->visualizeDatabaseEdges_);
    error += !get(name, rpnh, "visualize/database_coverage", sparseGraph->visualizeDatabaseCoverage_);
    error += !get(name, rpnh, "visualize/projection", sparseGraph->visualizeProjection_);
    error += !get(name, rpnh, "visualize/graph_after_loading", sparseGraph->visualizeGraphAfterLoading_);
    error += !get(name, rpnh, "visualize/quality_path_simp", sparseGraph->visualizeQualityPathSimp_);
    error += !get(name, rpnh, "visualize/astar", sparseGraph->visualizeAstar_);
    error += !get(name, rpnh, "visualize/astar_speed", sparseGraph->visualizeAstarSpeed_);
    error += !get(name, rpnh, "visualize/voronoi_diagram", sparseGraph->visualizeVoronoiDiagram_);
    error += !get(name, rpnh, "visualize/voronoi_diagram_animated", sparseGraph->visualizeVoronoiDiagramAnimated_);
    shutdownIfError(name, error);
  }

  // SparseCriteria
  {
    ros::NodeHandle rpnh(nh, "sparse_criteria");
    error += !get(name, rpnh, "sparse_delta_fraction", sparseCriteria->sparseDeltaFraction_);
    error += !get(name, rpnh, "dense_delta_fraction", sparseCriteria->denseDeltaFraction_);
    error += !get(name, rpnh, "near_sample_points_multiple", sparseCriteria->nearSamplePointsMultiple_);
    error += !get(name, rpnh, "stretch_factor", sparseCriteria->stretchFactor_);
    error += !get(name, rpnh, "penetration_overlap_fraction", sparseCriteria->penetrationOverlapFraction_);
    error += !get(name, rpnh, "use_l2_norm", sparseCriteria->useL2Norm_);
    error += !get(name, rpnh, "use_edge_improvement_rule", sparseCriteria->useEdgeImprovementRule_);
    error += !get(name, rpnh, "use_check_remove_close_vertices", sparseCriteria->useCheckRemoveCloseVertices_);
    error += !get(name, rpnh, "use_clear_edges_near_vertex", sparseCriteria->useClearEdgesNearVertex_);
    error += !get(name, rpnh, "use_original_smoother", sparseCriteria->useOriginalSmoother_);
    error += !get(name, rpnh, "verbose/criteria", sparseCriteria->vCriteria_);
    error += !get(name, rpnh, "verbose/quality", sparseCriteria->vQuality_);
    error += !get(name, rpnh, "verbose/quality_max_spanner", sparseCriteria->vQualityMaxSpanner_);
    error += !get(name, rpnh, "verbose/remove_close", sparseCriteria->vRemoveClose_);
    error += !get(name, rpnh, "verbose/added_reason", sparseCriteria->vAddedReason_);
    error += !get(name, rpnh, "visualize/attempted_states", sparseCriteria->visualizeAttemptedStates_);
    error += !get(name, rpnh, "visualize/connectivity", sparseCriteria->visualizeConnectivity_);
    error += !get(name, rpnh, "visualize/remove_close_vertices", sparseCriteria->visualizeRemoveCloseVertices_);
    error += !get(name, rpnh, "visualize/quality_criteria", sparseCriteria->visualizeQualityCriteria_);
    error += !get(name, rpnh, "visualize/quality_criteria_close_reps", sparseCriteria->visualizeQualityCriteriaCloseReps_);
    error += !get(name, rpnh, "visualize/quality_criteria_sampler", sparseCriteria->visualizeQualityCriteriaSampler_);
    error += !get(name, rpnh, "visualize/quality_criteria_astar", sparseCriteria->visualizeQualityCriteriaAstar_);
    shutdownIfError(name, error);
  }

  // SparseGenerator
  {
    ros::NodeHandle rpnh(nh, "sparse_generator");
    error += !get(name, rpnh, "terminate_after_failures", sparseGenerator->terminateAfterFailures_);
    error += !get(name, rpnh, "fourth_criteria_after_failures", sparseGenerator->fourthCriteriaAfterFailures_);
    error += !get(name, rpnh, "use_discretized_samples", sparseGenerator->useDiscretizedSamples_);
    error += !get(name, rpnh, "use_random_samples", sparseGenerator->useRandomSamples_);
    error += !get(name, rpnh, "save_interval", sparseGenerator->saveInterval_);

  }

  // TaskGraph
  {
    ros::NodeHandle rpnh(nh, "task_graph");
    error += !get(name, rpnh, "num_neighbors_connect_to_cart", taskGraph->numNeighborsConnectToCart_);
    error += !get(name, rpnh, "verbose/add", taskGraph->vAdd_);
    error += !get(name, rpnh, "verbose/search", taskGraph->vSearch_);
    error += !get(name, rpnh, "verbose/visualize", taskGraph->vVisualize_);
    error += !get(name, rpnh, "verbose/heuristic", taskGraph->vHeuristic_);
    error += !get(name, rpnh, "verbose/clear", taskGraph->vClear_);
    error += !get(name, rpnh, "verbose/generate_task", taskGraph->vGenerateTask_);
    error += !get(name, rpnh, "verbose/verbose", taskGraph->verbose_);
    error += !get(name, rpnh, "visualize/task_graph", taskGraph->visualizeTaskGraph_);
    error += !get(name, rpnh, "visualize/task_graph_speed", taskGraph->visualizeTaskGraphSpeed_);
    error += !get(name, rpnh, "visualize/database_vertices", taskGraph->visualizeDatabaseVertices_);
    error += !get(name, rpnh, "visualize/database_edges", taskGraph->visualizeDatabaseEdges_);
    error += !get(name, rpnh, "visualize/astar", taskGraph->visualizeAstar_);
    error += !get(name, rpnh, "visualize/astar_speed", taskGraph->visualizeAstarSpeed_);
    shutdownIfError(name, error);
  }

  // // Dense Cache
  // {
  //   ros::NodeHandle rpnh(nh, "dense_cache");
  //   error += !get(name, rpnh, "disable_cache", denseCache->disableCache_);
  //   error += !get(name, rpnh, "enable_cache_saving", denseCache->enableCacheSaving_);
  //   error += !get(name, rpnh, "save_every_n_edges", denseCache->saveEveryNEdges_);
  //   shutdownIfError(name, error);
  // }

  // BoltPlanner
  {
    ros::NodeHandle rpnh(nh, "bolt_planner");
    error += !get(name, rpnh, "verbose/verbose", boltPlanner->verbose_);
    shutdownIfError(name, error);
  }
}

  /**
   * \brief Creates a directory names *database_direction* in the user's *home* folder, and inside that creates a file
   *        named *database_name.ompl*
   * \param file_path - result to generate
   * \param file_name - name of file to create
   * \param home_directory - name of folder to save in user directory
   * \return true on success
   */
  bool getFilePath(std::string &file_path, const std::string &file_name, const std::string &home_directory)
  {
    namespace fs = boost::filesystem;
    // Check that the directory exists, if not, create it
    fs::path rootPath;

    //rootPath = fs::path("/home/dave");
    //std::cout << "Over-rode root path to : " << rootPath << std::endl;

    if (!std::string(getenv("HOME")).empty())
      rootPath = fs::path(getenv("HOME"));  // Support Linux/Mac
    else if (!std::string(getenv("HOMEPATH")).empty())
      rootPath = fs::path(getenv("HOMEPATH"));  // Support Windows
    else
    {
      ROS_WARN("Unable to find a home path for this computer");
      rootPath = fs::path("");
    }
    rootPath = rootPath / fs::path(home_directory);

    boost::system::error_code returnedError;
    fs::create_directories(rootPath, returnedError);

    if (returnedError)
    {
      // did not successfully create directories
      ROS_ERROR("Unable to create directory %s", home_directory.c_str());
      return false;
    }

    // directories successfully created, append the group name as the file name
    rootPath = rootPath / fs::path(file_name);
    file_path = rootPath.string();

    return true;
  }

}  // namespace

#endif  // MOVEIT_OMPL_OMPL_ROSPARAM_H_
