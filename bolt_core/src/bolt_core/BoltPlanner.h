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

/* Author: Dave Coleman */

#ifndef BOLT_CORE_BOLT_PLANNER_
#define BOLT_CORE_BOLT_PLANNER_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <bolt_core/TaskGraph.h>
#include <ompl/tools/debug/Visualizer.h>

// Boost
#include <boost/lambda/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

namespace ompl
{
namespace tools
{
namespace bolt
{
/// @cond IGNORE
/** \brief Forward declaration of ompl::base::BoltPlanner */
OMPL_CLASS_FORWARD(BoltPlanner);
OMPL_CLASS_FORWARD(TaskGraph);
/// @endcond

/** \class ompl::base::BoltPlannerPtr
    \brief A boost shared pointer wrapper for ompl::base::BoltPlanner */

typedef const base::PlannerTerminationCondition Termination;

/**
   @anchor BoltPlanner
   @par Short description
   Bolt is an experience-based planning framework that learns to reduce computation time
   required to solve high-dimensional planning problems in varying environments.
*/

/** \brief The Bolt Framework's Retrieve-Repair component */
class BoltPlanner : public base::Planner
{
public:
  /** \brief Constructor */
  BoltPlanner(const base::SpaceInformationPtr modelSI, const base::SpaceInformationPtr compoundSI,
              const TaskGraphPtr &taskGraph, VisualizerPtr visual);

  virtual ~BoltPlanner(void);

  /** \brief Wrapper function to show good user feedback while smoothing a path */
  bool simplifyNonTaskPath(geometric::PathGeometricPtr path, Termination &ptc, std::size_t indent);

  /** \brief Simplify a multi-modal path for different task levels */
  bool simplifyTaskPath(geometric::PathGeometricPtr compoundPath, Termination &ptc, std::size_t indent);

  /** \brief Main entry function for finding a path plan */
  virtual base::PlannerStatus solve(Termination &ptc);

  /** \brief Solving after converting states to compound state space */
  base::PlannerStatus solve(base::CompoundState *startState, base::CompoundState *goalState, Termination &ptc,
                            std::size_t indent);

  /** \brief Clear memory */
  virtual void clear(void);

  /**
   * \brief Pass a pointer of the database from the bolt framework
   */
  void setExperienceDB(const TaskGraphPtr &taskGraph);

  /** \brief Setup function */
  virtual void setup(void);

  /** \brief Optionally smooth retrieved and repaired paths from database */
  void enableSmoothing(bool enable)
  {
    smoothingEnabled_ = enable;
  }

  /**
   * \brief Search the roadmap for the best path close to the given start and goal states that is valid
   * \param start
   * \param goal
   * \param compoundSolution - the resulting path
   * \return
   */
  bool getPathOffGraph(const base::CompoundState *start, const base::CompoundState *goal,
                       geometric::PathGeometricPtr compoundSolution, Termination &ptc, std::size_t indent);

  /** \brief Clear verticies not on the specified level */
  bool removeVerticesNotOnLevel(std::vector<bolt::TaskVertex> &neighbors, int level);

  /**
   * \brief Convert astar results to correctly ordered path
   * \param vertexPath - in reverse
   * \param start - actual start that is probably not included in new path
   * \param goal - actual goal that is probably not included in new path
   * \param path - returned solution
   * \return true on success
   */
  bool convertVertexPathToStatePath(std::vector<bolt::TaskVertex> &vertexPath, const base::CompoundState *actualStart,
                                    const base::CompoundState *actualGoal, geometric::PathGeometricPtr compoundSolution,
                                    std::size_t indent);

  /**
   * \brief Finds nodes in the graph near state NOTE: note tested for visibility
   * \param state - vertex to find neighbors around
   * \param neighbors - result vector
   * \param requiredLevel - if -1, allows states from all levels, otherwise only returns states from a certain level
   * \return false is no neighbors found
   */
  bool findGraphNeighbors(const base::CompoundState *state, std::vector<bolt::TaskVertex> &neighbors,
                          int requiredLevel = -1, std::size_t indent = 0);

  /** \brief Check if there exists a solution, i.e., there exists a pair of milestones such that the
   *   first is in \e start and the second is in \e goal, and the two milestones are in the same
   *   connected component. If a solution is found, the path is saved.
   * \param debug - whether to show the failure points
   * to conenct
   */
  bool getPathOnGraph(const std::vector<bolt::TaskVertex> &candidateStarts,
                      const std::vector<bolt::TaskVertex> &candidateGoals, const base::CompoundState *actualStart,
                      const base::CompoundState *actualGoal, geometric::PathGeometricPtr compoundSolution,
                      Termination &ptc, bool debug, std::size_t indent);

  /**
   * \brief Repeatidly search through graph for connection then check for collisions then repeat
   * \return true if a valid path is found
   */
  bool onGraphSearch(const bolt::TaskVertex &start, const bolt::TaskVertex &goal,
                     const base::CompoundState *actualStart, const base::CompoundState *actualGoal,
                     geometric::PathGeometricPtr compoundSolution, Termination &ptc, std::size_t indent);

  /** \brief Check recalled path for collision and disable as needed */
  bool lazyCollisionCheck(std::vector<bolt::TaskVertex> &vertexPath, Termination &ptc, std::size_t indent);

  /** \brief Test if the passed in random state can connect to a nearby vertex in the graph */
  bool canConnect(const base::CompoundState *randomState, Termination &ptc, std::size_t indent);

  /** \brief User feedback */
  void visualizeBadEdge(TaskVertex fromVertex, TaskVertex toVertex);
  void visualizeBadEdge(const base::State *from, const base::State *to);

  void addSamples(const base::State *near, std::size_t indent);

  TaskGraphPtr getTaskGraph()
  {
    return taskGraph_;
  }

  geometric::PathGeometricPtr getOrigCompoundSolPath()
  {
    return origCompoundSolPath_;
  }

  geometric::PathGeometricPtr getOrigModelSolPath()
  {
    return origModelSolPath_;
  }

  geometric::PathGeometricPtr getSmoothedCompoundSolPath()
  {
    return smoothedCompoundSolPath_;
  }

  // Same as the planner "solution"
  geometric::PathGeometricPtr getSmoothedModelSolPath()
  {
    return smoothedModelSolPath_;
  }

  std::vector<geometric::PathGeometricPtr> getModelSolSegments()
  {
    return smoothedModelSolSegments_;
  }

  void visualizeRaw(std::size_t indent);
  void visualizeSmoothed(std::size_t indent);

private:
  /** \brief This is included in parent class, but mentioned here. Use modelSI_ instead to reduce confusion   */
  // using Planner::si_;

protected:
  /** \brief Short name of class */
  const std::string name_ = "BoltPlanner";

  /** \brief The database of motions to search through */
  TaskGraphPtr taskGraph_;

  /** \brief The space information for joint states e.g. model_based_state_space */
  base::SpaceInformationPtr modelSI_;

  /** \brief Space information for combined joint states and discrete state */
  base::SpaceInformationPtr compoundSI_;

  /** \brief Class for managing various visualization features */
  VisualizerPtr visual_;

  /** \brief The instance of the path simplifier */
  geometric::PathSimplifierPtr path_simplifier_;

  base::ValidStateSamplerPtr sampler_;

  /** \brief Used by getPathOffGraph */
  std::vector<bolt::TaskVertex> startVertexCandidateNeighbors_;
  std::vector<bolt::TaskVertex> goalVertexCandidateNeighbors_;

  // SOLUTION VERSIONS ----------------------------

  /** \brief Original solution path before smoothing for introspection later - CompoundStateSpace */
  geometric::PathGeometricPtr origCompoundSolPath_;

  /** \brief Original solution path before smoothing for introspection later - ModelBasedStateSpace */
  geometric::PathGeometricPtr origModelSolPath_;

  /** \brief Solution path after smoothing - CompoundStateSpace */
  geometric::PathGeometricPtr smoothedCompoundSolPath_;

  /** \brief Solution path after smoothing, this version is set as Planner solution - ModelBasedStateSpace */
  geometric::PathGeometricPtr smoothedModelSolPath_;

  /** \brief Solution path divided into separate pieces for each discrete mode - ModelBasedStateSpace */
  std::vector<geometric::PathGeometricPtr> smoothedModelSolSegments_;

  std::size_t kNearestNeighbors_ = 60;

public:
  /** \brief Optionally smooth retrieved and repaired paths from database */
  bool smoothingEnabled_ = true;

  /** \brief Output user feedback to console */
  bool verbose_ = false;
  bool vCollisionCheck_ = false;
  bool vSampling_ = true;
  bool vNearestNeighbor_ = false;
  /** \brief Visualize original solution from graph before smoothing */
  bool visualizeRawTrajectory_ = true;

  /** \brief Visualize solution from graph after smoothing */
  bool visualizeSmoothTrajectory_ = true;

  /** \brief Wait after visualization */
  bool visualizeWait_ = false;

  bool visualizeStartGoal_ = false;
  bool visualizeLazyCollisionCheck_ = true;
  bool visualizeEachSolutionStep_ = false;
  bool visualizeStartGoalUnconnected_ = false;
  bool visualizeSampling_ = true;

  int numStartGoalStatesAddedToTask_ = 0;
};
}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // BOLT_CORE_BOLT_PLANNER_
