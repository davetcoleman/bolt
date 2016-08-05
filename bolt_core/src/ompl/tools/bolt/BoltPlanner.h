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

#ifndef OMPL_BOLT_BOLT_PLANNER_
#define OMPL_BOLT_BOLT_PLANNER_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/tools/bolt/TaskGraph.h>
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
  BoltPlanner(const base::SpaceInformationPtr &si, const TaskGraphPtr &taskGraph, VisualizerPtr visual);

  virtual ~BoltPlanner(void);

  /** \brief Wrapper function to show good user feedback while smoothing a path */
  bool simplifyPath(geometric::PathGeometric &path, Termination &ptc, std::size_t indent);

  /** \brief Simplify a multi-modal path for different task levels */
  bool simplifyTaskPath(geometric::PathGeometric &path, Termination &ptc, std::size_t indent);

  /** \brief Main entry function for finding a path plan */
  virtual base::PlannerStatus solve(Termination &ptc);

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
   * \param geometricSolution - the resulting path
   * \return
   */
  bool getPathOffGraph(const base::State *start, const base::State *goal, geometric::PathGeometric &geometricSolution,
                       Termination &ptc, std::size_t indent);

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
  bool convertVertexPathToStatePath(std::vector<bolt::TaskVertex> &vertexPath, const base::State *actualStart,
                                    const base::State *actualGoal, geometric::PathGeometric &geometricSolution);

  /**
   * \brief Finds nodes in the graph near state NOTE: note tested for visibility
   * \param state - vertex to find neighbors around
   * \param neighbors - result vector
   * \param requiredLevel - if -1, allows states from all levels, otherwise only returns states from a certain level
   * \return false is no neighbors found
   */
  bool findGraphNeighbors(const base::State *state, std::vector<bolt::TaskVertex> &neighbors, int requiredLevel = -1,
                          std::size_t indent = 0);

  /** \brief Check if there exists a solution, i.e., there exists a pair of milestones such that the
   *   first is in \e start and the second is in \e goal, and the two milestones are in the same
   *   connected component. If a solution is found, the path is saved.
   * \param debug - whether to show the failure points
   * \param feedbackStartFailed - if getPathOnGraph returns false, this flag determines if the start or goal node failed
   * to conenct
   */
  bool getPathOnGraph(const std::vector<bolt::TaskVertex> &candidateStarts,
                      const std::vector<bolt::TaskVertex> &candidateGoals, const base::State *actualStart,
                      const base::State *actualGoal, geometric::PathGeometric &geometricSolution, Termination &ptc,
                      bool debug, bool &feedbackStartFailed, std::size_t indent);

  /**
   * \brief Repeatidly search through graph for connection then check for collisions then repeat
   * \return true if a valid path is found
   */
  bool lazyCollisionSearch(const bolt::TaskVertex &start, const bolt::TaskVertex &goal, const base::State *actualStart,
                           const base::State *actualGoal, geometric::PathGeometric &geometricSolution, Termination &ptc,
                           std::size_t indent);

  /** \brief Check recalled path for collision and disable as needed */
  bool lazyCollisionCheck(std::vector<bolt::TaskVertex> &vertexPath, Termination &ptc, std::size_t indent);

  /** \brief Test if the passed in random state can connect to a nearby vertex in the graph */
  bool canConnect(const base::State *randomState, Termination &ptc, std::size_t indent);

  TaskGraphPtr getTaskGraph()
  {
    return taskGraph_;
  }

  std::shared_ptr<geometric::PathGeometric> getOriginalSolutionPath()
  {
    return originalSolutionPath_;
  }

protected:
  /** \brief The database of motions to search through */
  TaskGraphPtr taskGraph_;

  /** \brief Class for managing various visualization features */
  VisualizerPtr visual_;

  /** \brief Save the recalled path before smoothing for introspection later */
  std::shared_ptr<geometric::PathGeometric> originalSolutionPath_;

  /** \brief The instance of the path simplifier */
  geometric::PathSimplifierPtr path_simplifier_;

  /** \brief Optionally smooth retrieved and repaired paths from database */
  bool smoothingEnabled_ = true;

  /** \brief Used by getPathOffGraph */
  std::vector<bolt::TaskVertex> startVertexCandidateNeighbors_;
  std::vector<bolt::TaskVertex> goalVertexCandidateNeighbors_;

public:
  /** \brief Output user feedback to console */
  bool verbose_ = true;

  int numStartGoalStatesAddedToTask_ = 0;
};
}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_BOLT_BOLT_PLANNER_
