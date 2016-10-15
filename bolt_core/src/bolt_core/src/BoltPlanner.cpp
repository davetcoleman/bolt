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

// OMPL
#include <bolt_core/BoltPlanner.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/util/Console.h>

// Boost
#include <boost/thread.hpp>

// C++
#include <limits>

namespace og = ompl::geometric;
namespace ob = ompl::base;
namespace ot = ompl::tools;
namespace otb = ompl::tools::bolt;

namespace ompl
{
namespace tools
{
namespace bolt
{
BoltPlanner::BoltPlanner(const base::SpaceInformationPtr &si, const base::SpaceInformationPtr modelSI,
                         const TaskGraphPtr &taskGraph, VisualizerPtr visual)
  : base::Planner(si, "Bolt_Planner"), modelSI_(modelSI), taskGraph_(taskGraph), visual_(visual)
{
  specs_.approximateSolutions = false;
  specs_.directed = false;

  // Note that the path simplifier operates in the model_based_state_space, not the compound space
  path_simplifier_.reset(new geometric::PathSimplifier(modelSI));

  compoundSpace_ = std::dynamic_pointer_cast<base::CompoundStateSpace>(si_->getStateSpace());
  BOLT_ASSERT(compoundSpace_->isCompound(), "State space should be compound");
  BOLT_ASSERT(compoundSpace_->getSubspace(1)->getType() == ob::STATE_SPACE_DISCRETE, "Missing discrete subspace");
  BOLT_ASSERT(compoundSpace_->getSubspace(1)->getDimension() == 1, "Dimension is not 1");
  BOLT_ASSERT(compoundSpace_->getSubspace(0)->getType() == ob::STATE_SPACE_UNKNOWN,
              "State is wrong type");  // model_based_state_space
}

BoltPlanner::~BoltPlanner(void)
{
}

void BoltPlanner::clear(void)
{
  Planner::clear();
}

void BoltPlanner::setExperienceDB(const TaskGraphPtr &taskGraph)
{
  taskGraph_ = taskGraph;
}

void BoltPlanner::setup(void)
{
  Planner::setup();
}

base::PlannerStatus BoltPlanner::solve(Termination &ptc)
{
  std::size_t indent = 0;
  BOLT_FUNC(indent, verbose_, "BoltPlanner::solve()");

  // Check if the database is empty
  if (taskGraph_->isEmpty())
  {
    BOLT_DEBUG(indent, verbose_, "Task experience database is empty so unable to run BoltPlanner algorithm.");

    return base::PlannerStatus::ABORT;
  }

  taskGraph_->clearEdgeCollisionStates();

  // Restart the Planner Input States so that the first start and goal state can be fetched
  pis_.restart();  // PlannerInputStates

  // Get a single start and goal state
  BOLT_INFO(indent, verbose_, "Getting OMPL start and goal state");
  base::State *startState = modelSI_->getStateSpace()->cloneState(pis_.nextStart());  // PlannerInputStates
  base::State *goalState = modelSI_->getStateSpace()->cloneState(pis_.nextGoal(ptc));

  if (startState == nullptr)
  {
    BOLT_ERROR(indent, "No start state found");
    return base::PlannerStatus::ABORT;
  }

  if (goalState == nullptr)
  {
    BOLT_ERROR(indent, "No goal state found");
    return base::PlannerStatus::ABORT;
  }

  // Convert start and goal to compound states
  const int level0 = 0;
  const int level2 = 2;

  // These compound states now own the startState/goalState
  base::State *startStateCompound = taskGraph_->createCompoundState(startState, level0, indent);
  base::State *goalStateCompound = taskGraph_->createCompoundState(goalState, level2, indent);

  base::PlannerStatus result = solve(startStateCompound, goalStateCompound, ptc, indent);

  // Free states
  si_->getStateSpace()->freeState(startStateCompound);
  si_->getStateSpace()->freeState(goalStateCompound);

  return result;
}

base::PlannerStatus BoltPlanner::solve(base::State *startState, base::State *goalState, Termination &ptc,
                                       std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "solve() 2");

  // Create solution path as pointer so memory is not unloaded
  ob::PathPtr pathSolutionBase(new og::PathGeometric(si_));
  og::PathGeometric &geometricSolution = static_cast<og::PathGeometric &>(*pathSolutionBase);

  // Search
  if (!getPathOffGraph(startState, goalState, geometricSolution, ptc, indent))
  {
    BOLT_WARN(indent, true, "BoltPlanner::solve() No near start or goal found");
    return base::PlannerStatus::TIMEOUT;  // The planner failed to find a solution
  }

  BOLT_DEBUG(indent, verbose_, "getPathOffGraph() found a solution of size " << geometricSolution.getStateCount());

  // Save this for future debugging
  originalSolutionPath_.reset(new geometric::PathGeometric(geometricSolution));

  // All save trajectories should be at least 1 state long, then we append the start and goal states, for min of 3
  assert(geometricSolution.getStateCount() >= 3);

  // Smooth the result
  if (smoothingEnabled_)
  {
    if (taskGraph_->taskPlanningEnabled())
      simplifyTaskPath(geometricSolution, ptc, indent);
    else
      simplifyPath(geometricSolution, ptc, indent);
  }
  else
    BOLT_WARN(indent, true, "Smoothing not enabled");

  // Convert solution back to joint trajectory only (no discrete component)
  ob::PathPtr modelSolutionBase = taskGraph_->convertPathToNonCompound(pathSolutionBase);
  og::PathGeometric &modelSolution = static_cast<og::PathGeometric &>(*modelSolutionBase);

  // Show the smoothed path
  if (visualizeSmoothedTrajectory_)
  {
    visual_->viz4()->deleteAllMarkers();
    visual_->viz4()->path(&modelSolution, tools::MEDIUM, tools::BLACK, tools::BLACK);
    visual_->viz4()->trigger();
  }

  // Save solution
  double approximateDifference = -1;
  bool approximate = false;
  pdef_->addSolutionPath(modelSolutionBase, approximate, approximateDifference, getName());
  bool solved = true;

  BOLT_DEBUG(indent, verbose_, "Finished BoltPlanner.solve()");
  return base::PlannerStatus(solved, approximate);
}

bool BoltPlanner::getPathOffGraph(const base::State *start, const base::State *goal,
                                  og::PathGeometric &geometricSolution, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "BoltPlanner::getPathOffGraph()");

  // Attempt to connect to graph x times, because if it fails we start adding samples
  std::size_t maxAttempts = 2;
  std::size_t attempt = 0;
  for (; attempt < maxAttempts; ++attempt)
  {
    BOLT_DEBUG(indent, verbose_, "Starting getPathOffGraph() attempt " << attempt);

    // Get neighbors near start and goal. Note: potentially they are not *visible* - will test for this later

    // Start
    int level = taskGraph_->getTaskLevel(start);
    BOLT_DEBUG(indent, verbose_, "Looking for a node near the problem start on level " << level);
    if (!findGraphNeighbors(start, startVertexCandidateNeighbors_, level, indent))
    {
      BOLT_DEBUG(indent, verbose_, "No graph neighbors found for start");
      return false;
    }
    BOLT_DEBUG(indent, verbose_, "Found " << startVertexCandidateNeighbors_.size() << " nodes near start");

    // Goal
    level = taskGraph_->getTaskLevel(goal);
    BOLT_DEBUG(indent, verbose_, "Looking for a node near the problem goal on level " << level);
    if (!findGraphNeighbors(goal, goalVertexCandidateNeighbors_, level, indent))
    {
      BOLT_DEBUG(indent, verbose_, "No graph neighbors found for goal");
      return false;
    }
    BOLT_DEBUG(indent, verbose_, "Found " << goalVertexCandidateNeighbors_.size() << " nodes near goal");

    // Get paths between start and goal
    bool feedbackStartFailed;
    const bool debug = false;
    bool result = getPathOnGraph(startVertexCandidateNeighbors_, goalVertexCandidateNeighbors_, start, goal,
                                 geometricSolution, ptc, debug, feedbackStartFailed, indent);

    // Error check
    if (!result)
    {
      BOLT_WARN(indent, true, "getPathOffGraph(): BoltPlanner returned FALSE for getPathOnGraph. Trying again in debug "
                              "mode");

      // Run getPathOnGraph again in debug mode
      getPathOnGraph(startVertexCandidateNeighbors_, goalVertexCandidateNeighbors_, start, goal, geometricSolution, ptc,
                     /*debug*/ true, feedbackStartFailed, indent);

      /*
      // Add the point that failed
      if (feedbackStartFailed)  // start state failed
      {
        // Create the vertex then connect to taskGraph
        // TODO(davetcoleman): how to prevent from adding same state twice?
        TaskVertex taskV = taskGraph_->addVertex(si_->cloneState(start), QUALITY);
        taskGraph_->connectNewVertex(taskV);
      }
      else  // goal state failed
      {
        // Create the vertex then connect to taskGraph
        // TODO(davetcoleman): how to prevent from adding same state twice?
        TaskVertex taskV = taskGraph_->addVertex(si_->cloneState(goal), QUALITY);
        taskGraph_->connectNewVertex(taskV);
      }
      numStartGoalStatesAddedToTask_++;  // for later analysis

      */
      std::cout << "Shutting down for debugging " << std::endl;
      exit(-1);
    }
    else
      break;  // success, continue on
  }

  // Did we finally get it?
  if (attempt >= maxAttempts)
  {
    return false;
  }

  // All save trajectories should be at least 1 state long, then we append the start and goal states, for min of 3
  assert(geometricSolution.getStateCount() >= 3);

  // Debug output
  if (false)
  {
    for (std::size_t i = 0; i < geometricSolution.getStateCount(); ++i)
    {
      BOLT_DEBUG(indent, verbose_, "getPathOffGraph(): Adding state " << i << " to plannerData");
      si_->printState(geometricSolution.getState(i), std::cout);
    }
  }

  return true;
}

bool BoltPlanner::getPathOnGraph(const std::vector<TaskVertex> &candidateStarts,
                                 const std::vector<TaskVertex> &candidateGoals, const base::State *actualStart,
                                 const base::State *actualGoal, og::PathGeometric &geometricSolution, Termination &ptc,
                                 bool debug, bool &feedbackStartFailed, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "BoltPlanner::getPathOnGraph()");

  bool foundValidStart = false;
  bool foundValidGoal = false;

  // Try every combination of nearby start and goal pairs
  for (TaskVertex startVertex : candidateStarts)
  {
    // Check if this start is visible from the actual start
    if (!taskGraph_->checkMotion(actualStart, taskGraph_->getState(startVertex)))
    {
      BOLT_WARN(indent, verbose_, "Found start candidate that is not visible on vertex " << startVertex);

      if (debug)
      {
        // visual_->viz4()->state(taskGraph_->getModelBasedState(startVertex), tools::LARGE, tools::RED, 1);
        visual_->viz4()->state(taskGraph_->getModelBasedState(startVertex), tools::ROBOT, tools::RED, 1);
        visual_->viz5()->state(taskGraph_->getModelBasedState(actualStart), tools::ROBOT, tools::GREEN, 1);

        visual_->viz4()->edge(taskGraph_->getModelBasedState(actualStart), taskGraph_->getModelBasedState(startVertex),
                              tools::MEDIUM, tools::BLACK);
        visual_->viz4()->trigger();
        // usleep(0.1 * 1000000);
        visual_->waitForUserFeedback("not visible");
      }
      continue;  // this is actually not visible
    }
    foundValidStart = true;

    for (TaskVertex goal : candidateGoals)
    {
      BOLT_DEBUG(indent, verbose_, "foreach_goal: Checking motion from " << actualGoal << " to "
                                                                         << taskGraph_->getState(goal));

      if (ptc)  // Check if our planner is out of time
      {
        BOLT_DEBUG(indent, verbose_, "getPathOnGraph function interrupted because termination condition is true.");
        return false;
      }

      // Check if this goal is visible from the actual goal
      if (!taskGraph_->checkMotion(actualGoal, taskGraph_->getState(goal)))
      {
        BOLT_WARN(indent, verbose_, "FOUND GOAL CANDIDATE THAT IS NOT VISIBLE! ");

        if (debug)
        {
          visual_->viz4()->state(taskGraph_->getModelBasedState(goal), tools::SMALL, tools::RED, 1);
          visual_->viz4()->edge(taskGraph_->getModelBasedState(actualGoal), taskGraph_->getModelBasedState(goal),
                                tools::MEDIUM, tools::BLACK);
          visual_->viz4()->trigger();
          usleep(0.1 * 1000000);
        }

        continue;  // this is actually not visible
      }
      foundValidGoal = true;

      // Repeatidly search through graph for connection then check for collisions then repeat
      if (lazyCollisionSearch(startVertex, goal, actualStart, actualGoal, geometricSolution, ptc, indent))
      {
        // All save trajectories should be at least 1 state long, then we append the start and goal states, for
        // min of 3
        assert(geometricSolution.getStateCount() >= 3);

        // Found a path
        return true;
      }
      else
      {
        // Did not find a path
        BOLT_DEBUG(indent, verbose_, "Did not find a path, looking for other start/goal combinations ");
      }

    }  // foreach
  }    // foreach

  if (foundValidStart && foundValidGoal)
  {
    BOLT_ERROR(indent, "Unexpected condition - both a valid start and goal were found but still no path found. "
                       "TODO ");
    exit(-1);
  }

  if (foundValidStart && !foundValidGoal)
  {
    BOLT_WARN(indent, true, "Unable to connect GOAL state to graph");
    feedbackStartFailed = false;  // it was the goal state that failed us
  }
  else
  {
    BOLT_WARN(indent, true, "Unable to connect START state to graph");
    feedbackStartFailed = true;  // the start state failed us
  }

  return false;
}

bool BoltPlanner::lazyCollisionSearch(const TaskVertex &startVertex, const TaskVertex &goalVertex,
                                      const base::State *actualStart, const base::State *actualGoal,
                                      og::PathGeometric &geometricSolution, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "BoltPlanner::lazyCollisionSearch()");

  // Vector to store candidate paths in before they are converted to PathPtrs
  std::vector<TaskVertex> vertexPath;
  double distance;  // resulting path distance

  // Make sure that the start and goal aren't so close together that they find the same vertex
  if (startVertex == goalVertex)
  {
    BOLT_DEBUG(indent, verbose_, "    Start equals goal, creating simple solution ");
    visual_->waitForUserFeedback("    Start equals goal, creating simple solution ");

    // There are only three verticies in this path - start, middle, goal
    vertexPath.push_back(startVertex);

    convertVertexPathToStatePath(vertexPath, actualStart, actualGoal, geometricSolution, indent);
    return true;
  }

  // Error check all states are non-nullptr
  assert(actualStart);
  assert(actualGoal);
  assert(taskGraph_->getState(startVertex));
  assert(taskGraph_->getState(goalVertex));

  // Visualize start vertex
  if (visualizeStartGoal_)
  {
    BOLT_DEBUG(indent, verbose_, "viz start -----------------------------");
    // visual_->viz5()->state(taskGraph_->getModelBasedState(startVertex), tools::VARIABLE_SIZE, tools::PURPLE, 1);
    visual_->viz4()->state(taskGraph_->getModelBasedState(startVertex), tools::ROBOT, tools::ORANGE);
    // visual_->viz5()->edge(actualStart, taskGraph_->getModelBasedState(startVertex), tools::MEDIUM, tools::BLACK);
    //visual_->viz5()->trigger();
    //visual_->waitForUserFeedback("start viz");

    // Visualize goal vertex
    BOLT_DEBUG(indent, verbose_, "viz goal ------------------------------");
    // visual_->viz5()->state(taskGraph_->getModelBasedState(goalVertex), tools::VARIABLE_SIZE, tools::PURPLE, 1);
    visual_->viz5()->state(taskGraph_->getModelBasedState(goalVertex), tools::ROBOT, tools::GREEN);
    // visual_->viz5()->edge(actualGoal, taskGraph_->getModelBasedState(goalVertex), tools::MEDIUM, tools::BLACK);
    //visual_->viz5()->trigger();
    //visual_->waitForUserFeedback("goal viz");
  }

  // Keep looking for paths between chosen start and goal until one is found that is valid,
  // or no further paths can be found between them because of disabled edges
  // this is necessary for lazy collision checking i.e. rerun after marking invalid edges we found
  while (!visual_->viz1()->shutdownRequested())
  {
    // Check if our planner is out of time
    if (ptc)
    {
      BOLT_DEBUG(indent + 2, verbose_, "lazyCollisionSearch: function interrupted because termination condition is "
                                       "true.");
      return false;
    }

    // Attempt to find a solution from start to goal
    if (!taskGraph_->astarSearch(startVertex, goalVertex, vertexPath, distance, indent + 2))
    {
      BOLT_DEBUG(indent + 2, verbose_, "unable to construct solution between start and goal using astar");

      // no path found what so ever
      return false;
    }

    // Check if all the points in the potential solution are valid
    if (lazyCollisionCheck(vertexPath, ptc, indent + 2))
    {
      BOLT_DEBUG(indent + 2, verbose_, "Lazy collision check returned valid ");

      // the path is valid, we are done!
      convertVertexPathToStatePath(vertexPath, actualStart, actualGoal, geometricSolution, indent + 2);
      return true;
    }
    // else, loop with updated graph that has the invalid edges/states disabled
  }  // end while

  // we never found a valid path
  return false;
}

bool BoltPlanner::lazyCollisionCheck(std::vector<TaskVertex> &vertexPath, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "BoltPlanner::lazyCollisionCheck() path of size " << vertexPath.size());

  bool hasInvalidEdges = false;

  // Initialize
  TaskVertex fromVertex = vertexPath[0];
  TaskVertex toVertex;

  // Loop through every pair of states and make sure path is valid.
  for (std::size_t toID = 1; toID < vertexPath.size(); ++toID)
  {
    // Increment location on path
    toVertex = vertexPath[toID];

    // Check if our planner is out of time
    if (ptc)
    {
      BOLT_DEBUG(indent, verbose_, "Lazy collision check function interrupted because termination condition is true.");
      return false;
    }

    TaskEdge thisEdge = boost::edge(fromVertex, toVertex, taskGraph_->g_).first;

    // Has this edge already been checked before?
    if (taskGraph_->getGraphNonConst()[thisEdge].collision_state_ == NOT_CHECKED)
    {
      // TODO - is checking edges sufficient, or do we also need to check vertices? I think its fine.
      // Check path between states
      if (!taskGraph_->checkMotion(taskGraph_->getState(fromVertex), taskGraph_->getState(toVertex)))
      {
        // Path between (from, to) states not valid, disable the edge

        BOLT_MAGENTA(indent, vCollisionCheck_, "LAZY CHECK: disabling edge from vertex " << fromVertex << " to " << toVertex);

        if (visualizeLazyCollisionCheck_)
          visualizeBadEdge(fromVertex, toVertex);

        // Disable edge
        taskGraph_->getGraphNonConst()[thisEdge].collision_state_ = IN_COLLISION;
      }
      else
      {
        // Mark edge as free so we no longer need to check for collision
        taskGraph_->getGraphNonConst()[thisEdge].collision_state_ = FREE;
      }
    }
    else if (taskGraph_->getGraphNonConst()[thisEdge].collision_state_ == IN_COLLISION)
    {
      if (visualizeLazyCollisionCheck_)
        visualizeBadEdge(fromVertex, toVertex);

      BOLT_ERROR(indent, "Somehow an edge " << thisEdge << " was found that is already in collision before lazy collision checking");
    }

    // Check final result
    if (taskGraph_->getGraphNonConst()[thisEdge].collision_state_ == IN_COLLISION)
    {
      // Remember that this path is no longer valid, but keep checking remainder of path edges
      hasInvalidEdges = true;
    }

    // switch vertex focus
    fromVertex = toVertex;

    if (visual_->viz1()->shutdownRequested())
      break;

  }  // for

  // Only return true if nothing was found invalid
  return !hasInvalidEdges;
}

bool BoltPlanner::findGraphNeighbors(const base::State *state, std::vector<TaskVertex> &neighbors, int requiredLevel,
                                     std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "BoltPlanner::findGraphNeighbors()");

  BOLT_ASSERT(requiredLevel == 0 || requiredLevel == 2, "Wrong required level");

  // Reset
  neighbors.clear();

  // Search within double the radius of the sparse delta
  // Technically you should only need to search within 1x, but collisions etc could possibly require looking in
  // a larger region
  // double radius = taskGraph_->getSparseGraph()->getSparseDelta() * 2.0;

  // Choose how many neighbors
  double kNearestNeighbors;
  // if (si_->getStateSpace()->getDimension() == 3)
  //   kNearestNeighbors = 10;
  // else
  kNearestNeighbors = 30;

  // Setup search by getting a non-const version of the focused state
  const std::size_t threadID = 0;
  base::State *stateCopy = si_->cloneState(state);  // TODO avoid this memory allocation but keeping as member variable

  // Search
  taskGraph_->getQueryStateNonConst(taskGraph_->queryVertices_[threadID]) = stateCopy;
  // taskGraph_->nn_->nearestR(taskGraph_->queryVertices_[threadID], radius, neighbors);
  taskGraph_->nn_->nearestK(taskGraph_->queryVertices_[threadID], kNearestNeighbors, neighbors);
  taskGraph_->getQueryStateNonConst(taskGraph_->queryVertices_[threadID]) = nullptr;

  // Convert our list of neighbors to the proper level
  if (requiredLevel == 2)
  {
    BOLT_DEBUG(indent, verbose_, "Converting vector of level 0 neighbors to level 2 neighbors");

    for (std::size_t i = 0; i < neighbors.size(); ++i)
    {
      const TaskVertex &nearVertex = neighbors[i];

      // Get the vertex on the opposite level and replace it in the vector
      TaskVertex newVertex = taskGraph_->getGraphNonConst()[nearVertex].task_mirror_;

      // Replace
      neighbors[i] = newVertex;  // TODO: can this allocation be done in-place?
    }
  }

  // Free memory
  si_->getStateSpace()->freeState(stateCopy);

  return neighbors.size();
}

bool BoltPlanner::convertVertexPathToStatePath(std::vector<TaskVertex> &vertexPath, const base::State *actualStart,
                                               const base::State *actualGoal, og::PathGeometric &geometricSolution,
                                               std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "convertVertexPathToStatePath()");

  // Ensure the input path is not empty
  if (!vertexPath.size())
    return false;

 // TODO: remove this check
  BOLT_ASSERT(actualStart != taskGraph_->getState(vertexPath.back()), "Unexpected same states, should not append actualStart");
  BOLT_WARN(indent, true, "not sure if this is needed");
  // Add original start
  geometricSolution.append(actualStart);

  // Error check that no consequtive verticies are the same
  if (verbose_)
  {
    std::cout << "BoltPlanner.Vertices: ";
    for (std::size_t i = vertexPath.size(); i > 0; --i)
    {
      std::cout << vertexPath[i - 1] << ", ";
    }
    std::cout << std::endl;
  }

  // Reverse the vertexPath and convert to state path
  for (std::size_t i = vertexPath.size(); i > 0; --i)
  {
    geometricSolution.append(taskGraph_->getState(vertexPath[i - 1]));

    // Add the edge status
    if (i > 1)  // skip the last vertex (its reversed)
    {
      // Error check that no consequtive verticies are the same
      if (vertexPath[i - 1] == vertexPath[i - 2])
      {
        BOLT_ERROR(indent, "Found repeated vertices " << vertexPath[i - 1] << " to " << vertexPath[i - 2]
                                                      << " from index " << i);
        exit(-1);
      }

      TaskEdge edge = boost::edge(vertexPath[i - 1], vertexPath[i - 2], taskGraph_->g_).first;

      // Check if any edges in path are not free (then it an approximate path)
      if (taskGraph_->getGraphNonConst()[edge].collision_state_ == IN_COLLISION)
      {
        BOLT_ERROR(indent, "Found invalid edge / approximate solution - how did this happen?");
      }
      else if (taskGraph_->getGraphNonConst()[edge].collision_state_ == NOT_CHECKED)
      {
        BOLT_ERROR(indent, "A chosen path has an edge " << edge << " that has not been checked for collision. This "
                                                                   "should not happen");
      }
    }
  }

  // Add original goal if it is different than the last state
  if (actualGoal != taskGraph_->getState(vertexPath.front()))
  {
    geometricSolution.append(actualGoal);
  }

  return true;
}

bool BoltPlanner::simplifyPath(og::PathGeometric &path, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "BoltPlanner: simplifyPath(): non-task version");

  time::point simplifyStart = time::now();
  std::size_t numStates = path.getStateCount();

  BOLT_ERROR(indent, "this path simplifier might be using the wrong statespace path");
  path_simplifier_->simplify(path, ptc);
  double simplifyTime = time::seconds(time::now() - simplifyStart);

  int diff = numStates - path.getStateCount();
  BOLT_DEBUG(indent, verbose_ || true, "BoltPlanner: Path simplification took "
                                           << simplifyTime << " seconds and removed " << diff << " states");

  return true;
}

bool BoltPlanner::simplifyTaskPath(og::PathGeometric &geometricSolution, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, true, "BoltPlanner: simplifyTaskPath()");

  time::point simplifyStart = time::now();
  std::size_t origNumStates = geometricSolution.getStateCount();

  // Number of levels
  const std::size_t NUM_LEVELS = 3;

  // Create three path segments
  std::vector<og::PathGeometric> pathSegment;
  for (int segmentLevel = 0; segmentLevel < int(NUM_LEVELS); ++segmentLevel)
    pathSegment.push_back(og::PathGeometric(modelSI_));

  // Create the solution path
  og::PathGeometric smoothedPath(si_);

  // Divide the path into different levels
  VertexLevel previousLevel = 0;  // Error check ordering of input path
  for (std::size_t i = 0; i < geometricSolution.getStateCount(); ++i)
  {
    base::State *state = geometricSolution.getState(i);
    VertexLevel level = taskGraph_->getTaskLevel(state);

    BOLT_DEBUG(indent, verbose_, "Path on level " << level);

    assert(level < NUM_LEVELS);

    pathSegment[level].append(taskGraph_->getModelBasedState(state));

    if (previousLevel > level)  // Error check ordering of input path
      throw Exception(name_, "Level increasing in wrong order");
    previousLevel = level;
  }

  // Add the start and end vertex of the Cartesian path into the respective freespace paths
  if (pathSegment[1].getStateCount() >= 2)
  {
    // For checking for errors
    std::size_t seg0Size = pathSegment[0].getStateCount();
    std::size_t seg1Size = pathSegment[1].getStateCount();
    std::size_t seg2Size = pathSegment[2].getStateCount();

    // Move first state
    pathSegment[0].append(pathSegment[1].getStates().front());
    pathSegment[1].getStates().erase(pathSegment[1].getStates().begin());

    // Move last state
    pathSegment[2].prepend(pathSegment[1].getStates().back());
    pathSegment[1].getStates().pop_back();

    // Check the operations were correct
    BOLT_ASSERT(seg0Size + 1 == pathSegment[0].getStateCount(), "Invalid size of pathSegement after rearrangment");
    BOLT_ASSERT(seg1Size - 2 == pathSegment[1].getStateCount(), "Invalid size of pathSegement after rearrangment");
    BOLT_ASSERT(seg2Size + 1 == pathSegment[2].getStateCount(), "Invalid size of pathSegement after rearrangment");
  }
  else
  {
    BOLT_WARN(indent, true, "The Cartesian path segement 1 has only " << pathSegment[1].getStateCount() << " states");
  }

  // Smooth the freespace paths
  path_simplifier_->simplifyMax(pathSegment[0]);
  path_simplifier_->simplifyMax(pathSegment[2]);

  // Combine the path segments back together
  for (int segmentLevel = 0; segmentLevel < int(NUM_LEVELS); ++segmentLevel)
  {
    for (std::size_t i = 0; i < pathSegment[segmentLevel].getStateCount(); ++i)
    {
      // This state is not compound
      base::State *modelBasedState = pathSegment[segmentLevel].getState(i);

      base::State *compoundState = taskGraph_->createCompoundState(modelBasedState, segmentLevel, indent);

      if (false)
      {
        visual_->viz4()->state(taskGraph_->getModelBasedState(compoundState), tools::ROBOT, tools::DEFAULT, 0);
        visual_->waitForUserFeedback("next step");
      }

      // Check for repeated states
      if (i > 0)
      {
        if (modelSI_->equalStates(pathSegment[segmentLevel].getState(i - 1), pathSegment[segmentLevel].getState(i)))
        {
          BOLT_ERROR(indent, "Repeated states at " << i << " on level " << segmentLevel);
        }
      }

      // Add to solution path
      smoothedPath.append(compoundState);
    }
  }


  // Replace the input path with the new smoothed path
  geometricSolution = smoothedPath;

  double simplifyTime = time::seconds(time::now() - simplifyStart);

  int diff = origNumStates - geometricSolution.getStateCount();
  BOLT_DEBUG(indent, verbose_, "BoltPlanner: Path simplification took " << simplifyTime << " seconds and removed "
                                                                        << diff << " states");

  return true;
}

// This is used to check connectivity of graph
bool BoltPlanner::canConnect(const base::State *randomState, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "BoltPlanner::canConnect()");

  std::vector<TaskVertex> candidateNeighbors;

  // Find neighbors to rand state
  BOLT_DEBUG(indent, verbose_, "Looking for a node near the random state");
  if (!findGraphNeighbors(randomState, candidateNeighbors))
  {
    BOLT_DEBUG(indent, verbose_, "No graph neighbors found for randomState");
    return false;
  }
  BOLT_DEBUG(indent, verbose_, "Found " << candidateNeighbors.size() << " nodes near randomState");

  // Try every combination of nearby start and goal pairs
  std::size_t count = 0;
  for (TaskVertex nearState : candidateNeighbors)
  {
    const base::State *s1 = randomState;
    const base::State *s2 = taskGraph_->getState(nearState);

    // Check if this nearState is visible from the random state
    if (!taskGraph_->checkMotion(s1, s2))
    {
      BOLT_WARN(indent, true, "NEIGHBOR " << count++ << " NOT VISIBLE ");

      if (false)
      {
        visual_->viz5()->state(taskGraph_->getModelBasedState(s2), tools::MEDIUM, tools::BLUE, 1);
        visual_->viz5()->edge(taskGraph_->getModelBasedState(s1), taskGraph_->getModelBasedState(s2), tools::MEDIUM,
                              tools::BLACK);
        visual_->viz5()->trigger();
        usleep(1 * 1000000);
      }

      // Optional Debug
      if (false)
      {
        std::cout << "checking path " << std::endl;
        std::vector<base::State *> states;
        unsigned int count = si_->getStateSpace()->validSegmentCount(s1, s2);
        // std::cout << "count: " << count << std::endl;

        bool endpoints = false;
        bool alloc = true;
        si_->getMotionStates(s1, s2, states, count, endpoints, alloc);
        // std::cout << "state size: " << states.size() << std::endl;

        for (base::State *interState : states)
        {
          // Check if our planner is out of time
          if (ptc)
          {
            BOLT_DEBUG(indent, verbose_, "Quit requested");
            return false;
          }

          if (!si_->isValid(interState))
          {
            visual_->viz5()->state(taskGraph_->getModelBasedState(interState), tools::LARGE, tools::RED, 1);
            visual_->viz5()->trigger();
            usleep(1 * 1000000);
          }
          else
          {
            // visual_->viz5()->state(taskGraph_->getModelBasedState(interState), /*mode=*/1, 1); // GREEN
          }
        }
      }
    }
    else
    {
      BOLT_DEBUG(indent, verbose_, "Has connection");
      return true;
    }
  }
  return false;
}

void BoltPlanner::visualizeBadEdge(TaskVertex fromVertex, TaskVertex toVertex)
{
  const base::State* from = taskGraph_->getModelBasedState(fromVertex);
  const base::State* to = taskGraph_->getModelBasedState(toVertex);

  // Line
  visual_->viz5()->edge(from, to, tools::MEDIUM, tools::RED);
  visual_->viz5()->trigger();

  // Robot states
  visual_->viz4()->state(from, tools::ROBOT, tools::RED, 0);
  visual_->viz5()->state(to, tools::ROBOT, tools::RED, 0);

  visual_->waitForUserFeedback("collision");
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
