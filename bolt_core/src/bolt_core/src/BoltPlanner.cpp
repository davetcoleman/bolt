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
BoltPlanner::BoltPlanner(const base::SpaceInformationPtr modelSI, const base::SpaceInformationPtr compoundSI,
                         const TaskGraphPtr &taskGraph, VisualizerPtr visual)
  : base::Planner(modelSI, "Bolt_Planner")
  , modelSI_(modelSI)
  , compoundSI_(compoundSI)
  , taskGraph_(taskGraph)
  , visual_(visual)
{
  specs_.approximateSolutions = false;
  specs_.directed = false;

  // Note that the path simplifier operates in the model_based_state_space, not the compound space
  path_simplifier_.reset(new geometric::PathSimplifier(modelSI_));

  base::CompoundStateSpacePtr compoundSpace =
      std::dynamic_pointer_cast<base::CompoundStateSpace>(compoundSI_->getStateSpace());
  BOLT_ASSERT(compoundSpace->isCompound(), "State space should be compound");
  BOLT_ASSERT(compoundSpace->getSubspace(1)->getType() == ob::STATE_SPACE_DISCRETE, "Missing discrete subspace");
  BOLT_ASSERT(compoundSpace->getSubspace(1)->getDimension() == 1, "Dimension is not 1");
  BOLT_ASSERT(compoundSpace->getSubspace(0)->getType() == ob::STATE_SPACE_UNKNOWN,
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
  BOLT_FUNC(indent, verbose_, "solve()");

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
  compoundSI_->getStateSpace()->freeState(startStateCompound);
  compoundSI_->getStateSpace()->freeState(goalStateCompound);

  return result;
}

base::PlannerStatus BoltPlanner::solve(base::State *startState, base::State *goalState, Termination &ptc,
                                       std::size_t indent)
{
  // Create solution path as pointer so memory is not unloaded
  ob::PathPtr compoundSolutionBase(new og::PathGeometric(compoundSI_));
  og::PathGeometric &compoundSolution = static_cast<og::PathGeometric &>(*compoundSolutionBase);

  // Search
  if (!getPathOffGraph(startState, goalState, compoundSolution, ptc, indent))
  {
    BOLT_WARN(indent, true, "solve() No near start or goal found");
    return base::PlannerStatus::TIMEOUT;  // The planner failed to find a solution
  }

  BOLT_DEBUG(indent, verbose_, "getPathOffGraph() found a solution of size " << compoundSolution.getStateCount());

  // Save this for future debugging
  originalSolutionPath_.reset(new geometric::PathGeometric(compoundSolution));

  // All save trajectories should be at least 1 state long, then we append the start and goal states, for min of 3
  assert(compoundSolution.getStateCount() >= 3);

  // Smooth the result
  if (smoothingEnabled_)
  {
    if (taskGraph_->taskPlanningEnabled())
      simplifyTaskPath(compoundSolution, ptc, indent);
    else
      simplifyPath(compoundSolution, ptc, indent);
  }
  else
    BOLT_WARN(indent, true, "Smoothing not enabled");

  // Convert solution back to joint trajectory only (no discrete component)
  ob::PathPtr modelSolutionBase = taskGraph_->convertPathToNonCompound(compoundSolutionBase);
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
                                  og::PathGeometric &compoundSolution, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "getPathOffGraph()");

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
                                 compoundSolution, ptc, debug, feedbackStartFailed, indent);

    // Error check
    if (!result)
    {
      return false;
      /*
      BOLT_WARN(indent, true, "getPathOffGraph(): BoltPlanner returned FALSE for getPathOnGraph. Trying again in debug "
                              "mode");

      // Run getPathOnGraph again in debug mode
      getPathOnGraph(startVertexCandidateNeighbors_, goalVertexCandidateNeighbors_, start, goal, compoundSolution, ptc,
                     /*debug* / true, feedbackStartFailed, indent);

      std::cout << "Shutting down for debugging " << std::endl;
      exit(-1);
      */
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
  assert(compoundSolution.getStateCount() >= 3);

  return true;
}

bool BoltPlanner::getPathOnGraph(const std::vector<TaskVertex> &candidateStarts,
                                 const std::vector<TaskVertex> &candidateGoals, const base::State *actualStart,
                                 const base::State *actualGoal, og::PathGeometric &compoundSolution, Termination &ptc,
                                 bool debug, bool &feedbackStartFailed, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "getPathOnGraph()");

  bool foundValidStart = false;
  bool foundValidGoal = false;

  // Try every combination of nearby start and goal pairs
  for (TaskVertex startVertex : candidateStarts)
  {
    // Check if this start is visible from the actual start
    if (!taskGraph_->checkMotion(actualStart, taskGraph_->getCompoundState(startVertex)))
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
      BOLT_DEBUG(indent, true || verbose_, "Planning from candidate start/goal pair "
                 << actualGoal << " to " << taskGraph_->getCompoundState(goal));

      if (ptc)  // Check if our planner is out of time
      {
        BOLT_DEBUG(indent, verbose_, "getPathOnGraph function interrupted because termination condition is true.");
        return false;
      }

      // Check if this goal is visible from the actual goal
      if (!taskGraph_->checkMotion(actualGoal, taskGraph_->getCompoundState(goal)))
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
      if (onGraphSearch(startVertex, goal, actualStart, actualGoal, compoundSolution, ptc, indent))
      {
        // All save trajectories should be at least 1 state long, then we append the start and goal states, for
        // min of 3
        assert(compoundSolution.getStateCount() >= 3);

        // Found a path
        return true;
      }
      else
      {
        // Did not find a path
        BOLT_DEBUG(indent, verbose_, "Did not find a path, looking for other start/goal combinations ");
      }

      if (visual_->viz1()->shutdownRequested())
        break;
    }  // foreach

    if (visual_->viz1()->shutdownRequested())
      break;
  }    // foreach

  if (foundValidStart && foundValidGoal)
  {
    BOLT_ERROR(indent, "Both a valid start and goal were found but still no path found.");
    return false;
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

bool BoltPlanner::onGraphSearch(const TaskVertex &startVertex, const TaskVertex &goalVertex,
                                const base::State *actualStart, const base::State *actualGoal,
                                og::PathGeometric &compoundSolution, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "onGraphSearch()");

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

    convertVertexPathToStatePath(vertexPath, actualStart, actualGoal, compoundSolution, indent);
    return true;
  }

  // Error check all states are non-nullptr
  assert(actualStart);
  assert(actualGoal);
  assert(taskGraph_->getCompoundState(startVertex));
  assert(taskGraph_->getCompoundState(goalVertex));

  // Visualize start vertex
  if (visualizeStartGoal_)
  {
    BOLT_DEBUG(indent, verbose_, "viz start -----------------------------");
    // visual_->viz5()->state(taskGraph_->getModelBasedState(startVertex), tools::VARIABLE_SIZE, tools::PURPLE, 1);
    visual_->viz4()->state(taskGraph_->getModelBasedState(startVertex), tools::ROBOT, tools::ORANGE);
    // visual_->viz5()->edge(actualStart, taskGraph_->getModelBasedState(startVertex), tools::MEDIUM, tools::BLACK);
    // visual_->viz5()->trigger();
    // visual_->waitForUserFeedback("start viz");

    // Visualize goal vertex
    BOLT_DEBUG(indent, verbose_, "viz goal ------------------------------");
    // visual_->viz5()->state(taskGraph_->getModelBasedState(goalVertex), tools::VARIABLE_SIZE, tools::PURPLE, 1);
    visual_->viz5()->state(taskGraph_->getModelBasedState(goalVertex), tools::ROBOT, tools::GREEN);
    // visual_->viz5()->edge(actualGoal, taskGraph_->getModelBasedState(goalVertex), tools::MEDIUM, tools::BLACK);
    // visual_->viz5()->trigger();
    // visual_->waitForUserFeedback("goal viz");
  }

  // Keep looking for paths between chosen start and goal until one is found that is valid,
  // or no further paths can be found between them because of disabled edges
  // this is necessary for lazy collision checking i.e. rerun after marking invalid edges we found
  while (!visual_->viz1()->shutdownRequested())
  {
    // Check if our planner is out of time
    if (ptc)
    {
      BOLT_DEBUG(indent, verbose_, "Function interrupted because termination condition is true");
      return false;
    }

    // Attempt to find a solution from start to goal
    if (!taskGraph_->astarSearch(startVertex, goalVertex, vertexPath, distance, indent))
    {
      BOLT_WARN(indent, true || verbose_, "Unable to construct solution between start and goal using astar");

      // no path found what so ever
      return false;
    }

    // Check if all the points in the potential solution are valid
    if (lazyCollisionCheck(vertexPath, ptc, indent))
    {
      BOLT_DEBUG(indent, verbose_, "Lazy collision check returned valid ");

      // the path is valid, we are done!
      convertVertexPathToStatePath(vertexPath, actualStart, actualGoal, compoundSolution, indent);
      return true;
    }
    // else, loop with updated graph that has the invalid edges/states disabled
  }  // end while

  // we never found a valid path
  return false;
}

bool BoltPlanner::lazyCollisionCheck(std::vector<TaskVertex> &vertexPath, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "lazyCollisionCheck() path of size " << vertexPath.size());

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

    TaskEdge thisEdge = boost::edge(fromVertex, toVertex, taskGraph_->getGraph()).first;

    // Has this edge already been checked before?
    if (taskGraph_->getGraphNonConst()[thisEdge].collision_state_ == NOT_CHECKED)
    {
      // TODO - is checking edges sufficient, or do we also need to check vertices? I think its fine.
      // Check path between states
      const base::State *fromState = taskGraph_->getModelBasedState(fromVertex);
      const base::State *toState = taskGraph_->getModelBasedState(toVertex);
      if (!modelSI_->getMotionValidator()->checkMotion(fromState, toState))
      {
        BOLT_MAGENTA(indent, vCollisionCheck_, "LAZY CHECK: disabling edge from vertex " << fromVertex << " to "
                                                                                                 << toVertex);
        if (visualizeLazyCollisionCheck_)
        {
          visual_->waitForUserFeedback("see edge");

          // Path between (from, to) states not valid, disable the edge
          modelSI_->getMotionValidator()->checkMotion(fromState, toState, visual_);

          visualizeBadEdge(fromVertex, toVertex);
        }

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

      BOLT_ERROR(indent, "Somehow an edge " << thisEdge << " was found that is already in collision before lazy "
                                                           "collision checking");
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
  BOLT_FUNC(indent, verbose_, "findGraphNeighbors()");

  BOLT_ASSERT(requiredLevel == 0 || requiredLevel == 2, "Wrong required level");

  // Reset
  neighbors.clear();

  // Search within double the radius of the sparse delta
  // Technically you should only need to search within 1x, but collisions etc could possibly require looking in
  // a larger region
  // double radius = taskGraph_->getSparseGraph()->getSparseDelta() * 2.0;

  // Choose how many neighbors
  double kNearestNeighbors;
  // if (modelSI_->getStateSpace()->getDimension() == 3)
  //   kNearestNeighbors = 10;
  // else
  kNearestNeighbors = 30;

  // Setup search by getting a non-const version of the focused state
  const std::size_t threadID = 0;
  // TODO avoid this memory allocation but keeping as member variable
  base::State *stateCopy = compoundSI_->cloneState(state);

  // Search
  taskGraph_->getCompoundQueryStateNonConst(taskGraph_->getQueryVertices()[threadID]) = stateCopy;
  // taskGraph_->nn_->nearestR(taskGraph_->getQueryVertices()[threadID], radius, neighbors);
  taskGraph_->getNN()->nearestK(taskGraph_->getQueryVertices()[threadID], kNearestNeighbors, neighbors);
  taskGraph_->getCompoundQueryStateNonConst(taskGraph_->getQueryVertices()[threadID]) = nullptr;

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
  compoundSI_->getStateSpace()->freeState(stateCopy);

  return neighbors.size();
}

bool BoltPlanner::convertVertexPathToStatePath(std::vector<TaskVertex> &vertexPath, const base::State *actualStart,
                                               const base::State *actualGoal, og::PathGeometric &compoundSolution,
                                               std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "convertVertexPathToStatePath()");

  // Ensure the input path is not empty
  if (!vertexPath.size())
    return false;

  // Add original start
  compoundSolution.append(actualStart);

  // Error check that no consequtive verticies are the same
  if (verbose_)
  {
    std::stringstream o;
    for (std::size_t i = vertexPath.size(); i > 0; --i)
      o << vertexPath[i - 1] << ", ";
    std::cout << "o.str(): " << o.str() << std::endl;
    std::cout << "vertexPath.size(): " << vertexPath.size() << std::endl;
    BOLT_DEBUG(indent, true, "BoltPlanner.Vertices: " << o.str());
  }

  // Reverse the vertexPath and convert to state path
  for (std::size_t i = vertexPath.size(); i > 0; --i)
  {
    compoundSolution.append(taskGraph_->getCompoundState(vertexPath[i - 1]));

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

      TaskEdge edge = boost::edge(vertexPath[i - 1], vertexPath[i - 2], taskGraph_->getGraph()).first;

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
  if (actualGoal != taskGraph_->getCompoundState(vertexPath.front()))
  {
    compoundSolution.append(actualGoal);
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

bool BoltPlanner::simplifyTaskPath(og::PathGeometric &compoundPath, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, true, "BoltPlanner: simplifyTaskPath()");

  time::point simplifyStart = time::now();
  std::size_t origNumStates = compoundPath.getStateCount();

  // Number of levels
  const std::size_t NUM_LEVELS = 3;

  // Create three path segments
  std::vector<og::PathGeometric> modelPathSegments;
  for (int segmentLevel = 0; segmentLevel < int(NUM_LEVELS); ++segmentLevel)
    modelPathSegments.push_back(og::PathGeometric(modelSI_));

  // Create the solution path
  og::PathGeometric compoundSmoothedPath(compoundSI_);

  // Divide the path into different levels
  VertexLevel previousLevel = 0;  // Error check ordering of input path
  std::stringstream o;
  for (std::size_t i = 0; i < compoundPath.getStateCount(); ++i)
  {
    base::State *compoundState = compoundPath.getState(i);
    VertexLevel level = taskGraph_->getTaskLevel(compoundState);

    // Debug
    o << level << ", ";
    assert(level < NUM_LEVELS);

    modelPathSegments[level].append(taskGraph_->getModelBasedState(compoundState));

    if (previousLevel > level)  // Error check ordering of input path
      throw Exception(name_, "Level increasing in wrong order");
    previousLevel = level;
  }
  BOLT_DEBUG(indent, verbose_, "Path levels: " << o.str());

  // Add the start and end vertex of the Cartesian path into the respective freespace paths
  if (modelPathSegments[1].getStateCount() >= 2)
  {
    // For checking for errors
    std::size_t seg0Size = modelPathSegments[0].getStateCount();
    std::size_t seg1Size = modelPathSegments[1].getStateCount();
    std::size_t seg2Size = modelPathSegments[2].getStateCount();

    // Move first state
    modelPathSegments[0].append(modelPathSegments[1].getStates().front());
    modelPathSegments[1].getStates().erase(modelPathSegments[1].getStates().begin());

    // Move last state
    modelPathSegments[2].prepend(modelPathSegments[1].getStates().back());
    modelPathSegments[1].getStates().pop_back();

    // Check the operations were correct
    BOLT_ASSERT(seg0Size + 1 == modelPathSegments[0].getStateCount(), "Invalid size of pathSegement after "
                                                                      "rearrangment");
    BOLT_ASSERT(seg1Size - 2 == modelPathSegments[1].getStateCount(), "Invalid size of pathSegement after "
                                                                      "rearrangment");
    BOLT_ASSERT(seg2Size + 1 == modelPathSegments[2].getStateCount(), "Invalid size of pathSegement after "
                                                                      "rearrangment");
  }
  else
  {
    BOLT_WARN(indent, true, "The Cartesian path segement 1 has only " << modelPathSegments[1].getStateCount() << " stat"
                                                                                                                 "es");
  }

  // Smooth the freespace paths
  path_simplifier_->simplifyMax(modelPathSegments[0]);
  path_simplifier_->simplifyMax(modelPathSegments[2]);

  // Interpolate the freespace paths but not the cartesian path
  modelPathSegments[0].interpolate();
  modelPathSegments[2].interpolate();

  // Combine the path segments back together
  for (int segmentLevel = 0; segmentLevel < int(NUM_LEVELS); ++segmentLevel)
  {
    for (std::size_t i = 0; i < modelPathSegments[segmentLevel].getStateCount(); ++i)
    {
      // This state is not compound
      base::State *modelState = modelPathSegments[segmentLevel].getState(i);
      base::State *compoundState = taskGraph_->createCompoundState(modelState, segmentLevel, indent);

      // Debug
      if (visualizeEachSolutionStep_)
      {
        visual_->viz6()->state(modelState, tools::ROBOT, tools::DEFAULT, 0);
        visual_->waitForUserFeedback("next solution step");
      }

      // Check for repeated states
      if (i > 0)
      {
        if (modelSI_->equalStates(modelPathSegments[segmentLevel].getState(i - 1),
                                  modelPathSegments[segmentLevel].getState(i)))
        {
          BOLT_ERROR(indent, "Repeated states at " << i << " on level " << segmentLevel);
        }
      }

      // Add to solution path
      compoundSmoothedPath.append(compoundState);
    }
  }

  // Replace the input path with the new smoothed path
  compoundPath = compoundSmoothedPath;

  double simplifyTime = time::seconds(time::now() - simplifyStart);

  int diff = origNumStates - compoundPath.getStateCount();
  BOLT_DEBUG(indent, verbose_, "BoltPlanner: Path simplification took " << simplifyTime << " seconds and removed "
                                                                        << diff << " states");

  return true;
}

// This is used to check connectivity of graph
bool BoltPlanner::canConnect(const base::State *randomState, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "canConnect()");

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
    const base::State *s2 = taskGraph_->getCompoundState(nearState);

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
        unsigned int count = compoundSI_->getStateSpace()->validSegmentCount(s1, s2);
        // std::cout << "count: " << count << std::endl;

        bool endpoints = false;
        bool alloc = true;
        compoundSI_->getMotionStates(s1, s2, states, count, endpoints, alloc);
        // std::cout << "state size: " << states.size() << std::endl;

        for (base::State *interState : states)
        {
          // Check if our planner is out of time
          if (ptc)
          {
            BOLT_DEBUG(indent, verbose_, "Quit requested");
            return false;
          }

          if (!compoundSI_->isValid(interState))
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
  const base::State *from = taskGraph_->getModelBasedState(fromVertex);
  const base::State *to = taskGraph_->getModelBasedState(toVertex);

  // Line
  visual_->viz5()->deleteAllMarkers();
  visual_->viz5()->edge(from, to, tools::MEDIUM, tools::RED);
  visual_->viz5()->trigger();

  // Robot states
  visual_->viz4()->state(from, tools::ROBOT, tools::RED, 0);
  visual_->viz5()->state(to, tools::ROBOT, tools::RED, 0);

  visual_->waitForUserFeedback("collision on edge from viz4 to viz5");
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
