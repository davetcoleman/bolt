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
  path_simplifier_.reset(new geometric::PathSimplifier(modelSI_, base::GoalPtr(), visual_));

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

  // Reset datastructures
  origCompoundSolPath_.reset();
  origModelSolPath_.reset();
  smoothedCompoundSolPath_.reset();
  smoothedModelSolPath_.reset();
  smoothedModelSolSegments_.clear();
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
  int level0 = 0;
  int level2;
  if (taskGraph_->taskPlanningEnabled())
    level2 = 2;
  else
    level2 = 0;  // regular single-task planning

  // These compound states now own the startState/goalState
  base::CompoundState *startStateCompound = taskGraph_->createCompoundState(startState, level0, indent);
  base::CompoundState *goalStateCompound = taskGraph_->createCompoundState(goalState, level2, indent);

  base::PlannerStatus result = solve(startStateCompound, goalStateCompound, ptc, indent);

  // Free states
  compoundSI_->getStateSpace()->freeState(startStateCompound);
  compoundSI_->getStateSpace()->freeState(goalStateCompound);

  return result;
}

base::PlannerStatus BoltPlanner::solve(base::CompoundState *startState, base::CompoundState *goalState,
                                       Termination &ptc, std::size_t indent)
{
  // Create solution structure
  origCompoundSolPath_ = std::make_shared<og::PathGeometric>(compoundSI_);

  // Search
  if (!getPathOffGraph(startState, goalState, origCompoundSolPath_, ptc, indent))
  {
    BOLT_WARN(indent, true, "BoltPlanner::solve() No near start or goal found");
    return base::PlannerStatus::ABORT;  // The planner failed to find a solution
  }

  BOLT_DEBUG(indent, verbose_, "getPathOffGraph() found a solution of size " << origCompoundSolPath_->getStateCount());

  // All save trajectories should be at least 1 state long, then we append the start and goal states, for min of 3
  assert(origCompoundSolPath_->getStateCount() >= 3);

  // Convert orginal ModelBasedStateSpace
  origModelSolPath_ = taskGraph_->convertPathToNonCompound(origCompoundSolPath_);

  // Visualize
  visualizeRaw(indent);

  // Create new PathGeometric that we can modify by smoothing
  smoothedCompoundSolPath_.reset(new geometric::PathGeometric(*origCompoundSolPath_));

  // Smooth the result
  if (taskGraph_->taskPlanningEnabled())
    simplifyTaskPath(smoothedCompoundSolPath_, ptc, indent);
  else
    simplifyNonTaskPath(smoothedCompoundSolPath_, ptc, indent);

  // Convert solution back to joint trajectory only (no discrete component)
  smoothedModelSolPath_.reset();
  smoothedModelSolPath_ = taskGraph_->convertPathToNonCompound(smoothedCompoundSolPath_);

  // Visualize
  visualizeSmoothed(indent);

  // Save solution
  // double approximateDifference = -1;
  bool approximate = false;
  // pdef_->addSolutionPath(smoothedModelSolPath_, approximate, approximateDifference, getName());
  bool solved = true;

  BOLT_DEBUG(indent, verbose_, "Finished BoltPlanner.solve()");
  return base::PlannerStatus(solved, approximate);
}

bool BoltPlanner::getPathOffGraph(const base::CompoundState *start, const base::CompoundState *goal,
                                  og::PathGeometricPtr compoundSolution, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "getPathOffGraph()");

  // Attempt to connect to graph x times, because if it fails we start adding samples
  std::size_t maxAttempts = 10;
  std::size_t attempt = 0;
  for (; attempt < maxAttempts; ++attempt)
  {
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    BOLT_DEBUG(indent, true, "Starting getPathOffGraph() attempt " << attempt);
    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;

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
    if (result)
      break;

    BOLT_WARN(indent, true, "getPathOffGraph(): BoltPlanner returned FALSE for getPathOnGraph, attempt " << attempt);

    if (visual_->viz1()->shutdownRequested())
      break;
  }

  // Did we finally get it?
  if (attempt >= maxAttempts)
  {
    return false;
  }

  // All save trajectories should be at least 1 state long, then we append the start and goal states, for min of 3
  assert(compoundSolution->getStateCount() >= 3);

  return true;
}

bool BoltPlanner::getPathOnGraph(const std::vector<TaskVertex> &candidateStarts,
                                 const std::vector<TaskVertex> &candidateGoals, const base::CompoundState *actualStart,
                                 const base::CompoundState *actualGoal, og::PathGeometricPtr compoundSolution,
                                 Termination &ptc, bool debug, bool &feedbackStartFailed, std::size_t indent)
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
      BOLT_WARN(indent, verbose_, "getPathOnGraph() Found start candidate that is not visible to nearby vertex "
                                      << startVertex);

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

    for (TaskVertex goalVertex : candidateGoals)
    {
      BOLT_DEBUG(indent, verbose_, "getPathOnGraph() Planning from candidate start/goal pair "
                                               << actualGoal << " to " << taskGraph_->getCompoundState(goalVertex));

      if (ptc)  // Check if our planner is out of time
      {
        BOLT_DEBUG(indent + 2, verbose_, "getPathOnGraph() interrupted because termination condition is true.");
        return false;
      }

      // Check if this goal is visible from the actual goal
      if (!taskGraph_->checkMotion(actualGoal, taskGraph_->getCompoundState(goalVertex)))
      {
        BOLT_WARN(indent + 2, verbose_, "getPathOnGraph() Found goal candidate that is not visible on vertex "
                                            << goalVertex);

        if (visualizeStartGoalUnconnected_)
          visualizeBadEdge(actualGoal, taskGraph_->getCompoundState(goalVertex));

        continue;  // this is actually not visible
      }
      foundValidGoal = true;

      // Repeatidly search through graph for connection then check for collisions then repeat
      if (onGraphSearch(startVertex, goalVertex, actualStart, actualGoal, compoundSolution, ptc, indent + 2))
      {
        // All save trajectories should be at least 1 state long, then we append the start and goal states, for
        // min of 3
        assert(compoundSolution->getStateCount() >= 3);

        // Found a path
        return true;
      }
      else
      {
        // Did not find a path
        BOLT_DEBUG(indent, verbose_, "getPathOnGraph() Did not find a path, looking for other start/goal "
                                     "combinations ");
      }

      if (visual_->viz1()->shutdownRequested())
        break;
    }  // foreach

    if (visual_->viz1()->shutdownRequested())
      break;
  }  // foreach

  if (foundValidStart && foundValidGoal)
  {
    BOLT_ERROR(indent, "getPathOnGraph() Both a valid start and goal were found but still no path found.");

    // Re-attempt to connect both
    addSamples(taskGraph_->getModelBasedState(actualGoal), indent);
    addSamples(taskGraph_->getModelBasedState(actualStart), indent);
    // addSamples(NULL, indent); // do general sampling
    return false;
  }

  if (foundValidStart && !foundValidGoal)
  {
    BOLT_WARN(indent, true, "getPathOnGraph() Unable to connect GOAL state to graph");
    feedbackStartFailed = false;  // it was the goal state that failed us
    addSamples(taskGraph_->getModelBasedState(actualGoal), indent);
  }
  else
  {
    BOLT_WARN(indent, true, "getPathOnGraph() Unable to connect START state to graph");
    feedbackStartFailed = true;  // the start state failed us
    addSamples(taskGraph_->getModelBasedState(actualStart), indent);
  }

  return false;
}

bool BoltPlanner::onGraphSearch(const TaskVertex &startVertex, const TaskVertex &goalVertex,
                                const base::CompoundState *actualStart, const base::CompoundState *actualGoal,
                                og::PathGeometricPtr compoundSolution, Termination &ptc, std::size_t indent)
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
    // time::point startTime0 = time::now(); // Benchmark
    if (!taskGraph_->astarSearch(startVertex, goalVertex, vertexPath, distance, indent))
    {
      BOLT_WARN(indent, true || verbose_, "Unable to construct solution between start and goal using astar");

      // no path found what so ever
      return false;
    }
    // OMPL_INFORM("astar search took %f seconds", time::seconds(time::now() - startTime0)); // Benchmark

    // Check if all the points in the potential solution are valid

    // time::point startTime1 = time::now(); // Benchmark
    if (lazyCollisionCheck(vertexPath, ptc, indent))
    {
      BOLT_DEBUG(indent, verbose_, "Lazy collision check returned valid ");

      // the path is valid, we are done!
      convertVertexPathToStatePath(vertexPath, actualStart, actualGoal, compoundSolution, indent);
      return true;
    }
    // OMPL_INFORM("collision check took %f seconds", time::seconds(time::now() - startTime1)); // Benchmark

    // else, loop with updated graph that has the invalid edges/states disabled
  }  // end while

  // we never found a valid path
  return false;
}

bool BoltPlanner::lazyCollisionCheck(std::vector<TaskVertex> &vertexPath, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "lazyCollisionCheck() path of size " << vertexPath.size());

  std::size_t origNumEdges = taskGraph_->getNumEdges();

  bool hasInvalidEdges = false;

  // Initialize
  TaskVertex fromVertex = vertexPath[0];
  TaskVertex toVertex;

  // Loop through every pair of states and make sure path is valid.
  for (std::size_t toID = 1; toID < vertexPath.size(); ++toID)
  {
    // Increment location on path
    toVertex = vertexPath[toID];

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

        // Disable edge
        // taskGraph_->getGraphNonConst()[thisEdge].collision_state_ = IN_COLLISION;

        // Remove the edge
        taskGraph_->removeEdge(thisEdge, indent);

        // Remember that this path is no longer valid, but keep checking remainder of path edges
        hasInvalidEdges = true;

        // Check if our planner is out of time - only do this after the slow checkMotion() action has occured to save
        // time
        if (ptc || visual_->viz1()->shutdownRequested())
        {
          BOLT_DEBUG(indent, verbose_, "Lazy collision check function interrupted because termination condition is "
                                       "true.");
          return false;
        }

        // Debug
        if (visualizeLazyCollisionCheck_)
        {
          visual_->waitForUserFeedback("see edge");

          // Path between (from, to) states not valid, disable the edge
          modelSI_->getMotionValidator()->checkMotion(fromState, toState, visual_);

          visualizeBadEdge(fromVertex, toVertex);
        }
      }
      else
      {
        // Mark edge as free so we no longer need to check for collision
        taskGraph_->getGraphNonConst()[thisEdge].collision_state_ = FREE;
      }
    }
    else if (taskGraph_->getGraphNonConst()[thisEdge].collision_state_ == IN_COLLISION)
    {
      // Remember that this path is no longer valid, but keep checking remainder of path edges
      hasInvalidEdges = true;

      if (visualizeLazyCollisionCheck_)
        visualizeBadEdge(fromVertex, toVertex);

      BOLT_ERROR(indent, "Somehow an edge " << thisEdge << " was found that is already in collision before lazy "
                                                           "collision checking");
    }  // if in collision

    // switch vertex focus
    fromVertex = toVertex;
  }  // for

  BOLT_MAGENTA(indent, verbose_, "Removed edges: " << origNumEdges - taskGraph_->getNumEdges()
                                               << " total edges: " << taskGraph_->getNumEdges());

  // Only return true if nothing was found invalid
  return !hasInvalidEdges;
}

bool BoltPlanner::findGraphNeighbors(const base::CompoundState *state, std::vector<TaskVertex> &neighbors,
                                     int requiredLevel, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "findGraphNeighbors()");

  BOLT_ASSERT(requiredLevel == 0 || requiredLevel == 2, "Wrong required level");

  // Reset
  neighbors.clear();

  // Search within double the radius of the sparse delta
  // Technically you should only need to search within 1x, but collisions etc could possibly require looking in
  // a larger region
  // double radius = taskGraph_->getSparseGraph()->getSparseDelta() * 2.0;

  // Setup search by getting a non-const version of the focused state
  const std::size_t threadID = 0;

  // TODO avoid this memory allocation but keeping as member variable
  const base::State *castedState = state->as<base::State>();
  base::CompoundState *stateCopy = compoundSI_->cloneState(castedState)->as<base::CompoundState>();

  // Search
  taskGraph_->getCompoundQueryStateNonConst(taskGraph_->getQueryVertices()[threadID]) = stateCopy;
  // taskGraph_->nn_->nearestR(taskGraph_->getQueryVertices()[threadID], radius, neighbors);
  taskGraph_->getNN()->nearestK(taskGraph_->getQueryVertices()[threadID], kNearestNeighbors_, neighbors);
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

bool BoltPlanner::convertVertexPathToStatePath(std::vector<TaskVertex> &vertexPath,
                                               const base::CompoundState *actualStart,
                                               const base::CompoundState *actualGoal,
                                               og::PathGeometricPtr compoundSolution, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "convertVertexPathToStatePath()");

  // Ensure the input path is not empty
  if (!vertexPath.size())
    return false;

  // Add original start
  compoundSolution->append(actualStart);

  // Error check that no consequtive verticies are the same
  if (verbose_)
  {
    std::stringstream o;
    for (std::size_t i = vertexPath.size(); i > 0; --i)
      o << vertexPath[i - 1] << ", ";
    std::cout << std::string(indent, ' ') << "Verticies: " << o.str() << std::endl;
  }

  // Reverse the vertexPath and convert to state path
  for (std::size_t i = vertexPath.size(); i > 0; --i)
  {
    compoundSolution->append(taskGraph_->getCompoundState(vertexPath[i - 1]));

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
    compoundSolution->append(actualGoal);
  }

  return true;
}

bool BoltPlanner::simplifyNonTaskPath(og::PathGeometricPtr compoundPath, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, true, "simplifyNonTaskPath()");

  // Stats
  time::point simplifyStart = time::now();
  std::size_t origNumStates = compoundPath->getStateCount();

  geometric::PathGeometricPtr modelPath;

  // Do until a positive path length is found
  while (!visual_->viz6()->shutdownRequested())
  {
    // Convert to ModelBasedStateSpace
    modelPath = taskGraph_->convertPathToNonCompound(compoundPath);
    double origLength = modelPath->length();

    // Debug
    // visual_->viz5()->deleteAllMarkers();
    // visual_->viz5()->path(modelPath.get(), tools::LARGE, tools::BLACK, tools::BLUE);
    // visual_->viz5()->trigger();
    // visual_->waitForUserFeedback("raw");

    // Smooth
    if (smoothingEnabled_)
      path_simplifier_->simplifyMax(*modelPath);
    else
      BOLT_WARN(indent, true, "Smoothing not enabled");
    double simplifyTime = time::seconds(time::now() - simplifyStart);

    // Feedback
    int statesDiff = origNumStates - modelPath->getStateCount();
    double length = modelPath->length();
    double lengthDiff = origLength - length;
    BOLT_DEBUG(indent, true, "Path simplification took " << simplifyTime << " seconds and removed " << statesDiff
               << " states. Path length (" << length << ") was decreased by " << lengthDiff);

    if (lengthDiff < 0)
    {
      BOLT_ERROR(indent, "Path simplification increased path length!!");
      visual_->viz6()->deleteAllMarkers();
      visual_->viz6()->path(modelPath.get(), tools::LARGE, tools::RED, tools::BLUE);
      visual_->viz6()->trigger();
      visual_->waitForUserFeedback("bad");
    }
    else
    {
      // visual_->viz6()->deleteAllMarkers();
      // visual_->viz6()->path(modelPath.get(), tools::LARGE, tools::BLACK, tools::BLUE);
      // visual_->viz6()->trigger();
      // visual_->waitForUserFeedback("good");

      break;  // stop looping
    }
  }

  // Interpolate
  if (true)
  {
    origNumStates = modelPath->getStateCount();
    modelPath->interpolate();
    BOLT_DEBUG(indent, true, "Interpolation added: " << modelPath->getStateCount() - int(origNumStates) << " states");
  }

  // Create single path segments
  smoothedModelSolSegments_.clear();
  smoothedModelSolSegments_.push_back(modelPath);

  // Re-create compoundPath
  compoundPath->clear();
  for (std::size_t i = 0; i < modelPath->getStateCount(); ++i)
  {
    int segmentLevel = 0;
    base::State *modelState = modelPath->getState(i);
    base::CompoundState *compoundState = taskGraph_->createCompoundState(modelState, segmentLevel, indent);
    compoundPath->append(compoundState);
  }
  BOLT_ASSERT(compoundPath->getStateCount() == modelPath->getStateCount(), "Non-matching number of states when copying "
                                                                           "between state spaces");

  return true;
}

bool BoltPlanner::simplifyTaskPath(og::PathGeometricPtr compoundPath, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, true, "simplifyTaskPath()");

  time::point simplifyStart = time::now();
  std::size_t origNumStates = compoundPath->getStateCount();

  // Number of levels
  const std::size_t NUM_LEVELS = 3;

  // Create three path segments
  smoothedModelSolSegments_.clear();
  for (int segmentLevel = 0; segmentLevel < int(NUM_LEVELS); ++segmentLevel)
    smoothedModelSolSegments_.push_back(std::make_shared<og::PathGeometric>(modelSI_));

  // Create the solution path
  og::PathGeometricPtr compoundSmoothedPath = std::make_shared<og::PathGeometric>(compoundSI_);

  // Divide the path into different levels
  VertexLevel previousLevel = 0;  // Error check ordering of input path
  std::stringstream o;
  for (std::size_t i = 0; i < compoundPath->getStateCount(); ++i)
  {
    base::CompoundState *compoundState = compoundPath->getState(i)->as<base::CompoundState>();
    VertexLevel level = taskGraph_->getTaskLevel(compoundState);

    // Debug
    o << level << ", ";
    assert(level < NUM_LEVELS);

    smoothedModelSolSegments_[level]->append(taskGraph_->getModelBasedState(compoundState));

    if (previousLevel > level)  // Error check ordering of input path
    {
      BOLT_ERROR(indent, "Level " << previousLevel << " increasing in wrong order to " << level
                                  << ". Path levels: " << o.str());

      visual_->viz6()->state(taskGraph_->getModelBasedState(compoundState), tools::ROBOT, tools::RED, 0);
      visual_->waitForUserFeedback("prev level");
      base::CompoundState *compoundState2 = compoundPath->getState(i - 1)->as<base::CompoundState>();
      visual_->viz6()->state(taskGraph_->getModelBasedState(compoundState2), tools::ROBOT, tools::RED, 0);

      exit(-1);
    }
    previousLevel = level;
  }
  std::cout << ANSI_COLOR_GREEN << std::string(indent, ' ') << "Path levels: " << o.str() << ANSI_COLOR_RESET
            << std::endl;

  // Add the start and end vertex of the Cartesian path into the respective freespace paths
  if (smoothedModelSolSegments_[1]->getStateCount() >= 2)
  {
    // For checking for errors
    std::size_t seg0Size = smoothedModelSolSegments_[0]->getStateCount();
    std::size_t seg1Size = smoothedModelSolSegments_[1]->getStateCount();
    std::size_t seg2Size = smoothedModelSolSegments_[2]->getStateCount();

    // Move first state
    smoothedModelSolSegments_[0]->append(smoothedModelSolSegments_[1]->getStates().front());
    smoothedModelSolSegments_[1]->getStates().erase(smoothedModelSolSegments_[1]->getStates().begin());

    // Move last state
    smoothedModelSolSegments_[2]->prepend(smoothedModelSolSegments_[1]->getStates().back());
    smoothedModelSolSegments_[1]->getStates().pop_back();

    // Check the operations were correct
    BOLT_ASSERT(seg0Size + 1 == smoothedModelSolSegments_[0]->getStateCount(), "Invalid size of pathSegement after "
                                                                               "rearrangment");
    BOLT_ASSERT(seg1Size - 2 == smoothedModelSolSegments_[1]->getStateCount(), "Invalid size of pathSegement after "
                                                                               "rearrangment");
    BOLT_ASSERT(seg2Size + 1 == smoothedModelSolSegments_[2]->getStateCount(), "Invalid size of pathSegement after "
                                                                               "rearrangment");
  }
  else
  {
    BOLT_WARN(indent, true, "The Cartesian path segement 1 has only " << smoothedModelSolSegments_[1]->getStateCount()
                                                                      << " states");
  }

  // Loop through two numbers [0,2]
  for (std::size_t i = 0; i < 3; i += 2)
  {
    // Smooth the freespace paths
    if (smoothingEnabled_)
      path_simplifier_->simplifyMax(*smoothedModelSolSegments_[i]);
    else
      BOLT_WARN(indent, true, "Smoothing not enabled");

    // Interpolate the freespace paths but not the cartesian path
    std::size_t stateCount = smoothedModelSolSegments_[i]->getStateCount();
    smoothedModelSolSegments_[i]->interpolate();

    BOLT_DEBUG(indent, true, "Interpolation added: " << smoothedModelSolSegments_[i]->getStateCount() - stateCount
                                                     << " state"
                                                        "s");
  }

  // Combine the path segments back together
  for (int segmentLevel = 0; segmentLevel < int(NUM_LEVELS); ++segmentLevel)
  {
    for (std::size_t i = 0; i < smoothedModelSolSegments_[segmentLevel]->getStateCount(); ++i)
    {
      // This state is not compound
      base::State *modelState = smoothedModelSolSegments_[segmentLevel]->getState(i);
      base::CompoundState *compoundState = taskGraph_->createCompoundState(modelState, segmentLevel, indent);

      // Debug
      if (visualizeEachSolutionStep_)
      {
        visual_->viz6()->state(modelState, tools::ROBOT, tools::DEFAULT, 0);
        visual_->waitForUserFeedback("next solution step");
      }

      // Check for repeated states
      if (i > 0)
      {
        if (modelSI_->equalStates(smoothedModelSolSegments_[segmentLevel]->getState(i - 1),
                                  smoothedModelSolSegments_[segmentLevel]->getState(i)))
        {
          BOLT_ERROR(indent, "Repeated states at " << i << " on level " << segmentLevel);
        }
      }

      // Add to solution path
      compoundSmoothedPath->append(compoundState);
    }
  }

  // Replace the input path with the new smoothed path
  compoundPath = compoundSmoothedPath;

  double simplifyTime = time::seconds(time::now() - simplifyStart);

  int diff = origNumStates - compoundPath->getStateCount();
  BOLT_DEBUG(indent, verbose_, "Path simplification took " << simplifyTime << " seconds and removed " << diff << " stat"
                                                                                                                 "es");

  return true;
}

// This is used to check connectivity of graph
bool BoltPlanner::canConnect(const base::CompoundState *randomState, Termination &ptc, std::size_t indent)
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
    const base::CompoundState *s1 = randomState;
    const base::CompoundState *s2 = taskGraph_->getCompoundState(nearState);

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
        /*
          std::cout << "checking path " << std::endl;
          std::vector<base::CompoundState *> states;
          unsigned int count = compoundSI_->getStateSpace()->validSegmentCount(s1, s2);
          // std::cout << "count: " << count << std::endl;

          bool endpoints = false;
          bool alloc = true;
          compoundSI_->getMotionStates(s1, s2, states, count, endpoints, alloc);
          // std::cout << "state size: " << states.size() << std::endl;

          for (base::CompoundState *interState : states)
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
          // visual_->viz5()->state(taskGraph_->getModelBasedState(interState), 1, 1); // GREEN
          }
          } // for
        */
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
  const base::State *modelFrom = taskGraph_->getModelBasedState(fromVertex);
  const base::State *modelTo = taskGraph_->getModelBasedState(toVertex);
  visualizeBadEdge(modelFrom, modelTo);
}

void BoltPlanner::visualizeBadEdge(const base::State *modelFrom, const base::State *modelTo)
{
  // Line
  visual_->viz4()->deleteAllMarkers();
  visual_->viz4()->edge(modelFrom, modelTo, tools::MEDIUM, tools::RED);
  visual_->viz4()->trigger();

  // Line again
  visual_->viz5()->deleteAllMarkers();
  visual_->viz5()->edge(modelFrom, modelTo, tools::MEDIUM, tools::RED);
  visual_->viz5()->trigger();

  // Robot states
  visual_->viz4()->state(modelFrom, tools::ROBOT, tools::RED, 0);
  visual_->viz5()->state(modelTo, tools::ROBOT, tools::RED, 0);

  visual_->waitForUserFeedback("collision on edge from viz4 to viz5");
}

void BoltPlanner::addSamples(const base::State *near, std::size_t indent)
{
  BOLT_FUNC(indent, true, "addSamples()");

  // Choose sampler based on clearance
  base::ValidStateSamplerPtr sampler =
      taskGraph_->getSparseGraph()->getSampler(modelSI_, taskGraph_->getSparseGraph()->getObstacleClearance(), indent);
  sampler->setNrAttempts(1000);

  std::size_t numAttempts = 100;
  for (std::size_t i = 0; i < numAttempts; ++i)
  {
    // 0.1 is magic num
    const double magic_fraction = 0.05;  // TODO
    double distance = i / double(numAttempts) * magic_fraction * taskGraph_->getSparseGraph()->getSparseDelta();

    base::State *candidateState = modelSI_->getStateSpace()->allocState();

    if (near)
    {
      if (!sampler->sampleNear(candidateState, near, distance))
        throw Exception(name_, "Unable to find valid sample near state");
    }
    else
    {
      if (!sampler->sample(candidateState))
        throw Exception(name_, "Unable to find valid sample");
    }

    // Visualize
    if (visualizeSampling_)
      visual_->viz6()->state(candidateState, tools::ROBOT, tools::DEFAULT, 1);

    // Add the vertex
    VertexLevel level = 0;
    TaskVertex v1 = taskGraph_->addVertexWithLevel(candidateState, level, indent);

    base::CompoundState *compoundState1 = taskGraph_->getCompoundStateNonConst(v1);

    // Add edges around vertex ----------------------------------------------------

    // Get neighbors
    std::size_t threadID = 0;

    std::vector<TaskVertex> graphNeighborhood;
    taskGraph_->getQueryStateNonConst(threadID) = compoundState1;
    // taskGraph_->getNN()->nearestR(taskGraph_->getQueryVertices(threadID), distance, graphNeighborhood);
    taskGraph_->getNN()->nearestK(taskGraph_->getQueryVertices(threadID), kNearestNeighbors_, graphNeighborhood);
    taskGraph_->getQueryStateNonConst(threadID) = nullptr;

    // if (visualizeSampling_)
    //   visual_->viz6()->deleteAllMarkers(); // in preparation for many edges

    // Now that we got the neighbors from the NN, we must remove any we can't see
    std::size_t invalid = 0;
    for (std::size_t i = 0; i < graphNeighborhood.size(); ++i)
    {
      const TaskVertex &v2 = graphNeighborhood[i];
      base::CompoundState *compoundState2 = taskGraph_->getCompoundStateNonConst(v2);

      // Don't collision check if they are the same state
      if (compoundState1 == compoundState2)
      {
        // std::cout << "Skipping collision checking because same vertex " << std::endl;
        continue;
      }

      // Collision check
      if (true)  // let lazy checking work
        if (!taskGraph_->checkMotion(compoundState1, compoundState2))
        {
          invalid++;
          continue;
        }

      // The two are visible to each other, add edge
      TaskEdge e = taskGraph_->addEdge(v1, v2, indent);

      // Mark edge as free so we no longer need to check for collision
      // taskGraph_->getGraphNonConst()[e].collision_state_ = FREE;

      if (visualizeSampling_)
        visual_->viz6()->edge(taskGraph_->getModelBasedState(v2), candidateState, tools::XXSMALL, tools::ORANGE);
    }  // for each neighbor

    BOLT_INFO(indent, vSampling_, "Sampling " << i << " at distance " << distance << ", removed " << invalid
                                              << " edges out of " << graphNeighborhood.size() << " neighbors");

    if (visualizeSampling_)
    {
      visual_->viz6()->trigger();
      visual_->waitForUserFeedback("next sample");
    }

    if (visual_->viz3()->shutdownRequested())
      break;
  }  // for each sample
}

void BoltPlanner::visualizeRaw(std::size_t indent)
{
  // Show raw trajectory
  if (visualizeRawTrajectory_)
  {
    // Make the chosen path a different color and thickness
    visual_->viz5()->path(origModelSolPath_.get(), tools::MEDIUM, tools::BLUE, tools::BLACK);
    visual_->viz5()->trigger();

    // Don't show raw trajectory twice in larger dimensions
    if (si_->getStateSpace()->getDimension() == 3)
    {
      visual_->viz6()->path(origModelSolPath_.get(), tools::MEDIUM, tools::BLUE, tools::BLACK);
      visual_->viz6()->trigger();
    }
  }
}

void BoltPlanner::visualizeSmoothed(std::size_t indent)
{
  // Show smoothed trajectory
  if (visualizeSmoothTrajectory_)
  {
    visual_->viz6()->deleteAllMarkers();
    for (std::size_t i = 0; i < smoothedModelSolSegments_.size(); ++i)
    {
      geometric::PathGeometricPtr modelSolutionSegment = smoothedModelSolSegments_[i];
      if (i == 1)
        visual_->viz6()->path(modelSolutionSegment.get(), tools::LARGE, tools::BLACK, tools::PURPLE);
      else
        visual_->viz6()->path(modelSolutionSegment.get(), tools::LARGE, tools::BLACK, tools::BLUE);
    }
    visual_->viz6()->trigger();
  }

  // Show robot trajectory
  if (visualizeRobotTrajectory_)
  {
    visual_->viz6()->path(smoothedModelSolPath_.get(), tools::ROBOT, tools::DEFAULT, tools::DEFAULT);
  }

  if (visualizeWait_)
    visual_->waitForUserFeedback("after visualization");
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
