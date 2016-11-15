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
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/util/Console.h>
#include <ompl/base/goals/GoalStates.h>

// Bolt
#include <bolt_core/BoltPlanner.h>
#include <bolt_core/SparseCriteria.h>

#include <thread>

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
  : base::Planner(modelSI, "BoltPlanner")
  , modelSI_(modelSI)
  , compoundSI_(compoundSI)
  , taskGraph_(taskGraph)
  , visual_(visual)
{
  specs_.approximateSolutions = false;
  specs_.directed = false;

  // Note that the path simplifier operates in the model_based_state_space, not the compound space
  path_simplifier_ = std::make_shared<geometric::PathSimplifier>(modelSI_, base::GoalPtr(), visual_);

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

  maxAttempt_ = 0;
}

void BoltPlanner::setup(void)
{
  Planner::setup();

  // Choose sampler based on clearance
  std::size_t indent = 0;
  sampler_ =
      taskGraph_->getSparseGraph()->getSampler(modelSI_, taskGraph_->getSparseGraph()->getObstacleClearance(), indent);
}

base::PlannerStatus BoltPlanner::solve(Termination &ptc)
{
  std::size_t indent = 0;
  BOLT_FUNC(verbose_, "solve()");

  // Check if the database is empty
  if (taskGraph_->isEmpty())
  {
    BOLT_DEBUG(true, "Task experience database is empty so unable to run BoltPlanner algorithm.");
    return base::PlannerStatus::ABORT;
  }

  // Solve -------------------------------------------------
  if (!solveMultiAttempt(ptc, indent))
  {
    return base::PlannerStatus::ABORT;
  }

  // Save solution
  double approximateDifference = -1;
  bool approximate = false;
  pdef_->addSolutionPath(origModelSolPath_, approximate, approximateDifference, getName());
  bool solved = true;

  BOLT_DEBUG(verbose_, "Finished BoltPlanner.solve()");
  return base::PlannerStatus(solved, approximate);
}

bool BoltPlanner::solve(const base::State *start, const base::State *goal, Termination &ptc, std::size_t indent)
{
  // Check if the database is empty
  if (taskGraph_->isEmpty())
  {
    BOLT_DEBUG(true, "Task experience database is empty so unable to run BoltPlanner algorithm.");
    return base::PlannerStatus::ABORT;
  }

  const std::size_t attempt = 1;
  const std::size_t maxAttempts = 1;
  return solveSingleGoal(start, goal, attempt, maxAttempts, ptc, indent);
}

// bool BoltPlanner::solveThreadLayer(base::CompoundState *startState, base::CompoundState *goalState, Termination &ptc,
//                                    std::size_t indent)
// {
//   // Mark the solution as not found so the thread knows to continue
//   solutionFound_ = false;

//   // Create thread for sampling
//   BOLT_INFO(true, "Using sampling thread");
//   sThread_ = std::thread([this, &startState, &goalState, &ptc, &indent]
//                          {
//                            samplingThread(startState, goalState, ptc, indent);
//                          });

//   bool result = solve(startState, goalState, ptc, indent);

//   // Ensure thread is ceased before exiting
//   // Mark the solution as found so the thread knows to stop
//   solutionFound_ = true;

//   BOLT_DEBUG(true, "Joining sampling thread");
//   if (sThread_.joinable())
//     sThread_.join();

//   return result;
// }

// // Visualize
// if (visualizeRawTrajectory_)
//   visualizeRaw(indent);

// // Create new PathGeometric that we can modify by smoothing
// smoothedCompoundSolPath_ = std::make_shared<geometric::PathGeometric>(*origCompoundSolPath_);

// // Smooth the result
// if (taskGraph_->taskPlanningEnabled())
//   simplifyTaskPath(smoothedCompoundSolPath_, ptc, indent);
// else
//   simplifyNonTaskPath(smoothedCompoundSolPath_, ptc, indent);

// // Convert solution back to joint trajectory only (no discrete component)
// smoothedModelSolPath_.reset();
// smoothedModelSolPath_ = taskGraph_->convertPathToNonCompound(smoothedCompoundSolPath_);

// // Visualize
// if (visualizeSmoothTrajectory_)
//   visualizeSmoothed(indent);

bool BoltPlanner::solveMultiAttempt(Termination &ptc, std::size_t indent)
{
  // Attempt to connect to graph x times, because if it fails we start adding samples
  std::size_t attempt = 0;
  while (attempt < maxAttempt_ || maxAttempt_ == 0)
  {
    attempt++;
    BOLT_DEBUG(verbose_, "solveMultiAttempt() attempt " << attempt);

    // Solve -------------------------------------------------
    if (solveMultiGoal(attempt, maxAttempt_, ptc, indent))
    {
      return true;  // done solving
    }

    BOLT_WARN(verbose_, "solveMultiAttempt(): BoltPlanner returned FALSE for getPathOnGraph(), attempt " << attempt);

    if (terminationRequested(ptc, indent))  // check PlannerTerminationCondition
      return false;

    // Sample
    // if (!useSamplingThread_ && maxAttempt_ != 1)
    // {
    //   const std::size_t attempts = 500 / 3.0;
    //   addSamples(taskGraph_->getModelBasedState(start), attempts, threadID, ptc, indent);
    //   addSamples(taskGraph_->getModelBasedState(goal), attempts, threadID, ptc, indent);
    //   addSamples(NULL, attempts, threadID, ptc, indent);  // do general sampling
    // }
  }  // while

  return false;
}

bool BoltPlanner::solveMultiGoal(std::size_t attempt, std::size_t maxAttempts, Termination &ptc, std::size_t indent)
{
  // Restart the Planner Input States so that the first start and goal state can be fetched
  pis_.restart();  // PlannerInputStates

  // Get a single start state
  const base::State *start = pis_.nextStart();
  if (start == nullptr)
  {
    BOLT_ERROR("No start state found");
    return false;
  }

  std::shared_ptr<ob::GoalStates> goals  = std::dynamic_pointer_cast<ob::GoalStates>(pdef_->getGoal());

  // Get multiple goal states
  const base::State *goal;
  for (std::size_t i = 0; i < goals->getStateCount(); ++i)
  {
    goal = goals->getState(i);

    if (goal == nullptr)
    {
      BOLT_ERROR("No goal state found");
      return false;
    }

    BOLT_DEBUG(true, "Solving with goal " << i);

    // Solve -------------------------------------------------
    if (solveSingleGoal(start, goal, attempt, maxAttempts, ptc, indent))
    {
      BOLT_DEBUG(verbose_, "Solved using goal " << i);
      return true;  // solved
    }
  }  // for

  return false;  // not solved
}

bool BoltPlanner::solveSingleGoal(const base::State *start, const base::State *goal, std::size_t attempt,
                                  std::size_t maxAttempts, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(verbose_, "solveSingleGoal()");

  // Visualize start vertex
  if (visualizeStartGoal_)
  {
    //visual_->viz4()->state(start, tools::ROBOT, tools::ORANGE);
    visual_->viz6()->state(goal, tools::ROBOT, tools::YELLOW);
  }

  // TODO: allocate this memory once for entire class
  base::State *startState = modelSI_->getStateSpace()->cloneState(start);  // PlannerInputStates
  base::State *goalState = modelSI_->getStateSpace()->cloneState(goal);

  // Convert start and goal to compound states
  int level0 = 0;
  int level2 = 0;  // regular single-task planning
  if (taskGraph_->taskPlanningEnabled())
    level2 = 2;

  // These compound states now own the startState/goalState
  base::CompoundState *startStateCompound = taskGraph_->createCompoundState(startState, level0, indent);
  base::CompoundState *goalStateCompound = taskGraph_->createCompoundState(goalState, level2, indent);

  // Create solution path if necessary
  origCompoundSolPath_ = std::make_shared<og::PathGeometric>(compoundSI_);

  // Solve -------------------------------------------------
  bool result =
      getPathOffGraph(startStateCompound, goalStateCompound, origCompoundSolPath_, attempt, maxAttempts, ptc, indent);

  // Free states
  compoundSI_->getStateSpace()->freeState(startStateCompound);
  compoundSI_->getStateSpace()->freeState(goalStateCompound);

  if (!result)
  {
    BOLT_WARN(!ptc, "solveSingleGoal() No solution found");
    return false;
  }

  BOLT_DEBUG(verbose_, "solve() found a solution of size " << origCompoundSolPath_->getStateCount());

  // All save trajectories should be at least 1 state long, then we append the start and goal states, for min of 3
  BOLT_ASSERT(origCompoundSolPath_->getStateCount() >= 3, "Trajectory too short");

  // Convert orginal ModelBasedStateSpace
  origModelSolPath_ = taskGraph_->convertPathToNonCompound(origCompoundSolPath_);

  return true;
}

bool BoltPlanner::getPathOffGraph(const base::CompoundState *start, const base::CompoundState *goal,
                                  og::PathGeometricPtr compoundSolution, std::size_t attempt, std::size_t maxAttempts,
                                  Termination &ptc, std::size_t indent)
{
  // Get neighbors near start and goal. Note: potentially they are not *visible* - will test for this later

  // Start
  int level = taskGraph_->getTaskLevel(start);
  BOLT_DEBUG(verbose_, "Looking for a node near the problem start on level " << level);
  if (!findGraphNeighbors(start, startVertexCandidateNeighbors_, level, attempt, maxAttempts, ptc, indent))
  {
    BOLT_DEBUG(verbose_, "No graph neighbors found for start");
    return false;
  }
  // BOLT_DEBUG(verbose_, "Found " << startVertexCandidateNeighbors_.size() << " nodes near start");

  if (terminationRequested(ptc, indent))  // check PlannerTerminationCondition
    return false;

  // Goal
  level = taskGraph_->getTaskLevel(goal);
  BOLT_DEBUG(vNearestNeighbor_, "Looking for a node near the problem goal on level " << level);
  if (!findGraphNeighbors(goal, goalVertexCandidateNeighbors_, level, attempt, maxAttempts, ptc, indent))
  {
    BOLT_DEBUG(vNearestNeighbor_, "No graph neighbors found for goal");
    return false;
  }
  BOLT_DEBUG(vNearestNeighbor_, "Found " << goalVertexCandidateNeighbors_.size() << " nodes near goal");

  if (terminationRequested(ptc, indent))  // check PlannerTerminationCondition
    return false;

  // Get paths between start and goal
  const bool debug = false;
  bool result = getPathOnGraph(startVertexCandidateNeighbors_, goalVertexCandidateNeighbors_, start, goal,
                               compoundSolution, ptc, debug, indent);

  // Error check
  if (result)
  {
    // All save trajectories should be at least 1 state long, then we append the start and goal states, for min of 3
    BOLT_ASSERT(compoundSolution->getStateCount() >= 3, "Compound solution has " << compoundSolution->getStateCount()
                                                                                 << " states, should be at least 3");

    return true;
  }

  return false;
}

bool BoltPlanner::getPathOnGraph(const std::vector<TaskVertex> &candidateStarts,
                                 const std::vector<TaskVertex> &candidateGoals, const base::CompoundState *actualStart,
                                 const base::CompoundState *actualGoal, og::PathGeometricPtr compoundSolution,
                                 Termination &ptc, bool debug, std::size_t indent)
{
  BOLT_FUNC(verbose_, "getPathOnGraph()");

  bool foundValidStart = false;
  bool foundValidGoal = false;

  // Try every combination of nearby start and goal pairs
  for (TaskVertex startVertex : candidateStarts)
  {
    if (terminationRequested(ptc, indent))  // check PlannerTerminationCondition
      return false;

    // Check if this start is visible from the actual start
    if (!taskGraph_->checkMotion(actualStart, taskGraph_->getCompoundState(startVertex)))
    {
      BOLT_WARN(verbose_, "getPathOnGraph() Found start candidate that is not visible to nearby vertex "
                              << startVertex);

      if (debug)
      {
        // visual_->viz4()->state(taskGraph_->getModelBasedState(startVertex), tools::LARGE, tools::RED);
        visual_->viz4()->state(taskGraph_->getModelBasedState(startVertex), tools::ROBOT, tools::RED);
        visual_->viz5()->state(taskGraph_->getModelBasedState(actualStart), tools::ROBOT, tools::LIME_GREEN);

        visual_->viz4()->edge(taskGraph_->getModelBasedState(actualStart), taskGraph_->getModelBasedState(startVertex),
                              tools::MEDIUM, tools::BLACK);
        visual_->viz4()->trigger();
        // usleep(0.1 * 1000000);
        visual_->prompt("not visible");
      }
      continue;  // this is actually not visible
    }
    foundValidStart = true;

    for (TaskVertex goalVertex : candidateGoals)
    {
      BOLT_DEBUG(verbose_, "getPathOnGraph() Planning from candidate start/goal pair "
                               << actualGoal << " to " << taskGraph_->getCompoundState(goalVertex));

      if (terminationRequested(ptc, indent))
        return false;

      // visual_->viz6()->state(taskGraph_->getModelBasedState(goalVertex), tools::ROBOT, tools::LIME_GREEN);
      // visual_->prompt("nearby goal");

      // Check if this goal is visible from the actual goal
      if (!taskGraph_->checkMotion(actualGoal, taskGraph_->getCompoundState(goalVertex)))
      {
        BOLT_WARN(verbose_, "getPathOnGraph() Found goal candidate that is not visible on vertex " << goalVertex);

        if (visualizeStartGoalUnconnected_)
          visualizeBadEdge(actualGoal, taskGraph_->getCompoundState(goalVertex));

        continue;  // this is actually not visible
      }
      foundValidGoal = true;

      // Repeatidly search through graph for connection then check for collisions then repeat
      if (onGraphSearch(startVertex, goalVertex, actualStart, actualGoal, compoundSolution, ptc, indent + 2))
      {
        // Found a path
        return true;
      }
      else
      {
        // Did not find a path
        BOLT_DEBUG(verbose_, "getPathOnGraph() Did not find a path, looking for other start/goal combinations");
      }
    }  // foreach
  }    // foreach

  if (foundValidStart && foundValidGoal)
  {
    BOLT_WARN(verbose_ || true, "getPathOnGraph() Both a valid start and goal were found but still no path found");
  }
  else if (foundValidStart && !foundValidGoal)
  {
    BOLT_WARN(verbose_ || true, "getPathOnGraph() Unable to connect GOAL state to graph");
  }
  else
  {
    BOLT_WARN(verbose_ || true, "getPathOnGraph() Unable to connect START state to graph");
  }

  // Feedback on growth of graph
  // BOLT_DEBUG(true, "SparseGraph edges: " << taskGraph_->getSparseGraph()->getNumEdges()
  //                                        << " vertices: " << taskGraph_->getSparseGraph()->getNumVertices());
  // BOLT_DEBUG(true, "TaskGraph   edges: " << taskGraph_->getNumEdges() << " vertices: " <<
  // taskGraph_->getNumVertices());

  return false;
}

bool BoltPlanner::onGraphSearch(const TaskVertex &startVertex, const TaskVertex &goalVertex,
                                const base::CompoundState *actualStart, const base::CompoundState *actualGoal,
                                og::PathGeometricPtr compoundSolution, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(verbose_, "onGraphSearch()");

  // Vector to store candidate paths in before they are converted to PathPtrs
  std::vector<TaskVertex> vertexPath;
  double distance;  // resulting path distance

  // Make sure that the start and goal aren't so close together that they find the same vertex
  if (startVertex == goalVertex)
  {
    BOLT_DEBUG(verbose_, "    Start equals goal, creating simple solution ");
    visual_->prompt("    Start equals goal, creating simple solution ");

    // There are only three verticies in this path - start, middle, goal
    vertexPath.push_back(startVertex);

    convertVertexPathToStatePath(vertexPath, actualStart, actualGoal, compoundSolution, indent);
    return true;
  }

#ifndef NDEBUG
  // Error check all states are non-nullptr
  assert(actualStart);
  assert(actualGoal);
  assert(taskGraph_->getCompoundState(startVertex));
  assert(taskGraph_->getCompoundState(goalVertex));
#endif

  // Keep looking for paths between chosen start and goal until one is found that is valid,
  // or no further paths can be found between them because of disabled edges
  // this is necessary for lazy collision checking i.e. rerun after marking invalid edges we found
  while (true)
  {
    if (terminationRequested(ptc, indent))  // check PlannerTerminationCondition
      return false;

    // attempt to find a solution from start to goal
    // graphMutex_.lock();
    bool result = taskGraph_->astarSearch(startVertex, goalVertex, vertexPath, distance, indent);
    // graphMutex_.unlock()

    if (!result)
    {
      BOLT_WARN(verbose_, "Unable to construct solution between start and goal using astar");

      // no path found what so ever
      return false;
    }

    if (terminationRequested(ptc, indent))  // check PlannerTerminationCondition
      return false;

    // Check if all the points in the potential solution are valid
    bool result2 = lazyCollisionCheck(vertexPath, ptc, indent);
    if (result2)
    {
      BOLT_DEBUG(verbose_, "Lazy collision check returned valid ");

      if (terminationRequested(ptc, indent))  // check PlannerTerminationCondition
        return false;

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
  BOLT_FUNC(verbose_, "lazyCollisionCheck() path of size " << vertexPath.size());

  bool hasInvalidEdges = false;

  // Initialize
  TaskVertex fromVertex = vertexPath[0];
  TaskVertex toVertex;

  // Loop through every pair of states and make sure path is valid.
  for (std::size_t toID = 1; toID < vertexPath.size(); ++toID)
  {
    // Increment location on path
    toVertex = vertexPath[toID];

    TaskEdge edge = boost::edge(fromVertex, toVertex, taskGraph_->getGraph()).first;
    int collision_state = taskGraph_->getGraphNonConst()[edge].collision_state_;

    // Has this edge already been checked before?
    if (collision_state == NOT_CHECKED)
    {
      // TODO - is checking edges sufficient, or do we also need to check vertices? I think its fine.
      // Check path between states
      const base::State *fromState = taskGraph_->getModelBasedState(fromVertex);
      const base::State *toState = taskGraph_->getModelBasedState(toVertex);

      if (terminationRequested(ptc, indent))  // check PlannerTerminationCondition
        return false;

      bool result = modelSI_->getMotionValidator()->checkMotion(fromState, toState);

      if (!result)
      {
        BOLT_MAGENTA(vCollisionCheck_, "LAZY CHECK: disabling edge from vertex " << fromVertex << " to " << toVertex);

        // Disable edge
        // Note: benchmarked this method vs. taskGraph_->removeEdge(edge, indent); and from 20 trials each
        // I found that marking invalid is 0.1 s faster for average problem
        taskGraph_->getGraphNonConst()[edge].collision_state_ = IN_COLLISION;

        // Remember that this path is no longer valid, but keep checking remainder of path edges
        hasInvalidEdges = true;

#ifndef NDEBUG
        if (visualizeLazyCollisionCheck_)
          visualizeBadEdge(fromVertex, toVertex);
#endif
      }
      else
      {
        // Mark edge as free so we no longer need to check for collision
        taskGraph_->getGraphNonConst()[edge].collision_state_ = FREE;
      }
    }
    else if (collision_state == IN_COLLISION)
    {
      // Remember that this path is no longer valid, but keep checking remainder of path edges
      hasInvalidEdges = true;

      if (visualizeLazyCollisionCheck_)
        visualizeBadEdge(fromVertex, toVertex);

      BOLT_ERROR("Somehow an edge " << edge << " was found that is already in collision before lazy "
                                               "collision checking");
    }  // if in collision

    // switch vertex focus
    fromVertex = toVertex;
  }  // for

  // Only return true if nothing was found invalid
  return !hasInvalidEdges;
}

bool BoltPlanner::findGraphNeighbors(const base::CompoundState *state, std::vector<TaskVertex> &neighbors,
                                     int requiredLevel, std::size_t attempt, std::size_t maxAttempts, Termination &ptc,
                                     std::size_t indent)
{
  bool useRadius = false;
  if (maxAttempts == 1)  // if doing only one attempt, search by radius
    useRadius = true;

  const double sparseDeltaFractionSecondary = 0.1;
  const double radius = sparseDeltaFractionSecondary * modelSI_->getMaximumExtent();

  BOLT_ASSERT(attempt > 0, "Attempt is zero");

  // Increase the number of neighbors we search through with each attempt
  const std::size_t &numVertices = taskGraph_->getNumVertices();
  static const double FRACTION_TOTAL_VERTICES = 0.02;  // 2%
  std::size_t knearest = std::max(std::size_t(FRACTION_TOTAL_VERTICES * numVertices * attempt), (std::size_t)20);

  if (useRadius)
    BOLT_FUNC(vNearestNeighbor_ || true, "findGraphNeighbors() search radius " << radius);
  else
    BOLT_FUNC(vNearestNeighbor_ || true, "findGraphNeighbors() search k " << knearest);

  BOLT_ASSERT(requiredLevel == 0 || requiredLevel == 2, "Wrong required level");

  // Reset
  neighbors.clear();

  // Setup search by getting a non-const version of the focused state
  const std::size_t threadID = 0;

  // TODO avoid this memory allocation but keeping as member variable
  const base::State *castedState = state->as<base::State>();
  base::CompoundState *stateCopy = compoundSI_->cloneState(castedState)->as<base::CompoundState>();

  // Search
  taskGraph_->getCompoundQueryStateNonConst(taskGraph_->getQueryVertices()[threadID]) = stateCopy;
  if (useRadius)
    taskGraph_->getNN()->nearestR(taskGraph_->getQueryVertices()[threadID], radius, neighbors);
  else
    taskGraph_->getNN()->nearestK(taskGraph_->getQueryVertices()[threadID], knearest, neighbors);
  taskGraph_->getCompoundQueryStateNonConst(taskGraph_->getQueryVertices()[threadID]) = nullptr;

  if (terminationRequested(ptc, indent))  // check PlannerTerminationCondition
    return false;

  // Convert our list of neighbors to the proper level
  if (requiredLevel == 2)
  {
    BOLT_DEBUG(vNearestNeighbor_, "Converting vector of level 0 neighbors to level 2 neighbors");

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
  BOLT_FUNC(verbose_, "convertVertexPathToStatePath()");

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
        BOLT_ERROR("Found repeated vertices " << vertexPath[i - 1] << " to " << vertexPath[i - 2] << " from index "
                                              << i);
        exit(-1);
      }

      TaskEdge edge = boost::edge(vertexPath[i - 1], vertexPath[i - 2], taskGraph_->getGraph()).first;

      // Check if any edges in path are not free (then it an approximate path)
      if (taskGraph_->getGraphNonConst()[edge].collision_state_ == IN_COLLISION)
      {
        BOLT_ERROR("Found invalid edge / approximate solution - how did this happen?");
      }
      else if (taskGraph_->getGraphNonConst()[edge].collision_state_ == NOT_CHECKED)
      {
        BOLT_ERROR("A chosen path has an edge " << edge << " that has not been checked for collision. This "
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
  BOLT_FUNC(true, "simplifyNonTaskPath()");

  // Stats
  time::point simplifyStart = time::now();
  std::size_t origNumStates = compoundPath->getStateCount();

  geometric::PathGeometricPtr modelPath;

  // Do until a positive path length is found
  while (!ptc)
  {
    // Convert to ModelBasedStateSpace
    modelPath = taskGraph_->convertPathToNonCompound(compoundPath);
    double origLength = modelPath->length();

    // Debug
    // visual_->viz5()->deleteAllMarkers();
    // visual_->viz5()->path(modelPath.get(), tools::LARGE, tools::BLACK, tools::BLUE);
    // visual_->viz5()->trigger();
    // visual_->prompt("raw");

    // Smooth
    if (smoothingEnabled_)
      path_simplifier_->simplifyMax(*modelPath);
    else
      BOLT_WARN(true, "Smoothing not enabled");
    double simplifyTime = time::seconds(time::now() - simplifyStart);

    // Feedback
    int statesDiff = origNumStates - modelPath->getStateCount();
    double length = modelPath->length();
    double lengthDiff = origLength - length;
    BOLT_DEBUG(true, "Path simplification took " << simplifyTime << " seconds and removed " << statesDiff
                                                 << " states. Path length (" << length << ") was decreased by "
                                                 << lengthDiff);

    if (lengthDiff < 0)
    {
      BOLT_ERROR("Path simplification increased path length!!");
      visual_->viz6()->deleteAllMarkers();
      visual_->viz6()->path(modelPath.get(), tools::LARGE, tools::RED, tools::BLUE);
      visual_->viz6()->trigger();
      visual_->prompt("bad");
    }
    else
    {
      // visual_->viz6()->deleteAllMarkers();
      // visual_->viz6()->path(modelPath.get(), tools::LARGE, tools::BLACK, tools::BLUE);
      // visual_->viz6()->trigger();
      // visual_->prompt("good");

      break;  // stop looping
    }
  }

  // Interpolate
  if (false)
  {
    origNumStates = modelPath->getStateCount();
    modelPath->interpolate();
    BOLT_DEBUG(true, "Interpolation added: " << modelPath->getStateCount() - int(origNumStates) << " states");
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
  BOLT_FUNC(true, "simplifyTaskPath()");

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
      BOLT_ERROR("Level " << previousLevel << " increasing in wrong order to " << level
                          << ". Path levels: " << o.str());

      visual_->viz6()->state(taskGraph_->getModelBasedState(compoundState), tools::ROBOT, tools::RED, 0);
      visual_->prompt("prev level");
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
    BOLT_WARN(true, "The Cartesian path segement 1 has only " << smoothedModelSolSegments_[1]->getStateCount()
                                                              << " states");
  }

  // Loop through two numbers [0,2]
  for (std::size_t i = 0; i < 3; i += 2)
  {
    // Smooth the freespace paths
    if (smoothingEnabled_)
      path_simplifier_->simplifyMax(*smoothedModelSolSegments_[i]);
    else
      BOLT_WARN(true, "Smoothing not enabled");

    // Interpolate the freespace paths but not the cartesian path
    std::size_t stateCount = smoothedModelSolSegments_[i]->getStateCount();
    smoothedModelSolSegments_[i]->interpolate();

    BOLT_DEBUG(true, "Interpolation added: " << smoothedModelSolSegments_[i]->getStateCount() - stateCount << " state"
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
        visual_->prompt("next solution step");
      }

      // Check for repeated states
      if (i > 0)
      {
        if (modelSI_->equalStates(smoothedModelSolSegments_[segmentLevel]->getState(i - 1),
                                  smoothedModelSolSegments_[segmentLevel]->getState(i)))
        {
          BOLT_ERROR("Repeated states at " << i << " on level " << segmentLevel);
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
  BOLT_DEBUG(verbose_, "Path simplification took " << simplifyTime << " seconds and removed " << diff << " stat"
                                                                                                         "es");

  return true;
}

// This is used to check connectivity of graph
bool BoltPlanner::canConnect(const base::CompoundState *randomState, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(verbose_, "canConnect()");

  std::vector<TaskVertex> candidateNeighbors;

  // Find neighbors to rand state
  BOLT_DEBUG(verbose_, "Looking for a node near the random state");
  const std::size_t attempt = 1;
  const std::size_t maxAttempts = 1;
  const int level = -1;
  if (!findGraphNeighbors(randomState, candidateNeighbors, level, attempt, maxAttempts, ptc, indent))
  {
    BOLT_DEBUG(verbose_, "No graph neighbors found for randomState");
    return false;
  }
  BOLT_DEBUG(verbose_, "Found " << candidateNeighbors.size() << " nodes near randomState");

  // Try every combination of nearby start and goal pairs
  std::size_t count = 0;
  for (TaskVertex nearState : candidateNeighbors)
  {
    const base::CompoundState *s1 = randomState;
    const base::CompoundState *s2 = taskGraph_->getCompoundState(nearState);

    // Check if this nearState is visible from the random state
    if (!taskGraph_->checkMotion(s1, s2))
    {
      BOLT_WARN(true, "NEIGHBOR " << count++ << " NOT VISIBLE ");

      if (false)
      {
        visual_->viz5()->state(taskGraph_->getModelBasedState(s2), tools::MEDIUM, tools::BLUE);
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
          BOLT_DEBUG(verbose_, "Quit requested");
          return false;
          }

          if (!compoundSI_->isValid(interState))
          {
          visual_->viz5()->state(taskGraph_->getModelBasedState(interState), tools::LARGE, tools::RED);
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
      BOLT_DEBUG(verbose_, "Has connection");
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

  visual_->prompt("collision on edge from viz4 to viz5");
}

void BoltPlanner::addSamples(const base::State *near, std::size_t attempts, std::size_t threadID, Termination &ptc,
                             std::size_t indent)
{
  BOLT_FUNC(verbose_, "addSamples()");

  sampler_->setNrAttempts(1000);

  base::CompoundState *compoundState;
  bool usedState = true;  // flag indicating whether memory needs to be allocated again for compoundState

  std::size_t attempt = 0;
  std::size_t numTimesModifiedGraph = 0;
  while (attempt < attempts || numTimesModifiedGraph == 0)
  {
    BOLT_DEBUG(false, "addSamples() attempt: " << attempt << " out of " << attempts);

    attempt++;
    // check often so that thread does not hold up main process when solution found
    if (solutionFound_ || terminationRequested(ptc, indent))  // check PlannerTerminationCondition
      return;

    if (usedState)
    {
      compoundState = compoundSI_->allocState()->as<base::CompoundState>();
      const VertexLevel level0 = 0;
      taskGraph_->setStateTaskLevel(compoundState, level0);
      usedState = false;
    }
    base::State *jointState = taskGraph_->getModelBasedStateNonConst(compoundState);

    // const double magic_fraction = 0.05;  // TODO
    const double magic_fraction = 0.1;  // TODO
    double distance = attempt / double(attempts) * magic_fraction * taskGraph_->getSparseGraph()->getSparseDelta();
    // double fraction = std::max(0.5, i / double(numAttempts));
    // double distance = magic_fraction * taskGraph_->getSparseGraph()->getSparseDelta();

    if (near)
    {
      if (!sampler_->sampleNear(jointState, near, distance))
      {
        BOLT_ERROR("Unable to find valid sample near state");
        continue;
      }
    }
    else
    {
      if (!sampler_->sample(jointState))
      {
        BOLT_ERROR("Unable to find valid sample");
        continue;
      }
    }

    // check often so that thread does not hold up main process when solution found
    if (solutionFound_ || terminationRequested(ptc, indent))  // check PlannerTerminationCondition
      break;                                                  // must break so memory can be freed

    // Visualize
    if (visualizeSampling_)
    {
      // visual_->viz6()->deleteAllMarkers();
      visual_->viz6()->state(jointState, tools::ROBOT, tools::DEFAULT);

      // if (near)
      //   visual_->viz6()->state(jointState, tools::MEDIUM, tools::CYAN);
      // else // sampling whole space
      // visual_->viz6()->state(jointState, tools::SMALL, tools::BLUE);
    }

    // Add to TaskGraph if it obeys sparse properties
    numTimesModifiedGraph += addSampleSparseCriteria(compoundState, usedState, threadID, ptc, indent);

    if (visualizeSampling_)
    {
      visual_->viz6()->trigger();
    }
  }  // for each sample attempt

  std::cout << "numTimesModifiedGraph: " << numTimesModifiedGraph << std::endl;

  // free state if necessary
  if (!usedState)
    compoundSI_->freeState(compoundState);
}

bool BoltPlanner::addSampleSparseCriteria(base::CompoundState *compoundState, bool &usedState, std::size_t threadID,
                                          Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(vCriteria_, "addSampleSparseCriteria()");

  // Calculate secondary sparse delta (smaller than normal sparse delta)
  double maxExtent = modelSI_->getMaximumExtent();
  // double sparseDelta = maxExtent * taskGraph_->getSparseGraph()->getSparseCriteria()->sparseDeltaFractionSecondary_;
  double sparseDelta = taskGraph_->getSparseGraph()->getSparseCriteria()->getSparseDelta();

  // Helper pointer
  base::State *jointState = taskGraph_->getModelBasedStateNonConst(compoundState);

  // Get neighbors of new state
  std::vector<TaskVertex> graphNeighborhood;
  taskGraph_->getQueryStateNonConst(threadID) = compoundState;
  taskGraph_->getNN()->nearestR(taskGraph_->getQueryVertices(threadID), sparseDelta, graphNeighborhood);
  taskGraph_->getQueryStateNonConst(threadID) = nullptr;

  // check often so that thread does not hold up main process when solution found
  if (solutionFound_ || terminationRequested(ptc, indent))  // check PlannerTerminationCondition
    return false;

  if (visualizeSampling_)
    visual_->viz6()->deleteAllMarkers();  // in preparation for many edges

  // Find all visibile neighbors
  std::vector<TaskVertex> visibleNeighborhood;
  std::size_t count = 0;
  for (const SparseVertex &v2 : graphNeighborhood)
  {
    count++;

    // Stop-gap: assume after checking 100 nearest nodes that this state has no neighbors
    // This saves a lot of computation at the expense of possibly adding too many coverage nodes
    static const std::size_t MAX_NEIGHBOR_CHECK = 100;
    if (count > MAX_NEIGHBOR_CHECK)
      break;

    // Visualize
    if (visualizeSampling_)
    {
      visual_->viz6()->state(taskGraph_->getModelBasedState(v2), tools::MEDIUM, tools::ORANGE);
    }

    if (!secondarySI_->checkMotion(jointState, taskGraph_->getModelBasedState(v2)))
    {
      continue;
    }

    // check often so that thread does not hold up main process when solution found
    if (solutionFound_ || terminationRequested(ptc, indent))  // check PlannerTerminationCondition
      return false;

    // Visualize
    if (visualizeSampling_)
    {
      visual_->viz6()->edge(jointState, taskGraph_->getModelBasedState(v2), tools::MEDIUM, tools::PINK);
    }

    // The two are visible to each other!
    visibleNeighborhood.push_back(v2);

    // We only care about the first two visibile neighbors
    if (visibleNeighborhood.size() == 2)
    {
      BOLT_DEBUG(vCriteria_, "Collision checked: " << count << " motions of " << graphNeighborhood.size() << " nearby "
                                                                                                             "nodes");
      break;
    }

    // Enforce that the two closest nodes must also be visible for an edge to be added
    // But this only applies if at least oen visible neighbor has been found,
    // otherwise we could be finding a coverage node
    if (count > 1)
      break;
  }

  // Criteria 1: add guard (no nearby visibile nodes
  if (visibleNeighborhood.empty())
  {
    // No free paths means we add for coverage
    BOLT_DEBUG(vCriteria_, "Adding node for COVERAGE ");

    // check often so that thread does not hold up main process when solution found
    if (solutionFound_ || terminationRequested(ptc, indent))  // check PlannerTerminationCondition
      return false;

    // graphMutex_.lock();
    taskGraph_->addVertex(compoundState, indent);
    usedState = true;
    // graphMutex_.unlock();

    if (visualizeSampling_)
    {
      visual_->viz6()->state(jointState, tools::ROBOT, tools::LIME_GREEN);
      visual_->prompt("coverage");
    }

    return true;  // modified graph
  }

  // Criteria 2: determine if two closest visible nodes need connecting
  if (visibleNeighborhood.size() == 1)
  {
    BOLT_DEBUG(vCriteria_, "Not enough visibile neighbors (1)");
    return false;
  }

  const SparseVertex &v1 = visibleNeighborhood[0];
  const SparseVertex &v2 = visibleNeighborhood[1];

  // Ensure two closest neighbors don't share an edge
  if (taskGraph_->hasEdge(v1, v2))
  {
    BOLT_DEBUG(vCriteria_, "Two closest two neighbors already share an edge, not connecting them");

    // Sampled state was not used - free it
    return false;
  }

  // Don't add an interface edge if dist between the two verticies on graph are already the minimum in L1 space
  // if (!taskGraph_->checkPathLength(v1, v2, indent))
  // {
  //   visual_->prompt("checkPathLenght");
  //   return false;
  // }

  // check often so that thread does not hold up main process when solution found
  if (solutionFound_ || terminationRequested(ptc, indent))  // check PlannerTerminationCondition
    return false;

  // If they can be directly connected
  if (secondarySI_->checkMotion(taskGraph_->getModelBasedState(v1), taskGraph_->getModelBasedState(v2)))
  {
    BOLT_DEBUG(vCriteria_, "INTERFACE: directly connected nodes");

    // Connect them
    taskGraph_->addEdge(v1, v2, indent);

    if (visualizeSampling_)
    {
      visual_->viz6()->edge(taskGraph_->getModelBasedState(v1), taskGraph_->getModelBasedState(v2), tools::MEDIUM,
                            tools::LIME_GREEN);
      visual_->viz6()->trigger();
      // visual_->prompt("directly added edge");
    }

    return true;  // modified graph
  }

  // check often so that thread does not hold up main process when solution found
  if (solutionFound_ || terminationRequested(ptc, indent))  // check PlannerTerminationCondition
    return false;

  // They cannot be directly connected, so add the new node to the graph, to bridge the interface
  BOLT_DEBUG(vCriteria_, "INTERFACE2: Added new NODE and surrounding edges");
  // graphMutex_.lock();
  TaskVertex newVertex = taskGraph_->addVertex(compoundState, indent);
  usedState = true;
  // graphMutex_.unlock();

  taskGraph_->addEdge(newVertex, v1, indent);
  taskGraph_->addEdge(newVertex, v2, indent);

  if (visualizeSampling_)
  {
    visual_->viz6()->state(jointState, tools::MEDIUM, tools::YELLOW);
    visual_->viz6()->edge(jointState, taskGraph_->getModelBasedState(v1), tools::MEDIUM, tools::LIME_GREEN);
    visual_->viz6()->edge(jointState, taskGraph_->getModelBasedState(v2), tools::MEDIUM, tools::LIME_GREEN);
    visual_->viz6()->trigger();
    // visual_->prompt("added node for interface");
  }

  return true;  // modified graph
}

void BoltPlanner::visualizeRaw(std::size_t indent)
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

void BoltPlanner::visualizeSmoothed(std::size_t indent)
{
  // Show smoothed trajectory
  visual_->viz6()->deleteAllMarkers();
  for (std::size_t i = 0; i < smoothedModelSolSegments_.size(); ++i)
  {
    geometric::PathGeometricPtr modelSolutionSegment = smoothedModelSolSegments_[i];
    if (i == 1)
      visual_->viz6()->path(modelSolutionSegment.get(), tools::MEDIUM, tools::BLACK, tools::PURPLE);
    else
      visual_->viz6()->path(modelSolutionSegment.get(), tools::MEDIUM, tools::BLACK, tools::BLUE);
  }
  visual_->viz6()->trigger();

  if (visualizeWait_)
    visual_->prompt("after visualization");
}

void BoltPlanner::samplingThread(const base::CompoundState *start, const base::CompoundState *goal, Termination &ptc,
                                 std::size_t indent)
{
  BOLT_FUNC(true, "samplingThread()");

  std::size_t threadID = 1;  // TODO: dynamically assign this
  std::size_t attempts = 100;

  while (!solutionFound_ && !ptc)
  {
    BOLT_INFO(true, "Searching around goal");
    addSamples(taskGraph_->getModelBasedState(goal), attempts, threadID, ptc, indent);  // sample around goal

    if (solutionFound_ && !ptc)  // check often so that thread does not hold up main process when solution found
      break;

    BOLT_INFO(true, "Searching around start");
    addSamples(taskGraph_->getModelBasedState(start), attempts, threadID, ptc, indent);  // sample around start

    if (solutionFound_ && !ptc)  // check often so that thread does not hold up main process when solution found
      break;

    BOLT_INFO(true, "Searching around all");
    addSamples(NULL, attempts, threadID, ptc, indent);  // do general sampling
  }                                                     // while search is running

  BOLT_INFO(true, "Sampling thread finished");
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
