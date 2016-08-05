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
#include <ompl/tools/bolt/BoltPlanner.h>
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
BoltPlanner::BoltPlanner(const base::SpaceInformationPtr &si, const TaskGraphPtr &taskGraph, VisualizerPtr visual)
  : base::Planner(si, "Bolt_Planner"), taskGraph_(taskGraph), visual_(visual)
{
  specs_.approximateSolutions = false;
  specs_.directed = false;

  path_simplifier_.reset(new geometric::PathSimplifier(si_));
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

  bool solved = false;

  // Check if the database is empty
  if (taskGraph_->isEmpty())
  {
    BOLT_DEBUG(indent, verbose_, "Task experience database is empty so unable to run BoltPlanner algorithm.");

    return base::PlannerStatus::ABORT;
  }

  // Restart the Planner Input States so that the first start and goal state can be fetched
  pis_.restart();  // PlannerInputStates

  // Get a single start and goal state
  BOLT_DEBUG(indent, verbose_, "Getting OMPL start and goal state");
  const base::State *startState = pis_.nextStart();  // PlannerInputStates
  const base::State *goalState = pis_.nextGoal(ptc);

  if (startState == nullptr)
  {
    OMPL_ERROR("No start state found");
    return base::PlannerStatus::ABORT;
  }

  if (goalState == nullptr)
  {
    OMPL_ERROR("No goal state found");
    return base::PlannerStatus::ABORT;
  }

  // Error check task planning
  if (taskGraph_->taskPlanningEnabled())
  {
    if (taskGraph_->getTaskLevel(startState) != 0)
    {
      OMPL_ERROR("solve: start level is %u", taskGraph_->getTaskLevel(startState));
      exit(-1);
    }
    if (taskGraph_->getTaskLevel(goalState) != 2)
    {
      OMPL_ERROR("solve: goal level is %u", taskGraph_->getTaskLevel(goalState));
      exit(-1);
    }
  }

  // Create solution path as pointer so memory is not unloaded
  ob::PathPtr pathSolutionBase(new og::PathGeometric(si_));
  og::PathGeometric &geometricSolution = static_cast<og::PathGeometric &>(*pathSolutionBase);

  // Search
  if (!getPathOffGraph(startState, goalState, geometricSolution, ptc, indent))
  {
    OMPL_WARN("BoltPlanner::solve() No near start or goal found");
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

  // Add more points to path
  // geometricSolution.interpolate();

  // Save solution
  double approximateDifference = -1;
  bool approximate = false;
  pdef_->addSolutionPath(pathSolutionBase, approximate, approximateDifference, getName());
  solved = true;

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
    BOLT_DEBUG(indent, verbose_, "Starting getPathOffGraph attempt " << attempt);

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
    bool result = getPathOnGraph(startVertexCandidateNeighbors_, goalVertexCandidateNeighbors_, start, goal,
                                 geometricSolution, ptc, /*debug*/ false, feedbackStartFailed, indent);

    // Error check
    if (!result)
    {
      OMPL_WARN("getPathOffGraph(): BoltPlanner returned FALSE for getPathOnGraph. Trying again in debug mode");

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
  for (TaskVertex start : candidateStarts)
  {
    if (actualStart == taskGraph_->getState(start))
    {
      OMPL_ERROR("Comparing same start state");
      exit(-1);  // just curious if this ever happens, no need to actually exit
      continue;
    }

    // Check if this start is visible from the actual start
    if (!si_->checkMotion(actualStart, taskGraph_->getState(start)))
    {
      if (verbose_)
      {
        OMPL_WARN("FOUND START CANDIDATE THAT IS NOT VISIBLE ");
      }
      if (debug)
      {
        visual_->viz4()->state(taskGraph_->getState(start), tools::LARGE, tools::RED, 1);
        visual_->viz4()->edge(actualStart, taskGraph_->getState(start), 100);
        visual_->viz4()->trigger();
        usleep(0.1 * 1000000);
      }
      continue;  // this is actually not visible
    }
    foundValidStart = true;

    for (TaskVertex goal : candidateGoals)
    {
      if (actualGoal == taskGraph_->getState(goal))
      {
        OMPL_ERROR("Comparing same goal state");
        continue;
      }

      BOLT_DEBUG(indent, verbose_, "foreach_goal: Checking motion from " << actualGoal << " to "
                                                                         << taskGraph_->getState(goal));

      if (ptc)  // Check if our planner is out of time
      {
        OMPL_DEBUG("getPathOnGraph function interrupted because termination condition is true.");
        return false;
      }

      // Check if this goal is visible from the actual goal
      if (!si_->checkMotion(actualGoal, taskGraph_->getState(goal)))
      {
        if (verbose_)
        {
          OMPL_WARN("FOUND GOAL CANDIDATE THAT IS NOT VISIBLE! ");
        }

        if (debug)
        {
          visual_->viz4()->state(taskGraph_->getState(goal), tools::SMALL, tools::RED, 1);
          visual_->viz4()->edge(actualGoal, taskGraph_->getState(goal), 100);
          visual_->viz4()->trigger();
          usleep(0.1 * 1000000);
        }

        continue;  // this is actually not visible
      }
      foundValidGoal = true;

      // Repeatidly search through graph for connection then check for collisions then repeat
      if (lazyCollisionSearch(start, goal, actualStart, actualGoal, geometricSolution, ptc, indent))
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
    OMPL_ERROR("Unexpected condition - both a valid start and goal were found but still no path found. TODO ");
    exit(-1);
  }

  if (foundValidStart && !foundValidGoal)
  {
    OMPL_WARN("Unable to connect GOAL state to graph");
    feedbackStartFailed = false;  // it was the goal state that failed us
  }
  else
  {
    OMPL_WARN("Unable to connect START state to graph");
    feedbackStartFailed = true;  // the start state failed us
  }

  return false;
}

bool BoltPlanner::lazyCollisionSearch(const TaskVertex &start, const TaskVertex &goal, const base::State *actualStart,
                                      const base::State *actualGoal, og::PathGeometric &geometricSolution,
                                      Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "BoltPlanner::lazyCollisionSearch()");

  // Vector to store candidate paths in before they are converted to PathPtrs
  std::vector<TaskVertex> vertexPath;
  double distance;  // resulting path distance

  // Make sure that the start and goal aren't so close together that they find the same vertex
  if (start == goal)
  {
    BOLT_DEBUG(indent, verbose_, "    Start equals goal, creating simple solution ");

    // There are only three verticies in this path - start, middle, goal
    vertexPath.push_back(start);

    convertVertexPathToStatePath(vertexPath, actualStart, actualGoal, geometricSolution);
    return true;
  }

  // Error check all states are non-nullptr
  assert(actualStart);
  assert(actualGoal);
  assert(taskGraph_->getState(start));
  assert(taskGraph_->getState(goal));

  // Check that our states are on the same connected component
  // TODO: in the future the graph should always just be fully connected
  // so perhaps this check would not be necessary
  if (!taskGraph_->sameComponent(start, goal))
  {
    OMPL_WARN("Found start and goal states that are on different connected components!");
    return false;
  }

  // Visualize start vertex
  const bool visualize = false;
  if (visualize)
  {
    BOLT_DEBUG(indent, verbose_, "viz start -----------------------------");
    visual_->viz5()->state(taskGraph_->getState(start), tools::VARIABLE_SIZE, tools::PURPLE, 1);
    visual_->viz5()->edge(actualStart, taskGraph_->getState(start), 30);
    visual_->viz5()->trigger();
    usleep(5 * 1000000);

    // Visualize goal vertex
    BOLT_DEBUG(indent, verbose_, "viz goal ------------------------------");
    visual_->viz5()->state(taskGraph_->getState(goal), tools::VARIABLE_SIZE, tools::PURPLE, 1);
    visual_->viz5()->edge(actualGoal, taskGraph_->getState(goal), 0);
    visual_->viz5()->trigger();
    usleep(5 * 1000000);
  }

  // Keep looking for paths between chosen start and goal until one is found that is valid,
  // or no further paths can be found between them because of disabled edges
  // this is necessary for lazy collision checking i.e. rerun after marking invalid edges we found
  while (true)
  {
    BOLT_DEBUG(indent, verbose_, "  AStar: looking for path through graph between start and goal");

    // Check if our planner is out of time
    if (ptc)
    {
      OMPL_DEBUG("lazyCollisionSearch: function interrupted because termination condition is true.");
      return false;
    }

    // Attempt to find a solution from start to goal
    if (!taskGraph_->astarSearch(start, goal, vertexPath, distance, indent))
    {
      BOLT_DEBUG(indent, verbose_, "unable to construct solution between start and goal using astar");

      // no path found what so ever
      return false;
    }

    BOLT_DEBUG(indent, verbose_, "Has at least a partial solution, maybe exact solution");
    BOLT_DEBUG(indent, verbose_, "Solution has " << vertexPath.size() << " vertices");

    // Check if all the points in the potential solution are valid
    if (lazyCollisionCheck(vertexPath, ptc, indent))
    {
      BOLT_DEBUG(indent, verbose_, "Lazy collision check returned valid ");

      // the path is valid, we are done!
      convertVertexPathToStatePath(vertexPath, actualStart, actualGoal, geometricSolution);
      return true;
    }
    // else, loop with updated graph that has the invalid edges/states disabled
  }  // end while

  // we never found a valid path
  return false;
}

bool BoltPlanner::lazyCollisionCheck(std::vector<TaskVertex> &vertexPath, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "BoltPlanner::lazyCollisionCheck()");

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
      OMPL_DEBUG("Lazy collision check function interrupted because termination condition is true.");
      return false;
    }

    TaskEdge thisEdge = boost::edge(fromVertex, toVertex, taskGraph_->g_).first;

    // Has this edge already been checked before?
    if (taskGraph_->edgeCollisionStatePropertyTask_[thisEdge] == NOT_CHECKED)
    {
      // Check path between states
      if (!si_->checkMotion(taskGraph_->getState(fromVertex), taskGraph_->getState(toVertex)))
      {
        // Path between (from, to) states not valid, disable the edge
        // BOLT_GREEN_DEBUG(indent, verbose_, "LAZY CHECK: disabling edge from vertex " << fromVertex << " to vertex "
        // << toVertex);

        // Disable edge
        taskGraph_->edgeCollisionStatePropertyTask_[thisEdge] = IN_COLLISION;
      }
      else
      {
        // Mark edge as free so we no longer need to check for collision
        taskGraph_->edgeCollisionStatePropertyTask_[thisEdge] = FREE;
      }
    }

    // Check final result
    if (taskGraph_->edgeCollisionStatePropertyTask_[thisEdge] == IN_COLLISION)
    {
      // Remember that this path is no longer valid, but keep checking remainder of path edges
      hasInvalidEdges = true;
    }

    // switch vertex focus
    fromVertex = toVertex;
  }

  BOLT_DEBUG(indent, verbose_, "Done lazy collision checking");

  // Only return true if nothing was found invalid
  return !hasInvalidEdges;
}

bool BoltPlanner::findGraphNeighbors(const base::State *state, std::vector<TaskVertex> &neighbors, int requiredLevel,
                                     std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "BoltPlanner::findGraphNeighbors()");

  assert(requiredLevel == 0 || requiredLevel == 2);

  // Reset
  neighbors.clear();

  // Choose how many neighbors
  double findNearestKNeighbors;
  if (si_->getStateSpace()->getDimension() == 3)
    findNearestKNeighbors = 10;
  else
    findNearestKNeighbors = 30;

  // Setup search by getting a non-const version of the focused state
  const std::size_t threadID = 0;
  base::State *stateCopy = si_->cloneState(state);

  // Search
  taskGraph_->getQueryStateNonConst(taskGraph_->queryVertices_[threadID]) = stateCopy;
  taskGraph_->nn_->nearestK(taskGraph_->queryVertices_[threadID], findNearestKNeighbors, neighbors);
  taskGraph_->getQueryStateNonConst(taskGraph_->queryVertices_[threadID]) = nullptr;

  // Convert our list of neighbors to the proper level
  if (requiredLevel == 2)
  {
    BOLT_DEBUG(indent, true, "Converting vector of level 0 neighbors to level 2 neighbors");

    for (std::size_t i = 0; i < neighbors.size(); ++i)
    {
      TaskVertex nearVertex = neighbors[i];

      // Get the vertex on the opposite level and replace it in the vector
      TaskVertex newVertex = taskGraph_->vertexTaskMirrorProperty_[nearVertex];

      // Replace
      neighbors[i] = newVertex;
    }
  }

  // Free memory
  si_->getStateSpace()->freeState(stateCopy);

  return neighbors.size();
}

bool BoltPlanner::convertVertexPathToStatePath(std::vector<TaskVertex> &vertexPath, const base::State *actualStart,
                                               const base::State *actualGoal, og::PathGeometric &geometricSolution)
{
  // Ensure the input path is not empty
  if (!vertexPath.size())
    return false;

  // Add original start if it is different than the first state
  if (actualStart != taskGraph_->getState(vertexPath.back()))
  {
    geometricSolution.append(actualStart);
  }

  // Error check that no consequtive verticies are the same
  /*std::cout << "BoltPlanner.Vertices: ";
  for (std::size_t i = vertexPath.size(); i > 0; --i)
  {
      std::cout << vertexPath[i - 1] << ", ";
  }
  std::cout << std::endl;*/

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
        OMPL_ERROR("Found repeated vertices %u to %u on index %u", vertexPath[i - 1], vertexPath[i - 2], i);
        exit(-1);
      }

      TaskEdge edge = boost::edge(vertexPath[i - 1], vertexPath[i - 2], taskGraph_->g_).first;

      // Check if any edges in path are not free (then it an approximate path)
      if (taskGraph_->edgeCollisionStatePropertyTask_[edge] == IN_COLLISION)
      {
        OMPL_ERROR("Found invalid edge / approximate solution - how did this happen?");
      }
      else if (taskGraph_->edgeCollisionStatePropertyTask_[edge] == NOT_CHECKED)
      {
        OMPL_ERROR("A chosen path has an edge that has not been checked for collision. This should not happen");
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
  BOLT_FUNC(indent, verbose_, "BoltPlanner: simplifyPath()");
  BOLT_ERROR(indent, true, "BoltPlanner: simplifyPath() - why no task??");

  time::point simplifyStart = time::now();
  std::size_t numStates = path.getStateCount();

  path_simplifier_->simplify(path, ptc);
  double simplifyTime = time::seconds(time::now() - simplifyStart);

  int diff = numStates - path.getStateCount();
  BOLT_DEBUG(indent, verbose_, "BoltPlanner: Path simplification took " << simplifyTime << " seconds and removed "
                                                                        << diff << " states");

  return true;
}

bool BoltPlanner::simplifyTaskPath(og::PathGeometric &path, Termination &ptc, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "BoltPlanner: simplifyTaskPath()");

  time::point simplifyStart = time::now();
  std::size_t origNumStates = path.getStateCount();

  // Number of levels
  const std::size_t NUM_LEVELS = 3;

  // Create three path segments
  std::vector<og::PathGeometric> pathSegment;
  for (int segmentLevel = 0; segmentLevel < int(NUM_LEVELS); ++segmentLevel)
    pathSegment.push_back(og::PathGeometric(si_));

  // Create the solution path
  og::PathGeometric smoothedPath(si_);

  // Divide the path into different levels
  VertexLevel previousLevel = 0;  // Error check ordering of input path
  for (std::size_t i = 0; i < path.getStateCount(); ++i)
  {
    base::State *state = path.getState(i);
    VertexLevel level = si_->getStateSpace()->getLevel(state);

    assert(level < NUM_LEVELS);

    pathSegment[level].append(state);

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
    (void)seg0Size;  // silence unused warning
    (void)seg1Size;  // silence unused warning
    (void)seg2Size;  // silence unused warning

    // Move first state
    pathSegment[0].append(pathSegment[1].getStates().front());
    pathSegment[1].getStates().erase(pathSegment[1].getStates().begin());

    // Move last state
    pathSegment[2].prepend(pathSegment[1].getStates().back());
    pathSegment[1].getStates().pop_back();

    // Check the operations were correct
    BOOST_ASSERT_MSG(seg0Size + 1 == pathSegment[0].getStateCount(), "Invalid size of pathSegement after rearrangment");
    BOOST_ASSERT_MSG(seg1Size - 2 == pathSegment[1].getStateCount(), "Invalid size of pathSegement after rearrangment");
    BOOST_ASSERT_MSG(seg2Size + 1 == pathSegment[2].getStateCount(), "Invalid size of pathSegement after rearrangment");
  }
  else
  {
    OMPL_WARN("The Cartesian path segement 1 has only %u states", pathSegment[1].getStateCount());
  }

  // Smooth the freespace paths
  path_simplifier_->simplifyMax(pathSegment[0]);
  path_simplifier_->simplifyMax(pathSegment[2]);

  // Combine the path segments back together
  for (int segmentLevel = 0; segmentLevel < int(NUM_LEVELS); ++segmentLevel)
  {
    for (std::size_t i = 0; i < pathSegment[segmentLevel].getStateCount(); ++i)
    {
      base::State *state = pathSegment[segmentLevel].getState(i);

      // Enforce the correct level on the state because the OMPL components don't understand the concept
      si_->getStateSpace()->setLevel(state, segmentLevel);

      // Add to solution path
      smoothedPath.append(state);
    }
  }

  // Replace the input path with the new smoothed path
  path = smoothedPath;

  double simplifyTime = time::seconds(time::now() - simplifyStart);

  int diff = origNumStates - path.getStateCount();
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
    if (!si_->checkMotion(s1, s2))
    {
      OMPL_WARN("NEIGHBOR %u NOT VISIBLE ", count++);

      if (false)
      {
        visual_->viz5()->state(s2, tools::MEDIUM, tools::BLUE, 1);
        visual_->viz5()->edge(s1, s2, 100);
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
            visual_->viz5()->state(interState, tools::LARGE, tools::RED, 1);
            visual_->viz5()->trigger();
            usleep(1 * 1000000);
          }
          else
          {
            // visual_->viz5()->state(interState, /*mode=*/1, 1); // GREEN
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

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
