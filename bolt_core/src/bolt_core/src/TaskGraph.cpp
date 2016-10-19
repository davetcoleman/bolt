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
   Desc:   Near-asypmotically optimal roadmap datastructure
*/

// OMPL
#include <bolt_core/TaskGraph.h>
#include <ompl/util/Console.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>

// Boost
#include <boost/graph/incremental_components.hpp>
#include <boost/foreach.hpp>
#include <boost/unordered_set.hpp>
#include <boost/assert.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

// C++
#include <limits>
#include <queue>

// Profiling
//#include <valgrind/callgrind.h>

#define foreach BOOST_FOREACH

namespace og = ompl::geometric;
namespace ot = ompl::tools;
namespace otb = ompl::tools::bolt;
namespace ob = ompl::base;

namespace ompl
{
namespace tools
{
namespace bolt
{
TaskGraph::TaskGraph(const base::SpaceInformationPtr &modelSI, const base::SpaceInformationPtr &compoundSI,
                     SparseGraphPtr sg)
  : modelSI_(modelSI), compoundSI_(compoundSI), sg_(sg)
{
  // Save number of threads available
  numThreads_ = boost::thread::hardware_concurrency();

  // Copy the pointers of various components
  visual_ = sg_->getVisual();
  compoundSpace_ = std::dynamic_pointer_cast<base::CompoundStateSpace>(compoundSI_->getStateSpace());

  // Add search state
  initializeQueryState();

  // Initialize nearest neighbor datastructure
  // TODO(davetcoleman): do we need to have a separate NN_ structure for the TaskGraph??
  nn_.reset(new NearestNeighborsGNAT<TaskVertex>());
  nn_->setDistanceFunction(boost::bind(&otb::TaskGraph::distanceVertex, this, _1, _2));
}

TaskGraph::~TaskGraph()
{
  freeMemory();
}

bool TaskGraph::setup()
{
  // Initialize path simplifier
  if (!pathSimplifier_)
  {
    pathSimplifier_.reset(new geometric::PathSimplifier(compoundSI_));
    pathSimplifier_->freeStates(true);
  }

  return true;
}

void TaskGraph::clear()
{
  freeMemory();
  initializeQueryState();

  graphUnsaved_ = false;
  taskPlanningEnabled_ = false;
}

void TaskGraph::freeMemory()
{
  foreach (TaskVertex v, boost::vertices(g_))
  {
    if (g_[v].state_ == nullptr)
      continue;

    base::CompoundState *compoundState = g_[v].state_->as<base::CompoundState>();
    base::DiscreteStateSpace::StateType *discreteState =
        compoundState->as<base::DiscreteStateSpace::StateType>(DISCRETE);
    base::State *modelBasedState = compoundState->as<base::DiscreteStateSpace::StateType>(MODEL_BASED);

    // std::cout << "compoundState: " << compoundState << std::endl;
    // std::cout << "discreteState: " << discreteState << std::endl;
    // std::cout << "modelBasedState: " << modelBasedState << std::endl;

    // Do not free the joint state data (ModelBasedStateSpace) if its part of the SPARS graph also
    if (discreteState->value == 1)  // the memory was allocated by cartesian planner and should be freed
    {
      // std::cout << "discreteState->value == 1 " << std::endl;
      compoundSpace_->getSubspace(MODEL_BASED)->freeState(modelBasedState);
    }

    // Either way the Discrete state should be unloaded
    // std::cout << "free discrete " << std::endl;
    compoundSpace_->getSubspace(DISCRETE)->freeState(discreteState);

    // Delete the outer compound state
    delete[] compoundState->components;
    delete compoundState;
  }

  g_.clear();
  nn_->clear();
}

void TaskGraph::initializeQueryState()
{
  if (boost::num_vertices(g_) > 0)
  {
    OMPL_WARN("Not initializing query state because already is of size %u", boost::num_vertices(g_));
    return;  // assume its already been setup
  }

  // Create a query state for each possible thread
  queryVertices_.resize(numThreads_);
  queryStates_.resize(numThreads_);

  for (std::size_t threadID = 0; threadID < numThreads_; ++threadID)
  {
    // Add a fake vertex to the graph
    queryVertices_[threadID] = boost::add_vertex(g_);
    g_[queryVertices_[threadID]].state_ = NULL;  // set stateID to the zeroth stateID - NULL
  }
}

bool TaskGraph::astarSearch(const TaskVertex start, const TaskVertex goal, std::vector<TaskVertex> &vertexPath,
                            double &distance, std::size_t indent)
{
  BOLT_FUNC(indent, vSearch_, "TaskGraph.astarSearch()");

  // Hold a list of the shortest path parent to each vertex
  TaskVertex *vertexPredecessors = new TaskVertex[getNumVertices()];
  // boost::vector_property_map<TaskVertex> vertexPredecessors(getNumVertices());

  bool foundGoal = false;
  double *vertexDistances = new double[getNumVertices()];

#ifndef NDEBUG
  // Reset statistics
  numNodesOpened_ = 0;
  numNodesClosed_ = 0;
#endif

  if (visualizeAstar_)
  {
    // Assume this was cleared by the parent program
    visual_->viz4()->deleteAllMarkers();
  }

  try
  {
    boost::astar_search(g_, start,  // graph, start state
                        [this, goal](TaskVertex v)
                        {
                          return astarTaskHeuristic(v, goal);  // the heuristic
                        },
                        // ability to disable edges (set cost to inifinity):
                        boost::weight_map(TaskEdgeWeightMap(g_))
                            // boost::weight_map(boost::get(&TaskEdgeStruct::weight_, g_))
                            .predecessor_map(vertexPredecessors)
                            .distance_map(&vertexDistances[0])
                            .visitor(TaskAstarVisitor(goal, this)));
  }
  catch (FoundGoalException &)
  {
    distance = vertexDistances[goal];

#ifndef NDEBUG
    // the custom exception from TaskAstarVisitor
    BOLT_DEBUG(indent, vSearch_, "AStar found solution. Distance to goal: " << vertexDistances[goal]);
    BOLT_DEBUG(indent, vSearch_, "Number nodes opened: " << numNodesOpened_
                                                         << ", Number nodes closed: " << numNodesClosed_);
#endif

    // Only clear the vertexPath after we know we have a new solution, otherwise it might have a good
    // previous one
    vertexPath.clear();  // remove any old solutions

    // Trace back the shortest path in reverse and only save the states
    TaskVertex v;
    for (v = goal; v != vertexPredecessors[v]; v = vertexPredecessors[v])
    {
      vertexPath.push_back(v);
    }

    // Add the start state to the path, unless this path is just one vertex long and the start==goal
    if (v != goal)
    {
      vertexPath.push_back(v);
    }

    foundGoal = true;
  }

  if (!foundGoal)
    BOLT_WARN(indent, vSearch_, "Did not find goal");

#ifndef NDEBUG
  // Show all predecessors
  if (visualizeAstar_)
  {
    BOLT_DEBUG(indent + 2, vSearch_, "Show all predecessors");
    for (std::size_t i = numThreads_; i < getNumVertices(); ++i)  // skip vertex 0-11 because those are query vertices
    {
      const TaskVertex v1 = i;
      const TaskVertex v2 = vertexPredecessors[v1];
      if (v1 != v2)
      {
        visual_->viz4()->edge(getModelBasedState(v1), getModelBasedState(v2), 10);
      }
    }
    visual_->viz4()->trigger();
  }
#endif

  // Unload
  delete[] vertexPredecessors;
  delete[] vertexDistances;

  // No solution found from start to goal
  return foundGoal;
}

double TaskGraph::distanceVertex(const TaskVertex a, const TaskVertex b) const
{
  // Special case: query vertices store their states elsewhere. Both cannot be query vertices
  if (a < numThreads_)
  {
    return distanceState(queryStates_[a], getCompoundState(b));
  }
  if (b < numThreads_)
  {
    return distanceState(getCompoundState(a), queryStates_[b]);
  }

  assert(getCompoundState(a) != NULL);
  assert(getCompoundState(b) != NULL);

  return distanceState(getCompoundState(a), getCompoundState(b));
}

double TaskGraph::distanceState(const base::State *a, const base::State *b) const
{
  // Disregard task level
  // const base::CompoundState *cA = static_cast<const base::CompoundState *>(a);
  // const base::CompoundState *cB = static_cast<const base::CompoundState *>(b);
  // return compoundSpace_->getSubspace(MODEL_BASED)->distance(cA->components[MODEL_BASED],
  // cB->components[MODEL_BASED]);

  return compoundSpace_->getSubspace(MODEL_BASED)->distance(getModelBasedState(a), getModelBasedState(b));
}

double TaskGraph::astarTaskHeuristic(const TaskVertex a, const TaskVertex b) const
{
  // Do not use task distance if that mode is not enabled
  if (!taskPlanningEnabled_)
    return distanceVertex(a, b);

  // Reorder a & b so that we are sure that a.level <= b.level
  VertexLevel taskLevelA = getTaskLevel(a);
  VertexLevel taskLevelB = getTaskLevel(b);
  if (taskLevelA > taskLevelB)
  {
    // Call itself again, this time switching ordering
    BOLT_DEBUG(0, vHeuristic_, "Switched ordering for distanceFunctionTasks()");
    return astarTaskHeuristic(b, a);
  }

  double dist = 0;  // the result

  // Note: this value should be synced with TaskGraph
  static const double TASK_LEVEL_COST = 100.0;  // cost to change levels/tasks

  // Error check
  assert(g_[a].state_);
  assert(g_[b].state_);
  assert(g_[startConnectorVertex_].state_);
  assert(g_[goalConnectorVertex_].state_);

  if (taskLevelA == 0)
  {
    if (taskLevelB == 0)  // regular distance for bottom level
    {
      BOLT_DEBUG(0, vHeuristic_, "Distance Mode a");
      dist = distanceState(g_[a].state_, g_[b].state_);
    }
    else if (taskLevelB == 1)
    {
      BOLT_DEBUG(0, vHeuristic_, "Distance Mode b");
      dist = distanceState(g_[a].state_, g_[startConnectorVertex_].state_) + TASK_LEVEL_COST +
             distanceState(g_[startConnectorVertex_].state_, g_[b].state_);
    }
    else if (taskLevelB == 2)
    {
      BOLT_DEBUG(0, vHeuristic_, "Distance Mode c");
      dist = distanceState(g_[a].state_, g_[startConnectorVertex_].state_) + TASK_LEVEL_COST +
             shortestDistAcrossCartGraph_ + TASK_LEVEL_COST +
             distanceState(g_[goalConnectorVertex_].state_, g_[b].state_);
    }
    else
    {
      throw Exception(name_, "Unknown task level mode");
    }
  }
  else if (taskLevelA == 1)
  {
    if (taskLevelB == 0)
    {
      throw Exception(name_, "Unknown task level mode");
    }
    else if (taskLevelB == 1)
    {
      BOLT_DEBUG(0, vHeuristic_, "Distance Mode d");
      dist = distanceState(g_[a].state_, g_[b].state_);
    }
    else if (taskLevelB == 2)
    {
      BOLT_DEBUG(0, vHeuristic_, "Distance Mode e");

      dist = distanceState(g_[a].state_, g_[goalConnectorVertex_].state_) + TASK_LEVEL_COST +
             distanceState(g_[goalConnectorVertex_].state_, g_[b].state_);
    }
    else
    {
      throw Exception(name_, "Unknown task level mode");
    }
  }
  else if (taskLevelA == 2)
  {
    if (taskLevelB == 0 || taskLevelB == 1)
    {
      throw Exception(name_, "Unknown task level mode");
    }
    else if (taskLevelB == 2)
    {
      BOLT_DEBUG(0, vHeuristic_, "Distance Mode f");
      dist = distanceState(g_[a].state_, g_[b].state_);
    }
    else
    {
      throw Exception(name_, "Unknown task level mode");
    }
  }
  else
  {
    throw Exception(name_, "Unknown task level mode");
  }

  BOLT_DEBUG(0, vHeuristic_, "Vertex " << a << " @level " << taskLevelA << " to Vertex " << b << " @level "
                                       << taskLevelB << " has distance " << dist);
  return dist;
}

bool TaskGraph::isEmpty() const
{
  assert(!(getNumVertices() < getNumQueryVertices()));
  return (getNumVertices() == getNumQueryVertices() && getNumEdges() == 0);
}

void TaskGraph::generateMonoLevelTaskSpace(std::size_t indent)
{
  BOLT_ERROR(indent, "TODO: implement");
  /*
  BOLT_FUNC(indent, verbose_, "TaskGraph.generateMonoLevelTaskTaskSpace()");
  time::point startTime = time::now();  // Benchmark

  // Clear pre-existing graphs
  if (!isEmpty())
  {
    BOLT_DEBUG(indent, vGenerateTask_, "Clearing previous graph");
    clear();
  }

  // Record a mapping from SparseVertex to the TaskVertex
  std::vector<TaskVertex> sparseToTaskVertex0(sg_->getNumVertices());

  // Loop through every vertex in sparse graph and copy twice to task graph
  BOLT_DEBUG(indent + 2, true || vGenerateTask_, "Adding " << sg_->getNumVertices() << " task space vertices");
  foreach (SparseVertex sparseV, boost::vertices(sg_->getGraph()))
  {
    // The first thread number of verticies are used for queries and should be skipped
    if (sparseV < sg_->getNumQueryVertices())
      continue;

    const base::State *state = sg_->getState(sparseV);

    // Create level 0 vertex
    VertexLevel level = 0;
    TaskVertex taskV0 = addVertex(compoundSI_->cloneState(state), level, indent);
    sparseToTaskVertex0[sparseV] = taskV0;  // record mapping
  }

  // Loop through every edge in sparse graph and copy to task graph
  BOLT_DEBUG(indent + 2, true || vGenerateTask_, "Adding " << sg_->getNumEdges() << " task space edges");
  foreach (const SparseEdge sparseE, boost::edges(sg_->getGraph()))
  {
    const SparseVertex sparseE_v0 = boost::source(sparseE, sg_->getGraph());
    const SparseVertex sparseE_v2 = boost::target(sparseE, sg_->getGraph());

    // Error check
#ifndef NDEBUG
    BOLT_ASSERT(sparseE_v0 >= sg_->getNumQueryVertices(), "Found query vertex in sparse graph that has an edge!");
    BOLT_ASSERT(sparseE_v2 >= sg_->getNumQueryVertices(), "Found query vertex in sparse graph that has an edge!");
#endif

    // Create level 0 edge
    addEdge(sparseToTaskVertex0[sparseE_v0], sparseToTaskVertex0[sparseE_v2], indent);
  }

  // Visualize
  // displayDatabase();

  printGraphStats(time::seconds(time::now() - startTime));

  // Tell the planner to require task planning
  taskPlanningEnabled_ = false;
  */
}

void TaskGraph::generateTaskSpace(std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "TaskGraph.generateTaskSpace()");
  time::point startTime = time::now();  // Benchmark

  // Clear pre-existing graphs
  if (!isEmpty())
  {
    BOLT_DEBUG(indent, vGenerateTask_, "Clearing previous graph");
    clear();
  }

  // Record a mapping from SparseVertex to the two TaskVertices
  std::vector<TaskVertex> sparseToTaskVertex0(sg_->getNumVertices());
  std::vector<TaskVertex> sparseToTaskVertex2(sg_->getNumVertices());

  // Loop through every vertex in sparse graph and copy twice to task graph
  BOLT_DEBUG(indent + 2, true || vGenerateTask_, "Adding " << 2 * sg_->getNumVertices() << " task space vertices");
  foreach (SparseVertex sparseV, boost::vertices(sg_->getGraph()))
  {
    // The first thread number of verticies are used for queries and should be skipped
    if (sparseV < sg_->getNumQueryVertices())
      continue;

    base::State *jointState = sg_->getStateNonConst(sparseV);  // TODO - should this be const?

    // Create level 0 vertex
    const VertexLevel level0 = 0;
    TaskVertex taskV0 = addVertexWithLevel(jointState, level0, indent);
    sparseToTaskVertex0[sparseV] = taskV0;  // record mapping

    // Create level 2 vertex
    const VertexLevel level2 = 2;
    const TaskVertex taskV2 = addVertexWithLevel(jointState, level2, indent);
    sparseToTaskVertex2[sparseV] = taskV2;  // record mapping

    // Link the two vertices to each other for future bookkeeping
    g_[taskV0].task_mirror_ = taskV2;
    g_[taskV2].task_mirror_ = taskV0;
  }

  // Loop through every edge in sparse graph and copy twice to task graph
  BOLT_DEBUG(indent + 2, true || vGenerateTask_, "Adding " << 2 * sg_->getNumEdges() << " task space edges");
  foreach (const SparseEdge sparseE, boost::edges(sg_->getGraph()))
  {
    const SparseVertex &sparseE_v0 = boost::source(sparseE, sg_->getGraph());
    const SparseVertex &sparseE_v2 = boost::target(sparseE, sg_->getGraph());

    // Create level 0 edge
    addEdge(sparseToTaskVertex0[sparseE_v0], sparseToTaskVertex0[sparseE_v2], indent);

    // Create level 2 edge
    addEdge(sparseToTaskVertex2[sparseE_v0], sparseToTaskVertex2[sparseE_v2], indent);
  }

  // Visualize
  // displayDatabase();

  printGraphStats(time::seconds(time::now() - startTime), indent);

  // Tell the planner to require task planning
  taskPlanningEnabled_ = true;
}

bool TaskGraph::addCartPath(std::vector<base::State *> path, std::size_t indent)
{
  BOLT_ERROR(indent, "TODO implement");
  /*
  BOLT_FUNC(indent, verbose_, "TaskGraph.addCartPath()");

  // Error check
  if (path.size() < 2)
  {
    OMPL_ERROR("Invalid cartesian path - too few states");
    return false;
  }
  // TODO: check for validity

  // Clear previous cartesian path
  generateTaskSpace(indent);

  // Create verticies for the extremas - start & goal
  VertexLevel level = 1;  // middle layer
  TaskVertex startVertex = addVertex(path.front(), level, indent);
  TaskVertex goalVertex = addVertex(path.back(), level, indent);

  // Record min cost for cost-to-go heurstic distance function later
  shortestDistAcrossCartGraph_ = distanceVertex(startVertex, goalVertex);

  // Connect Start to graph --------------------------------------
  BOLT_DEBUG(indent, verbose_, "Creating start connector");
  const VertexLevel level0 = 0;
  bool isStart = true;
  if (!connectVertexToNeighborsAtLevel(startVertex, level0, isStart, indent))
  {
    OMPL_ERROR("Failed to connect start of cartesian path");
    return false;
  }

  // Connect goal to graph --------------------------------------
  BOLT_DEBUG(indent, verbose_, "Creating goal connector");
  const VertexLevel level2 = 2;
  isStart = false;
  if (!connectVertexToNeighborsAtLevel(goalVertex, level2, isStart, indent))
  {
    OMPL_ERROR("Failed to connect goal of cartesian path");
    return false;
  }

  // Add cartesian path to mid level graph --------------------
  TaskVertex v1 = startVertex;
  TaskVertex v2;
  VertexLevel cartLevel = 1;
  BOLT_DEBUG(indent, verbose_, "Add cartesian path");

  for (std::size_t i = 1; i < path.size(); ++i)
  {
    // Check if we are on the goal vertex
    if (i == path.size() - 1)
    {
      v2 = goalVertex;  // Do not create the goal vertex twice
    }
    else
    {
      v2 = addVertex(path[i], cartLevel, indent);
    }

    addEdge(v1, v2, indent);
    v1 = v2;
  }

  // Tell the planner to require task planning
  taskPlanningEnabled_ = true;
  */

  return true;
}

bool TaskGraph::connectVertexToNeighborsAtLevel(TaskVertex fromVertex, const VertexLevel level, bool isStart,
                                                std::size_t indent)
{
  BOLT_FUNC(indent, vGenerateTask_, "TaskGraph.connectVertexToNeighborsAtLevel()");

  // Get nearby states to goal
  std::vector<TaskVertex> neighbors;
  getNeighborsAtLevel(fromVertex, level, numNeighborsConnectToCart_, neighbors, indent);

  // Error check
  if (neighbors.empty())
  {
    BOLT_ERROR(indent, "No neighbors found when connecting cartesian path");
    return false;
  }
  else if (neighbors.size() < 3)
  {
    BOLT_DEBUG(indent, vGenerateTask_, "Only found " << neighbors.size() << " neighbors on level " << level);
  }
  else
    BOLT_DEBUG(indent, vGenerateTask_, "Found " << neighbors.size() << " neighbors on level " << level);

  // Find the shortest connector out of all the options

  // Loop through each neighbor
  for (TaskVertex v : neighbors)
  {
    // Add edge from nearby graph vertex to cart path goal
    double connectorCost = distanceVertex(fromVertex, v);
    addEdge(fromVertex, v, indent);

    // Remember which cartesian start/goal states should be used for distanceVertex
    if (isStart && connectorCost < startConnectorMinCost_)
    {
      startConnectorMinCost_ = connectorCost;  // TODO(davetcoleman): should we save the cost, or just use 1.0?
      startConnectorVertex_ = v;
    }
    else if (!isStart && connectorCost < goalConnectorMinCost_)
    {
      goalConnectorMinCost_ = connectorCost;  // TODO(davetcoleman): should we save the cost, or just use 1.0?
      goalConnectorVertex_ = v;
    }
  }

  return true;
}

void TaskGraph::getNeighborsAtLevel(const TaskVertex origVertex, const VertexLevel level, const std::size_t kNeighbors,
                                    std::vector<TaskVertex> &neighbors, std::size_t indent)
{
  BOLT_FUNC(indent, vGenerateTask_, "TaskGraph.getNeighborsAtLevel()");

  BOLT_ASSERT(level != 1, "Unhandled level, does not support level 1");

  const std::size_t threadID = 0;
  base::State *origState = getCompoundStateNonConst(origVertex);

  // Get nearby state
  queryStates_[threadID] = origState;
  nn_->nearestK(queryVertices_[threadID], kNeighbors, neighbors);
  queryStates_[threadID] = nullptr;

  /*
  // Run various checks
  for (std::size_t i = 0; i < neighbors.size(); ++i)
  {
    TaskVertex nearVertex = neighbors[i];

    bool useCollisionChecking = false;

    if (useCollisionChecking)
    {
      // Collision check
      if (!compoundSI_->checkMotion(origState, getState(nearVertex)))  // is not valid motion
      {
        BOLT_DEBUG(indent, vGenerateTask_, "Skipping neighbor " << nearVertex << ", i=" << i
                                                                << ", at level=" << getTaskLevel(nearVertex)
                                                                << " because invalid motion");
        neighbors.erase(neighbors.begin() + i);
        i--;
        continue;
      }
    }

    BOLT_DEBUG(indent, vGenerateTask_, "Keeping neighbor " << nearVertex);
  }
  */

  // Convert our list of neighbors to the proper level
  if (level == 2)
  {
    BOLT_DEBUG(indent, vGenerateTask_, "Converting vector of level 0 neighbors to level 2 neighbors");

    for (std::size_t i = 0; i < neighbors.size(); ++i)
    {
      const TaskVertex nearVertex = neighbors[i];

      // Get the vertex on the opposite level and replace it in the vector
      neighbors[i] = g_[nearVertex].task_mirror_;
    }
  }
}

bool TaskGraph::checkTaskPathSolution(og::PathGeometric &path, ob::State *start, ob::State *goal)
{
  // TODO: this assumes the path has task data, which it no longer does
  BOLT_ERROR(0, "checkTaskPathSolution() - broken");

  bool error = true; // false
  /*
  VertexLevel current_level = 0;

  for (std::size_t i = 0; i < path.getStateCount(); ++i)
  {
    VertexLevel level = getTaskLevel(path.getCompoundState(i));

    // Check if start state is correct
    if (i == 0)
    {
      if (!compoundSI_->getStateSpace()->equalStates(path.getState(i), start))
      {
        OMPL_ERROR("Start state of path is not same as original problem");
        error = true;
      }

      if (level != 0)
      {
        OMPL_ERROR("Start state is not at level 0, instead %i", level);
        error = true;
      }
    }

    // Check if goal state is correct
    if (i == path.getStateCount() - 1)
    {
      if (!compoundSI_->getStateSpace()->equalStates(path.getState(i), goal))
      {
        OMPL_ERROR("Goal state of path is not same as original problem");
        error = true;
      }

      if (level != 2)
      {
        OMPL_ERROR("Goal state is not at level 2, instead %i", level);
        error = true;
      }
    }

    // Ensure that level is always increasing
    if (level < current_level)
    {
      OMPL_ERROR("State decreased in level (%i) from previous level of ", current_level);
      error = true;
    }
    current_level = level;

  }  // for loop

  // Show more data if error
  if (error)
  {
    OMPL_ERROR("Showing data on path:");
    for (std::size_t i = 0; i < path.getStateCount(); ++i)
    {
      VertexLevel level = getTaskLevel(path.getState(i));
      OMPL_INFORM(" - Path state %i has level %i", i, level);
    }
  }
  */

  return error;
}

void TaskGraph::clearEdgeCollisionStates()
{
  foreach (const TaskEdge e, boost::edges(g_))
    g_[e].collision_state_ = NOT_CHECKED;  // each edge has an unknown state
}

void TaskGraph::errorCheckDuplicateStates(std::size_t indent)
{
  BOLT_ERROR(indent, "TaskGraph.errorCheckDuplicateStates() - NOT IMPLEMENTEDpart of super debug");

  // bool found = false;
  // // Error checking: check for any duplicate states
  // for (std::size_t i = 0; i < dense_Cache_->getStateCacheSize(); ++i)
  // {
  //   for (std::size_t j = i + 1; j < dense_Cache_->getStateCacheSize(); ++j)
  //   {
  //     if (compoundSI_->getStateSpace()->equalStates(getState(i), getState(j)))
  //     {
  //       BOLT_ERROR(indent, 1, "Found equal state: " << i << ", " << j);
  //       debugState(getState(i));
  //       found = true;
  //     }
  //   }
  // }
  // if (found)
  //   throw Exception(name_, "Duplicate state found");
}

bool TaskGraph::smoothQualityPath(geometric::PathGeometric *path, double clearance, std::size_t indent)
{
  BOLT_FUNC(indent, visualizeQualityPathSmoothing_, "TaskGraph.smoothQualityPath()");

  BOLT_ERROR(indent, "smoothQualityPath is using compound state space which has unknown smoothing results...");

  // Visualize path
  if (visualizeQualityPathSmoothing_)
  {
    visual_->viz2()->deleteAllMarkers();
    visual_->viz2()->path(path, tools::SMALL, tools::BLACK, tools::BLUE);
    visual_->viz2()->trigger();
    usleep(0.001 * 1000000);
  }

  BOLT_DEBUG(indent, visualizeQualityPathSmoothing_, "Created 'quality path' candidate with " << path->getStateCount()
                                                                                              << " states");
  if (visualizeQualityPathSmoothing_)
    visual_->waitForUserFeedback("path simplification");

  // Set the motion validator to use clearance, this way isValid() checks clearance before confirming valid
  base::DiscreteMotionValidator *dmv =
      dynamic_cast<base::DiscreteMotionValidator *>(modelSI_->getMotionValidator().get());
  dmv->setRequiredStateClearance(clearance);

  for (std::size_t i = 0; i < 3; ++i)
  {
    pathSimplifier_->simplifyMax(*path);

    if (visualizeQualityPathSmoothing_)
    {
      visual_->viz2()->deleteAllMarkers();
      visual_->viz2()->path(path, tools::SMALL, tools::BLACK, tools::ORANGE);
      visual_->viz2()->trigger();
      usleep(0.1 * 1000000);
      // visual_->waitForUserFeedback("optimizing path");
    }

    pathSimplifier_->reduceVertices(*path, 1000, path->getStateCount() * 4);

    if (visualizeQualityPathSmoothing_)
    {
      visual_->viz2()->deleteAllMarkers();
      visual_->viz2()->path(path, tools::SMALL, tools::BLACK, tools::BLUE);
      visual_->viz2()->trigger();
      usleep(0.1 * 1000000);
      // visual_->waitForUserFeedback("optimizing path");
    }
  }
  // Turn off the clearance requirement
  dmv->setRequiredStateClearance(0.0);

  pathSimplifier_->reduceVertices(*path, 1000, path->getStateCount() * 4);

  if (visualizeQualityPathSmoothing_)
  {
    visual_->viz2()->deleteAllMarkers();
    visual_->viz2()->path(path, tools::SMALL, tools::BLACK, tools::GREEN);
    visual_->viz2()->trigger();
    visual_->waitForUserFeedback("finished quality path");
  }

  std::pair<bool, bool> repairResult = path->checkAndRepair(100);

  if (!repairResult.second)  // Repairing was not successful
  {
    throw Exception(name_, "check and repair failed?");
  }
  return true;
}

/** \brief Create a compound state that includes a discrete step
 *  \param state - the base state of joint values
 *  \param level - the discrete step of a level
 *  \return new state that is compound
*/
base::State *TaskGraph::createCompoundState(base::State *jointState, const VertexLevel level, std::size_t indent)
{
  base::CompoundState *state = new base::CompoundState();
  state->components = new base::State *[compoundSpace_->getSubspaceCount()];
  BOLT_ASSERT(compoundSpace_->getSubspaceCount() == 2, "Invalid number of subspaces");

  // Create components
  state->components[MODEL_BASED] = jointState;
  state->components[DISCRETE] = compoundSpace_->getSubspaces()[DISCRETE]->allocState();

  // Set level
  state->as<ob::DiscreteStateSpace::StateType>(DISCRETE)->value = level;

  return static_cast<base::State *>(state);
}

TaskVertex TaskGraph::addVertexWithLevel(base::State *state, VertexLevel level, std::size_t indent)
{
  // Create the state then add to the graph
  return addVertex(createCompoundState(state, level, indent), indent);
}

TaskVertex TaskGraph::addVertex(base::State *state, std::size_t indent)
{
  // Create vertex
  TaskVertex v = boost::add_vertex(g_);
  BOLT_FUNC(indent, vAdd_, "TaskGraph.addVertex(): v: " << v);

  // Add properties
  g_[v].state_ = state;

  // Add vertex to nearest neighbor structure - except only do this for level 0
  if (getTaskLevel(state) == 0)
  {
    nn_->add(v);
  }

  // Visualize
  if (visualizeTaskGraph_)
  {
    visualizeVertex(v);

    if (visualizeTaskGraphSpeed_ > std::numeric_limits<double>::epsilon())
    {
      visual_->viz2()->trigger();
      usleep(visualizeTaskGraphSpeed_ * 1000000);
    }
  }

  return v;
}

void TaskGraph::removeVertex(TaskVertex v)
{
  // Remove from nearest neighbor
  nn_->remove(v);

  // Delete state
  compoundSI_->freeState(g_[v].state_);
  g_[v].state_ = NULL;

  // Remove all edges to and from vertex
  boost::clear_vertex(v, g_);

  // We do not actually remove the vertex from the graph
  // because that would invalidate the nearest neighbor tree
}

void TaskGraph::removeDeletedVertices(std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "TaskGraph.removeDeletedVertices()");
  bool verbose = true;

  // Remove all vertices that are set to 0
  std::size_t numRemoved = 0;

  // Iterate manually through graph
  typedef boost::graph_traits<TaskAdjList>::vertex_iterator VertexIterator;
  for (VertexIterator v = boost::vertices(g_).first; v != boost::vertices(g_).second; /* manual */)
  {
    if (*v < numThreads_)  // Skip query vertices
    {
      v++;
      continue;
    }

    if (getCompoundState(*v) == NULL)  // Found vertex to delete
    {
      BOLT_DEBUG(indent, verbose, "Removing TaskVertex " << *v << " state: " << getCompoundState(*v));

      boost::remove_vertex(*v, g_);
      numRemoved++;
    }
    else  // only proceed if no deletion happened
    {
      v++;
    }
  }
  BOLT_DEBUG(indent, verbose, "Removed " << numRemoved << " vertices from graph that were abandoned");

  if (numRemoved == 0)
  {
    BOLT_DEBUG(indent, verbose, "No verticies deleted, skipping resetting NN");
    return;
  }

  // Reset the nearest neighbor tree
  nn_->clear();

  // Reinsert vertices into nearest neighbor
  foreach (TaskVertex v, boost::vertices(g_))
  {
    if (v < numThreads_)  // Skip the query vertices
      continue;

    nn_->add(v);
  }
}

TaskEdge TaskGraph::addEdge(TaskVertex v1, TaskVertex v2, std::size_t indent)
{
  // BOLT_FUNC(indent, vAdd_, "TaskGraph.addEdge(): from vertex " << v1 << " to " << v2);

  if (superDebug_)  // Extra checks
  {
    BOLT_ASSERT(v1 <= getNumVertices(), "Vertex1 is larger than max vertex id");
    BOLT_ASSERT(v2 <= getNumVertices(), "Vertex2 is larger than max vertex id");
    BOLT_ASSERT(v1 >= sg_->getNumQueryVertices(), "Found query vertex in sparse graph that has an edge!");
    BOLT_ASSERT(v2 >= sg_->getNumQueryVertices(), "Found query vertex in sparse graph that has an edge!");
    BOLT_ASSERT(v1 != v2, "Verticex IDs are the same");
    BOLT_ASSERT(!hasEdge(v1, v2), "There already exists an edge between two vertices requested");
    BOLT_ASSERT(hasEdge(v1, v2) == hasEdge(v2, v1), "There already exists an edge between two vertices requested, "
                                                    "other direction");
    BOLT_ASSERT(getCompoundState(v1) != getCompoundState(v2), "States on both sides of an edge are the same");
    BOLT_ASSERT(!compoundSI_->getStateSpace()->equalStates(getCompoundState(v1), getCompoundState(v2)), "Vertex IDs are different but "
                                                                                        "states are the equal");
  }

  // Create the new edge
  TaskEdge e = (boost::add_edge(v1, v2, g_)).first;

  // Weight properties
  g_[e].weight_ = distanceVertex(v1, v2);

  // Collision properties
  g_[e].collision_state_ = NOT_CHECKED;

  // Visualize
  if (visualizeTaskGraph_)
  {
    visualizeEdge(v1, v2);

    if (visualizeTaskGraphSpeed_ > std::numeric_limits<double>::epsilon())
    {
      visual_->viz2()->trigger();
      usleep(visualizeTaskGraphSpeed_ * 1000000);
    }
  }

  return e;
}

bool TaskGraph::hasEdge(TaskVertex v1, TaskVertex v2)
{
  return boost::edge(v1, v2, g_).second;
}

void TaskGraph::displayDatabase(bool showVertices, std::size_t indent)
{
  BOLT_FUNC(indent, vVisualize_, "TaskGraph.displayDatabase()");

  // Error check
  if (getNumVertices() == 0 || getNumEdges() == 0)
  {
    OMPL_WARN("Unable to show database because no vertices/edges available");
    return;
  }

  // Clear previous visualization
  visual_->viz2()->deleteAllMarkers();

  const std::size_t MIN_FEEDBACK = 10000;
  if (visualizeDatabaseEdges_)
  {
    // Loop through each edge
    std::size_t count = 1;
    std::size_t debugFrequency = MIN_FEEDBACK;
    if (getNumEdges() > MIN_FEEDBACK)
    {
      std::string(indent, ' ');
      std::cout << "Displaying task edges: " << std::flush;
    }
    foreach (TaskEdge e, boost::edges(g_))
    {
      // Visualize
      visualizeEdge(e);

      // Prevent viz cache from getting too big
      if (count % debugFrequency == 0)
      {
        std::cout << static_cast<int>((static_cast<double>(count + 1) / getNumEdges()) * 100.0) << "% " << std::flush;
        visual_->viz2()->trigger();
        usleep(0.01 * 1000000);
      }

      count++;
    }
  }

  if (visualizeDatabaseVertices_)
  {
    // Loop through each vertex
    std::size_t count = 1;
    std::size_t debugFrequency = MIN_FEEDBACK;  // getNumVertices() / 10;
    if (getNumVertices() > MIN_FEEDBACK)
    {
      std::string(indent, ' ');
      std::cout << "Displaying task vertices: " << std::flush;
    }
    foreach (TaskVertex v, boost::vertices(g_))
    {
      // Skip query vertices
      if (v < queryVertices_.size())
        continue;

      // Skip deleted vertices
      if (g_[v].state_ == NULL)
      {
        std::cout << "skipped deleted vertex " << v << std::endl;
        continue;
      }

      // Check for null states
      if (!getCompoundState(v))
      {
        BOLT_ERROR(indent, "Null vertex found: " << v);
        continue;
      }

      // Visualize
      visualizeVertex(v);

      // Prevent viz cache from getting too big
      if (count % debugFrequency == 0)
      {
        std::cout << static_cast<int>((static_cast<double>(count + 1) / getNumVertices()) * 100.0) << "% "
                  << std::flush;
        visual_->viz2()->trigger();
        // usleep(0.01 * 1000000);
      }
      count++;
    }
  }

  // Publish remaining edges
  visual_->viz2()->trigger();
  usleep(0.001 * 1000000);
}

void TaskGraph::visualizeVertex(TaskVertex v, std::size_t windowID)
{
  tools::VizColors color;
  tools::VizSizes size;

  VertexLevel level = getTaskLevel(v);

  switch (level)
  {
    case 0:
      color = tools::BLUE;
      size = tools::LARGE;
      break;
    case 1:
      color = tools::RED;
      size = tools::LARGE;
      break;
    case 2:
      color = tools::GREEN;
      size = tools::LARGE;
      break;
    default:
      throw Exception(name_, "Unknown vertex levle");
  }

  // Show vertex
  visual_->viz(windowID)->state(getModelBasedState(v), size, color, 0);

  // Show robot state
  // visual_->viz(windowID)->state(getModelBasedState(v), tools::ROBOT, tools::DEFAULT, 0);
}

void TaskGraph::visualizeEdge(TaskEdge e, std::size_t windowID)
{
  // Add edge
  TaskVertex v1 = boost::source(e, g_);
  TaskVertex v2 = boost::target(e, g_);

  visualizeEdge(v1, v2, windowID);
}

void TaskGraph::visualizeEdge(TaskVertex v1, TaskVertex v2, std::size_t windowID)
{
  VertexLevel level1 = getTaskLevel(v1);
  VertexLevel level2 = getTaskLevel(v2);
  ompl::tools::VizColors color = DEFAULT;

  if (level1 == 0 && level2 == 0)
    color = BLUE;
  else if (level1 == 1 && level2 == 1)
    color = RED;
  else if (level1 == 2 && level2 == 2)
    color = GREEN;
  else if (level1 != level2)
    color = ORANGE;
  else
    OMPL_ERROR("Unknown task level combination");

  // Visualize
  visual_->viz(windowID)->edge(getModelBasedState(v1), getModelBasedState(v2), ompl::tools::MEDIUM, color);
}

void TaskGraph::debugState(const ompl::base::State *state)
{
  compoundSI_->printState(state, std::cout);
}

void TaskGraph::debugVertex(const TaskVertex v)
{
  debugState(getCompoundState(v));
}

void TaskGraph::debugNN()
{
  // Show contents of GNAT
  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  NearestNeighborsGNAT<TaskVertex> *gnat = dynamic_cast<NearestNeighborsGNAT<TaskVertex> *>(nn_.get());
  std::cout << "GNAT: " << *gnat << std::endl;
  std::cout << std::endl;
}

void TaskGraph::printGraphStats(double generationDuration, std::size_t indent)
{
  // Get the average vertex degree (number of connected edges)
  double averageDegree = (getNumEdges() * 2) / static_cast<double>(getNumVertices());

  // Find min, max, and average edge length
  double totalEdgeLength = 0;
  double maxEdgeLength = -1 * std::numeric_limits<double>::infinity();
  double minEdgeLength = std::numeric_limits<double>::infinity();
  foreach (const TaskEdge e, boost::edges(g_))
  {
    const double length = g_[e].weight_;
    totalEdgeLength += length;
    if (maxEdgeLength < length)
      maxEdgeLength = length;
    if (minEdgeLength > length)
      minEdgeLength = length;
  }
  double averageEdgeLength = getNumEdges() ? totalEdgeLength / getNumEdges() : 0;

  BOLT_INFO(indent, 1, "---------------------------------------------");
  BOLT_INFO(indent, 1, "TaskGraph stats:");
  BOLT_INFO(indent, 1, "   Generation time:        " << generationDuration << " s");
  BOLT_INFO(indent, 1, "   Total vertices:         " << getNumRealVertices());
  BOLT_INFO(indent, 1, "   Total edges:            " << getNumEdges());
  BOLT_INFO(indent, 1, "   Average degree:         " << averageDegree);
  BOLT_INFO(indent, 1, "   Edge Lengths:           ");
  BOLT_INFO(indent, 1, "      Max:                 " << maxEdgeLength);
  BOLT_INFO(indent, 1, "      Min:                 " << minEdgeLength);
  BOLT_INFO(indent, 1, "      Average:             " << averageEdgeLength);
  BOLT_INFO(indent, 1, "---------------------------------------------");
}

bool TaskGraph::checkMotion(const base::State *a, const base::State *b)
{
  return modelSI_->checkMotion(getModelBasedState(a), getModelBasedState(b));
}

geometric::PathGeometricPtr TaskGraph::convertPathToNonCompound(const geometric::PathGeometricPtr compoundPathGeometric)
{
  // Convert input path
  //og::PathGeometric &compoundPathGeometric = static_cast<og::PathGeometric &>(*compoundPath);

  // Create new path
  geometric::PathGeometricPtr modelPath(new og::PathGeometric(modelSI_));
  //og::PathGeometric &modelPath = static_cast<og::PathGeometric &>(*modelPathBase);

  for (std::size_t i = 0; i < compoundPathGeometric->getStateCount(); ++i)
  {
    modelPath->append(getModelBasedState(compoundPathGeometric->getState(i)));
  }

  return modelPath;
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

// TaskEdgeWeightMap methods ////////////////////////////////////////////////////////////////////////////

namespace boost
{
double get(const ompl::tools::bolt::TaskEdgeWeightMap &m, const ompl::tools::bolt::TaskEdge &e)
{
  return m.get(e);
}
}

BOOST_CONCEPT_ASSERT(
    (boost::ReadablePropertyMapConcept<ompl::tools::bolt::TaskEdgeWeightMap, ompl::tools::bolt::TaskEdge>));

// TaskAstarVisitor methods ////////////////////////////////////////////////////////////////////////////

BOOST_CONCEPT_ASSERT((boost::AStarVisitorConcept<otb::TaskAstarVisitor, otb::TaskAdjList>));

otb::TaskAstarVisitor::TaskAstarVisitor(TaskVertex goal, TaskGraph *parent) : goal_(goal), parent_(parent)
{
}

#ifndef NDEBUG
void otb::TaskAstarVisitor::discover_vertex(TaskVertex v, const TaskAdjList &) const
{
  // Statistics
  parent_->recordNodeOpened();

  if (parent_->visualizeAstar_)
    parent_->getVisual()->viz4()->state(parent_->getModelBasedState(v), tools::SMALL, tools::GREEN, 1);
}
#endif

void otb::TaskAstarVisitor::examine_vertex(TaskVertex v, const TaskAdjList &) const
{
#ifndef NDEBUG
  parent_->recordNodeClosed();  // Statistics

  if (parent_->visualizeAstar_)  // Visualize
  {
    // Show state
    parent_->visualizeVertex(v, 4 /*windowID*/);

    // Show edges
    foreach (const TaskEdge e, boost::out_edges(v, parent_->getGraph()))
    {
      parent_->visualizeEdge(e, 4 /*windowID*/);
    }

    parent_->getVisual()->viz4()->trigger();
    // usleep(parent_->visualizeAstarSpeed_ * 1000000);
    parent_->getVisual()->waitForUserFeedback("astar");
  }
#endif

  if (v == goal_)
    throw FoundGoalException();
}
