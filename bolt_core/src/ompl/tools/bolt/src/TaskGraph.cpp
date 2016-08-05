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
#include <ompl/tools/bolt/TaskGraph.h>
#include <ompl/util/Console.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/base/DiscreteMotionValidator.h>

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
#include <algorithm>  // std::random_shuffle

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
TaskGraph::TaskGraph(SparseGraphPtr sg)
  : sg_(sg)
  // Property accessors of edges
  , edgeWeightProperty_(boost::get(boost::edge_weight, g_))
  , edgeCollisionStatePropertyTask_(boost::get(edge_collision_state_t(), g_))
  // Property accessors of vertices
  , vertexStateProperty_(boost::get(vertex_state_t(), g_))
  , vertexTypeProperty_(boost::get(vertex_type_t(), g_))
  , vertexTaskMirrorProperty_(boost::get(vertex_task_mirror_t(), g_))
  // Disjoint set accessors
  , disjointSets_(boost::get(boost::vertex_rank, g_), boost::get(boost::vertex_predecessor, g_))
{
  // Save number of threads available
  numThreads_ = boost::thread::hardware_concurrency();

  // Copy the pointers of various components
  si_ = sg_->getSpaceInformation();
  visual_ = sg_->getVisual();

  // Add search state
  initializeQueryState();

  // Initialize nearest neighbor datastructure
  // TODO(davetcoleman): do we need to have a separate NN_ structure for the TaskGraph??
  nn_.reset(new NearestNeighborsGNAT<TaskVertex>());
  nn_->setDistanceFunction(boost::bind(&otb::TaskGraph::distanceFunction, this, _1, _2));
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
    pathSimplifier_.reset(new geometric::PathSimplifier(si_));
    pathSimplifier_->freeStates(false);
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
    if (vertexStateProperty_[v] != nullptr)
      si_->freeState(vertexStateProperty_[v]);
    vertexStateProperty_[v] = nullptr;  // TODO(davetcoleman): is this needed??
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
    vertexStateProperty_[queryVertices_[threadID]] = NULL;  // set stateID to the zeroth stateID - NULL
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

  // Reset statistics
  numNodesOpened_ = 0;
  numNodesClosed_ = 0;

  if (visualizeAstar_)
  {
    // Assume this was cleared by the parent program
    // visual_->viz4()->deleteAllMarkers();
  }

  try
  {
    double popularityBias = 0;
    bool popularityBiasEnabled = false;
    boost::astar_search(
        g_,     // graph
        start,  // start state
        // boost::bind(&otb::TaskGraph::astarHeuristic, this, _1, goal),  // the heuristic
        boost::bind(&otb::TaskGraph::astarTaskHeuristic, this, _1, goal),  // the heuristic
        // ability to disable edges (set cost to inifinity):
        boost::weight_map(TaskEdgeWeightMap(g_, edgeCollisionStatePropertyTask_, popularityBias, popularityBiasEnabled))
            .predecessor_map(vertexPredecessors)
            .distance_map(&vertexDistances[0])
            .visitor(TaskAstarVisitor(goal, this)));
  }
  catch (FoundGoalException &)
  {
    distance = vertexDistances[goal];

    // the custom exception from TaskAstarVisitor
    BOLT_DEBUG(indent, vSearch_, "AStar found solution. Distance to goal: " << vertexDistances[goal]);
    BOLT_DEBUG(indent, vSearch_, "Number nodes opened: " << numNodesOpened_
                                                         << ", Number nodes closed: " << numNodesClosed_);

    if (isinf(vertexDistances[goal]))  // TODO(davetcoleman): test that this works
    {
      throw Exception(name_, "Distance to goal is infinity");
      foundGoal = false;
    }
    else
    {
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
  }

  if (!foundGoal)
    BOLT_WARN(indent, vSearch_, "Did not find goal");

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
        visual_->viz4()->edge(getState(v1), getState(v2), 10);
      }
    }
    visual_->viz4()->trigger();
  }

  // Unload
  delete[] vertexPredecessors;
  delete[] vertexDistances;

  // No solution found from start to goal
  return foundGoal;
}

double TaskGraph::astarHeuristic(const TaskVertex a, const TaskVertex b) const
{
  // Assume vertex 'a' is the one we care about its populariy

  // Get the classic distance
  double dist = si_->distance(getState(a), getState(b));

  // if (false)  // method 1
  // {
  //   const double percentMaxExtent = (maxExtent_ * percentMaxExtentUnderestimate_);  // TODO(davetcoleman): cache
  //   double popularityComponent = percentMaxExtent * (vertexPopularity_[a] / 100.0);

  //   std::cout << "astarHeuristic - dist: " << std::setprecision(4) << dist << ", popularity: " <<
  //   vertexPopularity_[a]
  //             << ", max extent: " << maxExtent_ << ", percentMaxExtent: " << percentMaxExtent
  //             << ", popularityComponent: " << popularityComponent;
  //   dist = std::max(0.0, dist - popularityComponent);
  // }
  // else if (false)  // method 2
  // {
  //   const double percentDist = (dist * percentMaxExtentUnderestimate_);  // TODO(davetcoleman): cache
  //   double popularityComponent = percentDist * (vertexPopularity_[a] / 100.0);

  //   std::cout << "astarHeuristic - dist: " << std::setprecision(4) << dist << ", popularity: " <<
  //   vertexPopularity_[a]
  //             << ", percentDist: " << percentDist << ", popularityComponent: " << popularityComponent;
  //   dist = std::max(0.0, dist - popularityComponent);
  // }
  // else if (false)  // method 3
  // {
  //   std::cout << "astarHeuristic - dist: " << std::setprecision(4) << dist << ", popularity: " <<
  //   vertexPopularity_[a]
  //             << ", vertexPopularity_[a] / 100.0: " << vertexPopularity_[a] / 100.0
  //             << ", percentMaxExtentUnderestimate_: " << percentMaxExtentUnderestimate_;
  //   // if ((vertexPopularity_[a] / 100.0) < (1 - percentMaxExtentUnderestimate_))
  //   if (vertexPopularity_[a] > (100 - percentMaxExtentUnderestimate_ * 100.0))
  //   {
  //     dist = 0;
  //   }

  //   // dist = std::max(0.0, dist - popularityComponent);
  // }
  // else if (false)  // method 4
  // {
  //   dist *= (1 + percentMaxExtentUnderestimate_);
  // }
  // method 5: increasing the sparseDelta fraction

  // std::cout << ", new distance: " << dist << std::endl;

  return dist;
}

double TaskGraph::distanceFunction(const TaskVertex a, const TaskVertex b) const
{
  // Special case: query vertices store their states elsewhere. Both cannot be query vertices
  if (a < numThreads_)
  {
    return si_->distance(queryStates_[a], getState(b));
  }
  if (b < numThreads_)
  {
    return si_->distance(getState(a), queryStates_[b]);
  }

  assert(getState(a) != NULL);
  assert(getState(b) != NULL);

  return si_->distance(getState(a), getState(b));
}

double TaskGraph::astarTaskHeuristic(const TaskVertex a, const TaskVertex b) const
{
  // Do not use task distance if that mode is not enabled
  if (!taskPlanningEnabled_)
    return distanceFunction(a, b);

  std::size_t indent = 0;

  // Reorder a & b so that we are sure that a.level <= b.level
  VertexLevel taskLevelA = getTaskLevel(a);
  VertexLevel taskLevelB = getTaskLevel(b);
  if (taskLevelA > taskLevelB)
  {
    // Call itself again, this time switching ordering
    BOLT_DEBUG(indent, vHeuristic_, "Switched ordering for distanceFunctionTasks()");
    return astarTaskHeuristic(b, a);
  }

  double dist = 0;  // the result

  // Note: this value should be synced with TaskGraph
  static const double TASK_LEVEL_COST = 100.0;  // cost to change levels/tasks

  // Error check
  assert(vertexStateProperty_[a]);
  assert(vertexStateProperty_[b]);
  assert(vertexStateProperty_[startConnectorVertex_]);
  assert(vertexStateProperty_[goalConnectorVertex_]);

  if (taskLevelA == 0)
  {
    if (taskLevelB == 0)  // regular distance for bottom level
    {
      BOLT_DEBUG(indent, vHeuristic_, "Distance Mode a");
      dist = si_->distance(vertexStateProperty_[a], vertexStateProperty_[b]);
    }
    else if (taskLevelB == 1)
    {
      BOLT_DEBUG(indent, vHeuristic_, "Distance Mode b");
      dist = si_->distance(vertexStateProperty_[a], vertexStateProperty_[startConnectorVertex_]) + TASK_LEVEL_COST +
             si_->distance(vertexStateProperty_[startConnectorVertex_], vertexStateProperty_[b]);
    }
    else if (taskLevelB == 2)
    {
      BOLT_DEBUG(indent, vHeuristic_, "Distance Mode c");
      dist = si_->distance(vertexStateProperty_[a], vertexStateProperty_[startConnectorVertex_]) + TASK_LEVEL_COST +
             shortestDistAcrossCartGraph_ + TASK_LEVEL_COST +
             si_->distance(vertexStateProperty_[goalConnectorVertex_], vertexStateProperty_[b]);
    }
    else
    {
      OMPL_ERROR("Unknown task level mode");
      exit(-1);
    }
  }
  else if (taskLevelA == 1)
  {
    if (taskLevelB == 0)
    {
      OMPL_ERROR("Unknown task level mode");
      exit(-1);
    }
    else if (taskLevelB == 1)
    {
      BOLT_DEBUG(indent, vHeuristic_, "Distance Mode d");
      dist = si_->distance(vertexStateProperty_[a], vertexStateProperty_[b]);
    }
    else if (taskLevelB == 2)
    {
      BOLT_DEBUG(indent, vHeuristic_, "Distance Mode e");

      dist = si_->distance(vertexStateProperty_[a], vertexStateProperty_[goalConnectorVertex_]) + TASK_LEVEL_COST +
             si_->distance(vertexStateProperty_[goalConnectorVertex_], vertexStateProperty_[b]);
    }
    else
    {
      OMPL_WARN("Unknown task level mode");
    }
  }
  else if (taskLevelA == 2)
  {
    if (taskLevelB == 0 || taskLevelB == 1)
    {
      OMPL_ERROR("Unknown task level mode");
      exit(-1);
    }
    else if (taskLevelB == 2)
    {
      BOLT_DEBUG(indent, vHeuristic_, "Distance Mode f");
      dist = si_->distance(vertexStateProperty_[a], vertexStateProperty_[b]);
    }
    else
    {
      OMPL_WARN("Unknown task level mode");
    }
  }
  else
  {
    OMPL_WARN("Unknown task level mode");
  }

  BOLT_DEBUG(indent, vHeuristic_, "Vertex " << a << " @level " << taskLevelA << " to Vertex " << b << " @level "
                                            << taskLevelB << " has distance " << dist);
  return dist;
}

bool TaskGraph::isEmpty() const
{
  assert(!(getNumVertices() < getNumQueryVertices()));
  return (getNumVertices() == getNumQueryVertices() && getNumEdges() == 0);
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
  std::vector<TaskVertex> sparseToTaskVertex1(sg_->getNumVertices());
  std::vector<TaskVertex> sparseToTaskVertex2(sg_->getNumVertices());

  // Loop through every vertex in sparse graph and copy twice to task graph
  BOLT_DEBUG(indent + 2, vGenerateTask_, "Adding task space vertices");
  foreach (SparseVertex sparseV, boost::vertices(sg_->getGraph()))
  {
    // The first thread number of verticies are used for queries and should be skipped
    if (sparseV < sg_->getNumQueryVertices())
      continue;

    const VertexType type = DISCRETIZED;  // TODO: remove this, seems meaningless
    const base::State *state = sg_->getState(sparseV);

    // Create level 0 vertex
    VertexLevel level = 0;
    TaskVertex taskV1 = addVertex(si_->cloneState(state), type, level, indent);
    sparseToTaskVertex1[sparseV] = taskV1;  // record mapping

    // Create level 2 vertex
    level = 2;
    TaskVertex taskV2 = addVertex(si_->cloneState(state), type, level, indent);
    sparseToTaskVertex2[sparseV] = taskV2;  // record mapping

    // Link the two vertices to each other for future bookkeeping
    vertexTaskMirrorProperty_[taskV1] = taskV2;
    vertexTaskMirrorProperty_[taskV2] = taskV1;
  }

  // Loop through every edge in sparse graph and copy twice to task graph
  BOLT_DEBUG(indent + 2, vGenerateTask_, "Adding task space edges");
  foreach (const SparseEdge sparseE, boost::edges(sg_->getGraph()))
  {
    const SparseVertex sparseE_v1 = boost::source(sparseE, sg_->getGraph());
    const SparseVertex sparseE_v2 = boost::target(sparseE, sg_->getGraph());
    EdgeType type = sg_->getEdgeTypeProperty(sparseE);

    // Error check
    BOOST_ASSERT_MSG(sparseE_v1 >= sg_->getNumQueryVertices(), "Found query vertex in sparse graph that has an edge!");
    BOOST_ASSERT_MSG(sparseE_v2 >= sg_->getNumQueryVertices(), "Found query vertex in sparse graph that has an edge!");

    // Create level 0 edge
    // TaskEdge taskEdge1 =
    addEdge(sparseToTaskVertex1[sparseE_v1], sparseToTaskVertex1[sparseE_v2], type, indent);

    // Create level 2 edge
    // TaskEdge taskEdge2 =
    addEdge(sparseToTaskVertex2[sparseE_v1], sparseToTaskVertex2[sparseE_v2], type, indent);
  }

  // Visualize
  // displayDatabase();

  printGraphStats();

  BOLT_DEBUG(indent, verbose_, "Generated task space in " << time::seconds(time::now() - startTime) << " seconds");
}

bool TaskGraph::addCartPath(std::vector<base::State *> path, std::size_t indent)
{
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
  TaskVertex startVertex = addVertex(path.front(), CARTESIAN, level, indent);
  TaskVertex goalVertex = addVertex(path.back(), CARTESIAN, level, indent);

  // Record min cost for cost-to-go heurstic distance function later
  shortestDistAcrossCartGraph_ = distanceFunction(startVertex, goalVertex);

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
      v2 = addVertex(path[i], CARTESIAN, cartLevel, indent);
    }

    addEdge(v1, v2, eCARTESIAN, indent);
    v1 = v2;
  }

  // Tell the planner to require task planning
  taskPlanningEnabled_ = true;

  return true;
}

void TaskGraph::clearCartesianVerticesDeprecated(std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "TaskGraph.clearCartesianVerticesDeprecated()");

  time::point startTime = time::now();  // Benchmark

  // Remove all vertices that are of type CARTESIAN
  std::size_t numRemoved = 0;

  // Iterate manually through graph
  typedef boost::graph_traits<TaskAdjList>::vertex_iterator VertexIterator;
  for (VertexIterator v = boost::vertices(g_).first; v != boost::vertices(g_).second; /* manually iterate */)
  {
    if (*v < getNumQueryVertices())  // Skip query vertices
    {
      v++;
      continue;
    }

    if (getVertexTypeProperty(*v) == CARTESIAN)  // Found vertex to delete
    {
      BOLT_DEBUG(indent, vClear_, "Removing CARTESIAN TaskVertex " << *v << " total removed: " << numRemoved);

      boost::clear_vertex(*v, g_);  // delete the edges
      boost::remove_vertex(*v, g_);

      numRemoved++;
    }
    else  // only proceed if no deletion happened
    {
      v++;
    }
  }
  BOLT_DEBUG(indent, verbose_, "Removed " << numRemoved << " vertices from graph of type CARTESIAN");
  OMPL_INFORM("took %f seconds", time::seconds(time::now() - startTime));  // Benchmark

  startTime = time::now();  // Benchmark

  // Reset min connector vars
  startConnectorVertex_ = 0;
  goalConnectorVertex_ = 0;
  startConnectorMinCost_ = std::numeric_limits<double>::infinity();
  goalConnectorMinCost_ = std::numeric_limits<double>::infinity();

  if (numRemoved == 0)
  {
    BOLT_DEBUG(indent, vClear_, "No verticies deleted, skipping resetting NN and disjointSets");
    return;
  }

  // Reset the nearest neighbor tree
  nn_->clear();

  // Reset disjoint sets
  disjointSets_ = TaskDisjointSetType(boost::get(boost::vertex_rank, g_), boost::get(boost::vertex_predecessor, g_));

  // Reinsert vertices into nearest neighbor
  foreach (TaskVertex v, boost::vertices(g_))
  {
    if (v < numThreads_)  // Skip the query vertices
      continue;

    if (getTaskLevel(v) == 0)
      nn_->add(v);

    disjointSets_.make_set(v);
  }

  // Reinsert edges into disjoint sets
  foreach (TaskEdge e, boost::edges(g_))
  {
    TaskVertex v1 = boost::source(e, g_);
    TaskVertex v2 = boost::target(e, g_);
    disjointSets_.union_set(v1, v2);
  }

  // Clear the visualization and redisplay
  // bool showVertices = true;
  // displayDatabase(showVertices, indent);

  BOLT_DEBUG(indent, verbose_, "TaskGraph.clearCartesianVerticesDeprecated() completed");
  OMPL_INFORM("took %f seconds", time::seconds(time::now() - startTime));  // Benchmark
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
    BOLT_ERROR(indent, vGenerateTask_, "No neighbors found when connecting cartesian path");
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
    double connectorCost = distanceFunction(fromVertex, v);
    addEdge(fromVertex, v, eCARTESIAN, indent);

    // Remember which cartesian start/goal states should be used for distanceFunction
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

  BOOST_ASSERT_MSG(level != 1, "Unhandled level, does not support level 1");

  const std::size_t threadID = 0;
  base::State *origState = getStateNonConst(origVertex);

  // Get nearby state
  queryStates_[threadID] = origState;
  nn_->nearestK(queryVertices_[threadID], kNeighbors, neighbors);
  queryStates_[threadID] = nullptr;

  // Run various checks
  for (std::size_t i = 0; i < neighbors.size(); ++i)
  {
    TaskVertex nearVertex = neighbors[i];

    bool useCollisionChecking = false;

    if (useCollisionChecking)
    {
      // Collision check
      if (!si_->checkMotion(origState, getState(nearVertex)))  // is not valid motion
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

  // Convert our list of neighbors to the proper level
  if (level == 2)
  {
    BOLT_DEBUG(indent, vGenerateTask_, "Converting vector of level 0 neighbors to level 2 neighbors");

    for (std::size_t i = 0; i < neighbors.size(); ++i)
    {
      const TaskVertex nearVertex = neighbors[i];

      // Get the vertex on the opposite level and replace it in the vector
      neighbors[i] = vertexTaskMirrorProperty_[nearVertex];
    }
  }
}

bool TaskGraph::checkTaskPathSolution(og::PathGeometric &path, ob::State *start, ob::State *goal)
{
  bool error = false;
  VertexLevel current_level = 0;

  for (std::size_t i = 0; i < path.getStateCount(); ++i)
  {
    VertexLevel level = si_->getStateSpace()->getLevel(path.getState(i));

    // Check if start state is correct
    if (i == 0)
    {
      if (!si_->getStateSpace()->equalStates(path.getState(i), start))
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
      if (!si_->getStateSpace()->equalStates(path.getState(i), goal))
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
      VertexLevel level = si_->getStateSpace()->getLevel(path.getState(i));
      OMPL_INFORM(" - Path state %i has level %i", i, level);
    }
  }

  return error;
}

void TaskGraph::clearEdgeCollisionStates()
{
  foreach (const TaskEdge e, boost::edges(g_))
    edgeCollisionStatePropertyTask_[e] = NOT_CHECKED;  // each edge has an unknown state
}

void TaskGraph::errorCheckDuplicateStates(std::size_t indent)
{
  BOLT_ERROR(indent, verbose_, "TaskGraph.errorCheckDuplicateStates() - NOT IMPLEMENTEDpart of super debug");

  // bool found = false;
  // // Error checking: check for any duplicate states
  // for (std::size_t i = 0; i < dense_Cache_->getStateCacheSize(); ++i)
  // {
  //   for (std::size_t j = i + 1; j < dense_Cache_->getStateCacheSize(); ++j)
  //   {
  //     if (si_->getStateSpace()->equalStates(getState(i), getState(j)))
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

bool TaskGraph::smoothQualityPathOriginal(geometric::PathGeometric *path, std::size_t indent)
{
  BOLT_ERROR(indent, visualizeQualityPathSimp_, "smoothQualityPathOriginal()");

  // Visualize path
  if (visualizeQualityPathSimp_)
  {
    visual_->viz2()->deleteAllMarkers();
    visual_->viz2()->path(path, tools::SMALL, tools::BLUE);
    visual_->viz2()->trigger();
    usleep(0.001 * 1000000);
  }

  BOLT_DEBUG(indent, visualizeQualityPathSimp_, "Created 'quality path' candidate with " << path->getStateCount()
                                                                                         << " states");
  if (visualizeQualityPathSimp_)
    visual_->waitForUserFeedback("path simplification");

  pathSimplifier_->reduceVertices(*path, 10);
  pathSimplifier_->shortcutPath(*path, 50);

  std::pair<bool, bool> repairResult = path->checkAndRepair(100);

  if (!repairResult.second)  // Repairing was not successful
  {
    throw Exception(name_, "check and repair failed?");
  }
  return true;
}

bool TaskGraph::smoothQualityPath(geometric::PathGeometric *path, double clearance, std::size_t indent)
{
  BOLT_FUNC(indent, visualizeQualityPathSimp_, "TaskGraph.smoothQualityPath()");

  // Visualize path
  if (visualizeQualityPathSimp_)
  {
    visual_->viz2()->deleteAllMarkers();
    visual_->viz2()->path(path, tools::SMALL, tools::BLUE);
    visual_->viz2()->trigger();
    usleep(0.001 * 1000000);
  }

  BOLT_DEBUG(indent, visualizeQualityPathSimp_, "Created 'quality path' candidate with " << path->getStateCount()
                                                                                         << " states");
  if (visualizeQualityPathSimp_)
    visual_->waitForUserFeedback("path simplification");

  // Set the motion validator to use clearance, this way isValid() checks clearance before confirming valid
  base::DiscreteMotionValidator *dmv =
      dynamic_cast<base::DiscreteMotionValidator *>(si_->getMotionValidatorNonConst().get());
  dmv->setRequiredStateClearance(clearance);

  for (std::size_t i = 0; i < 3; ++i)
  {
    pathSimplifier_->simplifyMax(*path);

    if (visualizeQualityPathSimp_)
    {
      visual_->viz2()->deleteAllMarkers();
      visual_->viz2()->path(path, tools::SMALL, tools::ORANGE);
      visual_->viz2()->trigger();
      usleep(0.1 * 1000000);
      // visual_->waitForUserFeedback("optimizing path");
    }

    pathSimplifier_->reduceVertices(*path, 1000, path->getStateCount() * 4);

    if (visualizeQualityPathSimp_)
    {
      visual_->viz2()->deleteAllMarkers();
      visual_->viz2()->path(path, tools::SMALL, tools::BLUE);
      visual_->viz2()->trigger();
      usleep(0.1 * 1000000);
      // visual_->waitForUserFeedback("optimizing path");
    }
  }
  // Turn off the clearance requirement
  dmv->setRequiredStateClearance(0.0);

  pathSimplifier_->reduceVertices(*path, 1000, path->getStateCount() * 4);

  if (visualizeQualityPathSimp_)
  {
    visual_->viz2()->deleteAllMarkers();
    visual_->viz2()->path(path, tools::SMALL, tools::GREEN);
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

std::size_t TaskGraph::getDisjointSetsCount(bool verbose)
{
  std::size_t numSets = 0;
  foreach (TaskVertex v, boost::vertices(g_))
  {
    // Do not count the search vertex within the sets
    if (v <= queryVertices_.back())
      continue;

    if (boost::get(boost::get(boost::vertex_predecessor, g_), v) == v)
    {
      std::size_t indent = 0;
      BOLT_DEBUG(indent, verbose, "Disjoint set: " << v);
      ++numSets;
    }
  }

  return numSets;
}

void TaskGraph::getDisjointSets(TaskDisjointSetsMap &disjointSets)
{
  disjointSets.clear();

  // Flatten the parents tree so that the parent of every element is its representative.
  disjointSets_.compress_sets(boost::vertices(g_).first, boost::vertices(g_).second);

  // Count size of each disjoint set and group its containing vertices
  typedef boost::graph_traits<TaskAdjList>::vertex_iterator VertexIterator;
  for (VertexIterator v = boost::vertices(g_).first; v != boost::vertices(g_).second; ++v)
  {
    // Do not count the search vertex within the sets
    if (*v <= queryVertices_.back())
      continue;

    disjointSets[boost::get(boost::get(boost::vertex_predecessor, g_), *v)].push_back(*v);
  }
}

void TaskGraph::printDisjointSets(TaskDisjointSetsMap &disjointSets, std::size_t indent)
{
  BOLT_DEBUG(indent, verbose_, "Print disjoint sets");
  for (TaskDisjointSetsMap::const_iterator iterator = disjointSets.begin(); iterator != disjointSets.end(); iterator++)
  {
    const TaskVertex v = iterator->first;
    const std::size_t freq = iterator->second.size();
    BOLT_DEBUG(indent, verbose_, "Parent: " << v << " frequency " << freq);
  }
}

void TaskGraph::visualizeDisjointSets(TaskDisjointSetsMap &disjointSets, std::size_t indent)
{
  BOLT_DEBUG(indent, verbose_, "Visualizing disjoint sets");

  // Find the disjoint set that is the 'main' large one
  std::size_t maxDisjointSetSize = 0;
  TaskVertex maxDisjointSetParent;
  for (TaskDisjointSetsMap::const_iterator iterator = disjointSets.begin(); iterator != disjointSets.end(); iterator++)
  {
    const TaskVertex v = iterator->first;
    const std::size_t freq = iterator->second.size();

    if (freq > maxDisjointSetSize)
    {
      maxDisjointSetSize = freq;
      maxDisjointSetParent = v;
    }
  }
  BOLT_DEBUG(indent, verbose_, "The largest disjoint set is of size " << maxDisjointSetSize << " and parent vertex "
                                                                      << maxDisjointSetParent);

  // Display size of disjoint sets and visualize small ones
  for (TaskDisjointSetsMap::const_iterator iterator = disjointSets.begin(); iterator != disjointSets.end(); iterator++)
  {
    const TaskVertex v1 = iterator->first;
    const std::size_t freq = iterator->second.size();

    BOOST_ASSERT_MSG(freq > 0, "Frequency must be at least 1");

    if (freq == maxDisjointSetSize)  // any subgraph that is smaller than the full graph
      continue;                      // the main disjoint set is not considered a disjoint set

    // Visualize sets of size one
    if (freq == 1)
    {
      visual_->viz5()->state(getState(v1), tools::LARGE, tools::RED, 0);
      visual_->viz5()->trigger();
      visual_->waitForUserFeedback("showing disjoint set");
      continue;
    }

    // Visualize large disjoint sets (greater than one)
    if (freq > 1 && freq < 1000)
    {
      // Clear markers
      visual_->viz4()->deleteAllMarkers();

      // Visualize this subgraph that is disconnected
      // Loop through every every vertex and check if its part of this group
      typedef boost::graph_traits<TaskAdjList>::vertex_iterator VertexIterator;
      for (VertexIterator v2 = boost::vertices(g_).first; v2 != boost::vertices(g_).second; ++v2)
      {
        if (boost::get(boost::get(boost::vertex_predecessor, g_), *v2) == v1)
        {
          visual_->viz4()->state(getState(*v2), tools::LARGE, tools::RED, 0);

          // Show state's edges
          foreach (TaskEdge edge, boost::out_edges(*v2, g_))
          {
            TaskVertex e_v1 = boost::source(edge, g_);
            TaskVertex e_v2 = boost::target(edge, g_);
            visual_->viz4()->edge(getState(e_v1), getState(e_v2), edgeWeightProperty_[edge]);
          }
          visual_->viz4()->trigger();

          // Show this robot state
          // visual_->viz4()->state(getState(*v2), tools::ROBOT, tools::DEFAULT, 0);
          visual_->viz4()->state(getState(*v2), tools::SMALL, tools::RED, 0);

          usleep(0.1 * 1000000);
        }  // if
      }    // for

      visual_->waitForUserFeedback("showing large disjoint set");
    }  // if
  }
}

std::size_t TaskGraph::checkConnectedComponents()
{
  // Check how many disjoint sets are in the task graph (should be none)
  std::size_t numSets = getDisjointSetsCount();
  if (numSets > 1)
  {
    std::size_t indent = 0;
    BOLT_WARN(indent, true, "More than 1 connected component is in the TaskGraph: " << numSets);
  }

  return numSets;
}

bool TaskGraph::sameComponent(TaskVertex v1, TaskVertex v2)
{
  return boost::same_component(v1, v2, disjointSets_);
}

TaskVertex TaskGraph::addVertex(base::State *state, const VertexType &type, VertexLevel level, std::size_t indent)
{
  // Create vertex
  TaskVertex v = boost::add_vertex(g_);
  BOLT_FUNC(indent, vAdd_, "TaskGraph.addVertex(): v: " << v << " type " << type << " level: " << level);

  // Add level to state
  setStateTaskLevel(state, level);

  // Add properties
  vertexTypeProperty_[v] = type;  // TODO(davetcoleman): remove?
  vertexStateProperty_[v] = state;

  // Connected component tracking
  disjointSets_.make_set(v);

  // Add vertex to nearest neighbor structure - except only do this for level 0
  if (level == 0)
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
  si_->freeState(vertexStateProperty_[v]);
  vertexStateProperty_[v] = NULL;

  // TODO: disjointSets is now inaccurate
  // Our checkAddConnectivity() criteria is broken
  // because we frequntly delete edges and nodes..
  // disjointSets_.remove_set(v);

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

    if (getState(*v) == NULL)  // Found vertex to delete
    {
      BOLT_DEBUG(indent, verbose, "Removing TaskVertex " << *v << " state: " << getState(*v));

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
    BOLT_DEBUG(indent, verbose, "No verticies deleted, skipping resetting NN and disjointSets");
    return;
  }

  // Reset the nearest neighbor tree
  nn_->clear();

  // Reset disjoint sets
  disjointSets_ = TaskDisjointSetType(boost::get(boost::vertex_rank, g_), boost::get(boost::vertex_predecessor, g_));

  // Reinsert vertices into nearest neighbor
  foreach (TaskVertex v, boost::vertices(g_))
  {
    if (v < numThreads_)  // Skip the query vertices
      continue;

    nn_->add(v);
    disjointSets_.make_set(v);
  }

  // Reinsert edges into disjoint sets
  foreach (TaskEdge e, boost::edges(g_))
  {
    TaskVertex v1 = boost::source(e, g_);
    TaskVertex v2 = boost::target(e, g_);
    disjointSets_.union_set(v1, v2);
  }
}

TaskEdge TaskGraph::addEdge(TaskVertex v1, TaskVertex v2, EdgeType type, std::size_t indent)
{
  BOLT_FUNC(indent, vAdd_, "TaskGraph.addEdge(): from vertex " << v1 << " to " << v2 << " type " << type);

  BOOST_ASSERT_MSG(v1 <= getNumVertices(), "Vertex1 is larger than max vertex id");
  BOOST_ASSERT_MSG(v2 <= getNumVertices(), "Vertex2 is larger than max vertex id");
  BOOST_ASSERT_MSG(v1 != v2, "Verticex IDs are the same");
  BOOST_ASSERT_MSG(!hasEdge(v1, v2), "There already exists an edge between two vertices requested");
  BOOST_ASSERT_MSG(hasEdge(v1, v2) == hasEdge(v2, v1), "There already exists an edge between two vertices requested, "
                                                       "other direction");
  BOOST_ASSERT_MSG(getState(v1) != getState(v2), "States on both sides of an edge are the same");
  BOOST_ASSERT_MSG(!si_->getStateSpace()->equalStates(getState(v1), getState(v2)), "Vertex IDs are different but "
                                                                                   "states are the equal");

  // Create the new edge
  TaskEdge e = (boost::add_edge(v1, v2, g_)).first;

  // Weight properties
  edgeWeightProperty_[e] = distanceFunction(v1, v2);

  // Collision properties
  edgeCollisionStatePropertyTask_[e] = NOT_CHECKED;

  // Add the edge to the incrementeal connected components datastructure
  disjointSets_.union_set(v1, v2);

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

base::State *&TaskGraph::getQueryStateNonConst(TaskVertex v)
{
  BOOST_ASSERT_MSG(v < queryVertices_.size(), "Attempted to request state of regular vertex using query function");
  return queryStates_[v];
}

base::State *&TaskGraph::getStateNonConst(TaskVertex v)
{
  BOOST_ASSERT_MSG(v >= queryVertices_.size(), "Attempted to request state of query vertex using wrong function");
  return vertexStateProperty_[v];
}

const base::State *TaskGraph::getState(TaskVertex v) const
{
  BOOST_ASSERT_MSG(v >= queryVertices_.size(), "Attempted to request state of query vertex using wrong function");
  return vertexStateProperty_[v];
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
      if (vertexStateProperty_[v] == NULL)
      {
        std::cout << "skipped deleted vertex " << v << std::endl;
        continue;
      }

      // Check for null states
      if (!getState(v))
      {
        BOLT_ERROR(indent, verbose_, "Null vertex found: " << v);
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
  visual_->viz(windowID)->state(getState(v), size, color, 0);

  // Show robot state
  // visual_->viz(windowID)->state(getState(v), tools::ROBOT, tools::DEFAULT, 0);
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
  visual_->viz(windowID)->edge(getState(v1), getState(v2), ompl::tools::MEDIUM, color);
}

void TaskGraph::debugState(const ompl::base::State *state)
{
  si_->printState(state, std::cout);
}

void TaskGraph::debugVertex(const TaskVertex v)
{
  debugState(getState(v));
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

void TaskGraph::printGraphStats()
{
  // Get the average vertex degree (number of connected edges)
  double averageDegree = (getNumEdges() * 2) / static_cast<double>(getNumVertices());

  // Check how many disjoint sets are in the task graph (should be none)
  std::size_t numSets = checkConnectedComponents();

  // Find min, max, and average edge length
  double totalEdgeLength = 0;
  double maxEdgeLength = -1 * std::numeric_limits<double>::infinity();
  double minEdgeLength = std::numeric_limits<double>::infinity();
  foreach (const TaskEdge e, boost::edges(g_))
  {
    const double length = edgeWeightProperty_[e];
    totalEdgeLength += length;
    if (maxEdgeLength < length)
      maxEdgeLength = length;
    if (minEdgeLength > length)
      minEdgeLength = length;
  }
  double averageEdgeLength = getNumEdges() ? totalEdgeLength / getNumEdges() : 0;

  std::size_t indent = 0;
  BOLT_DEBUG(indent, 1, "------------------------------------------------------");
  BOLT_DEBUG(indent, 1, "TaskGraph stats:");
  BOLT_DEBUG(indent, 1, "   Total vertices:         " << getNumVertices());
  BOLT_DEBUG(indent, 1, "   Total edges:            " << getNumEdges());
  BOLT_DEBUG(indent, 1, "   Average degree:         " << averageDegree);
  BOLT_DEBUG(indent, 1, "   Connected Components:   " << numSets);
  BOLT_DEBUG(indent, 1, "   Edge Lengths:           ");
  BOLT_DEBUG(indent, 1, "      Max:                 " << maxEdgeLength);
  BOLT_DEBUG(indent, 1, "      Min:                 " << minEdgeLength);
  BOLT_DEBUG(indent, 1, "      Average:             " << averageEdgeLength);
  BOLT_DEBUG(indent, 1, "------------------------------------------------------");
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

void otb::TaskAstarVisitor::discover_vertex(TaskVertex v, const TaskAdjList &) const
{
  // Statistics
  parent_->recordNodeOpened();

  if (parent_->visualizeAstar_)
    parent_->getVisual()->viz4()->state(parent_->getState(v), tools::SMALL, tools::GREEN, 1);
}

void otb::TaskAstarVisitor::examine_vertex(TaskVertex v, const TaskAdjList &) const
{
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

  if (v == goal_)
    throw FoundGoalException();
}
