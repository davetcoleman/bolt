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
   Desc:   Dynamic graph for storing copies of a sparse graph along with cartesian task dimensions
*/

// OMPL
#include <ompl/tools/bolt/TaskGraph.h>
#include <ompl/base/ScopedState.h>
#include <ompl/util/Time.h>
#include <ompl/util/Console.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>  // TODO: remove, this is not space agnostic

// Boost
#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>

// C++
#include <limits>
#include <queue>

#define foreach BOOST_FOREACH

namespace og = ompl::geometric;
namespace ot = ompl::tools;
namespace ob = ompl::base;
namespace otb = ompl::tools::bolt;

namespace boost
{
double get(const ompl::tools::bolt::TaskEdgeWeightMap &m, const ompl::tools::bolt::TaskEdge &e)
{
  return m.get(e);
}
}

BOOST_CONCEPT_ASSERT(
    (boost::ReadablePropertyMapConcept<ompl::tools::bolt::TaskEdgeWeightMap, ompl::tools::bolt::TaskEdge>));

// CustomAstarVisitor methods ////////////////////////////////////////////////////////////////////////////

BOOST_CONCEPT_ASSERT((boost::AStarVisitorConcept<otb::TaskGraph::CustomAstarVisitor, otb::TaskAdjList>));

otb::TaskGraph::CustomAstarVisitor::CustomAstarVisitor(TaskVertex goal, TaskGraph *parent)
  : goal_(goal), parent_(parent)
{
}

void otb::TaskGraph::CustomAstarVisitor::discover_vertex(TaskVertex v, const TaskAdjList &) const
{
  if (parent_->visualizeAstar_)
    parent_->getVisual()->viz4()->state(parent_->getVertexState(v), tools::SMALL, tools::GREEN, 1);
}

void otb::TaskGraph::CustomAstarVisitor::examine_vertex(TaskVertex v, const TaskAdjList &) const
{
  if (parent_->visualizeAstar_)
  {
    parent_->getVisual()->viz4()->state(parent_->getVertexState(v), tools::MEDIUM, tools::BLACK, 1);
    parent_->getVisual()->viz4()->trigger();
    usleep(parent_->visualizeAstarSpeed_ * 1000000);
  }

  if (v == goal_)
    throw FoundGoalException();
}

// Actual class ////////////////////////////////////////////////////////////////////////////

namespace ompl
{
namespace tools
{
namespace bolt
{
TaskGraph::TaskGraph(base::SpaceInformationPtr si, VisualizerPtr visual)
  : si_(si)
  , visual_(visual)
  // Property accessors of edges
  , edgeWeightProperty_(boost::get(boost::edge_weight, g_))
  , edgeCollisionStateProperty_(boost::get(edge_collision_state_t(), g_))
  // Property accessors of vertices
  , stateProperty_(boost::get(vertex_state_t(), g_))
  , typeProperty_(boost::get(vertex_type_t(), g_))
  , representativesProperty_(boost::get(vertex_sparse_rep_t(), g_))
  // Disjoint set accessors
  , disjointSets_(boost::get(boost::vertex_rank, g_), boost::get(boost::vertex_predecessor, g_))
{
  // Add search state
  initializeQueryState();

  // Initialize nearest neighbor datastructure
  nn_.reset(new NearestNeighborsGNAT<TaskVertex>());
  nn_->setDistanceFunction(boost::bind(&TaskGraph::distanceFunction, this, _1, _2));
}

TaskGraph::~TaskGraph(void)
{
  freeMemory();
}

void TaskGraph::freeMemory()
{
  foreach (TaskVertex v, boost::vertices(g_))
  {
    if (getVertexState(v) != nullptr)
      si_->freeState(getVertexState(v));
    getVertexState(v) = nullptr;  // TODO(davetcoleman): is this needed??
  }

  g_.clear();

  if (nn_)
    nn_->clear();

  sampler_.reset();
}

bool TaskGraph::setup()
{
  if (!sampler_)
    sampler_ = si_->allocValidStateSampler();

  return true;
}

bool TaskGraph::astarSearch(const TaskVertex start, const TaskVertex goal, std::vector<TaskVertex> &vertexPath)
{
  // Hold a list of the shortest path parent to each vertex
  TaskVertex *vertexPredecessors = new TaskVertex[getNumVertices()];
  // boost::vector_property_map<TaskVertex> vertexPredecessors(getNumVertices());

  bool foundGoal = false;
  double *vertexDistances = new double[getNumVertices()];

  // Error check
  if (useTaskTask_)
  {
    if (getTaskLevel(start) != 0)
    {
      OMPL_ERROR("astarSearch: start level is %u", getTaskLevel(start));
      exit(-1);
    }
    if (getTaskLevel(goal) != 2)
    {
      OMPL_ERROR("astarSearch: goal level is %u", getTaskLevel(goal));
      exit(-1);
    }
  }

  OMPL_INFORM("Beginning AStar Search");
  try
  {
    // Note: could not get astar_search to compile within BoltRetrieveRepair.cpp class because of namespacing issues
    boost::astar_search(
        g_,     // graph
        start,  // start state
                // boost::bind(&TaskGraph::distanceFunction2, this, _1, goal),  // the heuristic
        boost::bind(&TaskGraph::distanceFunction, this, _1, goal),  // the heuristic
        // boost::bind(&TaskGraph::distanceFunctionTasks, this, _1, goal),  // the heuristic
        // ability to disable edges (set cost to inifinity):
        boost::weight_map(TaskEdgeWeightMap(g_, edgeCollisionStateProperty_, popularityBias_, popularityBiasEnabled_))
            .predecessor_map(vertexPredecessors)
            .distance_map(&vertexDistances[0])
            .visitor(CustomAstarVisitor(goal, this)));
  }
  catch (FoundGoalException &)
  {
    // the custom exception from CustomAstarVisitor
    OMPL_INFORM("astarSearch: Astar found goal vertex. distance to goal: %f", vertexDistances[goal]);

    if (vertexDistances[goal] > 1.7e+308)  // TODO(davetcoleman): fix terrible hack for detecting infinity
                                           // double diff = d[goal] - std::numeric_limits<double>::infinity();
    // if ((diff < std::numeric_limits<double>::epsilon()) && (-diff < std::numeric_limits<double>::epsilon()))
    // check if the distance to goal is inifinity. if so, it is unreachable
    // if (d[goal] >= std::numeric_limits<double>::infinity())
    {
      if (verbose_)
        OMPL_INFORM("Distance to goal is infinity");
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
      if (v != goal)  // TODO explain this because i don't understand
      {
        vertexPath.push_back(v);
      }

      foundGoal = true;
    }
  }

  if (!foundGoal)
    OMPL_WARN("        Did not find goal");

  // Show all predecessors
  if (visualizeAstar_)
  {
    OMPL_INFORM("        Show all predecessors");
    for (std::size_t i = 1; i < getNumVertices(); ++i)  // skip vertex 0 b/c that is the search vertex
    {
      const TaskVertex v1 = i;
      const TaskVertex v2 = vertexPredecessors[v1];
      if (v1 != v2)
      {
        // std::cout << "Edge " << v1 << " to " << v2 << std::endl;
        visual_->viz4()->edge(getVertexState(v1), getVertexState(v2), 10);
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

void TaskGraph::debugVertex(const ompl::base::PlannerDataVertex &vertex)
{
  debugState(vertex.getState());
}

void TaskGraph::debugState(const ompl::base::State *state)
{
  si_->printState(state, std::cout);
}

double TaskGraph::distanceFunction(const TaskVertex a, const TaskVertex b) const
{
  // const double dist = si_->distance(stateProperty_[a], stateProperty_[b]);
  // std::cout << "getting distance from " << a << " to " << b << " of value " << dist << std::endl;
  // return dist;
  return si_->distance(stateProperty_[a], stateProperty_[b]);
}

double TaskGraph::distanceFunction2(const TaskVertex a, const TaskVertex b) const
{
  // const double dist = si_->getStateSpace()->distance2(stateProperty_[a], stateProperty_[b]);
  // std::cout << "getting distance from " << a << " to " << b << " of value " << dist << std::endl;
  // return dist;
  return si_->getStateSpace()->distance2(stateProperty_[a], stateProperty_[b]);
}

std::size_t TaskGraph::getTaskLevel(const TaskVertex &v) const
{
  return si_->getStateSpace()->getLevel(stateProperty_[v]);
}

std::size_t TaskGraph::getTaskLevel(const base::State *state) const
{
  return si_->getStateSpace()->getLevel(state);
}

void TaskGraph::initializeQueryState()
{
  std::size_t numThreads = boost::thread::hardware_concurrency();

  if (boost::num_vertices(g_) > 0)
  {
    assert(boost::num_vertices(g_) >= numThreads);
    return;  // assume its already been setup
  }

  // Create a query state for each possible thread
  queryVertices_.resize(numThreads);

  for (std::size_t threadID = 0; threadID < numThreads; ++threadID)
  {
    // Add a fake vertex to the graph
    queryVertices_[threadID] = boost::add_vertex(g_);

    // Set its state to nullptr
    stateProperty_[queryVertices_[threadID]] = nullptr;
  }
}

void TaskGraph::clearEdgeCollisionStates()
{
  foreach (const TaskEdge e, boost::edges(g_))
    edgeCollisionStateProperty_[e] = NOT_CHECKED;  // each edge has an unknown state
}

void TaskGraph::displayDatabase()
{
  OMPL_INFORM("Displaying database");

  // Clear old database
  visual_->viz1()->deleteAllMarkers();

  if (visualizeDatabaseVertices_)
  {
    // Error check
    if (getNumVertices() == 0)
    {
      OMPL_WARN("Unable to show complete database because no vertices available");
    }

    // Loop through each vertex
    std::size_t count = 0;
    std::size_t debugFrequency = std::min(10000, static_cast<int>(getNumVertices() / 10));
    std::cout << "Displaying vertices: " << std::flush;
    foreach (TaskVertex v, boost::vertices(g_))
    {
      // Check for null states
      if (stateProperty_[v])
      {
        visual_->viz1()->state(stateProperty_[v], tools::SMALL, tools::BLUE, 1);
      }

      // Prevent cache from getting too big
      if (count % debugFrequency == 0)
      {
        std::cout << std::fixed << std::setprecision(0) << (static_cast<double>(count + 1) / getNumVertices()) * 100.0
                  << "% " << std::flush;
        visual_->viz1()->trigger();
      }
      count++;
    }
    std::cout << std::endl;
  }

  if (visualizeDatabaseEdges_)
  {
    // Error check
    if (getNumEdges() == 0)
    {
      OMPL_WARN("Unable to show complete database because no edges available");
    }
    // Loop through each edge
    std::size_t count = 0;
    std::size_t debugFrequency = std::min(10000, static_cast<int>(getNumEdges() / 10));
    std::cout << "Displaying edges: " << std::flush;
    foreach (TaskEdge e, boost::edges(g_))
    {
      // Add edge
      const TaskVertex &v1 = boost::source(e, g_);
      const TaskVertex &v2 = boost::target(e, g_);

      // Visualize
      assert(edgeWeightProperty_[e] <= MAX_POPULARITY_WEIGHT);
      visual_->viz1()->edge(getVertexState(v1), getVertexState(v2), edgeWeightProperty_[e]);

      // Prevent cache from getting too big
      if (count % debugFrequency == 0)
      {
        std::cout << std::fixed << std::setprecision(0) << (static_cast<double>(count + 1) / getNumEdges()) * 100.0
                  << "% " << std::flush;
        visual_->viz1()->trigger();
      }

      count++;
    }
    std::cout << std::endl;
  }

  // Publish remaining markers
  visual_->viz1()->trigger();
}

otb::TaskVertex TaskGraph::addVertex(base::State *state, const VertexType &type)
{
  // Create vertex
  TaskVertex v = boost::add_vertex(g_);

  // Add properties
  // typeProperty_[v] = type;
  stateProperty_[v] = state;
  // representativesProperty_[v] = 0;  // which sparse vertex reps this dense vertex

  // Connected component tracking
  // disjointSets_.make_set(v);

  // Add vertex to nearest neighbor structure
  // nn_->add(v);

  // Track vertex for later removal if temporary
  // if (type == CARTESIAN)
  // {
  //   tempVerticies_.push_back(v);
  // }

  return v;
}

otb::TaskEdge TaskGraph::addEdge(const TaskVertex &v1, const TaskVertex &v2, const double weight,
                                 const EdgeCollisionState collisionState)
{
  // Error check
  BOOST_ASSERT_MSG(v1 <= getNumVertices(), "Vertex 1 out of range of possible verticies");
  BOOST_ASSERT_MSG(v2 <= getNumVertices(), "Vertex 2 out of range of possible verticies");

  // Create the new edge
  TaskEdge e = (boost::add_edge(v1, v2, g_)).first;

  // std::cout << "Adding cost: " << weight << std::endl;

  // Add associated properties to the edge
  edgeWeightProperty_[e] = weight;
  // edgeWeightProperty_[e] = distanceFunction2(v1, v2);
  // edgeWeightProperty_[e] = distanceFunction(v1, v2);
  edgeCollisionStateProperty_[e] = collisionState;

  // Add the edge to the incrementeal connected components datastructure
  disjointSets_.union_set(v1, v2);

  return e;
}

void TaskGraph::cleanupTemporaryVerticies()
{
  // const bool verbose = false;

  if (tempVerticies_.empty())
  {
    OMPL_INFORM("Skipping verticies cleanup - no middle cartesian layer verticies found");
    return;
  }

  OMPL_INFORM("Cleaning up temp verticies - vertex count: %u, edge count: %u", getNumVertices(), getNumEdges());
  // BOOST_REVERSE_FOREACH(TaskVertex v, tempVerticies_)
  // {
  //   OMPL_ERROR("todo removeVertex");
  //   removeVertex(v);

  //   if (verbose)
  //     OMPL_DEBUG("Removed, updated - vertex count: %u, edge count: %u", getNumVertices(), getNumEdges());
  // }
  tempVerticies_.clear();
  OMPL_INFORM("Finished cleaning up temp verticies");
}

void TaskGraph::findGraphNeighbors(base::State *state, std::vector<TaskVertex> &graphNeighborhood,
                                   std::vector<TaskVertex> &visibleNeighborhood, double searchRadius,
                                   std::size_t threadID, std::size_t coutIndent)
{
  // Set a queryVertex to give us a TaskVertex
  stateProperty_[queryVertices_[threadID]] = state;

  // Search
  findGraphNeighbors(queryVertices_[threadID], graphNeighborhood, visibleNeighborhood, searchRadius, coutIndent);

  // Reset a queryVertex
  stateProperty_[queryVertices_[threadID]] = nullptr;
}

void TaskGraph::findGraphNeighbors(const TaskVertex &denseV, std::vector<TaskVertex> &graphNeighborhood,
                                   std::vector<TaskVertex> &visibleNeighborhood, double searchRadius,
                                   std::size_t coutIndent)
{
  bool verbose = false;
  if (verbose)
    std::cout << std::string(coutIndent, ' ') << "findGraphNeighbors()" << std::endl;

  nn_->nearestR(denseV, searchRadius, graphNeighborhood);

  // Now that we got the neighbors from the NN, remove any we can't see
  for (TaskVertex &denseV : graphNeighborhood)
  // for (std::size_t i = 0; i < graphNeighborhood.size(); ++i)
  {
    if (si_->checkMotion(stateProperty_[denseV], stateProperty_[denseV]))
    {
      visibleNeighborhood.push_back(denseV);
    }
  }

  if (verbose)
    std::cout << std::string(coutIndent + 2, ' ') << "Graph neighborhood: " << graphNeighborhood.size()
              << " | Visible neighborhood: " << visibleNeighborhood.size() << std::endl;
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
      if (verbose)
        OMPL_INFORM("Disjoint set: %u", v);
      ++numSets;
    }
  }

  return numSets;
}

std::size_t TaskGraph::checkConnectedComponents()
{
  // Check how many disjoint sets are in the dense graph (should be none)
  std::size_t numSets = getDisjointSetsCount();
  if (numSets > 1)
  {
    OMPL_ERROR("More than 1 connected component is in the dense graph: %u", numSets);
  }

  return numSets;
}

bool TaskGraph::sameComponent(const TaskVertex &v1, const TaskVertex &v2)
{
  return boost::same_component(v1, v2, disjointSets_);
}

void TaskGraph::getDisjointSets(DisjointSetsParentKey &disjointSets)
{
  OMPL_INFORM("Get disjoint sets...");
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

void TaskGraph::printDisjointSets(DisjointSetsParentKey &disjointSets)
{
  for (DisjointSetsParentKey::const_iterator iterator = disjointSets.begin(); iterator != disjointSets.end();
       iterator++)
  {
    const TaskVertex v = iterator->first;
    const std::size_t freq = iterator->second.size();
    std::cout << "Parent: " << v << " frequency " << freq << std::endl;
  }
}

void TaskGraph::visualizeDisjointSets(DisjointSetsParentKey &disjointSets)
{
  OMPL_INFORM("Visualizing disjoint sets");

  // Find the disjoint set that is the 'main' large one
  std::size_t maxDisjointSetSize = 0;
  TaskVertex maxDisjointSetParent;
  for (DisjointSetsParentKey::const_iterator iterator = disjointSets.begin(); iterator != disjointSets.end();
       iterator++)
  {
    const TaskVertex v = iterator->first;
    const std::size_t freq = iterator->second.size();

    if (freq > maxDisjointSetSize)
    {
      maxDisjointSetSize = freq;
      maxDisjointSetParent = v;
    }
  }
  OMPL_INFORM("The largest disjoint set is of size %u with parent %u", maxDisjointSetSize, maxDisjointSetParent);

  // Display size of disjoint sets and visualize small ones
  for (DisjointSetsParentKey::const_iterator iterator = disjointSets.begin(); iterator != disjointSets.end();
       iterator++)
  {
    const TaskVertex v1 = iterator->first;
    const std::size_t freq = iterator->second.size();
    std::cout << v1 << ": frequency: " << freq << std::endl;

    BOOST_ASSERT_MSG(freq > 0, "Frequnecy must be at least 1");

    if (freq == maxDisjointSetSize)  // any subgraph that is smaller than the full graph
      continue;                      // the main disjoint set is not considered a disjoint set

    // Visualize sets of size one
    if (freq == 1 && true)
    {
      visual_->viz5()->state(getVertexState(v1), tools::ROBOT, tools::RED, 0);
      visual_->viz5()->trigger();
      usleep(1.0 * 1000000);
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
          visual_->viz4()->state(stateProperty_[*v2], tools::LARGE, tools::RED, 0);

          // Show state's edges
          foreach (TaskEdge edge, boost::out_edges(*v2, g_))
          {
            TaskVertex e_v1 = boost::source(edge, g_);
            TaskVertex e_v2 = boost::target(edge, g_);
            visual_->viz4()->edge(stateProperty_[e_v1], stateProperty_[e_v2], edgeWeightProperty_[edge]);
          }
          visual_->viz4()->trigger();

          // Show this robot state
          visual_->viz4()->state(stateProperty_[*v2], tools::ROBOT, tools::DEFAULT, 0);

          usleep(0.1 * 1000000);
        }
      }
      if (true)
      {
        usleep(2 * 1000000);
      }
    }
  }
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
