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
#include <ompl/util/Console.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>

// Bolt
#include <bolt_core/SparseGraph.h>
#include <bolt_core/SparseCriteria.h>

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
#include <valgrind/callgrind.h>

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
SparseGraph::SparseGraph(base::SpaceInformationPtr si, VisualizerPtr visual)
  : si_(si)
  , visual_(visual)
    // Property accessors of edges
    //, edgeWeightProperty_(boost::get(boost::edge_weight, g_))
    //, edgeCollisionStatePropertySparse_(boost::get(edge_collision_state_t(), g_))
    // Property accessors of vertices
    ///, state_(boost::get(vertex_state_t(), g_))
#ifdef ENABLE_QUALITY
    //, vertexInterfaceProperty_(boost::get(vertex_interface_data_t(), g_))
#endif
    //, vertexPopularity_(boost::get(vertex_popularity_t(), g_))
    // Disjoint set accessors
  , disjointSets_(boost::get(&SparseVertexStruct::vertex_rank_, g_),
                                   boost::get(&SparseVertexStruct::vertex_predecessor_, g_))
{
  // Save number of threads available
  numThreads_ = boost::thread::hardware_concurrency();

  // Add search state
  initializeQueryState();

  // Saving and loading from file
  sparseStorage_.reset(new SparseStorage(si_, this));

  // Smoothing paths in ideal way for SPARS criteria */
  sparseSmoother_.reset(new SparseSmoother(si_, visual_));

  // Initialize nearest neighbor datastructure
  // nn_.reset(new NearestNeighborsGNATNoThreadSafety<SparseVertex>());
  nn_.reset(new NearestNeighborsGNAT<SparseVertex>());
  nn_->setDistanceFunction(boost::bind(&otb::SparseGraph::distanceFunction, this, _1, _2));

  if (superDebug_)
  {
    BOLT_WARN(0, true, "--------------------------------------------");
    BOLT_WARN(0, true, "Superdebug mode is enabled - will run slower");
    BOLT_WARN(0, true, "--------------------------------------------");
  }

#ifdef ENABLE_QUALITY
  BOLT_INFO(0, true, "Using Quality Criteria datastructures");
#endif
}

SparseGraph::~SparseGraph()
{
  freeMemory();
}

void SparseGraph::clear()
{
  freeMemory();
  clearStatistics();
  resetDisjointSets();

  // Add search states back
  initializeQueryState();
}

void SparseGraph::freeMemory()
{
  foreach (SparseVertex v, boost::vertices(g_))
  {
#ifdef ENABLE_QUALITY
    // Clear interface data
    foreach (InterfaceData &iData, vertexInterfaceProperty_[v] | boost::adaptors::map_values)
    {
      iData.clear(si_);
    }
#endif

    // Free states memory
    if (g_[v].state_ != nullptr)
      si_->freeState(g_[v].state_);
  }

  // Clear vertices and edges
  g_.clear();

  // Clear nearest neighbor
  nn_->clear();

  hasUnsavedChanges_ = false;
}

bool SparseGraph::setup()
{
  sparseSmoother_->setup();

  base::DiscreteMotionValidator *dmv = dynamic_cast<base::DiscreteMotionValidator *>(si_->getMotionValidator().get());
  dmv->setRequiredStateClearance(0.0);

  return true;
}

void SparseGraph::clearStatistics()
{
  numSamplesAddedForCoverage_ = 0;
  numSamplesAddedForConnectivity_ = 0;
  numSamplesAddedForInterface_ = 0;
  numSamplesAddedForQuality_ = 0;
  numNodesOpened_ = 0;
  numNodesClosed_ = 0;
}

void SparseGraph::initializeQueryState()
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
    g_[queryVertices_[threadID]].state_ = NULL;  // set state to NULL
  }
}

bool SparseGraph::load(std::size_t indent)
{
  // Benchmark
  time::point start = time::now();

  if (!sparseStorage_->load(filePath_.c_str()))
    return false;

  // Benchmark
  double duration = time::seconds(time::now() - start);
  BOLT_INFO(indent, true, "Graph total loading time: " << duration);

  // Error check
  if (!getNumVertices() || !getNumEdges())
  {
    OMPL_ERROR("Corrupted sparse graph loaded");
    return false;
  }

  // Show more data
  printGraphStats();

  // Nothing to save because was just loaded from file
  hasUnsavedChanges_ = false;

  if (visualizeGraphAfterLoading_)
    displayDatabase(/*vertices*/ false);

  return true;
}

bool SparseGraph::saveIfChanged(std::size_t indent)
{
  BOLT_FUNC(indent, true, "saveIfChanged()");

  if (hasUnsavedChanges_)
  {
    return save(indent);
  }
  else
  {
    BOLT_DEBUG(indent, true, "Not saving because database has not changed. Time: " << time::as_string(time::now()));
  }

  return true;
}

bool SparseGraph::save(std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "save()");

  if (!hasUnsavedChanges_)
    OMPL_WARN("No need to save because hasUnsavedChanges_ is false, but saving anyway because requested");

  // Disabled
  if (!savingEnabled_)
  {
    OMPL_INFORM("Saving is disabled");
    return false;
  }

  // Error checking
  if (filePath_.empty())
  {
    OMPL_ERROR("Empty filename passed to save function");
    return false;
  }

  // Always must clear out deleted veritices from graph before saving otherwise NULL state will throw exception
  removeDeletedVertices(indent);

  // Benchmark
  time::point start = time::now();

  // Save
  {
    // std::lock_guard<std::mutex> guard(modifyGraphMutex_);
    sparseStorage_->save(filePath_.c_str());
    hasUnsavedChanges_ = false;
  }

  // Benchmark
  double loadTime = time::seconds(time::now() - start);
  BOLT_INFO(indent, true, "Saved database to file in " << loadTime
            << " seconds. Time: " << time::as_string(time::now()));
  return true;
}

bool SparseGraph::astarSearch(const SparseVertex start, const SparseVertex goal, std::vector<SparseVertex> &vertexPath,
                              double &distance, std::size_t indent)
{
  BOLT_FUNC(indent, vSearch_, "astarSearch()");

  // Check if start and goal are the same
  if (si_->getStateSpace()->equalStates(getState(start), getState(goal)))
  {
    BOLT_DEBUG(indent, vSearch_, "astarSearch: start and goal states are the same");

    // Just add one vertex - this is the whole path
    vertexPath.push_back(start);

    distance = 0;
    return true;
  }

  // Hold a list of the shortest path parent to each vertex
  SparseVertex *vertexPredecessors = new SparseVertex[getNumVertices()];
  // boost::vector_property_map<SparseVertex> vertexPredecessors(getNumVertices());

  bool foundGoal = false;
  double *vertexDistances = new double[getNumVertices()];

#ifndef NDEBUG
  // Reset statistics
  numNodesOpened_ = 0;
  numNodesClosed_ = 0;
#endif

  if (visualizeAstar_)
  {
    visual_->viz4()->deleteAllMarkers();
  }

  try
  {
    boost::astar_search(g_, start, // graph, start state
                        [this, goal](SparseVertex v)
                        {
                          return astarHeuristic(v, goal); // the heuristic
                        },
                        // ability to disable edges (set cost to inifinity):
                        // boost::weight_map(SparseEdgeWeightMap(g_, edgeCollisionStatePropertySparse_))
                        // popularityBias, popularityBiasEnabled))
                        boost::weight_map(boost::get(&SparseEdgeStruct::weight_, g_))
                        .predecessor_map(vertexPredecessors)
                        .distance_map(&vertexDistances[0])
                        .visitor(SparseAstarVisitor(goal, this)));
  }
  catch (FoundGoalException &)
  {
    foundGoal = true;
  }

  // Search failed
  if (!foundGoal)
  {
    BOLT_WARN(indent, vSearch_, "Did not find goal");

    // Unload
    delete[] vertexPredecessors;
    delete[] vertexDistances;

    // No solution found from start to goal
    return false;
  }

  distance = vertexDistances[goal];

#ifndef NDEBUG
  // the custom exception from SparseAstarVisitor
  BOLT_DEBUG(indent, vSearch_, "AStar found solution. Distance to goal: " << vertexDistances[goal]);

  BOLT_DEBUG(indent, vSearch_, "Number nodes opened: " << numNodesOpened_
             << ", Number nodes closed: " << numNodesClosed_);
#endif

  // Only clear the vertexPath after we know we have a new solution, otherwise it might have a good
  // previous one
  vertexPath.clear();  // remove any old solutions

  // Trace back the shortest path in reverse and only save the states
  SparseVertex v;
  for (v = goal; v != vertexPredecessors[v]; v = vertexPredecessors[v])
    vertexPath.push_back(v);

  // Add the start state to the path, unless this path is just one vertex long and the start==goal
  if (v != goal)
  {
    vertexPath.push_back(v);
  }

#ifndef NDEBUG
  BOLT_ASSERT(vertexPath.size(), "Vertex path is empty! " << vertexPath.size());
  // Ensure start and goal states are included in path
  BOLT_ASSERT(si_->getStateSpace()->equalStates(getState(vertexPath.back()), getState(start)), "Start states are "
              "not the same");
  BOLT_ASSERT(si_->getStateSpace()->equalStates(getState(vertexPath.front()), getState(goal)), "Goal states are "
              "not the same");
  BOLT_ASSERT(vertexPath.size() >= 2, "Vertex path size is too small");

  // Show all predecessors
  if (visualizeAstar_)
  {
    BOLT_DEBUG(indent, vSearch_, "Show all predecessors");
    for (std::size_t i = getNumQueryVertices(); i < getNumVertices(); ++i)  // skip query vertices
    {
      const SparseVertex v1 = i;
      const SparseVertex v2 = vertexPredecessors[v1];
      if (v1 != v2)
      {
        visual_->viz4()->edge(getState(v1), getState(v2), 10);
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

bool SparseGraph::astarSearchLength(SparseVertex start, SparseVertex goal, double &distance, std::size_t indent)
{
  BOLT_FUNC(indent, vSearch_, "astarSearchLength()");

  bool foundGoal = false;

  // Hold a list of the shortest path parent to each vertex
  // TODO: reuse this initialized memory!
  SparseVertex *vertexPredecessors = new SparseVertex[getNumVertices()];
  double *vertexDistances = new double[getNumVertices()];

  distance = std::numeric_limits<double>::infinity();

#ifndef NDEBUG
  // Reset statistics
  numNodesOpened_ = 0;
  numNodesClosed_ = 0;

  if (visualizeAstar_)
  {
    visual_->viz4()->deleteAllMarkers();
  }
#endif

  try
  {
    boost::astar_search(g_, start, // graph, start state
                        [this, goal](SparseVertex v)
                        {
                          return astarHeuristic(v, goal); // the heuristic
                        },
                        // ability to disable edges (set cost to inifinity):
                        // boost::weight_map(SparseEdgeWeightMap(g_, edgeCollisionStatePropertySparse_))
                        // popularityBias, popularityBiasEnabled))
                        boost::weight_map(boost::get(&SparseEdgeStruct::weight_, g_))
                        .predecessor_map(vertexPredecessors)
                        .distance_map(&vertexDistances[0])
                        .visitor(SparseAstarVisitor(goal, this)));
  }
  catch (FoundGoalException &)
  {
    distance = vertexDistances[goal];

    if (!std::isinf(vertexDistances[goal]))
    {
      foundGoal = true;
    }
  }

  // Unload
  delete[] vertexPredecessors;
  delete[] vertexDistances;

  // No solution found from start to goal
  return foundGoal;
}

bool SparseGraph::checkPathLength(SparseVertex v1, SparseVertex v2, std::size_t indent)
{
  return checkPathLength(v1, v2, si_->distance(getState(v1), getState(v2)), indent);
}

bool SparseGraph::checkPathLength(SparseVertex v1, SparseVertex v2, double distance, std::size_t indent)
{
  static const double SMALL_EPSILON = 0.0001;

  double pathLength;
  if (!astarSearchLength(v1, v2, pathLength, indent))
    return true;

  if (pathLength < distance + SMALL_EPSILON)
  {
    BOLT_ERROR(indent, false, "New interface edge does not help enough, edge length: " << distance
               << ", astar: " << pathLength << ", difference between distances: "
               << fabs(distance - pathLength));
    return false;
  }

  BOLT_WARN(indent, false, "Interface edge qualifies - diff: " << (distance + SMALL_EPSILON - pathLength));
  return true;
}

double SparseGraph::astarHeuristic(SparseVertex a, SparseVertex b) const
{
  // Assume vertex 'a' is the one we care about its populariy

  // Get the classic distance
  // double dist = si_->distance(getState(a), getState(b));

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

  return si_->distance(getState(a), getState(b));
}

double SparseGraph::distanceFunction(SparseVertex a, SparseVertex b) const
{
  // std::cout << "sg.distancefunction() " << a << ", " << b << std::endl;
  // Special case: query vertices store their states elsewhere
  if (a < numThreads_)
  {
    return si_->distance(queryStates_[a], getState(b));
  }
  if (b < numThreads_)
  {
    return si_->distance(getState(a), queryStates_[b]);
  }

  // Error check
  if (superDebug_)
  {
    assert(!stateDeleted(a));
    assert(!stateDeleted(b));
  }

  return si_->distance(getState(a), getState(b));
}

bool SparseGraph::isEmpty() const
{
  assert(!(getNumVertices() < getNumQueryVertices()));
  return (getNumVertices() == getNumQueryVertices() && getNumEdges() == 0);
}

// void SparseGraph::clearEdgeCollisionStates()
// {
//   foreach (const SparseEdge e, boost::edges(g_))
//     edgeCollisionStatePropertySparse_[e] = NOT_CHECKED;  // each edge has an unknown state
// }

void SparseGraph::errorCheckDuplicateStates(std::size_t indent)
{
  BOLT_ERROR(indent, true, "errorCheckDuplicateStates() - part of super debug - NOT IMPLEMENTED");

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

std::size_t SparseGraph::getDisjointSetsCount(bool verbose) const
{
  std::size_t numSets = 0;
  /*
  foreach (SparseVertex v, boost::vertices(g_))
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
*/
  return numSets;
}

void SparseGraph::getDisjointSets(SparseDisjointSetsMap &disjointSets)
{
  /*
  if (!sparseCriteria_->useConnectivityCriteria_)
  {
    BOLT_WARN(0, true, "Disjoint Sets disabled");
    return;
  }

  disjointSets.clear();

  // Flatten the parents tree so that the parent of every element is its representative.
  disjointSets_.compress_sets(boost::vertices(g_).first, boost::vertices(g_).second);

  // Count size of each disjoint set and group its containing vertices
  typedef boost::graph_traits<SparseAdjList>::vertex_iterator VertexIterator;
  for (VertexIterator v = boost::vertices(g_).first; v != boost::vertices(g_).second; ++v)
  {
    // Do not count the search vertex within the sets
    if (*v <= queryVertices_.back())
      continue;

    disjointSets[boost::get(boost::get(boost::vertex_predecessor, g_), *v)].push_back(*v);
  }
  */
}

void SparseGraph::printDisjointSets(SparseDisjointSetsMap &disjointSets)
{
  /*
  OMPL_INFORM("Print disjoint sets");

  if (!sparseCriteria_->useConnectivityCriteria_)
  {
    BOLT_WARN(0, true, "Disjoint Sets disabled");
    return;
  }

  for (SparseDisjointSetsMap::const_iterator iterator = disjointSets.begin(); iterator != disjointSets.end();
       iterator++)
  {
    const SparseVertex v = iterator->first;
    const std::size_t freq = iterator->second.size();
    std::cout << "  Parent: " << v << " frequency " << freq << std::endl;
  }
  */
}

void SparseGraph::visualizeDisjointSets(SparseDisjointSetsMap &disjointSets)
{
  /*
  OMPL_INFORM("Visualizing disjoint sets");

  if (!sparseCriteria_->useConnectivityCriteria_)
  {
    BOLT_WARN(0, true, "Disjoint Sets disabled");
    return;
  }

  // Find the disjoint set that is the 'main' large one
  std::size_t maxDisjointSetSize = 0;
  SparseVertex maxDisjointSetParent = 0;
  for (SparseDisjointSetsMap::const_iterator iterator = disjointSets.begin(); iterator != disjointSets.end();
       iterator++)
  {
    const SparseVertex v = iterator->first;
    const std::size_t freq = iterator->second.size();

    if (freq > maxDisjointSetSize)
    {
      maxDisjointSetSize = freq;
      maxDisjointSetParent = v;
    }
  }
  OMPL_INFORM("The largest disjoint set is of size %u with root vertex %u (not visualizing)", maxDisjointSetSize, maxDisjointSetParent);

  // Display size of disjoint sets and visualize small ones
  for (SparseDisjointSetsMap::const_iterator iterator = disjointSets.begin(); iterator != disjointSets.end();
       iterator++)
  {
    const SparseVertex v1 = iterator->first;
    const std::size_t freq = iterator->second.size();

    BOLT_ASSERT(freq > 0, "Frequency must be at least 1");

    if (freq == maxDisjointSetSize)  // any subgraph that is smaller than the full graph
      continue;                      // the main disjoint set is not considered a disjoint set

    BOLT_INFO(0, true, "Showing disjoint set of size " << freq);

    // Visualize sets of size one
    if (freq == 1)
    {
      visual_->viz4()->deleteAllMarkers();
      visual_->viz4()->state(getState(v1), tools::LARGE, tools::RED, 0);
      visual_->viz4()->state(getState(v1), tools::ROBOT, tools::DEFAULT, 0);
      visual_->viz4()->trigger();
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
      typedef boost::graph_traits<SparseAdjList>::vertex_iterator VertexIterator;
      for (VertexIterator v2 = boost::vertices(g_).first; v2 != boost::vertices(g_).second; ++v2)
      {
        if (boost::get(boost::get(boost::vertex_predecessor, g_), *v2) == v1)
        {
          visual_->viz4()->state(getState(*v2), tools::LARGE, tools::RED, 0);

          // Show state's edges
          foreach (SparseEdge edge, boost::out_edges(*v2, g_))
          {
            SparseVertex e_v1 = boost::source(edge, g_);
            SparseVertex e_v2 = boost::target(edge, g_);
            visual_->viz4()->edge(getState(e_v1), getState(e_v2), edgeWeightProperty_[edge]);
          }
          visual_->viz4()->trigger(/*every* /50);

          // Show this robot state
          visual_->viz4()->state(getState(*v2), tools::ROBOT, tools::DEFAULT, 0);
          visual_->viz4()->state(getState(*v2), tools::SMALL, tools::RED, 0);

          usleep(0.001*1000000);
        }  // if
      }    // for
      visual_->viz4()->trigger();
      visual_->waitForUserFeedback("showing large disjoint set");
    }  // if
  }

*/
}

std::size_t SparseGraph::checkConnectedComponents()
{
  // Check how many disjoint sets are in the sparse graph (should be none)
  std::size_t numSets = getDisjointSetsCount();
  if (numSets > 1)
  {
    std::size_t indent = 0;
    BOLT_WARN(indent, true, "More than 1 connected component is in the sparse graph: " << numSets);
  }

  return numSets;
}

bool SparseGraph::sameComponent(SparseVertex v1, SparseVertex v2)
{
  return boost::same_component(v1, v2, disjointSets_);
}

void SparseGraph::resetDisjointSets()
{
  BOLT_ERROR(0, true, "not implemented reset disjoint sets");
  //disjointSets_ = SparseDisjointSetType(boost::get(boost::vertex_rank, g_), boost::get(boost::vertex_predecessor, g_));
}

SparseVertex SparseGraph::addVertex(base::State *state, const VertexType &type, std::size_t indent)
{
  // Create vertex
  SparseVertex v = boost::add_vertex(g_);

  // Add properties
  g_[v].state_ = state;

  // Quit early if just mirroring graph
  if (fastMirrorMode_)
    return v;

  // Feedback
  BOLT_FUNC(indent, vAdd_, "addVertex(): new_vertex: " << v << ", type " << type);

  // Clear all nearby interface data whenever a new vertex is added
#ifdef ENABLE_QUALITY
  if (sparseCriteria_ && sparseCriteria_->getUseFourthCriteria())
    clearInterfaceData(state);
#endif

  if (sparseCriteria_ && sparseCriteria_->useConnectivityCriteria_)
    disjointSets_.make_set(v);

  // Add vertex to nearest neighbor structure
  {
    std::lock_guard<std::mutex> guard(nearestNeighborMutex_);
    nn_->add(v);
  }

  // Book keeping for what was added
  switch (type)
  {
    case COVERAGE:
      numSamplesAddedForCoverage_++;
      break;
    case CONNECTIVITY:
      numSamplesAddedForConnectivity_++;
      break;
    case INTERFACE:
      numSamplesAddedForInterface_++;
      break;
    case QUALITY:
      numSamplesAddedForQuality_++;
      break;
    case DISCRETIZED:
      break;
    default:
      OMPL_ERROR("Unknown VertexType type %u", type);
  }

  // Visualize
  if (visualizeSparseGraph_)
  {
    visualizeVertex(v, type);

    if (visualizeSparseGraphSpeed_ > std::numeric_limits<double>::epsilon())
    {
      visual_->viz1()->trigger(visualizeTriggerEvery_);

      if (visualizeProjection_)  // Hack: Project to 2D space
        visual_->viz7()->trigger(visualizeTriggerEvery_);

      // usleep(visualizeSparseGraphSpeed_ * 1000000);
    }
  }

  // Optional Voronoi Diagram
  if (visualizeVoronoiDiagramAnimated_ || (visualizeVoronoiDiagram_ && sparseCriteria_->getUseFourthCriteria()))
    visual_->vizVoronoiDiagram();

  // Enable saving
  hasUnsavedChanges_ = true;

  // Debugging
  // if (!sparseCriteria_->getDiscretizedSamplesInsertion())
  // throw Exception(name_, "Added vertex randomly");
  // visual_->waitForUserFeedback("Added vertex randomly");

  return v;
}

SparseVertex SparseGraph::addVertexFromFile(base::State *state, std::size_t indent)
{
  // Create vertex
  SparseVertex v = boost::add_vertex(g_);

  // Add properties
  g_[v].state_ = state;

  // Connected component tracking
  if (sparseCriteria_ && sparseCriteria_->useConnectivityCriteria_)
    disjointSets_.make_set(v);

  return v;
}

void SparseGraph::removeVertex(SparseVertex v, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "removeVertex = " << v);

  // Remove from nearest neighbor
  {
    std::lock_guard<std::mutex> guard(nearestNeighborMutex_);
    nn_->remove(v);
  }

  // Delete state
  si_->freeState(g_[v].state_);
  g_[v].state_ = nullptr;

#ifdef ENABLE_QUALITY
  // Clear interface data
  foreach (InterfaceData &iData, vertexInterfaceProperty_[v] | boost::adaptors::map_values)
    iData.clear(si_);
#endif

  // TODO: disjointSets is now inaccurate
  // Our checkAddConnectivity() criteria is broken
  // because we frequntly delete edges and nodes..
  // disjointSets_.remove_set(v);

  // Remove all edges to and from vertex
  boost::clear_vertex(v, g_);

  // We do not actually remove the vertex from the graph
  // because that would invalidate the nearest neighbor tree
}

void SparseGraph::removeDeletedVertices(std::size_t indent)
{
  bool verbose = true;
  BOLT_FUNC(indent, verbose, "removeDeletedVertices()");

  // Remove all vertices that are set to 0
  std::size_t numRemoved = 0;

  // Iterate manually through graph
  typedef boost::graph_traits<SparseAdjList>::vertex_iterator VertexIterator;
  for (VertexIterator v = boost::vertices(g_).first; v != boost::vertices(g_).second; /* manual */)
  {
    if (*v < numThreads_)  // Skip query vertices
    {
      v++;
      continue;
    }

    if (stateDeleted(*v))  // Found vertex to delete
    {
      BOLT_DEBUG(indent, verbose, "Removing SparseVertex " << *v << " state: " << getState(*v));

      boost::remove_vertex(*v, g_);
      numRemoved++;
    }
    else  // only proceed if no deletion happened
    {
      // BOLT_DEBUG(indent, verbose, "Checking SparseVertex " << *v << " state: " << getState(*v));
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
  std::lock_guard<std::mutex> guard(nearestNeighborMutex_);
  nn_->clear();

  // Reset disjoint sets
  if (sparseCriteria_ && sparseCriteria_->useConnectivityCriteria_)
    resetDisjointSets();

  // Reinsert vertices into nearest neighbor
  foreach (SparseVertex v, boost::vertices(g_))
  {
    if (v <= queryVertices_.back())  // Ignore query vertices
      continue;

    nn_->add(v);
    if (sparseCriteria_ && sparseCriteria_->useConnectivityCriteria_)
      disjointSets_.make_set(v);
  }

  // Reinsert edges into disjoint sets
  if (sparseCriteria_ && sparseCriteria_->useConnectivityCriteria_)
    foreach (SparseEdge e, boost::edges(g_))
    {
      SparseVertex v1 = boost::source(e, g_);
      SparseVertex v2 = boost::target(e, g_);
      disjointSets_.union_set(v1, v2);
    }

  BOLT_DEBUG(indent, verbose, "Finished removing deleted vertices");
}

SparseEdge SparseGraph::addEdge(SparseVertex v1, SparseVertex v2, EdgeType type, std::size_t indent)
{
  return addEdge(v1, v2, distanceFunction(v1, v2), type, indent);
}

SparseEdge SparseGraph::addEdge(SparseVertex v1, SparseVertex v2, double weight, EdgeType type, std::size_t indent)
{
  if (superDebug_)  // Extra checks
  {
    BOLT_ASSERT(v1 <= getNumVertices(), "Vertex1 is larger than max vertex id");
    BOLT_ASSERT(v2 <= getNumVertices(), "Vertex2 is larger than max vertex id");
    BOLT_ASSERT(v1 != v2, "Verticex IDs are the same");
    BOLT_ASSERT(!hasEdge(v1, v2), "There already exists an edge between two vertices requested");
    BOLT_ASSERT(hasEdge(v1, v2) == hasEdge(v2, v1), "There already exists an edge between two vertices requested, "
                "other direction");
    BOLT_ASSERT(getState(v1) != getState(v2), "States on both sides of an edge are the same");
    BOLT_ASSERT(!si_->getStateSpace()->equalStates(getState(v1), getState(v2)), "Vertex IDs are different but "
                "states are the equal");
    // BOLT_ASSERT(si_->checkMotion(state_[v1], state_[v2]), "Edge is in collision");
  }

  // Create the new edge
  SparseEdge e = (boost::add_edge(v1, v2, g_)).first;

  // Weight properties
  g_[e].weight_ = weight;

  // Quit early if just mirroring graph
  if (fastMirrorMode_)
    return e;

  BOLT_FUNC(indent, vAdd_, "addEdge(): from vertex " << v1 << " to " << v2 << " type " << type);

  // Add the edge to the incrementeal connected components datastructure
  if (sparseCriteria_ && sparseCriteria_->useConnectivityCriteria_)
    disjointSets_.union_set(v1, v2);

  // Visualize
  if (visualizeSparseGraph_)
  {
    visualizeEdge(e, type, /*windowID*/ 1);
    visualizeEdge(e, type, /*windowID*/ 7);  // projection to 2D space

    if (visualizeSparseGraphSpeed_ > std::numeric_limits<double>::epsilon())
    {
      visual_->viz1()->trigger(visualizeTriggerEvery_);

      if (visualizeProjection_)  // Hack: Project to 2D space
        visual_->viz7()->trigger(visualizeTriggerEvery_);

      // usleep(visualizeSparseGraphSpeed_ * 1000000);
    }

    // if (g_[e].weight_ <= sparseCriteria_->getDiscretization() * 2.1)
    // {} // for copy-paste ease
    // else
    // {
    //   std::cout << "g_[e].weight_: " << g_[e].weight_ << std::endl;
    //   visual_->waitForUserFeedback("add edge");
    // }
  }

  // Enable saving
  hasUnsavedChanges_ = true;

  return e;
}

SparseEdge SparseGraph::addEdgeDummy(std::size_t numVertices)
{
  SparseVertex dummy1 = 0;
  SparseVertex dummy2 = numVertices - 1;

  std::cout << "edge from  " << dummy1 << " to " << dummy2 << std::endl;
  return (boost::add_edge(dummy1, dummy2, g_)).first;
}

void SparseGraph::removeEdge(SparseEdge e, std::size_t indent)
{
  boost::remove_edge(e, g_);
}

VizColors SparseGraph::edgeTypeToColor(EdgeType edgeType)
{
  return tools::BLUE;  // match SPARS2

  switch (edgeType)
  {
    case eCONNECTIVITY:
      return ORANGE;
      break;
    case eINTERFACE:
      return GREEN;
      break;
    case eQUALITY:
      return RED;
      break;
    case eCARTESIAN:
      return YELLOW;
      break;
    case eDISCRETIZED:
      return BLUE;
      break;
    default:
      throw Exception(name_, "Unknown edge type");
  }
  return ORANGE;  // dummy return value
}

base::State*& SparseGraph::getQueryStateNonConst(std::size_t threadID)
{
  BOLT_ASSERT(threadID < queryVertices_.size(), "Attempted to request state of regular vertex using query "
              "function");
  return queryStates_[threadID];
}

SparseVertex SparseGraph::getQueryVertices(std::size_t threadID)
{
  BOLT_ASSERT(threadID < queryVertices_.size(), "Attempted to request vertex beyond threadID count");
  return queryVertices_[threadID];
}

/** \brief Shortcut function for getting the state of a vertex */
base::State*& SparseGraph::getStateNonConst(SparseVertex v)
{
  BOLT_ASSERT(v >= queryVertices_.size(), "Attempted to request state of query vertex using wrong function");
  return g_[v].state_;
}

const base::State* SparseGraph::getState(SparseVertex v) const
{
  BOLT_ASSERT(v >= queryVertices_.size(), "Attempted to request state of query vertex using wrong function");
  return g_[v].state_;
}

/** \brief Determine if a vertex has been deleted (but not fully removed yet) */
bool SparseGraph::stateDeleted(SparseVertex v) const
{
  return g_[v].state_ == nullptr;
}

SparseVertex SparseGraph::getSparseRepresentative(base::State *state)
{
  std::vector<SparseVertex> graphNeighbors;
  const std::size_t threadID = 0;
  const std::size_t numNeighbors = 1;

  // Search for nearest sparse vertex of the provided state - this vertex provides its coverage
  queryStates_[threadID] = state;
  nn_->nearestK(queryVertices_[threadID], numNeighbors, graphNeighbors);
  queryStates_[threadID] = nullptr;

  if (graphNeighbors.empty())
  {
    throw Exception(name_, "No neighbors found for sparse representative");
  }
  return graphNeighbors[0];
}

void SparseGraph::clearEdgesNearVertex(SparseVertex vertex, std::size_t indent)
{
  BOLT_FUNC(indent, false, "clearEdgesNearVertex()");

  // Optionally disable this feature
  if (sparseCriteria_ && !sparseCriteria_->useClearEdgesNearVertex_)
    return;

  // TODO(davetcoleman): combine this with clearInterfaceData and ensure that all interface data is equally cleared
  // but do not clear out nearby edges if a non-quality-path vertex is added
  std::vector<SparseVertex> graphNeighbors;

  // Search
  nn_->nearestR(vertex, sparseCriteria_->getSparseDelta(), graphNeighbors);

#ifndef NDEBUG
  std::size_t origNumEdges = getNumEdges();
#endif

  // For each of the vertices
  foreach (SparseVertex v, graphNeighbors)
  {
    // Remove all edges to and from vertex
    boost::clear_vertex(v, g_);
  }

#ifndef NDEBUG
  BOLT_DEBUG(indent, false, "clearEdgesNearVertex() removed " << origNumEdges - getNumEdges());

  // Only display database if enabled
  if (visualizeSparseGraph_ && visualizeSparseGraphSpeed_ > std::numeric_limits<double>::epsilon())
  {
    // visual_->waitForUserFeedback("before clear edge near vertex");
    displayDatabase(true, true, 1, indent);
    // visual_->waitForUserFeedback("after clear edge near vertex");
  }
#endif
}

void SparseGraph::displayDatabase(bool showVertices, bool showEdges, std::size_t windowID, std::size_t indent)
{
  BOLT_FUNC(indent, vVisualize_, "displayDatabase() - Display Sparse Database");

  // Error check
  if (getNumVertices() == 0 && getNumEdges() == 0)
  {
    OMPL_WARN("Unable to show database because no vertices and no edges available");
    return;
  }

  // Clear previous visualization
  visual_->viz(windowID)->deleteAllMarkers();

  // Edges
  if (visualizeDatabaseEdges_ && showEdges)
  {
    // Loop through each edge
    foreach (SparseEdge e, boost::edges(g_))
    {
      visualizeEdge(e, eUNKNOWN, windowID);
    }
  }

  // Vertices
  if (visualizeDatabaseVertices_ && showVertices)
  {
    std::vector<const ompl::base::State *> states;
    states.reserve(getNumVertices());
    std::vector<ot::VizColors> colors;
    colors.reserve(getNumVertices());

    // Loop through each vertex
    foreach (SparseVertex v, boost::vertices(g_))
    {
      // Skip query vertices
      if (v < queryVertices_.size())
        continue;

      // Skip deleted vertices
      if (g_[v].state_ == 0)
        continue;

      // Check for null states
      if (!getState(v))
      {
        BOLT_ERROR(indent, true, "Null vertex found: " << v);
        continue;
      }

      // If desired show coverage, but we can't add to the main states array
      // TODO: add secondary array for this size spheres?
      if (visualizeDatabaseCoverage_ && sparseCriteria_)
        visual_->viz(windowID)
          ->state(getState(v), tools::VARIABLE_SIZE, tools::TRANSLUCENT_LIGHT, sparseCriteria_->getSparseDelta());

      // Populate properties
      colors.push_back(tools::BLACK); //vertexTypeToColor(vertexTypeProperty_[v]));
      states.push_back(getState(v));
    }

    // Create marker and push to queue
    visual_->viz(windowID)->states(states, colors, vertexSize_);
  }

  // Publish remaining edges
  visual_->viz(windowID)->trigger();

  usleep(0.001 * 1000000);
}

void SparseGraph::visualizeVertex(SparseVertex v, const VertexType &type)
{
  const VizColors color = vertexTypeToColor(type);

  // Show visibility region around vertex
  if (visualizeDatabaseCoverage_ && sparseCriteria_)
    visual_->viz1()->state(getState(v), tools::VARIABLE_SIZE, tools::TRANSLUCENT_LIGHT,
                           sparseCriteria_->getSparseDelta());

  // Show vertex
  visual_->viz1()->state(getState(v), vertexSize_, std::move(color), 0);

  // Show robot arm
  visual_->viz1()->state(getState(v), tools::ROBOT, tools::DEFAULT, 0);

  if (visualizeProjection_)  // For joint-space robots: project to 2D space
  {
    // Show visibility region around vertex
    if (visualizeDatabaseCoverage_ && sparseCriteria_)
      visual_->viz7()->state(getState(v), tools::VARIABLE_SIZE, tools::TRANSLUCENT_LIGHT,
                             sparseCriteria_->getSparseDelta() * 2.0);

    // Show vertex
    visual_->viz7()->state(getState(v), vertexSize_, color, 0);
  }
}

tools::VizColors SparseGraph::vertexTypeToColor(VertexType type)
{
  return tools::BLACK;  // match SPARS2

  switch (type)
  {
    case COVERAGE:
      return tools::MAGENTA;
      break;
    case CONNECTIVITY:
      return tools::BROWN;
      break;
    case INTERFACE:
      return tools::WHITE;
      break;
    case QUALITY:
      return tools::PINK;
      break;
    case CARTESIAN:
      return tools::BROWN;
      break;
    case DISCRETIZED:
      return tools::CYAN;
      break;
    case START:
    case GOAL:
    default:
      throw Exception(name_, "Unknown type");
  }
  return tools::BLACK;  // dummy return value, not actually ysed
}

void SparseGraph::visualizeEdge(SparseEdge e, EdgeType type, std::size_t windowID)
{
  // Add edge
  SparseVertex v1 = boost::source(e, g_);
  SparseVertex v2 = boost::target(e, g_);

  visualizeEdge(v1, v2, type, windowID);
}

void SparseGraph::visualizeEdge(SparseVertex v1, SparseVertex v2, EdgeType type, std::size_t windowID)
{
  // Visualize
  visual_->viz(windowID)->edge(getState(v1), getState(v2), edgeSize_, edgeTypeToColor(type));
}

#ifdef ENABLE_QUALITY
void SparseGraph::clearInterfaceData(base::State *state)
{
  std::vector<SparseVertex> graphNeighbors;
  const std::size_t threadID = 0;

  // Search
  queryStates_[threadID] = state;
  nn_->nearestR(queryVertices_[threadID], 2.0 * sparseCriteria_->getSparseDelta(), graphNeighbors);
  queryStates_[threadID] = nullptr;

  // For each of the vertices
  foreach (SparseVertex v, graphNeighbors)
  {
    foreach (VertexPair r, vertexInterfaceProperty_[v] | boost::adaptors::map_keys)
    {
      vertexInterfaceProperty_[v][r].clear(si_);
    }
  }
}

VertexPair SparseGraph::interfaceDataIndex(SparseVertex vp, SparseVertex vpp)
{
  if (vp < vpp)
    return VertexPair(vp, vpp);
  else if (vpp < vp)
    return VertexPair(vpp, vp);

  throw Exception(name_, "Trying to get an index where the pairs are the same point!");
  return VertexPair(0, 0);  // prevent compiler warnings
}

InterfaceData &SparseGraph::getInterfaceData(SparseVertex v, SparseVertex vp, SparseVertex vpp, std::size_t indent)
{
  // BOLT_FUNC(indent, sparseCriteria_->vQuality_, "getInterfaceData() " << v << ", " << vp << ", " << vpp);
  return vertexInterfaceProperty_[v][interfaceDataIndex(vp, vpp)];
}

InterfaceHash &SparseGraph::getVertexInterfaceProperty(SparseVertex v)
{
  return vertexInterfaceProperty_[v];
}
#endif

void SparseGraph::debugState(const ompl::base::State *state)
{
  si_->printState(state, std::cout);
}

void SparseGraph::debugVertex(const SparseVertex v)
{
  debugState(getState(v));
}

void SparseGraph::debugNN()
{
  // Show contents of GNAT
  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  NearestNeighborsGNAT<SparseVertex> *gnat = dynamic_cast<NearestNeighborsGNAT<SparseVertex> *>(nn_.get());
  std::cout << "GNAT: " << *gnat << std::endl;
  std::cout << std::endl;
}

void SparseGraph::printGraphStats()
{
  // Get the average vertex degree (number of connected edges)
  double averageDegree = (getNumEdges() * 2) / static_cast<double>(getNumRealVertices());

  // Check how many disjoint sets are in the sparse graph (should be none)
  std::size_t numSets = checkConnectedComponents();

  // Find min, max, and average edge length
  double totalEdgeLength = 0;
  double maxEdgeLength = -1 * std::numeric_limits<double>::infinity();
  double minEdgeLength = std::numeric_limits<double>::infinity();
  foreach (const SparseEdge e, boost::edges(g_))
  {
    const double length = g_[e].weight_;
    totalEdgeLength += length;
    if (maxEdgeLength < length)
      maxEdgeLength = length;
    if (minEdgeLength > length)
      minEdgeLength = length;
  }
  double averageEdgeLength = getNumEdges() ? totalEdgeLength / getNumEdges() : 0;

  std::size_t indent = 0;
  BOLT_DEBUG(indent, 1, "------------------------------------------------------");
  BOLT_DEBUG(indent, 1, "SparseGraph stats:");
  BOLT_DEBUG(indent, 1, "   Total vertices:         " << getNumRealVertices());
  BOLT_DEBUG(indent, 1, "   Total edges:            " << getNumEdges());
  BOLT_DEBUG(indent, 1, "   Average degree:         " << averageDegree);
  BOLT_DEBUG(indent, 1, "   Connected Components:   " << numSets);
  BOLT_DEBUG(indent, 1, "   Edge Lengths:           ");
  BOLT_DEBUG(indent, 1, "      Max:                 " << maxEdgeLength);
  BOLT_DEBUG(indent, 1, "      Min:                 " << minEdgeLength);
  BOLT_DEBUG(indent, 1, "      Average:             " << averageEdgeLength);
  if (sparseCriteria_)
  {
    BOLT_DEBUG(indent, 1, "      SparseDelta:         " << sparseCriteria_->getSparseDelta());
    BOLT_DEBUG(indent, 1, "      Difference:          " << averageEdgeLength - sparseCriteria_->getSparseDelta());
    BOLT_DEBUG(indent, 1, "      Penetration:         " << sparseCriteria_->getDiscretizePenetrationDist());
  }
  BOLT_DEBUG(indent, 1, "------------------------------------------------------");
}

bool SparseGraph::verifyGraph(std::size_t indent)
{
  BOLT_FUNC(indent, true, "verifyGraph()");

  foreach (const SparseVertex v, boost::vertices(g_))
  {
    if (v <= queryVertices_.back())  // Ignore query vertices
      continue;

    // Skip deleted vertices
    if (g_[v].state_ == 0)
      continue;

    // Check for null states
    if (!getState(v))
    {
      BOLT_ERROR(indent, true, "Null vertex found: " << v);
      return false;
    }

    // Collision check
    if (!si_->isValid(g_[v].state_))
    {
      BOLT_ERROR(indent, true, "Found invalid vertex " << v);
      return false;
    }
  }

  foreach (const SparseEdge e, boost::edges(g_))
  {
    SparseVertex v1 = boost::source(e, g_);
    SparseVertex v2 = boost::target(e, g_);

    // Collision check
    if (!si_->checkMotion(g_[v1].state_, g_[v2].state_))
    {
      BOLT_ERROR(indent, true, "Invalid edge found: " << v1 << " to " << v2);
      return false;
    }
  }

  return true;
}

base::ValidStateSamplerPtr SparseGraph::getSampler(base::SpaceInformationPtr si, double clearance, std::size_t indent)
{
  base::ValidStateSamplerPtr sampler;
  if (clearance > std::numeric_limits<double>::epsilon())
  {
    // BOLT_INFO(indent, true, "Sampling with clearance " << clearance);
    // Load minimum clearance state sampler
    sampler.reset(new base::MinimumClearanceValidStateSampler(si.get()));
    // Set the clearance
    base::MinimumClearanceValidStateSampler *mcvss =
      dynamic_cast<base::MinimumClearanceValidStateSampler *>(sampler.get());
    mcvss->setMinimumObstacleClearance(clearance);
    si->getStateValidityChecker()->setClearanceSearchDistance(clearance);
  }
  else  // regular sampler
  {
    // BOLT_INFO(indent, true, "Sampling without clearance");
    sampler.reset(new base::UniformValidStateSampler(si.get()));
  }
  return sampler;
}

double SparseGraph::getSparseDelta()
{
  return sparseCriteria_->getSparseDelta();
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

// SparseEdgeWeightMap methods ////////////////////////////////////////////////////////////////////////////

// namespace boost
// {
// double get(const ompl::tools::bolt::SparseEdgeWeightMap &m, const ompl::tools::bolt::SparseEdge &e)
// {
//   return m.get(e);
// }
// }

// BOOST_CONCEPT_ASSERT(
//                      (boost::ReadablePropertyMapConcept<ompl::tools::bolt::SparseEdgeWeightMap, ompl::tools::bolt::SparseEdge>));

// SparseAstarVisitor methods ////////////////////////////////////////////////////////////////////////////

BOOST_CONCEPT_ASSERT((boost::AStarVisitorConcept<otb::SparseAstarVisitor, otb::SparseAdjList>));

otb::SparseAstarVisitor::SparseAstarVisitor(SparseVertex goal, SparseGraph *parent) : goal_(goal), parent_(parent)
{
}

#ifndef NDEBUG
void otb::SparseAstarVisitor::discover_vertex(SparseVertex v, const SparseAdjList &) const
{
  // Statistics
  parent_->recordNodeOpened();

  if (parent_->visualizeAstar_)
    parent_->getVisual()->viz4()->state(parent_->getState(v), tools::SMALL, tools::GREEN, 1);
}
#endif

void otb::SparseAstarVisitor::examine_vertex(SparseVertex v, const SparseAdjList &) const
{
#ifndef NDEBUG
  // Statistics
  parent_->recordNodeClosed();
  if (parent_->visualizeAstar_)
  {
    parent_->getVisual()->viz4()->state(parent_->getState(v), tools::LARGE, tools::BLACK, 1);
    parent_->getVisual()->viz4()->trigger();
    usleep(parent_->visualizeAstarSpeed_ * 1000000);
  }
#endif

  if (v == goal_)
    throw FoundGoalException();
}
