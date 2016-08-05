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
   Desc:   Basic graph components for Bolt
*/

#ifndef OMPL_TOOLS_BOLT_BOOST_GRAPH_HEADERS_
#define OMPL_TOOLS_BOLT_BOOST_GRAPH_HEADERS_

// OMPL
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/util/Hash.h>
#include <ompl/tools/bolt/InterfaceData.h>
#include <ompl/base/samplers/MinimumClearanceValidStateSampler.h>

// Boost
#include <boost/range/adaptor/map.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/pending/disjoint_sets.hpp>

// C++
#include <unordered_map>

namespace ompl
{
namespace tools
{
namespace bolt
{
typedef ompl::base::MinimumClearanceValidStateSamplerPtr ClearanceSamplerPtr;

// TODO(davetcoleman): maybe make all popularity use ints instead of doubles for memory efficiency?
static const double MAX_POPULARITY_WEIGHT = 100.0;  // 100 means the edge is very unpopular
// Everytime an edge is used, it is reduced by this amount (becomes more popular)
static const double POPULARITY_WEIGHT_REDUCTION = 5;

////////////////////////////////////////////////////////////////////////////////////////
// VERTEX PROPERTIES
////////////////////////////////////////////////////////////////////////////////////////

/** \brief Enumeration which specifies the reason a guard is added to the spanner. */
enum VertexType
{
  START,
  GOAL,
  COVERAGE,
  CONNECTIVITY,
  INTERFACE,
  QUALITY,
  CARTESIAN,
  DISCRETIZED
};
enum EdgeType
{
  eCONNECTIVITY,
  eINTERFACE,
  eQUALITY,
  eCARTESIAN,
  eDISCRETIZED
};

/** \brief The type used internally for representing vertex IDs */
typedef std::size_t VertexIndexType;

/** \brief Pair of vertices which support an interface. */
typedef std::pair<VertexIndexType, VertexIndexType> VertexPair;

/** \brief the hash which maps pairs of neighbor points to pairs of states */
typedef std::unordered_map<VertexPair, InterfaceData> InterfaceHash;

/** \brief Identification for states in the StateCache */
// typedef VertexIndexType StateID;

/** \brief Task level dimension data type */
typedef std::size_t VertexLevel;  // TODO(davetcoleman): rename to TaskLevel

/** \brief Boost vertex properties */
struct vertex_state_t
{
  typedef boost::vertex_property_tag kind;
};

struct vertex_level_t
{
  typedef boost::vertex_property_tag kind;
};

struct vertex_task_mirror_t
{
  typedef boost::vertex_property_tag kind;
};

struct vertex_interface_data_t
{
  typedef boost::vertex_property_tag kind;
};

struct vertex_sparse_rep_t
{
  typedef boost::vertex_property_tag kind;
};

struct vertex_type_t
{
  typedef boost::vertex_property_tag kind;
};

struct vertex_popularity_t
{
  typedef boost::vertex_property_tag kind;
};

////////////////////////////////////////////////////////////////////////////////////////
// Edge properties

struct edge_collision_state_t
{
  typedef boost::edge_property_tag kind;
};

struct edge_type_t
{
  typedef boost::edge_property_tag kind;
};

/** \brief Possible collision states of an edge */
enum EdgeCollisionState
{
  NOT_CHECKED,
  IN_COLLISION,
  FREE
};

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
// BOOST GRAPH - SPARSE
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////
/**
   \brief The underlying roadmap graph.

   \par Any BGL graph representation could be used here. Because we
   expect the roadmap to be sparse (m<n^2), an adjacency_list is more
   appropriate than an adjacency_matrix. Edges are undirected.

   *Properties of vertices*
   - vertex_state_t:
   - vertex_predecessor_t: Requred by incremental connected components algorithm (disjoint sets)
   - vertex_rank_t: Requred by incremental connected components algorithm (disjoint sets)
   - vertex_type_t: The type of guard this node is
   - vertex_list_t: non interface list property?

   Note: If boost::vecS is not used for vertex storage, then there must also
   be a boost:vertex_index_t property manually added.

   *Properties of edges*
   - edge_weight_t - cost/distance between two vertices
   - edge_collision_state_t - used for lazy collision checking, determines if an edge has been checked
   already for collision. 0 = not checked/unknown, 1 = in collision, 2 = free
*/

/** Wrapper for the vertex's multiple as its property. */
// clang-format off
typedef boost::property<vertex_state_t, base::State*, // State
        boost::property<boost::vertex_predecessor_t, VertexIndexType, // Disjoint Sets
        boost::property<boost::vertex_rank_t, VertexIndexType, // Disjoint Sets
        boost::property<vertex_type_t, VertexType, // Sparse Type
        boost::property<vertex_popularity_t, double, // Popularity
        boost::property<vertex_interface_data_t, InterfaceHash // Sparse meta data
        > > > > > > SparseVertexProperties;
// clang-format on

/** Wrapper for the double assigned to an edge as its weight property. */
// clang-format off
typedef boost::property<boost::edge_weight_t, double,
        boost::property<edge_type_t, EdgeType,
        boost::property<edge_collision_state_t, int> > > SparseEdgeProperties;
// clang-format on

/** The underlying boost graph type (undirected weighted-edge adjacency list with above properties). */
typedef boost::adjacency_list<boost::vecS,  // store in std::vector
                              boost::vecS,  // store in std::vector
                              boost::undirectedS, SparseVertexProperties, SparseEdgeProperties> SparseAdjList;

/** \brief Vertex in Graph */
typedef boost::graph_traits<SparseAdjList>::vertex_descriptor SparseVertex;

/** \brief Edge in Graph */
typedef boost::graph_traits<SparseAdjList>::edge_descriptor SparseEdge;

////////////////////////////////////////////////////////////////////////////////////////
// Typedefs for property maps

/** \brief Access map that stores the lazy collision checking status of each edge */
typedef boost::property_map<SparseAdjList, edge_collision_state_t>::type SparseEdgeCollisionStateMap;

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
// BOOST GRAPH - DENSE
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////
/**
   \brief The underlying roadmap graph.

   \par Any BGL graph representation could be used here. Because we
   expect the roadmap to be sparse (m<n^2), an adjacency_list is more
   appropriate than an adjacency_matrix. Edges are undirected.

   *Properties of vertices*
   - vertex_state_t: an ompl::base::State* is required for OMPL
   - vertex_predecessor_t: The incremental connected components algorithm requires it
   - vertex_rank_t: The incremental connected components algorithm requires it
   - vertex_type_t - Type of vertex/guard in SPARS

   Note: If boost::vecS is not used for vertex storage, then there must also
   be a boost:vertex_index_t property manually added.

   *Properties of edges*
   - edge_weight_t - cost/distance between two vertices
   - edge_collision_state_t - used for lazy collision checking, determines if an edge has been checked
   already for collision. 0 = not checked/unknown, 1 = in collision, 2 = free
*/

/** Wrapper for the vertex's multiple as its property. */
// clang-format off
typedef boost::property<vertex_state_t, base::State*, // State
        boost::property<boost::vertex_predecessor_t, VertexIndexType, // Disjoint Sets
        boost::property<boost::vertex_rank_t, VertexIndexType, // Disjoint Sets
        boost::property<vertex_type_t, VertexType, // Task Type TODO is this needed?
        boost::property<vertex_task_mirror_t, VertexIndexType // Link to corresponding free space TaskVertex, if one exists TODO is this needed?
        > > > > > TaskVertexProperties;
// clang-format on

/** Wrapper for the double assigned to an edge as its weight property. */
// clang-format off
typedef boost::property<boost::edge_weight_t, double,
        boost::property<edge_collision_state_t, int
        > > TaskEdgeProperties;
// clang-format on

/** The underlying boost graph type (undirected weighted-edge adjacency list with above properties). */
typedef boost::adjacency_list<boost::vecS,  // store in std::vector
                              boost::vecS,  // store in std::vector
                              boost::undirectedS, TaskVertexProperties, TaskEdgeProperties> TaskAdjList;

/** \brief Vertex in Graph */
typedef boost::graph_traits<TaskAdjList>::vertex_descriptor TaskVertex;

/** \brief Edge in Graph */
typedef boost::graph_traits<TaskAdjList>::edge_descriptor TaskEdge;

/** \brief Internal representation of a dense path */
// typedef std::deque<base::State*> DensePath;

////////////////////////////////////////////////////////////////////////////////////////
// Typedefs for property maps

/** \brief Access map that stores the lazy collision checking status of each edge */
typedef boost::property_map<TaskAdjList, edge_collision_state_t>::type TaskEdgeCollisionStateMap;

////////////////////////////////////////////////////////////////////////////////////////
/**
 * Used to artifically supress edges during A* search.
 * \implements ReadablePropertyMapConcept
 */
class TaskEdgeWeightMap
{
private:
  const TaskAdjList& g_;  // Graph used
  const TaskEdgeCollisionStateMap& collisionStates_;
  const double popularityBias_;
  const bool popularityBiasEnabled_;

public:
  /** Map key type. */
  typedef TaskEdge key_type;
  /** Map value type. */
  typedef double value_type;
  /** Map auxiliary value type. */
  typedef double& reference;
  /** Map type. */
  typedef boost::readable_property_map_tag category;

  /**
   * Construct map for certain constraints.
   * \param graph         Graph to use
   */
  TaskEdgeWeightMap(const TaskAdjList& graph, const TaskEdgeCollisionStateMap& collisionStates,
                    const double& popularityBias, const bool popularityBiasEnabled)
    : g_(graph)
    , collisionStates_(collisionStates)
    , popularityBias_(popularityBias)
    , popularityBiasEnabled_(popularityBiasEnabled)
  {
  }

  /**
   * Get the weight of an edge.
   * \param e the edge
   * \return infinity if \a e lies in a forbidden neighborhood; actual weight of \a e otherwise
   */
  double get(TaskEdge e) const
  {
    // Get the status of collision checking for this edge
    if (collisionStates_[e] == IN_COLLISION)
      return std::numeric_limits<double>::infinity();

    double weight;
    if (popularityBiasEnabled_)
    {
      // static const double popularityBias = 10;
      weight = boost::get(boost::edge_weight, g_, e) / MAX_POPULARITY_WEIGHT * popularityBias_;
      // std::cout << "getting popularity weight of edge " << e << " with value " << weight << std::endl;
    }
    else
    {
      weight = boost::get(boost::edge_weight, g_, e);
    }

    // Method 3 - less optimal but faster planning time
    // const double weighted_astar = 0.8;
    // const double weight = boost::get(boost::edge_weight, g_, e) * weighted_astar;

    // std::cout << "getting weight of edge " << e << " with value " << weight << std::endl;

    return weight;
  }
};

////////////////////////////////////////////////////////////////////////////////////////
/**
 * Used to artifically supress edges during A* search.
 * \implements ReadablePropertyMapConcept
 */
class SparseEdgeWeightMap
{
private:
  const SparseAdjList& g_;  // Graph used
  const SparseEdgeCollisionStateMap& collisionStates_;
  const double popularityBias_;
  const bool popularityBiasEnabled_;

public:
  /** Map key type. */
  typedef SparseEdge key_type;
  /** Map value type. */
  typedef double value_type;
  /** Map auxiliary value type. */
  typedef double& reference;
  /** Map type. */
  typedef boost::readable_property_map_tag category;

  /**
   * Construct map for certain constraints.
   * \param graph         Graph to use
   */
  SparseEdgeWeightMap(const SparseAdjList& graph, const SparseEdgeCollisionStateMap& collisionStates,
                      const double& popularityBias, const bool popularityBiasEnabled)
    : g_(graph)
    , collisionStates_(collisionStates)
    , popularityBias_(popularityBias)
    , popularityBiasEnabled_(popularityBiasEnabled)
  {
  }

  /**
   * Get the weight of an edge.
   * \param e the edge
   * \return infinity if \a e lies in a forbidden neighborhood; actual weight of \a e otherwise
   */
  double get(SparseEdge e) const
  {
    // Get the status of collision checking for this edge
    if (collisionStates_[e] == IN_COLLISION)
      return std::numeric_limits<double>::infinity();

    double weight;
    if (popularityBiasEnabled_)
    {
      // Maximum cost an edge can have based on popularity
      const double MAX_POPULARITY_WEIGHT = 100.0;

      // static const double popularityBias = 10;
      weight = boost::get(boost::edge_weight, g_, e) / MAX_POPULARITY_WEIGHT * popularityBias_;
      std::cout << "getting popularity weight of edge " << e << " with value " << weight << std::endl;
    }
    else
    {
      weight = boost::get(boost::edge_weight, g_, e);
    }

    // Method 3 - less optimal but faster planning time
    // const double weighted_astar = 0.8;
    // const double weight = boost::get(boost::edge_weight, g_, e) * weighted_astar;

    // std::cout << "getting weight of edge " << e << " with value " << weight << std::endl;

    return weight;
  }
};

////////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Sparse disjoint sets structure
 */
typedef boost::disjoint_sets<boost::property_map<SparseAdjList, boost::vertex_rank_t>::type,
                             boost::property_map<SparseAdjList, boost::vertex_predecessor_t>::type>
    SparseDisjointSetType;

// Ability to copy the disjoint sets data into a hashtable
typedef std::map<SparseVertex, std::vector<SparseVertex> > SparseDisjointSetsMap;

////////////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Task disjoint sets structure
 */
typedef boost::disjoint_sets<boost::property_map<TaskAdjList, boost::vertex_rank_t>::type,
                             boost::property_map<TaskAdjList, boost::vertex_predecessor_t>::type> TaskDisjointSetType;

// Ability to copy the disjoint sets data into a hashtable
typedef std::map<TaskVertex, std::vector<TaskVertex> > TaskDisjointSetsMap;

////////////////////////////////////////////////////////////////////////////////////////
/**
 * Thrown to stop the A* search when finished.
 */
class FoundGoalException
{
};

////////////////////////////////////////////////////////////////////////////////////////
// CANDIDATE STATE STRUCT
////////////////////////////////////////////////////////////////////////////////////////
struct CandidateData
{
  CandidateData(base::State* state) : state_(state)
  {
  }
  // Graph version number - allow to determine if candidate was expired by time candidate was generated
  std::size_t graphVersion_;

  // The sampled state to be added to the graph
  base::State* state_;

  // Nodes near our input state
  std::vector<SparseVertex> graphNeighborhood_;

  // Visible nodes near our input state
  std::vector<SparseVertex> visibleNeighborhood_;

  // The generated state
  SparseVertex newVertex_;
};

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_BOLT_BOOST_GRAPH_HEADERS_
