/*
 * _graph_detail.h

 */

#ifndef __GRAPH_DETAIL_H
#define __GRAPH_DETAIL_H

// OMPL
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>

// Boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/property_map/property_map.hpp>

////////////////////////////////////////////////////////////////////////////////
// Forward declare
class Graph;

////////////////////////////////////////////////////////////////////////////////
// Properties
namespace boost
{
/** Want to associate an object to each vertex. */
enum vertex_prop_t
{
  vertex_prop
};
}

////////////////////////////////////////////////////////////////////////////////
/**
 * Collection of attributes for a vertex.
 */
class VertexAttributes
{
private:
  ompl::base::State *state;  // vertex's location in C-space

public:
  /** Construct attributes without specifying state. */
  VertexAttributes();

  /**
   * Construct attributes mapping the vertex to a state.
   * @param state vertex's location in the space in which the graph is embedded
   */
  VertexAttributes(ompl::base::State *state);

  /**
   * Free memory used for this attribute.
   * @param si    SpaceInformationPtr the vertex's state belongs to
   * @note Call this for each vertex before destroying graph.
   * @warning Assumes the state has not been freed elsewhere.
   */
  void freeState(ompl::base::SpaceInformationPtr si);

  /**
   * Get the state this vertex is associated to.
   * @return our \a state
   */
  const ompl::base::State *getState() const;
};

/** Wrapper for the object assigned to a vertex as its property. */
typedef boost::property<boost::vertex_prop_t, VertexAttributes> VertexProperty;

/** Wrapper for the double assigned to an edge as its weight property. */
typedef boost::property<boost::edge_weight_t, double> EdgeProperty;

/** The underlying boost graph type (directed weighted-edge adjacency list with above properties). */
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperty, EdgeProperty> boost_graph;

/** Our graph's graph_traits. */
typedef boost::graph_traits<boost_graph> graph_traits;

/** Vertex type. */
typedef graph_traits::vertex_descriptor Vertex;

/** Edge type. */
typedef graph_traits::edge_descriptor Edge;

namespace boost
{
// Install the property
BOOST_INSTALL_PROPERTY(vertex, prop);
}

////////////////////////////////////////////////////////////////////////////////
// Placeholder
/*class Neighborhood
{
public:
  Neighborhood()
  {
  }

  bool shouldAvoid(Edge e) const
  {
    return false; // TODO
  }
};
*/

////////////////////////////////////////////////////////////////////////////////
/**
 * Heuristic for the A* search.
 * @implements AStarHeuristicConcept
 */
class heuristic
{
private:
  const Graph &g;     // Graph used
  const Vertex goal;  // Goal location

public:
  /**
   * Construct heuristic for a given search query.
   * @param graph Graph to use
   * @param goal  goal location
   */
  heuristic(const Graph &graph, Vertex goal);

  /**
   * Estimate the cost remaining to the goal.
   * @param u current location
   * @return estimated distance left
   * @attention You should change this heuristic if the graph distance between two vertices
   *  could be less than their C-space distance. (http://en.wikipedia.org/wiki/Admissible_heuristic)
   */
  double operator()(Vertex u) const;
};

////////////////////////////////////////////////////////////////////////////////
/**
 * Used to artifically supress edges during A* search.
 * @implements ReadablePropertyMapConcept
 */
class edgeWeightMap
{
private:
  const Graph &g;                  // Graph used
  const std::vector<Edge> &avoid;  // List of neighborhoods to stay out of

public:
  /** Map key type. */
  typedef Edge key_type;
  /** Map value type. */
  typedef double value_type;
  /** Map auxiliary value type. */
  typedef double &reference;
  /** Map type. */
  typedef boost::readable_property_map_tag category;

  /**
   * Construct map for certain constraints.
   * @param graph         Graph to use
   * @param avoidThese    list of neighborhoods to stay out of
   */
  edgeWeightMap(const Graph &graph, const std::vector<Edge> &avoidThese);

  /**
   * Get the weight of an edge.
   * @param e     the edge
   * @return infinity if \a e lies in a forbidden neighborhood; actual weight of \a e otherwise
   */
  double get(Edge e) const;

private:
  /**
   * Should this edge be avoided?
   * @param e edge to consider
   * @return true if \a e lies within a forbidden neighborhood; false otherwise
   */
  bool shouldAvoid(Edge e) const;
};

namespace boost
{
/**
 * Overload boost::get to access our map.
 * @param m     map to access
 * @param e     edge to query
 * @return weight of \a e
 */
double get(const edgeWeightMap &m, const Edge &e);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * Thrown to stop the A* search when finished.
 */
class foundGoalException
{
};

////////////////////////////////////////////////////////////////////////////////
/**
 * Vertex visitor to check if A* search is finished.
 * @implements AStarVisitorConcept
 */
class visitor : public boost::default_astar_visitor
{
private:
  Vertex goal;  // Goal Vertex of the search

public:
  /**
   * Construct a visitor for a given search.
   * @param goal  goal vertex of the search
   */
  visitor(Vertex goal);

  /**
   * Check if we have arrived at the goal.
   * @param u current vertex
   * @param g graph we are searching on
   * @throw foundGoalException if \a u is the goal
   */
  void examine_vertex(Vertex u, const Graph &g) const;
};

#endif
