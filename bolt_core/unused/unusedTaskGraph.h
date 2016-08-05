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

#ifndef OMPL_TOOLS_BOLT_PLANNING_GRAPH_
#define OMPL_TOOLS_BOLT_PLANNING_GRAPH_

// OMPL
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/tools/debug/Visualizer.h>
#include <ompl/tools/bolt/SparseGraph.h>
#include <ompl/tools/bolt/BoostGraphHeaders.h>
#include <ompl/tools/bolt/DenseCache.h>

// Boost
#include <boost/graph/astar_search.hpp>

namespace ompl
{
namespace tools
{
namespace bolt
{
/**
   @anchor TaskGraph
   @par Short description
   Database for storing and retrieving past plans
*/

/// @cond IGNORE
OMPL_CLASS_FORWARD(TaskGraph);
OMPL_CLASS_FORWARD(SparseGraph);
/// @endcond

typedef std::map<TaskVertex, std::vector<TaskVertex> > DisjointSetsParentKey;

/** \class ompl::tools::bolt::TaskGraphPtr
    \brief A boost shared pointer wrapper for ompl::tools::bolt::TaskGraph */

/** \brief Save and load entire paths from file */
class TaskGraph
{
  friend class BoltRetrieveRepair;
  friend class SparseGraph;
  friend class SparseStorage;
  friend class DenseCache;

public:
  // ////////////////////////////////////////////////////////////////////////////////////////
  // /**
  //  * Used to artifically supress edges during A* search.
  //  * \implements ReadablePropertyMapConcept
  //  */
  // class TaskEdgeWeightMap
  // {
  // private:
  //   const TaskAdjList& g_;  // Graph used
  //   const TaskVertexCollisionStateMap& collisionStates_;
  //   const double popularityBias_;
  //   const bool popularityBiasEnabled_;

  // public:
  //   /** Map key type. */
  //   typedef TaskVertex key_type;
  //   /** Map value type. */
  //   typedef double value_type;
  //   /** Map auxiliary value type. */
  //   typedef double& reference;
  //   /** Map type. */
  //   typedef boost::readable_property_map_tag category;

  //   /**
  //    * Construct map for certain constraints.
  //    * \param graph         Graph to use
  //    */
  //   TaskEdgeWeightMap(const TaskAdjList& graph, const TaskVertexCollisionStateMap& collisionStates,
  //                 const double& popularityBias, const bool popularityBiasEnabled);

  //   /**
  //    * Get the weight of an edge.
  //    * \param e the edge
  //    * \return infinity if \a e lies in a forbidden neighborhood; actual weight of \a e otherwise
  //    */
  //   double get(TaskEdge e) const;
  // };

  ////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Vertex visitor to check if A* search is finished.
   * \implements AStarVisitorConcept
   * See http://www.boost.org/doc/libs/1_58_0/libs/graph/doc/AStarVisitor.html
   */
  class CustomAstarVisitor : public boost::default_astar_visitor
  {
  private:
    TaskVertex goal_;  // Goal Vertex of the search
    TaskGraph* parent_;

  public:
    /**
     * Construct a visitor for a given search.
     * \param goal  goal vertex of the search
     */
    CustomAstarVisitor(TaskVertex goal, TaskGraph* parent);

    /**
     * \brief Invoked when a vertex is first discovered and is added to the OPEN list.
     * \param v current Vertex
     * \param g graph we are searching on
     */
    void discover_vertex(TaskVertex v, const TaskAdjList& g) const;

    /**
     * \brief Check if we have arrived at the goal.
     * This is invoked on a vertex as it is popped from the queue (i.e., it has the lowest
     * cost on the OPEN list). This happens immediately before examine_edge() is invoked on
     * each of the out-edges of vertex u.
     * \param v current vertex
     * \param g graph we are searching on
     * \throw foundGoalException if \a u is the goal
     */
    void examine_vertex(TaskVertex v, const TaskAdjList& g) const;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  // TaskGraph MEMBER FUNCTIONS
  ////////////////////////////////////////////////////////////////////////////////////////

  /** \brief Constructor needs the state space used for planning.
   *  \param space - state space
   */
  TaskGraph(base::SpaceInformationPtr si, VisualizerPtr visual);

  /** \brief Deconstructor */
  virtual ~TaskGraph(void);

  /** \brief Initialize database */
  bool setup();

  /** \brief Given two milestones from the same connected component, construct a path connecting them and set it as
   * the solution
   *  \param start
   *  \param goal
   *  \param vertexPath
   *  \return true if candidate solution found
   */
  bool astarSearch(const TaskVertex start, const TaskVertex goal, std::vector<TaskVertex>& vertexPath);

  /** \brief Print info to screen */
  void debugVertex(const ompl::base::PlannerDataVertex& vertex);
  void debugState(const ompl::base::State* state);

  /**
   * \brief Check if anything has been loaded into DB
   * \return true if has no nodes
   */
  bool isEmpty()
  {
    return !getNumVertices();
  }

  /** \brief Retrieve the computed roadmap. */
  const TaskAdjList& getRoadmap() const
  {
    return g_;
  }

  /** \brief Get the number of vertices in the sparse roadmap. */
  unsigned int getNumVertices() const
  {
    return boost::num_vertices(g_);
  }

  /** \brief Get the number of edges in the sparse roadmap. */
  unsigned int getNumEdges() const
  {
    return boost::num_edges(g_);
  }

  /** \brief Hook for adding vertices from SparseStorage */
  // void addVertexFromFile(SparseStorage::PlannerDataVertex *v);
  void addVertexFromFile(SparseStorage::BoltVertexData v);

  /** \brief Hook for adding edges from SparseStorage */
  void addEdgeFromFile(SparseStorage::BoltEdgeData e);

  /** \brief Mark the datastructure as needing to be saved to file */
  void setGraphUnsaved()
  {
    graphUnsaved_ = true;
  }

  /** \brief Free all the memory allocated by the database */
  void freeMemory();

  /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
  double distanceFunction(const TaskVertex a, const TaskVertex b) const;

  /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
  double distanceFunction2(const TaskVertex a, const TaskVertex b) const;

  /** \brief Helper for getting the task level value from a state */
  std::size_t getTaskLevel(const TaskVertex& v) const;
  std::size_t getTaskLevel(const base::State* state) const;

  /** \brief Check that the query vertex is initialized (used for internal nearest neighbor searches) */
  void initializeQueryState();

  /**
   * \brief Get the sparse graph to save to file
   * \param data - where to convert the data into
   */
  // virtual void getPlannerData(base::PlannerData& data) const;

  /** \brief Clear all past edge state information about in collision or not */
  void clearEdgeCollisionStates();

  /** \brief Visualize the stored database in an external program using callbacks */
  void displayDatabase();

  /** \brief Helper for creating/loading graph vertices */
  TaskVertex addVertex(base::State* state, const VertexType& type);

  /** \brief Helper for creating/loading graph edges */
  TaskEdge addEdge(const TaskVertex& v1, const TaskVertex& v2, const double weight,
                   const EdgeCollisionState collisionState = NOT_CHECKED);

  /** \brief Get whether to bias search using popularity of edges */
  bool getPopularityBiasEnabled()
  {
    return popularityBias_;
  }

  /** \brief Remove parts of graph that were intended to be temporary */
  void cleanupTemporaryVerticies();

  /**
   * \brief Get neighbors within radius
   * \param denseV - origin state to search from
   * \param graphNeighborhood - resulting nearby states
   * \param visibleNeighborhood - resulting nearby states that are visible
   * \param searchRadius - how far to search
   * \param countIndent - debugging tool
   */
  void findGraphNeighbors(base::State* state, std::vector<TaskVertex>& graphNeighborhood,
                          std::vector<TaskVertex>& visibleNeighborhood, double searchRadius, std::size_t threadID,
                          std::size_t coutIndent);

  void findGraphNeighbors(const TaskVertex& denseV, std::vector<TaskVertex>& graphNeighborhood,
                          std::vector<TaskVertex>& visibleNeighborhood, double searchRadius, std::size_t coutIndent);

  /** \brief Shortcut for visualizing an edge */
  void viz1Edge(TaskEdge& e);

  /** \brief Getter for using task planning flag */
  const bool& getUseTaskTask() const
  {
    return useTaskTask_;
  }

  /** \brief Setter for using task planning flag */
  void setUseTaskTask(const bool& useTaskTask)
  {
    useTaskTask_ = useTaskTask;
  }

  /** \brief Get class for managing various visualization features */
  VisualizerPtr getVisual()
  {
    return visual_;
  }

  /** \brief Get class that contains the sparse DB */
  SparseGraphPtr getSparseGraph()
  {
    return sparseGraph_;
  }

  /** \brief Helper for counting the number of disjoint sets in the sparse graph */
  std::size_t getDisjointSetsCount(bool verbose = false);

  std::size_t checkConnectedComponents();

  bool sameComponent(const TaskVertex& v1, const TaskVertex& v2);

  /** \brief Get all the different conencted components in the graph, and print to console or visualize */
  void getDisjointSets(DisjointSetsParentKey& disjointSets);
  void printDisjointSets(DisjointSetsParentKey& disjointSets);
  void visualizeDisjointSets(DisjointSetsParentKey& disjointSets);

  std::size_t getNumQueryVertices()
  {
    return queryVertices_.size();
  }

protected:
  /** \brief The created space information */
  base::SpaceInformationPtr si_;

  /** \brief Class for managing various visualization features */
  VisualizerPtr visual_;

  /** \brief Class to store lighter version of graph */
  SparseGraphPtr sparseGraph_;

  /** \brief Sampler user for generating valid samples in the state space */
  base::ValidStateSamplerPtr sampler_;  // TODO(davetcoleman): remove this unused sampler

  /** \brief Nearest neighbors data structure */
  std::shared_ptr<NearestNeighbors<TaskVertex> > nn_;

  /** \brief Connectivity graph */
  TaskAdjList g_;

  /** \brief Vertices for performing nearest neighbor queries on multiple threads */
  std::vector<TaskVertex> queryVertices_;

  /** \brief Access to the weights of each Edge */
  boost::property_map<TaskAdjList, boost::edge_weight_t>::type edgeWeightProperty_;

  /** \brief Access to the collision checking state of each Edge */
  TaskEdgeCollisionStateMap edgeCollisionStateProperty_;

  /** \brief Access to the internal base::state at each Vertex */
  boost::property_map<TaskAdjList, vertex_state_cache__t>::type stateCacheProperty_;

  /** \brief Access to the SPARS vertex type for the vertices */
  boost::property_map<TaskAdjList, vertex_type_t>::type typeProperty_;

  /** \brief Access to the representatives of the Dense vertices */
  boost::property_map<TaskAdjList, vertex_sparse_rep_t>::type representativesProperty_;

  /** \brief Data structure that maintains the connected components */
  boost::disjoint_sets<boost::property_map<TaskAdjList, boost::vertex_rank_t>::type,
                       boost::property_map<TaskAdjList, boost::vertex_predecessor_t>::type> disjointSets_;

  /** \brief Class for storing collision check data of edges */
  DenseCachePtr denseCache_;

  /** \brief Track where to load/save datastructures */
  std::string filePath_;

  /** \brief Track vertex for later removal if temporary */
  std::vector<TaskVertex> tempVerticies_;
  TaskVertex startConnectorVertex_;
  TaskVertex endConnectorVertex_;
  double distanceAcrossCartesian_ = 0.0;

  bool graphUnsaved_ = false;

public:
  /** \brief Are we task planning i.e. for hybrid cartesian paths? */
  bool useTaskTask_ = false;

  /** \brief Option to enable debugging output */
  bool verbose_ = false;
  bool snapPathVerbose_ = false;
  bool disjointVerbose_ = true;

  /** \brief Various options for visualizing the algorithmns performance */
  bool visualizeAstar_ = false;
  bool visualizeCartNeighbors_ = false;
  bool visualizeCartPath_ = false;
  bool visualizeSnapPath_ = false;
  double visualizeSnapPathSpeed_ = 0.001;
  bool visualizeAddSample_ = false;
  bool visualizeDatabaseVertices_ = true;
  bool visualizeDatabaseEdges_ = true;

  /** \brief Visualization speed of astar search, num of seconds to show each vertex */
  double visualizeAstarSpeed_ = 0.1;

  /** \brief Keep the average cost of the graph at this level */
  double desiredAverageCost_ = 90;

};  // end of class TaskGraph

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_BOLT_PLANNING_GRAPH_
