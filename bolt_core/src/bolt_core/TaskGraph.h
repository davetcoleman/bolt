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

#ifndef OMPL_TOOLS_BOLT_TASK_GRAPH_
#define OMPL_TOOLS_BOLT_TASK_GRAPH_

#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>

// Bolt
#include <bolt_core/SparseGraph.h>
#include <bolt_core/BoostGraphHeaders.h>
#include <bolt_core/Debug.h>
#include <bolt_core/VertexDiscretizer.h>
#include <bolt_core/SparseStorage.h>
#include <ompl/tools/debug/Visualizer.h>

// Boost
#include <boost/function.hpp>
#include <boost/graph/astar_search.hpp>

// C++
#include <list>
#include <random>

namespace ompl
{
namespace tools
{
namespace bolt
{
/**
   @anchor TaskGraph
   @par Near-asypmotically optimal roadmap datastructure
*/

/// @cond IGNORE
OMPL_CLASS_FORWARD(TaskGraph);
/// @endcond

enum CompoundTypes
{
  MODEL_BASED = 0,
  DISCRETE = 1
};

/** \class ompl::tools::bolt::::TaskGraphPtr
    \brief A boost shared pointer wrapper for ompl::tools::TaskGraph */

/** \brief Near-asypmotically optimal roadmap datastructure */
class TaskGraph
{
public:
  /** \brief Constructor needs the state space used for planning.
   */
  TaskGraph(const base::SpaceInformationPtr &modelSI, const base::SpaceInformationPtr& compoundSI, SparseGraphPtr sg);

  /** \brief Deconstructor */
  virtual ~TaskGraph(void);

  /* ---------------------------------------------------------------------------------
   * Setup and cleanup
   * --------------------------------------------------------------------------------- */

  /** \brief Retrieve the computed roadmap. */
  const TaskAdjList& getGraph() const
  {
    return g_;
  }

  TaskAdjList getGraphNonConst()
  {
    return g_;
  }

  base::SpaceInformationPtr getCompoundSpaceInformation()
  {
    return compoundSI_;
  }

  SparseGraphPtr getSparseGraph()
  {
    return sg_;
  }

  /** \brief Get class for managing various visualization features */
  VisualizerPtr getVisual()
  {
    return visual_;
  }

  std::shared_ptr<NearestNeighbors<TaskVertex> > getNN()
  {
    return nn_;
  }

  /** \brief Initialize database */
  bool setup();

  /** \brief Reset the graph to be planned on again */
  void clear();

  /** \brief Free all the memory allocated by the database */
  void freeMemory();

  /* ---------------------------------------------------------------------------------
   * Astar search
   * --------------------------------------------------------------------------------- */

  /** \brief Check that the query vertex is initialized (used for internal nearest neighbor searches) */
  void initializeQueryState();

  /** \brief Given two milestones from the same connected component, construct a path connecting them and set it as
   * the solution
   *  \param start
   *  \param goal
   *  \param vertexPath
   *  \return true if candidate solution found
   */
  bool astarSearch(const TaskVertex start, const TaskVertex goal, std::vector<TaskVertex>& vertexPath, double& distance,
                   std::size_t indent);

  /** \brief Compute distance between two states ignoreing task level */
  double distanceVertex(const TaskVertex a, const TaskVertex b) const;
  double distanceState(const base::State* a, const base::State* b) const;

  /** \brief Distance between two vertices in a task space */
  double astarTaskHeuristic(const TaskVertex a, const TaskVertex b) const;

  /** \brief Custom A* visitor statistics */
  void recordNodeOpened()  // discovered
  {
    numNodesOpened_++;
  }

  void recordNodeClosed()  // examined
  {
    numNodesClosed_++;
  }

  /* ---------------------------------------------------------------------------------
   * Get graph properties
   * --------------------------------------------------------------------------------- */

  std::vector<TaskVertex> getQueryVertices()
  {
    return queryVertices_;
  }

  std::size_t getNumQueryVertices() const
  {
    return queryVertices_.size();
  }

  /** \brief Get the number of vertices in the task roadmap. */
  unsigned int getNumVertices() const
  {
    return boost::num_vertices(g_);
  }

  /** \brief Get the number of vertices in the task roadmap not including query vertices. */
  unsigned int getNumRealVertices() const
  {
    return boost::num_vertices(g_) - queryVertices_.size();
  }

  /** \brief Get the number of edges in the task roadmap. */
  unsigned int getNumEdges() const
  {
    return boost::num_edges(g_);
  }

  double getEdgeWeightProperty(TaskEdge e) const
  {
    return g_[e].weight_;
  }

  /** \brief Determine if no nodes or edges have been added to the graph except query vertices */
  bool isEmpty() const;

  /* ---------------------------------------------------------------------------------
   * Get/Set Task Level
   * --------------------------------------------------------------------------------- */
  inline VertexLevel getTaskLevel(const TaskVertex v) const
  {
    return g_[v].state_->as<base::CompoundState>()->as<base::DiscreteStateSpace::StateType>(DISCRETE)->value;
  }

  inline VertexLevel getTaskLevel(const base::State* state) const
  {
    // return state->as<base::DiscreteStateSpace::StateType>(DISCRETE)->value;
    return state->as<base::CompoundState>()->as<base::DiscreteStateSpace::StateType>(DISCRETE)->value;
  }

  inline void setVertexTaskLevel(TaskVertex v, VertexLevel level)
  {
    g_[v].state_->as<base::CompoundState>()->as<base::DiscreteStateSpace::StateType>(DISCRETE)->value = level;
  }

  inline void setStateTaskLevel(base::State* state, VertexLevel level)
  {
    state->as<base::CompoundState>()->as<base::DiscreteStateSpace::StateType>(DISCRETE)->value = level;
  }

  bool taskPlanningEnabled() const
  {
    return taskPlanningEnabled_;
  }

  void setTaskPlanningEnabled(bool enable = true)
  {
    taskPlanningEnabled_ = enable;
  }

  /* ---------------------------------------------------------------------------------
   * Task Planning
   * --------------------------------------------------------------------------------- */

  /** \brief Only create one level, basically provides non-task based planning but copies the graph */
  void generateMonoLevelTaskSpace(std::size_t indent);

  /** \brief Copy the sparse graph into a new task graph, and mirror it into two layers */
  void generateTaskSpace(std::size_t indent);

  /** \brief Add a cartesian path into the middle layer of the task dimension
   *         This is only used for the 2D planning case (toy problem)
   */
  bool addCartPath(std::vector<base::State*> path, std::size_t indent);

  /**
   * \brief Helper for connecting both sides of a cartesian path into a dual level graph
   * \param fromVertex - the endpoint (start or goal) we are connecting from the cartesian path to the graph
   * \param level - what task level we are connecting to - either 0 or 2 (bottom layer or top layer)
   * \param isStart - is this a start or goal vertex we are connecting?
   * \return true on success
   */
  bool connectVertexToNeighborsAtLevel(const TaskVertex fromVertex, const VertexLevel level, bool isStart,
                                       std::size_t indent);

  /** \brief Get k number of neighbors near a state at a certain level that have valid motions */
  void getNeighborsAtLevel(const TaskVertex nearVertex, const VertexLevel level, const std::size_t kNeighbors,
                           std::vector<TaskVertex>& neighbors, std::size_t indent);

  /** \brief Error checking function to ensure solution has correct task path/level changes */
  bool checkTaskPathSolution(geometric::PathGeometric& path, base::State* start, base::State* goal);

  /** \brief Getter for ShortestDistAcrossCart */
  const double& getShortestDistAcrossCart() const
  {
    OMPL_WARN("shortest_path_across_cart: %f", shortestDistAcrossCartGraph_);
    return shortestDistAcrossCartGraph_;
  }

  /** \brief Setter for ShortestDistAcrossCart */
  void setShortestDistAcrossCart(const double& shortestDistAcrossCartGraph)
  {
    shortestDistAcrossCartGraph_ = shortestDistAcrossCartGraph;
  }

  /* ---------------------------------------------------------------------------------
   * Error checking
   * --------------------------------------------------------------------------------- */

  /** \brief Clear all past edge state information about in collision or not */
  void clearEdgeCollisionStates();

  /** \brief Part of super debugging */
  void errorCheckDuplicateStates(std::size_t indent);

  /* ---------------------------------------------------------------------------------
   * Smoothing
   * --------------------------------------------------------------------------------- */

  /** \brief Path smoothing helpers */
  bool smoothQualityPath(geometric::PathGeometric* path, double clearance, std::size_t indent);

  /* ---------------------------------------------------------------------------------
   * Add/remove vertices, edges, states
   * --------------------------------------------------------------------------------- */

  /** \brief Create a compound state that includes a discrete step
   *  \param state - the base state of joint values
   *  \param level - the discrete step of a level
   *  \return new state that is compound
   */
  base::State* createCompoundState(base::State* jointState, const VertexLevel level, std::size_t indent);

  /** \brief Add vertices to graph. The state passed in will be owned by the AdjList graph */
  TaskVertex addVertexWithLevel(base::State* state, VertexLevel level, std::size_t indent);
  TaskVertex addVertex(base::State* state, std::size_t indent);

  /** \brief Remove vertex from graph */
  void removeVertex(TaskVertex v);

  /** \brief Cleanup graph because we leave deleted vertices in graph during construction */
  void removeDeletedVertices(std::size_t indent);

  /** \brief Add edge to graph */
  TaskEdge addEdge(TaskVertex v1, TaskVertex v2, std::size_t indent);

  /** \brief Check graph for edge existence */
  bool hasEdge(TaskVertex v1, TaskVertex v2);

  /** \brief Get the state of a vertex used for querying - i.e. vertices 0-11 for 12 thread system */
  inline base::State*& getCompoundQueryStateNonConst(TaskVertex v)
  {
#ifndef NDEBUG
    BOLT_ASSERT(v < queryVertices_.size(), "Attempted to request state of regular vertex using query function");
#endif
    return queryStates_[v];
  }

  /** \brief Shortcut function for getting the state of a vertex */
  inline base::State*& getCompoundStateNonConst(TaskVertex v)
  {
#ifndef NDEBUG
    BOLT_ASSERT(v >= queryVertices_.size(), "Attempted to request state of query vertex using wrong function");
#endif
    return g_[v].state_;
  }

  /** \brief Shortcut function for getting the state of a vertex */
  inline const base::State* getCompoundState(TaskVertex v) const
  {
#ifndef NDEBUG
    BOLT_ASSERT(v >= queryVertices_.size(), "Attempted to request state of query vertex using wrong function");
#endif
    return g_[v].state_;
  }

  /** \brief Convert a compound state to just the ModelBasedStateSpace component (joints) ignoring the discrete
   * component */
  inline const base::State* getModelBasedState(const base::State* state) const
  {
    // Disregard task level
    const base::CompoundState* compoundState = static_cast<const base::CompoundState*>(state);
    return compoundState->components[MODEL_BASED];
  }

  /** \brief Convert a compound state to just the ModelBasedStateSpace component (joints) ignoring the discrete
   * component */
  inline const base::State* getModelBasedState(TaskVertex v) const
  {
    // Disregard task level
    const base::CompoundState* compoundState = static_cast<const base::CompoundState*>(g_[v].state_);
    return compoundState->components[MODEL_BASED];
  }

  /* ---------------------------------------------------------------------------------
   * Visualizations
   * --------------------------------------------------------------------------------- */

  /** \brief Show in visualizer the task graph */
  void displayDatabase(bool showVertices = false, std::size_t indent = 0);

  /** \brief Display in viewer */
  void visualizeVertex(TaskVertex v, std::size_t windowID = 2);

  void visualizeEdge(TaskEdge e, std::size_t windowID = 2);

  void visualizeEdge(TaskVertex v1, TaskVertex v2, std::size_t windowID = 2);

  /* ---------------------------------------------------------------------------------
   * Debug Utilities
   * --------------------------------------------------------------------------------- */

  /** \brief Print info to console */
  void debugState(const base::State* state);

  /** \brief Print info to console */
  void debugVertex(const TaskVertex v);

  /** \brief Print nearest neighbor info to console */
  void debugNN();

  /** \brief Information about the loaded graph */
  void printGraphStats(double generationDuration, std::size_t indent);

  /** \brief Convert two CompoundStates to ModelBased states and check motion */
  bool checkMotion(const base::State* a, const base::State* b);

  /** \brief Convert a path of compound states into only the joint states component (ModelBasedStateSpace) */
  geometric::PathGeometricPtr convertPathToNonCompound(const geometric::PathGeometricPtr compoundPath);

protected:
  /** \brief Short name of this class */
  const std::string name_ = "TaskGraph";

  /** \brief Sparse graph main datastructure that this class operates on */
  SparseGraphPtr sg_;

  /** \brief The created space information */
  base::SpaceInformationPtr compoundSI_;
  base::CompoundStateSpacePtr compoundSpace_;

  /** \brief Space info for just the robot model state space, excluding the Discrete Compound State Space */
  base::SpaceInformationPtr modelSI_;

  /** \brief Class for managing various visualization features */
  VisualizerPtr visual_;

  /** \brief Nearest neighbors data structure */
  std::shared_ptr<NearestNeighbors<TaskVertex> > nn_;

  /** \brief Connectivity graph */
  TaskAdjList g_;

  /** \brief Vertices for performing nearest neighbor queries on multiple threads */
  std::vector<TaskVertex> queryVertices_;
  std::vector<base::State*> queryStates_;

  /** \brief A path simplifier used to simplify dense paths added to S */
  geometric::PathSimplifierPtr pathSimplifier_;

  /** \brief Number of cores available on system */
  std::size_t numThreads_;

  /** \brief Astar statistics */
  std::size_t numNodesOpened_ = 0;
  std::size_t numNodesClosed_ = 0;

  /** \brief Track if the graph has been modified */
  bool graphUnsaved_ = false;

  /** \brief Remember which cartesian start/goal states should be used for distanceVertex */
  TaskVertex startConnectorVertex_ = 0;
  TaskVertex goalConnectorVertex_ = 0;
  // Find the lowest cost edge between TaskGraph and CartesianGraph
  double startConnectorMinCost_ = std::numeric_limits<double>::infinity();
  double goalConnectorMinCost_ = std::numeric_limits<double>::infinity();

  /** \brief Remeber the distances to used for the task distance heuristic */
  double shortestDistAcrossCartGraph_;

  /** \brief Flag if we are in task planning mode or not. If false, just plan in free space mode */
  bool taskPlanningEnabled_ = false;

  bool superDebug_ = false;

public:  // user settings from other applications
  /** \brief How many neighbors to a Cartesian start or goal point to attempt to connect to in the free space graph */
  std::size_t numNeighborsConnectToCart_ = 10;

  /** \brief Visualization speed of astar search, num of seconds to show each vertex */
  bool visualizeAstar_ = false;
  double visualizeAstarSpeed_ = 0.1;
  bool visualizeQualityPathSmoothing_ = false;

  /** \brief Change verbosity levels */
  bool vAdd_ = false;  // message when adding edges and vertices
  bool vSearch_ = false;
  bool vVisualize_ = false;
  bool vHeuristic_ = false;
  bool vClear_ = false;         // clearing cartesian vertices
  bool vGenerateTask_ = false;  // functions that deal with Cartesian paths
  bool verbose_ = true;         // general verbosity level for everything else

  /** \brief Show the task graph being generated */
  bool visualizeCartPath_ = false;
  bool visualizeTaskGraph_ = false;
  double visualizeTaskGraphSpeed_ = 0.0;
  bool visualizeDatabaseVertices_ = true;
  bool visualizeDatabaseEdges_ = true;

};  // end class TaskGraph

////////////////////////////////////////////////////////////////////////////////////////
/**
 * Vertex visitor to check if A* search is finished.
 * \implements AStarVisitorConcept
 * See http://www.boost.org/doc/libs/1_58_0/libs/graph/doc/AStarVisitor.html
 */
class TaskAstarVisitor : public boost::default_astar_visitor
{
private:
  TaskVertex goal_;  // Goal Vertex of the search
  TaskGraph* parent_;

public:
  /**
   * Construct a visitor for a given search.
   * \param goal  goal vertex of the search
   */
  TaskAstarVisitor(TaskVertex goal, TaskGraph* parent);

/**
 * \brief Invoked when a vertex is first discovered and is added to the OPEN list.
 * \param v current Vertex
 * \param g graph we are searching on
 */
#ifndef NDEBUG
  void discover_vertex(TaskVertex v, const TaskAdjList& g) const;
#endif

  /**
   * \brief Check if we have arrived at the goal.
   * This is invoked on a vertex as it is popped from the queue (i.e., it has the lowest
   * cost on the OPEN list). This happens immediately before examine_edge() is invoked on
   * each of the out-edges of vertex u.
   * \param v current vertex
   * \param g graph we are searching on
   * \throw FoundGoalException if \a u is the goal
   */
  void examine_vertex(TaskVertex v, const TaskAdjList& g) const;
};  // end TaskGraph

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_BOLT_TASK_GRAPH_
