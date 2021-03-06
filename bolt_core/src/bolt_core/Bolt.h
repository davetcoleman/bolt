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

/* Author: Dave Coleman
*/

#ifndef OMPL_TOOLS_BOLT_BOLT_
#define OMPL_TOOLS_BOLT_BOLT_

#include <ompl/geometric/SimpleSetup.h>  // the parent class

#include <bolt_core/SparseGraph.h>
#include <bolt_core/TaskGraph.h>
#include <ompl/tools/debug/Visualizer.h>
#include <bolt_core/BoltPlanner.h>

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/StateSpace.h>  // for storing to file

#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>

#include <ompl/util/Console.h>
#include <ompl/util/Exception.h>

namespace ompl
{
namespace tools
{
namespace bolt
{
/**
   @anchor Bolt
   @par Short description
   Bolt is an experience-based planning framework that learns to reduce computation time
   required to solve high-dimensional planning problems in varying environments.
*/

/// @cond IGNORE
OMPL_CLASS_FORWARD(Bolt);
OMPL_CLASS_FORWARD(SparseGenerator);
OMPL_CLASS_FORWARD(SparseCriteria);
OMPL_CLASS_FORWARD(SparseMirror);
/// @endcond

/** \class BoltPtr
    \brief A boost shared pointer wrapper for Bolt */

/**
 * \brief Simple logging functionality encapsled in a struct
 */
struct ExperienceStats
{
  ExperienceStats()
    : numSolutionsFromRecall_(0)
    , numSolutionsFromRecallSaved_(0)
    , numSolutionsFromScratch_(0)
    , numSolutionsFailed_(0)
    , numSolutionsTimedout_(0)
    , numSolutionsApproximate_(0)
    , numSolutionsTooShort_(0)
    , numProblems_(0)
    , totalPlanningTime_(0.0)
    , totalInsertionTime_(0.0)
  {
  }

  double getAveragePlanningTime() const
  {
    if (!numProblems_)
      return 0.0;

    return totalPlanningTime_ / numProblems_;
  }

  double getAverageInsertionTime() const
  {
    if (!numProblems_)
      return 0.0;

    // Clean up output
    double time = totalInsertionTime_ / numProblems_;
    if (time < 1e-8)
      return 0.0;
    else
      return totalInsertionTime_ / numProblems_;
  }

  double numSolutionsFromRecall_;
  double numSolutionsFromRecallSaved_;
  double numSolutionsFromScratch_;
  double numSolutionsFailed_;
  double numSolutionsTimedout_;
  double numSolutionsApproximate_;
  double numSolutionsTooShort_;  // less than 3 states
  double numProblems_;           // input requests
  double totalPlanningTime_;     // of all input requests, used for averaging
  double totalInsertionTime_;    // of all input requests, used for averaging
};

/** \brief Built off of SimpleSetup but provides support for planning from experience */
class Bolt : public geometric::SimpleSetup
{
public:

  /** \brief Constructor needs the state space used for planning. */
  explicit Bolt(const base::SpaceInformationPtr &si);

  /** \brief Constructor needs the state space used for planning.
   *  \param space - the state space to plan in
   */
  explicit Bolt(const base::StateSpacePtr &space);

private:
  /** \brief Shared constructor functions */
  void initialize(std::size_t indent = 0);

public:
  /** \brief Display debug data about potential available solutions */
  void printResultsInfo(std::ostream &out = std::cout) const;

  /** \brief Display debug data about overall results from Bolt since being loaded */
  void printLogs(std::ostream &out = std::cout) const;

  /** \brief Load database from file  */
  bool load(std::size_t indent);

  /** \brief Get the current planner */
  base::PlannerPtr &getPlanner()
  {
    return planner_;
  }

  /** \brief Get a pointer to the retrieve repair planner */
  BoltPlannerPtr getBoltPlanner() const
  {
    return boltPlanner_;
  }

  /** \brief Set the planner to use for repairing experience paths
      inside the BoltPlanner planner. If the planner is not
      set, a default planner is set. */
  void setRepairPlanner(const base::PlannerPtr &planner)
  {
    // This is required by the parent class but we no longer use this feature
    // static_cast<BoltPlanner&>(*boltPlanner_).setRepairPlanner(planner);
  }

  /** \brief Set the planner allocator to use. This is only
      used if no planner has been set. This is optional -- a default
      planner will be used if no planner is otherwise specified. */
  void setPlannerAllocator(const base::PlannerAllocator &pa);

  /** \brief Run the planner for up to a specified amount of time (default is 1 second) */
  virtual base::PlannerStatus solve(double time = 1.0);

  void visualize(std::size_t indent);

  bool checkBoltPlannerOptimality(std::size_t indent);

  /** \brief Logging  data to file and visualizing*/
  void processResults(std::size_t indent);

  /** \brief Run the planner until \e ptc becomes true (at most) */
  virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

  /** \brief Sanity check for solution paths */
  bool checkRepeatedStates(const geometric::PathGeometric &path, std::size_t indent);

  /** \brief Set the database file to load. Actual loading occurs when setup() is called
   *  \param filePath - full absolute path to a experience database to load
   */
  bool setFilePath(const std::string &filePath);

  /** \brief Save the experience database to file */
  bool save();

  /** \brief Save the experience database to file if there has been a change */
  bool saveIfChanged();

  /** \brief Do not clear the experience database */
  void clearForNextPlan();

  /** \brief Clear all planning data. This only includes
      data generated by motion plan computation. Planner
      settings, start & goal states are not affected. */
  virtual void clear();

  /** \brief Print information about the current setup */
  virtual void print(std::ostream &out = std::cout) const;

  /** \brief This method will create the necessary classes
      for planning. The solve() method will call this function automatically. */
  virtual void setup();

  /** \brief Get a vector of all the planning data in the database */
  void getAllPlannerDatas(std::vector<base::PlannerDataPtr> &plannerDatas) const {};

  /** \brief Get the total number of paths stored in the database */
  std::size_t getExperiencesCount() const
  {
    return sparseGraph_->getNumVertices();
  }

  /** \brief Hook for getting access to common functions */
  SparseGraphPtr getSparseGraph()
  {
    return sparseGraph_;
  }

  TaskGraphPtr getTaskGraph()
  {
    return taskGraph_;
  }

  SparseCriteriaPtr getSparseCriteria()
  {
    return sparseCriteria_;
  }

  SparseMirrorPtr getSparseMirror()
  {
    return sparseMirror_;
  }

  SparseGeneratorPtr getSparseGenerator()
  {
    return sparseGenerator_;
  }

  /** \brief Get class for managing various visualization features */
  VisualizerPtr getVisual()
  {
    return visual_;
  }

  /** \brief Allow accumlated experiences to be processed */
  // bool doPostProcessing();

protected:

  // This is included in parent class, but mentioned here in contrast to modelSI_
  //base::SpaceInformationPtr si_;

  /** \brief The space information for combined states */
  base::SpaceInformationPtr compoundSI_;

  /**  The maintained experience planner instance */
  BoltPlannerPtr boltPlanner_;

  /** \brief The graph that contains a sparse roadmap of the space */
  SparseGraphPtr sparseGraph_;

  /** \brief Various tests to determine if a vertex/edge should be added to the graph, based on SPARS */
  SparseCriteriaPtr sparseCriteria_;

  /** \brief Generator of sparse vertices and edges */
  SparseGeneratorPtr sparseGenerator_;

  /** \brief Duplicate one arm to second arm */
  SparseMirrorPtr sparseMirror_;

  /** \brief Graph used for combining multiple layers of sparse graph */
  TaskGraphPtr taskGraph_;

  /** \brief Accumulated experiences to be later added to experience database */
  std::vector<geometric::PathGeometric> queuedSolutionPaths_;

  /** \brief Location to save logging file for benchmarks */
  std::string benchmarkFilePath_;

  /** \brief File location of database */
  std::string filePath_;

  /** \brief Class for managing various visualization features */
  VisualizerPtr visual_;

  /** \brief States data for display to console  */
  ExperienceStats stats_;

public:

  const std::string name_ = "Bolt";

  /** \brief Verbose settings */
  bool verbose_ = true;

  /** \brief Visualize original solution from graph before smoothing */
  bool visualizeRawTrajectory_ = false;
  /** \brief Visualize solution from graph after smoothing */
  bool visualizeSmoothTrajectory_ = true;
  bool visualizeRobotTrajectory_ = true;

};  // end of class Bolt

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
#endif
