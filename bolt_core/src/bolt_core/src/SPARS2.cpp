/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
*  All Rights Reserved.
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
*   * Neither the name of Rutgers University nor the names of its
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

/* Author: Andrew Dobson */

#include <bolt_core/SPARS2.h>
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"
#include <ompl/base/DiscreteMotionValidator.h>

#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>
#include <thread>

#include "GoalVisitor.hpp"

#define foreach BOOST_FOREACH
#define foreach_reverse BOOST_REVERSE_FOREACH

ompl::geometric::SPARS2::SPARS2(const base::SpaceInformationPtr &si)
  : base::Planner(si, "SPARS2")
  , stretchFactor_(3.)
  , sparseDeltaFraction_(.25)
  , denseDeltaFraction_(.001)
  , maxFailures_(5000)
  , nearSamplePoints_((2 * si_->getStateDimension()))
  , stateProperty_(boost::get(vertex_state_t(), g_))
  , weightProperty_(boost::get(boost::edge_weight, g_))
  , colorProperty_(boost::get(vertex_color_t(), g_))
  , interfaceDataProperty_(boost::get(vertex_interface_data_t(), g_))
  , disjointSets_(boost::get(boost::vertex_rank, g_), boost::get(boost::vertex_predecessor, g_))
  , addedSolution_(false)
  , consecutiveFailures_(0)
  , sparseDelta_(0.)
  , denseDelta_(0.)
  , iterations_(0)
  , bestCost_(std::numeric_limits<double>::quiet_NaN())
{
  specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
  specs_.approximateSolutions = false;
  specs_.optimizingPaths = true;
  specs_.multithreaded = true;

  psimp_ = std::make_shared<PathSimplifier>(si_);

  // Smoothing paths in ideal way for SPARS criteria */
  sparseSmoother_.reset(new ompl::tools::bolt::SparseSmoother(si_, visual_));

  Planner::declareParam<double>("stretch_factor", this, &SPARS2::setStretchFactor, &SPARS2::getStretchFactor, "1.1:0.1:"
                                                                                                              "3.0");
  Planner::declareParam<double>("sparse_delta_fraction", this, &SPARS2::setSparseDeltaFraction,
                                &SPARS2::getSparseDeltaFraction, "0.0:0.01:1.0");
  Planner::declareParam<double>("dense_delta_fraction", this, &SPARS2::setDenseDeltaFraction,
                                &SPARS2::getDenseDeltaFraction, "0.0:0.0001:0.1");
  Planner::declareParam<unsigned int>("max_failures", this, &SPARS2::setMaxFailures, &SPARS2::getMaxFailures, "100:10:"
                                                                                                              "3000");

  addPlannerProgressProperty("iterations INTEGER", [this]
                             {
                               return getIterationCount();
                             });
  addPlannerProgressProperty("best cost REAL", [this]
                             {
                               return getBestCost();
                             });
}

ompl::geometric::SPARS2::~SPARS2()
{
  freeMemory();
}

void ompl::geometric::SPARS2::setup()
{
  Planner::setup();
  if (!nn_)
    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
  nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                           {
                             return distanceFunction(a, b);
                           });
  double maxExt = si_->getMaximumExtent();
  sparseDelta_ = sparseDeltaFraction_ * maxExt;
  denseDelta_ = denseDeltaFraction_ * maxExt;

  // Setup optimization objective
  //
  // If no optimization objective was specified, then default to
  // optimizing path length as computed by the distance() function
  // in the state space.
  if (pdef_)
  {
    if (pdef_->hasOptimizationObjective())
    {
      opt_ = pdef_->getOptimizationObjective();
      if (!dynamic_cast<base::PathLengthOptimizationObjective *>(opt_.get()))
        OMPL_WARN("%s: Asymptotic optimality has only been proven with path length optimizaton; convergence "
                  "for other optimizaton objectives is not guaranteed.",
                  getName().c_str());
    }
    else
      opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
  }
  else
  {
    OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
    setup_ = false;
  }

  sparseSmoother_->setup();
}

void ompl::geometric::SPARS2::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
  Planner::setProblemDefinition(pdef);
  clearQuery();
}

void ompl::geometric::SPARS2::clearQuery()
{
  startM_.clear();
  goalM_.clear();
  pis_.restart();
}

void ompl::geometric::SPARS2::clear()
{
  Planner::clear();
  clearQuery();
  resetFailures();
  iterations_ = 0;
  bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
  freeMemory();
  if (nn_)
    nn_->clear();
}

void ompl::geometric::SPARS2::freeMemory()
{
  Planner::clear();
  sampler_.reset();

  foreach (Vertex v, boost::vertices(g_))
  {
    foreach (InterfaceData &d, interfaceDataProperty_[v] | boost::adaptors::map_values)
      d.clear(si_);
    if (stateProperty_[v] != nullptr)
      si_->freeState(stateProperty_[v]);
    stateProperty_[v] = nullptr;
  }
  g_.clear();

  if (nn_)
    nn_->clear();
}

bool ompl::geometric::SPARS2::haveSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals,
                                           base::PathPtr &solution)
{
  base::Goal *g = pdef_->getGoal().get();
  base::Cost sol_cost(opt_->infiniteCost());
  foreach (Vertex start, starts)
    foreach (Vertex goal, goals)
    {
      // we lock because the connected components algorithm is incremental and may change disjointSets_
      graphMutex_.lock();
      bool same_component = sameComponent(start, goal);
      graphMutex_.unlock();

      if (same_component && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
      {
        base::PathPtr p = constructSolution(start, goal);
        if (p)
        {
          base::Cost pathCost = p->cost(opt_);
          if (opt_->isCostBetterThan(pathCost, bestCost_))
            bestCost_ = pathCost;
          // Check if optimization objective is satisfied
          if (opt_->isSatisfied(pathCost))
          {
            solution = p;
            return true;
          }
          else if (opt_->isCostBetterThan(pathCost, sol_cost))
          {
            solution = p;
            sol_cost = pathCost;
          }
        }
      }
    }
  return false;
}

bool ompl::geometric::SPARS2::sameComponent(Vertex m1, Vertex m2)
{
  return boost::same_component(m1, m2, disjointSets_);
}

bool ompl::geometric::SPARS2::reachedFailureLimit() const
{
  return consecutiveFailures_ >= maxFailures_;
}

bool ompl::geometric::SPARS2::reachedTerminationCriterion() const
{
  return consecutiveFailures_ >= maxFailures_ || addedSolution_;
}

void ompl::geometric::SPARS2::copyPasteState(std::size_t numSets)
{
  lastGraphGenerationTime_ = time::seconds(time::now() - timeDiscretizeAndRandomStarted_);
  std::stringstream line;

  // clang-format off
    line << "=SPLIT(\"SPARS2, "
         << map_name_ << ", "
         << sparseDeltaFraction_ << ", "
         << sparseDelta_ << ", "
         << 0 << ", "
         << stretchFactor_ << ", "
         << 2 << ", " // nearSamplePointsFactor hardcoded at top
         << 0 << ", " // useDiscretizedSamples
         << 1 << ", " // useRandomSamples
         << 0 << ", " // useCheckRemoveCloseVertices
         << 0 << ", " // useClearEdgesNearVertex_
         << 0 << ", " // useOriginalSmoother_
         << 0 << ", " // useEdgeImprovementRule_
         << 0 << ", " // fourthCriteriaAfterFailures_
         << maxFailures_ << ", "
         << consecutiveFailures_ << ", "
         << milestoneCount() << ", "
         << getNumEdges() << ", "
         << 0 << ", "
         << lastGraphGenerationTime_ << "\", \",\")";
  // clang-format on

  // Save log
  stringLog_.push_back(line.str());

  if (stringLog_.size() > 1000)
    OMPL_WARN("Copy Paste Log is getting big: %i", stringLog_.size());

  // Output to console
  std::cout << stringLog_.back() << std::endl;
}

void ompl::geometric::SPARS2::dumpLog()
{
  // Dump to console
  for (auto line : stringLog_)
    std::cout << line << std::endl;

  stringLog_.clear();
}

void ompl::geometric::SPARS2::constructRoadmap(const base::PlannerTerminationCondition &ptc, bool stopOnMaxFail)
{
  if (stopOnMaxFail)
  {
    resetFailures();
    base::PlannerTerminationCondition ptcOrFail([this, &ptc]
                                                {
                                                  return ptc || reachedFailureLimit();
                                                });
    constructRoadmap(ptcOrFail);
  }
  else
    constructRoadmap(ptc);

  OMPL_INFORM("Construct Roadmap Finished");
  copyPasteState(0);
}

void ompl::geometric::SPARS2::constructRoadmap(const base::PlannerTerminationCondition &ptc)
{
  // DTC
  timeDiscretizeAndRandomStarted_ = time::now();
  maxConsecutiveFailures_ = 0;
  maxPercentComplete_ = 0;

  checkQueryStateInitialization();

  if (!isSetup())
    setup();
  if (!sampler_)
  {
    // Load minimum clearance state sampler
    assert(clearance_ > std::numeric_limits<double>::epsilon());
    sampler_ = base::MinimumClearanceValidStateSamplerPtr(new base::MinimumClearanceValidStateSampler(si_.get()));
    sampler_->setMinimumObstacleClearance(clearance_);
    si_->getStateValidityChecker()->setClearanceSearchDistance(clearance_);
  }

  base::State *qNew = si_->allocState();
  base::State *workState = si_->allocState();

  /* The whole neighborhood set which has been most recently computed */
  std::vector<Vertex> graphNeighborhood;
  /* The visible neighborhood set which has been most recently computed */
  std::vector<Vertex> visibleNeighborhood;

  bestCost_ = opt_->infiniteCost();
  while (ptc == false && !visual_->viz1()->shutdownRequested())
  {
    ++iterations_;
    ++consecutiveFailures_;

    showFailureProgress();
    if (visual_->viz1()->shutdownRequested())
      break;

    // Generate a single sample, and attempt to connect it to nearest neighbors.
    if (!sampler_->sample(qNew))
      continue;

    findGraphNeighbors(qNew, sparseDelta_, graphNeighborhood, visibleNeighborhood);

    if (!checkAddCoverage(qNew, visibleNeighborhood))
      if (!checkAddConnectivity(qNew, visibleNeighborhood))
        if (!checkAddInterface(qNew, graphNeighborhood, visibleNeighborhood))
        {
          if (visibleNeighborhood.size() > 0)
          {
            std::map<Vertex, base::State *> closeRepresentatives;
            findCloseRepresentatives(workState, qNew, visibleNeighborhood[0], closeRepresentatives, ptc);
            for (auto &closeRepresentative : closeRepresentatives)
            {
              updatePairPoints(visibleNeighborhood[0], qNew, closeRepresentative.first, closeRepresentative.second);
              updatePairPoints(closeRepresentative.first, closeRepresentative.second, visibleNeighborhood[0], qNew);
            }
            checkAddPath(visibleNeighborhood[0]);
            for (auto &closeRepresentative : closeRepresentatives)
            {
              checkAddPath(closeRepresentative.first);
              si_->freeState(closeRepresentative.second);
            }
          }
        }
  }
  si_->freeState(workState);
  si_->freeState(qNew);
}

void ompl::geometric::SPARS2::showFailureProgress()
{
  if (consecutiveFailures_ > maxConsecutiveFailures_)
  {
    maxConsecutiveFailures_ = consecutiveFailures_;

    std::size_t percentComplete = ceil(maxConsecutiveFailures_ / double(maxFailures_) * 100.0);

    // Every time the whole number of percent compelete changes, show to user
    if (percentComplete > maxPercentComplete_)
    {
      maxPercentComplete_ = percentComplete;

      // Show varying granularity based on number of dimensions
      static const std::size_t showEvery = std::max(1, int(12 - si_->getStateDimension() * 2));
      if (percentComplete % showEvery == 0)
      {
        OMPL_INFORM("Termination progress: %u", percentComplete);
      }
    }
  }
}

void ompl::geometric::SPARS2::checkQueryStateInitialization()
{
  std::lock_guard<std::mutex> _(graphMutex_);
  if (boost::num_vertices(g_) < 1)
  {
    queryVertex_ = boost::add_vertex(g_);
    stateProperty_[queryVertex_] = nullptr;
  }
}

ompl::base::PlannerStatus ompl::geometric::SPARS2::solve(const base::PlannerTerminationCondition &ptc)
{
  checkValidity();
  checkQueryStateInitialization();

  base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

  if (!goal)
  {
    OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
    return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
  }

  // Add the valid start states as milestones
  while (const base::State *st = pis_.nextStart())
    startM_.push_back(addGuard(si_->cloneState(st), START));
  if (startM_.empty())
  {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  if (!goal->couldSample())
  {
    OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
    return base::PlannerStatus::INVALID_GOAL;
  }

  // Add the valid goal states as milestones
  while (const base::State *st = (goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal()))
    goalM_.push_back(addGuard(si_->cloneState(st), GOAL));
  if (goalM_.empty())
  {
    OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
    return base::PlannerStatus::INVALID_GOAL;
  }

  unsigned int nrStartStates = boost::num_vertices(g_) - 1;  // don't count query vertex
  OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nrStartStates);

  // Reset addedSolution_ member
  addedSolution_ = false;
  resetFailures();
  base::PathPtr sol;
  base::PlannerTerminationCondition ptcOrFail([this, &ptc]
                                              {
                                                return ptc || reachedFailureLimit();
                                              });
  std::thread slnThread([this, &ptcOrFail, &sol]
                        {
                          checkForSolution(ptcOrFail, sol);
                        });

  // Construct planner termination condition which also takes M into account
  base::PlannerTerminationCondition ptcOrStop([this, &ptc]
                                              {
                                                return ptc || reachedTerminationCriterion();
                                              });
  constructRoadmap(ptcOrStop);

  // Ensure slnThread is ceased before exiting solve
  slnThread.join();

  OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

  if (sol)
    pdef_->addSolutionPath(sol, false, -1.0, getName());

  // Return true if any solution was found.
  return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::SPARS2::checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution)
{
  base::GoalSampleableRegion *goal = static_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
  while (!ptc && !addedSolution_)
  {
    // Check for any new goal states
    if (goal->maxSampleCount() > goalM_.size())
    {
      const base::State *st = pis_.nextGoal();
      if (st)
        goalM_.push_back(addGuard(si_->cloneState(st), GOAL));
    }

    // Check for a solution
    addedSolution_ = haveSolution(startM_, goalM_, solution);
    // Sleep for 1ms
    if (!addedSolution_)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

bool ompl::geometric::SPARS2::checkAddCoverage(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood)
{
  if (visibleNeighborhood.size() > 0)
    return false;
  // No free paths means we add for coverage
  addGuard(si_->cloneState(qNew), COVERAGE);
  return true;
}

bool ompl::geometric::SPARS2::checkAddConnectivity(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood)
{
  std::vector<Vertex> links;
  if (visibleNeighborhood.size() > 1)
  {
    // For each neighbor
    for (std::size_t i = 0; i < visibleNeighborhood.size(); ++i)
      // For each other neighbor
      for (std::size_t j = i + 1; j < visibleNeighborhood.size(); ++j)
        // If they are in different components
        if (!sameComponent(visibleNeighborhood[i], visibleNeighborhood[j]))
        {
          links.push_back(visibleNeighborhood[i]);
          links.push_back(visibleNeighborhood[j]);
        }

    if (links.size() > 0)
    {
      // Add the node
      Vertex g = addGuard(si_->cloneState(qNew), CONNECTIVITY);

      for (unsigned long link : links)
        // If there's no edge
        if (!boost::edge(g, link, g_).second)
          // And the components haven't been united by previous links
          if (!sameComponent(link, g))
            connectGuards(g, link);
      return true;
    }
  }
  return false;
}

bool ompl::geometric::SPARS2::checkAddInterface(const base::State *qNew, std::vector<Vertex> &graphNeighborhood,
                                                std::vector<Vertex> &visibleNeighborhood)
{
  bool skipThis = false;

  // If we have more than 1 or 0 neighbors
  if (visibleNeighborhood.size() > 1)
    if (skipThis || (graphNeighborhood[0] == visibleNeighborhood[0] && graphNeighborhood[1] == visibleNeighborhood[1]))
      // If our two closest neighbors don't share an edge
      if (!boost::edge(visibleNeighborhood[0], visibleNeighborhood[1], g_).second)
      {
        // If they can be directly connected
        if (si_->checkMotion(stateProperty_[visibleNeighborhood[0]], stateProperty_[visibleNeighborhood[1]]))
        {
          // Connect them
          connectGuards(visibleNeighborhood[0], visibleNeighborhood[1]);
          // And report that we added to the roadmap
          resetFailures();
          // Report success
          return true;
        }
        else
        {
          // Add the new node to the graph, to bridge the interface
          Vertex v = addGuard(si_->cloneState(qNew), INTERFACE);
          connectGuards(v, visibleNeighborhood[0]);
          connectGuards(v, visibleNeighborhood[1]);
          // Report success
          return true;
        }
      }
  return false;
}

bool ompl::geometric::SPARS2::checkAddPath(Vertex v)
{
  bool ret = false;

  std::vector<Vertex> rs;
  foreach (Vertex r, boost::adjacent_vertices(v, g_))
    rs.push_back(r);

  /* Candidate x vertices as described in the method, filled by function computeX(). */
  std::vector<Vertex> Xs;

  /* Candidate v" vertices as described in the method, filled by function computeVPP(). */
  std::vector<Vertex> VPPs;

  for (std::size_t i = 0; i < rs.size() && !ret; ++i)
  {
    Vertex r = rs[i];
    computeVPP(v, r, VPPs);
    foreach (Vertex rp, VPPs)
    {
      // First, compute the longest path through the graph
      computeX(v, r, rp, Xs);
      double rm_dist = 0.0;
      foreach (Vertex rpp, Xs)
      {
        double tmp_dist = (si_->distance(stateProperty_[r], stateProperty_[v]) +
                           si_->distance(stateProperty_[v], stateProperty_[rpp])) /
                          2.0;
        if (tmp_dist > rm_dist)
          rm_dist = tmp_dist;
      }

      InterfaceData &d = getData(v, r, rp);

      // Then, if the spanner property is violated
      if (rm_dist > stretchFactor_ * d.d_)  // TODO - check that d.d_ is not zero!!
      {
        ret = true;  // Report that we added for the path
        if (si_->checkMotion(stateProperty_[r], stateProperty_[rp]))
          connectGuards(r, rp);
        else
        {
          auto p(std::make_shared<PathGeometric>(si_));
          if (r < rp)
          {
            p->append(d.sigmaA_);
            p->append(d.pointA_);
            p->append(stateProperty_[v]);
            p->append(d.pointB_);
            p->append(d.sigmaB_);
          }
          else
          {
            p->append(d.sigmaB_);
            p->append(d.pointB_);
            p->append(stateProperty_[v]);
            p->append(d.pointA_);
            p->append(d.sigmaA_);
          }

          // Smoothing path - old method
          // psimp_->reduceVertices(*p, 10);
          // psimp_->shortcutPath(*p, 50);

          // Smoothing path - new method
          const bool debug = false;
          if (!sparseSmoother_->smoothQualityPath(p.get(), clearance_, debug, 0 /*indent*/))
            return false;  // unable to smooth

          if (p->checkAndRepair(100).second)
          {
            Vertex prior = r;
            Vertex vnew;
            std::vector<base::State *> &states = p->getStates();

            foreach (base::State *st, states)
            {
              // no need to clone st, since we will destroy p; we just copy the pointer
              vnew = addGuard(st, QUALITY);

              connectGuards(prior, vnew);
              prior = vnew;
            }
            // clear the states, so memory is not freed twice
            states.clear();
            connectGuards(prior, rp);
          }
        }
      }
    }
  }

  return ret;
}

void ompl::geometric::SPARS2::resetFailures()
{
  consecutiveFailures_ = 0;
}

void ompl::geometric::SPARS2::findGraphNeighbors(base::State *st, double dist, std::vector<Vertex> &graphNeighborhood,
                                                 std::vector<Vertex> &visibleNeighborhood)
{
  visibleNeighborhood.clear();
  stateProperty_[queryVertex_] = st;
  nn_->nearestR(queryVertex_, dist, graphNeighborhood);
  stateProperty_[queryVertex_] = nullptr;

  // Now that we got the neighbors from the NN, we must remove any we can't see
  for (unsigned long i : graphNeighborhood)
    if (si_->checkMotion(st, stateProperty_[i]))
      visibleNeighborhood.push_back(i);
}

void ompl::geometric::SPARS2::approachGraph(Vertex v)
{
  std::vector<Vertex> hold;
  nn_->nearestR(v, sparseDelta_, hold);

  std::vector<Vertex> neigh;
  for (unsigned long i : hold)
    if (si_->checkMotion(stateProperty_[v], stateProperty_[i]))
      neigh.push_back(i);

  foreach (Vertex vp, neigh)
    connectGuards(v, vp);
}

ompl::geometric::SPARS2::Vertex ompl::geometric::SPARS2::findGraphRepresentative(base::State *st)
{
  std::vector<Vertex> nbh;
  stateProperty_[queryVertex_] = st;
  nn_->nearestR(queryVertex_, sparseDelta_, nbh);
  stateProperty_[queryVertex_] = nullptr;

  Vertex result = boost::graph_traits<Graph>::null_vertex();

  for (unsigned long i : nbh)
    if (si_->checkMotion(st, stateProperty_[i]))
    {
      result = i;
      break;
    }
  return result;
}

void ompl::geometric::SPARS2::findCloseRepresentatives(base::State *workArea, const base::State *qNew,
                                                       const Vertex qRep,
                                                       std::map<Vertex, base::State *> &closeRepresentatives,
                                                       const base::PlannerTerminationCondition &ptc)
{
  for (auto &closeRepresentative : closeRepresentatives)
    si_->freeState(closeRepresentative.second);
  closeRepresentatives.clear();

  // Then, begin searching the space around him
  for (unsigned int i = 0; i < nearSamplePoints_; ++i)
  {
    do
    {
      sampler_->sampleNear(workArea, qNew, denseDelta_);
    } while (
        (!si_->isValid(workArea) || si_->distance(qNew, workArea) > denseDelta_ || !si_->checkMotion(qNew, workArea)) &&
        ptc == false);

    // if we were not successful at sampling a desirable state, we are out of time
    if (ptc == true)
      break;

    // Compute who his graph neighbors are
    Vertex representative = findGraphRepresentative(workArea);

    // Assuming this sample is actually seen by somebody (which he should be in all likelihood)
    if (representative != boost::graph_traits<Graph>::null_vertex())
    {
      // If his representative is different than qNew
      if (qRep != representative)
        // And we haven't already tracked this representative
        if (closeRepresentatives.find(representative) == closeRepresentatives.end())
          // Track the representative
          closeRepresentatives[representative] = si_->cloneState(workArea);
    }
    else
    {
      // This guy can't be seen by anybody, so we should take this opportunity to add him
      addGuard(si_->cloneState(workArea), COVERAGE);

      // We should also stop our efforts to add a dense path
      for (auto &closeRepresentative : closeRepresentatives)
        si_->freeState(closeRepresentative.second);
      closeRepresentatives.clear();
      break;
    }
  }
}

void ompl::geometric::SPARS2::updatePairPoints(Vertex rep, const base::State *q, Vertex r, const base::State *s)
{
  // First of all, we need to compute all candidate r'
  std::vector<Vertex> VPPs;
  computeVPP(rep, r, VPPs);

  // Then, for each pair Pv(r,r')
  foreach (Vertex rp, VPPs)
    // Try updating the pair info
    distanceCheck(rep, q, r, s, rp);
}

void ompl::geometric::SPARS2::computeVPP(Vertex v, Vertex vp, std::vector<Vertex> &VPPs)
{
  VPPs.clear();
  foreach (Vertex cvpp, boost::adjacent_vertices(v, g_))
    if (cvpp != vp)
      if (!boost::edge(cvpp, vp, g_).second)
        VPPs.push_back(cvpp);
}

void ompl::geometric::SPARS2::computeX(Vertex v, Vertex vp, Vertex vpp, std::vector<Vertex> &Xs)
{
  Xs.clear();

  foreach (Vertex cx, boost::adjacent_vertices(vpp, g_))
    if (boost::edge(cx, v, g_).second && !boost::edge(cx, vp, g_).second)
    {
      InterfaceData &d = getData(v, vpp, cx);
      if ((vpp < cx && d.pointA_) || (cx < vpp && d.pointB_))
        Xs.push_back(cx);
    }
  Xs.push_back(vpp);
}

ompl::geometric::SPARS2::VertexPair ompl::geometric::SPARS2::index(Vertex vp, Vertex vpp)
{
  if (vp < vpp)
    return VertexPair(vp, vpp);
  else if (vpp < vp)
    return VertexPair(vpp, vp);
  else
    throw Exception(name_, "Trying to get an index where the pairs are the same point!");
}

ompl::geometric::SPARS2::InterfaceData &ompl::geometric::SPARS2::getData(Vertex v, Vertex vp, Vertex vpp)
{
  return interfaceDataProperty_[v][index(vp, vpp)];
}

void ompl::geometric::SPARS2::distanceCheck(Vertex rep, const base::State *q, Vertex r, const base::State *s, Vertex rp)
{
  // Get the info for the current representative-neighbors pair
  InterfaceData &d = getData(rep, r, rp);

  if (r < rp)  // FIRST points represent r (the guy discovered through sampling)
  {
    if (d.pointA_ == nullptr)  // If the point we're considering replacing (P_v(r,.)) isn't there
      // Then we know we're doing better, so add it
      d.setFirst(q, s, si_);
    else  // Otherwise, he is there,
    {
      if (d.pointB_ == nullptr)  // But if the other guy doesn't exist, we can't compare.
      {
        // Should probably keep the one that is further away from rep?  Not known what to do in this case.
        // \todo: is this not part of the algorithm?
      }
      else  // We know both of these points exist, so we can check some distances
          if (si_->distance(q, d.pointB_) < si_->distance(d.pointA_, d.pointB_))
        // Distance with the new point is good, so set it.
        d.setFirst(q, s, si_);
    }
  }
  else  // SECOND points represent r (the guy discovered through sampling)
  {
    if (d.pointB_ == nullptr)  // If the point we're considering replacing (P_V(.,r)) isn't there...
      // Then we must be doing better, so add it
      d.setSecond(q, s, si_);
    else  // Otherwise, he is there
    {
      if (d.pointA_ == nullptr)  // But if the other guy doesn't exist, we can't compare.
      {
        // Should we be doing something cool here?
      }
      else if (si_->distance(q, d.pointA_) < si_->distance(d.pointB_, d.pointA_))
        // Distance with the new point is good, so set it
        d.setSecond(q, s, si_);
    }
  }

  // Lastly, save what we have discovered
  interfaceDataProperty_[rep][index(r, rp)] = d;
}

void ompl::geometric::SPARS2::abandonLists(base::State *st)
{
  stateProperty_[queryVertex_] = st;

  std::vector<Vertex> hold;
  nn_->nearestR(queryVertex_, sparseDelta_, hold);

  stateProperty_[queryVertex_] = nullptr;

  // For each of the vertices
  foreach (Vertex v, hold)
  {
    foreach (VertexPair r, interfaceDataProperty_[v] | boost::adaptors::map_keys)
      interfaceDataProperty_[v][r].clear(si_);
  }
}

ompl::geometric::SPARS2::Vertex ompl::geometric::SPARS2::addGuard(base::State *state, GuardType type)
{
  // Visualize vertex
  visual_->viz1()->state(state, tools::LARGE, tools::BLACK, 0);

  // std::lock_guard<std::mutex> _(graphMutex_);

  Vertex m = boost::add_vertex(g_);
  stateProperty_[m] = state;
  colorProperty_[m] = type;

  assert(si_->isValid(state));
  abandonLists(state);

  disjointSets_.make_set(m);
  nn_->add(m);
  resetFailures();

  return m;
}

void ompl::geometric::SPARS2::connectGuards(Vertex v, Vertex vp)
{
  assert(v <= milestoneCount());
  assert(vp <= milestoneCount());

  const base::Cost weight(costHeuristic(v, vp));
  const Graph::edge_property_type properties(weight);

  // Visualize edge
  visual_->viz1()->edge(stateProperty_[v], stateProperty_[vp], tools::MEDIUM, tools::BLUE);

  // std::lock_guard<std::mutex> _(graphMutex_);
  boost::add_edge(v, vp, properties, g_);
  disjointSets_.union_set(v, vp);
}

ompl::base::PathPtr ompl::geometric::SPARS2::constructSolution(const Vertex start, const Vertex goal) const
{
  std::lock_guard<std::mutex> _(graphMutex_);

  boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));

  try
  {
    boost::astar_search(g_, start,
                        [this, goal](Vertex v)
                        {
                          return costHeuristic(v, goal);
                        },
                        boost::predecessor_map(prev)
                            .distance_compare([this](base::Cost c1, base::Cost c2)
                                              {
                                                return opt_->isCostBetterThan(c1, c2);
                                              })
                            .distance_combine([this](base::Cost c1, base::Cost c2)
                                              {
                                                return opt_->combineCosts(c1, c2);
                                              })
                            .distance_inf(opt_->infiniteCost())
                            .distance_zero(opt_->identityCost())
                            .visitor(AStarGoalVisitor<Vertex>(goal)));
  }
  catch (AStarFoundGoal &)
  {
  }

  if (prev[goal] == goal)
    throw Exception(name_, "Could not find solution path");
  else
  {
    auto p(std::make_shared<PathGeometric>(si_));
    for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
    {
      p->append(stateProperty_[pos]);
    }
    p->append(stateProperty_[start]);
    p->reverse();

    return p;
  }
}

void ompl::geometric::SPARS2::getPlannerData(base::PlannerData &data) const
{
  Planner::getPlannerData(data);

  // Explicitly add start and goal states:
  for (unsigned long i : startM_)
    data.addStartVertex(base::PlannerDataVertex(stateProperty_[i], (int)START));

  for (unsigned long i : goalM_)
    data.addGoalVertex(base::PlannerDataVertex(stateProperty_[i], (int)GOAL));

  // If there are even edges here
  if (boost::num_edges(g_) > 0)
  {
    // Adding edges and all other vertices simultaneously
    foreach (const Edge e, boost::edges(g_))
    {
      const Vertex v1 = boost::source(e, g_);
      const Vertex v2 = boost::target(e, g_);
      data.addEdge(base::PlannerDataVertex(stateProperty_[v1], (int)colorProperty_[v1]),
                   base::PlannerDataVertex(stateProperty_[v2], (int)colorProperty_[v2]));

      // Add the reverse edge, since we're constructing an undirected roadmap
      data.addEdge(base::PlannerDataVertex(stateProperty_[v2], (int)colorProperty_[v2]),
                   base::PlannerDataVertex(stateProperty_[v1], (int)colorProperty_[v1]));
    }
  }
  else
    OMPL_INFORM("%s: There are no edges in the graph!", getName().c_str());

  // Make sure to add edge-less nodes as well
  foreach (const Vertex n, boost::vertices(g_))
    if (boost::out_degree(n, g_) == 0)
      data.addVertex(base::PlannerDataVertex(stateProperty_[n], (int)colorProperty_[n]));
}

void ompl::geometric::SPARS2::printDebug(std::ostream &out) const
{
  out << "SPARS2 Debug Output: " << std::endl;
  out << "  Settings: " << std::endl;
  out << "    Max Failures: " << getMaxFailures() << std::endl;
  out << "    Dense Delta Fraction: " << getDenseDeltaFraction() << std::endl;
  out << "    Sparse Delta Fraction: " << getSparseDeltaFraction() << std::endl;
  out << "    Stretch Factor: " << getStretchFactor() << std::endl;
  out << "  Status: " << std::endl;
  out << "    Milestone Count: " << milestoneCount() << std::endl;
  out << "    Iterations: " << getIterationCount() << std::endl;
  out << "    Consecutive Failures: " << consecutiveFailures_ << std::endl;
}

bool ompl::geometric::SPARS2::checkGraphOptimality()
{
  OMPL_INFORM("checkGraphOptimality()");
  std::size_t indent = 0;

  std::size_t numTests = 1000;
  std::size_t numFailedPlans = 0;

  // Make sure motion validator is set to zero clearance
  // base::DiscreteMotionValidator *dmv =
  //     dynamic_cast<base::DiscreteMotionValidator *>(si_->getMotionValidator().get());
  // BOOST_ASSERT_MSG(dmv->getRequiredStateClearance() == 0, "Discrete motion validator should have clearance = 0");

  // For each test
  for (std::size_t i = 0; i < numTests; ++i)
  {
    if (visual_->viz1()->shutdownRequested())
      break;

    time::point startTime = time::now();  // Benchmark

    /* The whole neighborhood set which has been most recently computed */
    std::vector<std::vector<Vertex>> graphNeighborhood(2);
    /* The visible neighborhood set which has been most recently computed */
    std::vector<std::vector<Vertex>> visibleNeighborhood(2);

    // Choose random start and goal state that has a nearest neighbor
    std::vector<base::State *> endPoints(2);

    for (std::size_t i = 0; i < 2; ++i)
    {
      base::State *&state = endPoints[i];

      // Allocate
      state = si_->getStateSpace()->allocState();

      // Sample
      if (!sampler_->sample(state))
        throw Exception(name_, "No valid sample found");

      // Allow edges to be 5% longer than SparseDelta
      const double dist = sparseDelta_ * 1.05;

      // Find nearest neighbor
      findGraphNeighbors(state, dist, graphNeighborhood[i], visibleNeighborhood[i]);

      // Check if neighbor exists - should always have at least one neighbor otherwise graph lacks coverage
      if (visibleNeighborhood[i].empty())
      {
        for (std::size_t i = 0; i < 5; ++i)
          OMPL_ERROR("-----------------------------------------");
        OMPL_ERROR("No neighbors");
        for (std::size_t i = 0; i < 5; ++i)
          OMPL_ERROR("-----------------------------------------");
        throw Exception(name_, "no neighbors");
        return false;
      }

      // Check if first state is same as input
      if (si_->getStateSpace()->equalStates(state, stateProperty_[visibleNeighborhood[i][0]]))
        throw Exception(name_, "First neighbor is itself");
    }

    // Astar search through graph
    const base::State *actualStart = endPoints.front();
    const base::State *actualGoal = endPoints.back();
    const Vertex start = visibleNeighborhood[0][0];
    const Vertex goal = visibleNeighborhood[1][0];

    base::PathPtr vertexPath;
    bool solutionFound = true;

    // Do not search astar if start goal are the same
    if (si_->getStateSpace()->equalStates(stateProperty_[start], stateProperty_[goal]))
    {
      // OMPL_INFORM("start and goal are the same");
      auto p(std::make_shared<PathGeometric>(si_));
      p->append(stateProperty_[start]);
      vertexPath = p;
    }
    else  // search astar
    {
      try
      {
        vertexPath = constructSolution(start, goal);
      }
      catch (...)
      {
        solutionFound = false;
      }
    }

    if (!solutionFound || !vertexPath)
    {
      OMPL_ERROR("No path found through graph");

      // Clear out previous path for clarity
      visual_->viz2()->deleteAllMarkers();
      visual_->viz2()->trigger();
      usleep(0.001 * 1000000);

      // Visualize actual and snapped states
      visual_->viz3()->deleteAllMarkers();
      visual_->viz3()->state(actualStart, tools::LARGE, tools::RED, 0);
      visual_->viz3()->state(actualGoal, tools::LARGE, tools::GREEN, 0);
      visual_->viz3()->state(stateProperty_[start], tools::LARGE, tools::BLACK, 0);
      visual_->viz3()->state(stateProperty_[goal], tools::LARGE, tools::BLACK, 0);
      visual_->viz3()->trigger();
      visual_->viz1()->spin();
      exit(0);
      continue;
    }
    double duration = time::seconds(time::now() - startTime);
    // OMPL_INFORM("PLANNING took %f seconds", duration); // Benchmark
    avgPlanTime_.push_back(duration);

    // If path found (same connected component) sum up total length
    geometric::PathGeometric geometricSolution(si_);
    // convertVertexPathToStatePath(vertexPath, actualStart, actualGoal, geometricSolution);
    {
      geometric::PathGeometric *vertexPathGeometric = static_cast<geometric::PathGeometric *>(vertexPath.get());
      geometricSolution.append(actualGoal);

      // Reverse the vertexPath and convert to state path
      for (std::size_t i = vertexPathGeometric->getStateCount(); i > 0; --i)
        geometricSolution.append(vertexPathGeometric->getState(i - 1));

      geometricSolution.append(actualStart);
    }

    // Smooth path to find the "optimal" path
    geometric::PathGeometric smoothedPath = geometricSolution;
    geometric::PathGeometric *smoothedPathPtr = &smoothedPath;
    sparseSmoother_->smoothMax(smoothedPathPtr, indent);

    // Show the two paths
    bool showEveryPath = false;
    if (showEveryPath)
    {
      // Visualize actual and snapped states
      visual_->viz3()->deleteAllMarkers();
      visual_->viz3()->state(actualStart, tools::LARGE, tools::RED, 0);
      visual_->viz3()->state(actualGoal, tools::LARGE, tools::GREEN, 0);
      visual_->viz3()->state(stateProperty_[start], tools::LARGE, tools::BLACK, 0);
      visual_->viz3()->state(stateProperty_[goal], tools::LARGE, tools::BLACK, 0);
      visual_->viz3()->trigger();

      visual_->viz2()->deleteAllMarkers();
      visual_->viz2()->path(&geometricSolution, tools::SMALL, tools::BLACK, tools::RED);
      visual_->viz2()->path(smoothedPathPtr, tools::MEDIUM, tools::BLACK, tools::GREEN);
      visual_->viz2()->trigger();
      usleep(0.001 * 1000000);
    }

    // calculate theoretical guarantees
    double optimalLength = smoothedPathPtr->length();
    double sparseLength = geometricSolution.length();
    double theoryLength = stretchFactor_ * optimalLength + 4 * sparseDelta_;
    double percentOfMaxAllows = sparseLength / theoryLength * 100.0;

    // Save path quality
    double pathQuality = optimalLength / sparseLength;
    // BOLT_DEBUG(indent, true, "pathQuality: " << pathQuality << " optimalLength: " << optimalLength << " sparseLength:
    // " << sparseLength);
    avgPathQuality_.push_back(pathQuality);

    // Output to console
    bool show = sparseLength >= theoryLength || false;
    BOLT_DEBUG(indent, show, "-----------------------------------------");
    BOLT_DEBUG(indent, show, "Checking Asymptotic Optimality Guarantees");
    BOLT_DEBUG(indent + 2, show, "Raw Path Length:         " << sparseLength);
    BOLT_DEBUG(indent + 2, show, "Smoothed Path Length:    " << optimalLength);
    BOLT_DEBUG(indent + 2, show, "Smoothed Path States:    " << smoothedPathPtr->getStateCount());
    BOLT_DEBUG(indent + 2, show, "Theoretical Path Length: " << theoryLength);
    BOLT_DEBUG(indent + 2, show, "Stretch Factor t:        " << stretchFactor_);
    BOLT_DEBUG(indent + 2, show, "Sparse Delta:            " << sparseDelta_);

    BOLT_ASSERT(sparseLength > std::numeric_limits<double>::epsilon(), "Path is zero length");

    if (sparseLength >= theoryLength)
    {
      BOLT_ERROR(indent + 2, "Asymptotic optimality guarantee VIOLATED");

      // Show the two paths
      visual_->viz2()->deleteAllMarkers();
      visual_->viz2()->path(&geometricSolution, tools::SMALL, tools::BLACK, tools::RED);
      visual_->viz2()->path(smoothedPathPtr, tools::MEDIUM, tools::BLACK, tools::GREEN);
      visual_->viz2()->trigger();
      usleep(0.001 * 1000000);

      // Show database with visibility regions (coverage)
      // sg_->visualizeDatabaseCoverage_ = true;
      // sg_->displayDatabase();

      visual_->viz1()->spin();
      exit(0);
      return false;
    }
    else
      BOLT_GREEN(indent + 2, show, "Asymptotic optimality guarantee maintained");
    BOLT_WARN(indent + 2, show, "Percent of max allowed:  " << percentOfMaxAllows << " %");
    BOLT_DEBUG(indent, show, "-----------------------------------------");

    // visual_->waitForUserFeedback("next problem");
  }

  // Summary
  BOLT_DEBUG(indent, 1, "-----------------------------------------");
  BOLT_DEBUG(indent, 1, "Checking Asymptotic Optimality Guarantees");
  BOLT_DEBUG(indent + 2, 1, "Total tests:               " << numTests);
  BOLT_DEBUG(indent + 2, 1, "Number failed plans:       " << numFailedPlans);
  BOLT_DEBUG(indent, 1, "-----------------------------------------");

  return true;
}

ompl::base::Cost ompl::geometric::SPARS2::costHeuristic(Vertex u, Vertex v) const
{
  return opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]);
}
