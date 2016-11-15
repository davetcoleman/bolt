/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include <bolt_core/ERRTConnect.h>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

ompl::geometric::ERRTConnect::ERRTConnect(const base::SpaceInformationPtr &si, tools::VisualizerPtr visual)
  : base::Planner(si, "ERRTConnect"), visual_(visual)
{
  specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
  specs_.directed = true;

  maxDistance_ = 0.0;

  Planner::declareParam<double>("range", this, &ERRTConnect::setRange, &ERRTConnect::getRange, "0.:1.:10000.");
  connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
}

ompl::geometric::ERRTConnect::~ERRTConnect()
{
  freeMemory();
}

void ompl::geometric::ERRTConnect::setup()
{
  Planner::setup();
  tools::SelfConfig sc(si_, getName());
  sc.configurePlannerRange(maxDistance_);

  if (!tStart_)
    tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
  if (!tGoal_)
    tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
  tStart_->setDistanceFunction([this](const Motion *a, const Motion *b)
                               {
                                 return distanceFunction(a, b);
                               });
  tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b)
                              {
                                return distanceFunction(a, b);
                              });
}

void ompl::geometric::ERRTConnect::freeMemory()
{
  std::vector<Motion *> motions;

  if (tStart_)
  {
    tStart_->list(motions);
    for (auto &motion : motions)
    {
      if (motion->state)
        si_->freeState(motion->state);
      delete motion;
    }
  }

  if (tGoal_)
  {
    tGoal_->list(motions);
    for (auto &motion : motions)
    {
      if (motion->state)
        si_->freeState(motion->state);
      delete motion;
    }
  }
}

void ompl::geometric::ERRTConnect::clear()
{
  Planner::clear();
  sampler_.reset();
  freeMemory();
  if (tStart_)
    tStart_->clear();
  if (tGoal_)
    tGoal_->clear();
  connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
}

ompl::geometric::ERRTConnect::GrowState ompl::geometric::ERRTConnect::growTree(TreeData &tree,
                                                                               TreeGrowingInfo &treeGrowInfo,
                                                                               Motion *randMotion)
{
  std::size_t indent = 0;
  //BOLT_FUNC(true, "growTree()");

  /* find closest state in the tree */
  Motion *nearestMotion = tree->nearest(randMotion);

  /* assume we can reach the state we go towards */
  bool reach = true;

  /* find state to add */
  base::State *dstate = randMotion->state;
  double d = si_->distance(nearestMotion->state, randMotion->state);
  if (d > maxDistance_)
  {
    si_->getStateSpace()->interpolate(nearestMotion->state, // from
                                      randMotion->state, // to
                                      maxDistance_ / d, // time t
                                      treeGrowInfo.xstate); // output state
    dstate = treeGrowInfo.xstate;
    reach = false;
  }
  // if we are in the start tree, we just check the motion like we normally do;
  // if we are in the goal tree, we need to check the motion in reverse, but checkMotion() assumes the first state it
  // receives as argument is valid,
  // so we check that one first
  bool validMotion = treeGrowInfo.start ? si_->checkMotion(nearestMotion->state, dstate) :
                                          si_->getStateValidityChecker()->isValid(dstate) &&
                                              si_->checkMotion(dstate, nearestMotion->state);

  if (validMotion)
  {
    /* create a motion */
    auto *motion = new Motion(si_);
    si_->copyState(motion->state, dstate);
    motion->parent = nearestMotion;
    motion->root = nearestMotion->root;
    treeGrowInfo.xmotion = motion;

    tree->add(motion);
    if (reach)
      return REACHED;
    else
      return ADVANCED;
  }
  else
    return TRAPPED;
}

ompl::base::PlannerStatus ompl::geometric::ERRTConnect::solve(const base::PlannerTerminationCondition &ptc)
{
  checkValidity();
  base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

  if (!goal)
  {
    OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
    return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
  }

  while (const base::State *st = pis_.nextStart())
  {
    auto *motion = new Motion(si_);
    si_->copyState(motion->state, st);
    motion->root = motion->state;
    tStart_->add(motion);
  }

  if (tStart_->size() == 0)
  {
    OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  if (!goal->couldSample())
  {
    OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
    return base::PlannerStatus::INVALID_GOAL;
  }

  if (!sampler_)
    sampler_ = si_->allocStateSampler();

  OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
              (int)(tStart_->size() + tGoal_->size()));

  TreeGrowingInfo treeGrowInfo;
  treeGrowInfo.xstate = si_->allocState();

  auto *randMotion = new Motion(si_);
  base::State *randState = randMotion->state;
  bool startTree = true;
  bool solved = false;

  // Find neighbors to start and goal
  pis_.restart();  // restart the start and goal state
  base::State *goalState = si_->cloneState(pis_.nextGoal(ptc));
  base::State *startState = si_->cloneState(pis_.nextStart());
  pis_.restart();  // restart the start and goal state

  std::size_t indent = 0;
  loadSampler(startState, goalState, indent);
  si_->freeState(startState);
  si_->freeState(goalState);

  while (ptc == false)
  {
    TreeData &tree = startTree ? tStart_ : tGoal_;

    treeGrowInfo.start = startTree;
    startTree = !startTree;
    TreeData &otherTree = startTree ? tStart_ : tGoal_;

    if (pis_.haveMoreGoalStates())
    {
      const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
      if (st)
      {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tGoal_->add(motion);
      }

      if (tGoal_->size() == 0)
      {
        OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
        break;
      }
    }

    /* sample random state */
    // sampler_->sampleUniform(randState);

    /* sample state near either start or goal */
    sampleFromSparseGraph(randState, startTree, indent);

    GrowState gs = growTree(tree, treeGrowInfo, randMotion);

    if (ptc)
      break;

    if (gs != TRAPPED)
    {
      // Show status
      if (tree->size() % 100 == 0)
        std::cout << "ERRTConnect - Start Tree " << tStart_->size() << ", Goal Tree " << tGoal_->size() << std::endl;

      /* remember which motion was just added */
      Motion *addedMotion = treeGrowInfo.xmotion;

      /* attempt to connect trees */

      /* if reached, it means we used randState directly, no need top copy again */
      if (gs != REACHED)
        si_->copyState(randState, treeGrowInfo.xstate);

      GrowState gsc = ADVANCED;
      treeGrowInfo.start = startTree;

      while (gsc == ADVANCED)
        gsc = growTree(otherTree, treeGrowInfo, randMotion);

      Motion *startMotion = startTree ? treeGrowInfo.xmotion : addedMotion;
      Motion *goalMotion = startTree ? addedMotion : treeGrowInfo.xmotion;

      /* if we connected the trees in a valid way (start and goal pair is valid)*/
      if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
      {
        if (ptc)
          break;

        // it must be the case that either the start tree or the goal tree has made some progress
        // so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
        // on the solution path
        if (startMotion->parent)
          startMotion = startMotion->parent;
        else
          goalMotion = goalMotion->parent;

        connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

        /* construct the solution path */
        Motion *solution = startMotion;
        std::vector<Motion *> mpath1;
        while (solution != nullptr)
        {
          mpath1.push_back(solution);
          solution = solution->parent;
        }

        solution = goalMotion;
        std::vector<Motion *> mpath2;
        while (solution != nullptr)
        {
          mpath2.push_back(solution);
          solution = solution->parent;
        }

        auto path(std::make_shared<PathGeometric>(si_));
        path->getStates().reserve(mpath1.size() + mpath2.size());
        for (int i = mpath1.size() - 1; i >= 0; --i)
          path->append(mpath1[i]->state);
        for (auto &i : mpath2)
          path->append(i->state);

        pdef_->addSolutionPath(path, false, 0.0, getName());
        solved = true;
        break;
      }
    }
  }

  si_->freeState(treeGrowInfo.xstate);
  si_->freeState(randState);
  delete randMotion;

  OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
              tStart_->size(), tGoal_->size());
  OMPL_INFORM("Sampled %u states", totalSamples_);

  return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::ERRTConnect::getPlannerData(base::PlannerData &data) const
{
  Planner::getPlannerData(data);

  std::vector<Motion *> motions;
  if (tStart_)
    tStart_->list(motions);

  for (auto &motion : motions)
  {
    if (motion->parent == nullptr)
      data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
    else
    {
      data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
    }
  }

  motions.clear();
  if (tGoal_)
    tGoal_->list(motions);

  for (auto &motion : motions)
  {
    if (motion->parent == nullptr)
      data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
    else
    {
      // The edges in the goal tree are reversed to be consistent with start tree
      data.addEdge(base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2));
    }
  }

  // Add the edge connecting the two trees
  data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));
}

void ompl::geometric::ERRTConnect::loadSampler(base::State *start, base::State *goal, std::size_t indent)
{
  //BOLT_FUNC(true, "loadSampler()");

  getNeighbors(start, startGraphNeighborhood_, indent);
  getNeighbors(goal, goalGraphNeighborhood_, indent);

  // Reset
  startNeighborID_ = 0;
  goalNeighborID_ = 0;
  totalSamples_ = 0;

  //BOLT_INFO(true, "Done loading sampler");
}

void ompl::geometric::ERRTConnect::getNeighbors(base::State *state,
                                                std::vector<tools::bolt::SparseVertex> &graphNeighborhood,
                                                std::size_t indent)
{
  //BOLT_FUNC(true, "getNeighbors()");

  graphNeighborhood.clear();
  std::size_t threadID = 3;  // TODO choose better!

  // const std::size_t kNearest = 0;
  const std::size_t kNearest = std::min(sparseGraph_->getNumRealVertices(), (unsigned int)10000);

  // Search in thread-safe manner
  sparseGraph_->getQueryStateNonConst(threadID) = state;
  sparseGraph_->getNN()->nearestK(sparseGraph_->getQueryVertices(threadID), kNearest, graphNeighborhood);
  sparseGraph_->getQueryStateNonConst(threadID) = nullptr;

  //BOLT_INFO(true, "Found " << graphNeighborhood.size() << " neighbors");
}

void ompl::geometric::ERRTConnect::sampleFromSparseGraph(base::State *randState, bool isStart, std::size_t indent)
{
  bool verbose = true;
  //BOLT_FUNC(verbose, "sampleFromSparseGraph()");
  totalSamples_++;

  if (isStart)  // start
  {
    if (startNeighborID_ >= startGraphNeighborhood_.size() || totalSamples_ % 2 == 0)
    {
      //BOLT_FUNC(verbose, "Sampling start " << startNeighborID_ << " sampleUniform");
      sampler_->sampleUniform(randState);
    }
    else
    {
      //BOLT_FUNC(verbose, "Sampling start " << startNeighborID_ << " getStateNonConst");
      // Copy from SparseGraph to pre-allocated randState
      si_->copyState(randState, sparseGraph_->getStateNonConst(startGraphNeighborhood_[startNeighborID_++]));
    }
  }
  else  // goal
  {
    if (goalNeighborID_ >= goalGraphNeighborhood_.size() || totalSamples_ % 2 == 0)
    {
      //BOLT_FUNC(verbose, "Sampling goal " << goalNeighborID_ << " sampleUniform");
      sampler_->sampleUniform(randState);
    }
    else
    {
      //BOLT_FUNC(verbose, "Sampling goal " << goalNeighborID_ << " getStateNonConst, vertex " << goalGraphNeighborhood_[goalNeighborID_]);
      // Copy from SparseGraph to pre-allocated randState
      si_->copyState(randState, sparseGraph_->getStateNonConst(goalGraphNeighborhood_[goalNeighborID_++]));
    }
  }

  //BOLT_DEBUG(verbose, "Done sampling from SparseGraph");
}
