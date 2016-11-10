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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RRT_CONNECT_BOLT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_RRT_CONNECT_BOLT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <ompl/tools/debug/Visualizer.h>
#include <bolt_core/SparseGraph.h> // TODO remove this dep

namespace ompl
{
namespace geometric
{
/**
   @anchor gRRTC
   @par Short description
   The basic idea is to grow two RRTs, one from the start and
   one from the goal, and attempt to connect them.
   @par External documentation
   J. Kuffner and S.M. LaValle, RRT-connect: An efficient approach to single-query path planning, in <em>Proc.
   2000 IEEE Intl. Conf. on Robotics and Automation</em>, pp. 995–1001, Apr. 2000. DOI:
   [10.1109/ROBOT.2000.844730](http://dx.doi.org/10.1109/ROBOT.2000.844730)<br>
   [[PDF]](http://ieeexplore.ieee.org/ielx5/6794/18246/00844730.pdf?tp=&arnumber=844730&isnumber=18246)
   [[more]](http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html)
*/

/// @cond IGNORE
/** \brief Forward declaration of ompl::base::BoltPlanner */
OMPL_CLASS_FORWARD(ERRTConnect);
/// @endcond

/** \brief RRT-Connect (ERRTConnect) */
class ERRTConnect : public base::Planner
{
public:
  /** \brief Constructor */
  ERRTConnect(const base::SpaceInformationPtr &si, tools::VisualizerPtr visual);

  ~ERRTConnect() override;

  void getPlannerData(base::PlannerData &data) const override;

  base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

  void clear() override;

  /** \brief Set the range the planner is supposed to use.

      This parameter greatly influences the runtime of the
      algorithm. It represents the maximum length of a
      motion to be added in the tree of motions. */
  void setRange(double distance)
  {
    maxDistance_ = distance;
  }

  /** \brief Get the range the planner is using */
  double getRange() const
  {
    return maxDistance_;
  }

  /** \brief Set a different nearest neighbors datastructure */
  template <template <typename T> class NN>
  void setNearestNeighbors()
  {
    tStart_ = std::make_shared<NN<Motion *>>();
    tGoal_ = std::make_shared<NN<Motion *>>();
  }

  void setup() override;

  void setSparseGraph(tools::bolt::SparseGraphPtr sparseGraph)
  {
    sparseGraph_ = sparseGraph;
  }

  void loadSampler(base::State *start, base::State *goal, std::size_t indent);
  void getNeighbors(base::State *state, std::vector<tools::bolt::SparseVertex> &graphNeighborhood, std::size_t indent);
  void sampleFromSparseGraph(base::State *rstate, bool isStart, std::size_t indent);

protected:
  /** \brief Representation of a motion */
  class Motion
  {
  public:
    Motion() : root(nullptr), state(nullptr), parent(nullptr)
    {
      parent = nullptr;
      state = nullptr;
    }

    Motion(const base::SpaceInformationPtr &si) : root(nullptr), state(si->allocState()), parent(nullptr)
    {
    }

    ~Motion() = default;

    const base::State *root;
    base::State *state;
    Motion *parent;
  };

  /** \brief A nearest-neighbor datastructure representing a tree of motions */
  using TreeData = std::shared_ptr<NearestNeighbors<Motion *>>;

  /** \brief Information attached to growing a tree of motions (used internally) */
  struct TreeGrowingInfo
  {
    base::State *xstate;
    Motion *xmotion;
    bool start;
  };

  /** \brief The state of the tree after an attempt to extend it */
  enum GrowState
  {
    /// no progress has been made
    TRAPPED,
    /// progress has been made towards the randomly sampled state
    ADVANCED,
    /// the randomly sampled state was reached
    REACHED
  };

  /** \brief Free the memory allocated by this planner */
  void freeMemory();

  /** \brief Compute distance between motions (actually distance between contained states) */
  double distanceFunction(const Motion *a, const Motion *b) const
  {
    return si_->distance(a->state, b->state);
  }

  /** \brief Grow a tree towards a random state */
  GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);

  /** \brief State sampler */
  base::StateSamplerPtr sampler_;

  /** \brief Class for managing various visualization features */
  tools::VisualizerPtr visual_;

  /** \brief The start tree */
  TreeData tStart_;

  /** \brief The goal tree */
  TreeData tGoal_;

  /** \brief The maximum length of a motion to be added to a tree */
  double maxDistance_;

  /** \brief The random number generator */
  RNG rng_;

  /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
  std::pair<base::State *, base::State *> connectionPoint_;

  tools::bolt::SparseGraphPtr sparseGraph_;
  std::vector<tools::bolt::SparseVertex> startGraphNeighborhood_;
  std::vector<tools::bolt::SparseVertex> goalGraphNeighborhood_;
  std::size_t startNeighborID_;
  std::size_t goalNeighborID_;
  std::size_t totalSamples_;

};
}
}

#endif
