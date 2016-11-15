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
   Desc:   Various tests to determine if a vertex/edge should be added to the graph, based on SPARS
*/

// OMPL
#include <bolt_core/TaskCriteria.h>
#include <bolt_core/SparseFormula.h>

// Boost
#include <boost/foreach.hpp>

// Profiling
#include <valgrind/callgrind.h>

// C++
#include <thread>

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
TaskCriteria::TaskCriteria(TaskGraphPtr taskGraph)
  : taskGraph_(taskGraph), compoundSI_(taskGraph_->getCompoundSpaceInformation()), visual_(taskGraph_->getVisual())
{
  // We really only need one, but this is setup to be threaded for future usage
  for (std::size_t i = 0; i < taskGraph_->getNumQueryVertices(); ++i)
  {
    closeRepSampledState_.push_back(compoundSI_->allocState());
  }
}

TaskCriteria::~TaskCriteria(void)
{
  for (std::size_t i = 0; i < taskGraph_->getNumQueryVertices(); ++i)
  {
    compoundSI_->freeState(closeRepSampledState_[i]);
  }
}

bool TaskCriteria::setup(std::size_t indent)
{
  SparseFormula formulas;
  formulas.calc(compoundSI_, stretchFactor_, sparseDeltaFraction_, penetrationOverlapFraction_,
                nearSamplePointsMultiple_, useL2Norm_, indent);
  // Copy values back
  maxExtent_ = formulas.maxExtent_;
  sparseDelta_ = formulas.sparseDelta_;
  denseDelta_ = formulas.denseDelta_;
  discretizePenetrationDist_ = formulas.discretizePenetrationDist_;
  nearSamplePoints_ = formulas.nearSamplePoints_;
  discretization_ = formulas.discretization_;
  stretchFactor_ = formulas.stretchFactor_;

  // Check
  assert(maxExtent_ > 0);
  assert(denseDelta_ > 0);
  assert(nearSamplePoints_ > 0);
  assert(sparseDelta_ > 0);
  assert(sparseDelta_ > 0.000000001);  // Sanity check

  return true;
}

void TaskCriteria::clear()
{
  resetStats();
}

void TaskCriteria::resetStats()
{
  numVerticesMoved_ = 0;
  useFourthCriteria_ = false;
  // TODO: move addVertex stats in TaskGraph here
}

bool TaskCriteria::addStateToRoadmap(TaskCandidateData &candidateD, VertexType &addReason, std::size_t threadID,
                                     std::size_t indent)
{
  BOLT_FUNC(vCriteria_, "addStateToRoadmap() Adding candidate state ID " << candidateD.state_);

  if (visualizeAttemptedStates_)
  {
    // visual_->viz2()->deleteAllMarkers();
    // visual_->viz2()->state(candidateD.state_, tools::LARGE, tools::LIME_GREEN, 0);
    visual_->viz6()->state(candidateD.state_, tools::ROBOT, tools::DEFAULT, 0);
    // visual_->viz2()->trigger();
    // usleep(0.001 * 1000000);
  }

  bool stateAdded = false;

  // Always add a node if no other nodes around it are visible (GUARD)
  if (checkAddCoverage(candidateD, indent))
  {
    BOLT_DEBUG(vAddedReason_, "Graph updated: COVERAGE Fourth: " << useFourthCriteria_
                                                                 << " State: " << candidateD.state_);

    addReason = COVERAGE;
    stateAdded = true;
  }
  else if (checkAddConnectivity(candidateD, indent + 2))
  {
    BOLT_MAGENTA(vAddedReason_, "Graph updated: CONNECTIVITY Fourth: " << useFourthCriteria_
                                                                       << " State: " << candidateD.state_);

    addReason = CONNECTIVITY;
    stateAdded = true;
  }
  else if (checkAddInterface(candidateD, indent + 4))
  {
    BOLT_BLUE(vAddedReason_, "Graph updated: INTERFACE Fourth: " << useFourthCriteria_
                                                                 << " State: " << candidateD.state_);
    addReason = INTERFACE;
    stateAdded = true;
  }
  else
  {
    BOLT_DEBUG(vCriteria_, "Did NOT add state for any criteria "
                               << " State: " << candidateD.state_);
  }

  return stateAdded;
}

bool TaskCriteria::checkAddCoverage(TaskCandidateData &candidateD, std::size_t indent)
{
  BOLT_FUNC(vCriteria_, "checkAddCoverage() Are other nodes around it visible?");

  // Only add a node for coverage if it has no neighbors
  if (candidateD.visibleNeighborhood_.size() > 0)
  {
    BOLT_DEBUG(vCriteria_, "NOT adding node for coverage ");
    return false;  // has visible neighbors
  }

  // No free paths means we add for coverage
  BOLT_DEBUG(vCriteria_, "Adding node for COVERAGE ");

  candidateD.newVertex_ = taskGraph_->addVertex(candidateD.state_, indent + 4);

  // Note: we do not connect this node with any edges because we have already determined
  // it is too far away from any nearby nodes

  return true;
}

bool TaskCriteria::checkAddConnectivity(TaskCandidateData &candidateD, std::size_t indent)
{
  BOLT_FUNC(vCriteria_, "checkAddConnectivity() Does this node connect two disconnected components?");

  if (!useConnectivityCriteria_)
  {
    BOLT_DEBUG(vCriteria_, "NOT adding node for connectivity - disabled ");
    return false;
  }
  // Not implemented
  return false;
  /*
  if (!useFourthCriteria_)
  {
    BOLT_DEBUG(vCriteria_, "NOT adding node for connectivity - waiting until fourth criteria ");
    return false;
  }

  // If less than 2 neighbors there is no way to find a pair of nodes in
  // different connected components
  if (candidateD.visibleNeighborhood_.size() < 2)
  {
    BOLT_DEBUG(vCriteria_, "NOT adding node for connectivity");
    return false;
  }

  // Identify visibile nodes around our new state that are unconnected (in
  // different connected components) and connect them
  std::set<TaskVertex> statesInDiffConnectedComponents;

  // For each neighbor
  for (const TaskVertex &v1 : candidateD.visibleNeighborhood_)
  {
    // For each other neighbor
    for (const TaskVertex &v2 : candidateD.visibleNeighborhood_)
    {
      // If they are in different components
      if (!taskGraph_->sameComponent(v1, v2))
      {
        BOLT_DEBUG(vCriteria_, "Different connected component: " << v1 << ", " << v2);

        if (visualizeConnectivity_)  // Debug
        {
          visual_->viz2()->state(taskGraph_->getModelBasedState(v1), tools::MEDIUM, tools::BLUE, 0);
          visual_->viz2()->state(taskGraph_->getModelBasedState(v2), tools::MEDIUM, tools::BLUE, 0);
          visual_->viz2()->trigger();
          usleep(0.001 * 1000000);
        }

        BOLT_ASSERT(!taskGraph_->hasEdge(v1, v2), "Edge exist but not in same component");

        // Can they be connected directly?
        if (useDirectConnectivyCriteria_)
        {
          if (compoundSI_->checkMotion(taskGraph_->getCompoundState(v1), taskGraph_->getCompoundState(v2)))
          {
            taskGraph_->addEdge(v1, v2, eCONNECTIVITY, indent);

            // We return true (state was used to improve graph) but we didn't actually use
            // the state, so we much manually free the memory
            compoundSI_->freeState(candidateD.state_);

            return true;
          }
        }

        // Add to potential list
        statesInDiffConnectedComponents.insert(v1);
        statesInDiffConnectedComponents.insert(v2);
      }
    }
  }

  // Were any disconnected states found?
  if (statesInDiffConnectedComponents.empty())
  {
    BOLT_DEBUG(vCriteria_, "NOT adding node for connectivity");
    return false;
  }

  BOLT_DEBUG(vCriteria_, "Adding node for CONNECTIVITY ");

  // Add the node
  candidateD.newVertex_ = taskGraph_->addVertex(candidateD.state_, indent + 2);

  // Remove all edges from all vertices near our new vertex
  taskGraph_->clearEdgesNearVertex(candidateD.newVertex_, indent);

  // Check if there are really close vertices nearby which should be merged
  // This feature doesn't really do anything but slow things down
  // checkRemoveCloseVertices(candidateD.newVertex_, indent);

  // Add the edges
  for (std::set<TaskVertex>::const_iterator vertexIt = statesInDiffConnectedComponents.begin();
       vertexIt != statesInDiffConnectedComponents.end(); ++vertexIt)
  {
    BOLT_DEBUG(vCriteria_, "Loop: Adding vertex " << *vertexIt);

    if (taskGraph_->stateDeleted(*vertexIt))
    {
      BOLT_DEBUG(vCriteria_, "Skipping because vertex " << *vertexIt << " was removed (state marked as 0)");
      continue;
    }

    // Do not add edge from self to self
    if (compoundSI_->getStateSpace()->equalStates(taskGraph_->getState(*vertexIt),
  taskGraph_->getState(candidateD.newVertex_)))
    {
      BOLT_ERROR("Prevented same vertex from being added twice ");
      continue;  // skip this pairing
    }

    // New vertex should not be connected to anything - there's no edge between the two states
    if (taskGraph_->hasEdge(candidateD.newVertex_, *vertexIt))
    {
      BOLT_DEBUG(vCriteria_, "The new vertex " << candidateD.newVertex_ << " is already connected to old "
                                                                                       "vertex");
      continue;
    }

    // The components haven't been united by previous edges created in this for
    // loop
    if (!taskGraph_->sameComponent(*vertexIt, candidateD.newVertex_))
    {
      // Connect
      taskGraph_->addEdge(candidateD.newVertex_, *vertexIt, eCONNECTIVITY, indent + 4);
    }
  }
  */

  return true;
}

bool TaskCriteria::checkAddInterface(TaskCandidateData &candidateD, std::size_t indent)
{
  BOLT_FUNC(vCriteria_, "checkAddInterface() Does this node's neighbor's need it to better connect them?");

  // If there are less than two neighbors the interface property is not applicable, because requires
  // two closest visible neighbots
  if (candidateD.visibleNeighborhood_.size() < 2)
  {
    BOLT_DEBUG(vCriteria_, "NOT adding node (less than 2 visible neighbors)");
    return false;
  }

  const TaskVertex &v1 = candidateD.visibleNeighborhood_[0];
  const TaskVertex &v2 = candidateD.visibleNeighborhood_[1];

  // Ensure the two closest nodes are also visible
  bool skipThis = false;  // when true, is a new experimental feature
  if (!skipThis && !(candidateD.graphNeighborhood_[0] == v1 && candidateD.graphNeighborhood_[1] == v2))
  {
    BOLT_DEBUG(vCriteria_, "NOT adding because two closest nodes are not visible to each other");

    // TEMP
    if (vCriteria_)
    {
      visual_->viz1()->edge(taskGraph_->getModelBasedState(candidateD.graphNeighborhood_[0]), candidateD.state_,
                            tools::SMALL, tools::BLUE);
      visual_->viz1()->edge(taskGraph_->getModelBasedState(candidateD.graphNeighborhood_[1]), candidateD.state_,
                            tools::SMALL, tools::BLUE);
      visual_->viz1()->trigger();
      usleep(0.001 * 1000000);
    }

    return false;
  }

  // Ensure two closest neighbors don't share an edge
  if (taskGraph_->hasEdge(v1, v2))
  {
    BOLT_DEBUG(vCriteria_, "Two closest two neighbors already share an edge, not connecting them");
    return false;
  }

  // Don't add an interface edge if dist between the two verticies on graph are already the minimum in L1 space
  if (!taskGraph_->checkPathLength(v1, v2, indent))
    return false;

  // If they can be directly connected
  if (compoundSI_->checkMotion(taskGraph_->getCompoundState(v1), taskGraph_->getCompoundState(v2)))
  {
    BOLT_DEBUG(vCriteria_, "INTERFACE: directly connected nodes");

    // Connect them
    taskGraph_->addEdge(v1, v2, indent);

    // We return true (state was used to improve graph) but we didn't actually use
    // the state, so we much manually free the memory
    compoundSI_->freeState(candidateD.state_);

    return true;
  }

  // They cannot be directly connected
  // Add the new node to the graph, to bridge the interface
  BOLT_DEBUG(vCriteria_, "Adding node for INTERFACE");

  candidateD.newVertex_ = taskGraph_->addVertex(candidateD.state_, indent);

  // Remove all edges from all vertices near our new vertex
  // taskGraph_->clearEdgesNearVertex(candidateD.newVertex_, indent);

  // Check if there are really close vertices nearby which should be merged
  // This feature doesn't really do anything but slow things down
  // if (checkRemoveCloseVertices(candidateD.newVertex_, indent))
  //   return true;// New vertex replaced a nearby vertex, we can continue no further because graph has been re-indexed

  taskGraph_->addEdge(candidateD.newVertex_, v1, indent);
  taskGraph_->addEdge(candidateD.newVertex_, v2, indent);

  BOLT_DEBUG(vCriteria_, "INTERFACE: connected two neighbors through new interface node");

  // Report success
  return true;
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
