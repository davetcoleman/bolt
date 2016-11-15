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
#include <bolt_core/SparseCriteria.h>
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
SparseCriteria::SparseCriteria(SparseGraphPtr sg) : sg_(sg), si_(sg_->getSpaceInformation()), visual_(sg_->getVisual())
{
  // We really only need one, but this is setup to be threaded for future usage
  for (std::size_t i = 0; i < sg_->getNumQueryVertices(); ++i)
  {
    closeRepSampledState_.push_back(si_->allocState());
  }
}

SparseCriteria::~SparseCriteria(void)
{
  sampler_.reset();

  for (std::size_t i = 0; i < sg_->getNumQueryVertices(); ++i)
  {
    si_->freeState(closeRepSampledState_[i]);
  }
}

bool SparseCriteria::setup(std::size_t indent)
{
  SparseFormula formulas;
  static bool verboseOnce = true;
  formulas.calc(si_, stretchFactor_, sparseDeltaFraction_, penetrationOverlapFraction_, nearSamplePointsMultiple_,
                useL2Norm_, verboseOnce, indent);
  if (verboseOnce)
    verboseOnce = false;  // only show this once
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

  // Get a sampler with or without clearance sampling
  sampler_ = sg_->getSampler(si_, sg_->getObstacleClearance(), indent);

  if (si_->getStateValidityChecker()->getClearanceSearchDistance() < sg_->getObstacleClearance())
    OMPL_WARN("State validity checker clearance search distance %f is less than the required obstacle clearance %f for "
              "our state sampler, incompatible settings!",
              si_->getStateValidityChecker()->getClearanceSearchDistance(), sg_->getObstacleClearance());

  return true;
}

void SparseCriteria::clear()
{
  resetStats();
}

void SparseCriteria::resetStats()
{
  numVerticesMoved_ = 0;
  useFourthCriteria_ = false;
  // TODO: move addVertex stats in SparseGraph here
}

bool SparseCriteria::addStateToRoadmap(SparseCandidateData &candidateD, VertexType &addReason, std::size_t threadID,
                                       std::size_t indent)
{
  BOLT_FUNC(vCriteria_, "addStateToRoadmap() Adding candidate state with " << candidateD.visibleNeighborhood_.size()
                                                                           << " visible neighbors");

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
  else if (checkAddConnectivity(candidateD, indent))
  {
    BOLT_MAGENTA(vAddedReason_, "Graph updated: CONNECTIVITY Fourth: " << useFourthCriteria_
                                                                       << " State: " << candidateD.state_);

    addReason = CONNECTIVITY;
    stateAdded = true;
  }
  else if (checkAddInterface(candidateD, indent))
  {
    BOLT_BLUE(vAddedReason_, "Graph updated: INTERFACE Fourth: " << useFourthCriteria_
                                                                 << " State: " << candidateD.state_);
    addReason = INTERFACE;
    stateAdded = true;
  }
#ifdef ENABLE_QUALITY
  else if (checkAddQuality(candidateD, threadID, indent))
  {
    BOLT_GREEN(vAddedReason_, "Graph updated: QUALITY Fourth: " << useFourthCriteria_
                                                                << " State: " << candidateD.state_);
    addReason = QUALITY;
    stateAdded = true;

    // We return true (state was used to improve graph) but we didn't actually use
    // the state, so we must manually free the memory
    si_->freeState(candidateD.state_);
  }
#endif
  else
  {
    BOLT_WARN(vCriteria_, "Did NOT add state for any criteria");
  }

  return stateAdded;
}

bool SparseCriteria::checkAddCoverage(SparseCandidateData &candidateD, std::size_t indent)
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

  candidateD.newVertex_ = sg_->addVertex(candidateD.state_, COVERAGE, indent);

  // Note: we do not connect this node with any edges because we have already determined
  // it is too far away from any nearby nodes

  return true;
}

bool SparseCriteria::checkAddConnectivity(SparseCandidateData &candidateD, std::size_t indent)
{
  BOLT_FUNC(vCriteria_, "checkAddConnectivity() Does this node connect two disconnected components?");

  if (!useConnectivityCriteria_)
  {
    BOLT_DEBUG(vCriteria_, "NOT adding node for connectivity - disabled ");
    return false;
  }
  // if (!useFourthCriteria_)
  // {
  //   BOLT_DEBUG(vCriteria_, "NOT adding node for connectivity - waiting until fourth criteria ");
  //   return false;
  // }

  // If less than 2 neighbors there is no way to find a pair of nodes in
  // different connected components
  if (candidateD.visibleNeighborhood_.size() < 2)
  {
    BOLT_DEBUG(vCriteria_, "NOT adding node for connectivity");
    return false;
  }

  // Identify visibile nodes around our new state that are unconnected (in
  // different connected components) and connect them
  std::set<SparseVertex> statesInDiffConnectedComponents;

  // For each neighbor
  for (const SparseVertex &v1 : candidateD.visibleNeighborhood_)
  {
    // For each other neighbor
    for (const SparseVertex &v2 : candidateD.visibleNeighborhood_)
    {
      // If they are in different components
      if (!sg_->sameComponent(v1, v2, indent))
      {
        BOLT_DEBUG(vCriteria_, "Different connected component: " << v1 << ", " << v2);

        if (visualizeConnectivity_)  // Debug
        {
          visual_->viz2()->state(sg_->getState(v1), tools::MEDIUM, tools::BLUE, 0);
          visual_->viz2()->state(sg_->getState(v2), tools::MEDIUM, tools::BLUE, 0);
          visual_->viz2()->trigger();
          usleep(0.001 * 1000000);
        }

        BOLT_ASSERT(!sg_->hasEdge(v1, v2), "Edge exist but not in same component");

        // Can they be connected directly?
        if (useDirectConnectivyCriteria_)
        {
          // Distance check to make sure still within SparseDelta
          if (si_->distance(sg_->getState(v1), sg_->getState(v2)) < sparseDelta_)
          {
            if (si_->checkMotion(sg_->getState(v1), sg_->getState(v2)))
            {
              sg_->addEdge(v1, v2, eCONNECTIVITY, indent);
              BOLT_MAGENTA(true, "Added just an edge for CONNECTIVITY");
              // We return true (state was used to improve graph) but we didn't actually use
              // the state, so we must manually free the memory
              si_->freeState(candidateD.state_);

              return true;
            }
          }  // if distance less than sparseDelta
        }    // if criteria enabled

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
  candidateD.newVertex_ = sg_->addVertex(candidateD.state_, CONNECTIVITY, indent);
  BOLT_MAGENTA(true, "Adding node and some edges for CONNECTIVITY ");

  // Remove all edges from all vertices near our new vertex
  // TODO: re-enable
  // sg_->clearEdgesNearVertex(candidateD.newVertex_, indent);

  // Check if there are really close vertices nearby which should be merged
  // This feature doesn't really do anything but slow things down
  // checkRemoveCloseVertices(candidateD.newVertex_, indent);

  // Add the edges
  for (std::set<SparseVertex>::const_iterator vertexIt = statesInDiffConnectedComponents.begin();
       vertexIt != statesInDiffConnectedComponents.end(); ++vertexIt)
  {
    BOLT_DEBUG(vCriteria_, "Loop: Adding vertex " << *vertexIt);

    if (sg_->stateDeleted(*vertexIt))
    {
      BOLT_DEBUG(vCriteria_, "Skipping because vertex " << *vertexIt << " was removed (state marked as 0)");
      continue;
    }

    // Do not add edge from self to self
    if (si_->getStateSpace()->equalStates(sg_->getState(*vertexIt), sg_->getState(candidateD.newVertex_)))
    {
      BOLT_ERROR("Prevented same vertex from being added twice ");
      continue;  // skip this pairing
    }

    // New vertex should not be connected to anything - there's no edge between the two states
    // This check is probably completely redundant
    if (sg_->hasEdge(candidateD.newVertex_, *vertexIt))
    {
      BOLT_DEBUG(vCriteria_, "The new vertex " << candidateD.newVertex_ << " is already connected to old vertex");
      continue;
    }

    // The components haven't been united by previous edges created in this for loop
    if (!sg_->sameComponent(*vertexIt, candidateD.newVertex_, indent))
    {
      // Connect
      sg_->addEdge(candidateD.newVertex_, *vertexIt, eCONNECTIVITY, indent);
    }
  }

  return true;
}

bool SparseCriteria::checkAddInterface(SparseCandidateData &candidateD, std::size_t indent)
{
  BOLT_FUNC(vCriteria_, "checkAddInterface() Does this node's neighbor's need it to better connect them?");

  // If there are less than two neighbors the interface property is not applicable, because requires
  // two closest visible neighbots
  if (candidateD.visibleNeighborhood_.size() < 2)
  {
    BOLT_DEBUG(vCriteria_, "NOT adding node (less than 2 visible neighbors)");
    return false;
  }

  const SparseVertex &v1 = candidateD.visibleNeighborhood_[0];
  const SparseVertex &v2 = candidateD.visibleNeighborhood_[1];

  // Ensure the two closest nodes are also visible
  bool skipThis = false;  // when true, is a new experimental feature
  if (!skipThis && !(candidateD.graphNeighborhood_[0] == v1 && candidateD.graphNeighborhood_[1] == v2))
  {
    BOLT_DEBUG(vCriteria_, "NOT adding because two closest nodes are not visible to each other");
    return false;
  }

  // Ensure two closest neighbors don't share an edge
  if (sg_->hasEdge(v1, v2))
  {
    BOLT_DEBUG(vCriteria_, "Two closest two neighbors already share an edge, not connecting them");
    return false;
  }

  // Don't add an interface edge if dist between the two verticies on graph are already the minimum in L1 space
  if (!sg_->checkPathLength(v1, v2, indent))
  {
    numInterfacesSkippedByLength_++;
    std::cout << "numInterfacesSkippedByLength: " << numInterfacesSkippedByLength_ << std::endl;
    // return false;
  }

  // If they can be directly connected
  if (si_->checkMotion(sg_->getState(v1), sg_->getState(v2)))
  {
    BOLT_DEBUG(vCriteria_, "INTERFACE: directly connected nodes");

    // Connect them
    sg_->addEdge(v1, v2, eINTERFACE, indent);

    // We return true (state was used to improve graph) but we didn't actually use
    // the state, so we much manually free the memory
    si_->freeState(candidateD.state_);

    return true;
  }

  // They cannot be directly connected
  // Add the new node to the graph, to bridge the interface
  BOLT_DEBUG(vCriteria_, "Adding node for INTERFACE");

  candidateD.newVertex_ = sg_->addVertex(candidateD.state_, INTERFACE, indent);

  // Remove all edges from all vertices near our new vertex
  // Removed because hurts speed of learning in Bolt2
  // sg_->clearEdgesNearVertex(candidateD.newVertex_, indent);

  // Check if there are really close vertices nearby which should be merged
  // This feature doesn't really do anything but slow things down
  // if (checkRemoveCloseVertices(candidateD.newVertex_, indent))
  //   return true;// New vertex replaced a nearby vertex, we can continue no further because graph has been re-indexed

  if (sg_->stateDeleted(v1))
  {
    BOLT_ERROR("Skipping edge 0 because vertex was removed");
  }
  else
  {
    sg_->addEdge(candidateD.newVertex_, v1, eINTERFACE, indent);
  }

  if (sg_->stateDeleted(v2))
  {
    BOLT_ERROR("Skipping edge 1 because vertex was removed");
  }
  else
  {
    sg_->addEdge(candidateD.newVertex_, v2, eINTERFACE, indent);
  }

  BOLT_DEBUG(vCriteria_, "INTERFACE: connected two neighbors through new interface node");

  // Report success
  return true;
}

#ifdef ENABLE_QUALITY
bool SparseCriteria::checkAddQuality(SparseCandidateData &candidateD, std::size_t threadID, std::size_t indent)
{
  if (!useFourthCriteria_)
    return false;

  BOLT_FUNC(vQuality_, "checkAddQuality() Ensure SPARS asymptotic optimality");

  if (candidateD.visibleNeighborhood_.empty())
  {
    BOLT_DEBUG(vQuality_, "no visible neighbors, not adding 4th criteria ");
    return false;
  }

  // base::State *candidateState  // paper's name: q
  SparseVertex candidateRep = candidateD.visibleNeighborhood_[0];  // paper's name: v

  bool added = false;
  std::map<SparseVertex, base::State *> closeRepresentatives;  // [nearSampledRep, nearSampledState]

  findCloseRepresentatives(candidateD.state_, candidateRep, closeRepresentatives, threadID, indent);

  BOLT_DEBUG(vQuality_, "back in checkAddQuality(): Found " << closeRepresentatives.size() << " close representatives");

  bool updated = false;  // track whether a change was made to any vertices' representatives

  // For each pair of close representatives
  for (std::map<SparseVertex, base::State *>::iterator it = closeRepresentatives.begin();
       it != closeRepresentatives.end(); ++it)
  {
    BOLT_DEBUG(vQuality_, "Looping through close representatives");
    base::State *nearSampledState = it->second;  // paper: q'
    SparseVertex nearSampledRep = it->first;     // paper: v'

    if (visualizeQualityCriteriaCloseReps_)  // Visualization
    {
      visual_->viz3()->edge(sg_->getState(nearSampledRep), nearSampledState, tools::MEDIUM, tools::RED);

      visual_->viz3()->state(nearSampledState, tools::MEDIUM, tools::LIME_GREEN, 0);

      // Replicate a regular vertex visualization
      // visual_->viz3()->state(sg_->getState(nearSampledRep),
      // tools::VARIABLE_SIZE, tools::TRANSLUCENT_LIGHT,
      // sparseDelta_);
      visual_->viz3()->state(sg_->getState(nearSampledRep), tools::LARGE, tools::PURPLE, sparseDelta_);

      visual_->viz3()->trigger();
      usleep(0.001 * 1000000);
    }

    // Update interface bookkeeping
    // Process:
    // 1. Get adjacent vertieces of the candidateRep (v) that are unconnected to
    // nearSampledRep (v')
    //      e.g. v''
    // 2. For every adj vertex that is unconnected to v'
    // 3. Check distance:
    //    3.1. Get the interface data stored on vertex candidateRep(v) for max
    //    distance between
    //           nearSampledRep (v') and adjVertexUnconnected (v'')
    //    3.2. Add the candidateState (q) and nearSampledState (q') as 'first'

    // Attempt to update bookkeeping for candidateRep (v)
    if (updatePairPoints(candidateRep, candidateD.state_, nearSampledRep, nearSampledState, indent))
      updated = true;

    // ALSO attempt to update bookkeeping for neighboring node nearSampleRep
    // (v')
    if (updatePairPoints(nearSampledRep, nearSampledState, candidateRep, candidateD.state_, indent))
      updated = true;
  }

  BOLT_DEBUG(vQuality_, "Done updating pair points");

  if (!updated)
  {
    BOLT_DEBUG(vQuality_, "No representatives were updated, so not calling checkAddPath()");

    // Free memory before returning
    for (std::map<SparseVertex, base::State *>::iterator it = closeRepresentatives.begin();
         it != closeRepresentatives.end(); ++it)
    {
      // Delete state that was allocated and sampled within this function
      si_->freeState(it->second);
    }

    return false;
  }

  // Attempt to find shortest path through closest neighbour
  if (checkAddPath(candidateRep, indent))
  {
    BOLT_DEBUG(vQuality_, "nearest visible neighbor added for path ");
    added = true;
  }

  // Attempt to find shortest path through other pairs of representatives
  for (std::map<SparseVertex, base::State *>::iterator it = closeRepresentatives.begin();
       it != closeRepresentatives.end(); ++it)
  {
    BOLT_WARN(vQuality_, "Looping through close representatives to add path ===============");

    base::State *nearSampledState = it->second;  // paper: q'
    SparseVertex nearSampledRep = it->first;     // paper: v'
    if (checkAddPath(nearSampledRep, indent))
    {
      BOLT_DEBUG(vQuality_, "Close representative added for path");
      added = true;
    }

    // Delete state that was allocated and sampled within this function
    si_->freeState(nearSampledState);
  }

  return added;
}

void SparseCriteria::visualizeCheckAddQuality(base::State *candidateState, SparseVertex candidateRep)
{
  visual_->viz3()->deleteAllMarkers();

  visual_->viz3()->edge(candidateState, sg_->getState(candidateRep), tools::MEDIUM, tools::ORANGE);

  // Show candidate state
  // visual_->viz3()->state(candidateState, tools::VARIABLE_SIZE,
  // tools::TRANSLUCENT_LIGHT,
  // denseDelta_);
  visual_->viz3()->state(candidateState, tools::LARGE, tools::RED, 0);

  // Show candidate state's representative
  visual_->viz3()->state(sg_->getState(candidateRep), tools::LARGE, tools::BLUE, 0);

  // Show candidate state's representative's neighbors
  foreach (SparseVertex adjVertex, boost::adjacent_vertices(candidateRep, sg_->getGraph()))
  {
    visual_->viz3()->edge(sg_->getState(adjVertex), sg_->getState(candidateRep), tools::MEDIUM, tools::LIME_GREEN);
    visual_->viz3()->state(sg_->getState(adjVertex), tools::LARGE, tools::PURPLE, 0);
  }

  visual_->viz3()->trigger();
  usleep(0.001 * 1000000);
}

bool SparseCriteria::checkAddPath(SparseVertex v, std::size_t indent)
{
  BOLT_FUNC(vQuality_, "checkAddPath() v = " << v);
  bool spannerPropertyWasViolated = false;

  // Candidate v" vertices as described in the method, filled by function
  // getAdjVerticesOfV1UnconnectedToV2().
  std::vector<SparseVertex> adjVerticesUnconnected;

  // Copy adjacent vertices into vector because we might add additional edges
  // during this function
  std::vector<SparseVertex> adjVertices;
  foreach (SparseVertex adjVertex, boost::adjacent_vertices(v, sg_->getGraph()))
    adjVertices.push_back(adjVertex);

  BOLT_DEBUG(vQuality_, "Vertex v = " << v << " has " << adjVertices.size() << " adjacent vertices, "
                                                                               "looping:");

  // Loop through adjacent vertices
  for (std::size_t i = 0; i < adjVertices.size() && !spannerPropertyWasViolated; ++i)
  {
    SparseVertex vp = adjVertices[i];  // vp = v' from paper

    BOLT_DEBUG(vQuality_, "Checking v' = " << vp << " ----------------------------");

    // Compute all nodes which qualify as a candidate v" for v and vp
    getAdjVerticesOfV1UnconnectedToV2(v, vp, adjVerticesUnconnected, indent);

    // for each vertex v'' that is adjacent to v (has a valid edge) and does not
    // share an edge with v'
    foreach (SparseVertex vpp, adjVerticesUnconnected)  // vpp = v'' from paper
    {
      BOLT_DEBUG(vQuality_, "Checking v'' = " << vpp);

      // Check if by chance vpp was removed during addQualityPath() due to other optimization
      if (sg_->stateDeleted(vpp))
      {
        BOLT_INFO(vQuality_, "State vpp=" << vpp << " was deleted, skipping quality checks");
        return spannerPropertyWasViolated;
      }

      InterfaceData &iData = sg_->getInterfaceData(v, vp, vpp, indent);

      double shortestPathVpVpp;  // remember what the shortest path is from astar

      // Check if we need to actually add path
      if (spannerTest(v, vp, vpp, iData, shortestPathVpVpp, indent))
      {
        // Actually add the vertices and possibly edges
        if (addQualityPath(v, vp, vpp, iData, shortestPathVpVpp, indent))
        {
          spannerPropertyWasViolated = true;

          // Check if by chance v was removed during process
          if (sg_->stateDeleted(v))
          {
            BOLT_INFO(vQuality_, "State v=" << v << " was deleted, skipping quality checks");
            return spannerPropertyWasViolated;
          }

          // Check if by chance vp was removed during process
          if (sg_->stateDeleted(vp))
          {
            BOLT_INFO(vQuality_, "State vp=" << vp << " was deleted, skipping quality checks");
            return spannerPropertyWasViolated;
          }
        }
      }

    }  // foreach vpp
  }    // foreach vp

  if (!spannerPropertyWasViolated)
  {
    BOLT_DEBUG(vQuality_, "Spanner property was NOT violated, SKIPPING");
  }

  return spannerPropertyWasViolated;
}

void SparseCriteria::visualizeCheckAddPath(SparseVertex v, SparseVertex vp, SparseVertex vpp, InterfaceData &iData,
                                           std::size_t indent)
{
  BOLT_FUNC(vQuality_, "visualizeCheckAddPath()");
  visual_->viz5()->deleteAllMarkers();

  // Show all the vertices
  const bool showVertices = true;
  const bool showEdges = false;
  const std::size_t windowID = 5;
  sg_->displayDatabase(showVertices, showEdges, windowID, indent);

  // Show candidate rep
  visual_->viz5()->state(sg_->getState(v), tools::LARGE, tools::BLUE, 0);

  // Show adjacent state
  visual_->viz5()->state(sg_->getState(vp), tools::LARGE, tools::PURPLE, 0);
  // visual_->viz5()->state(sg_->getState(vp), tools::VARIABLE_SIZE,
  // tools::TRANSLUCENT_LIGHT, sparseDelta_);

  // Show edge between them
  visual_->viz5()->edge(sg_->getState(vp), sg_->getState(v), tools::MEDIUM, tools::LIME_GREEN);

  // Show adjacent state
  visual_->viz5()->state(sg_->getState(vpp), tools::LARGE, tools::PURPLE, 0);
  // visual_->viz5()->state(sg_->getState(vpp), tools::VARIABLE_SIZE,
  // tools::TRANSLUCENT_LIGHT, sparseDelta_);

  // Show edge between them
  visual_->viz5()->edge(sg_->getState(vpp), sg_->getState(v), tools::MEDIUM, tools::ORANGE);

  // Show iData
  if (iData.hasInterface1())
  {
    visual_->viz5()->state(iData.getInterface1Inside(), tools::MEDIUM, tools::ORANGE, 0);
    visual_->viz5()->state(iData.getInterface1Outside(), tools::MEDIUM, tools::LIME_GREEN, 0);
    visual_->viz5()->edge(iData.getInterface1Inside(), iData.getInterface1Outside(), tools::MEDIUM, tools::RED);

    if (vp < vpp)
      visual_->viz5()->edge(sg_->getState(vp), iData.getInterface1Outside(), tools::MEDIUM, tools::RED);
    else
      visual_->viz5()->edge(sg_->getState(vpp), iData.getInterface1Outside(), tools::MEDIUM, tools::RED);
  }
  if (iData.hasInterface2())
  {
    visual_->viz5()->state(iData.getInterface2Inside(), tools::MEDIUM, tools::ORANGE, 0);
    visual_->viz5()->state(iData.getInterface2Outside(), tools::MEDIUM, tools::LIME_GREEN, 0);
    visual_->viz5()->edge(iData.getInterface2Inside(), iData.getInterface2Outside(), tools::MEDIUM, tools::RED);

    if (vp < vpp)
      visual_->viz5()->edge(sg_->getState(vpp), iData.getInterface2Outside(), tools::MEDIUM, tools::RED);
    else
      visual_->viz5()->edge(sg_->getState(vp), iData.getInterface2Outside(), tools::MEDIUM, tools::RED);
  }

  // Show nearby vertices 'x' that could also be used to find the path to v''
  // Note: this section of code is copied from maxSpannerPath
  std::size_t color = 9;  // orange
  foreach (SparseVertex x, boost::adjacent_vertices(vpp, sg_->getGraph()))
  {
    if (sg_->hasEdge(x, v) && !sg_->hasEdge(x, vp))
    {
      InterfaceData &iData = sg_->getInterfaceData(v, vpp, x, indent);

      // Check if we previously had found a pair of points that support this
      // interface
      if ((vpp < x && iData.getInterface1Inside()) || (x < vpp && iData.getInterface2Inside()))
      {
        BOLT_INFO(vQuality_, "Visualizing (orange, purple, red, pink, white) additional qualified vertex " << x);
        visual_->viz5()->state(sg_->getState(x), tools::LARGE, static_cast<tools::VizColors>(color++), 0);
      }
    }
  }

  visual_->viz5()->trigger();
  usleep(0.001 * 1000000);
}

bool SparseCriteria::addQualityPath(SparseVertex v, SparseVertex vp, SparseVertex vpp, InterfaceData &iData,
                                    const double shortestPathVpVpp, std::size_t indent)
{
  BOLT_FUNC(vQuality_, "addQualityPath()");

  if (visualizeQualityCriteria_)
    visualizeCheckAddPath(v, vp, vpp, iData, indent);

  // Can we connect these two vertices directly?
  if (si_->checkMotion(sg_->getState(vp), sg_->getState(vpp)))
  {
    BOLT_DEBUG(vQuality_, "Adding edge between vp and vpp");

    BOLT_ASSERT(!sg_->hasEdge(vp, vpp), "Edge already exists, cannot add quality");
    sg_->addEdge(vp, vpp, eQUALITY, indent);
    // visual_->prompt("addQualityPath");

    return true;
  }

  BOLT_WARN(vQuality_, "Unable to connect directly - geometric path must be created for spanner");

  geometric::PathGeometric *path = new geometric::PathGeometric(si_);

  // Populate path - memory is copied from original location
  if (vp < vpp)
  {
    path->append(sg_->getState(vp));
    path->append(iData.getInterface1Outside());
    path->append(iData.getInterface1Inside());
    path->append(sg_->getState(v));
    path->append(iData.getInterface2Inside());
    path->append(iData.getInterface2Outside());
    path->append(sg_->getState(vpp));
  }
  else
  {
    path->append(sg_->getState(vp));
    path->append(iData.getInterface2Outside());
    path->append(iData.getInterface2Inside());
    path->append(sg_->getState(v));
    path->append(iData.getInterface1Inside());
    path->append(iData.getInterface1Outside());
    path->append(sg_->getState(vpp));
  }

  // Create path and simplify
  const bool debug = false;
  if (!sg_->getSparseSmoother()->smoothQualityPath(path, sg_->getObstacleClearance(), debug, indent))
  {
    delete path;
    return false;
  }

  // Determine if this smoothed path actually helps improve connectivity
  // "Improving the Smoothed Quality Path Criteria"
  if (useSmoothedPathImprovementRule_)
  {
    if (path->length() > shortestPathVpVpp)
    {
      BOLT_WARN(vQuality_, "Smoothed path does not improve connectivity");
      delete path;
      return false;
    }
  }

  // Insert simplified path into graph
  SparseVertex prior = vp;
  SparseVertex newVertex;

  BOLT_DEBUG(vQuality_, "Shortcuted path now has " << path->getStateCount() << " states");

  // first and last states are vp and vpp so don't get added
  for (std::size_t i = 1; i < path->getStates().size() - 1; ++i)
  {
    base::State *state = path->getStates()[i];

    // We don't have to copy the state memory because we did already above and use smart unloading
    BOLT_DEBUG(vQuality_, "Adding node from shortcut path for QUALITY");
    newVertex = sg_->addVertex(si_->getStateSpace()->cloneState(state), QUALITY, indent);

    // Remove all edges from all vertices near our new vertex
    sg_->clearEdgesNearVertex(newVertex, indent);

    assert(prior != newVertex);
    sg_->addEdge(prior, newVertex, eQUALITY, indent);
    prior = newVertex;

  }  // for each state in path

  // Add last edge back onto graph
  assert(prior != vpp);
  sg_->addEdge(prior, vpp, eQUALITY, indent);

  delete path;
  return true;
}

bool SparseCriteria::spannerTest(SparseVertex v, SparseVertex vp, SparseVertex vpp, InterfaceData &iData,
                                 double &shortestPathVpVpp, std::size_t indent)
{
  BOLT_FUNC(vQuality_, "spannerTest()");
  // Computes all nodes which qualify as a candidate x for v, v', and v" and get the length of the longest one
  double midpointPathLength = maxSpannerPath(v, vp, vpp, indent);

  if (stretchFactor_ * iData.getLastDistance() < midpointPathLength)
  {
    BOLT_WARN(vQuality_, "Spanner property violated");
    BOLT_DEBUG(vQuality_, "Sparse Graph Midpoint Length  = " << midpointPathLength);
    BOLT_DEBUG(vQuality_, "Spanner Path Length * Stretch = " << (stretchFactor_ * iData.getLastDistance()));
    BOLT_DEBUG(vQuality_, "last distance = " << iData.getLastDistance());
    BOLT_DEBUG(vQuality_, "stretch factor = " << stretchFactor_);
    double rejectStretchFactor = midpointPathLength / iData.getLastDistance();
    BOLT_DEBUG(vQuality_, "to reject, stretch factor > " << rejectStretchFactor);

    // Modification of Quality Criteria for $L_1$ Space
    if (useEdgeImprovementRule_)
    {
      // Get the length of the proposed edge
      double newEdgeDistance = si_->distance(sg_->getState(vp), sg_->getState(vpp));

      // First do a quick simple test for a shorter path through the supporting vertices
      const double SMALL_EPSILON = 0.0001;
      if (newEdgeDistance / 2.0 >= midpointPathLength - SMALL_EPSILON)
      {
        return false;  // skip because new edge wouldn't help anything
      }

      // Second test: Compare to the length of the shortest path through the graph with those endpoints
      shortestPathVpVpp = qualityEdgeAstarTest(vp, vpp, iData, indent);
      BOLT_DEBUG(vQuality_, "newEdgeDistance: " << newEdgeDistance);
      BOLT_DEBUG(vQuality_, "shortestPathVpVpp: " << shortestPathVpVpp);
      // BOLT_DEBUG(vQuality_ || 1, "shortestPathVpVpp+: " << shortestPathVpVpp - SMALL_EPSILON);
      // visual_->viz6()->state(sg_->getState(vp), tools::MEDIUM, tools::BLACK, 0);
      // visual_->viz6()->state(sg_->getState(vpp), tools::MEDIUM, tools::BLACK, 0);
      // visual_->viz6()->trigger();

      if (visualizeQualityCriteria_)                       // TEMP
        visualizeCheckAddPath(v, vp, vpp, iData, indent);  // TEMP

      if (newEdgeDistance > shortestPathVpVpp - SMALL_EPSILON)
      {
        return false;  // skip because there is already a path that achieves this
      }
    }

    return true;  // spanner property was violated
  }

  BOLT_DEBUG(vQuality_, "Spanner property not violated");
  return false;  // spanner property was NOT violated
}

double SparseCriteria::qualityEdgeAstarTest(SparseVertex vp, SparseVertex vpp, InterfaceData &iData, std::size_t indent)
{
  BOLT_FUNC(vQuality_, "qualityEdgeAstarTest()");

  // Experimental calculations
  double pathLength = 0;

  if (visualizeQualityCriteriaAstar_)
  {
    // Search and get path
    std::vector<SparseVertex> vertexPath;
    if (!sg_->astarSearch(vp, vpp, vertexPath, pathLength, indent))
    {
      BOLT_ERROR(vQuality_, "No path found");
      visual_->prompt("No path found");
      return std::numeric_limits<double>::infinity();
    }

    // Visualize
    visual_->viz6()->deleteAllMarkers();
    assert(vertexPath.size() > 1);
    for (std::size_t i = 1; i < vertexPath.size(); ++i)
    {
      visual_->viz6()->edge(sg_->getState(vertexPath[i - 1]), sg_->getState(vertexPath[i]), tools::MEDIUM,
                            tools::LIME_GREEN);
    }
    visual_->viz6()->trigger();
  }
  else  // faster search
  {
    if (!sg_->astarSearchLength(vp, vpp, pathLength, indent))
    {
      BOLT_ERROR(vQuality_, "No path found");
      visual_->prompt("No path found");
      return std::numeric_limits<double>::infinity();
    }
  }

  return pathLength;
}

SparseVertex SparseCriteria::findGraphRepresentative(base::State *state, std::size_t threadID, std::size_t indent)
{
  BOLT_FUNC(vQuality_, "findGraphRepresentative()");

  std::vector<SparseVertex> graphNeighbors;

  // Search
  sg_->getQueryStateNonConst(threadID) = state;
  sg_->getNN()->nearestR(sg_->getQueryVertices(threadID), sparseDelta_, graphNeighbors);
  sg_->getQueryStateNonConst(threadID) = nullptr;

  BOLT_DEBUG(vQuality_, "Found " << graphNeighbors.size() << " nearest neighbors (graph rep) within "
                                                             "SparseDelta " << sparseDelta_);

  SparseVertex result = boost::graph_traits<SparseAdjList>::null_vertex();

  for (std::size_t i = 0; i < graphNeighbors.size(); ++i)
  {
    BOLT_DEBUG(vQuality_, "Checking motion of graph representative candidate " << i);
    if (si_->checkMotion(state, sg_->getState(graphNeighbors[i])))
    {
      BOLT_DEBUG(vQuality_, "graph representative valid ");
      result = graphNeighbors[i];
      break;
    }
    else
      BOLT_DEBUG(vQuality_, "graph representative NOT valid, checking next ");
  }
  return result;
}

void SparseCriteria::findCloseRepresentatives(const base::State *candidateState, const SparseVertex candidateRep,
                                              std::map<SparseVertex, base::State *> &closeRepresentatives,
                                              std::size_t threadID, std::size_t indent)
{
  BOLT_FUNC(vQuality_, "findCloseRepresentatives()");
  BOLT_DEBUG(vQuality_, "nearSamplePoints: " << nearSamplePoints_ << " denseDelta: " << denseDelta_);

  base::State *sampledState = closeRepSampledState_[threadID];  // alias the var name

  assert(closeRepresentatives.empty());

  // Search the space around new potential state candidateState
  for (std::size_t i = 0; i < nearSamplePoints_; ++i)
  {
    BOLT_DEBUG(vQuality_, "Get supporting representative #" << i);

    bool foundValidSample = false;
    static const std::size_t MAX_SAMPLE_ATTEMPT = 1000;
    for (std::size_t attempt = 0; attempt < MAX_SAMPLE_ATTEMPT; ++attempt)
    {
      sampler_->sampleNear(sampledState, candidateState, denseDelta_);

      if (!si_->isValid(sampledState))
      {
        BOLT_DEBUG(vQuality_ && false, "Sample attempt " << attempt << " notValid ");

        if (visualizeQualityCriteriaSampler_)
          visual_->viz3()->state(sampledState, tools::SMALL, tools::RED, 0);

        continue;
      }
      if (si_->distance(candidateState, sampledState) > denseDelta_)
      {
        BOLT_DEBUG(vQuality_ && false, "Sample attempt " << attempt << " Distance too far "
                                                         << si_->distance(candidateState, sampledState)
                                                         << " needs to be less than " << denseDelta_);

        if (visualizeQualityCriteriaSampler_)
          visual_->viz3()->state(sampledState, tools::SMALL, tools::RED, 0);
        continue;
      }
      if (!si_->checkMotion(candidateState, sampledState))
      {
        BOLT_DEBUG(vQuality_ && false, "Sample attempt " << attempt << " motion invalid ");

        if (visualizeQualityCriteriaSampler_)
          visual_->viz3()->state(sampledState, tools::SMALL, tools::RED, 0);
        continue;
      }

      if (visualizeQualityCriteriaSampler_)
        visual_->viz3()->state(sampledState, tools::SMALL, tools::LIME_GREEN, 0);

      BOLT_DEBUG(vQuality_ && false, "Sample attempt " << attempt << " valid ");

      foundValidSample = true;
      break;
    }  // for each attempt

    if (visualizeQualityCriteriaSampler_)
    {
      visual_->viz3()->trigger();
      usleep(0.001 * 1000000);
    }

    if (!foundValidSample)
    {
      BOLT_DEBUG(vQuality_, "Unable to find valid sample after " << MAX_SAMPLE_ATTEMPT << " attempts"
                                                                                          " ");
    }
    else
      BOLT_DEBUG(vQuality_, "Found valid nearby sample");

    // Compute which sparse vertex represents this new candidate vertex
    SparseVertex sampledStateRep = findGraphRepresentative(sampledState, threadID, indent);

    // Check if sample is not visible to any other node (it should be visible
    // in all likelihood)
    if (sampledStateRep == boost::graph_traits<SparseAdjList>::null_vertex())
    {
      BOLT_DEBUG(vQuality_, "Sampled state has no representative (is null) ");

      // It can't be seen by anybody, so we should take this opportunity to add it
      // Check for proper clearance if in superdebug mode
      if (sg_->superDebugEnabled() && !checkSufficientClearance(sampledState))
      {
        BOLT_WARN(true, "Skipping adding node for COVERAGE because improper clearance");
      }
      else
      {
        BOLT_DEBUG(vQuality_ || true, "Adding node for COVERAGE - findCloseRepresentatives");
        sg_->addVertex(si_->cloneState(sampledState), COVERAGE, indent);
      }

      BOLT_DEBUG(vQuality_, "STOP EFFORTS TO ADD A DENSE PATH");

      // We should also stop our efforts to add a dense path
      for (std::map<SparseVertex, base::State *>::iterator it = closeRepresentatives.begin();
           it != closeRepresentatives.end(); ++it)
        si_->freeState(it->second);
      closeRepresentatives.clear();
      break;
    }

    BOLT_DEBUG(vQuality_, "Sampled state has representative (is not null)");

    // If its representative is different than candidateState
    if (sampledStateRep != candidateRep)
    {
      BOLT_DEBUG(vQuality_, "candidateRep != sampledStateRep ");

      // And we haven't already tracked this representative
      if (closeRepresentatives.find(sampledStateRep) == closeRepresentatives.end())
      {
        BOLT_DEBUG(vQuality_, "Track the representative");

        // Track the representative
        closeRepresentatives[sampledStateRep] = si_->cloneState(sampledState);
      }
      else
      {
        BOLT_DEBUG(vQuality_, "Already tracking the representative");
      }
    }
    else
    {
      BOLT_DEBUG(vQuality_, "candidateRep == sampledStateRep, do not keep this sample ");
    }
  }  // for each supporting representative
}

bool SparseCriteria::updatePairPoints(SparseVertex candidateRep, const base::State *candidateState,
                                      SparseVertex nearSampledRep, const base::State *nearSampledState,
                                      std::size_t indent)
{
  BOLT_FUNC(vQuality_, "updatePairPoints()");
  bool updated = false;  // track whether a change was made to any vertices'
                         // representatives

  // First of all, we need to compute all candidate r'
  std::vector<SparseVertex> adjVerticesUnconnected;
  getAdjVerticesOfV1UnconnectedToV2(candidateRep, nearSampledRep, adjVerticesUnconnected, indent);

  // for each pair Pv(r,r')
  foreach (SparseVertex adjVertexUnconnected, adjVerticesUnconnected)
  {
    // Try updating the pair info
    if (distanceCheck(candidateRep, candidateState, nearSampledRep, nearSampledState, adjVertexUnconnected, indent))
      updated = true;
  }

  return updated;
}

void SparseCriteria::getAdjVerticesOfV1UnconnectedToV2(SparseVertex v1, SparseVertex v2,
                                                       std::vector<SparseVertex> &adjVerticesUnconnected,
                                                       std::size_t indent)
{
  BOLT_FUNC(vQuality_, "getAdjVerticesOfV1UnconnectedToV2()");

  adjVerticesUnconnected.clear();
  foreach (SparseVertex adjVertex, boost::adjacent_vertices(v1, sg_->getGraph()))
  {
    BOLT_ASSERT(sg_->getState(adjVertex) != NULL, "Adjacent vertex found that is null");

    if (adjVertex != v2)
      if (!sg_->hasEdge(adjVertex, v2))
        adjVerticesUnconnected.push_back(adjVertex);
  }

  BOLT_DEBUG(vQuality_, "adjVerticesUnconnected size: " << adjVerticesUnconnected.size());
}

double SparseCriteria::maxSpannerPath(SparseVertex v, SparseVertex vp, SparseVertex vpp, std::size_t indent)
{
  BOLT_FUNC(vQualityMaxSpanner_, "maxSpannerPath()");

  // Candidate x vertices as described in paper in Max_Spanner_Path
  // From my diagram nomenclature:
  // The algorithm compares the candidate edge length with all other spanner paths
  // from [midpoint(v', v) = p'] to the interfaces of all neighbors of v which
  // are not connected to v' but share an interface with v''
  std::vector<SparseVertex> qualifiedVertices;

  // Get nearby vertices 'x' that could also be used to find the path to v''
  // TODO: original SPARS2 implementation used vpp to search for adjacent vertices,
  // so I am also, but according to paper it should be v not vpp
  foreach (SparseVertex x, boost::adjacent_vertices(vpp, sg_->getGraph()))
  {
    BOLT_ASSERT(!sg_->stateDeleted(x), "State is deleted in maxSpannerPath");

    if (sg_->hasEdge(x, v) && !sg_->hasEdge(x, vp))
    {
      const InterfaceData &iData = sg_->getInterfaceData(v, vpp, x, indent);

      // Check if we previously had found a pair of points that support this interface
      if (iData.getInsideInterfaceOfV1(vpp, x) != nullptr)
      // if ((vpp < x && iData.getInterface1Inside()) || (x < vpp && iData.getInterface2Inside()))
      {
        BOLT_DEBUG(vQualityMaxSpanner_, "Found an additional qualified vertex " << x);

        // This is a possible alternative path to m(v,v')
        qualifiedVertices.push_back(x);
      }
    }
  }

  // vpp is always qualified because of its previous checks
  qualifiedVertices.push_back(vpp);
  BOLT_DEBUG(vQualityMaxSpanner_, "Total qualified vertices found: " << qualifiedVertices.size());

  // Find the maximum spanner distance
  BOLT_DEBUG(vQualityMaxSpanner_, "Finding the maximum spanner distance between v' and v''");
  double maxDist = 0.0;
  foreach (SparseVertex qualifiedVertex, qualifiedVertices)
  {
    if (visualizeQualityCriteria_ && false)
      visual_->viz5()->state(sg_->getState(qualifiedVertex), tools::SMALL, tools::PINK, 0);

    // Divide by 2 because of the midpoint path 'M'
    double tempDist = (si_->distance(sg_->getState(vp), sg_->getState(v)) +
                       si_->distance(sg_->getState(v), sg_->getState(qualifiedVertex))) /
                      2.0;
    BOLT_DEBUG(vQualityMaxSpanner_,
               "Checking vertex: " << qualifiedVertex << " distance: " << tempDist
                                   << " dist1: " << si_->distance(sg_->getState(vp), sg_->getState(v))
                                   << " dist2: " << si_->distance(sg_->getState(v), sg_->getState(qualifiedVertex)));

    // Compare with previous max
    if (tempDist > maxDist)
    {
      BOLT_DEBUG(vQualityMaxSpanner_, "Is larger than previous");
      maxDist = tempDist;
    }
  }
  BOLT_DEBUG(vQualityMaxSpanner_, "Max distance: " << maxDist);

  return maxDist;
}

bool SparseCriteria::distanceCheck(SparseVertex v, const base::State *q, SparseVertex vp, const base::State *qp,
                                   SparseVertex vpp, std::size_t indent)
{
  BOLT_FUNC(vQuality_, "distanceCheck()");

  bool updated = false;  // track whether a change was made to any vertices'
                         // representatives

  // Get the info for the current representative-neighbors pair
  InterfaceData &iData = sg_->getInterfaceData(v, vp, vpp, indent);

  if (vp < vpp)  // FIRST points represent r (the interface discovered through
                 // sampling)
  {
    if (!iData.hasInterface1())  // No previous interface has been found here,
                                 // just save it
    {
      BOLT_DEBUG(vQuality_, "setInterface1");
      iData.setInterface1(q, qp, si_);
      updated = true;
    }
    else if (!iData.hasInterface2())  // The other interface doesn't exist,
                                      // so we can't compare.
    {
      // Should probably keep the one that is further away from rep?  Not
      // known what to do in this case.
      // TODO: is this not part of the algorithm?
      BOLT_WARN(vQuality_, "TODO no interface 2");
    }
    else  // We know both of these points exist, so we can check some
          // distances
    {
      assert(iData.getLastDistance() < std::numeric_limits<double>::infinity());
      if (si_->distance(q, iData.getInterface2Inside()) < iData.getLastDistance())
      // si_->distance(iData.getInterface1Inside(),
      // iData.getInterface2Inside()))
      {  // Distance with the new point is good, so set it.
        BOLT_GREEN(vQuality_, "setInterface1 UPDATED");
        iData.setInterface1(q, qp, si_);
        updated = true;
      }
      else
      {
        BOLT_DEBUG(vQuality_, "Distance was not better, not updating bookkeeping");
      }
    }
  }
  else  // SECOND points represent r (the interfaec discovered through
        // sampling)
  {
    if (!iData.hasInterface2())  // No previous interface has been found here,
                                 // just save it
    {
      BOLT_DEBUG(vQuality_, "setInterface2");
      iData.setInterface2(q, qp, si_);
      updated = true;
    }
    else if (!iData.hasInterface1())  // The other interface doesn't exist,
                                      // so we can't compare.
    {
      // Should we be doing something cool here?
      BOLT_WARN(vQuality_, "TODO no interface 1");
    }
    else  // We know both of these points exist, so we can check some
          // distances
    {
      assert(iData.getLastDistance() < std::numeric_limits<double>::infinity());
      if (si_->distance(q, iData.getInterface1Inside()) < iData.getLastDistance())
      // si_->distance(iData.getInterface2Inside(),
      // iData.getInterface1Inside()))
      {  // Distance with the new point is good, so set it
        BOLT_GREEN(vQuality_, "setInterface2 UPDATED");
        iData.setInterface2(q, qp, si_);
        updated = true;
      }
      else
      {
        BOLT_DEBUG(vQuality_, "Distance was not better, not updating bookkeeping");
      }
    }
  }

  // Lastly, save what we have discovered
  if (updated)
  {
    // TODO(davetcoleman): do we really need to copy this back in or is it
    // already passed by reference?
    sg_->getInterfaceData(v, vp, vpp, indent) = iData;
    // sg_->vertexInterfaceProperty_[v][sg_->interfaceDataIndex(vp, vpp)] =
    // iData;
  }

  return updated;
}

void SparseCriteria::visualizeInterfaces(SparseVertex v, std::size_t indent)
{
  BOLT_FUNC(vQuality_, "visualizeInterfaces()");

  InterfaceHash &iData = sg_->getVertexInterfaceProperty(v);

  visual_->viz6()->deleteAllMarkers();
  visual_->viz6()->state(sg_->getState(v), tools::LARGE, tools::RED, 0);

  for (auto it = iData.begin(); it != iData.end(); ++it)
  {
    const VertexPair &pair = it->first;
    InterfaceData &iData = it->second;

    SparseVertex v1 = pair.first;
    SparseVertex v2 = pair.second;

    visual_->viz6()->state(sg_->getState(v1), tools::LARGE, tools::PURPLE, 0);
    visual_->viz6()->state(sg_->getState(v2), tools::LARGE, tools::PURPLE, 0);
    // visual_->viz6()->edge(sg_->getState(v1), sg_->getState(v2),
    // tools::eGREEN);

    if (iData.hasInterface1())
    {
      visual_->viz6()->state(iData.getInterface1Inside(), tools::MEDIUM, tools::ORANGE, 0);
      visual_->viz6()->state(iData.getInterface1Outside(), tools::MEDIUM, tools::LIME_GREEN, 0);
      visual_->viz6()->edge(iData.getInterface1Inside(), iData.getInterface1Outside(), tools::MEDIUM, tools::YELLOW);
    }
    else
    {
      visual_->viz6()->edge(sg_->getState(v1), sg_->getState(v2), tools::MEDIUM, tools::RED);
    }

    if (iData.hasInterface2())
    {
      visual_->viz6()->state(iData.getInterface2Inside(), tools::MEDIUM, tools::ORANGE, 0);
      visual_->viz6()->state(iData.getInterface2Outside(), tools::MEDIUM, tools::LIME_GREEN, 0);
      visual_->viz6()->edge(iData.getInterface2Inside(), iData.getInterface2Outside(), tools::MEDIUM, tools::YELLOW);
    }
    else
    {
      visual_->viz6()->edge(sg_->getState(v1), sg_->getState(v2), tools::MEDIUM, tools::RED);
    }
  }

  visual_->viz6()->trigger();
  usleep(0.01 * 1000000);
}

void SparseCriteria::visualizeAllInterfaces(std::size_t indent)
{
  BOLT_FUNC(vQuality_, "visualizeAllInterfaces()");

  visual_->viz6()->deleteAllMarkers();

  foreach (SparseVertex v, boost::vertices(sg_->getGraph()))
  {
    // typedef std::unordered_map<VertexPair, InterfaceData> InterfaceHash;
    InterfaceHash &hash = sg_->getVertexInterfaceProperty(v);

    for (auto it = hash.begin(); it != hash.end(); ++it)
    {
      InterfaceData &iData = it->second;

      if (iData.hasInterface1())
      {
        visual_->viz6()->state(iData.getInterface1Inside(), tools::MEDIUM, tools::ORANGE, 0);
        visual_->viz6()->state(iData.getInterface1Outside(), tools::MEDIUM, tools::LIME_GREEN, 0);
      }

      if (iData.hasInterface2())
      {
        visual_->viz6()->state(iData.getInterface2Inside(), tools::MEDIUM, tools::ORANGE, 0);
        visual_->viz6()->state(iData.getInterface2Outside(), tools::MEDIUM, tools::LIME_GREEN, 0);
      }
    }
  }
  visual_->viz6()->trigger();
  usleep(0.01 * 1000000);
}

std::pair<std::size_t, std::size_t> SparseCriteria::getInterfaceStateStorageSize()
{
  std::size_t numStates = 0;
  std::size_t numMissingInterfaces = 0;

  foreach (SparseVertex v, boost::vertices(sg_->getGraph()))
  {
    InterfaceHash &hash = sg_->getVertexInterfaceProperty(v);

    for (auto it = hash.begin(); it != hash.end(); ++it)
    {
      InterfaceData &iData = it->second;

      if (iData.hasInterface1())
        numStates += 2;
      else
        numMissingInterfaces++;

      if (iData.hasInterface2())
        numStates += 2;
      else
        numMissingInterfaces++;
    }
  }
  return std::pair<std::size_t, std::size_t>(numStates, numMissingInterfaces);
}
#endif

bool SparseCriteria::checkSufficientClearance(base::State *state)
{
  // Check if new vertex has enough clearance
  // Note that it does not check for collision!
  double dist = si_->getStateValidityChecker()->clearance(state);
  return dist >= sg_->getObstacleClearance();
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
