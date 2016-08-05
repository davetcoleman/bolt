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
#include <ompl/tools/bolt/SparseCriteria.h>

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
  clearanceSampler_.reset();

  for (std::size_t i = 0; i < sg_->getNumQueryVertices(); ++i)
  {
    si_->freeState(closeRepSampledState_[i]);
  }
}

bool SparseCriteria::setup(std::size_t indent)
{
  // Dimensions / joints
  std::size_t dim = si_->getStateDimension();

  // Max distance across configuration space
  maxExtent_ = si_->getMaximumExtent();

  // Vertex visibility region size
  sparseDelta_ = sparseDeltaFraction_ * maxExtent_;

  // Sampling for interfaces visibility size
  // denseDelta_ = denseDeltaFraction_;
  denseDelta_ = sparseDeltaFraction_ * 0.1 * maxExtent_;

  // How much overlap should the discretization factor provide for ensuring edge connection
  discretizePenetrationDist_ = penetrationOverlapFraction_ * sparseDelta_;

  // Number of points to test for interfaces around a sample for the quality criterion
  nearSamplePoints_ = nearSamplePointsMultiple_ * si_->getStateDimension();

  // Discretization for initial input into sparse graph
  if (useL2Norm_)  // this is for the 2D world
  {
    BOLT_WARN(indent, true, "Using L2 Norm for discretization");

    // const double discFactor = 2 * sparseDelta_;
    // discretization_ = sqrt(std::pow(discFactor, 2) / dim) -
    // discretizePenetrationDist_;
    discretization_ = sparseDelta_ - discretizePenetrationDist_;
  }
  else  // this is for joint space
  {
    BOLT_DEBUG(indent, true, "Using L1 Norm for discretization");
    // L1 Norm
    discretization_ = 2 * sparseDelta_ / dim - discretizePenetrationDist_;
  }

  // Calculate optimum stretch factor
  bool autoStretchFactor = stretchFactor_ < std::numeric_limits<double>::epsilon();
  if (autoStretchFactor)  // if stretchFactor is zero, auto set it
  {
    BOLT_DEBUG(indent, 1, "Auto settings stretch factor because input value was 0");

    // 2D case without estimated interface amount - old
    // nearestDiscretizedV_ = sqrt(dim * std::pow(0.5 * discretization_, 2));
    // // z in my calculations
    // stretchFactor_ = 2.0 * discretization_ / (nearestDiscretizedV_) +
    // stretchFactor_;

    // N-D case - old
    // stretchFactor_ = discretization_ / (discretization_ - 2.0 * denseDelta_);
    // // N-D case

    // 2D: New version July 30th
    // stretchFactor_ = 2.0*discretization_/(discretization_ - 2*denseDelta_);

    // 3D: New version July 30th
    // stretchFactor_ = 3 * discretization_ / (discretization_ - 2 *
    // denseDelta_);

    // ND: August 4th
    std::cout << "dim: " << dim << std::endl;
    stretchFactor_ = dim * discretization_ / (discretization_ - 2 * denseDelta_);
  }

  // Estimate size of graph
  ob::RealVectorBounds bounds = si_->getStateSpace()->getBounds();
  const std::size_t jointID = 0;
  const double range = bounds.high[jointID] - bounds.low[jointID];
  const std::size_t jointIncrements = floor(range / discretization_);
  const std::size_t maxStatesCount = pow(jointIncrements, dim);
  BOLT_INFO(indent, 1, "--------------------------------------------------");
  BOLT_INFO(indent, 1, "SparseCriteria Setup:");
  BOLT_INFO(indent + 2, 1, "Dimensions              = " << dim);
  BOLT_INFO(indent + 2, 1, "Max Extent              = " << maxExtent_);
  BOLT_INFO(indent + 2, 1, "Sparse Delta            = " << sparseDelta_);
  BOLT_INFO(indent + 2, 1, "Sparse Delta Fraction   = " << sparseDeltaFraction_);
  BOLT_INFO(indent + 2, 1, "Dense Delta             = " << denseDelta_);
  BOLT_INFO(indent + 2, 1, "State Dimension         = " << dim);
  BOLT_INFO(indent + 2, 1, "Discretization          = " << discretization_);
  BOLT_INFO(indent + 2, 1, "Joint Increments        = " << jointIncrements);
  BOLT_INFO(indent + 2, 1, "Max States Count        = " << maxStatesCount);
  BOLT_INFO(indent + 2, 1, "Near Sample Points      = " << nearSamplePoints_);
  BOLT_INFO(indent + 2, 1, "Pentrat. Overlap Frac   = " << penetrationOverlapFraction_);
  BOLT_INFO(indent + 2, 1, "Discret Penetration     = " << discretizePenetrationDist_);
  BOLT_INFO(indent + 2, autoStretchFactor, "Nearest Discretized V   = " << nearestDiscretizedV_);
  BOLT_INFO(indent + 2, 1, "Stretch Factor          = " << stretchFactor_);
  BOLT_INFO(indent, 1, "--------------------------------------------------");

  assert(maxExtent_ > 0);
  assert(denseDelta_ > 0);
  assert(nearSamplePoints_ > 0);
  assert(sparseDelta_ > 0);
  assert(sparseDelta_ > 0.000000001);  // Sanity check

  // Load minimum clearance state sampler
  // TODO: remove this if we stick to samplingQueue
  clearanceSampler_ = ob::MinimumClearanceValidStateSamplerPtr(new ob::MinimumClearanceValidStateSampler(si_.get()));
  clearanceSampler_->setMinimumObstacleClearance(sg_->getObstacleClearance());
  si_->getStateValidityChecker()->setClearanceSearchDistance(sg_->getObstacleClearance());

  if (si_->getStateValidityChecker()->getClearanceSearchDistance() < sg_->getObstacleClearance())
    OMPL_WARN("State validity checker clearance search distance %f is less than the required obstacle clearance %f for our state sampler, incompatible settings!",
              si_->getStateValidityChecker()->getClearanceSearchDistance(), sg_->getObstacleClearance());

  return true;
}

bool SparseCriteria::addStateToRoadmap(CandidateData &candidateD, VertexType &addReason, std::size_t threadID,
                                       std::size_t indent)
{
  BOLT_FUNC(indent, vCriteria_, "addStateToRoadmap() Adding candidate state ID " << candidateD.state_);

  if (visualizeAttemptedStates_)
  {
    visual_->viz2()->deleteAllMarkers();
    visual_->viz2()->state(candidateD.state_, tools::LARGE, tools::GREEN, 0);
    visual_->viz2()->state(candidateD.state_, tools::ROBOT, tools::DEFAULT, 0);
    visual_->viz2()->trigger();
    usleep(0.001 * 1000000);
  }

  bool stateAdded = false;

  // Always add a node if no other nodes around it are visible (GUARD)
  if (checkAddCoverage(candidateD, indent))
  {
    BOLT_DEBUG(indent, vAddedReason_, "Graph updated: COVERAGE Fourth: " << useFourthCriteria_);

    addReason = COVERAGE;
    stateAdded = true;
  }
  else if (checkAddConnectivity(candidateD, indent + 2))
  {
    BOLT_MAGENTA_DEBUG(indent, vAddedReason_, "Graph updated: CONNECTIVITY Fourth: " << useFourthCriteria_);

    addReason = CONNECTIVITY;
    stateAdded = true;
  }
  else if (checkAddInterface(candidateD, indent + 4))
  {
    BOLT_BLUE_DEBUG(indent, vAddedReason_, "Graph updated: INTERFACE Fourth: " << useFourthCriteria_);
    addReason = INTERFACE;
    stateAdded = true;
  }
  else if (checkAddQuality(candidateD, threadID, indent + 6))
  {
    BOLT_GREEN_DEBUG(indent, vAddedReason_, "Graph updated: QUALITY Fourth: " << useFourthCriteria_);
    addReason = QUALITY;
    stateAdded = true;
  }
  else
  {
    BOLT_DEBUG(indent, vCriteria_, "Did NOT add state for any criteria ");
  }

  return stateAdded;
}

bool SparseCriteria::checkAddCoverage(CandidateData &candidateD, std::size_t indent)
{
  BOLT_FUNC(indent, vCriteria_, "checkAddCoverage() Are other nodes around it visible?");

  // Only add a node for coverage if it has no neighbors
  if (candidateD.visibleNeighborhood_.size() > 0)
  {
    BOLT_DEBUG(indent, vCriteria_, "NOT adding node for coverage ");
    return false;  // has visible neighbors
  }

  // No free paths means we add for coverage
  BOLT_DEBUG(indent, vCriteria_, "Adding node for COVERAGE ");

  candidateD.newVertex_ = sg_->addVertex(candidateD.state_, COVERAGE, indent + 4);

  // Note: we do not connect this node with any edges because we have already determined
  // it is too far away from any nearby nodes

  return true;
}

bool SparseCriteria::checkAddConnectivity(CandidateData &candidateD, std::size_t indent)
{
  BOLT_FUNC(indent, vCriteria_, "checkAddConnectivity() Does this node connect "
                                "two disconnected components?");
  BOLT_DEBUG(indent, vCriteria_, "NOT adding node for connectivity - disabled ");
  return false;

  // If less than 2 neighbors there is no way to find a pair of nodes in
  // different connected components
  if (candidateD.visibleNeighborhood_.size() < 2)
  {
    BOLT_DEBUG(indent, vCriteria_, "NOT adding node for connectivity");
    return false;
  }

  // Identify visibile nodes around our new state that are unconnected (in
  // different connected components)
  // and connect them
  std::set<SparseVertex> statesInDiffConnectedComponents;

  // For each neighbor
  for (std::size_t i = 0; i < candidateD.visibleNeighborhood_.size(); ++i)
  {
    // For each other neighbor
    for (std::size_t j = i + 1; j < candidateD.visibleNeighborhood_.size(); ++j)
    {
      // If they are in different components
      if (!sg_->sameComponent(candidateD.visibleNeighborhood_[i], candidateD.visibleNeighborhood_[j]))
      {
        BOLT_DEBUG(indent, vCriteria_, "Different connected component: " << candidateD.visibleNeighborhood_[i] << ", "
                                                                         << candidateD.visibleNeighborhood_[j]);

        if (visualizeConnectivity_)
        {
          visual_->viz2()->state(sg_->getState(candidateD.visibleNeighborhood_[i]), tools::MEDIUM, tools::BLUE, 0);
          visual_->viz2()->state(sg_->getState(candidateD.visibleNeighborhood_[j]), tools::MEDIUM, tools::BLUE, 0);
          visual_->viz2()->trigger();
          usleep(0.001 * 1000000);
        }

        statesInDiffConnectedComponents.insert(candidateD.visibleNeighborhood_[i]);
        statesInDiffConnectedComponents.insert(candidateD.visibleNeighborhood_[j]);
      }
    }
  }

  // Were any disconnected states found?
  if (statesInDiffConnectedComponents.empty())
  {
    BOLT_DEBUG(indent, vCriteria_, "NOT adding node for connectivity");
    return false;
  }

  BOLT_DEBUG(indent, vCriteria_, "Adding node for CONNECTIVITY ");

  // Add the node
  candidateD.newVertex_ = sg_->addVertex(candidateD.state_, CONNECTIVITY, indent + 2);

  // Check if there are really close vertices nearby which should be merged
  checkRemoveCloseVertices(candidateD.newVertex_, indent);

  // Add the edges
  for (std::set<SparseVertex>::const_iterator vertexIt = statesInDiffConnectedComponents.begin();
       vertexIt != statesInDiffConnectedComponents.end(); ++vertexIt)
  {
    BOLT_DEBUG(indent + 4, vCriteria_, "Loop: Adding vertex " << *vertexIt);

    if (sg_->getState(*vertexIt) == NULL)
    {
      BOLT_DEBUG(indent + 4, vCriteria_, "Skipping because vertex " << *vertexIt << " was removed (state marked as 0)");
      continue;
    }

    // Do not add edge from self to self
    if (si_->getStateSpace()->equalStates(sg_->getState(*vertexIt), sg_->getState(candidateD.newVertex_)))
    {
      BOLT_ERROR(indent + 4, 1, "Prevented same vertex from being added twice ");
      continue;  // skip this pairing
    }

    // New vertex should not be connected to anything - there's no edge between
    // the two states
    if (sg_->hasEdge(candidateD.newVertex_, *vertexIt))
    {
      BOLT_DEBUG(indent + 4, vCriteria_, "The new vertex " << candidateD.newVertex_ << " is already connected to old "
                                                                                       "vertex");
      continue;
    }

    // The components haven't been united by previous edges created in this for
    // loop
    if (!sg_->sameComponent(*vertexIt, candidateD.newVertex_))
    {
      // Connect
      sg_->addEdge(candidateD.newVertex_, *vertexIt, eCONNECTIVITY, indent + 4);
    }
    else
    {
      // This is not a big deal
      // OMPL_WARN("Two states that where not prev in the same component were
      // joined during the same for "
      //"loop");
    }
  }

  return true;
}

bool SparseCriteria::checkAddInterface(CandidateData &candidateD, std::size_t indent)
{
  BOLT_FUNC(indent, vCriteria_, "checkAddInterface() Does this node's "
                                "neighbor's need it to better connect them?");

  // If there are less than two neighbors the interface property is not
  // applicable, because requires
  // two closest visible neighbots
  if (candidateD.visibleNeighborhood_.size() < 2)
  {
    BOLT_DEBUG(indent, vCriteria_, "NOT adding node for interface (less than 2 visible neighbors)");
    return false;
  }

  SparseVertex v1 = candidateD.visibleNeighborhood_[0];
  SparseVertex v2 = candidateD.visibleNeighborhood_[1];

  // If the two closest nodes are also visible
  if (candidateD.graphNeighborhood_[0] == v1 && candidateD.graphNeighborhood_[1] == v2)
  {
    // If our two closest neighbors don't share an edge
    if (!sg_->hasEdge(v1, v2))
    {
      // If they can be directly connected
      if (si_->checkMotion(sg_->getState(v1), sg_->getState(v2)))
      {
        BOLT_DEBUG(indent, vCriteria_, "INTERFACE: directly connected nodes");

        // TODO(davetcoleman): remove this debug code
        if (false && si_->getStateSpace()->equalStates(sg_->getState(v1), sg_->getState(v2)))
        {
          OMPL_ERROR("States are equal");
          visualizeRemoveCloseVertices(v1, v2);

          std::cout << "v1: " << v1 << " state1: " << sg_->getState(v1) << " state address: " << sg_->getState(v1)
                    << " state: ";
          sg_->debugState(sg_->getState(v1));
          std::cout << "v2: " << v2 << " state2: " << sg_->getState(v2) << " state address: " << sg_->getState(v2)
                    << " state: ";
          sg_->debugState(sg_->getState(v2));
        }

        // Connect them
        sg_->addEdge(v1, v2, eINTERFACE, indent);
      }
      else  // They cannot be directly connected
      {
        // Add the new node to the graph, to bridge the interface
        BOLT_DEBUG(indent, vCriteria_, "Adding node for INTERFACE");

        candidateD.newVertex_ = sg_->addVertex(candidateD.state_, INTERFACE, indent);

        // Check if there are really close vertices nearby which should be
        // merged
        if (checkRemoveCloseVertices(candidateD.newVertex_, indent))
        {
          // New vertex replaced a nearby vertex, we can continue no further
          // because graph has been re-indexed
          return true;
        }

        if (sg_->getState(v1) == NULL)
        {
          BOLT_ERROR(indent + 3, 1, "Skipping edge 0 because vertex was removed");
          visual_->waitForUserFeedback("skipping edge 0");
        }
        else
        {
          sg_->addEdge(candidateD.newVertex_, v1, eINTERFACE, indent);
        }

        if (sg_->getState(v2) == NULL)
        {
          BOLT_ERROR(indent + 3, 1, "Skipping edge 1 because vertex was removed");
          visual_->waitForUserFeedback("skipping edge 2");
        }
        else
        {
          sg_->addEdge(candidateD.newVertex_, v2, eINTERFACE, indent);
        }

        BOLT_DEBUG(indent, vCriteria_, "INTERFACE: connected two neighbors through new interface node");
      }

      // Report success
      return true;
    }
    else
    {
      BOLT_DEBUG(indent, vCriteria_, "Two closest two neighbors already share "
                                     "an edge, not connecting them");
    }
  }
  BOLT_DEBUG(indent, vCriteria_, "NOT adding node for interface");
  return false;
}

bool SparseCriteria::checkAddQuality(CandidateData &candidateD, std::size_t threadID, std::size_t indent)
{
  if (!useFourthCriteria_)
    return false;

  BOLT_FUNC(indent, vQuality_, "checkAddQuality() Ensure SPARS asymptotic optimality");

  if (candidateD.visibleNeighborhood_.empty())
  {
    BOLT_DEBUG(indent, vQuality_, "no visible neighbors, not adding 4th criteria ");
    return false;
  }

  // base::State *candidateState  // paper's name: q
  SparseVertex candidateRep = candidateD.visibleNeighborhood_[0];  // paper's name: v

  bool added = false;
  std::map<SparseVertex, base::State *> closeRepresentatives;  // [nearSampledRep, nearSampledState]

  findCloseRepresentatives(candidateD.state_, candidateRep, closeRepresentatives, threadID, indent);

  BOLT_DEBUG(indent, vQuality_, "back in checkAddQuality(): Found " << closeRepresentatives.size()
                                                                    << " close representatives");

  bool updated = false;  // track whether a change was made to any vertices' representatives

  // For each pair of close representatives
  for (std::map<SparseVertex, base::State *>::iterator it = closeRepresentatives.begin();
       it != closeRepresentatives.end(); ++it)
  {
    BOLT_DEBUG(indent, vQuality_, "Looping through close representatives");
    base::State *nearSampledState = it->second;  // paper: q'
    SparseVertex nearSampledRep = it->first;     // paper: v'

    if (visualizeQualityCriteriaCloseReps_)  // Visualization
    {
      visual_->viz3()->edge(sg_->getState(nearSampledRep), nearSampledState, tools::MEDIUM, tools::RED);

      visual_->viz3()->state(nearSampledState, tools::MEDIUM, tools::GREEN, 0);

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

  BOLT_DEBUG(indent, vQuality_, "Done updating pair points");

  if (!updated)
  {
    BOLT_DEBUG(indent, vQuality_, "No representatives were updated, so not calling checkAddPath()");
    return false;
  }

  // Attempt to find shortest path through closest neighbour
  if (checkAddPath(candidateRep, indent))
  {
    BOLT_DEBUG(indent, vQuality_, "nearest visible neighbor added for path ");
    added = true;
  }

  // Attempt to find shortest path through other pairs of representatives
  for (std::map<SparseVertex, base::State *>::iterator it = closeRepresentatives.begin();
       it != closeRepresentatives.end(); ++it)
  {
    BOLT_WARN(indent, vQuality_, "Looping through close representatives to add path ===============");

    base::State *nearSampledState = it->second;  // paper: q'
    SparseVertex nearSampledRep = it->first;     // paper: v'
    if (checkAddPath(nearSampledRep, indent))
    {
      BOLT_DEBUG(indent, vQuality_, "Close representative added for path");
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
    visual_->viz3()->edge(sg_->getState(adjVertex), sg_->getState(candidateRep), tools::MEDIUM, tools::GREEN);
    visual_->viz3()->state(sg_->getState(adjVertex), tools::LARGE, tools::PURPLE, 0);
  }

  visual_->viz3()->trigger();
  usleep(0.001 * 1000000);
}

bool SparseCriteria::checkAddPath(SparseVertex v, std::size_t indent)
{
  BOLT_FUNC(indent, vQuality_, "checkAddPath() v = " << v);
  bool spannerPropertyWasViolated = false;

  // Candidate v" vertices as described in the method, filled by function
  // getAdjVerticesOfV1UnconnectedToV2().
  std::vector<SparseVertex> adjVerticesUnconnected;

  // Copy adjacent vertices into vector because we might add additional edges
  // during this function
  std::vector<SparseVertex> adjVertices;
  foreach (SparseVertex adjVertex, boost::adjacent_vertices(v, sg_->getGraph()))
    adjVertices.push_back(adjVertex);

  BOLT_DEBUG(indent, vQuality_, "Vertex v = " << v << " has " << adjVertices.size() << " adjacent vertices, "
                                                                                       "looping:");

  // Loop through adjacent vertices
  for (std::size_t i = 0; i < adjVertices.size() && !spannerPropertyWasViolated; ++i)
  {
    SparseVertex vp = adjVertices[i];  // vp = v' from paper

    BOLT_DEBUG(indent + 2, vQuality_, "Checking v' = " << vp << " ----------------------------");

    // Compute all nodes which qualify as a candidate v" for v and vp
    getAdjVerticesOfV1UnconnectedToV2(v, vp, adjVerticesUnconnected, indent + 4);

    // for each vertex v'' that is adjacent to v (has a valid edge) and does not
    // share an edge with v'
    foreach (SparseVertex vpp, adjVerticesUnconnected)  // vpp = v'' from paper
    {
      BOLT_DEBUG(indent + 4, vQuality_, "Checking v'' = " << vpp);

      InterfaceData &iData = sg_->getInterfaceData(v, vp, vpp, indent + 6);

      double shortestPathVpVpp;  // remember what the shortest path is from astar

      // Check if we need to actually add path
      if (spannerTestOriginal(v, vp, vpp, iData, shortestPathVpVpp, indent + 2))
      {
        // Actually add the vertices and possibly edges
        if (addQualityPath(v, vp, vpp, iData, shortestPathVpVpp, indent + 6))
        {
          spannerPropertyWasViolated = true;

          // Check if by chance v was removed during process
          if (sg_->stateDeleted(v))
          {
            BOLT_INFO(indent, vQuality_, "State v=" << v << " was deleted, skipping quality checks");
            return spannerPropertyWasViolated;
          }

          if (sg_->stateDeleted(vp))
          {
            BOLT_INFO(indent, vQuality_, "State vp=" << vp << " was deleted, skipping quality checks");
            return spannerPropertyWasViolated;
          }
        }
      }

    }  // foreach vpp
  }    // foreach vp

  if (!spannerPropertyWasViolated)
  {
    BOLT_DEBUG(indent, vQuality_, "Spanner property was NOT violated, SKIPPING");
  }

  return spannerPropertyWasViolated;
}

void SparseCriteria::visualizeCheckAddPath(SparseVertex v, SparseVertex vp, SparseVertex vpp, InterfaceData &iData,
                                           std::size_t indent)
{
  BOLT_FUNC(indent, vQuality_, "visualizeCheckAddPath()");
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
  visual_->viz5()->edge(sg_->getState(vp), sg_->getState(v), tools::MEDIUM, tools::GREEN);

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
    visual_->viz5()->state(iData.getInterface1Outside(), tools::MEDIUM, tools::GREEN, 0);
    visual_->viz5()->edge(iData.getInterface1Inside(), iData.getInterface1Outside(), tools::MEDIUM, tools::RED);

    if (vp < vpp)
      visual_->viz5()->edge(sg_->getState(vp), iData.getInterface1Outside(), tools::MEDIUM, tools::RED);
    else
      visual_->viz5()->edge(sg_->getState(vpp), iData.getInterface1Outside(), tools::MEDIUM, tools::RED);
  }
  if (iData.hasInterface2())
  {
    visual_->viz5()->state(iData.getInterface2Inside(), tools::MEDIUM, tools::ORANGE, 0);
    visual_->viz5()->state(iData.getInterface2Outside(), tools::MEDIUM, tools::GREEN, 0);
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
      InterfaceData &iData = sg_->getInterfaceData(v, vpp, x, indent + 2);

      // Check if we previously had found a pair of points that support this
      // interface
      if ((vpp < x && iData.getInterface1Inside()) || (x < vpp && iData.getInterface2Inside()))
      {
        BOLT_INFO(indent + 2, vQuality_, "Visualizing (orange, purple, red, pink, white) additional qualified vertex "
                                             << x);
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
  BOLT_FUNC(indent, vQuality_, "addQualityPath()");

  if (visualizeQualityCriteria_)
    visualizeCheckAddPath(v, vp, vpp, iData, indent + 4);

  // Can we connect these two vertices directly?
  if (si_->checkMotion(sg_->getState(vp), sg_->getState(vpp)))
  {
    BOLT_DEBUG(indent, vQuality_, "Adding edge between vp and vpp");

    BOOST_ASSERT_MSG(!sg_->hasEdge(vp, vpp), "Edge already exists, cannot add quality");
    sg_->addEdge(vp, vpp, eQUALITY, indent + 2);
    // visual_->waitForUserFeedback("addQualityPath");

    return true;
  }

  BOLT_WARN(indent, vQuality_, "Unable to connect directly - geometric path must be created for spanner");

  geometric::PathGeometric *path = new geometric::PathGeometric(si_);

  // Populate path
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
  if (useOriginalSmoother_)
    sg_->smoothQualityPathOriginal(path, indent + 4);
  else
  {
    const bool debug = false;
    sg_->smoothQualityPath(path, sg_->getObstacleClearance(), debug, indent + 4);
  }

  // Determine if this smoothed path actually helps improve connectivity
  if (path->length() > shortestPathVpVpp)
  {
    BOLT_WARN(indent, vQuality_ || 1, "Smoothed path does not improve connectivity");
    //visual_->waitForUserFeedback("smoothed path");
    return false;
  }

  // Insert simplified path into graph
  SparseVertex prior = vp;
  SparseVertex newVertex;
  std::vector<base::State *> &states = path->getStates();

  BOLT_DEBUG(indent + 2, vQuality_, "Shortcuted path now has " << path->getStateCount() << " states");
  BOOST_ASSERT_MSG(states.size() > 2, "Somehow path has shrunk to less than three vertices");

  bool addEdgeEnabled = true;                          // if a vertex is skipped, stop adding edges
  for (std::size_t i = 1; i < states.size() - 1; ++i)  // first and last states are vp and vpp, don't sg_->addVertex()
  {
    base::State *state = states[i];

    // Check if this vertex already exists
    if (si_->distance(sg_->getState(v), state) < denseDelta_)  // TODO: is it ok to re-use this denseDelta var?
    {
      BOLT_ERROR(indent + 2, vQuality_, "Add path state is too similar to v!");

      if (visualizeQualityCriteria_ && false)
      {
        visual_->viz2()->deleteAllMarkers();
        visual_->viz2()->path(path, tools::SMALL, tools::RED);
        visual_->viz2()->trigger();
        usleep(0.001 * 1000000);

        visual_->waitForUserFeedback("Add path state is too similar to v");
      }

      delete path;
      return false;
    }

    // Check if new vertex has enough clearance
    if (!sufficientClearance(state))
    {
      BOLT_WARN(indent + 2, true, "Skipped adding vertex in new path b/c insufficient clearance");
      visual_->waitForUserFeedback("insufficient clearance");
      addEdgeEnabled = false;
      continue;
    }

    // no need to clone state, since we will destroy p; we just copy the pointer
    BOLT_DEBUG(indent + 2, vQuality_, "Adding node from shortcut path for QUALITY");
    newVertex = sg_->addVertex(si_->cloneState(state), QUALITY, indent + 4);

    // Check if there are really close vertices nearby which should be merged
    if (checkRemoveCloseVertices(newVertex, indent + 4))
    {
      // New vertex replaced a nearby vertex, we can continue no further because
      // graph has been re-indexed

      // Remove all edges from all vertices near our new vertex
      sg_->clearEdgesNearVertex(newVertex, indent);

      // TODO should we clearEdgesNearVertex before return true ?
      delete path;
      return true;
    }

    // Remove all edges from all vertices near our new vertex
    sg_->clearEdgesNearVertex(newVertex, indent);

    if (addEdgeEnabled)
    {
      assert(prior != newVertex);
      sg_->addEdge(prior, newVertex, eQUALITY, indent + 2);
      prior = newVertex;
    }
  }

  // clear the states, so memory is not freed twice
  states.clear();

  if (addEdgeEnabled)
  {
    assert(prior != vpp);
    sg_->addEdge(prior, vpp, eQUALITY, indent + 2);
  }

  delete path;

  // visual_->waitForUserFeedback("addQualityPath");

  return true;
}

bool SparseCriteria::spannerTestOriginal(SparseVertex v, SparseVertex vp, SparseVertex vpp, InterfaceData &iData,
                                         double &shortestPathVpVpp, std::size_t indent)
{
  BOLT_FUNC(indent, vQuality_, "spannerTestOriginal()");
  // Computes all nodes which qualify as a candidate x for v, v', and v"
  double midpointPathLength = maxSpannerPath(v, vp, vpp, indent + 2);

  if (stretchFactor_ * iData.getLastDistance() < midpointPathLength)
  {
    BOLT_WARN(indent, vQuality_, "Spanner property violated");
    BOLT_DEBUG(indent + 2, vQuality_, "Sparse Graph Midpoint Length  = " << midpointPathLength);
    BOLT_DEBUG(indent + 2, vQuality_, "Spanner Path Length * Stretch = " << (stretchFactor_ * iData.getLastDistance()));
    BOLT_DEBUG(indent + 2, vQuality_, "last distance = " << iData.getLastDistance());
    BOLT_DEBUG(indent + 2, vQuality_, "stretch factor = " << stretchFactor_);
    double rejectStretchFactor = midpointPathLength / iData.getLastDistance();
    BOLT_DEBUG(indent + 2, vQuality_, "to reject, stretch factor > " << rejectStretchFactor);

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
      BOLT_DEBUG(indent + 2, vQuality_, "newEdgeDistance: " << newEdgeDistance);
      BOLT_DEBUG(indent + 2, vQuality_, "shortestPathVpVpp: " << shortestPathVpVpp);
      // BOLT_DEBUG(indent + 2, vQuality_ || 1, "shortestPathVpVpp+: " << shortestPathVpVpp - SMALL_EPSILON);
      // visual_->viz6()->state(sg_->getState(vp), tools::MEDIUM, tools::BLACK, 0);
      // visual_->viz6()->state(sg_->getState(vpp), tools::MEDIUM, tools::BLACK, 0);
      // visual_->viz6()->trigger();

      if (visualizeQualityCriteria_)                           // TEMP
        visualizeCheckAddPath(v, vp, vpp, iData, indent + 4);  // TEMP

      if (newEdgeDistance > shortestPathVpVpp - SMALL_EPSILON)
      {
        return false;  // skip because there is already a path that achieves this
      }
    }

    return true;  // spanner property was violated
  }

  BOLT_DEBUG(indent + 4, vQuality_, "Spanner property not violated");
  return false;  // spanner property was NOT violated
}

double SparseCriteria::qualityEdgeAstarTest(SparseVertex vp, SparseVertex vpp, InterfaceData &iData, std::size_t indent)
{
  BOLT_FUNC(indent, vQuality_, "qualityEdgeAstarTest()");

  // Experimental calculations
  double pathLength = 0;
  std::vector<SparseVertex> vertexPath;
  if (!sg_->astarSearch(vp, vpp, vertexPath, pathLength, indent))
  {
    BOLT_ERROR(indent, vQuality_, "No path found");
    visual_->waitForUserFeedback("No path found");
    return std::numeric_limits<double>::infinity();
  }

  if (visualizeQualityCriteriaAstar_)
  {
    visual_->viz6()->deleteAllMarkers();
    assert(vertexPath.size() > 1);
    for (std::size_t i = 1; i < vertexPath.size(); ++i)
    {
      visual_->viz6()->edge(sg_->getState(vertexPath[i - 1]), sg_->getState(vertexPath[i]), tools::MEDIUM,
                            tools::GREEN);
    }
    visual_->viz6()->trigger();
  }

  return pathLength;
}

SparseVertex SparseCriteria::findGraphRepresentative(base::State *state, std::size_t threadID, std::size_t indent)
{
  BOLT_FUNC(indent, vQuality_, "findGraphRepresentative()");

  std::vector<SparseVertex> graphNeighbors;

  // Search
  sg_->getQueryStateNonConst(threadID) = state;
  sg_->getNN()->nearestR(sg_->getQueryVertices(threadID), sparseDelta_, graphNeighbors);
  sg_->getQueryStateNonConst(threadID) = nullptr;

  BOLT_DEBUG(indent, vQuality_, "Found " << graphNeighbors.size() << " nearest neighbors (graph rep) within "
                                                                     "SparseDelta " << sparseDelta_);

  SparseVertex result = boost::graph_traits<SparseAdjList>::null_vertex();

  for (std::size_t i = 0; i < graphNeighbors.size(); ++i)
  {
    BOLT_DEBUG(indent, vQuality_, "Checking motion of graph representative candidate " << i);
    if (si_->checkMotion(state, sg_->getState(graphNeighbors[i])))
    {
      BOLT_DEBUG(indent + 2, vQuality_, "graph representative valid ");
      result = graphNeighbors[i];
      break;
    }
    else
      BOLT_DEBUG(indent + 2, vQuality_, "graph representative NOT valid, checking next ");
  }
  return result;
}

void SparseCriteria::findCloseRepresentatives(const base::State *candidateState, const SparseVertex candidateRep,
                                              std::map<SparseVertex, base::State *> &closeRepresentatives,
                                              std::size_t threadID, std::size_t indent)
{
  BOLT_FUNC(indent, vQuality_, "findCloseRepresentatives()");
  BOLT_DEBUG(indent, vQuality_, "nearSamplePoints: " << nearSamplePoints_ << " denseDelta: " << denseDelta_);

  base::State *sampledState = closeRepSampledState_[threadID];  // alias the var name

  assert(closeRepresentatives.empty());

  // Search the space around new potential state candidateState
  for (std::size_t i = 0; i < nearSamplePoints_; ++i)
  {
    BOLT_DEBUG(indent, vQuality_, "Get supporting representative #" << i);

    bool foundValidSample = false;
    static const std::size_t MAX_SAMPLE_ATTEMPT = 1000;
    for (std::size_t attempt = 0; attempt < MAX_SAMPLE_ATTEMPT; ++attempt)
    {
      clearanceSampler_->sampleNear(sampledState, candidateState, denseDelta_);

      if (!si_->isValid(sampledState))
      {
        BOLT_DEBUG(indent + 4, vQuality_ && false, "Sample attempt " << attempt << " notValid ");

        if (visualizeQualityCriteriaSampler_)
          visual_->viz3()->state(sampledState, tools::SMALL, tools::RED, 0);

        continue;
      }
      if (si_->distance(candidateState, sampledState) > denseDelta_)
      {
        BOLT_DEBUG(indent + 4, vQuality_ && false, "Sample attempt " << attempt << " Distance too far "
                                                                     << si_->distance(candidateState, sampledState)
                                                                     << " needs to be less than " << denseDelta_);

        if (visualizeQualityCriteriaSampler_)
          visual_->viz3()->state(sampledState, tools::SMALL, tools::RED, 0);
        continue;
      }
      if (!si_->checkMotion(candidateState, sampledState))
      {
        BOLT_DEBUG(indent + 4, vQuality_ && false, "Sample attempt " << attempt << " motion invalid ");

        if (visualizeQualityCriteriaSampler_)
          visual_->viz3()->state(sampledState, tools::SMALL, tools::RED, 0);
        continue;
      }

      if (visualizeQualityCriteriaSampler_)
        visual_->viz3()->state(sampledState, tools::SMALL, tools::GREEN, 0);

      BOLT_DEBUG(indent + 4, vQuality_ && false, "Sample attempt " << attempt << " valid ");

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
      BOLT_DEBUG(indent + 2, vQuality_, "Unable to find valid sample after " << MAX_SAMPLE_ATTEMPT << " attempts"
                                                                                                      " ");
    }
    else
      BOLT_DEBUG(indent + 2, vQuality_, "Found valid nearby sample");

    // Compute which sparse vertex represents this new candidate vertex
    SparseVertex sampledStateRep = findGraphRepresentative(sampledState, threadID, indent + 6);

    // Check if sample is not visible to any other node (it should be visible
    // in all likelihood)
    if (sampledStateRep == boost::graph_traits<SparseAdjList>::null_vertex())
    {
      BOLT_DEBUG(indent + 2, vQuality_, "Sampled state has no representative (is null) ");

      // It can't be seen by anybody, so we should take this opportunity to add it
      // But first check for proper clearance
      if (sufficientClearance(sampledState))
      {
        BOLT_DEBUG(indent + 2, vQuality_, "Adding node for COVERAGE");
        sg_->addVertex(si_->cloneState(sampledState), COVERAGE, indent + 2);
      }

      BOLT_DEBUG(indent + 2, vQuality_, "STOP EFFORTS TO ADD A DENSE PATH");

      // We should also stop our efforts to add a dense path
      for (std::map<SparseVertex, base::State *>::iterator it = closeRepresentatives.begin();
           it != closeRepresentatives.end(); ++it)
        si_->freeState(it->second);
      closeRepresentatives.clear();
      break;
    }

    BOLT_DEBUG(indent + 2, vQuality_, "Sampled state has representative (is not null)");

    // If its representative is different than candidateState
    if (sampledStateRep != candidateRep)
    {
      BOLT_DEBUG(indent + 2, vQuality_, "candidateRep != sampledStateRep ");

      // And we haven't already tracked this representative
      if (closeRepresentatives.find(sampledStateRep) == closeRepresentatives.end())
      {
        BOLT_DEBUG(indent + 2, vQuality_, "Track the representative");

        // Track the representative
        closeRepresentatives[sampledStateRep] = si_->cloneState(sampledState);
      }
      else
      {
        BOLT_DEBUG(indent + 2, vQuality_, "Already tracking the representative");
      }
    }
    else
    {
      BOLT_DEBUG(indent + 2, vQuality_, "candidateRep == sampledStateRep, do not keep this sample ");
    }
  }  // for each supporting representative
}

bool SparseCriteria::updatePairPoints(SparseVertex candidateRep, const base::State *candidateState,
                                      SparseVertex nearSampledRep, const base::State *nearSampledState,
                                      std::size_t indent)
{
  BOLT_FUNC(indent, vQuality_, "updatePairPoints()");
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
  BOLT_FUNC(indent, vQuality_, "getAdjVerticesOfV1UnconnectedToV2()");

  adjVerticesUnconnected.clear();
  foreach (SparseVertex adjVertex, boost::adjacent_vertices(v1, sg_->getGraph()))
    if (adjVertex != v2)
      if (!sg_->hasEdge(adjVertex, v2))
        adjVerticesUnconnected.push_back(adjVertex);

  BOLT_DEBUG(indent, vQuality_, "adjVerticesUnconnected size: " << adjVerticesUnconnected.size());
}

double SparseCriteria::maxSpannerPath(SparseVertex v, SparseVertex vp, SparseVertex vpp, std::size_t indent)
{
  BOLT_FUNC(indent, vQualityMaxSpanner_, "maxSpannerPath()");

  // Candidate x vertices as described in paper in Max_Spanner_Path
  std::vector<SparseVertex> qualifiedVertices;

  // Get nearby vertices 'x' that could also be used to find the path to v''
  foreach (SparseVertex x, boost::adjacent_vertices(vpp, sg_->getGraph()))
  {
    // Check if vertex is deleted
    if (sg_->getState(x) == NULL)
    {
      BOLT_ERROR(indent, true, "State is deleted in maxSpannerPath!");
      throw Exception(name_, "error");
    }

    if (sg_->hasEdge(x, v) && !sg_->hasEdge(x, vp))
    {
      InterfaceData &iData = sg_->getInterfaceData(v, vpp, x, indent + 2);

      // Check if we previously had found a pair of points that support this
      // interface
      if ((vpp < x && iData.getInterface1Inside()) || (x < vpp && iData.getInterface2Inside()))
      {
        BOLT_WARN(indent, vQualityMaxSpanner_, "Found an additional qualified vertex " << x);
        // This is a possible alternative path to v''
        qualifiedVertices.push_back(x);

        if (visualizeQualityCriteria_ && false)
        {
          visual_->viz5()->state(sg_->getState(x), tools::LARGE, tools::BLACK, 0);
        }
      }
    }
  }

  // vpp is always qualified because of its previous checks
  qualifiedVertices.push_back(vpp);
  BOLT_DEBUG(indent, vQualityMaxSpanner_, "Total qualified vertices found: " << qualifiedVertices.size());

  // Find the maximum spanner distance
  BOLT_DEBUG(indent, vQualityMaxSpanner_, "Finding the maximum spanner distance between v' and v''");
  double maxDist = 0.0;
  foreach (SparseVertex qualifiedVertex, qualifiedVertices)
  {
    if (visualizeQualityCriteria_ && false)
      visual_->viz5()->state(sg_->getState(qualifiedVertex), tools::SMALL, tools::PINK, 0);

    // Divide by 2 because of the midpoint path 'M'
    double tempDist = (si_->distance(sg_->getState(vp), sg_->getState(v)) +
                       si_->distance(sg_->getState(v), sg_->getState(qualifiedVertex))) /
                      2.0;
    BOLT_DEBUG(indent + 2, vQualityMaxSpanner_,
               "Checking vertex: " << qualifiedVertex << " distance: " << tempDist
                                   << " dist1: " << si_->distance(sg_->getState(vp), sg_->getState(v))
                                   << " dist2: " << si_->distance(sg_->getState(v), sg_->getState(qualifiedVertex)));

    // Compare with previous max
    if (tempDist > maxDist)
    {
      BOLT_DEBUG(indent + 4, vQualityMaxSpanner_, "Is larger than previous");
      maxDist = tempDist;
    }
  }
  BOLT_DEBUG(indent, vQualityMaxSpanner_, "Max distance: " << maxDist);

  return maxDist;
}

bool SparseCriteria::distanceCheck(SparseVertex v, const base::State *q, SparseVertex vp, const base::State *qp,
                                   SparseVertex vpp, std::size_t indent)
{
  BOLT_FUNC(indent, vQuality_, "distanceCheck()");

  bool updated = false;  // track whether a change was made to any vertices'
                         // representatives

  // Get the info for the current representative-neighbors pair
  InterfaceData &iData = sg_->getInterfaceData(v, vp, vpp, indent + 4);

  if (vp < vpp)  // FIRST points represent r (the interface discovered through
                 // sampling)
  {
    if (!iData.hasInterface1())  // No previous interface has been found here,
                                 // just save it
    {
      BOLT_DEBUG(indent, vQuality_, "setInterface1");
      iData.setInterface1(q, qp, si_);
      updated = true;
    }
    else if (!iData.hasInterface2())  // The other interface doesn't exist,
                                      // so we can't compare.
    {
      // Should probably keep the one that is further away from rep?  Not
      // known what to do in this case.
      // TODO: is this not part of the algorithm?
      BOLT_WARN(indent, vQuality_, "TODO no interface 2");
    }
    else  // We know both of these points exist, so we can check some
          // distances
    {
      assert(iData.getLastDistance() < std::numeric_limits<double>::infinity());
      if (si_->distance(q, iData.getInterface2Inside()) < iData.getLastDistance())
      // si_->distance(iData.getInterface1Inside(),
      // iData.getInterface2Inside()))
      {  // Distance with the new point is good, so set it.
        BOLT_GREEN_DEBUG(indent, vQuality_, "setInterface1 UPDATED");
        iData.setInterface1(q, qp, si_);
        updated = true;
      }
      else
      {
        BOLT_DEBUG(indent, vQuality_, "Distance was not better, not updating bookkeeping");
      }
    }
  }
  else  // SECOND points represent r (the interfaec discovered through
        // sampling)
  {
    if (!iData.hasInterface2())  // No previous interface has been found here,
                                 // just save it
    {
      BOLT_DEBUG(indent, vQuality_, "setInterface2");
      iData.setInterface2(q, qp, si_);
      updated = true;
    }
    else if (!iData.hasInterface1())  // The other interface doesn't exist,
                                      // so we can't compare.
    {
      // Should we be doing something cool here?
      BOLT_WARN(indent, vQuality_, "TODO no interface 1");
    }
    else  // We know both of these points exist, so we can check some
          // distances
    {
      assert(iData.getLastDistance() < std::numeric_limits<double>::infinity());
      if (si_->distance(q, iData.getInterface1Inside()) < iData.getLastDistance())
      // si_->distance(iData.getInterface2Inside(),
      // iData.getInterface1Inside()))
      {  // Distance with the new point is good, so set it
        BOLT_GREEN_DEBUG(indent, vQuality_, "setInterface2 UPDATED");
        iData.setInterface2(q, qp, si_);
        updated = true;
      }
      else
      {
        BOLT_DEBUG(indent, vQuality_, "Distance was not better, not updating bookkeeping");
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

bool SparseCriteria::checkRemoveCloseVertices(SparseVertex v1, std::size_t indent)
{
  // This feature can be optionally disabled
  if (!useCheckRemoveCloseVertices_)
    return false;

  BOLT_FUNC(indent, vRemoveClose_, "checkRemoveCloseVertices() v1 = " << v1);

  // Get neighbors
  std::vector<SparseVertex> graphNeighbors;
  const std::size_t numNeighbors = 2;  // the first neighbor is (usually?) the vertex itself
  sg_->getNN()->nearestK(v1, numNeighbors, graphNeighbors);

  if (vRemoveClose_)
  {
    std::stringstream ss;
    std::copy(graphNeighbors.begin(), graphNeighbors.end(), std::ostream_iterator<SparseVertex>(ss, ", "));
    BOLT_DEBUG(indent, vRemoveClose_, "Neighbors of " << v1 << " are: [" << ss.str() << "]");
  }

  // Error check no neighbors
  if (graphNeighbors.size() <= 1)
  {
    BOLT_ERROR(indent, vRemoveClose_, "checkRemoveCloseVertices: no neighbors found for sparse vertex " << v1);
    return false;
  }

  SparseVertex v2 = graphNeighbors[1];

  // Error check: Do not remove itself
  if (v1 == v2)
  {
    BOLT_ERROR(indent, vRemoveClose_, "checkRemoveCloseVertices: error occured, cannot remove self: " << v1);
    exit(-1);
  }

  // Check that nearest neighbor is not an quality node - these should not be
  // moved
  if (sg_->getVertexTypeProperty(v2) == QUALITY)
  {
    if (visualizeRemoveCloseVertices_)
    {
      visualizeRemoveCloseVertices(v1, v2);
      visual_->waitForUserFeedback("Skipping this vertex because is QUALITY");
    }
    return false;
  }

  // Check if nearest neighbor is within distance threshold
  if (sg_->distanceFunction(v1, v2) > sparseDelta_ * sparseDeltaFractionCheck_)
  {
    // BOLT_DEBUG(indent, vRemoveClose_, "Distance " <<
    // sg_->distanceFunction(v1, v2) << " is greater than max "
    //<< sparseDelta_ * sparseDeltaFractionCheck_);
    return false;
  }

  // Check if nearest neighbor is collision free
  if (!si_->checkMotion(sg_->getState(v1), sg_->getState(v2)))
  {
    BOLT_ERROR(indent, vRemoveClose_, "checkRemoveCloseVertices: not collision free v1=" << v1 << ", v2=" << v2);
    return false;
  }

  BOLT_DEBUG(indent, vRemoveClose_, "Found visible nearby node, testing if able to replace " << v2 << " with " << v1);

  // Nearest neighbor is good candidate, next check if all of its connected
  // neighbors can be connected to new vertex
  foreach (SparseEdge edge, boost::out_edges(v2, sg_->getGraph()))
  {
    SparseVertex v3 = boost::target(edge, sg_->getGraph());
    BOLT_DEBUG(indent + 2, vRemoveClose_, "checking edge v1= " << v1 << " to v3=" << v3);

    // Check if distance is within sparseDelta
    if (si_->distance(sg_->getState(v1), sg_->getState(v3)) > sparseDelta_)
    {
      BOLT_DEBUG(indent + 2, vRemoveClose_, "checkRemoveCloseVertices: distance too far from previous neighbor " << v3);
      return false;
    }

    // Check if collision free path to connected vertex
    if (!si_->checkMotion(sg_->getState(v1), sg_->getState(v3)))
    {
      BOLT_ERROR(indent + 2, vRemoveClose_,
                 "checkRemoveCloseVertices: not collision free path from new vertex to potential neighbor " << v3);
      return false;
    }
  }

  BOLT_DEBUG(indent, vRemoveClose_, "Found qualified node to replace with nearby");

  if (visualizeRemoveCloseVertices_)
  {
    visualizeRemoveCloseVertices(v1, v2);
    visual_->waitForUserFeedback("found qualified node to replace with nearby");
  }

  // Remove all interface data for old state
  sg_->clearInterfaceData(sg_->getStateNonConst(v2));

  // Connect new vertex to old vertex's connected neighbors
  foreach (SparseEdge edge, boost::out_edges(v2, sg_->getGraph()))
  {
    SparseVertex v3 = boost::target(edge, sg_->getGraph());
    BOLT_DEBUG(indent + 2, vRemoveClose_, "Connecting v1= " << v1 << " to v3=" << v3);
    sg_->addEdge(v1, v3, eINTERFACE, indent + 4);
  }

  // Delete old vertex
  sg_->removeVertex(v2, indent);
  BOLT_DEBUG(indent, vRemoveClose_, "REMOVING VERTEX " << v2 << " which was replaced with " << v1 << " with state ");

  // Statistics
  numVerticesMoved_++;

  // Only display database if enabled
  if (sg_->visualizeSparseGraph_ && sg_->visualizeSparseGraphSpeed_ > std::numeric_limits<double>::epsilon())
    sg_->displayDatabase(true, indent + 2);

  // if (visualizeRemoveCloseVertices_)
  // visual_->waitForUserFeedback("finished moving vertex");

  if (visualizeRemoveCloseVertices_)
  {
    visual_->viz6()->deleteAllMarkers();
    visual_->viz6()->trigger();
    usleep(0.001 * 1000000);
  }

  return true;
}

void SparseCriteria::visualizeRemoveCloseVertices(SparseVertex v1, SparseVertex v2)
{
  visual_->viz6()->deleteAllMarkers();
  visual_->viz6()->state(sg_->getState(v1), tools::LARGE, tools::GREEN, 0);
  visual_->viz6()->state(sg_->getState(v2), tools::LARGE, tools::RED, 0);  // RED = to be removed
  visual_->viz6()->trigger();
  usleep(0.001 * 1000000);
}

void SparseCriteria::visualizeInterfaces(SparseVertex v, std::size_t indent)
{
  BOLT_FUNC(indent, vQuality_, "visualizeInterfaces()");

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
      visual_->viz6()->state(iData.getInterface1Outside(), tools::MEDIUM, tools::GREEN, 0);
      visual_->viz6()->edge(iData.getInterface1Inside(), iData.getInterface1Outside(), tools::MEDIUM, tools::YELLOW);
    }
    else
    {
      visual_->viz6()->edge(sg_->getState(v1), sg_->getState(v2), tools::MEDIUM, tools::RED);
    }

    if (iData.hasInterface2())
    {
      visual_->viz6()->state(iData.getInterface2Inside(), tools::MEDIUM, tools::ORANGE, 0);
      visual_->viz6()->state(iData.getInterface2Outside(), tools::MEDIUM, tools::GREEN, 0);
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
  BOLT_FUNC(indent, vQuality_, "visualizeAllInterfaces()");

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
        visual_->viz6()->state(iData.getInterface1Outside(), tools::MEDIUM, tools::GREEN, 0);
      }

      if (iData.hasInterface2())
      {
        visual_->viz6()->state(iData.getInterface2Inside(), tools::MEDIUM, tools::ORANGE, 0);
        visual_->viz6()->state(iData.getInterface2Outside(), tools::MEDIUM, tools::GREEN, 0);
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

bool SparseCriteria::sufficientClearance(base::State *state)
{
  // Check if new vertex has enough clearance
  double dist = si_->getStateValidityChecker()->clearance(state);
  return dist >= sg_->getObstacleClearance();
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
