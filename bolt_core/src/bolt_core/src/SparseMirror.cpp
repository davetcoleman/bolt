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
   Desc:   Mirror graph for secondary arm
*/

// OMPL
#include <bolt_core/SparseMirror.h>

// Boost
#include <boost/foreach.hpp>

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
SparseMirror::SparseMirror(SparseGraphPtr sg) : sg_(sg), si_(sg_->getSpaceInformation()), visual_(sg_->getVisual())
{
}

SparseMirror::~SparseMirror(void)
{
}

void SparseMirror::clear()
{
}

bool SparseMirror::setup(std::size_t indent)
{
  return true;
}

void SparseMirror::mirrorGraphDualArm(base::SpaceInformationPtr dualSpaceInfo,
                                      base::SpaceInformationPtr leftArmSpaceInfo, const std::string &outputFile,
                                      std::size_t indent)
{
  BOLT_FUNC(indent, true, "mirrorGraphDualArm()");

  // Error check
  BOLT_ASSERT(dualSpaceInfo->getStateSpace()->getDimension() == si_->getStateSpace()->getDimension() * 2,
              "Number of dimensions in dual space info should be double the mono space info");

  // -----------------------------------------------------------------------------
  // Create SparseGraph for dual arm
  SparseGraphPtr dualSparseGraph = SparseGraphPtr(new SparseGraph(dualSpaceInfo, visual_));
  dualSparseGraph->setFilePath(outputFile + ".ompl");

  if (visualizeMiroring_)
  {
    visual_->viz1()->deleteAllMarkers();
    visual_->viz1()->trigger();
  }

  // -----------------------------------------------------------------------------
  // Insert mono SparseGraph into dual SparseGraph

  // Map from every sparseV1 vertex to every vertex in the new graph that has the same roots
  std::vector<std::vector<SparseVertex>> vertexMapMatrix(sg_->getNumVertices());

  // Allocate memory for copying the mirrored version of state2
  base::State *state2Mirrored = leftArmSpaceInfo->getStateSpace()->allocState();

  // Loop through every vertex in sparse graph
  BOLT_DEBUG(indent, true, "Adding " << sg_->getNumVertices() << " sparse graph vertices into dual arm graph");
  std::size_t showEvery = std::max((unsigned int)(1), sg_->getNumVertices() / 100);
  foreach (SparseVertex sparseV1, boost::vertices(sg_->getGraph()))
  {
    // The first thread number of verticies are used for queries and should be skipped
    if (sparseV1 < sg_->getNumQueryVertices())
      continue;

    BOLT_INFO(indent, (sparseV1 % showEvery == 0) || true,
              "Mirroring graph progress: " << (double(sparseV1) / sg_->getNumVertices() * 100.0)
                                           << "%. Dual graph verticies: " << dualSparseGraph->getNumVertices()
                                           << ", Dual graph edges: " << dualSparseGraph->getNumEdges());

    const base::State *state1 = sg_->getState(sparseV1);
    if (visualizeMiroring_)
      visual_->viz1()->state(state1, tools::ROBOT, tools::DEFAULT, 0);

    // Record a mapping from the old vertex index to the new index
    std::vector<SparseVertex> sparseV2ToDualVertex(sg_->getNumVertices(), /*initial value*/ 0);

    // Loop through every vertex again
    std::size_t skippedStates = 0;
    foreach (SparseVertex sparseV2, boost::vertices(sg_->getGraph()))
    {
      // The first thread number of verticies are used for queries and should be skipped
      if (sparseV2 < sg_->getNumQueryVertices())
        continue;

      const base::State *state2 = sg_->getState(sparseV2);
      mirrorState(state2, state2Mirrored, indent);

      if (visualizeMiroring_)
        visual_->viz2()->state(state2Mirrored, tools::ROBOT, tools::DEFAULT, /*extraData*/ 0, leftArmSpaceInfo);

      // Create dual state
      base::State *stateCombined = combineStates(state1, state2Mirrored, dualSpaceInfo, indent);

      // Check that new state is still valid
      if (collisionCheckMirror_ && !dualSpaceInfo->isValid(stateCombined))
      {
        BOLT_DEBUG(indent, vMirror_, "found invalid combined state");

        if (visualizeMiroring_)
        {
          visual_->viz3()->state(stateCombined, tools::ROBOT, tools::RED, /*extraData*/ 0, dualSpaceInfo);

          visual_->waitForUserFeedback("found invalid combined state");
        }

        skippedStates++;
        continue;
      }
      else  // show valid robot
      {
        // visual_->viz3()->state(stateCombined, tools::ROBOT, tools::DEFAULT, /*extraData*/ 0, dualSpaceInfo);
        // usleep(0.001*1000000);
      }

      // Insert into new graph
      SparseVertex vertexCombined = dualSparseGraph->addVertex(stateCombined, DISCRETIZED, indent);

      // Short term mapping
      sparseV2ToDualVertex[sparseV2] = vertexCombined;
    }
    BOLT_DEBUG(indent, vMirrorStatus_, "Total states skipped: " << (double(skippedStates) / sg_->getNumVertices() * 100)
                                                                << "%. Skipped " << skippedStates
                                                                << " states out of total " << sg_->getNumVertices());
    // visual_->waitForUserFeedback("before adding edges");

    // Copy the short term mapping into the long term mapping
    vertexMapMatrix[sparseV1] = sparseV2ToDualVertex;

    // Loop through every edge in sparse graph and connect this subgraph
    addEdgesForDim(sparseV2ToDualVertex, dualSparseGraph, dualSpaceInfo, indent);

    // visual_->waitForUserFeedback("Done copying edges for inner loop");

    if (visual_->viz1()->shutdownRequested())
      break;
  }  // end foreach vertex

  // Free memory
  leftArmSpaceInfo->getStateSpace()->freeState(state2Mirrored);

  if (visual_->viz1()->shutdownRequested())
    return;

  BOLT_INFO(indent, true, "Done generating intial copy of graphs (but not interconnected edges).");

  // Save graph
  // dualSparseGraph->save(indent);

  // Add all cross edges
  addEdgesForAll(vertexMapMatrix, dualSparseGraph, dualSpaceInfo, indent);

  // Save graph
  dualSparseGraph->save(indent);
}

base::State *SparseMirror::combineStates(const base::State *state1, const base::State *state2,
                                         base::SpaceInformationPtr dualSpaceInfo, std::size_t indent)
{
  base::State *stateCombined = dualSpaceInfo->getStateSpace()->allocState();

  // Get the values of the individual states
  std::vector<double> values1, values2;
  si_->getStateSpace()->copyToReals(values1, state1);
  si_->getStateSpace()->copyToReals(values2, state2);

  // TODO: this is specific to Baxter, NOT robot agnostic
  // Combine vector2 into vector1
  values2.insert(values2.end(), values1.begin(), values1.end());

  // Fill the state with current values
  dualSpaceInfo->getStateSpace()->copyFromReals(stateCombined, values2);

  if (vMirror_)
  {
    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    si_->getStateSpace()->printState(state1);

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    si_->getStateSpace()->printState(state2);

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    dualSpaceInfo->getStateSpace()->printState(stateCombined);
  }

  return stateCombined;
}

void SparseMirror::addEdgesForDim(std::vector<SparseVertex> &sparseV2ToDualVertex, SparseGraphPtr &dualSparseGraph,
                                  base::SpaceInformationPtr dualSpaceInfo, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "addEdgesForDim");

  // Loop through every edge in sparse graph and connect this subgraph
  std::size_t skippedEdges = 0;
  std::size_t skippedInvalidEdges = 0;
  foreach (const SparseEdge sparseE, boost::edges(sg_->getGraph()))
  {
    const SparseVertex sparseE_v0 = sparseV2ToDualVertex[boost::source(sparseE, sg_->getGraph())];
    const SparseVertex sparseE_v1 = sparseV2ToDualVertex[boost::target(sparseE, sg_->getGraph())];
    if (sparseE_v0 == 0 || sparseE_v1 == 0)
    {
      skippedEdges++;
      continue;
    }

    EdgeType type = sg_->getEdgeTypeProperty(sparseE);

    if (collisionCheckMirror_ &&
        !dualSpaceInfo->checkMotion(dualSparseGraph->getState(sparseE_v0), dualSparseGraph->getState(sparseE_v1)))
    {
      BOLT_DEBUG(indent, vMirror_, "found invalid combined state");

      if (vMirror_)
      {
        visual_->viz3()->state(dualSparseGraph->getState(sparseE_v0), tools::ROBOT, tools::RED, /*extraData*/ 0,
                               dualSpaceInfo);
        usleep(0.001 * 1000000);
        visual_->waitForUserFeedback("found invalid combined state");
      }

      skippedInvalidEdges++;
      continue;
    }

    // Create edge
    // dualSparseGraph->visualizeSparseGraph_ = true;
    // dualSparseGraph->visualizeSparseGraphSpeed_ = 0.001;
    // dualSparseGraph->visualizeDatabaseVertices_ = false;
    // dualSparseGraph->visualizeDatabaseEdges_ = true;
    dualSparseGraph->addEdge(sparseE_v0, sparseE_v1, type, indent);
  }

  BOLT_DEBUG(indent, vMirrorStatus_,
             "Total edges skipped: " << (double(skippedEdges + skippedInvalidEdges) / sg_->getNumEdges() * 100)
                                     << "%. Skipped " << skippedEdges << " edges for lack of endpoints, "
                                     << skippedInvalidEdges
                                     << " due to collision, out of total: " << sg_->getNumEdges());
}

void SparseMirror::addEdgesForAll(std::vector<std::vector<SparseVertex>> &vertexMapMatrix,
                                  SparseGraphPtr dualSparseGraph, base::SpaceInformationPtr dualSpaceInfo,
                                  std::size_t indent)
{
  BOLT_FUNC(indent, true, "addEdgesForAll()");

  // Add all edges across each dimension
  // Loop through every edge in sparse graph and connect this subgraph
  std::size_t skippedEdges = 0;
  std::size_t skippedInvalidEdges = 0;
  std::size_t count = 1;
  const std::size_t showEvery = std::max((unsigned int)(1), sg_->getNumEdges() / 100);
  foreach (const SparseEdge sparseE, boost::edges(sg_->getGraph()))
  {
    const SparseVertex sparseE_v0 = boost::source(sparseE, sg_->getGraph());
    const SparseVertex sparseE_v1 = boost::target(sparseE, sg_->getGraph());

    EdgeType type = sg_->getEdgeTypeProperty(sparseE);

    // Duplicate this edge for all verticies derived from this pair of vertices
    for (const SparseVertex dualVertex0 : vertexMapMatrix[sparseE_v0])
    {
      // The first thread number of verticies are used for queries and should be skipped
      if (dualVertex0 < dualSparseGraph->getNumQueryVertices())
        continue;

      for (const SparseVertex dualVertex1 : vertexMapMatrix[sparseE_v1])
      {
        // The first thread number of verticies are used for queries and should be skipped
        if (dualVertex1 < dualSparseGraph->getNumQueryVertices())
          continue;

        if (dualSparseGraph->hasEdge(dualVertex0, dualVertex1))
        {
          skippedEdges++;
          continue;
        }

        if (collisionCheckMirror_ &&
            !dualSpaceInfo->checkMotion(dualSparseGraph->getState(dualVertex0), dualSparseGraph->getState(dualVertex1)))
        {
          skippedInvalidEdges++;
          continue;
        }

        // Create edge
        dualSparseGraph->addEdge(dualVertex0, dualVertex1, type, indent);
      }

      if (visual_->viz1()->shutdownRequested())
        break;
    }

    // Status
    count++;
    BOLT_DEBUG(indent, (count % showEvery == 0), "Mirror edge "
                                                     << count << " of " << sg_->getNumEdges() << " ("
                                                     << int(ceil(double(count) / sg_->getNumEdges() * 100))
                                                     << "%). Skipped dup: " << skippedEdges
                                                     << ". Skipped invalid: " << skippedInvalidEdges
                                                     << ", DualGraph vert: " << dualSparseGraph->getNumVertices()
                                                     << ", DualGraph edge: " << dualSparseGraph->getNumEdges());

    if (visual_->viz1()->shutdownRequested())
      break;
  }

  BOLT_DEBUG(indent, vMirrorStatus_, "Skipped " << skippedEdges << " edges because duplicate, " << skippedInvalidEdges
                                                << " due to collision, out of total: " << sg_->getNumEdges());
}

void SparseMirror::mirrorState(const base::State *source, base::State *dest, std::size_t indent)
{
  const std::vector<ompl::base::StateSpace::ValueLocation> &valueLocations = si_->getStateSpace()->getValueLocations();

  // Loop through each value
  for (std::size_t i = 0; i < valueLocations.size(); ++i)
  {
    double jointValue = *si_->getStateSpace()->getValueAddressAtLocation(source, valueLocations[i]);

    BOLT_DEBUG(indent, vMirror_, "jointValue: " << jointValue);
    double newJointValue;

    // TODO: this is specific to Baxter, NOT robot agnostic
    if (i % 2 == 0)
    {
      // Get bounds
      const ompl::base::RealVectorBounds &bounds = si_->getStateSpace()->getBounds();

      BOLT_DEBUG(indent, vMirror_, "bounds.high[i]: " << bounds.high[i]);
      BOLT_DEBUG(indent, vMirror_, "bounds.low[i]: " << bounds.low[i]);
      double range = bounds.high[i] - bounds.low[i];
      BOLT_DEBUG(indent, vMirror_, "range: " << range);
      double percent = (jointValue - bounds.low[i]) / range;
      BOLT_DEBUG(indent, vMirror_, "percent: " << percent);
      double inversePercent = 1.0 - percent;
      BOLT_DEBUG(indent, vMirror_, "inversePercent: " << inversePercent);
      newJointValue = inversePercent * range + bounds.low[i];
      BOLT_DEBUG(indent, vMirror_, "newJointValue: " << newJointValue);
      if (vMirror_)
      {
        printJointLimits(bounds.low[i], bounds.high[i], jointValue, "joint");
        printJointLimits(bounds.low[i], bounds.high[i], newJointValue, "joint");
      }
      BOLT_DEBUG(indent, vMirror_, "-------------------------------------------------------");
    }
    else
      newJointValue = jointValue;

    *si_->getStateSpace()->getValueAddressAtLocation(dest, valueLocations[i]) = newJointValue;
  }
}

void SparseMirror::printJointLimits(double min, double max, double value, const std::string &name)
{
  std::cout << "   " << std::fixed << std::setprecision(5) << min << "\t";
  double delta = max - min;
  // std::cout << "delta: " << delta << " ";
  double step = delta / 20.0;

  bool marker_shown = false;
  for (double value_it = min; value_it < max; value_it += step)
  {
    // show marker of current value
    if (!marker_shown && value < value_it)
    {
      std::cout << "|";
      marker_shown = true;
    }
    else
      std::cout << "-";
  }
  // show max position
  std::cout << " \t" << std::fixed << std::setprecision(5) << max << "  \t" << name << " current: " << std::fixed
            << std::setprecision(5) << value << std::endl;
}

// Loop through every vertex in sparse graph and check if state is valid on opposite arm
void SparseMirror::checkValidityOfArmMirror(base::SpaceInformationPtr dualSpaceInfo,
                                            base::SpaceInformationPtr leftArmSpaceInfo, std::size_t indent)
{
  BOLT_FUNC(indent, true, "checkValidityOfArmMirror()");
  // Error check
  BOLT_ASSERT(dualSpaceInfo->getStateSpace()->getDimension() == si_->getStateSpace()->getDimension() * 2,
              "Number of dimensions in dual space info should be double the mono space info");

  // -----------------------------------------------------------------------------
  // Create SparseGraph for dual arm
  SparseGraphPtr dualSparseGraph = SparseGraphPtr(new SparseGraph(dualSpaceInfo, visual_));

  visual_->viz1()->deleteAllMarkers();
  visual_->viz1()->trigger();

  // Allocate memory for copying the mirrored version of state2
  base::State *state2Mirrored = leftArmSpaceInfo->getStateSpace()->allocState();

  // Loop through every vertex in sparse graph and check if state is valid on opposite arm
  BOLT_DEBUG(indent, true, "Checking " << sg_->getNumVertices() << " sparse graph vertices for validity on "
                                                                   "opposite arm");
  const std::size_t showEvery = std::max((unsigned int)(1), sg_->getNumVertices() / 100);
  std::size_t skippedStates = 0;
  foreach (SparseVertex sparseV1, boost::vertices(sg_->getGraph()))
  {
    // The first thread number of verticies are used for queries and should be skipped
    if (sparseV1 < sg_->getNumQueryVertices())
      continue;

    BOLT_INFO(indent, (sparseV1 % showEvery == 0),
              "Mirroring graph progress: " << (double(sparseV1) / sg_->getNumVertices() * 100.0) << "%");

    const base::State *state1 = sg_->getState(sparseV1);
    visual_->viz1()->state(state1, tools::ROBOT, tools::DEFAULT, 0);

    mirrorState(state1, state2Mirrored, indent);

    // visual_->viz2()->state(state2Mirrored, tools::ROBOT, tools::DEFAULT, /*extraData*/ 0, leftArmSpaceInfo);

    // Check that new state is still valid
    if (!leftArmSpaceInfo->isValid(state2Mirrored))
    {
      BOLT_DEBUG(indent, true, "found invalid left state");
      visual_->viz2()->state(state2Mirrored, tools::ROBOT, tools::RED, /*extraData*/ 0, leftArmSpaceInfo);

      visual_->waitForUserFeedback("found invalid left state");

      skippedStates++;
      continue;
    }
    else if (false)
    {
      visual_->viz2()->state(state2Mirrored, tools::ROBOT, tools::DEFAULT, /*extraData*/ 0, leftArmSpaceInfo);
      // usleep(0.001*1000000);
      visual_->waitForUserFeedback("found valid state");
    }

    if (visual_->viz1()->shutdownRequested())
      return;

  }  // end for each vertex
  BOLT_DEBUG(indent, true, "Skipped " << skippedStates << " states out of total " << sg_->getNumRealVertices());

  // Free memory
  leftArmSpaceInfo->getStateSpace()->freeState(state2Mirrored);
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
