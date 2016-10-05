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
#include <bolt_core/SparseCriteria.h>

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
SparseMirror::SparseMirror(SparseGraphPtr sg)
  : monoSG_(sg), monoSI_(monoSG_->getSpaceInformation()), visual_(monoSG_->getVisual())
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
  BOLT_ASSERT(monoSI_->getStateSpace()->getDimension() == leftArmSpaceInfo->getStateSpace()->getDimension(),
              "Number of dimensions for both arms must be the same");
  BOLT_ASSERT(dualSpaceInfo->getStateSpace()->getDimension() >= monoSI_->getStateSpace()->getDimension() * 2,
              "Number of dimensions in dual space info should be double or more the mono space number of dimension");

  // -----------------------------------------------------------------------------
  // Create SparseGraph for dual arm
  SparseGraphPtr dualSG = SparseGraphPtr(new SparseGraph(dualSpaceInfo, visual_));
  dualSG->setFilePath(outputFile + ".ompl");
  // Copy settings
  dualSG->savingEnabled_ = monoSG_->savingEnabled_;

  // -----------------------------------------------------------------------------
  // Load sparse criteria formulas
  // SparseFormula dualFormulas;
  SparseCriteriaPtr monoSC = monoSG_->getSparseCriteria();
  sparseDelta_ = monoSC->getSparseDelta();

  // dualFormulas.calc(dualSpaceInfo, monoSC->getStretchFactor(), monoSC->getSparseDeltaFraction(),
  //                   monoSC->getPenetrationOverlapFraction(), monoSC->getNearSamplePointsMultiple(),
  //                   monoSC->getUseL2Norm());
  // Copy values back
  // maxExtent_ = dualFormulas.maxExtent_;
  // sparseDelta_ = dualFormulas.sparseDelta_;
  // denseDelta_ = dualFormulas.denseDelta_;
  // discretizePenetrationDist_ = dualFormulas.discretizePenetrationDist_;
  // nearSamplePoints_ = dualFormulas.nearSamplePoints_;
  // discretization_ = dualFormulas.discretization_;
  // stretchFactor_ = dualFormulas.stretchFactor_;

  // Check
  // assert(maxExtent_ > 0);
  // assert(denseDelta_ > 0);
  // assert(nearSamplePoints_ > 0);
  assert(sparseDelta_ > 0);
  assert(sparseDelta_ > 0.000000001);  // Sanity check

  // -----------------------------------------------------------------------------
  // Improve sparsegraph speed
  dualSG->setFastMirrorMode(true);
  dualSG->setHasUnsavedChanges(true);  // because this is not automatically enabled when in fast mirror mode
  BOLT_ASSERT(dualSG->getSparseCriteria() == false, "SparseCriteria should not be initialized so that "
                                                    "disjoint sets is not used");

  // Visualize
  if (visualizeMiroring_)
  {
    visual_->viz1()->deleteAllMarkers();
    visual_->viz1()->trigger();
  }

  // -----------------------------------------------------------------------------
  // Insert mono SparseGraph into dual SparseGraph

  // Map from every sparseV1 vertex to every vertex in the new graph that has the same roots
  std::vector<std::vector<SparseVertex>> vertexMapMatrix(monoSG_->getNumVertices());

  // Allocate memory for copying the mirrored version of state2
  base::State *state2Mirrored = leftArmSpaceInfo->getStateSpace()->allocState();

  // Loop through every vertex in sparse graph
  BOLT_DEBUG(indent, true, "Adding " << monoSG_->getNumVertices() << " sparse graph vertices into dual arm "
                                                                     "graph");
  std::size_t showEvery = std::max((unsigned int)(1), monoSG_->getNumVertices() / 100);
  foreach (SparseVertex sparseV1, boost::vertices(monoSG_->getGraph()))
  {
    // The first thread number of verticies are used for queries and should be skipped
    if (sparseV1 < monoSG_->getNumQueryVertices())
      continue;

    BOLT_INFO(indent, (sparseV1 % showEvery == 0) || true,
              "Mirroring graph progress: " << (double(sparseV1) / monoSG_->getNumVertices() * 100.0)
                                           << "%. Dual graph verticies: " << dualSG->getNumVertices()
                                           << ", Dual graph edges: " << dualSG->getNumEdges());

    const base::State *state1 = monoSG_->getState(sparseV1);
    if (visualizeMiroring_)
      visual_->viz1()->state(state1, tools::ROBOT, tools::DEFAULT, 0);

    // Record a mapping from the old vertex index to the new index
    std::vector<SparseVertex> sparseV2ToDualVertex(monoSG_->getNumVertices(), /*initial value*/ 0);

    // Loop through every vertex again
    std::size_t skippedStates = 0;
    foreach (SparseVertex sparseV2, boost::vertices(monoSG_->getGraph()))
    {
      // The first thread number of verticies are used for queries and should be skipped
      if (sparseV2 < monoSG_->getNumQueryVertices())
        continue;

      const base::State *state2 = monoSG_->getState(sparseV2);
      mirrorState(state2, state2Mirrored, indent);

      if (visualizeMiroring_)
        visual_->viz2()->state(state2Mirrored, tools::ROBOT, tools::DEFAULT, /*extraData*/ 0, leftArmSpaceInfo);

      // Create dual state via callback
      base::State *stateCombined = combineStatesCallback_(state1, state2Mirrored);

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
      SparseVertex vertexCombined = dualSG->addVertex(stateCombined, DISCRETIZED, indent);

      // Short term mapping
      sparseV2ToDualVertex[sparseV2] = vertexCombined;
    }
    BOLT_DEBUG(indent, vMirrorStatus_,
               "Total states skipped: " << (double(skippedStates) / monoSG_->getNumVertices() * 100) << "%. Skipped "
                                        << skippedStates << " states out of total " << monoSG_->getNumVertices());
    // visual_->waitForUserFeedback("before adding edges");

    // Copy the short term mapping into the long term mapping
    vertexMapMatrix[sparseV1] = sparseV2ToDualVertex;

    // Loop through every edge in sparse graph and connect this subgraph
    addEdgesForDim(sparseV2ToDualVertex, dualSG, dualSpaceInfo, indent);

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
  // dualSG->save(indent);

  // Add all cross edges
  addEdgesForAll(vertexMapMatrix, dualSG, dualSpaceInfo, indent);

  // Save graph
  dualSG->save(indent);
}

void SparseMirror::addEdgesForDim(std::vector<SparseVertex> &sparseV2ToDualVertex, SparseGraphPtr &dualSG,
                                  base::SpaceInformationPtr dualSpaceInfo, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "addEdgesForDim");

  // Loop through every edge in sparse graph and connect this subgraph
  std::size_t skippedUnconnectedEdges = 0;
  std::size_t skippedCollisionEdges = 0;
  std::size_t skippedTooLongEdges = 0;
  foreach (const SparseEdge sparseE, boost::edges(monoSG_->getGraph()))
  {
    const SparseVertex sparseE_v0 = sparseV2ToDualVertex[boost::source(sparseE, monoSG_->getGraph())];
    const SparseVertex sparseE_v1 = sparseV2ToDualVertex[boost::target(sparseE, monoSG_->getGraph())];
    if (sparseE_v0 == 0 || sparseE_v1 == 0)
    {
      skippedUnconnectedEdges++;
      continue;
    }

    // Check if edge is too long
    double newEdgeDist = monoSI_->distance(dualSG->getState(sparseE_v0), dualSG->getState(sparseE_v1));
    if (newEdgeDist > 2 * sparseDelta_)
    {
      BOLT_WARN(indent, true, "Edge is longer than 2*sparseDelta=" << 2 * sparseDelta_ << ", value=" << newEdgeDist);
      visual_->waitForUserFeedback("edge is too long");
      skippedTooLongEdges++;
      continue;
    }

    // Check if edge is in collision
    if (collisionCheckMirror_ &&
        !dualSpaceInfo->checkMotion(dualSG->getState(sparseE_v0), dualSG->getState(sparseE_v1)))
    {
      BOLT_DEBUG(indent, vMirror_, "found collision combined state");

      if (vMirror_)
      {
        visual_->viz3()->state(dualSG->getState(sparseE_v0), tools::ROBOT, tools::RED, /*extraData*/ 0, dualSpaceInfo);
        usleep(0.001 * 1000000);
        visual_->waitForUserFeedback("found collision combined state");
      }

      skippedCollisionEdges++;
      continue;
    }

    // Create edge
    // dualSG->visualizeSparseGraph_ = true;
    // dualSG->visualizeSparseGraphSpeed_ = 0.001;
    // dualSG->visualizeDatabaseVertices_ = false;
    // dualSG->visualizeDatabaseEdges_ = true;
    dualSG->addEdge(sparseE_v0, sparseE_v1, newEdgeDist, eUNKNOWN, indent);
  }

  BOLT_DEBUG(indent, vMirrorStatus_, "Total edges skipped: " << (double(skippedUnconnectedEdges + skippedCollisionEdges) /
                                                                 monoSG_->getNumEdges() * 100)
                                                             << "%. Skipped edges: " << skippedUnconnectedEdges
                                                             << " no endpoints, " << skippedCollisionEdges
                                                             << " collision, " << skippedTooLongEdges << " length, "
                                                             << "out of total: " << monoSG_->getNumEdges());
}

void SparseMirror::addEdgesForAll(std::vector<std::vector<SparseVertex>> &vertexMapMatrix, SparseGraphPtr dualSG,
                                  base::SpaceInformationPtr dualSpaceInfo, std::size_t indent)
{
  BOLT_FUNC(indent, true, "addEdgesForAll()");

  // Add all edges across each dimension
  // Loop through every edge in sparse graph and connect this subgraph
  std::size_t skippedDuplicateEdges = 0;
  std::size_t skippedTooLongEdges = 0;
  std::size_t skippedAstarPath = 0;
  std::size_t skippedCollisionEdges = 0;
  std::size_t skippedUnconnectedEdges = 0;
  std::size_t addedPath = 0;
  std::size_t count = 1;
  const std::size_t showEvery = std::max((unsigned int)(1), monoSG_->getNumEdges() / 100);

  // Cache graph size before we begin growing it
  std::size_t numDualVertices = dualSG->getNumVertices();

  foreach (const SparseEdge sparseE, boost::edges(monoSG_->getGraph()))
  {
    const SparseVertex sparseE_v0 = boost::source(sparseE, monoSG_->getGraph());
    const SparseVertex sparseE_v1 = boost::target(sparseE, monoSG_->getGraph());

    // Duplicate this edge for all verticies derived from this pair of vertices
    for (const SparseVertex dualVertex0 : vertexMapMatrix[sparseE_v0])
    {
      BOLT_ASSERT(dualVertex0 < numDualVertices, "dualVertex0 out of range");

      // if (dualVertex0 == 0)
      // {
      //   skippedUnconnectedEdges++;
      //   continue;
      // }

      // The first thread number of verticies are used for queries and should be skipped
      if (dualVertex0 < dualSG->getNumQueryVertices())
        continue;

      for (const SparseVertex dualVertex1 : vertexMapMatrix[sparseE_v1])
      {
        BOLT_ASSERT(dualVertex1 < numDualVertices, "dualVertex1 out of range");

        // if (dualVertex1 == 0)
        // {
        //   skippedUnconnectedEdges++;
        //   continue;
        // }

        // The first thread number of verticies are used for queries and should be skipped
        if (dualVertex1 < dualSG->getNumQueryVertices())
          continue;

        // Check duplicate
        // time::point startTime0 = time::now(); // Benchmark
        if (dualSG->hasEdge(dualVertex0, dualVertex1))
        {
          skippedDuplicateEdges++;
          continue;
        }
        // OMPL_INFORM("duplicate took %f seconds", time::seconds(time::now() - startTime0)); // Benchmark

        // Check if edge is too long
        // time::point startTime1 = time::now(); // Benchmark
        double newEdgeDist = monoSI_->distance(dualSG->getState(dualVertex0), dualSG->getState(dualVertex1));
        if (newEdgeDist > 2 * sparseDelta_)
        {
          // BOLT_WARN(indent, true, "Edge is longer than 2*sparseDelta=" << 2 * sparseDelta_ << ", value=" <<
          // newEdgeDist);
          skippedTooLongEdges++;
          continue;
        }
        // OMPL_INFORM("length    took %f seconds", time::seconds(time::now() - startTime1)); // Benchmark

        // Check if there already is a path of equal distance in graph
        // time::point startTime2 = time::now(); // Benchmark
        // if (!dualSG->checkPathLength(dualVertex0, dualVertex1, newEdgeDist, indent))
        // {
        //   //BOLT_WARN(indent, true, "There is already a path through graph the same length or shorter");
        //   skippedAstarPath++;
        //   continue;
        // }
        // //OMPL_INFORM("astar     took %f seconds", time::seconds(time::now() - startTime2)); // Benchmark

        // Check if edge is in collision
        // time::point startTime3 = time::now(); // Benchmark
        if (collisionCheckMirror_ &&
            !dualSpaceInfo->checkMotion(dualSG->getState(dualVertex0), dualSG->getState(dualVertex1)))
        {
          skippedCollisionEdges++;
          continue;
        }
        // OMPL_INFORM("collision took %f seconds", time::seconds(time::now() - startTime3)); // Benchmark

        // Create edge
        dualSG->addEdge(dualVertex0, dualVertex1, newEdgeDist, eUNKNOWN, indent);
        addedPath++;
      }

      if (visual_->viz1()->shutdownRequested())
        break;
    }

    // clang-format off
    BOLT_DEBUG(indent, (count % showEvery == 0),
               "Mirror edge " << count << " of " << monoSG_->getNumEdges() << " ("
               << int(ceil(double(count) / monoSG_->getNumEdges() * 100)) << "%). Skipped -"
               << " dup: " << skippedDuplicateEdges
               << ", unconnected: " << skippedUnconnectedEdges
               << ", length: " << skippedTooLongEdges
               << ", astar: " << skippedAstarPath
               << ", collision: " << skippedCollisionEdges
               << ", ADDED: " << addedPath
               << ", Total edge: " << dualSG->getNumEdges());
    // clang-format on

    count++;  // Status

    if (visual_->viz1()->shutdownRequested())
      break;
  }

  BOLT_DEBUG(indent, vMirrorStatus_, "Skipped " << skippedDuplicateEdges << " edges because duplicate, "
                                                << skippedCollisionEdges
                                                << " due to collision, out of total: " << monoSG_->getNumEdges());
}

// TODO: convert this function into a callback that is application specific
void SparseMirror::mirrorState(const base::State *source, base::State *dest, std::size_t indent)
{
  const std::vector<ompl::base::StateSpace::ValueLocation> &valueLocations =
      monoSI_->getStateSpace()->getValueLocations();

  // Loop through each value
  for (std::size_t i = 0; i < valueLocations.size(); ++i)
  {
    double jointValue = *monoSI_->getStateSpace()->getValueAddressAtLocation(source, valueLocations[i]);

    BOLT_DEBUG(indent, vMirror_, "jointValue: " << jointValue);
    double newJointValue;

    // TODO: this is specific to Baxter, NOT robot agnostic
    if (i % 2 == 0)
    {
      // Get bounds
      const ompl::base::RealVectorBounds &bounds = monoSI_->getStateSpace()->getBounds();

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

    *monoSI_->getStateSpace()->getValueAddressAtLocation(dest, valueLocations[i]) = newJointValue;
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
  BOLT_ASSERT(dualSpaceInfo->getStateSpace()->getDimension() == monoSI_->getStateSpace()->getDimension() * 2,
              "Number of dimensions in dual space info should be double the mono space info");

  // -----------------------------------------------------------------------------
  // Create SparseGraph for dual arm
  SparseGraphPtr dualSG = SparseGraphPtr(new SparseGraph(dualSpaceInfo, visual_));

  visual_->viz1()->deleteAllMarkers();
  visual_->viz1()->trigger();

  // Allocate memory for copying the mirrored version of state2
  base::State *state2Mirrored = leftArmSpaceInfo->getStateSpace()->allocState();

  // Loop through every vertex in sparse graph and check if state is valid on opposite arm
  BOLT_DEBUG(indent, true, "Checking " << monoSG_->getNumVertices() << " sparse graph vertices for validity on "
                                                                       "opposite arm");
  const std::size_t showEvery = std::max((unsigned int)(1), monoSG_->getNumVertices() / 100);
  std::size_t skippedStates = 0;
  foreach (SparseVertex sparseV1, boost::vertices(monoSG_->getGraph()))
  {
    // The first thread number of verticies are used for queries and should be skipped
    if (sparseV1 < monoSG_->getNumQueryVertices())
      continue;

    BOLT_INFO(indent, (sparseV1 % showEvery == 0),
              "Mirroring graph progress: " << (double(sparseV1) / monoSG_->getNumVertices() * 100.0) << "%");

    const base::State *state1 = monoSG_->getState(sparseV1);
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
  BOLT_DEBUG(indent, true, "Skipped " << skippedStates << " states out of total " << monoSG_->getNumRealVertices());

  // Free memory
  leftArmSpaceInfo->getStateSpace()->freeState(state2Mirrored);
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
