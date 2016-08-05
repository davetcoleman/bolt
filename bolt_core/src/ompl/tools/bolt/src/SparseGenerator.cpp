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
   Desc:   Populates sparse graph with sparse criteria using both discretization and random sampling
*/

// OMPL
#include <ompl/tools/bolt/SparseGenerator.h>
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
SparseGenerator::SparseGenerator(SparseGraphPtr sg)
  : sg_(sg), si_(sg_->getSpaceInformation()), visual_(sg_->getVisual())
{
  // Initialize discretizer
  vertexDiscretizer_.reset(new VertexDiscretizer(sg_));
}

SparseGenerator::~SparseGenerator(void)
{
  clearanceSampler_.reset();
}

bool SparseGenerator::setup(std::size_t indent)
{
  // Load minimum clearance state sampler
  // TODO: remove this if we stick to samplingQueue
  clearanceSampler_ = ob::MinimumClearanceValidStateSamplerPtr(new ob::MinimumClearanceValidStateSampler(si_.get()));
  clearanceSampler_->setMinimumObstacleClearance(sg_->getObstacleClearance());
  si_->getStateValidityChecker()->setClearanceSearchDistance(sg_->getObstacleClearance());

  // Speed up random sampling with these threads
  samplingQueue_.reset(new SamplingQueue(sg_));
  candidateQueue_.reset(new CandidateQueue(sg_, samplingQueue_, shared_from_this()));

  // Configure vertex discretizer
  vertexDiscretizer_->setMinimumObstacleClearance(sg_->getObstacleClearance());
  vertexDiscretizer_->setDiscretization(sparseCriteria_->getDiscretization());

  return true;
}

void SparseGenerator::createSPARS()
{
  std::size_t indent = 0;
  BOLT_FUNC(indent, true, "createSPARS()");

  // Error check
  if (!useRandomSamples_ && !useDiscretizedSamples_)
  {
    OMPL_WARN("Unable to create SPARS because both random sampling and discretized sampling is disabled");
    return;
  }

  // Reset stats
  sparseCriteria_->resetStats();
  numConsecutiveFailures_ = 0;
  sparseCriteria_->setUseFourthCriteria(false);  // initially we do not do this step

  // Benchmark runtime
  timeDiscretizeAndRandomStarted_ = time::now();

  // Profiler
  CALLGRIND_TOGGLE_COLLECT;

  // Start the graph off with discretized states
  if (useDiscretizedSamples_)
  {
    BOLT_INFO(indent, true, "Adding discretized states");
    addDiscretizedStates(indent);
  }

  // Only display database if enabled
  // if (sg_->visualizeSparseGraph_ && sg_->visualizeSparseGraphSpeed_ > std::numeric_limits<double>::epsilon())
  //   sg_->displayDatabase(true, indent);

  // Finish the graph with random samples
  if (useRandomSamples_)
  {
    BOLT_INFO(indent, true, "Adding random samples states");
    // addRandomSamples(indent);
    // addRandomSamplesOneThread(indent);
    addRandomSamplesTwoThread(indent);
  }

  // Profiler
  CALLGRIND_TOGGLE_COLLECT;
  CALLGRIND_DUMP_STATS;

  // Cleanup removed vertices
  sg_->removeDeletedVertices(indent);

  // Benchmark runtime
  double duration = time::seconds(time::now() - timeDiscretizeAndRandomStarted_);

  // Check how many connected components exist
  std::size_t numSets = sg_->getDisjointSetsCount();
  std::pair<std::size_t, std::size_t> interfaceStats = sparseCriteria_->getInterfaceStateStorageSize();

  // Find min, max, and average edge length
  double totalEdgeLength = 0;
  double maxEdgeLength = -1 * std::numeric_limits<double>::infinity();
  double minEdgeLength = std::numeric_limits<double>::infinity();
  foreach (const SparseEdge e, boost::edges(sg_->getGraph()))
  {
    const double length = sg_->getEdgeWeightProperty(e);
    // std::cout << "Edge " << e << " length: " << length << std::endl;
    totalEdgeLength += length;
    if (maxEdgeLength < length)
      maxEdgeLength = length;
    if (minEdgeLength > length)
      minEdgeLength = length;
  }
  double averageEdgeLength = sg_->getNumEdges() ? totalEdgeLength / sg_->getNumEdges() : 0;

  BOLT_INFO(indent, 1, "-----------------------------------------");
  BOLT_INFO(indent, 1, "Created SPARS graph                      ");
  BOLT_INFO(indent, 1, "  Vertices:                  " << sg_->getNumRealVertices());
  BOLT_INFO(indent, 1, "  Edges:                     " << sg_->getNumEdges());
  BOLT_INFO(indent, 1, "  Generation time:           " << duration);
  BOLT_INFO(indent, 1, "  Disjoint sets:             " << numSets);
  BOLT_INFO(indent, 1, "  Sparse Criteria:           ");
  BOLT_INFO(indent, 1, "     SparseDelta:            " << sparseCriteria_->getSparseDelta());
  BOLT_INFO(indent, 1, "     Stretch Factor:         " << sparseCriteria_->getStretchFactor());
  BOLT_INFO(indent, 1, "     Discretization:         " << sparseCriteria_->getDiscretization());
  BOLT_INFO(indent, 1, "  Edge Lengths:              ");
  BOLT_INFO(indent, 1, "     Max:                    " << maxEdgeLength);
  BOLT_INFO(indent, 1, "     Min:                    " << minEdgeLength);
  BOLT_INFO(indent, 1, "     Average:                " << averageEdgeLength);
  BOLT_INFO(indent, 1, "     Difference:             " << averageEdgeLength - sparseCriteria_->getSparseDelta());
  BOLT_INFO(indent, 1, "     Penetration:            " << sparseCriteria_->getDiscretizePenetrationDist());
  BOLT_INFO(indent, 1, "  Criteria additions:        ");
  BOLT_INFO(indent, 1, "    Coverage:                " << sg_->numSamplesAddedForCoverage_);
  BOLT_INFO(indent, 1, "    Connectivity:            " << sg_->numSamplesAddedForConnectivity_);
  BOLT_INFO(indent, 1, "    Interface:               " << sg_->numSamplesAddedForInterface_);
  BOLT_INFO(indent, 1, "    Quality:                 " << sg_->numSamplesAddedForQuality_);
  BOLT_INFO(indent, 1, "  Num random samples added:  " << numRandSamplesAdded_);
  BOLT_INFO(indent, 1, "  Num vertices moved:        " << sparseCriteria_->getNumVerticesMoved());
  BOLT_INFO(indent, 1, "  CandidateQueue Misses:     " << candidateQueue_->getTotalMisses());
  BOLT_INFO(indent, 1, "  InterfaceData:             ");
  BOLT_INFO(indent, 1, "    States stored:           " << interfaceStats.first);
  BOLT_INFO(indent, 1, "    Missing interfaces:      " << interfaceStats.second);
  BOLT_INFO(indent, 1, "-----------------------------------------");

  // Copy-paste data
  copyPasteState(numSets);

  if (!sg_->verifyGraph(indent))
  {
    OMPL_ERROR("Sparse graph did not pass test");
  }

  if (!sg_->visualizeSparseGraph_)
    sg_->displayDatabase(true, indent);

  OMPL_INFORM("Finished creating sparse database");
}

void SparseGenerator::copyPasteState(std::size_t numSets)
{
  double duration = time::seconds(time::now() - timeDiscretizeAndRandomStarted_);

  // clang-format off
  std::cout << "=SPLIT(\"Bolt, "
            << sparseCriteria_->sparseDeltaFraction_ << ", "
            << sparseCriteria_->getSparseDelta() << ", "
            << sparseCriteria_->getDiscretization() << ", "
            << sparseCriteria_->getStretchFactor() << ", "
            << sparseCriteria_->getNearSamplePointsMultiple() << ", "
            << useDiscretizedSamples_ << ", "
            << useRandomSamples_ << ", "
            << sparseCriteria_->useCheckRemoveCloseVertices_ << ", "
            << sparseCriteria_->useClearEdgesNearVertex_ << ", "
            << sparseCriteria_->useOriginalSmoother_ << ", "
            << sparseCriteria_->useEdgeImprovementRule_ << ", "
            << fourthCriteriaAfterFailures_ << ", "
            << terminateAfterFailures_ << ", "
            << maxConsecutiveFailures_ << ", "
            << sg_->getNumRealVertices() << ", "
            << sg_->getNumEdges() << ", "
            << numSets << ", "
            << duration << "\", \",\")" << std::endl;
  // clang-format on
}

void SparseGenerator::addDiscretizedStates(std::size_t indent)
{
  BOLT_FUNC(indent, true, "addDiscretizedStates()");
  bool vAdd = sg_->vAdd_;
  sg_->vAdd_ = false;  // only do this for random sampling

  // This only runs if the graph is empty
  if (!sg_->isEmpty())
  {
    BOLT_WARN(indent, true, "Unable to generate discretized states because graph is not empty");
    return;
  }

  // Generate discretization
  vertexDiscretizer_->generateGrid(indent);

  // Make sure discretization doesn't have any bugs
  if (sg_->superDebug_)
    sg_->errorCheckDuplicateStates(indent);

  sg_->vAdd_ = vAdd;  // reset value
}

bool SparseGenerator::addRandomSamples(std::size_t indent)
{
  BOLT_FUNC(indent, true, "addRandomSamples()");

  // Clear stats
  numRandSamplesAdded_ = 0;
  timeRandSamplesStarted_ = time::now();
  maxConsecutiveFailures_ = 0;
  maxPercentComplete_ = 0;

  base::State *candidateState = si_->allocState();
  const std::size_t threadID = 0;

  while (true)
  {
    // Sample randomly
    if (!clearanceSampler_->sample(candidateState))
    {
      OMPL_ERROR("Unable to find valid sample");
      exit(-1);  // this should never happen
    }

    // Debug
    if (false)
    {
      BOLT_DEBUG(indent, true, "Randomly sampled state: " << candidateState);
      sg_->debugState(candidateState);
    }

    // Find nearby nodes
    CandidateData candidateD(candidateState);
    findGraphNeighbors(candidateD, threadID, indent);

    bool usedState = false;
    if (!addSample(candidateD, threadID, usedState, indent))
    {
      return true;  // no more states needed
    }

    if (usedState)
    {
      // State was used, so allocate new state
      candidateState = si_->allocState();
    }
  }  // while(true) create random sample

  return true;  // program should never reach here
}

bool SparseGenerator::addRandomSamplesOneThread(std::size_t indent)
{
  BOLT_FUNC(indent, true, "addRandomSamplesOneThread()");

  // Clear stats
  numRandSamplesAdded_ = 0;
  timeRandSamplesStarted_ = time::now();
  maxConsecutiveFailures_ = 0;
  maxPercentComplete_ = 0;

  samplingQueue_->startSampling(indent);

  base::State *candidateState;
  const std::size_t threadID = 0;

  while (true)
  {
    samplingQueue_->getNextState(candidateState, indent);

    // Find nearby nodes
    CandidateData candidateD(candidateState);
    findGraphNeighbors(candidateD, threadID, indent);

    bool usedState = false;
    if (!addSample(candidateD, threadID, usedState, indent))
    {
      samplingQueue_->stopSampling(indent);
      return true;  // no more states needed
    }

    // Tell other thread whether the state should be re-used
    // samplingQueue_->setNextStateUsed(usedState);
  }  // while(true) create random sample

  return true;  // program should never reach here
}

bool SparseGenerator::addRandomSamplesTwoThread(std::size_t indent)
{
  BOLT_FUNC(indent, true, "addRandomSamplesTwoThread()");

  // Clear stats
  numRandSamplesAdded_ = 0;
  timeRandSamplesStarted_ = time::now();
  maxConsecutiveFailures_ = 0;
  maxPercentComplete_ = 0;

  samplingQueue_->startSampling(indent);

  candidateQueue_->startGenerating(indent);

  const std::size_t threadID = 0;
  while (!visual_->viz1()->shutdownRequested())
  {
    // time::point startTime2 = time::now(); // Benchmark

    // Find nearby nodes
    CandidateData &candidateD = candidateQueue_->getNextCandidate(indent);

    bool usedState = false;

    // time::point startTime = time::now(); // Benchmark
    bool result = addSample(candidateD, threadID, usedState, indent);
    // BOLT_GREEN_DEBUG(0, 1, time::seconds(time::now() - startTime) << " add sample"); // Benchmark

    if (!result)
    {
      samplingQueue_->stopSampling(indent);
      candidateQueue_->stopGenerating(indent);

      return true;  // no more states needed
    }

    // BOLT_DEBUG(indent, true, "SparseGenerator: used: " << usedState << " numRandSamplesAdded: " <<
    // numRandSamplesAdded_);

    // Tell other thread whether the candidate was used
    candidateQueue_->setCandidateUsed(usedState, indent);

    // BOLT_DEBUG(0, 1, time::seconds(time::now() - startTime2) << " whole sampling loop"); // Benchmark
  }  // while(true) create random sample

  return true;  // program should never reach here
}

bool SparseGenerator::addSample(CandidateData &candidateD, std::size_t threadID, bool &usedState, std::size_t indent)
{
  BOLT_FUNC(indent, false, "addSample() threadID: " << threadID);

  // Run SPARS checks
  VertexType addReason;  // returns why the state was added
  if (sparseCriteria_->addStateToRoadmap(candidateD, addReason, threadID, indent))
  {
    // State was added
    numConsecutiveFailures_ = 0;

    // Save on interval of new state addition
    if ((numRandSamplesAdded_ + 1) % saveInterval_ == 0)
    {
      sg_->saveIfChanged(indent);
      double duration = time::seconds(time::now() - timeRandSamplesStarted_);
      BOLT_DEBUG(indent, true, "Adding samples at rate: " << numRandSamplesAdded_ / duration << " hz");
      copyPasteState();
    }

    // Increment statistics
    numRandSamplesAdded_++;

    usedState = true;

    // Check if shutdown requested
    if (visual_->viz1()->shutdownRequested())
    {
      BOLT_INFO(indent, true, "Shutdown requested");
      sg_->saveIfChanged(indent);
      exit(0);
    }
  }
  else
  {
    // State was not added
    numConsecutiveFailures_++;

    if (numConsecutiveFailures_ > maxConsecutiveFailures_)
    {
      maxConsecutiveFailures_ = numConsecutiveFailures_;

      std::size_t percentComplete;
      if (!sparseCriteria_->getUseFourthCriteria())
        percentComplete = ceil(maxConsecutiveFailures_ / double(fourthCriteriaAfterFailures_) / 2.0 * 100.0);
      else
        percentComplete = ceil(maxConsecutiveFailures_ / double(terminateAfterFailures_) * 100.0);

      // Every time the whole number of percent compelete changes, show to user
      if (percentComplete > maxPercentComplete_)
      {
        maxPercentComplete_ = percentComplete;

        // Show varying granularity based on number of dimensions
        static const std::size_t showEvery = std::max(1, int(12 - si_->getStateDimension() * 2));
        if (percentComplete % showEvery == 0)
        {
          BOLT_WARN(indent, true, "Termination progress: " << percentComplete << "%");
          copyPasteState();
        }
      }
    }
    // if (numConsecutiveFailures_ % 500 == 0)
    // {
    //   BOLT_INFO(indent, true, "Random sample failed, consecutive failures: " << numConsecutiveFailures_);
    // }
  }

  // Check consecutive failures to determine if quality criteria needs to be enabled
  if (!sparseCriteria_->getUseFourthCriteria() && numConsecutiveFailures_ >= fourthCriteriaAfterFailures_)
  {
    BOLT_INFO(indent, true, "Starting to check for 4th quality criteria because "
                                << numConsecutiveFailures_ << " consecutive failures have occured");
    bool disableFourth = false;
    if (disableFourth)
    {
      OMPL_WARN("Disabled use fourth criteria");
      return false;  // stop inserting states
    }
    else
      sparseCriteria_->setUseFourthCriteria(true);

    numConsecutiveFailures_ = 0;  // reset for new criteria
    maxConsecutiveFailures_ = 0;  // reset for new criteria

    // Show it just once if it has not already been animated
    if (!sg_->visualizeVoronoiDiagramAnimated_ && sg_->visualizeVoronoiDiagram_)
      visual_->vizVoronoiDiagram();
  }

  // Check consequitive failures to determine termination
  if (sparseCriteria_->getUseFourthCriteria() && numConsecutiveFailures_ > terminateAfterFailures_)
  {
    BOLT_WARN(indent, true, "SPARS creation finished because " << terminateAfterFailures_ << " consecutive insertion "
                                                                                             "failures reached");
    return false;  // stop inserting states
  }

  return true;
}

void SparseGenerator::findGraphNeighbors(CandidateData &candidateD, std::size_t threadID, std::size_t indent)
{
  BOLT_FUNC(indent, true, "findGraphNeighbors() within sparse delta " << sparseCriteria_->getSparseDelta());
  const bool verbose = false;

  // Search in thread-safe manner
  // Note that the main thread could be modifying the NN, so we have to lock it
  sg_->getQueryStateNonConst(threadID) = candidateD.state_;
  sg_->getNN()->nearestR(sg_->getQueryVertices(threadID), sparseCriteria_->getSparseDelta(),
                         candidateD.graphNeighborhood_);
  sg_->getQueryStateNonConst(threadID) = nullptr;

  // Now that we got the neighbors from the NN, we must remove any we can't see
  for (std::size_t i = 0; i < candidateD.graphNeighborhood_.size(); ++i)
  {
    SparseVertex v2 = candidateD.graphNeighborhood_[i];

    // Don't collision check if they are the same state
    if (candidateD.state_ != sg_->getState(v2))
    {
      if (!si_->checkMotion(candidateD.state_, sg_->getState(v2)))
      {
        continue;
      }
    }
    else if (verbose)
      std::cout << " ---- Skipping collision checking because same vertex " << std::endl;

    // The two are visible to each other!
    candidateD.visibleNeighborhood_.push_back(candidateD.graphNeighborhood_[i]);
  }

  BOLT_DEBUG(indent, true, "Graph neighborhood: " << candidateD.graphNeighborhood_.size() << " | Visible neighborhood: "
                                                  << candidateD.visibleNeighborhood_.size());
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
