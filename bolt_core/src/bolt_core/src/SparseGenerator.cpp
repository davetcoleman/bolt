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
#include <bolt_core/SparseGenerator.h>
#include <bolt_core/SparseCriteria.h>
#include <ompl/base/DiscreteMotionValidator.h>

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

void SparseGenerator::clear()
{
  numRandSamplesAdded_ = 0;
  numConsecutiveFailures_ = 0;
  maxConsecutiveFailures_ = 0;
  maxPercentComplete_ = 0;

  samplingQueue_->clear();
  candidateQueue_->clear();
}

bool SparseGenerator::setup(std::size_t indent)
{
  // Load minimum clearance state sampler
  clearanceSampler_ = ClearanceSamplerPtr(new ob::MinimumClearanceValidStateSampler(si_.get()));
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
  BOLT_FUNC(indent, verbose_ || true, "createSPARS()");

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
    BOLT_INFO(indent, verbose_, "Adding discretized states");
    addDiscretizedStates(indent);
  }

  // Only display database if enabled
  // if (sg_->visualizeSparseGraph_ && sg_->visualizeSparseGraphSpeed_ > std::numeric_limits<double>::epsilon())
  // sg_->displayDatabase(true, true, 1, indent);

  // Finish the graph with random samples
  if (useRandomSamples_)
  {
    BOLT_INFO(indent, true, "Adding random samples states");
    addRandomSamplesOneThread(indent);
    //addRandomSamplesThreaded(indent);
  }

  // Profiler
  CALLGRIND_TOGGLE_COLLECT;
  CALLGRIND_DUMP_STATS;

  // Save graph - this also calls removeDeletedVertices();
  sg_->saveIfChanged(indent);

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
    sg_->displayDatabase(true, true, 1, indent);

  // Ensure the graph is valid
  checkSparseGraphOptimality(indent);

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
  BOLT_FUNC(indent, verbose_, "addDiscretizedStates()");
  bool vAdd = sg_->vAdd_;
  sg_->vAdd_ = false;  // only do this for random sampling

  // This only runs if the graph is empty
  if (!sg_->isEmpty())
  {
    BOLT_WARN(indent, verbose_, "Unable to generate discretized states because graph is not empty");
    return;
  }

  // Generate discretization
  vertexDiscretizer_->generateGrid(indent);

  // Make sure discretization doesn't have any bugs
  if (sg_->superDebug_)
    sg_->errorCheckDuplicateStates(indent);

  sg_->vAdd_ = vAdd;  // reset value
}

bool SparseGenerator::addRandomSamplesOneThread(std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "addRandomSamplesOneThread()");

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
      BOLT_DEBUG(indent, verbose_, "Randomly sampled state: " << candidateState);
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

bool SparseGenerator::addRandomSamplesThreaded(std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "addRandomSamplesThreaded()");

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
    // BOLT_GREEN(0, 1, time::seconds(time::now() - startTime) << " add sample"); // Benchmark

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
  BOLT_FUNC(indent, verbose_, "addSample() threadID: " << threadID);

  // Run SPARS checks
  VertexType addReason;  // returns why the state was added
  if (sparseCriteria_->addStateToRoadmap(candidateD, addReason, threadID, indent))
  {
    // State was added
    numConsecutiveFailures_ = 0;

    // Save on interval of new state addition
    if ((numRandSamplesAdded_ + 1) % saveInterval_ == 0)
    {
      stopCandidateQueueAndSave(indent);

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
      stopCandidateQueueAndSave(indent);
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
      {
        percentComplete = ceil(maxConsecutiveFailures_ / double(fourthCriteriaAfterFailures_) * 100.0);
      }
      else
      {
        percentComplete = ceil(maxConsecutiveFailures_ / double(terminateAfterFailures_) * 100.0);
      }

      // Every time the whole number of percent compelete changes, show to user
      if (percentComplete > maxPercentComplete_)
      {
        maxPercentComplete_ = percentComplete;

        // Show varying granularity based on number of dimensions
        static const std::size_t showEvery = std::max(1, int(12 - si_->getStateDimension() * 2));
        if (percentComplete % showEvery == 0)
        {
          if (!sparseCriteria_->getUseFourthCriteria())
          {
            BOLT_WARN(indent, true, "Discretization progress: " << percentComplete << "%");
          }
          else
          {
            BOLT_GREEN(indent, true, "Quality termination progress: " << percentComplete << "%");
          }
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
    if (!sparseCriteria_->useQualityCriteria_)
    {
      BOLT_INFO(0, true, "---------------------------------------------------");
      BOLT_WARN(0, true, "Fourth criteria (quality) is disabled");
      return false;  // stop inserting states
    }

    BOLT_INFO(0, true, "---------------------------------------------------");
    BOLT_INFO(0, true, "Starting to check for 4th quality criteria because "
                                << numConsecutiveFailures_ << " consecutive failures have occured");
    BOLT_INFO(0, true, "");

    sparseCriteria_->setUseFourthCriteria(true);

    maxPercentComplete_ = 0;      // reset for new criteria
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
  findGraphNeighbors(candidateD, sparseCriteria_->getSparseDelta(), threadID, indent);
}

void SparseGenerator::findGraphNeighbors(CandidateData &candidateD, double distance, std::size_t threadID, std::size_t indent)
{
  BOLT_FUNC(indent, vFindGraphNeighbors_, "findGraphNeighbors() within sparse delta " << sparseCriteria_->getSparseDelta());

  // Search in thread-safe manner
  // Note that the main thread could be modifying the NN, so we have to lock it
  sg_->getQueryStateNonConst(threadID) = candidateD.state_;
  sg_->getNN()->nearestR(sg_->getQueryVertices(threadID), distance,
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
    else if (vFindGraphNeighbors_)
      std::cout << " ---- Skipping collision checking because same vertex " << std::endl;

    // The two are visible to each other!
    candidateD.visibleNeighborhood_.push_back(candidateD.graphNeighborhood_[i]);
  }

  BOLT_DEBUG(indent, vFindGraphNeighbors_, "Graph neighborhood: " << candidateD.graphNeighborhood_.size()
                                    << " | Visible neighborhood: " << candidateD.visibleNeighborhood_.size());
}

bool SparseGenerator::checkSparseGraphOptimality(std::size_t indent)
{
  BOLT_FUNC(indent, true, "checkSparseGraphOptimality()");

  std::size_t numTests = 1000;
  std::size_t numFailedPlans = 0;

  // Set the motion validator to use clearance, this way isValid() checks clearance before confirming valid
  base::DiscreteMotionValidator *dmv =
      dynamic_cast<base::DiscreteMotionValidator *>(si_->getMotionValidator().get());
  BOLT_ASSERT(dmv->getRequiredStateClearance() == 0, "Discrete motion validator should have clearance = 0");

  // For each test
  for (std::size_t i = 0; i < numTests; ++i)
  {
    if (visual_->viz1()->shutdownRequested())
      break;

    // Choose random start and goal state that has a nearest neighbor
    std::vector<CandidateData> endPoints(2);

    for (CandidateData &point : endPoints)
    {
      // Allocate
      point.state_ = si_->getStateSpace()->allocState();

      // Sample
      if (!clearanceSampler_->sample(point.state_))
        throw Exception(name_, "No valid sample found");

      // Find nearest neighbor
      findGraphNeighbors(point, 0 /*threadID*/, indent);

      // Check if neighbor exists - should always have at least one neighbor otherwise graph lacks coverage
      if (point.visibleNeighborhood_.empty())
        debugNoNeighbors(point, indent);

      // Check if first state is same as input
      if (si_->getStateSpace()->equalStates(point.state_, sg_->getState(point.visibleNeighborhood_[0])))
        throw Exception(name_, "First neighbor is itself");
    }

    // Astar search through graph
    const base::State *actualStart = endPoints.front().state_;
    const base::State *actualGoal = endPoints.back().state_;
    const SparseVertex start = endPoints.front().visibleNeighborhood_[0];
    const SparseVertex goal = endPoints.back().visibleNeighborhood_[0];

    std::vector<SparseVertex> vertexPath;
    double distance;
    if (!sg_->astarSearch(start, goal, vertexPath, distance, indent))
    {
      BOLT_ERROR(indent, true, "No path found through graph");

      // Clear out previous path for clarity
      visual_->viz2()->deleteAllMarkers();
      visual_->viz2()->trigger();
      usleep(0.001 * 1000000);

      // Visualize actual and snapped states
      visual_->viz3()->deleteAllMarkers();
      visual_->viz3()->state(actualStart, tools::LARGE, tools::RED, 0);
      visual_->viz3()->state(actualGoal, tools::LARGE, tools::GREEN, 0);
      visual_->viz3()->state(sg_->getState(start), tools::LARGE, tools::BLACK, 0);
      visual_->viz3()->state(sg_->getState(goal), tools::LARGE, tools::BLACK, 0);
      visual_->viz3()->trigger();

      exit(0);
      numFailedPlans++;
      continue;
    }

    // If path found (same connected component) sum up total length
    geometric::PathGeometric geometricSolution(si_);
    convertVertexPathToStatePath(vertexPath, actualStart, actualGoal, geometricSolution);

    // Smooth path to find the "optimal" path
    geometric::PathGeometric smoothedPath = geometricSolution;
    geometric::PathGeometric *smoothedPathPtr = &smoothedPath;
    geometric::PathSimplifier pathSimplifier(si_);
    sg_->smoothMax(smoothedPathPtr, indent);

    // Show the two paths
    bool showEveryPath = false;
    if (showEveryPath)
    {
      visual_->viz2()->deleteAllMarkers();
      visual_->viz2()->path(&geometricSolution, tools::SMALL, tools::RED);
      visual_->viz2()->path(smoothedPathPtr, tools::MEDIUM, tools::GREEN);
      visual_->viz2()->trigger();
      usleep(0.001 * 1000000);
    }

    // Calculate theoretical guarantees
    double optimalLength = smoothedPathPtr->length();
    double sparseLength = geometricSolution.length();
    double theoryLength = sparseCriteria_->getStretchFactor() * optimalLength + std::numeric_limits<double>::epsilon(); // + 4 * sparseCriteria_->getSparseDelta();
    double percentOfMaxAllows = sparseLength / theoryLength * 100.0;

    bool show = sparseLength >= theoryLength || vGuarantees_;

    BOLT_DEBUG(indent, show, "-----------------------------------------");
    BOLT_DEBUG(indent, show, "Checking Asymptotic Optimality Guarantees");
    BOLT_DEBUG(indent + 2, show, "Raw Path Length:         " << sparseLength);
    BOLT_DEBUG(indent + 2, show, "Smoothed Path Length:    " << optimalLength);
    BOLT_DEBUG(indent + 2, show, "Smoothed Path States:    " << smoothedPathPtr->getStateCount());
    BOLT_DEBUG(indent + 2, show, "Theoretical Path Length: " << theoryLength);
    BOLT_DEBUG(indent + 2, show, "Stretch Factor t:        " << sparseCriteria_->getStretchFactor());
    BOLT_DEBUG(indent + 2, show, "Sparse Delta:            " << sparseCriteria_->getSparseDelta());

    if (sparseLength >= theoryLength)
    {
      BOLT_ERROR(indent + 2, true, "Asymptotic optimality guarantee VIOLATED");

      // Show the two paths
      visual_->viz2()->deleteAllMarkers();
      visual_->viz2()->path(&geometricSolution, tools::SMALL, tools::RED);
      visual_->viz2()->path(smoothedPathPtr, tools::MEDIUM, tools::GREEN);
      visual_->viz2()->trigger();
      usleep(0.001 * 1000000);

      exit(0);
      return false;
    }
    else
      BOLT_GREEN(indent + 2, show, "Asymptotic optimality guarantee maintained");
    BOLT_WARN(indent + 2, show, "Percent of max allowed:  " << percentOfMaxAllows << " %");
    BOLT_DEBUG(indent, show, "-----------------------------------------");
  }

  // Summary
  BOLT_DEBUG(indent, 1, "-----------------------------------------");
  BOLT_DEBUG(indent, 1, "Checking Asymptotic Optimality Guarantees");
  BOLT_DEBUG(indent + 2, 1, "Total tests:               " << numTests);
  BOLT_DEBUG(indent + 2, 1, "Number failed plans:       " << numFailedPlans);
  BOLT_DEBUG(indent, 1, "-----------------------------------------");

  return true;
}

void SparseGenerator::debugNoNeighbors(CandidateData &point, std::size_t indent)
{
  BOLT_ERROR(indent, true, "Found sampled vertex with no neighbors");

  // Clear Window 2
  visual_->viz2()->deleteAllMarkers();
  visual_->viz2()->trigger();

  // Visualize
  visual_->viz3()->deleteAllMarkers();
  visual_->viz3()->state(point.state_, tools::LARGE, tools::RED, 0);

  std::cout << "point.graphNeighborhood_.size() " << point.graphNeighborhood_.size() << std::endl;
  for (auto v : point.graphNeighborhood_)
  {
    std::cout << " - showing nearest neighbor " << v << std::endl;
    visual_->viz3()->state(sg_->getState(v), tools::LARGE, tools::ORANGE, 0);
    visual_->viz3()->edge(sg_->getState(v), point.state_, tools::MEDIUM, tools::BLUE);

    std::cout << "   point.state_: " << point.state_ << std::endl;
    std::cout << "   sg_->getState(v): " << sg_->getState(v) << std::endl;

    if (!si_->getMotionValidator()->checkMotion(point.state_, sg_->getState(v), visual_))
      std::cout << "   in collision " << std::endl;
    else
      std::cout << "   not in collision " << std::endl;

    // Trigger after checkMotion
    visual_->viz2()->state(point.state_, tools::LARGE, tools::GREEN, 0); // show start
    visual_->viz2()->trigger();
  }
  visual_->viz3()->trigger();
  usleep(0.001*1000000);

  // Find nearest neighbor - SECOND TRY
  double dist = sparseCriteria_->getSparseDelta() * 1.5;
  findGraphNeighbors(point, dist, 0 /*threadID*/, indent);
  if (point.visibleNeighborhood_.empty())
    std::cout << "still empty " << std::endl;
  else
    std::cout << "not empty anymore! " << std::endl;

  BOLT_ASSERT(false, "Found sampled vertex with no neighbors");
}

bool SparseGenerator::convertVertexPathToStatePath(std::vector<SparseVertex> &vertexPath,
                                                   const base::State *actualStart, const base::State *actualGoal,
                                                   og::PathGeometric &geometricSolution)
{
  BOLT_ASSERT(!vertexPath.empty(), "Vertex path is empty");

  // Add original start if it is different than the first state
  // if (!si_->getStateSpace()->equalStates(actualStart, sg_->getState(vertexPath.back())))
  // {
  //   geometricSolution.append(actualStart);
  // }

  // Reverse the vertexPath and convert to state path
  for (std::size_t i = vertexPath.size(); i > 0; --i)
  {
    geometricSolution.append(sg_->getState(vertexPath[i - 1]));
  }

  // Add original goal if it is different than the last state
  // if (!si_->getStateSpace()->equalStates(actualGoal, sg_->getState(vertexPath.front())))
  // {
  //   geometricSolution.append(actualGoal);
  // }

  return true;
}

void SparseGenerator::stopCandidateQueueAndSave(std::size_t indent)
{
  BOLT_FUNC(indent, true, "stopCandidateQueueAndSave()");

  if (sg_->hasUnsavedChanges())
  {
    // Stop and reset the candidate queue because it uses the nearest neighbors and will have bad vertices stored
    candidateQueue_->stopGenerating(indent);

    // Save
    sg_->saveIfChanged(indent);

    // Restart the queue
    candidateQueue_->startGenerating(indent);
  }
}

void SparseGenerator::benchmarkRandValidSampling()
{
  std::cout << "-------------------------------------------------------" << std::endl;
  OMPL_INFORM("Running system performance benchmark");
  base::State *candidateState = si_->getStateSpace()->allocState();

  base::StateSamplerPtr sampler;
  sampler = si_->allocStateSampler();

  const std::size_t benchmarkRuns = 10000;
  std::size_t debugIncrement = benchmarkRuns / 10;
  std::size_t validCount = 0;

  // Benchmark runtime
  double totalDurationValid = 0;
  time::point startTimeValid;
  double totalDurationSampler = 0;
  time::point startTimeSampler;
  for (std::size_t i = 0; i < benchmarkRuns; ++i)
  {
    startTimeSampler = time::now();
    sampler->sampleUniform(candidateState);

    startTimeValid = time::now();
    totalDurationSampler += time::seconds(startTimeValid - startTimeSampler);

    validCount += si_->isValid(candidateState);
    totalDurationValid += time::seconds(time::now() - startTimeValid);

    if (i % debugIncrement == 0)
      std::cout << "Progress: " << i / double(benchmarkRuns) * 100.0 << "%" << std::endl;
  }
  // Benchmark runtime
  OMPL_INFORM("  isValid() took %f seconds (%f per run)", totalDurationValid, totalDurationValid / benchmarkRuns);
  OMPL_INFORM("  sampleUniform() took %f seconds (%f per run)", totalDurationSampler,
              totalDurationSampler / benchmarkRuns);
  OMPL_INFORM("  Percent valid: %f", validCount / double(benchmarkRuns) * 100);
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << std::endl;
}

void SparseGenerator::benchmarkVisualizeSampling()
{
  std::cout << "-------------------------------------------------------" << std::endl;
  OMPL_INFORM("Running system performance benchmark - benchmarkVisualizeSampling()");

  ClearanceSamplerPtr clearanceSampler(new ob::MinimumClearanceValidStateSampler(si_.get()));
  clearanceSampler->setMinimumObstacleClearance(sg_->getObstacleClearance());

  base::StateSamplerPtr sampler;
  sampler = si_->allocStateSampler();

  const std::size_t benchmarkRuns = 1000000;
  std::size_t debugIncrement = std::max(benchmarkRuns, std::size_t(benchmarkRuns / 100.0));

  // Pre-allocate state
  std::vector<ompl::base::State *> stateMemory(debugIncrement);

  for (base::State *&state : stateMemory)
    state = si_->getStateSpace()->allocState();

  std::vector<const ompl::base::State *> states;
  states.reserve(debugIncrement);
  std::vector<ot::VizColors> colors;
  colors.reserve(debugIncrement);

  // Allow time to reset image
  visual_->viz1()->trigger();
  usleep(0.1 * 1000000);

  // Benchmark runtime
  for (std::size_t i = 0; i < benchmarkRuns; ++i)
  {
    base::State *candidateState = stateMemory[i % debugIncrement];
    clearanceSampler->sample(candidateState);

    states.push_back(candidateState);
    colors.push_back(tools::GREEN);

    if ((i + 1) % debugIncrement == 0 || i == benchmarkRuns - 1)
    {
      visual_->viz1()->states(states, colors, tools::SMALL);
      visual_->viz1()->trigger();
      usleep(0.1 * 1000000);

      states.clear();
      colors.clear();
      if (visual_->viz1()->shutdownRequested())
        break;
    }
  }

  for (base::State *state : stateMemory)
    si_->freeState(state);

  // Benchmark runtime
  OMPL_INFORM("Done");
  exit(0);
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << std::endl;
}

void SparseGenerator::benchmarkSparseGraphGeneration()
{
  std::cout << "-------------------------------------------------------" << std::endl;
  OMPL_INFORM("Running graph generation benchmark");
  time::point startTime = time::now();  // Benchmark

  // Create graph
  createSPARS();

  double time = time::seconds(time::now() - startTime);
  OMPL_INFORM("Graph generation took %f seconds", time);  // Benchmark

  // std::ofstream loggingFile;  // open to append
  // // loggingFile.open(benchmarkFilePath_.c_str(), std::ios::out);  // no append
  // loggingFile.open(benchmarkFilePath_.c_str(), std::ios::app);  // append
  // loggingFile << time << ", " << sg_->getNumEdges() << ", " << sg_->getNumVertices() << std::endl;
  // loggingFile.close();

  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << std::endl;
}


}  // namespace bolt
}  // namespace tools
}  // namespace ompl
