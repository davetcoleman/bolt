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
#include <ompl/base/spaces/RealVectorStateSpace.h>

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
  vertexDiscretizer_ = std::make_shared<VertexDiscretizer>(sg_);

  // Initialize threading tools
  // Speed up random sampling with these threads
candidateQueue_ = std::make_shared<CandidateQueue>(sg_, this);
}

SparseGenerator::~SparseGenerator(void)
{
  sampler_.reset();
}

void SparseGenerator::clear()
{
  numRandSamplesAdded_ = 0;
  numConsecutiveFailures_ = 0;
  maxConsecutiveFailures_ = 0;
  maxPercentComplete_ = 0;

  candidateQueue_->clear();
}

bool SparseGenerator::setup(std::size_t indent)
{
  // Load minimum clearance state sampler
  sampler_ = sg_->getSampler(si_, sg_->getObstacleClearance(), indent);

  // Configure vertex discretizer
  vertexDiscretizer_->setMinimumObstacleClearance(sg_->getObstacleClearance());
  vertexDiscretizer_->setDiscretization(sparseCriteria_->getDiscretization());

  return true;
}

void SparseGenerator::createSPARS(std::size_t indent)
{
  BOLT_FUNC(verbose_ || true, "createSPARS()");

  // Error check
  if (!useRandomSamples_ && !useDiscretizedSamples_)
  {
    OMPL_WARN("Unable to create SPARS because both random sampling and discretized sampling is disabled");
    return;
  }

  if (!si_->isSetup())
  {
    OMPL_INFORM("Space information setup was not yet called. Calling now.");
    si_->setup();
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
    BOLT_INFO(verbose_, "Adding discretized states");
    addDiscretizedStates(indent);
  }

  // Only display database if enabled
  // if (sg_->visualizeSparseGraph_ && sg_->visualizeSparseGraphSpeed_ > std::numeric_limits<double>::epsilon())
  // sg_->displayDatabase(true, true, 1, indent);

  // Finish the graph with random samples
  if (useRandomSamples_)
  {
    BOLT_INFO(true, "Adding random samples states");
    // addRandomSamplesOneThread(indent);
    addRandomSamplesThreaded(indent);
  }

  // Profiler
  CALLGRIND_TOGGLE_COLLECT;
  CALLGRIND_DUMP_STATS;

  // Save graph - this also calls removeDeletedVertices();
  sg_->saveIfChanged(indent);

  // Benchmark runtime
  double duration = time::seconds(time::now() - timeDiscretizeAndRandomStarted_);

  // Check how many connected components exist
  std::size_t numSets = sg_->getDisjointSetsCount(indent);

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

  BOLT_INFO(1, "-----------------------------------------");
  BOLT_INFO(1, "Created SPARS graph                      ");
  BOLT_INFO(1, "  Vertices:                  " << sg_->getNumRealVertices());
  BOLT_INFO(1, "  Edges:                     " << sg_->getNumEdges());
  BOLT_INFO(1, "  Generation time:           " << duration);
  BOLT_INFO(1, "  Disjoint sets:             " << numSets);
  BOLT_INFO(1, "  Sparse Criteria:           ");
  BOLT_INFO(1, "     SparseDelta:            " << sparseCriteria_->getSparseDelta());
  BOLT_INFO(1, "     Stretch Factor:         " << sparseCriteria_->getStretchFactor());
  BOLT_INFO(1, "     Discretization:         " << sparseCriteria_->getDiscretization());
  BOLT_INFO(1, "  Edge Lengths:              ");
  BOLT_INFO(1, "     Max:                    " << maxEdgeLength);
  BOLT_INFO(1, "     Min:                    " << minEdgeLength);
  BOLT_INFO(1, "     Average:                " << averageEdgeLength);
  BOLT_INFO(1, "     Difference:             " << averageEdgeLength - sparseCriteria_->getSparseDelta());
  BOLT_INFO(1, "     Penetration:            " << sparseCriteria_->getDiscretizePenetrationDist());
  BOLT_INFO(1, "  Criteria additions:        ");
  BOLT_INFO(1, "    Coverage:                " << sg_->numSamplesAddedForCoverage_);
  BOLT_INFO(1, "    Connectivity:            " << sg_->numSamplesAddedForConnectivity_);
  BOLT_INFO(1, "    Interface:               " << sg_->numSamplesAddedForInterface_);
  BOLT_INFO(1, "    Quality:                 " << sg_->numSamplesAddedForQuality_);
  BOLT_INFO(1, "  Num random samples added:  " << numRandSamplesAdded_);
  BOLT_INFO(1, "  Num vertices moved:        " << sparseCriteria_->getNumVerticesMoved());
  BOLT_INFO(1, "  CandidateQueue Misses:     " << candidateQueue_->getTotalMisses());
#ifdef ENABLE_QUALITY
  std::pair<std::size_t, std::size_t> interfaceStats = sparseCriteria_->getInterfaceStateStorageSize();
  BOLT_INFO(1, "  InterfaceData:             ");
  BOLT_INFO(1, "    States stored:           " << interfaceStats.first);
  BOLT_INFO(1, "    Missing interfaces:      " << interfaceStats.second);
  BOLT_INFO(1, "-----------------------------------------");
#endif

  // Copy-paste data
  copyPasteState(numSets);

#ifndef NDEBUG
  // Ensure the graph is valid
  if (false && !sg_->verifyGraph(indent))
  {
    OMPL_ERROR("Sparse graph did not pass test");
  }
#endif

  // Visualize database if it was not visualized during construction
  if (sg_->visualizeSparseGraph_ && sg_->visualizeSparseGraphSpeed_ < std::numeric_limits<double>::epsilon())
    sg_->displayDatabase(true, true, 1, indent);

  // Check optimality of graph
  if (verifyGraphProperties_)
    checkGraphOptimality(indent);

  OMPL_INFORM("Finished creating sparse database");
}

void SparseGenerator::copyPasteState(std::size_t numSets)
{
  lastGraphGenerationTime_ = time::seconds(time::now() - timeDiscretizeAndRandomStarted_);
  std::stringstream line;

  // clang-format off
  line << "=SPLIT(\"Bolt, "
       << mapName_ << ", "
       << sparseCriteria_->sparseDeltaFraction_ << ", "
       << sparseCriteria_->getSparseDelta() << ", "
       << sparseCriteria_->getDiscretization() << ", "
       << sparseCriteria_->getStretchFactor() << ", "
       << sparseCriteria_->getNearSamplePointsMultiple() << ", "
       << useDiscretizedSamples_ << ", "
       << useRandomSamples_ << ", "
       << 0 << ", "
       << sparseCriteria_->useClearEdgesNearVertex_ << ", "
       << 0 << ", "
       << sparseCriteria_->useEdgeImprovementRule_ << ", "
       << fourthCriteriaAfterFailures_ << ", "
       << terminateAfterFailures_ << ", "
       << maxConsecutiveFailures_ << ", "
       << sg_->getNumRealVertices() << ", "
       << sg_->getNumEdges() << ", "
       << numSets << ", "
       << lastGraphGenerationTime_ << "\", \",\")";
  // clang-format on

  // Save log
  stringLog_.push_back(line.str());

  std::size_t indent = 0;
  if (stringLog_.size() > 1000)
    BOLT_WARN(true, "Copy Paste Log is getting big: " << stringLog_.size());

  // Output to console
  std::cout << stringLog_.back() << std::endl;
}

void SparseGenerator::dumpLog()
{
  // Dump to console
  for (auto line : stringLog_)
    std::cout << line << std::endl;

  stringLog_.clear();
}

void SparseGenerator::addDiscretizedStates(std::size_t indent)
{
  BOLT_FUNC(verbose_, "addDiscretizedStates()");
  bool vAdd = sg_->vAdd_;
  sg_->vAdd_ = false;  // only do this for random sampling

  // This only runs if the graph is empty
  if (!sg_->isEmpty())
  {
    BOLT_WARN(verbose_, "Unable to generate discretized states because graph is not empty");
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
  BOLT_FUNC(verbose_, "addRandomSamplesOneThread()");

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
    if (!sampler_->sample(candidateState))
    {
      OMPL_ERROR("Unable to find valid sample");
      exit(-1);  // this should never happen
    }

    // Debug
    if (false)
    {
      BOLT_DEBUG(verbose_, "Randomly sampled state: " << candidateState);
      sg_->debugState(candidateState);
    }

    bool usedState = false;
    if (!addSample(candidateState, threadID, usedState, indent))
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
  BOLT_FUNC(verbose_, "addRandomSamplesThreaded()");

  // Clear stats
  numRandSamplesAdded_ = 0;
  timeRandSamplesStarted_ = time::now();
  maxConsecutiveFailures_ = 0;
  maxPercentComplete_ = 0;

  candidateQueue_->startGenerating(indent);

  const std::size_t threadID = 0;
  while (!visual_->viz1()->shutdownRequested())
  {
    // time::point startTime2 = time::now(); // Benchmark

    // Get next state and its corresponding neighbors
    SparseCandidateData &candidateD = candidateQueue_->getNextCandidate(indent);

    bool usedState = false;

    // time::point startTime = time::now(); // Benchmark
    bool sparseGraphNotCompleted = addSample(candidateD, threadID, usedState, indent);
    // BOLT_GREEN(0, 1, time::seconds(time::now() - startTime) << " add sample"); // Benchmark

    if (!sparseGraphNotCompleted)
    {
      candidateQueue_->stopGenerating(indent);
      return true;  // no more states needed
    }

    // BOLT_DEBUG(true, "used: " << usedState << " numRandSamplesAdded: " <<
    // numRandSamplesAdded_);

    // Tell other threads whether the candidate was used - this unloads the memory
    candidateQueue_->setCandidateUsed(usedState, indent);

    // BOLT_DEBUG(1, time::seconds(time::now() - startTime2) << " whole sampling loop"); // Benchmark
  }  // while(true) create random sample

  return true;  // program should never reach here
}

bool SparseGenerator::addSample(ob::State *state, std::size_t threadID, bool &usedState, std::size_t indent)
{
  // Find nearby nodes
  SparseCandidateData candidateD(state);
  findGraphNeighbors(candidateD, threadID, indent);

  return addSample(candidateD, threadID, usedState, indent);
}

bool SparseGenerator::addSample(SparseCandidateData &candidateD, std::size_t threadID, bool &usedState,
                                std::size_t indent)
{
  BOLT_FUNC(verbose_, "addSample() threadID: " << threadID);

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
      double addHz = numRandSamplesAdded_ / duration;

      // Change save interval based on addition rate
      const double saveEveryXSeconds = 5 * 60.0;  // 5*pow(2, si_->getStateDimension());
      const std::size_t saveEveryXNodeAdditions = saveEveryXSeconds * addHz;
      saveInterval_ = saveEveryXNodeAdditions;

      BOLT_DEBUG(true, "Adding samples at rate: " << addHz << " hz");
      BOLT_DEBUG(true, "saveEveryXMinutes: " << saveEveryXSeconds / 60.0);
      BOLT_DEBUG(true, "saveEveryXNodeAdditions: " << saveEveryXNodeAdditions);

      // copyPasteState();
    }

    // Increment statistics
    numRandSamplesAdded_++;

    usedState = true;

    // Check if shutdown requested
    if (visual_->viz1()->shutdownRequested())
    {
      BOLT_INFO(true, "Shutdown requested");
      stopCandidateQueueAndSave(indent);
      return false;
    }
  }
  else
  {
    // State was not added
    numConsecutiveFailures_++;

    if (numConsecutiveFailures_ > maxConsecutiveFailures_)
    {
      maxConsecutiveFailures_ = numConsecutiveFailures_;

      if (sparseCriteria_->useQualityCriteria_)
        showStatus(indent);
      else
        showNoQualityStatus(indent);
    }
  }

  // Check consecutive failures to determine if quality criteria needs to be enabled
  if (!sparseCriteria_->getUseFourthCriteria() && numConsecutiveFailures_ >= fourthCriteriaAfterFailures_)
  {
    if (!sparseCriteria_->useQualityCriteria_)
    {
      BOLT_INFO(true, "---------------------------------------------------");
      BOLT_WARN(true, "Fourth criteria (quality) is disabled");
      return false;  // stop inserting states
    }

    BOLT_INFO(true, "---------------------------------------------------");
    BOLT_INFO(true, "Starting to check for 4th quality criteria because " << numConsecutiveFailures_
                                                                          << " consecutive failures have occured");
    BOLT_INFO(true, "");

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
    BOLT_WARN(true, "SPARS creation finished because " << terminateAfterFailures_ << " consecutive insertion "
                                                                                     "failures reached");
    return false;  // stop inserting states
  }

  return true;
}

void SparseGenerator::showStatus(std::size_t indent)
{
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
    const std::size_t showEvery = std::max(1, int(12 - si_->getStateDimension() * 2));
    if (percentComplete % showEvery == 0)
    {
      if (!sparseCriteria_->getUseFourthCriteria())
      {
        BOLT_WARN(true, "Pre-quality progress: " << percentComplete
                                                 << "% (maxConsecutiveFailures: " << maxConsecutiveFailures_
                                                 << ", numRandSamplesAdded: " << numRandSamplesAdded_ << ")");
      }
      else
      {
        BOLT_GREEN(true, "Quality termination progress: " << percentComplete << "%");
      }
      // copyPasteState();
    }
  }
}

void SparseGenerator::showNoQualityStatus(std::size_t indent)
{
  std::size_t percentComplete = ceil(maxConsecutiveFailures_ / double(terminateAfterFailures_) * 100.0);

  // Every time the whole number of percent complete changes, show to user
  if (percentComplete > maxPercentComplete_)
  {
    maxPercentComplete_ = percentComplete;

    // Show varying granularity based on number of dimensions
    const std::size_t showEvery = std::max(1, int(12 - si_->getStateDimension() * 2));
    if (percentComplete % showEvery == 0)
    {
      BOLT_WARN(true, "Progress: " << percentComplete << "% (maxConsecutiveFailures: " << maxConsecutiveFailures_
                                   << ", numRandSamplesAdded: " << numRandSamplesAdded_ << ")");
      // copyPasteState();
    }
  }
}

void SparseGenerator::findGraphNeighbors(SparseCandidateData &candidateD, std::size_t threadID, std::size_t indent)
{
  findGraphNeighbors(candidateD, sparseCriteria_->getSparseDelta(), threadID, indent);
}

void SparseGenerator::findGraphNeighbors(SparseCandidateData &candidateD, double distance, std::size_t threadID,
                                         std::size_t indent)
{
  BOLT_FUNC(vFindGraphNeighbors_, "findGraphNeighbors() within sparse delta " << sparseCriteria_->getSparseDelta());

  // Search in thread-safe manner
  sg_->getQueryStateNonConst(threadID) = candidateD.state_;
  sg_->getNN()->nearestR(sg_->getQueryVertices(threadID), distance, candidateD.graphNeighborhood_);
  sg_->getQueryStateNonConst(threadID) = nullptr;

  // Now that we got the neighbors from the NN, we must remove any we can't see
  for (std::size_t i = 0; i < candidateD.graphNeighborhood_.size(); ++i)
  {
    const SparseVertex &v2 = candidateD.graphNeighborhood_[i];

    // Don't collision check if they are the same state
    if (candidateD.state_ != sg_->getState(v2))  // TODO: remove this check
    {
      if (!si_->checkMotion(candidateD.state_, sg_->getState(v2)))
      {
        continue;
      }
    }
    else                                                                           // if (vFindGraphNeighbors_)
      throw Exception(name_, "Skipping collision checking because same vertex ");  // TODO remove check

    // The two are visible to each other!
    candidateD.visibleNeighborhood_.push_back(candidateD.graphNeighborhood_[i]);
  }

  BOLT_DEBUG(vFindGraphNeighbors_,
             "Graph neighborhood: " << candidateD.graphNeighborhood_.size()
                                    << " | Visible neighborhood: " << candidateD.visibleNeighborhood_.size());
}

bool SparseGenerator::checkGraphOptimality(std::size_t indent)
{
  BOLT_FUNC(true, "checkGraphOptimality()");

  std::size_t numTests = 1000;
  std::size_t numFailedPlans = 0;

  // Make sure motion validator is set to zero clearance
  base::DiscreteMotionValidator *dmv = dynamic_cast<base::DiscreteMotionValidator *>(si_->getMotionValidator().get());
  BOLT_ASSERT(dmv->getRequiredStateClearance() == 0, "Discrete motion validator should have clearance = 0");

  // For each test
  for (std::size_t i = 0; i < numTests; ++i)
  {
    if (visual_->viz1()->shutdownRequested())
      break;

    time::point startTime = time::now();  // Benchmark

    // Choose random start and goal state that has a nearest neighbor
    std::vector<SparseCandidateData> endPoints(2);

    for (SparseCandidateData &point : endPoints)
    {
      // Allocate
      point.state_ = si_->getStateSpace()->allocState();

      // Sample
      if (!sampler_->sample(point.state_))
        throw Exception(name_, "No valid sample found");

      // Allow edges to be 5% longer than SparseDelta
      const double dist = sparseCriteria_->getSparseDelta() * 1.05;

      // Find nearest neighbor
      findGraphNeighbors(point, dist, 0 /*threadID*/, indent);

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
      BOLT_ERROR("No path found through graph");

      // Clear out previous path for clarity
      visual_->viz2()->deleteAllMarkers();
      visual_->viz2()->trigger();
      usleep(0.001 * 1000000);

      // Visualize actual and snapped states
      visual_->viz3()->deleteAllMarkers();
      visual_->viz3()->state(actualStart, tools::LARGE, tools::RED, 0);
      visual_->viz3()->state(actualGoal, tools::LARGE, tools::LIME_GREEN, 0);
      visual_->viz3()->state(sg_->getState(start), tools::LARGE, tools::BLACK, 0);
      visual_->viz3()->state(sg_->getState(goal), tools::LARGE, tools::BLACK, 0);
      visual_->viz3()->trigger();

      // Show database with visibility regions (coverage)
      sg_->visualizeDatabaseCoverage_ = true;
      sg_->displayDatabase();

      visual_->viz1()->spin();
      numFailedPlans++;
      continue;
    }

    double duration = time::seconds(time::now() - startTime);
    // OMPL_INFORM("PLANNING took %f seconds", duration); // Benchmark
    avgPlanTime_.push_back(duration);

    // If path found (same connected component) sum up total length
    geometric::PathGeometric geometricSolution(si_);
    convertVertexPathToStatePath(vertexPath, actualStart, actualGoal, geometricSolution);

    // Smooth path to find the "optimal" path
    geometric::PathGeometric smoothedPath = geometricSolution;
    geometric::PathGeometric *smoothedPathPtr = &smoothedPath;
    sg_->getSparseSmoother()->smoothMax(smoothedPathPtr, indent);

    // Show the two paths
    bool showEveryPath = false;
    if (showEveryPath)
    {
      // Visualize actual and snapped states
      visual_->viz3()->deleteAllMarkers();
      visual_->viz3()->state(actualStart, tools::LARGE, tools::RED, 0);
      visual_->viz3()->state(actualGoal, tools::LARGE, tools::LIME_GREEN, 0);
      visual_->viz3()->state(sg_->getState(start), tools::LARGE, tools::BLACK, 0);
      visual_->viz3()->state(sg_->getState(goal), tools::LARGE, tools::BLACK, 0);
      visual_->viz3()->trigger();

      visual_->viz2()->deleteAllMarkers();
      visual_->viz2()->path(&geometricSolution, tools::SMALL, tools::BLACK, tools::RED);
      visual_->viz2()->path(smoothedPathPtr, tools::MEDIUM, tools::BLACK, tools::LIME_GREEN);
      visual_->viz2()->trigger();
      usleep(0.001 * 1000000);
    }

    // Calculate theoretical guarantees
    double optimalLength = smoothedPathPtr->length();
    double sparseLength = geometricSolution.length();
    double theoryLength = sparseCriteria_->getStretchFactor() * optimalLength + 4 * sparseCriteria_->getSparseDelta();
    double percentOfMaxAllows = sparseLength / theoryLength * 100.0;

    // Save path quality
    double pathQuality = optimalLength / sparseLength;
    // BOLT_DEBUG(true, "pathQuality: " << pathQuality << " optimalLength: " << optimalLength << " sparseLength:
    // " << sparseLength);
    avgPathQuality_.push_back(pathQuality);

    // Output to console
    bool show = sparseLength >= theoryLength || vGuarantees_;
    BOLT_DEBUG(show, "-----------------------------------------");
    BOLT_DEBUG(show, "Checking Asymptotic Optimality Guarantees");
    BOLT_DEBUG(show, "Raw Path Length:         " << sparseLength);
    BOLT_DEBUG(show, "Smoothed Path Length:    " << optimalLength);
    BOLT_DEBUG(show, "Smoothed Path States:    " << smoothedPathPtr->getStateCount());
    BOLT_DEBUG(show, "Theoretical Path Length: " << theoryLength);
    BOLT_DEBUG(show, "Stretch Factor t:        " << sparseCriteria_->getStretchFactor());
    BOLT_DEBUG(show, "Sparse Delta:            " << sparseCriteria_->getSparseDelta());

    BOLT_ASSERT(sparseLength > std::numeric_limits<double>::epsilon(), "Path is zero length");

    if (sparseLength >= theoryLength)
    {
      BOLT_ERROR("Asymptotic optimality guarantee VIOLATED");

      // Show the two paths
      visual_->viz2()->deleteAllMarkers();
      visual_->viz2()->path(&geometricSolution, tools::SMALL, tools::BLACK, tools::RED);
      visual_->viz2()->path(smoothedPathPtr, tools::MEDIUM, tools::BLACK, tools::LIME_GREEN);
      visual_->viz2()->trigger();
      usleep(0.001 * 1000000);

      // Show database with visibility regions (coverage)
      sg_->visualizeDatabaseCoverage_ = true;
      sg_->displayDatabase();

      visual_->viz1()->spin();
      return false;
    }
    else
      BOLT_GREEN(show, "Asymptotic optimality guarantee maintained");
    BOLT_WARN(show, "Percent of max allowed:  " << percentOfMaxAllows << " %");
    BOLT_DEBUG(show, "-----------------------------------------");

    // visual_->prompt("next problem");
  }

  // Summary
  BOLT_DEBUG(1, "-----------------------------------------");
  BOLT_DEBUG(1, "Checking Asymptotic Optimality Guarantees");
  BOLT_DEBUG(1, "Total tests:               " << numTests);
  BOLT_DEBUG(1, "Number failed plans:       " << numFailedPlans);
  BOLT_DEBUG(1, "-----------------------------------------");

  return true;
}

void SparseGenerator::debugNoNeighbors(SparseCandidateData &point, std::size_t indent)
{
  BOLT_FUNC(true, "Found sampled vertex with no neighbors");

  // Clear Window 2
  visual_->viz2()->deleteAllMarkers();
  visual_->viz2()->trigger();

  // Visualize
  visual_->viz3()->deleteAllMarkers();
  visual_->viz3()->state(point.state_, tools::LARGE, tools::RED, 0);

  BOLT_DEBUG(true, "point.graphNeighborhood_.size() = " << point.graphNeighborhood_.size());

  for (auto v : point.graphNeighborhood_)
  {
    BOLT_DEBUG(true, " - showing nearest neighbor " << v);
    visual_->viz3()->state(sg_->getState(v), tools::LARGE, tools::ORANGE, 0);
    visual_->viz3()->edge(sg_->getState(v), point.state_, tools::MEDIUM, tools::BLUE);

    BOLT_DEBUG(true, " point.state_: " << point.state_);
    BOLT_DEBUG(true, " sg_->getState(v): " << sg_->getState(v));

    if (!si_->getMotionValidator()->checkMotion(point.state_, sg_->getState(v), visual_))
      BOLT_DEBUG(true, " in collision ");
    else
      BOLT_DEBUG(true, " not in collision ");

    // Trigger after checkMotion
    visual_->viz2()->state(point.state_, tools::LARGE, tools::LIME_GREEN, 0);  // show start
    visual_->viz2()->trigger();
  }
  visual_->viz3()->trigger();
  usleep(0.001 * 1000000);

  // Find nearest neighbor - SECOND TRY
  double dist = sparseCriteria_->getSparseDelta() * 1.5;
  findGraphNeighbors(point, dist, 0 /*threadID*/, indent);
  if (point.visibleNeighborhood_.empty())
  {
    BOLT_ERROR("still empty");
  }
  else
  {
    BOLT_DEBUG(true, "not empty anymore! ");

    double dist = si_->distance(sg_->getState(point.visibleNeighborhood_.front()), point.state_);
    BOLT_DEBUG(true, "distance to nearest vertex is " << dist
                                                      << ", sparse delta = " << sparseCriteria_->getSparseDelta());

    visual_->viz3()->edge(sg_->getState(point.visibleNeighborhood_.front()), point.state_, tools::SMALL, tools::RED);
    visual_->viz3()->trigger();
    usleep(0.001 * 1000000);
  }

  BOLT_ERROR("Found sampled vertex with no neighbors");
  visual_->viz1()->spin();
}

bool SparseGenerator::convertVertexPathToStatePath(std::vector<SparseVertex> &vertexPath,
                                                   const base::State *actualStart, const base::State *actualGoal,
                                                   og::PathGeometric &geometricSolution)
{
  BOLT_ASSERT(!vertexPath.empty(), "Vertex path is empty");

  geometricSolution.append(actualStart);

  // Reverse the vertexPath and convert to state path
  for (std::size_t i = vertexPath.size(); i > 0; --i)
  {
    geometricSolution.append(sg_->getState(vertexPath[i - 1]));
  }

  geometricSolution.append(actualGoal);

  return true;
}

void SparseGenerator::stopCandidateQueueAndSave(std::size_t indent)
{
  if (!(sg_->getSavingEnabled() && sg_->hasUnsavedChanges()))
    return;

  BOLT_FUNC(verbose_, "stopCandidateQueueAndSave()");

  // Stop and reset the candidate queue because it uses the nearest neighbors and will have bad vertices stored
  candidateQueue_->stopGenerating(indent);

  // Save
  sg_->saveIfChanged(indent);

  // Restart the queue
  candidateQueue_->startGenerating(indent);
}

void SparseGenerator::benchmarkValidClearanceSampler(std::size_t indent)
{
  std::cout << "-------------------------------------------------------" << std::endl;
  OMPL_INFORM("Running benchmark for random valid sampler with CLEARANCE");
  base::State *candidateState = si_->getStateSpace()->allocState();

  // Choose sampler based on clearance
  base::ValidStateSamplerPtr sampler = sg_->getSampler(si_, sg_->getObstacleClearance(), indent);

  const std::size_t benchmarkRuns = 10000;
  std::size_t debugIncrement = benchmarkRuns / 10;

  // Benchmark runtime
  time::point startTimeSampler;
  double totalDurationSampler = 0;
  for (std::size_t i = 0; i < benchmarkRuns; ++i)
  {
    startTimeSampler = time::now();
    sampler->sample(candidateState);

    totalDurationSampler += time::seconds(time::now() - startTimeSampler);

    if (i % debugIncrement == 0)
    {
      std::cout << "Progress: " << i / double(benchmarkRuns) * 100.0 << "%" << std::endl;
      if (visual_->viz1()->shutdownRequested())
        break;
    }
  }
  // Benchmark runtime
  OMPL_INFORM("  sample() took %f seconds (%f per run)", totalDurationSampler, totalDurationSampler / benchmarkRuns);
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << std::endl;
}

void SparseGenerator::benchmarkRandValidSampling(std::size_t indent)
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
    {
      std::cout << "Progress: " << i / double(benchmarkRuns) * 100.0 << "%" << std::endl;
      if (visual_->viz1()->shutdownRequested())
        break;
    }
  }
  // Benchmark runtime
  OMPL_INFORM("  isValid() took %f seconds (%f per run)", totalDurationValid, totalDurationValid / benchmarkRuns);
  OMPL_INFORM("  sampleUniform() took %f seconds (%f per run)", totalDurationSampler,
              totalDurationSampler / benchmarkRuns);
  OMPL_INFORM("  Percent valid: %f", validCount / double(benchmarkRuns) * 100);
}

void SparseGenerator::benchmarkVisualizeSampling(std::size_t indent)
{
  std::cout << "-------------------------------------------------------" << std::endl;
  OMPL_INFORM("Running system performance benchmark - benchmarkVisualizeSampling()");

  // Choose sampler based on clearance
  base::ValidStateSamplerPtr sampler = sg_->getSampler(si_, sg_->getObstacleClearance(), indent);

  const std::size_t benchmarkRuns = 100000;
  const std::size_t debugIncrement = std::max(1.0, (benchmarkRuns / 100.0));
  std::cout << "debugIncrement: " << debugIncrement << std::endl;

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
    sampler->sample(candidateState);

    states.push_back(candidateState);
    colors.push_back(tools::LIME_GREEN);

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
  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << std::endl;
}

void SparseGenerator::benchmarkSparseGraphGeneration(std::size_t indent)
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

void SparseGenerator::benchmarkMemoryAllocation(std::size_t indent)
{
  std::cout << "-------------------------------------------------------" << std::endl;
  OMPL_INFORM("Running memory allocation benchmark");

  std::size_t numberStates = 10000000;
  // std::size_t numberStates = 5;
  std::size_t dim = 14;
  std::size_t tests = 4;
  base::RealVectorStateSpace space(dim);

  // METHOD 1
  time::point startTime0 = time::now();  // Benchmark
  for (std::size_t test = 0; test < tests; ++test)
  {
    // Allocate
    std::vector<base::State *> states;
    for (std::size_t i = 0; i < numberStates; ++i)
      states.push_back(space.allocState());

    // Free
    for (std::size_t i = 0; i < numberStates; ++i)
      space.freeState(states[i]);
  }
  OMPL_INFORM("naive took  %f seconds", time::seconds(time::now() - startTime0));  // Benchmark
  // visual_->prompt("test2");

  // METHOD 2
  time::point startTime1 = time::now();  // Benchmark
  for (std::size_t test = 0; test < tests; ++test)
  {
    // Allocate
    double *allValues = new double[dim * numberStates];
    base::RealVectorStateSpace::StateType *states = new base::RealVectorStateSpace::StateType[numberStates];
    for (std::size_t i = 0; i < numberStates; ++i)
    {
      // std::cout << "before (&states[i])->values: " <<  (&states[i])->values << std::endl;
      (&states[i])->values = &allValues[i * dim];
      // std::cout << "after  (&states[i])->values: " <<  (&states[i])->values << std::endl;
    }
    // base::State* statesOMPL = states;

    delete[] allValues;
    delete[] states;

    // Free
    // for (std::size_t i = 0; i < numberStates; ++i)
    {
      // std::cout << "before (&states[i])->values: " <<  (&states[i])->values << std::endl;
      // delete[] (&states[i])->values;
      // std::cout << "after  (&states[i])->values: " <<  (&states[i])->values << std::endl;
      // delete[] (&states[i])->values;
      // delete[] (&statesOMPL[i])->as<base::RealVectorStateSpace::StateType>()->values;
      // delete (&statesOMPL[i])->as<base::RealVectorStateSpace::StateType>();
    }
  }
  OMPL_INFORM("vector took %f seconds", time::seconds(time::now() - startTime1));  // Benchmark

  // usleep(4*1000000);
  visual_->prompt("test2");

  std::cout << "-------------------------------------------------------" << std::endl;
  std::cout << std::endl;
}

ExperiencePathStats SparseGenerator::addExperiencePath(geometric::PathGeometricPtr path, std::size_t indent)
{
  BOLT_FUNC(true, "addExperiencePath() with " << path->getStateCount() << " states");
  const std::size_t threadID = 0;

  // Reset this class
  // clear();

  // Set sparseDelta to be way lower
  sparseCriteria_->sparseDeltaFraction_ = sparseCriteria_->sparseDeltaFractionSecondary_;
  sparseCriteria_->setup(indent);

  // Debug visualizations
  // sparseCriteria_->vAddedReason_ = true;
  // sparseCriteria_->vCriteria_ = true;
  // sparseCriteria_->visualizeAttemptedStates_ = true;
  sg_->visualizeSparseGraph_ = true;
  sg_->vAdd_ = true;

  // Clear visuals if necesary
  if (sg_->visualizeSparseGraph_)
  {
    visual_->viz1()->deleteAllMarkers();
    visual_->viz1()->trigger();
  }


  std::size_t segmentCount = path->length() / sparseCriteria_->getDenseDelta();
  path->interpolate(segmentCount);
  BOLT_INFO(true, "After interpolation has " << path->getStateCount() << " states");

  std::vector<std::size_t> shuffledIDs;
  for (std::size_t i = 1; i < path->getStateCount(); ++i)
  {
    shuffledIDs.push_back(i);
  }                                                             // for each state in path
  std::random_shuffle(shuffledIDs.begin(), shuffledIDs.end());  // using built-in random generator

  // Insert vertices at random
  base::State *candidateState;
  bool usedState = true;  // flag indicating whether memory needs to be allocated again for candidateState

  bool addEdges = true; // a mode for skipping adding edges when graph is first constructed
  for (std::size_t i = 1; i < shuffledIDs.size(); ++i)
  {
    if (usedState)
    {
      candidateState = si_->allocState();
      usedState = false;
    }

    // Copy the state so the graph can own it
    si_->copyState(candidateState, path->getState(shuffledIDs[i]));

    // Add to roadmap
    usedState = addSampleSparseCriteria(candidateState, addEdges, indent);

    // visual_->viz6()->state(candidateState, tools::ROBOT, tools::BLUE);
    // visual_->prompt("adding state");
  }

  // free state if necessary
  if (!usedState)
    si_->freeState(candidateState);

  ExperiencePathStats stats;

  if (sg_->hasUnsavedChanges())
  {
    SparseStorage::GraphSizeChange result = sg_->save(indent);

    stats.numVerticesAdded_ = result.numVerticesAdded_;
    stats.numEdgesAdded_ = result.numEdgesAdded_;
  }
  else
    BOLT_MAGENTA(true, "Sparse graph NOT changed  -------------------------------------");

  BOLT_INFO(true, "Finished adding experience path to SparseGraph");

  return stats;
}

void SparseGenerator::createSPARS2(std::size_t indent)
{
  BOLT_FUNC(true, "createSPARS2()");
  BOLT_DEBUG(true, "si_->getMaximumExtent() " << si_->getMaximumExtent());

  // Setup sampler
  sampler_->setNrAttempts(1000);

  base::State *candidateState;
  bool usedState = true;  // flag indicating whether memory needs to be allocated again for candidateState

  std::size_t attempts = 1;  // start at 1 to allow modulo to skip 0
  std::size_t numStatesUsed = 0;
  bool addEdges = false; // a mode for skipping adding edges when graph is first constructed
  double nodeAdditionRate = std::numeric_limits<double>::infinity();
  while (!visual_->viz1()->shutdownRequested())
  {
    if (usedState)
    {
      candidateState = si_->allocState();
      usedState = false;
    }

    if (!sampler_->sample(candidateState))
      throw Exception(name_, "Unable to find valid sample");

    // Visualize
    if (visualizeSampling_)
    {
      // visual_->viz6()->deleteAllMarkers();
      visual_->viz6()->state(candidateState, tools::ROBOT, tools::DEFAULT);
      // visual_->viz6()->state(candidateState, tools::SMALL, tools::BLUE);
    }

    // Add to TaskGraph if it obeys sparse properties
    usedState = addSampleSparseCriteria(candidateState, addEdges, indent);

    // Record statitics
    numStatesUsed += usedState;

    if (visualizeSampling_)
    {
      visual_->viz6()->trigger();
    }

    // Only run at certain intervals
    static const std::size_t FEEDBACK_SAVE_INTERVAL = 1000;
    if (attempts % FEEDBACK_SAVE_INTERVAL == 0)
    {
      // Check if time to change modes
      static const double NODE_ADDITION_RATE_THRESHOLD = 0.01;
      nodeAdditionRate = numStatesUsed / double(attempts);
      if (!addEdges && nodeAdditionRate < NODE_ADDITION_RATE_THRESHOLD)
      {
        BOLT_INFO(true, "Add edges is now enabled");
        addEdges = true;
      }

      // User feedback
      std::size_t numSets = sg_->getDisjointSetsCount(indent);
      BOLT_INFO(true, "V: " << sg_->getNumVertices() << " E: " << sg_->getNumEdges() << " attempts: " << attempts
                            << " nodeAdditionRate: " << nodeAdditionRate << " DisjointSets: " << numSets);

      // Save less frequently
      if (attempts % (FEEDBACK_SAVE_INTERVAL * 4) == 0)
      {
        std::cout << std::endl;
        sg_->saveIfChanged(indent);
      }
    }
    attempts++;
  }  // for each sample attempt

  // free state if necessary
  if (!usedState)
    si_->freeState(candidateState);

  // Save graph
  sg_->saveIfChanged(indent);
}

bool SparseGenerator::addSampleSparseCriteria(base::State *candidateState, bool addEdges, std::size_t indent)
{
  BOLT_FUNC(vCriteria_, "addSampleSparseCriteria()");

  // Calculate secondary sparse delta (smaller than normal sparse delta)
  const double &sparseDelta = sg_->getSparseCriteria()->getSparseDelta();

  // Get neighbors of new state
  static const std::size_t THREAD_ID = 0;

  // Here we save computation time by finding only the nearest x nodes which typically result
  // in finding that a new node is not a candidate to be added. Must be greater than two for the
  // edge addition rule
  static const std::size_t AVG_NEAREST_NODES_TO_VALID = 30;  // TODO: auto tune this value rather than guess

  std::vector<SparseVertex> graphNeighborhood;
  graphNeighborhood.reserve(AVG_NEAREST_NODES_TO_VALID);

  sg_->getQueryStateNonConst(THREAD_ID) = candidateState;
  sg_->getNN()->nearestK(sg_->getQueryVertices(THREAD_ID), AVG_NEAREST_NODES_TO_VALID, graphNeighborhood);
  sg_->getQueryStateNonConst(THREAD_ID) = nullptr;

  // BOLT_DEBUG(true, graphNeighborhood.size() / double(sg_->getNumVertices())
  //                      << "%: Nearest neighbor: " << graphNeighborhood.size() << " out of " <<
  //                      sg_->getNumVertices());

  // Find all visibile neighbors
  std::vector<SparseVertex> visibleNeighborhood;
  for (std::size_t i = 0; i < graphNeighborhood.size(); ++i)
  {
    const SparseVertex &v2 = graphNeighborhood[i];

    // Visualize
    if (visualizeSampling_)
      visual_->viz6()->state(sg_->getState(v2), tools::MEDIUM, tools::ORANGE, 1);

    // Check for visible nearby neighbor
    if (!si_->checkMotion(candidateState, sg_->getState(v2)))
    {
      continue; // not visible, check next one
    }

    // we've found a valid edge - see if its the longest one yet
    // TODO: remove
    // double dist = si_->distance(candidateState, sg_->getState(v2));
    // if (dist > max_dist_)
    // {
    //   max_dist_ = dist;
    //   double percentOfSpace = max_dist_ / si_->getMaximumExtent();
    //   BOLT_WARN(percentOfSpace > 0.35, "max_dist_ found: " << max_dist_ << " percentOfSpace: " << percentOfSpace);
    // }

    // A visible state was found nearby so there is no way we would add candidate state
    // for coverage. We can now stop while in no addEdge mode
    if (!addEdges)
      return false;

    // Visualize
    if (visualizeSampling_)
      visual_->viz6()->edge(candidateState, sg_->getState(v2), tools::MEDIUM, tools::PINK);

    // The two are visible to each other!
    visibleNeighborhood.push_back(v2);

    // We only care about the first two visibile neighbors
    if (visibleNeighborhood.size() == 2)
    {
      BOLT_DEBUG(vCriteria_, "Collision checked: " << i << " motions of " << graphNeighborhood.size() << " nearby "
                                                                                                         "nodes");
      break;
    }

    // Enforce that the two closest nodes must also be visible for an edge to be added
    // But this only applies if at least oen visible neighbor has been found,
    // otherwise we could be finding a coverage node
    if (i > 1)
      break;
  }

  // Criteria 1: add guard (no nearby visibile nodes found *so far*)
  if (visibleNeighborhood.empty())
  {
    graphNeighborhood.clear();  // reset input structure
    graphNeighborhood.reserve(sg_->getNumVertices());

    // Check *all* vertices this time
    sg_->getQueryStateNonConst(THREAD_ID) = candidateState;
    sg_->getNN()->nearestR(sg_->getQueryVertices(THREAD_ID), sparseDelta, graphNeighborhood);
    sg_->getQueryStateNonConst(THREAD_ID) = nullptr;

    // Skip the first x because those were already collision checked
    for (std::size_t i = AVG_NEAREST_NODES_TO_VALID; i < graphNeighborhood.size(); ++i)
    {
      const SparseVertex &v2 = graphNeighborhood[i];

      // Check for visible nearby neighbor - if one is found visible, then this state should not be added
      if (si_->checkMotion(candidateState, sg_->getState(v2)))
      {
        //BOLT_WARN(true, "Missed a state that invalidates candidate state, on index " << i);
        return false;
      }
    }

    // No free paths means we add for coverage
    BOLT_DEBUG(vCriteria_, "Adding node for COVERAGE ");
    sg_->addVertex(candidateState, COVERAGE, indent);

    if (visualizeSampling_)
    {
      visual_->viz6()->state(candidateState, tools::ROBOT, tools::PINK, 1);
    }

    return true;
  }

  // Check if in state-only mode
  if (!addEdges)
    return false;

  // Criteria 2: determine if two closest visible nodes need connecting
  if (visibleNeighborhood.size() == 1)
  {
    BOLT_DEBUG(vCriteria_, "Not enough visibile neighbors (1)");
    // visual_->prompt("not enough");
    return false;  // did not use memory of candidateState
  }

  const SparseVertex &v1 = visibleNeighborhood[0];
  const SparseVertex &v2 = visibleNeighborhood[1];

  // Ensure two closest neighbors don't share an edge
  if (sg_->hasEdge(v1, v2))
  {
    BOLT_DEBUG(vCriteria_, "Two closest two neighbors already share an edge, not connecting them");

    // Sampled state was not used - free it
    // visual_->prompt("already share edge");
    return false;  // did not use memory of candidateState
  }

  // Don't add an interface edge if dist between the two verticies on graph are already the minimum in L1 space
  // if (!sg_->checkPathLength(v1, v2, indent))
  // {
  //   visual_->prompt("checkPathLenght");
  //   return false; // did not use memory of candidateState
  // }

  // If they can be directly connected
  if (si_->checkMotion(sg_->getState(v1), sg_->getState(v2)))
  {
    BOLT_DEBUG(vCriteria_, "INTERFACE: directly connected nodes");

    // Connect them
    sg_->addEdge(v1, v2, eINTERFACE, indent);

    if (visualizeSampling_)
    {
      visual_->viz6()->edge(sg_->getState(v1), sg_->getState(v2), tools::SMALL, tools::LIME_GREEN);
      visual_->viz6()->trigger();
      // visual_->prompt("directly added edge");
    }

    return false;  // did not use memory of candidateState
  }

  // They cannot be directly connected, so add the new node to the graph, to bridge the interface
  BOLT_DEBUG(vCriteria_, "INTERFACE2: Added new NODE and surrounding edges");
  SparseVertex newVertex = sg_->addVertex(candidateState, COVERAGE, indent);

  sg_->addEdge(newVertex, v1, eINTERFACE, indent);
  sg_->addEdge(newVertex, v2, eINTERFACE, indent);

  if (visualizeSampling_)
  {
    visual_->viz6()->state(candidateState, tools::MEDIUM, tools::YELLOW, 1);
    visual_->viz6()->edge(candidateState, sg_->getState(v1), tools::SMALL, tools::LIME_GREEN);
    visual_->viz6()->edge(candidateState, sg_->getState(v2), tools::SMALL, tools::LIME_GREEN);
    visual_->viz6()->trigger();
    // visual_->prompt("added node for interface");
  }

  return true;
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
