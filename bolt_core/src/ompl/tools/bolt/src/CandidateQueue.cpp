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
   Desc:   Maintain queue of potential smaples
*/

// OMPL
#include <ompl/tools/bolt/CandidateQueue.h>
#include <ompl/tools/bolt/SparseCriteria.h>
#include <ompl/tools/bolt/SparseGenerator.h>

// C++
#include <queue>
#include <thread>

namespace ompl
{
namespace tools
{
namespace bolt
{
CandidateQueue::CandidateQueue(SparseGraphPtr sg, SamplingQueuePtr samplingQueue, SparseGeneratorPtr sparseGenerator)
  : sg_(sg)
  , sparseCriteria_(sg_->getSparseCriteria())
  , sparseGenerator_(sparseGenerator)
  , samplingQueue_(samplingQueue)
  , si_(sg_->getSpaceInformation())
  , visual_(sg_->getVisual())
{
}

CandidateQueue::~CandidateQueue()
{
  while (!queue_.empty())
  {
    si_->freeState(queue_.front().state_);
    queue_.pop();
  }
}

void CandidateQueue::startGenerating(std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "startGenerating() Starting candidate queue thread");
  if (threadsRunning_)
  {
    BOLT_ERROR(indent, true, "CandidateQueue already running");
    return;
  }
  threadsRunning_ = true;

  // Stats
  totalMisses_ = 0;

  // Set number threads - should be at least less than 1 from total number of threads on system
  // 1 thread is for parent, 1 is for sampler, 1 is for GUIs, etc, remainder are for this
  numThreads_ = std::max(1, int(sg_->getNumQueryVertices() - 3));
  BOLT_DEBUG(indent, true, "Running CandidateQueue with " << numThreads_ << " threads");
  if (numThreads_ < 2)
    BOLT_WARN(indent, true, "Only running CandidateQueue with 1 thread");
  if (numThreads_ >= sg_->getNumQueryVertices())
  {
    BOLT_ERROR(indent, true, "Too many threads requested for candidate queue");
    exit(-1);
  }

  // Set the SamplingQueue queue size based on how many threads are here consuming
  samplingQueue_->setTargetQueueByThreads(numThreads_);

  // Create threads
  generatorThreads_.resize(numThreads_);

  // Starts on thread 1 because thread 0 is reserved for parent process
  for (std::size_t i = 0; i < generatorThreads_.size(); ++i)
  {
    // Create new collision checker
    base::SpaceInformationPtr si(new base::SpaceInformation(si_->getStateSpace()));
    si->setStateValidityChecker(si_->getStateValidityChecker());
    si->setMotionValidator(si_->getMotionValidator());

    // Load minimum clearance state sampler
    ob::MinimumClearanceValidStateSamplerPtr clearanceSampler =
        ob::MinimumClearanceValidStateSamplerPtr(new ob::MinimumClearanceValidStateSampler(si.get()));
    clearanceSampler->setMinimumObstacleClearance(sg_->getObstacleClearance());
    si->getStateValidityChecker()->setClearanceSearchDistance(sg_->getObstacleClearance());

    std::size_t threadID = i + 1;  // the first thread (0) is reserved for the parent process for use of samplingQuery
    generatorThreads_[i] =
        new boost::thread(boost::bind(&CandidateQueue::generatingThread, this, threadID, si, clearanceSampler, indent));
  }

  // Wait for first sample to be found
  // BOLT_DEBUG(indent, true, "Waiting for first candidate to be found");
  while (queue_.empty())
  {
    usleep(0.001 * 1000000);
  }
}

void CandidateQueue::stopGenerating(std::size_t indent)
{
  BOLT_FUNC(indent, true, "CandidateQueue.stopGenerating() Stopping generating thread");
  threadsRunning_ = false;

  // Join threads
  for (std::size_t i = 0; i < generatorThreads_.size(); ++i)
  {
    generatorThreads_[i]->join();
    delete generatorThreads_[i];
  }

  BOLT_FUNC(indent, true, "CandidateQueue.stopGenerating() Generating threads have stopped");
}

void CandidateQueue::generatingThread(std::size_t threadID, base::SpaceInformationPtr si,
                                      ClearanceSamplerPtr clearanceSampler, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "generatingThread() " << threadID);

  base::State *candidateState;

  while (threadsRunning_ && !visual_->viz1()->shutdownRequested())
  {
    BOLT_DEBUG(indent + 2, vThread_, "generatingThread: Running while loop on thread " << threadID);

    // Do not add more states if queue is full
    if (queue_.size() > targetQueueSize_)
      waitForQueueNotFull(indent + 2);

    // Get next sample
    getNextState(candidateState, clearanceSampler, indent + 2);

    BOLT_DEBUG(indent + 2, vThread_, "New candidateState: " << candidateState << " on thread " << threadID);

    if (!threadsRunning_)  // Check for thread ending
      break;

    // Find nearby nodes
    CandidateData candidateD(candidateState);

    // time::point startTime = time::now(); // Benchmark

    findGraphNeighbors(candidateD, threadID, indent + 2);

    // Benchmark
    // double time = time::seconds(time::now() - startTime);
    // totalTime_ += time;
    // totalCandidates_ ++;
    // double average = totalTime_ / totalCandidates_;
    // BOLT_MAGENTA_DEBUG(0, true,  time << " CandidateQueue, average: " << average << " queue: " << queue_.size()); //
    // Benchmark

    // Add to queue - thread-safe
    if (candidateD.graphVersion_ == sparseGenerator_->getNumRandSamplesAdded())
    {
      // std::cout << "pushCandidate: waiting for lock, " << candidateD.graphVersion_ << " =========================== "
      // << std::endl;
      boost::lock_guard<boost::shared_mutex> lock(candidateQueueMutex_);
      queue_.push(candidateD);
      // std::cout << "pushCandidate: added candidate ==================================" << std::endl;
    }
  }
}

void CandidateQueue::getNextState(base::State *&candidateState, ClearanceSamplerPtr clearanceSampler,
                                  std::size_t indent)
{
  // First attempt to get state from queue, otherwise we do it ourselves
  if (!samplingQueue_->getNextState(candidateState, indent + 2))
  {
    // Create new state ourselves
    candidateState = si_->allocState();

    // Sample randomly
    if (!clearanceSampler->sample(candidateState))
    {
      OMPL_ERROR("Unable to find valid sample");
      exit(-1);  // this should never happen
    }
  }
}

CandidateData &CandidateQueue::getNextCandidate(std::size_t indent)
{
  BOLT_CYAN_DEBUG(indent, false, "CandidateQueue.getNextCanidate(): queue size: "
                                     << queue_.size()
                                     << " num samples added: " << sparseGenerator_->getNumRandSamplesAdded());
  // This function is run in the parent thread

  // Keep looping until a non-expired candidate exists or the thread ends
  while (threadsRunning_)
  {
    // Clear all expired candidates
    // std::cout << "getNextCandidate: waiting for lock ++++++++++++++++++++++++++++" << std::endl;
    {
      boost::lock_guard<boost::shared_mutex> lock(candidateQueueMutex_);
      std::size_t numCleared = 0;
      while (!queue_.empty() && queue_.front().graphVersion_ != sparseGenerator_->getNumRandSamplesAdded() &&
             threadsRunning_)
      {
        // Next Candidate state is expired, delete
        si_->freeState(queue_.front().state_);
        queue_.pop();
        numCleared++;
      }
      BOLT_ERROR(indent, vClear_ && numCleared > 0, "Cleared " << numCleared << " states from CandidateQueue");

      // Return the first non-expired candidate if one exists
      if (!queue_.empty() && queue_.front().graphVersion_ == sparseGenerator_->getNumRandSamplesAdded())
      {
        // std::cout << "getNextCandidate: free lock ++++++++++++++++++++++++++" << std::endl;
        break;
      }
    }
    // std::cout << "getNextCandidate: free lock ++++++++++++++++++++++++++" << std::endl;

    // Wait for queue to not be empty
    bool oneTimeFlag = true;
    totalMisses_++;
    while (queue_.empty() && threadsRunning_)
    {
      if (oneTimeFlag)
      {
        BOLT_WARN(indent, vQueueEmpty_, "CandidateQueue: Queue is empty, waiting for next generated "
                                        "CandidateData");
        oneTimeFlag = false;
      }
      usleep(100);
    }
    if (!oneTimeFlag)
      BOLT_DEBUG(indent, vQueueEmpty_ && false, "CandidateQueue: No longer waiting on queue");
  }

  return queue_.front();
}

void CandidateQueue::setCandidateUsed(bool wasUsed, std::size_t indent)
{
  // This function is run in the parent thread

  if (!wasUsed)  // if was used the state is now in use elsewhere
    si_->freeState(queue_.front().state_);

  // std::cout << "setCandidateUsed: waiting for lock ------------------------" << std::endl;
  boost::lock_guard<boost::shared_mutex> lock(candidateQueueMutex_);
  queue_.pop();
  // std::cout << "setCandidateUsed: free lock ----------------------------- " << std::endl;
}

void CandidateQueue::waitForQueueNotFull(std::size_t indent)
{
  bool oneTimeFlag = true;
  while (queue_.size() >= targetQueueSize_ && threadsRunning_)
  {
    if (oneTimeFlag)
    {
      BOLT_DEBUG(indent, vQueueFull_, "CandidateQueue: Queue is full, generator is waiting");
      oneTimeFlag = false;
    }
    usleep(0.001 * 1000000);
  }
  if (!oneTimeFlag)
    BOLT_DEBUG(indent, vQueueFull_, "CandidateQueue: No longer waiting on full queue");
}

bool CandidateQueue::findGraphNeighbors(CandidateData &candidateD, std::size_t threadID, std::size_t indent)
{
  BOLT_FUNC(indent, vNeighbor_, "findGraphNeighbors() within sparse delta " << sparseCriteria_->getSparseDelta());

  // Get the version number of the graph, which is simply the number of states that have thus far been added
  // during this program's execution (not of all time). This allows us to know if the candidate has potentially
  // become "expired" because of a change in the graph
  candidateD.graphVersion_ = sparseGenerator_->getNumRandSamplesAdded();

  // Search in thread-safe manner
  // Note that the main thread could be modifying the NN, so we have to lock it
  sg_->getQueryStateNonConst(threadID) = candidateD.state_;
  {
    // std::cout << "getting nn lock " << std::endl;
    std::lock_guard<std::mutex> lock(sg_->getNNGuard());
    sg_->getNN()->nearestR(sg_->getQueryVertices(threadID), sparseCriteria_->getSparseDelta(),
                           candidateD.graphNeighborhood_);
  }
  // std::cout << "released nn lock " << std::endl;
  sg_->getQueryStateNonConst(threadID) = nullptr;

  // Now that we got the neighbors from the NN, we must remove any we can't see
  for (std::size_t i = 0; i < candidateD.graphNeighborhood_.size(); ++i)
  {
    SparseVertex v2 = candidateD.graphNeighborhood_[i];

    // Check for termination condition
    if (candidateD.graphVersion_ != sparseGenerator_->getNumRandSamplesAdded() || !threadsRunning_)
    {
      BOLT_WARN(indent, vNeighbor_, "findGraphNeighbors aborted b/c term cond");
      return false;
    }

    // Don't collision check if they are the same state
    if (candidateD.state_ != sg_->getState(v2))
    {
      if (!si_->checkMotion(candidateD.state_, sg_->getState(v2)))
      {
        continue;
      }
    }

    // Check for termination condition
    if (candidateD.graphVersion_ != sparseGenerator_->getNumRandSamplesAdded() || !threadsRunning_)
    {
      BOLT_WARN(indent, vNeighbor_, "findGraphNeighbors aborted b/c term cond");
      return false;
    }

    // The two are visible to each other!
    candidateD.visibleNeighborhood_.push_back(candidateD.graphNeighborhood_[i]);
  }

  BOLT_DEBUG(indent, vNeighbor_,
             "Graph neighborhood: " << candidateD.graphNeighborhood_.size()
                                    << " | Visible neighborhood: " << candidateD.visibleNeighborhood_.size());

  return true;
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
