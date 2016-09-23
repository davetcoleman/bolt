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
#include <bolt_core/CandidateQueue.h>
#include <bolt_core/SparseCriteria.h>
#include <bolt_core/SparseGenerator.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>

// C++
#include <queue>
#include <thread>

namespace ompl
{
namespace tools
{
namespace bolt
{
CandidateQueue::CandidateQueue(SparseGraphPtr sg, SparseGenerator* sparseGenerator)
  : sg_(sg)
  , sparseCriteria_(sg_->getSparseCriteria())
  , sparseGenerator_(sparseGenerator)
  , si_(sg_->getSpaceInformation())
  , visual_(sg_->getVisual())
{
}

CandidateQueue::~CandidateQueue()
{
  clear();
}

void CandidateQueue::clear()
{
  BOLT_ASSERT(!threadsRunning_, "Cannot clear while thread is running");

  // Add up all misses before resetting
  if (totalMisses_ > 0)
  {
    totalMissesOverAllResets_ += totalMisses_;
    totalResets_++;
  }

  // Clear
  totalMisses_ = 0;
  totalTime_ = 0;
  totalCandidates_ = 0;

  // TODO: is it possible the front item in the queue is being used elsewhere and setCandidateUsed()
  // was not called yet?
  while (!queue_.empty())
  {
    si_->freeState(queue_.front().state_);
    queue_.pop();
  }
}

void CandidateQueue::startGenerating(std::size_t indent)
{
  BOLT_FUNC(indent, true, "startGenerating() Starting candidate queue thread");
  if (threadsRunning_)
  {
    BOLT_ERROR(indent, true, "CandidateQueue already running");
    return;
  }
  threadsRunning_ = true;

  // Add up all misses before resetting
  if (totalMisses_ > 0)
  {
    totalMissesOverAllResets_ += totalMisses_;
    totalResets_++;
  }
  if (totalResets_ > 0)
  {
    BOLT_INFO(indent, true, "Average CandidateQueue misses: " << totalMissesOverAllResets_ / totalResets_);
  }

  // Stats
  totalMisses_ = 0;

  // Set number threads - should be at least less than 1 from total number of threads on system
  // 1 thread is for parent, 1 is for sampler, 1 is for GUIs, etc, remainder are for this
  //numThreads_ = std::max(1, int(sg_->getNumQueryVertices() - 3));

  // TODO: how to choose this number best to minimize queue misses?
  /* For 2D environment test data...
     Threads:
     2 AVERAGE CandidateQueue MISSES: 447
     3 AVERAGE CandidateQueue MISSES: 432
     4 AVERAGE CandidateQueue MISSES: 385
     5 AVERAGE CandidateQueue MISSES: 398

     For 3D environment test data...
     4 AVERAGE CandidateQueue MISSES: 2621
     5 AVERAGE CandidateQueue MISSES: 2429
     6 AVERAGE CandidateQueue MISSES: 2493

     so we choose 5.
  */
  numThreads_ = 5;
  BOLT_DEBUG(indent, true, "Running CandidateQueue with " << numThreads_ << " threads");
  if (numThreads_ < 2)
    BOLT_WARN(indent, true, "Only running CandidateQueue with 1 thread");
  if (numThreads_ >= sg_->getNumQueryVertices())
  {
    BOLT_ERROR(indent, true, "Too many threads requested for candidate queue");
    exit(-1);
  }

  // Create threads
  generatorThreads_.resize(numThreads_);

  for (std::size_t i = 0; i < generatorThreads_.size(); ++i)
  {
    // Create new collision checker
    base::SpaceInformationPtr si(new base::SpaceInformation(si_->getStateSpace()));
    si->setStateValidityChecker(si_->getStateValidityChecker());
    si->setMotionValidator(si_->getMotionValidator());

    // Choose sampler based on clearance
    base::ValidStateSamplerPtr sampler;
    if (sg_->getObstacleClearance() > std::numeric_limits<double>::epsilon())
    {
      BOLT_ERROR(indent, true, "Using clearance");
      // Load minimum clearance state sampler
      sampler.reset(new base::MinimumClearanceValidStateSampler(si.get()));
      // Set the clearance
      base::MinimumClearanceValidStateSampler* mcvss =
        dynamic_cast<base::MinimumClearanceValidStateSampler *>(sampler.get());
      mcvss->setMinimumObstacleClearance(sg_->getObstacleClearance());
      si->getStateValidityChecker()->setClearanceSearchDistance(sg_->getObstacleClearance());
    }
    else // regular sampler
    {
      BOLT_ERROR(indent, true, "NOT using clearance " << sg_->getObstacleClearance());
      sampler.reset(new base::UniformValidStateSampler(si.get()));
      //sampler = si_->allocStateSampler();
    }

    std::size_t threadID = i + 1;  // the first thread (0) is reserved for the parent process
    generatorThreads_[i] =
        new boost::thread(boost::bind(&CandidateQueue::generatingThread, this, threadID, si, sampler, indent));
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

  BOLT_DEBUG(indent, true, "CandidateQueue.stopGenerating() joined");

  // Clear the first state in the queue without freeing the memory
  // TODO: this is a memory leak, but sometimes it segfaults because perhaps that state was already
  // freed somewhere in the main thread (SparseCritera) and it was never reflected back here
  if (!queue_.empty())
  {
    // skip freeing
    queue_.pop();
  }

  // Clear remaining data
  while (!queue_.empty())
  {
    //std::cout << "Queue num " << queue_.size() << ": queue_.front().state_: " << queue_.front().state_;

    if (queue_.front().state_ == nullptr)
    {
      std::cout << std::endl;
      BOLT_WARN(indent, true, "Found state in queue that is null!");
    }
    else
    {
      // Do not free the state of the last item on the queue, because it is used somewhere else (?)
      si_->getStateSpace()->freeState(queue_.front().state_);
      //std::cout << "... freed " << std::endl;
    }

    queue_.pop();
  }

  BOLT_DEBUG(indent, true, "CandidateQueue.stopGenerating() finished");
}

void CandidateQueue::generatingThread(std::size_t threadID, base::SpaceInformationPtr si,
                                      base::ValidStateSamplerPtr sampler, std::size_t indent)
{
  BOLT_FUNC(indent, verbose_, "generatingThread() " << threadID);

  base::State *candidateState;

  while (threadsRunning_ && !visual_->viz1()->shutdownRequested())
  {
    BOLT_DEBUG(indent + 2, vThread_, "generatingThread: Running while loop on thread " << threadID);

    // Do not add more states if queue is full
    if (queue_.size() > targetQueueSize_)
      waitForQueueNotFull(indent + 2);

    // Create new state
    candidateState = si_->allocState();

    // Sample randomly
    if (!sampler->sample(candidateState))
      throw Exception(name_, "Unable to find valid sample");

    BOLT_DEBUG(indent + 2, vThread_, "New candidateState: " << candidateState << " on thread " << threadID);

    if (!threadsRunning_)  // Check for thread ending
    {
      BOLT_DEBUG(indent + 2, vThread_, "Thread ended, freeing memory");
      si_->freeState(candidateState);
      return;
    }

    // Find nearby nodes
    CandidateData candidateD(candidateState);

    // time::point startTime = time::now(); // Benchmark

    if (!findGraphNeighbors(candidateD, threadID, indent + 2))
    {
      // The search for neighbors was aborted
      si_->freeState(candidateState);
      continue;
    }

    // Benchmark
    // double time = time::seconds(time::now() - startTime);
    // totalTime_ += time;
    // totalCandidates_ ++;
    // double average = totalTime_ / totalCandidates_;
    // BOLT_MAGENTA(0, true,  time << " CandidateQueue, average: " << average << " queue: " << queue_.size()); //
    // Benchmark

    // Ensure this candidate is still valid
    if (candidateD.graphVersion_ != sparseGenerator_->getNumRandSamplesAdded())
    {
      // Expired graph
      si_->freeState(candidateState);
      continue;
    }

    // Add to queue - thread-safe
    boost::lock_guard<boost::shared_mutex> lock(candidateQueueMutex_);
    queue_.push(candidateD);
  }
}

CandidateData &CandidateQueue::getNextCandidate(std::size_t indent)
{
  BOLT_CYAN(indent, verbose_, "CandidateQueue.getNextCanidate(): queue size: " << queue_.size()
            << " num samples added: " << sparseGenerator_->getNumRandSamplesAdded());
  // This function is run in the parent thread

  // Keep looping until a non-expired candidate exists or the thread ends
  while (threadsRunning_)
  {
    // Clear all expired candidates
    {
      boost::lock_guard<boost::shared_mutex> lock(candidateQueueMutex_);
      std::size_t numCleared = 0;
      BOLT_DEBUG(indent, verbose_, "Current graph version: " << sparseGenerator_->getNumRandSamplesAdded());
      while (!queue_.empty() && queue_.front().graphVersion_ != sparseGenerator_->getNumRandSamplesAdded() &&
             threadsRunning_)
      {
        BOLT_DEBUG(indent, verbose_, "Expired candidate state with graph version: " << queue_.front().graphVersion_);

        // Next Candidate state is expired, delete
        si_->freeState(queue_.front().state_);
        queue_.pop();
        numCleared++;
      }
      BOLT_ERROR(indent, vClear_ && numCleared > 0, "Cleared " << numCleared << " states from CandidateQueue");

      // Stop seaching for expired states after we find the first one that is not expired
      if (!queue_.empty() && queue_.front().graphVersion_ == sparseGenerator_->getNumRandSamplesAdded())
      {
        break;
      }
    }

    // Do not continue until there is something in the queue
    waitForQueueNotEmpty(indent);

    // Now check that the queue isn't expired by looping through again
  }

  return queue_.front();
}

void CandidateQueue::setCandidateUsed(bool wasUsed, std::size_t indent)
{
  // Note: This function is run in the parent thread

  if (!wasUsed)  // if was used the state is now in use elsewhere, otherwise free the memory
    si_->freeState(queue_.front().state_);

  boost::lock_guard<boost::shared_mutex> lock(candidateQueueMutex_);
  queue_.pop();
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

void CandidateQueue::waitForQueueNotEmpty(std::size_t indent)
{
  // Wait for queue to not be empty
  bool oneTimeFlag = true;
  totalMisses_++;
  while (queue_.empty() && threadsRunning_)
  {
    if (oneTimeFlag)
    {
      BOLT_WARN(indent, vQueueEmpty_, "CandidateQueue: Queue is empty, waiting for next generated CandidateData");
      oneTimeFlag = false;
    }
    usleep(100);
  }
  if (!oneTimeFlag)
    BOLT_DEBUG(indent, vQueueEmpty_ && false, "CandidateQueue: No longer waiting on queue");
}

bool CandidateQueue::findGraphNeighbors(CandidateData &candidateD, std::size_t threadID, std::size_t indent)
{
  BOLT_FUNC(indent, vNeighbor_, "findGraphNeighbors() within sparse delta " << sparseCriteria_->getSparseDelta() << " state: " << candidateD.state_);

  // Get the version number of the graph, which is simply the number of states that have thus far been added
  // during this program's execution (not of all time). This allows us to know if the candidate has potentially
  // become "expired" because of a change in the graph
  candidateD.graphVersion_ = sparseGenerator_->getNumRandSamplesAdded();

  // Search in thread-safe manner
  // Note that the main thread could be modifying the NN, so we have to lock it
  sg_->getQueryStateNonConst(threadID) = candidateD.state_;
  {
    std::lock_guard<std::mutex> lock(sg_->getNNGuard());
    sg_->getNN()->nearestR(sg_->getQueryVertices(threadID), sparseCriteria_->getSparseDelta(),
                           candidateD.graphNeighborhood_);
  }
  sg_->getQueryStateNonConst(threadID) = nullptr;

  // Now that we got the neighbors from the NN, we must remove any we can't see
  for (std::size_t i = 0; i < candidateD.graphNeighborhood_.size(); ++i)
  {
    SparseVertex v2 = candidateD.graphNeighborhood_[i];

    // Check for expired graph or termination condition
    if (candidateD.graphVersion_ != sparseGenerator_->getNumRandSamplesAdded() || !threadsRunning_)
    {
      BOLT_WARN(indent, vNeighbor_, "findGraphNeighbors aborted b/c expired graph or term cond");
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

    // Check for expired graph or termination condition
    if (candidateD.graphVersion_ != sparseGenerator_->getNumRandSamplesAdded() || !threadsRunning_)
    {
      BOLT_WARN(indent, vNeighbor_, "findGraphNeighbors aborted b/c expired graph or term cond");
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
