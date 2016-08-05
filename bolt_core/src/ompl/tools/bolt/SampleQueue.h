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

#ifndef OMPL_TOOLS_BOLT_SAMPLE_QUEUE_H
#define OMPL_TOOLS_BOLT_SAMPLE_QUEUE_H

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/tools/bolt/BoostGraphHeaders.h>
#include <ompl/base/samplers/MinimumClearanceValidStateSampler.h>

// Boost
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread.hpp>
#include <boost/thread/shared_mutex.hpp>

// C++
#include <queue>

namespace ompl
{
namespace tools
{
namespace bolt
{
OMPL_CLASS_FORWARD(SampleQueue);
OMPL_CLASS_FORWARD(SparseGraph);

// struct Sample
// {
//   base::State* state_;
//   bool used_; // Flag indicating whether the state needs to be garbage collected
// }

class SampleQueue
{
public:
  /** \brief Constructor */
  SampleQueue(base::SpaceInformationPtr si, VisualizerPtr visual,
              base::MinimumClearanceValidStateSamplerPtr clearanceSampler)
    : si_(si), visual_(visual), clearanceSampler_(clearanceSampler)
  {
    // statesQueue_.reserve(targetQueueSize_);
  }

  ~SampleQueue()
  {
    // Clear all left over states that weren't used
    for (std::size_t i = 0; i < recycling_.size(); ++i)
    {
      si_->freeState(recycling_[i]);
    }
  }

  void startSampling(std::size_t indent)
  {
    BOLT_FUNC(indent, true, "startSampling() Starting sampling thread");
    if (threadRunning_)
    {
      BOLT_ERROR(indent, true, "SampleQueue already running");
      return;
    }
    threadRunning_ = true;

    // Setup
    base::SpaceInformationPtr si(new base::SpaceInformation(si_->getStateSpace()));
    si->setStateValidityChecker(si_->getStateValidityChecker());
    si->setMotionValidator(si_->getMotionValidator());

    // Create thread
    samplingThread_ = new boost::thread(boost::bind(&SampleQueue::samplingThread, this, si, indent));

    // Wait for first sample to be found
    BOLT_DEBUG(indent, true, "Waiting for first sample to be found");
    while (statesQueue_.empty())
    {
      usleep(0.001 * 1000000);
    }
  }

  void stopSampling(std::size_t indent)
  {
    BOLT_FUNC(indent, true, "stopSampling() Stopping sampling thread");
    threadRunning_ = false;

    // End thread
    samplingThread_->join();
  }

  /** \brief This function is called from the parent thread */
  base::State *getNextState(std::size_t indent)
  {
    if (statesQueue_.empty())
      waitForQueueNotEmpty(indent + 2);

    return statesQueue_.front();
  }

  /** \brief This function is called from the parent thread */
  void setNextStateUsed(bool wasUsed)
  {
    // If state was not used, recycle
    if (!wasUsed)
      recycleState(statesQueue_.front());

    std::cout << "statesQueue_.size(): " << statesQueue_.size() << std::endl;
    std::cout << "before statesQueue_.front(): " << statesQueue_.front() << std::endl;
    statesQueue_.pop();
    std::cout << "after  statesQueue_.front(): " << statesQueue_.front() << std::endl;
  }

  /** \brief This function is called from the parent thread */
  void recycleState(base::State *state)
  {
    {  // Get write mutex
      boost::lock_guard<boost::shared_mutex> writeLock(recyclingMutex_);
      recycling_.push_back(state);
    }
  }

private:
  void samplingThread(base::SpaceInformationPtr si, std::size_t indent)
  {
    BOLT_FUNC(indent, true, "samplingThread()");

    while (threadRunning_ && !visual_->viz1()->shutdownRequested())
    {
      // Do not add more states if queue is full
      waitForQueueNotFull(indent);

      // Create new state or recycle one
      base::State *candidateState;

      // Attempt to reuse a state
      if (!getRecycledState(candidateState))
      {
        candidateState = si_->allocState();
      }

      // Sample randomly
      if (!clearanceSampler_->sample(candidateState))
      {
        OMPL_ERROR("Unable to find valid sample");
        exit(-1);  // this should never happen
      }

      statesQueue_.push(candidateState);
    }
  }

  bool getRecycledState(base::State *&unusedState)
  {
    // If none found, copy in any available states from parent thread's cache
    if (!recycling_.empty())
    {
      // std::cout << "emptying parent thread's recycling cache " << std::endl;
      {  // Get write mutex
        boost::lock_guard<boost::shared_mutex> writeLock(recyclingMutex_);
        unusedState = recycling_.back();
        recycling_.pop_back();
      }
      BOLT_DEBUG(0, true, "using recycled state");
      return true;
    }
    return false;  // no recycled state is available
  }

  /** \brief Do not add more states if queue is full */
  void waitForQueueNotFull(std::size_t indent)
  {
    bool oneTimeFlag = true;
    while (statesQueue_.size() >= targetQueueSize_ && threadRunning_)
    {
      if (oneTimeFlag)
      {
        BOLT_DEBUG(indent, vStatus_, "SampleQueue: Queue is full, sampler is waiting");
        oneTimeFlag = false;
      }
      usleep(0.001 * 1000000);
    }
    if (!oneTimeFlag)
      BOLT_DEBUG(indent, vStatus_, "SampleQueue: No longer waiting on full queue");
  }

  /** \brief Wait until there is at least one state ready */
  void waitForQueueNotEmpty(std::size_t indent)
  {
    bool oneTimeFlag = true;
    while (statesQueue_.empty() && threadRunning_)
    {
      if (oneTimeFlag)
      {
        BOLT_WARN(indent, true, "SampleQueue: Queue is empty, waiting to provide State");
        oneTimeFlag = false;
      }
      usleep(0.001 * 1000000);
    }
    if (!oneTimeFlag)
      BOLT_DEBUG(indent, false, "SampleQueue: No longer waiting on queue");
  }

private:
  /** \brief The created space information */
  base::SpaceInformationPtr si_;

  /** \brief Class for managing various visualization features */
  VisualizerPtr visual_;

  std::queue<base::State *> statesQueue_;
  std::vector<base::State *> recycling_;

  std::size_t targetQueueSize_ = 100;

  boost::thread *samplingThread_;

  /** \brief Sampler user for generating valid samples in the state space */
  base::MinimumClearanceValidStateSamplerPtr clearanceSampler_;

  /** \brief Flag indicating sampler is active */
  bool threadRunning_ = false;

  /** \brief Mutex for  */
  boost::shared_mutex recyclingMutex_;

  /** \brief Mutex for only getting one sample at a time from the thread */
  boost::shared_mutex sampleQueueMutex_;

public:
  bool vStatus_ = false;

};  // end of class SampleQueue

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_BOLT_SAMPLE_QUEUE_H
