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

#ifndef OMPL_TOOLS_BOLT_SAMPLING_QUEUE_H
#define OMPL_TOOLS_BOLT_SAMPLING_QUEUE_H

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/samplers/MinimumClearanceValidStateSampler.h>
#include <ompl/tools/debug/Visualizer.h>
#include <ompl/tools/bolt/Debug.h>
#include <ompl/tools/bolt/BoostGraphHeaders.h>

// Boost
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread.hpp>

// C++
#include <queue>

namespace ob = ompl::base;

namespace ompl
{
namespace tools
{
namespace bolt
{
OMPL_CLASS_FORWARD(SamplingQueue);
OMPL_CLASS_FORWARD(SparseGraph);
OMPL_CLASS_FORWARD(SparseCriteria);

// struct Sample
// {
//   base::State* state_;
//   bool used_; // Flag indicating whether the state needs to be garbage collected
// }

class SamplingQueue
{
public:
  /** \brief Constructor */
  SamplingQueue(SparseGraphPtr sg);

  ~SamplingQueue();

  void startSampling(std::size_t indent);

  void stopSampling(std::size_t indent);

  /** \brief This function is called from the parent thread
      \return false if queue is currently empty
   */
  bool getNextState(base::State *&state, std::size_t indent);

  void setTargetQueue(std::size_t targetQueueSize)
  {
    targetQueueSize_ = targetQueueSize;
  }

  /** \brief Auto-configure desired queue size by how many consumers will be reading from this */
  void setTargetQueueByThreads(std::size_t numThreads)
  {
    targetQueueSize_ = 20 * numThreads;
  }

private:
  void samplingThread(base::SpaceInformationPtr si, ClearanceSamplerPtr clearanceSampler, std::size_t indent);

  /** \brief Do not add more states if queue is full */
  void waitForQueueNotFull(std::size_t indent);

  /** \brief Wait until there is at least one state ready */
  void waitForQueueNotEmpty(std::size_t indent);

  SparseGraphPtr sg_;
  SparseCriteriaPtr sc_;

  /** \brief The created space information */
  base::SpaceInformationPtr si_;

  /** \brief Class for managing various visualization features */
  VisualizerPtr visual_;

  std::queue<base::State *> statesQueue_;

  // When to stop generating states - this is preferrably calculated by formulas, above
  std::size_t targetQueueSize_ = 100;

  boost::thread *samplingThread_;

  /** \brief Flag indicating sampler is active */
  bool threadRunning_ = false;

  /** \brief Mutex for only getting one sample at a time from the thread */
  boost::shared_mutex sampleQueueMutex_;

public:
  bool verbose_ = false;
  bool vQueueFull_ = false;   // when queue is full
  bool vQueueEmpty_ = false;  // when queue is empty and holding up process

};  // end of class SamplingQueue

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_BOLT_SAMPLING_QUEUE_H
