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

#ifndef OMPL_TOOLS_BOLT_CANDIDATE_QUEUE_H
#define OMPL_TOOLS_BOLT_CANDIDATE_QUEUE_H

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <bolt_core/BoostGraphHeaders.h>

// BOLT
#include <bolt_core/SparseGraph.h>

// Boost
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread.hpp>

// C++
#include <queue>
#include <thread>

namespace ompl
{
namespace tools
{
namespace bolt
{
OMPL_CLASS_FORWARD(CandidateQueue);
// OMPL_CLASS_FORWARD(SparseGraph);
OMPL_CLASS_FORWARD(SparseCriteria);
OMPL_CLASS_FORWARD(SparseGenerator);

class CandidateQueue
{
public:
  /** \brief Constructor */
  CandidateQueue(SparseGraphPtr sg, SparseGenerator *sparseGenerator);

  ~CandidateQueue();

  /** \brief Reset class */
  void clear();

  void startGenerating(std::size_t indent);

  void stopGenerating(std::size_t indent);

  /** \brief This function is called from the parent thread */
  CandidateData &getNextCandidate(std::size_t indent);

  /** \brief This function is called from the parent thread */
  void setCandidateUsed(bool wasUsed, std::size_t indent);

  std::size_t getTotalMisses()
  {
    return totalMisses_;
  }

private:
  void generatingThread(std::size_t threadID, base::SpaceInformationPtr si, base::ValidStateSamplerPtr sampler,
                        std::size_t indent);

  /** \brief Do not add more states if queue is full */
  void waitForQueueNotFull(std::size_t indent);
  void waitForQueueNotEmpty(std::size_t indent);

  bool findGraphNeighbors(CandidateData &candidateD, std::size_t threadID, std::size_t indent);

  /** \brief Short name of this class */
  const std::string name_ = "CandidateQueue";

  SparseGraphPtr sg_;
  SparseCriteriaPtr sparseCriteria_;
  SparseGenerator *sparseGenerator_;

  /** \brief The created space information */
  base::SpaceInformationPtr si_;

  /** \brief Class for managing various visualization features */
  VisualizerPtr visual_;

  std::queue<CandidateData> queue_;

  std::size_t targetQueueSize_ = 20;

  std::vector<boost::thread *> generatorThreads_;

  /** \brief Mutex for adding to the queue */
  boost::shared_mutex candidateQueueMutex_;

  /** \brief Flag indicating generator is active */
  bool threadsRunning_ = false;

  std::size_t numThreads_ = 1;
  std::size_t totalMisses_ = 0;

  std::size_t totalMissesOverAllResets_ = 0;
  std::size_t totalResets_ = 0;

  // Compute average time to do candidate queue
  double totalTime_ = 0;
  std::size_t totalCandidates_ = 0;

public:
  bool verbose_ = false;      // general program direction
  bool vNeighbor_ = false;    // nearest neighbor search
  bool vClear_ = false;       // when queue is being cleared because of change
  bool vQueueFull_ = false;   // alert when queue is full and waiting
  bool vQueueEmpty_ = false;  // alert when queue is empty and holding up process
  bool vThread_ = false;

};  // end of class CandidateQueue

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_BOLT_CANDIDATE_QUEUE_H
