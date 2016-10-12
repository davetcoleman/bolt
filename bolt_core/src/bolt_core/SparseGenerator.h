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

#ifndef OMPL_TOOLS_BOLT_SPARSE_GENERATOR_
#define OMPL_TOOLS_BOLT_SPARSE_GENERATOR_

// OMPL
#include <bolt_core/SparseGraph.h>
#include <bolt_core/CandidateQueue.h>

namespace ompl
{
namespace tools
{
namespace bolt
{
/// @cond IGNORE
OMPL_CLASS_FORWARD(SparseGenerator);
/// @endcond

/** \class ompl::tools::bolt::::SparseGeneratorPtr
    \brief A boost shared pointer wrapper for ompl::tools::SparseGenerator */

class SparseGenerator
{
public:
  /** \brief Constructor needs the state space used for planning.
   */
  SparseGenerator(SparseGraphPtr sg);

  /** \brief Deconstructor */
  virtual ~SparseGenerator();

  /** \brief Reset class */
  void clear();

  /** \brief Give the sparse graph reference to the criteria, because sometimes it needs data from there */
  void setSparseCriteria(SparseCriteriaPtr sparseCriteria)
  {
    sparseCriteria_ = sparseCriteria;
  }

  /** \brief Initialize sparse parameters */
  bool setup(std::size_t indent);

  /** \brief Create a SPARS graph */
  void createSPARS();

  void copyPasteState(std::size_t numSets = 0);
  void dumpLog();

  void setMapName(std::string mapName)
  {
    mapName_ = mapName;
  }

  double getLastGraphGenerationTime()
  {
    return lastGraphGenerationTime_;
  }

  void addDiscretizedStates(std::size_t indent);

  /** \brief Randomly sample */
  bool addRandomSamplesOneThread(std::size_t indent);
  bool addRandomSamplesThreaded(std::size_t indent);

  /**
   * \brief Add state to sparse graph
   * \param stateID representing a pre-populate state
   * \return true if sparse graph is still accepting states, false if the sparse graph has completed
   */
  bool addSample(base::State *state, std::size_t threadID, bool &usedState, std::size_t indent);
  bool addSample(CandidateData &candidateD, std::size_t threadID, bool &usedState, std::size_t indent);

  void showStatus(std::size_t indent);
  void showNoQualityStatus(std::size_t indent);

  /**
   * \brief Get neighbors within sparseDelta radius
   * \param indent - debugging tool
   */
  void findGraphNeighbors(CandidateData &candidateD, std::size_t threadID, std::size_t indent);
  void findGraphNeighbors(CandidateData &candidateD, double distance, std::size_t threadID, std::size_t indent);

  /**
   * \brief Tests for ensuring the generated roadmap obeys the theoretical guarantees
   */
  bool checkGraphOptimality(std::size_t indent);

  /** \brief Helper for debugging specific issue */
  void debugNoNeighbors(CandidateData &point, std::size_t indent);

  bool convertVertexPathToStatePath(std::vector<SparseVertex> &vertexPath, const base::State *actualStart,
                                    const base::State *actualGoal, geometric::PathGeometric &geometricSolution);

  /** \brief Stop and reset the candidate queue because it uses the nearest neighbors and will have bad vertices stored
   */
  void stopCandidateQueueAndSave(std::size_t indent);

  void benchmarkValidClearanceSampler(std::size_t indent = 0);
  void benchmarkRandValidSampling(std::size_t indent = 0);
  void benchmarkVisualizeSampling(std::size_t indent = 0);
  void benchmarkSparseGraphGeneration(std::size_t indent = 0);
  void benchmarkMemoryAllocation(std::size_t indent = 0);

  /** \brief Getter for vertexDiscretizer */
  VertexDiscretizerPtr &getVertexDiscretizer()
  {
    return vertexDiscretizer_;
  }

  std::size_t getNumRandSamplesAdded()
  {
    return numRandSamplesAdded_;
  }

  CandidateQueuePtr getCandidateQueue()
  {
    return candidateQueue_;
  }

  double getAvgPlanTime()
  {
    double sum = std::accumulate(avgPlanTime_.begin(), avgPlanTime_.end(), 0.0);
    double mean = sum / avgPlanTime_.size();
    avgPlanTime_.clear();  // reset for next run
    return mean;
  }

  double getAvgPathQuality()
  {
    double sum = std::accumulate(avgPathQuality_.begin(), avgPathQuality_.end(), 0.0);
    double mean = sum / avgPathQuality_.size();
    avgPathQuality_.clear();  // reset for next run
    return mean;
  }

protected:
  /** \brief Short name of this class */
  const std::string name_ = "SparseGenerator";

  /** \brief Sparse graph main datastructure that this class operates on */
  SparseGraphPtr sg_;

  SparseCriteriaPtr sparseCriteria_;

  /** \brief The created space information */
  base::SpaceInformationPtr si_;

  /** \brief Class for managing various visualization features */
  VisualizerPtr visual_;

  /** \brief Sampler user for generating valid samples in the state space */
  base::ValidStateSamplerPtr sampler_;

  /** \brief Multiple threads for finding nearest neighbors from samples */
  CandidateQueuePtr candidateQueue_;

  std::size_t numConsecutiveFailures_;
  std::size_t maxConsecutiveFailures_ = 0;  // find the closest to completion the process has gotten
  std::size_t maxPercentComplete_;          // the whole number percentage presented to user

  VertexDiscretizerPtr vertexDiscretizer_;

  /** \brief For statistics */
  std::size_t numRandSamplesAdded_ = 0;
  time::point timeRandSamplesStarted_;  // calculate rate at which the graph is being built
  time::point timeDiscretizeAndRandomStarted_;

  // All for benchmarking / testing
  std::vector<std::string> stringLog_;
  std::string mapName_;                 // meta data for the logging
  double lastGraphGenerationTime_ = 0;  // store how long it took to generate the last SPARS2 graph
  std::vector<double> avgPlanTime_;
  std::vector<double> avgPathQuality_;

  /** \brief How often to save */
  std::size_t saveInterval_ = 10;

public:
  bool verbose_ = false;
  bool vGuarantees_ = false;
  bool vFindGraphNeighbors_ = false;

  /** \brief Number of failed state insertion attempts before stopping the algorithm */
  std::size_t terminateAfterFailures_ = 1000;

  /** \brief Number of failed state insertion attempts before starting to apply the fourth quality criteria from SPARS
   */
  std::size_t fourthCriteriaAfterFailures_ = 500;

  /** \brief Generate the Sparse graph with discretized and/or random samples */
  bool useDiscretizedSamples_;
  bool useRandomSamples_;
  bool verifyGraphProperties_ = false;

};  // end SparseGenerator

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_BOLT_SPARSE_GENERATOR_
