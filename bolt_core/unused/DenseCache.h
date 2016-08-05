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
   Desc:   Speed up collision checking of SparseVertex edges
*/

#ifndef OMPL_TOOLS_BOLT_DENSE_CACHE_
#define OMPL_TOOLS_BOLT_DENSE_CACHE_

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/tools/bolt/BoostGraphHeaders.h>

// Boost
#include <boost/thread/shared_mutex.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

// C++
#include <map>

namespace ompl
{
namespace tools
{
namespace bolt
{
OMPL_CLASS_FORWARD(DenseCache);
OMPL_CLASS_FORWARD(SparseGraph);

typedef base::State *CachedState;
typedef std::vector<CachedState> StateCache;
typedef std::pair<StateID, StateID> CachedEdge;
typedef std::map<CachedEdge, bool> EdgeCacheMap;

class DenseCache
{
public:
  /** \brief Constructor */
  DenseCache(base::SpaceInformationPtr si, SparseGraph *sparseGraph, VisualizerPtr visual);

  /** \brief Clear the edge cache completely */
  void clear();

  /** \brief Setup */
  void initialize();

  /** \brief Reset the counters */
  void resetCounters();

  /** \brief Save at an interval to the new edges that have been added */
  bool saveIfReady();

  /** \brief Save cache to file */
  bool save();

  /** \brief Load cache from file */
  bool load();

  void saveStates(boost::archive::binary_oarchive &oa);
  void loadStates(unsigned int numStates, boost::archive::binary_iarchive &ia);

  /** \brief Add a state to the cache */
  StateID addState(base::State *state);

  /** \brief Get a state from the cache */
  const base::State *getState(StateID stateID) const;
  base::State *&getStateNonConst(StateID stateID);

  /** \brief Returns true if motion is valid, false if in collision. Checks cache first and also stores result  */
  bool checkMotionWithCacheVertex(const SparseVertex &v1, const SparseVertex &v2, const std::size_t &threadID);

  bool checkMotionWithCache(const StateID &v1, const StateID &v2, const std::size_t &threadID);

  /** \brief Test for benchmarking cache */
  void checkMotionCacheBenchmark();

  /** \brief Ensure that the collision edge data loaded from file is correct, for debuging */
  void errorCheckData();

  void setFilePath(const std::string &filePath);

  std::size_t getStateCacheSize();
  std::size_t getNewStatesCount();
  std::size_t getEdgeCacheSize();
  std::size_t getNewEdgesCount();

  std::size_t getTotalCollisionChecksFromCache();

  std::size_t getTotalCollisionChecks();

  /** \brief Get stats */
  double getPercentCachedCollisionChecks();

  void print();

private:
  /* \brief Information stored at the beginning of the DenseCache archive */
  struct Header
  {
    /* \brief Bolt specific marker (fixed value) */
    boost::uint32_t marker;

    /* \brief Number of vertices stored in the archive */
    std::size_t vertex_count;

    /* \brief Number of edges stored in the archive */
    std::size_t edge_count;

    /* \brief Signature of state space that allocated the saved states in the vertices (see */
    /* ompl::base::StateSpace::computeSignature()) */
    std::vector<int> signature;

    /* \brief boost::serialization routine  */
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int /*version*/)
    {
      ar &marker;
      ar &vertex_count;
      ar &edge_count;
      ar &signature;
    }
  };

  /// \brief The object containing all vertex data that will be stored
  struct StateSerialized
  {
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int /*version*/)
    {
      ar &stateSerialized_;
    }

    std::vector<unsigned char> stateSerialized_;
  };

  /** \brief The created space information */
  base::SpaceInformationPtr si_;

  /** \brief The database of motions to search through */
  SparseGraph *sparseGraph_;

  /** \brief Class for managing various visualization features */
  VisualizerPtr visual_;

  /** \brief Cache previously performed collision checks */
  EdgeCacheMap collisionCheckDenseCache_;

  /** \brief Cache sampled states */
  StateCache stateCache_;

  /** \brief Stats for the checkMotionWithCache feature */
  std::vector<std::size_t> totalCollisionChecks_;
  std::vector<std::size_t> totalCollisionChecksFromCache_;

  /** \brief Where to store the cache on disk */
  std::string filePath_;

  /** \brief Mutex for both reading or writing to the EdgeCacheMap */
  boost::shared_mutex denseCacheMutex_;

  std::vector<CachedEdge> keys_;

  /** \brief Available cores */
  std::size_t numThreads_;

  /** \brief Number of cached item at last load/save */
  std::size_t prevNumCachedEdges_ = 0;
  std::size_t prevNumCachedStates_ = 0;

public:
  /** \brief Force all collision checks to be performed from scratch */
  bool disableCache_ = false;

  /** \brief Use cache but do not save or load from file */
  bool enableCacheSaving_ = true;

  std::size_t saveEveryNEdges_ = 50000;
};  // end of class DenseCache

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_BOLT_DENSE_CACHE_
