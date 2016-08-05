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
   Desc:   Speed up collision checking of StateID edges
*/

// OMPL
#include <ompl/tools/bolt/SparseGraph.h>

// Boost
#include <boost/serialization/map.hpp>
#include <boost/thread.hpp>

// C++
#include <fstream>

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
static const boost::uint32_t BOLT_DENSE_CACHE_MARKER = 0x1044414D;  // unknown value

DenseCache::DenseCache(base::SpaceInformationPtr si, SparseGraph *sparseGraph, VisualizerPtr visual)
  : si_(si), sparseGraph_(sparseGraph), visual_(visual)
{
  if (disableCache_)
    OMPL_WARN("Dense cache disabled");

  // Threading setup
  numThreads_ = boost::thread::hardware_concurrency();
  keys_.resize(numThreads_);
  totalCollisionChecks_.resize(numThreads_, 0);
  totalCollisionChecksFromCache_.resize(numThreads_, 0);

  // Init
  initialize();
}

void DenseCache::clear()
{
  OMPL_INFORM("Clearing caches");

  collisionCheckDenseCache_.clear();
  stateCache_.clear();

  // Init
  initialize();
}

void DenseCache::initialize()
{
  resetCounters();

  // Estimate the size of the state cache
  stateCache_.reserve(30000);  // based on experiments in 2D

  // Add zeroth state - which indicates a deleted/NULL vertex
  addState(si_->allocState());
}

void DenseCache::resetCounters()
{
  for (std::size_t i = 0; i < totalCollisionChecksFromCache_.size(); ++i)
  {
    totalCollisionChecks_[i] = 0;
    totalCollisionChecksFromCache_[i] = 0;
  }
}

bool DenseCache::saveIfReady()
{
  if (!enableCacheSaving_)
    return false;

  if (getNewEdgesCount() > saveEveryNEdges_)
  {
    return save();
  }
  return false;
}

bool DenseCache::save()
{
  if (disableCache_)
  {
    OMPL_INFORM("DenseCache disabled, not saving");
    return false;
  }

  if (!enableCacheSaving_)
  {
    OMPL_INFORM("DenseCache saving disabled, not saving");
    return false;
  }

  OMPL_INFORM("------------------------------------------------");
  OMPL_INFORM("Saving Dense Cache");
  OMPL_INFORM("  Path:            %s", filePath_.c_str());
  OMPL_INFORM("  Edges:");
  OMPL_INFORM("    Cache Size:    %u (new: %u)", getEdgeCacheSize(), getNewEdgesCount());
  OMPL_INFORM("    Cached checks: %u (%f %)", getTotalCollisionChecksFromCache(), getPercentCachedCollisionChecks());
  OMPL_INFORM("  States:");
  OMPL_INFORM("    Cache Size:    %u (new: %u)", getStateCacheSize(), getNewStatesCount());
  OMPL_INFORM("------------------------------------------------");

  // Save previous data
  prevNumCachedEdges_ = getEdgeCacheSize();
  prevNumCachedStates_ = getStateCacheSize();

  std::ofstream out(filePath_, std::ios::binary);

  if (!out.good())
  {
    OMPL_ERROR("Failed to save dense cache: file is invalid");
    return false;
  }

  {  // Get write mutex
    boost::lock_guard<boost::shared_mutex> writeLock(denseCacheMutex_);
    try
    {
      boost::archive::binary_oarchive oa(out);

      // Write header
      Header header;
      header.marker = BOLT_DENSE_CACHE_MARKER;
      header.vertex_count = stateCache_.size();
      header.edge_count = collisionCheckDenseCache_.size();
      si_->getStateSpace()->computeSignature(header.signature);
      oa << header;

      saveStates(oa);
      oa << collisionCheckDenseCache_;
    }
    catch (boost::archive::archive_exception &ae)
    {
      OMPL_ERROR("Failed to save dense cache: %s", ae.what());
      return false;
    }
  }

  out.close();
  return true;
}

bool DenseCache::load()
{
  if (disableCache_)
  {
    //OMPL_INFORM("DenseCache: disabled, not loading");
    return false;
  }

  if (!enableCacheSaving_)
  {
    OMPL_INFORM("DenseCache loading disabled, not loading");
    return false;
  }

  OMPL_INFORM("DenseCache: Loading from %s", filePath_.c_str());

  // Benchmark runtime
  time::point startTime = time::now();

  if (!collisionCheckDenseCache_.empty())
  {
    OMPL_ERROR("Collision check cache has %u edges and is not empty, unable to load.",
               collisionCheckDenseCache_.size());
    return false;
  }

  if (stateCache_.size() > 1)
  {
    OMPL_ERROR("State cache has %u states and is not empty, unable to load.", stateCache_.size());
    return false;
  }

  std::ifstream in(filePath_, std::ios::binary);

  // Error check
  if (!in.good())
  {
    OMPL_INFORM("Failed to load dense cache: file not found.");
    return false;
  }

  // Reset first state (id=0)
  stateCache_.clear();  // remove the null state id=0, because this should have already been saved to file

  try
  {
    boost::archive::binary_iarchive ia(in);

    // Read header
    Header header;
    ia >> header;

    // Checking the archive marker
    if (header.marker != BOLT_DENSE_CACHE_MARKER)
    {
      OMPL_ERROR("Failed to load DenseCachea: header marker not found");
      return false;
    }

    // Verify that the state space is the same
    std::vector<int> sig;
    si_->getStateSpace()->computeSignature(sig);
    if (header.signature != sig)
    {
      OMPL_ERROR("Failed to load DenseCache: StateSpace signature mismatch");
      return false;
    }

    // File seems ok - load vertices and edges
    loadStates(header.vertex_count, ia);
    ia >> collisionCheckDenseCache_;
  }
  catch (boost::archive::archive_exception &ae)
  {
    OMPL_ERROR("Failed to load dense cache: %s", ae.what());
    return false;
  }

  in.close();

  // This is really slow
  // errorCheckData();

  // Benchmark runtime
  double duration = time::seconds(time::now() - startTime);

  OMPL_INFORM("------------------------------------------------------");
  OMPL_INFORM("Loaded dense cache stats:");
  OMPL_INFORM("  Edge Cache Size:   %u", getEdgeCacheSize());
  OMPL_INFORM("  State Cache Size:  %u", getStateCacheSize());
  OMPL_INFORM("  Loading time:      %f", duration);
  OMPL_INFORM("------------------------------------------------------");

  // Save previous data
  prevNumCachedEdges_ = getEdgeCacheSize();
  prevNumCachedStates_ = getStateCacheSize();

  return true;
}

void DenseCache::saveStates(boost::archive::binary_oarchive &oa)
{
  const base::StateSpacePtr &space = si_->getStateSpace();

  std::vector<unsigned char> state(space->getSerializationLength());
  std::size_t feedbackFrequency = stateCache_.size() / 10;

  std::cout << "Saving states: " << std::flush;
  for (std::size_t stateID = 0; stateID < stateCache_.size(); ++stateID)
  {
    assert(stateCache_[stateID] != NULL);

    // Convert to new structure
    StateSerialized stateData;

    // Serializing the state contained in this vertex
    space->serialize(&state[0], stateCache_[stateID]);
    stateData.stateSerialized_ = state;

    // Save to file
    oa << stateData;

    // Feedback
    if (stateID % feedbackFrequency == 0)
      std::cout << static_cast<int>(stateID / double(stateCache_.size()) * 100.0) << "% "
                << std::flush;
  }

  std::cout << std::endl;
}

void DenseCache::loadStates(unsigned int numStates, boost::archive::binary_iarchive &ia)
{
  const base::StateSpacePtr &space = si_->getStateSpace();
  std::size_t feedbackFrequency = numStates / 10;

  std::cout << "         States loaded: ";
  for (unsigned int i = 0; i < numStates; ++i)
  {
    // Copy in data from file
    StateSerialized stateData;
    ia >> stateData;

    // Allocating a new state and deserializing it from the buffer
    base::State *state = space->allocState();
    space->deserialize(state, &stateData.stateSerialized_[0]);

    // Add to vector
    stateCache_.push_back(state);

    // Feedback
    if ((i + 1) % feedbackFrequency == 0)
      std::cout << static_cast<int>((i / double(numStates)) * 100.0) << "% " << std::flush;
  }
  std::cout << std::endl;
}

StateID DenseCache::addState(base::State *state)
{
  // TODO: thread safety?
  stateCache_.push_back(state);
  StateID id = stateCache_.size() - 1;

  // Check if it is time for a save
  saveIfReady();

  return id;
}

const base::State *DenseCache::getState(StateID stateID) const
{
  // TODO: thread safety?
  return stateCache_[stateID];
}

base::State *&DenseCache::getStateNonConst(StateID stateID)
{
  // TODO: thread safety?
  return stateCache_[stateID];
}

bool DenseCache::checkMotionWithCacheVertex(const SparseVertex &v1, const SparseVertex &v2, const std::size_t &threadID)
{
  return checkMotionWithCache(sparseGraph_->getStateID(v1), sparseGraph_->getStateID(v2), threadID);
}

bool DenseCache::checkMotionWithCache(const StateID &stateID1, const StateID &stateID2, const std::size_t &threadID)
{
  // Optionally skip caching
  if (disableCache_)
    return si_->checkMotion(stateCache_[stateID1], stateCache_[stateID2]);

  CachedEdge &edge = keys_[threadID];

  // Error check
  // BOOST_ASSERT_MSG(stateID1 >= numThreads_, "stateID1: The queryVertex_ should not be checked within the DenseCache,
  // because it is subject to change");
  // BOOST_ASSERT_MSG(stateID2 >= numThreads_, "stateID2: The queryVertex_ should not be checked within the DenseCache,
  // because it is subject to change");

  // Create edge to search for - only store pairs in one direction
  if (stateID1 < stateID2)
    edge = CachedEdge(stateID1, stateID2);
  else
    edge = CachedEdge(stateID2, stateID1);

  EdgeCacheMap::iterator lb = collisionCheckDenseCache_.lower_bound(edge);

  bool result;
  {  // Get read-only mutex
    boost::shared_lock<boost::shared_mutex> readLock(denseCacheMutex_);

    // Search for existing key
    result = (lb != collisionCheckDenseCache_.end() && !(collisionCheckDenseCache_.key_comp()(edge, lb->first)));

    // Statistics
    totalCollisionChecks_[threadID]++;
  }

  if (result)  // Cache available
  {
    totalCollisionChecksFromCache_[threadID]++;
    return lb->second;
  }

  // No cache available
  result = si_->checkMotion(stateCache_[stateID1], stateCache_[stateID2]);

  {  // Get write mutex
    boost::lock_guard<boost::shared_mutex> writeLock(denseCacheMutex_);

    // The key does not exist in the map, so add it to the map
    // Use lb as a hint to insert, so it can avoid another lookup
    collisionCheckDenseCache_.insert(lb, EdgeCacheMap::value_type(edge, result));
  }

  return result;
}

void DenseCache::checkMotionCacheBenchmark()
{
  std::cout << std::endl;
  std::cout << "-------------------------------------------------------" << std::endl;
  OMPL_WARN("Running check motion cache benchmark");
  std::cout << std::endl;

  // Test new checkMotionWithCache
  StateID stateID1 = 2;
  StateID stateID2 = 3;
  std::size_t numRuns = 100000;
  bool baselineResult = si_->checkMotion(stateCache_[stateID1], stateCache_[stateID2]);

  // Benchmark collision check without cache
  {
    // Benchmark runtime
    time::point startTime = time::now();

    for (std::size_t i = 0; i < numRuns; ++i)
    {
      if (si_->checkMotion(stateCache_[stateID1], stateCache_[stateID2]) != baselineResult)
      {
        OMPL_ERROR("benchmark checkmotion does not match baseline result");
        exit(-1);
      }
    }

    // Benchmark runtime
    double duration = time::seconds(time::now() - startTime);
    OMPL_INFORM(" - no cache took %f seconds (%f hz)", duration, 1.0 / duration);
  }

  // Benchmark collision check with cache
  {
    // Benchmark runtime
    time::point startTime = time::now();

    const std::size_t threadID = 0;
    for (std::size_t i = 0; i < numRuns / 2; ++i)
    {
      if (checkMotionWithCache(stateID1, stateID2, threadID) != baselineResult)
      {
        OMPL_ERROR("benchmark checkmotion does not match baseline result");
        exit(-1);
      }

      if (checkMotionWithCache(stateID2, stateID1, threadID) != baselineResult)
      {
        OMPL_ERROR("benchmark checkmotion does not match baseline result");
        exit(-1);
      }
    }
    OMPL_INFORM("Cache size: %u", collisionCheckDenseCache_.size());

    // Benchmark runtime
    double duration = time::seconds(time::now() - startTime);
    OMPL_INFORM(" - with cache took %f seconds (%f hz)", duration, 1.0 / duration);
  }
}

void DenseCache::errorCheckData()
{
  OMPL_INFORM("Error checking dense cache...");
  std::size_t counter = 0;
  for (EdgeCacheMap::const_iterator iterator = collisionCheckDenseCache_.begin();
       iterator != collisionCheckDenseCache_.end(); iterator++)
  {
    std::pair<StateID, StateID> thing = iterator->first;
    StateID &stateID1 = thing.first;
    StateID &stateID2 = thing.second;
    bool cachedResult = iterator->second;
    if (si_->checkMotion(stateCache_[stateID1], stateCache_[stateID2]) != cachedResult)
    {
      OMPL_ERROR("Found instance where cached edge data is wrong, on iteration %u", counter);
      std::cout << "stateID1: " << stateID1 << std::endl;
      std::cout << "stateID2: " << stateID2 << std::endl;
      exit(-1);
    }
    if (counter++ % 1000 == 0)
      std::cout << "Checking edge " << counter << std::endl;
  }
}

void DenseCache::setFilePath(const std::string &filePath)
{
  filePath_ = filePath;
}

std::size_t DenseCache::getStateCacheSize()
{
  return stateCache_.size();
}

std::size_t DenseCache::getNewStatesCount()
{
  return getStateCacheSize() - prevNumCachedStates_;
}

std::size_t DenseCache::getEdgeCacheSize()
{
  return collisionCheckDenseCache_.size();
}

std::size_t DenseCache::getNewEdgesCount()
{
  return getEdgeCacheSize() - prevNumCachedEdges_;
}

std::size_t DenseCache::getTotalCollisionChecksFromCache()
{
  std::size_t total = 0;
  for (std::size_t i = 0; i < totalCollisionChecksFromCache_.size(); ++i)
  {
    total += totalCollisionChecksFromCache_[i];
  }
  return total;
}

std::size_t DenseCache::getTotalCollisionChecks()
{
  std::size_t total = 0;
  for (std::size_t i = 0; i < totalCollisionChecks_.size(); ++i)
  {
    total += totalCollisionChecks_[i];
  }
  return total;
}

double DenseCache::getPercentCachedCollisionChecks()
{
  std::size_t totalCollisionChecks = getTotalCollisionChecks();
  if (totalCollisionChecks == 0)
    return 0;

  return static_cast<double>(getTotalCollisionChecksFromCache()) / totalCollisionChecks * 100.0;
}

void DenseCache::print()
{
  std::size_t indent = 0;
  BOLT_DEBUG(indent, 1, "Contents of DenseCache States:");
  for (base::State *state : stateCache_)
  {
    si_->printState(state, std::cout);
  }
}

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
