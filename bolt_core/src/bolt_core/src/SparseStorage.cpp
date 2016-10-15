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
   Desc:   Save and load from file
*/

// OMPL
#include <bolt_core/SparseStorage.h>
#include <bolt_core/SparseGraph.h>
#include <bolt_core/BoostGraphHeaders.h>

// Boost
#include <boost/foreach.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

// Profiling
#include <valgrind/callgrind.h>

#define foreach BOOST_FOREACH

namespace ompl
{
namespace tools
{
namespace bolt
{
SparseStorage::SparseStorage(const base::SpaceInformationPtr &si, SparseGraph *sparseGraph)
  : si_(si), sparseGraph_(sparseGraph)
{
  numQueryVertices_ = boost::thread::hardware_concurrency();
}

void SparseStorage::save(const std::string &filePath, std::size_t indent)
{
  indent += 2;
  std::size_t diffEdges = sparseGraph_->getNumEdges() - prevNumEdges_;
  std::size_t diffVertices = sparseGraph_->getNumVertices() - prevNumVertices_;

  BOLT_INFO(indent, true, "------------------------------------------------");
  BOLT_INFO(indent, true, "Saving Sparse Graph");
  BOLT_INFO(indent, true, "  Path:     " << filePath.c_str());
  BOLT_INFO(indent, true, "  Edges:    " << sparseGraph_->getNumEdges() << " (Change: " << diffEdges << ")");
  BOLT_INFO(indent, true, "  Vertices: " << sparseGraph_->getNumVertices() << " (Change: " << diffVertices << ")");
  BOLT_INFO(indent, true, "------------------------------------------------");

  std::ofstream out(filePath.c_str(), std::ios::binary);

  // Save previous graph size
  prevNumEdges_ = sparseGraph_->getNumEdges();
  prevNumVertices_ = sparseGraph_->getNumVertices();

  save(out);
  out.close();

  // Log the graph size
  std::ofstream loggingFile;                              // open to append
  loggingFile.open(loggingPath_.c_str(), std::ios::out);  // no append | std::ios::app);
  loggingFile << sparseGraph_->getNumEdges() << ", " << sparseGraph_->getNumVertices() << std::endl;
  loggingFile.close();
}

void SparseStorage::save(std::ostream &out)
{
  if (!out.good())
  {
    OMPL_ERROR("Failed to save BoltData: output stream is invalid");
    return;
  }

  try
  {
    boost::archive::binary_oarchive oa(out);

    // Writing the header
    Header h;
    h.marker = OMPL_PLANNER_DATA_ARCHIVE_MARKER;
    h.vertex_count = sparseGraph_->getNumVertices() - numQueryVertices_;
    h.edge_count = sparseGraph_->getNumEdges();
    si_->getStateSpace()->computeSignature(h.signature);
    oa << h;

    saveVertices(oa);
    saveEdges(oa);
  }
  catch (boost::archive::archive_exception &ae)
  {
    OMPL_ERROR("Failed to save BoltData: %s", ae.what());
  }
}

void SparseStorage::saveVertices(boost::archive::binary_oarchive &oa)
{
  const base::StateSpacePtr &space = si_->getStateSpace();

  std::vector<unsigned char> state(space->getSerializationLength());
  std::size_t feedbackFrequency = sparseGraph_->getNumVertices() / 10;

  std::cout << "         Saving vertices: " << std::flush;
  std::size_t count = 0;
  std::size_t errorCheckNumQueryVertices = 0;
  foreach (const SparseVertex v, boost::vertices(sparseGraph_->getGraph()))
  {
    // Skip the query vertex that is nullptr
    if (v < sparseGraph_->getNumQueryVertices())
    {
      errorCheckNumQueryVertices++;
      continue;
    }

    // Check that the vertex wasn't deleted
    BOLT_ASSERT(sparseGraph_->getStateNonConst(v) != NULL, "State is null");

    // Convert to new structure
    BoltVertexData vertexData;

    // Serializing the state contained in this vertex
    space->serialize(&state[0], sparseGraph_->getStateNonConst(v));
    vertexData.stateSerialized_ = state;

    // Save to file
    oa << vertexData;

    // Feedback
    if ((++count) % feedbackFrequency == 0)
      std::cout << static_cast<int>(count / double(sparseGraph_->getNumVertices()) * 100.0) << "% " << std::flush;
  }
  BOLT_ASSERT(errorCheckNumQueryVertices == numQueryVertices_, "There should be the same number of query vertex "
              "as threads that were skipped while saving");

  std::cout << std::endl;
}

void SparseStorage::saveEdges(boost::archive::binary_oarchive &oa)
{
  std::size_t feedbackFrequency = std::max(10.0, sparseGraph_->getNumEdges() / 10.0);

  std::cout << "         Saving edges: " << std::flush;
  std::size_t count = 1;
  foreach (const SparseEdge e, boost::edges(sparseGraph_->getGraph()))
  {
    const SparseVertex v1 = boost::source(e, sparseGraph_->getGraph());
    const SparseVertex v2 = boost::target(e, sparseGraph_->getGraph());

    // Convert to new structure
    BoltEdgeData edgeData;
    // Note: we increment all vertex indexes by the number of queryVertices_ because [0,11] is in use
    edgeData.endpoints_.first = v1 - numQueryVertices_;
    edgeData.endpoints_.second = v2 - numQueryVertices_;

    // Other properties
    edgeData.weight_ = sparseGraph_->getEdgeWeightProperty(e);

    // Copy to file
    oa << edgeData;

    // Feedback
    if ((++count) % feedbackFrequency == 0)
      std::cout << static_cast<int>(count / double(sparseGraph_->getNumEdges()) * 100.0) << "% " << std::flush;

  }  // for each edge
  std::cout << std::endl;
}

bool SparseStorage::load(const std::string &filePath, std::size_t indent)
{
  BOLT_INFO(indent, true, "------------------------------------------------");
  BOLT_INFO(indent, true, "SparseStorage: Loading Sparse Graph");
  indent += 2;
  BOLT_INFO(indent, true, "Path: " << filePath.c_str());

  // Error checking
  if (sparseGraph_->getNumEdges() > numQueryVertices_ ||
      sparseGraph_->getNumVertices() > numQueryVertices_)  // the search verticie may already be there
  {
    BOLT_INFO(indent, true, "Database is not empty, unable to load from file");
    return false;
  }
  if (filePath.empty())
  {
    OMPL_ERROR("Empty filename passed to save function");
    return false;
  }
  if (!boost::filesystem::exists(filePath))
  {
    BOLT_INFO(indent, true, "Database file does not exist: " << filePath.c_str());
    return false;
  }

  // Disable visualizations while loading
  bool visualizeSparseGraph = sparseGraph_->visualizeSparseGraph_;
  sparseGraph_->visualizeSparseGraph_ = false;

  // Open file stream
  std::ifstream in(filePath.c_str(), std::ios::binary);
  bool result = load(in, indent);
  in.close();

  // Re-enable visualizations
  sparseGraph_->visualizeSparseGraph_ = visualizeSparseGraph;

  // Save previous graph size
  prevNumEdges_ = sparseGraph_->getNumEdges();
  prevNumVertices_ = sparseGraph_->getNumVertices();

  return result;
}

bool SparseStorage::load(std::istream &in, std::size_t indent)
{
  BOLT_ASSERT(!sparseGraph_->visualizeSparseGraph_, "Visualizations should be off when loading sparse graph");

  if (!in.good())
  {
    OMPL_ERROR("Failed to load BoltData: input stream is invalid");
    return false;
  }

  try
  {
    boost::archive::binary_iarchive ia(in);

    // Read the header
    Header h;
    ia >> h;

    // Checking the archive marker
    if (h.marker != OMPL_PLANNER_DATA_ARCHIVE_MARKER)
    {
      OMPL_ERROR("Failed to load BoltData: BoltData archive marker not found");
      return false;
    }

    // Verify that the state space is the same
    std::vector<int> sig;
    si_->getStateSpace()->computeSignature(sig);
    if (h.signature != sig)
    {
      OMPL_ERROR("Failed to load BoltData: StateSpace signature mismatch");
      return false;
    }

    // Pre-allocate memory in graph
    sparseGraph_->getGraphNonConst().m_vertices.resize(h.vertex_count);
    sparseGraph_->getGraphNonConst().m_edges.resize(h.edge_count);

    // Read from file
    loadVertices(h.vertex_count, ia, indent);
    loadEdges(h.edge_count, ia, indent);
  }
  catch (boost::archive::archive_exception &ae)
  {
    OMPL_ERROR("Failed to load BoltData: %s", ae.what());
  }

  return true;
}

void SparseStorage::loadVertices(std::size_t numVertices, boost::archive::binary_iarchive &ia, std::size_t indent)
{
  BOLT_FUNC(indent, true, "Loading vertices from file: " << numVertices);

  // Reserve memory
  sparseGraph_->getGraphNonConst().m_vertices.reserve(numVertices);

  // Create thread to populate nearest neighbor structure, because that is the slowest component
  loadVerticesFinished_ = false;
  SparseVertex startingVertex = sparseGraph_->getNumVertices(); // should be number of threads
  boost::thread nnThread(boost::bind(&SparseStorage::populateNNThread, this, startingVertex, numVertices));

  const base::StateSpacePtr &space = si_->getStateSpace();
  std::size_t feedbackFrequency = numVertices / 10;
  BoltVertexData vertexData;

  std::cout << "         Vertices loaded: ";
  for (std::size_t i = 0; i < numVertices; ++i)
  {
    // Copy in data from file
    ia >> vertexData;

    // Allocating a new state and deserializing it from the buffer
    base::State *state = space->allocState();
    space->deserialize(state, &vertexData.stateSerialized_[0]);
    // Add to Sparse graph
    sparseGraph_->addVertexFromFile(state, indent);
    // Feedback
    if ((i + 1) % feedbackFrequency == 0)
      std::cout << static_cast<int>(ceil(i / double(numVertices) * 100.0)) << "% " << std::flush;
  }
  std::cout << std::endl;

  // Join thread
  loadVerticesFinished_ = true;

  time::point startTime;
  if (vThreadTiming_)
    startTime = time::now();  // Benchmark

  nnThread.join();

  if (vThreadTiming_)
    OMPL_INFORM("NN thread took %f seconds to catch up", time::seconds(time::now() - startTime));  // Benchmark
}

void SparseStorage::populateNNThread(std::size_t startingVertex, std::size_t numVertices)
{
  SparseVertex vertexID = startingVertex;  // skip the query vertices
  SparseVertex endID = startingVertex + numVertices - 1;

  while (vertexID < endID)
  {
    // Check if there are any vertices ready to be inserted into the NN
    // We subtract 1 because we do not want to process the newest vertex for threading reasons
    while (vertexID < sparseGraph_->getNumVertices() - 1)
    {
      // There are vertices ready to be inserted
      sparseGraph_->getNN()->add(vertexID);
      vertexID++;
    }

    if (!loadVerticesFinished_)  // only wait if the parent thread isn't already done
    {
      usleep(0.00001 * 1000000);
    }
  }

  // There should be one vertex left
  BOLT_ASSERT(vertexID == sparseGraph_->getNumVertices() - 1,
              "There should only be one vertex left. sparseGraph_->getNumVertices(): " << sparseGraph_->getNumVertices()
              << " vertexID: " << vertexID);

  // Add the last vertex
  sparseGraph_->getNN()->add(vertexID);
}

void SparseStorage::loadEdges(std::size_t numEdges, boost::archive::binary_iarchive &ia, std::size_t indent)
{
  BOLT_FUNC(indent, true, "Loading edges from file: " << numEdges);

  std::size_t feedbackFrequency = std::max(10.0, numEdges / 10.0);
  BoltEdgeData edgeData;

  std::cout << "         Edges loaded:    ";
  for (std::size_t i = 0; i < numEdges; ++i)
  {
    ia >> edgeData;

    // Note: we increment all vertex indexes by the number of query vertices
    const SparseVertex v1 = edgeData.endpoints_.first += numQueryVertices_;
    const SparseVertex v2 = edgeData.endpoints_.second += numQueryVertices_;

    // Add
    sparseGraph_->addEdge(v1, v2, edgeData.weight_, eUNKNOWN, indent);

    // Feedback
    if ((i + 1) % feedbackFrequency == 0)
      std::cout << static_cast<int>(ceil(i / static_cast<double>(numEdges) * 100.0)) << "% " << std::flush;
  }
  std::cout << std::endl;
}

}  // namespace bolt

}  // namespace tools
}  // namespace ompl
