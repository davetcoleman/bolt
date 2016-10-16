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

#ifndef OMPL_TOOLS_BOLT_SPARSE_STORAGE_H_
#define OMPL_TOOLS_BOLT_SPARSE_STORAGE_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/util/ClassForward.h>

// Boost
#include <boost/noncopyable.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/access.hpp>
#include <boost/archive/archive_exception.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

namespace ompl
{
namespace tools
{
namespace bolt
{
/**
   @anchor SparseStorage
   @par Short description
*/

/// @cond IGNORE
OMPL_CLASS_FORWARD(SparseStorage);
OMPL_CLASS_FORWARD(SparseGraph);
/// @endcond

/** \class ompl::tools::bolt::SparseStoragePtr
    \brief A boost shared pointer wrapper for ompl::tools::bolt::SparseStorage */

static const boost::uint32_t OMPL_PLANNER_DATA_ARCHIVE_MARKER = 0x5044414D;  // this spells PDAM

class SparseStorage
{
public:
  /* \brief Information stored at the beginning of the SparseStorage archive */
  struct Header
  {
    /* \brief OMPL specific marker (fixed value) */
    boost::uint32_t marker;

    /* \brief Number of vertices stored in the archive */
    std::size_t vertex_count;

    /* \brief Number of edges stored in the archive */
    std::size_t edge_count;

    /* \brief Signature of state space that allocated the saved states in the vertices (see */
    std::vector<int> signature;

    /* \brief boost::serialization routine */
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int /*version*/)
    {
      ar &marker;
      ar &vertex_count;
      ar &edge_count;
      ar &signature;
    }
  };

  /* \brief The object containing all vertex data that will be stored */
  struct BoltVertexData
  {
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int /*version*/)
    {
      ar &stateSerialized_;
    }

    std::vector<unsigned char> stateSerialized_;
  };

  /* \brief The object containing all edge data that will be stored */
  struct BoltEdgeData
  {
    template <typename Archive>
    void serialize(Archive &ar, const unsigned int /*version*/)
    {
      ar &endpoints_;
      ar &weight_;
    }

    std::pair<unsigned int, unsigned int> endpoints_;
    float weight_;
  };

  /** \brief Constructor */
  SparseStorage(const base::SpaceInformationPtr &si, SparseGraph *sparseGraph);

  void save(const std::string &filePath, std::size_t indent = 0);

  void save(std::ostream &out);

  /* \brief Serialize and save all vertices in \e pd to the binary archive. */
  void saveVertices(boost::archive::binary_oarchive &oa);

  /* \brief Serialize and store all edges in \e pd to the binary archive. */
  void saveEdges(boost::archive::binary_oarchive &oa);

  bool load(const std::string &filePath, std::size_t indent = 0);

  bool load(std::istream &in, std::size_t indent);

  /* \brief Read \e numVertices from the binary input \e ia and store them as SparseStorage */
  void loadVertices(std::size_t numVertices, boost::archive::binary_iarchive &ia, std::size_t indent = 0);

  /** \brief Thread to populate nearest neighbor structure, because that is the slowest component */
  void populateNNThread(std::size_t startingVertex, std::size_t numVertices);

  /* \brief Read \e numEdges from the binary input \e ia and store them as SparseStorage  */
  void loadEdges(std::size_t numEdges, boost::archive::binary_iarchive &ia, std::size_t indent = 0);

  /** \brief Getter for where to save auditing data about size of graph, etc */
  const std::string &getLoggingPath() const
  {
    return loggingPath_;
  }

  /** \brief Setter for where to save auditing data about size of graph, etc */
  void setLoggingPath(const std::string &loggingPath)
  {
    loggingPath_ = loggingPath;
  }

  /** \brief Short name of class */
  const std::string name_ = "SparseStorage";

  /* \brief The space information instance for this data. */
  base::SpaceInformationPtr si_;

  SparseGraph *sparseGraph_;

  /** \brief Based on number of threads system is using */
  std::size_t numQueryVertices_;

  /** \brief Size of graph at last load/save */
  std::size_t prevNumEdges_ = 0;
  std::size_t prevNumVertices_ = 0;

  bool loadVerticesFinished_;

  /** \brief Where to save auditing data about size of graph, etc */
  std::string loggingPath_;

  bool verbose_ = true;
  bool vThreadTiming_ = false;
};  // end of class SparseStorage

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif
