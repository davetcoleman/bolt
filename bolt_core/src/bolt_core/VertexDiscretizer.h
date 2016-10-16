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
   Desc:   Utility to discretize a space into a uniform grid
*/

#ifndef OMPL_TOOLS_BOLT_VERTEX_DISCRETIZER_
#define OMPL_TOOLS_BOLT_VERTEX_DISCRETIZER_

// OMPL
#include <ompl/base/StateSpace.h>
#include <ompl/tools/debug/Visualizer.h>

// Boost
#include <boost/thread/mutex.hpp>

namespace ompl
{
namespace tools
{
namespace bolt
{
/// @cond IGNORE
OMPL_CLASS_FORWARD(VertexDiscretizer);
OMPL_CLASS_FORWARD(SparseGraph);
/// @endcond

/** \class ompl::tools::bolt::VertexDiscretizerPtr
    \brief A boost shared pointer wrapper for ompl::tools::bolt::VertexDiscretizer */

class VertexDiscretizer
{
public:
  /** \brief Constructor needs the state space used for planning.
   *  \param space - state space
   */
  VertexDiscretizer(SparseGraphPtr sg);

  ~VertexDiscretizer();

  /**
   * \brief Discretize the space into a simple grid
   */
  bool generateGrid(std::size_t indent);

  std::size_t getVerticesAddedCount()
  {
    return verticesAdded_;
  }

  /** \brief Getter for Discretization */
  const double& getDiscretization() const
  {
    return discretization_;
  }

  /** \brief Setter for Discretization */
  void setDiscretization(const double& discretization)
  {
    discretization_ = discretization;
  }

  /** \brief Getter for StartingValueOffset */
  const double& getStartingValueOffset() const
  {
    return startingValueOffset_;
  }

  /** \brief Setter for StartingValueOffset */
  void setStartingValueOffset(const double& startingValueOffset)
  {
    startingValueOffset_ = startingValueOffset;
  }

  /** \brief Set the minimum required distance of sample from nearest obstacle to be considered valid */
  void setMinimumObstacleClearance(double clearance)
  {
    clearance_ = clearance;

    const double& clearanceSearchDistance = si_->getStateValidityChecker()->getClearanceSearchDistance();
    if (clearanceSearchDistance < clearance_)
    {
      OMPL_WARN("Desired clearance between vertices and obstacles (%f) is greater than the search distance (%f), will "
                "not work properly",
                clearance_, clearanceSearchDistance);
    }
  }

private:
  /**
   * \brief Generate a grid of vertices across the configuration space
   */
  void generateVertices(std::size_t indent);

  void generateVerticesThread(std::size_t threadID, double startJointValue, double endJointValue,
                              base::SpaceInformationPtr si, std::size_t indent);

  void recursiveDiscretization(std::size_t threadID, std::vector<double>& values, std::size_t jointID,
                               base::SpaceInformationPtr si, base::State* candidateState,
                               std::size_t maxDiscretizationLevel, std::size_t indent);

  void createState(std::size_t threadID, std::vector<double>& values, base::SpaceInformationPtr si,
                   base::State* candidateState, std::size_t indent);

  /** \brief Short name of class */
  const std::string name_ = "VertexDiscretizer";

  /** \brief Sparse graph main datastructure that this class operates on */
  SparseGraphPtr sg_;

  /** \brief The created space information */
  base::SpaceInformationPtr si_;

  /** \brief Class for managing various visualization features */
  VisualizerPtr visual_;

  /** \brief Prevent two invalid vertices from being added to the invalid list at the same time */
  boost::mutex vertexMutex_;

  /** \brief Prevent two vertices from being added to graph at same time */
  boost::mutex sparseGraphMutex_;

  /** \brief How many threads to use while generating vertices */
  std::size_t numThreads_;

  /** \brief Track total vertices added to graph */
  std::size_t verticesAdded_;

  /** \brief Distance between grid points (discretization level) */
  double discretization_ = 2.0;

  /** \brief Where to begin each dimension */
  double startingValueOffset_ = 0;

  /** \brief Minimum required distance of state from nearest obstacle to be considered valid */
  double clearance_;

public:
  /** \brief Show more debug info */
  bool verbose_ = false;
  bool vThread_ = false;

  /** \brief Various options for visualizing the algorithmns performance */
  bool visualizeGridGeneration_ = false;

  /** \brief Request user to press next for each generated state */
  bool visualizeGridGenerationWait_ = false;

  bool visualizeDistanceToCollision_ = false;

};  // end of class VertexDiscretizer

}  // namespace bolt
}  // namespace tools
}  // namespace ompl
#endif
