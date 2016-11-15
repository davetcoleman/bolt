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
   Desc:   Various tests to determine if a vertex/edge should be added to the graph, based on SPARS
*/

#ifndef OMPL_TOOLS_BOLT_TASK_CRITERIA_
#define OMPL_TOOLS_BOLT_TASK_CRITERIA_

// OMPL
#include <bolt_core/TaskGraph.h>

namespace ompl
{
namespace tools
{
namespace bolt
{
/// @cond IGNORE
OMPL_CLASS_FORWARD(TaskCriteria);
/// @endcond

/** \class ompl::tools::bolt::::TaskCriteriaPtr
    \brief A boost shared pointer wrapper for ompl::tools::TaskCriteria */

class TaskCriteria
{
public:
  /** \brief Constructor needs the state space used for planning.
   */
  TaskCriteria(TaskGraphPtr sg);

  /** \brief Deconstructor */
  virtual ~TaskCriteria();

  /** \brief Initialize sparse parameters */
  bool setup(std::size_t indent);

  void clear();

  void resetStats();

  /**
   * \brief Run various checks/criteria to determine if to keep TaskVertex in sparse graph
   * \param denseVertex - the original vertex to consider
   * \param addReason - if function returns true, the reson the denseVertex was added to the sparse graph
   * \return true on success
   */
  bool addStateToRoadmap(TaskCandidateData& candidateD, VertexType& addReason, std::size_t threadID,
                         std::size_t indent);

  /* ----------------------------------------------------------------------------------------*/
  /** \brief SPARS-related functions */
  bool checkAddCoverage(TaskCandidateData& candidateD, std::size_t indent);
  bool checkAddConnectivity(TaskCandidateData& candidateD, std::size_t indent);
  bool checkAddInterface(TaskCandidateData& candidateD, std::size_t indent);

  double getSparseDelta()
  {
    return sparseDelta_;
  }
  double getDenseDelta()
  {
    return denseDelta_;
  }
  double getStretchFactor()
  {
    return stretchFactor_;
  }

  double getDiscretization()
  {
    return discretization_;
  }

  bool getUseFourthCriteria()
  {
    return useFourthCriteria_;
  }

  void setUseFourthCriteria(bool useFourthCriteria)
  {
    useFourthCriteria_ = useFourthCriteria;
  }

  double getDiscretizePenetrationDist()
  {
    return discretizePenetrationDist_;
  }

  std::size_t getNearSamplePoints()
  {
    return nearSamplePoints_;
  }

  std::size_t getNearSamplePointsMultiple()
  {
    return nearSamplePointsMultiple_;
  }

  std::size_t getNumVerticesMoved()
  {
    return numVerticesMoved_;
  }

protected:
  /** \brief Short name of this class */
  const std::string name_ = "TaskCriteria";

  /** \brief Task graph main datastructure that this class operates on */
  TaskGraphPtr taskGraph_;

  /** \brief The created space information */
  base::SpaceInformationPtr compoundSI_;

  /** \brief Class for managing various visualization features */
  VisualizerPtr visual_;

  /** \brief Amount of sub-optimality allowed */
  double sparseDelta_;

  /** \brief SPARS parameter for dense graph connection distance */
  double denseDelta_;

  /** \brief How overlapping two visibility regions should be to each other, where 0 is just barely touching */
  double discretizePenetrationDist_;

  /** \brief Number of sample points to use when trying to detect interfaces. */
  std::size_t nearSamplePoints_;

  /** \brief Cache the maximum extent for later re-use */
  double maxExtent_;

  /** \brief Granuality of the discretized graph */
  double discretization_;

  bool useFourthCriteria_;

  /** \brief Temporary state for doing sparse criteria sampling */
  std::vector<base::State*> closeRepSampledState_;

  /** \brief For statistics */
  std::size_t numVerticesMoved_ = 0;

public:
  /** \brief SPARS parameter for dense graph connection distance as a fraction of max. extent */
  double denseDeltaFraction_ = 0.05;

  /** \brief Maximum visibility range for nodes in the graph as a fraction of maximum extent. */
  double sparseDeltaFraction_ = 0.25;

  /** \brief When saving experiences, save at smaller discretization */
  double sparseDeltaFractionSecondary_ = 0.25;

  /** \brief Multiply this number by the dimension of the state space to choose how much sampling to perform */
  double nearSamplePointsMultiple_ = 2.0;

  /** \brief The stretch factor in terms of graph spanners for SPARS to check against */
  double stretchFactor_;

  /** \brief Percent of sparse fraction that should overlap via the discretization  */
  double penetrationOverlapFraction_ = 0.1;

  /** \brief Percent of TaskDelta to allow for moving nearby vertices */
  double sparseDeltaFractionCheck_ = 0.5;

  bool useL2Norm_ = false;

  /** \brief Experimental feature that allows very closeby vertices to be merged with newly added ones */
  bool useClearEdgesNearVertex_ = true;
  bool useConnectivityCriteria_ = true;
  bool useDirectConnectivyCriteria_ = true;  // Add direct edge instead of also vertex
  bool useSmoothedPathImprovementRule_ = true;

  /** \brief Verbose flags */
  bool vCriteria_ = false;
  bool vRemoveClose_ = false;
  bool vAddedReason_ = false;  // print why each vertex or edge was added

  /** \brief Show the sparse graph being generated */
  bool visualizeAttemptedStates_ = false;
  bool visualizeConnectivity_ = false;
  bool visualizeRemoveCloseVertices_ = false;

};  // end TaskCriteria

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_BOLT_TASK_CRITERIA_
