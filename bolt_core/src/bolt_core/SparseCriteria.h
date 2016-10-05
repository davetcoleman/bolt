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

#ifndef OMPL_TOOLS_BOLT_SPARSE_CRITERIA_
#define OMPL_TOOLS_BOLT_SPARSE_CRITERIA_

// OMPL
#include <bolt_core/SparseGraph.h>

namespace ompl
{
namespace tools
{
namespace bolt
{
/// @cond IGNORE
OMPL_CLASS_FORWARD(SparseCriteria);
/// @endcond

/** \class ompl::tools::bolt::::SparseCriteriaPtr
    \brief A boost shared pointer wrapper for ompl::tools::SparseCriteria */

class SparseCriteria
{
public:
  /** \brief Constructor needs the state space used for planning.
   */
  SparseCriteria(SparseGraphPtr sg);

  /** \brief Deconstructor */
  virtual ~SparseCriteria();

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
  bool addStateToRoadmap(CandidateData& candidateD, VertexType& addReason, std::size_t threadID, std::size_t indent);

  /* ----------------------------------------------------------------------------------------*/
  /** \brief SPARS-related functions */
  bool checkAddCoverage(CandidateData& candidateD, std::size_t indent);
  bool checkAddConnectivity(CandidateData& candidateD, std::size_t indent);
  bool checkAddInterface(CandidateData& candidateD, std::size_t indent);
#ifdef ENABLE_QUALITY
  bool checkAddQuality(CandidateData& candidateD, std::size_t threadID, std::size_t indent);
  void visualizeCheckAddQuality(base::State* candidateState, SparseVertex candidateRep);

  /* ----------------------------------------------------------------------------------------*/
  // 4th Criteria
  /* ----------------------------------------------------------------------------------------*/

  /** \brief Checks vertex v for short paths through its region and adds when appropriate.
   *         Referred to as 'Test_Add_paths' in paper
   */
  bool checkAddPath(SparseVertex v, std::size_t indent);
  void visualizeCheckAddPath(SparseVertex v, SparseVertex vp, SparseVertex vpp, InterfaceData& iData,
                             std::size_t indent);

  bool addQualityPath(SparseVertex v, SparseVertex vp, SparseVertex vpp, InterfaceData& iData,
                      const double shortestPathVpVpp, std::size_t indent);

  /** \brief As described in paper */
  bool spannerTest(SparseVertex v, SparseVertex vp, SparseVertex vpp, InterfaceData& iData, double& shortestPathVpVpp,
                   std::size_t indent);

  /** \brief Get shortest path between two vertices */
  double qualityEdgeAstarTest(SparseVertex vp, SparseVertex vpp, InterfaceData& iData, std::size_t indent);

  /** \brief Finds the representative of the input state, st  */
  SparseVertex findGraphRepresentative(base::State* st, std::size_t threadID, std::size_t indent);

  /** \brief Finds representatives of samples near candidateState_ which are not his representative
             Referred to as 'Get_Close_Reps' in paper
   */
  void findCloseRepresentatives(const base::State* candidateState, SparseVertex candidateRep,
                                std::map<SparseVertex, base::State*>& closeRepresentatives, std::size_t threadID,
                                std::size_t indent);

  /** \brief Updates pair point information for a representative with neighbor r
             Referred to as 'Update_Points' in paper
      \return true if an update actually happend wihtin the representatives, false if no change
   */
  bool updatePairPoints(SparseVertex candidateRep, const base::State* candidateState, SparseVertex nearSampledRep,
                        const base::State* nearSampledState, std::size_t indent);

  /** \brief Computes all nodes which qualify as a candidate v" for v and vp */
  void getAdjVerticesOfV1UnconnectedToV2(SparseVertex v1, SparseVertex v2,
                                         std::vector<SparseVertex>& adjVerticesUnconnected, std::size_t indent);

  /** \brief Computes all nodes which qualify as a candidate x for v, v', and v"
   *  \return length of maximum midpoint path
   */
  double maxSpannerPath(SparseVertex v, SparseVertex vp, SparseVertex vpp, std::size_t indent);

  /** \brief Performs distance checking for the candidate new state, q against the current information
      \return true if an update actually happend wihtin the representatives, false if no change
  */
  bool distanceCheck(SparseVertex v, const base::State* q, SparseVertex vp, const base::State* qp, SparseVertex vpp,
                     std::size_t indent);

  void visualizeInterfaces(SparseVertex v, std::size_t indent);
  void visualizeAllInterfaces(std::size_t indent);

  /** \brief Count total number of states that are used for defining boundary regions of visibility interfaces
   *  \return first  - total num states
   *          second - num missing interfaces
   */
  std::pair<std::size_t, std::size_t> getInterfaceStateStorageSize();
#endif

  /** \brief Return true if state is far enough away from nearest obstacle */
  bool checkSufficientClearance(base::State* state);

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
  const std::string name_ = "SparseCriteria";

  /** \brief Sparse graph main datastructure that this class operates on */
  SparseGraphPtr sg_;

  /** \brief The created space information */
  base::SpaceInformationPtr si_;

  /** \brief Class for managing various visualization features */
  VisualizerPtr visual_;

  /** \brief Sampler user for generating valid samples in the state space */
  base::ValidStateSamplerPtr sampler_;

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

  /** \brief Multiply this number by the dimension of the state space to choose how much sampling to perform */
  double nearSamplePointsMultiple_ = 2.0;

  /** \brief The stretch factor in terms of graph spanners for SPARS to check against */
  double stretchFactor_;

  /** \brief Percent of sparse fraction that should overlap via the discretization  */
  double penetrationOverlapFraction_ = 0.1;

  /** \brief Percent of SparseDelta to allow for moving nearby vertices */
  double sparseDeltaFractionCheck_ = 0.5;

  bool useL2Norm_ = false;

  /** \brief New Quality criteria rule */
  bool useEdgeImprovementRule_ = true;

  /** \brief Experimental feature that allows very closeby vertices to be merged with newly added ones */
  bool useClearEdgesNearVertex_ = true;
  bool useConnectivityCriteria_ = true;
  bool useQualityCriteria_ = true;
  bool useDirectConnectivyCriteria_ = true;  // Add direct edge instead of also vertex
  bool useSmoothedPathImprovementRule_ = true;

  /** \brief Verbose flags */
  bool vCriteria_ = false;
  bool vQuality_ = false;
  bool vQualityMaxSpanner_ = false;
  bool vRemoveClose_ = false;
  bool vAddedReason_ = false;  // print why each vertex or edge was added

  /** \brief Show the sparse graph being generated */
  bool visualizeAttemptedStates_ = false;
  bool visualizeConnectivity_ = false;
  bool visualizeQualityCriteria_ = false;
  bool visualizeQualityCriteriaCloseReps_ = false;
  bool visualizeQualityCriteriaSampler_ = false;
  bool visualizeQualityCriteriaAstar_ = true;
  bool visualizeRemoveCloseVertices_ = false;

};  // end SparseCriteria

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_BOLT_SPARSE_CRITERIA_
