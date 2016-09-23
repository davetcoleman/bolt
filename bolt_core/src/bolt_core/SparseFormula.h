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
   Desc:   Calculations
*/

#ifndef OMPL_TOOLS_BOLT_SPARSE_FORMULA_
#define OMPL_TOOLS_BOLT_SPARSE_FORMULA_

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <bolt_core/Debug.h>

namespace ompl
{
namespace tools
{
namespace bolt
{

struct SparseFormula
{

  void calc(base::SpaceInformationPtr si, double stretchFactor, double sparseDeltaFraction,
            double penetrationOverlapFraction, double nearSamplePointsMultiple, bool useL2Norm)
  {
    std::size_t indent = 0;

    // Dimensions / joints
    std::size_t dim = si->getStateDimension();

    // Max distance across configuration space
    maxExtent_ = si->getMaximumExtent();

    // Vertex visibility region size
    sparseDelta_ = sparseDeltaFraction * maxExtent_;

    // Sampling for interfaces visibility size
    // denseDelta_ = denseDeltaFraction_;
    denseDelta_ = sparseDeltaFraction * 0.1 * maxExtent_;

    // How much overlap should the discretization factor provide for ensuring edge connection
    // TODO: test simply making this machine epsilon. will that affect creating edges?
    discretizePenetrationDist_ = penetrationOverlapFraction * sparseDelta_;

    // Number of points to test for interfaces around a sample for the quality criterion
    nearSamplePoints_ = nearSamplePointsMultiple * si->getStateDimension();

    // Discretization for initial input into sparse graph
    if (useL2Norm)  // this is for the 2D world
    {
      BOLT_WARN(indent, true, "Using L2 Norm for discretization");

      // const double discFactor = 2 * sparseDelta_;
      // discretization_ = sqrt(std::pow(discFactor, 2) / dim) -
      // discretizePenetrationDist_;
      discretization_ = sparseDelta_ - discretizePenetrationDist_;
    }
    else  // this is for joint space
    {
      BOLT_DEBUG(indent, true, "Using L1 Norm for discretization");
      // L1 Norm
      discretization_ = 2 * sparseDelta_ / dim - discretizePenetrationDist_;
    }

    // Calculate optimum stretch factor
    bool autoStretchFactor = stretchFactor < std::numeric_limits<double>::epsilon();
    if (autoStretchFactor)  // if stretchFactor is zero, auto set it
    {
      BOLT_DEBUG(indent, 1, "Auto setting stretch factor because input value was 0");

      // ND: August 4th
      //stretchFactor_ = dim * discretization_ / (discretization_ - 2 * denseDelta_);

      // Sept 14th
      // This is the paper-corrected version
      stretchFactor_ = dim * discretization_ / (discretization_ - 4 * denseDelta_);
      // 7 DOF:      = 7   * 0.862283        / (0.862283 - 4 * 0.312745)
      //             = 6.035981 / −0.388697
    }
    else
      stretchFactor_ = stretchFactor;

    // Estimate size of graph
    base::RealVectorBounds bounds = si->getStateSpace()->getBounds();
    const std::size_t jointID = 0;
    const double range = bounds.high[jointID] - bounds.low[jointID];
    const std::size_t jointIncrements = floor(range / discretization_);
    const std::size_t maxStatesCount = pow(jointIncrements, dim);
    BOLT_INFO(indent, 1, "--------------------------------------------------");
    BOLT_INFO(indent, 1, "SparseCriteria Setup:");
    BOLT_INFO(indent + 2, 1, "Dimensions              = " << dim);
    BOLT_INFO(indent + 2, 1, "Max Extent              = " << maxExtent_);
    BOLT_INFO(indent + 2, 1, "Sparse Delta            = " << sparseDelta_);
    BOLT_INFO(indent + 2, 1, "Sparse Delta Fraction   = " << sparseDeltaFraction);
    BOLT_INFO(indent + 2, 1, "Dense Delta             = " << denseDelta_);
    BOLT_INFO(indent + 2, 1, "State Dimension         = " << dim);
    BOLT_INFO(indent + 2, 1, "Discretization          = " << discretization_);
    BOLT_INFO(indent + 2, 1, "Joint Increments        = " << jointIncrements);
    BOLT_INFO(indent + 2, 1, "Max States Count        = " << maxStatesCount);
    BOLT_INFO(indent + 2, 1, "Near Sample Points      = " << nearSamplePoints_);
    BOLT_INFO(indent + 2, 1, "Pentrat. Overlap Frac   = " << penetrationOverlapFraction);
    BOLT_INFO(indent + 2, 1, "Discret Penetration     = " << discretizePenetrationDist_);
    BOLT_INFO(indent + 2, 1, "Stretch Factor          = " << stretchFactor_);
    BOLT_INFO(indent, 1, "--------------------------------------------------");
  } // calc

  double maxExtent_;
  double sparseDelta_;
  double denseDelta_;
  double discretizePenetrationDist_;
  double nearSamplePoints_;
  double discretization_;
  double stretchFactor_;
};


}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_BOLT_SPARSE_CRITERIA_
