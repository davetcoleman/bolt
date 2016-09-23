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
   Desc:   Smoothing tools for Sparse Graphs
*/

#ifndef OMPL_TOOLS_BOLT_SPARSE_SMOOTHER_
#define OMPL_TOOLS_BOLT_SPARSE_SMOOTHER_

// OMPL
#include <ompl/geometric/PathSimplifier.h>

// Bolt
#include <ompl/tools/debug/Visualizer.h>
#include <bolt_core/Debug.h>

namespace ompl
{
namespace tools
{
namespace bolt
{
/**
   @anchor SparseSmoother
   @par Near-asypmotically optimal roadmap datastructure
*/

/// @cond IGNORE
OMPL_CLASS_FORWARD(SparseSmoother);
/// @endcond

/** \class ompl::tools::bolt::::SparseSmootherPtr
    \brief A boost shared pointer wrapper for ompl::tools::SparseSmoother */

/** \brief Near-asypmotically optimal roadmap datastructure */
class SparseSmoother
{
public:
  /** \brief Constructor needs the state space used for planning.
   */
  SparseSmoother(base::SpaceInformationPtr si, VisualizerPtr visual);

  /** \brief Initialize smoother */
  void setup();

  /* ---------------------------------------------------------------------------------
   * Smoothing
   * --------------------------------------------------------------------------------- */

  /**
   * \brief Path smoothing - improved Dave version
   * \return true on success, false if something failed
   */
  bool smoothQualityPath(geometric::PathGeometric* path, double clearance, bool debug, std::size_t indent);

  /** \brief For finding the optimal path */
  bool smoothMax(geometric::PathGeometric* path, std::size_t indent);

protected:
  /** \brief Short name of this class */
  const std::string name_ = "SparseSmoother";

  /** \brief The created space information */
  base::SpaceInformationPtr si_;

  /** \brief Class for managing various visualization features */
  VisualizerPtr visual_;

  /** \brief A path simplifier used to simplify dense paths added to S */
  geometric::PathSimplifierPtr pathSimplifier_;

public:
  bool visualizeQualityPathSmoothing_ = false;
  bool vSmooth_ = false;

};  // end SparseSmoother

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_BOLT_SPARSE_SMOOTHER__
