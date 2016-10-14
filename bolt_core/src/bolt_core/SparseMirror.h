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
   Desc:   Mirror graph for secondary arm
*/

#ifndef OMPL_TOOLS_BOLT_SPARSE_MIRROR_
#define OMPL_TOOLS_BOLT_SPARSE_MIRROR_

// OMPL
#include <bolt_core/SparseGraph.h>
#include <bolt_core/SparseGenerator.h>

namespace ompl
{
namespace tools
{
namespace bolt
{
/// @cond IGNORE
OMPL_CLASS_FORWARD(SparseMirror);
/// @endcond

/** \class ompl::tools::bolt::::SparseMirrorPtr
    \brief A boost shared pointer wrapper for ompl::tools::SparseMirror */

typedef std::function<base::State *(const base::State *state1, const base::State *state2)> CombineStatesCallback;

class SparseMirror
{
public:
  /** \brief Constructor needs the state space used for planning.
   */
  SparseMirror(SparseGraphPtr sg);

  /** \brief Deconstructor */
  virtual ~SparseMirror();

  /** \brief Reset class */
  void clear();

  /** \brief Initialize sparse parameters */
  bool setup(std::size_t indent);

  void mirrorGraphDualArm(base::SpaceInformationPtr dualSpaceInfo, base::SpaceInformationPtr leftArmSpaceInfo,
                          const std::string &outputFile, std::size_t indent);

  void addEdgesForDim(std::vector<SparseVertex> &sparseV2ToDualVertex, SparseGraphPtr &dualSG,
                      base::SpaceInformationPtr dualSpaceInfo, std::size_t indent);

  void addEdgesForAll(std::vector<std::vector<SparseVertex>> &vertexMapMatrix, SparseGraphPtr dualSG,
                      base::SpaceInformationPtr dualSpaceInfo, std::size_t indent);

  void mirrorState(const base::State *source, base::State *dest, std::size_t indent);

  void printJointLimits(double min, double max, double value, const std::string &name);

  /** \brief Used to verify that the two arms are basically the same geometry/collision status. Just for testing */
  void checkValidityOfArmMirror(base::SpaceInformationPtr dualSpaceInfo, base::SpaceInformationPtr leftArmSpaceInfo,
                                std::size_t indent);

  /** \brief Set the callback to combine robot arms into unified state */
  void setCombineStatesCallback(ompl::tools::bolt::CombineStatesCallback callback)
  {
    combineStatesCallback_ = callback;
  }

protected:
  /** \brief Short name of this class */
  const std::string name_ = "SparseMirror";

  /** \brief Sparse graph main datastructure that this class operates on */
  SparseGraphPtr monoSG_;

  /** \brief The created space information */
  base::SpaceInformationPtr monoSI_;

  /** \brief Class for managing various visualization features */
  VisualizerPtr visual_;

  /** \brief Sparse criteria properties */
  double sparseDelta_ = 0;

  /** \brief Callback to combine robot arms into unified state */
  CombineStatesCallback combineStatesCallback_;

public:
  bool verbose_ = false;
  bool vMirror_ = false;
  bool vMirrorStatus_ = true;

  /** \brief Visualization */
  bool visualizeMiroring_ = false;

  bool collisionCheckMirror_ = true;

};  // end SparseMirror

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_BOLT_SPARSE_MIRROR_
