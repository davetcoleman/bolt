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
   Desc:   Interface information storage class, which does bookkeeping for criterion four.
*/

#ifndef OMPL_TOOLS_BOLT_INTERFACE_DATA_
#define OMPL_TOOLS_BOLT_INTERFACE_DATA_

// OMPL
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>

namespace ompl
{
namespace tools
{
namespace bolt
{
class InterfaceData
{
public:
  /** \brief Constructor */
  InterfaceData()
    : interface1Inside_(nullptr)
    , interface1Outside_(nullptr)
    , interface2Inside_(nullptr)
    , interface2Outside_(nullptr)
    , lastDistance_(std::numeric_limits<double>::infinity())
  {
  }

  /** \brief Clears the given interface data. */
  void clear(const base::SpaceInformationPtr& si)
  {
    if (interface1Inside_)
    {
      si->freeState(interface1Inside_);
      interface1Inside_ = nullptr;
    }
    if (interface2Inside_)
    {
      si->freeState(interface2Inside_);
      interface2Inside_ = nullptr;
    }
    if (interface1Outside_)
    {
      si->freeState(interface1Outside_);
      interface1Outside_ = nullptr;
    }
    if (interface2Outside_)
    {
      si->freeState(interface2Outside_);
      interface2Outside_ = nullptr;
    }
    lastDistance_ = std::numeric_limits<double>::infinity();
  }

  /** \brief Sets information for the first interface (i.e. interface with smaller index vertex). */
  void setInterface1(const base::State* q, const base::State* qp, const base::SpaceInformationPtr& si)
  {
    // Set point A
    if (interface1Inside_)
      si->copyState(interface1Inside_, q);
    else
      interface1Inside_ = si->cloneState(q);

    // Set sigma A
    if (interface1Outside_)
      si->copyState(interface1Outside_, qp);
    else
      interface1Outside_ = si->cloneState(qp);

    assert(interface1Inside_ != nullptr);
    assert(interface1Outside_ != nullptr);

    // Calc distance if we have found both representatives for this vertex pair
    if (hasInterface2())
    {
      lastDistance_ = si->distance(interface1Inside_, interface2Inside_);
    }
  }

  /** \brief Sets information for the second interface (i.e. interface with larger index vertex). */
  void setInterface2(const base::State* q, const base::State* qp, const base::SpaceInformationPtr& si)
  {
    // Set point B
    if (interface2Inside_)
      si->copyState(interface2Inside_, q);
    else
      interface2Inside_ = si->cloneState(q);

    // Set sigma B
    if (interface2Outside_)
      si->copyState(interface2Outside_, qp);
    else
      interface2Outside_ = si->cloneState(qp);

    assert(interface2Inside_ != nullptr);
    assert(interface2Outside_ != nullptr);

    // Calc distance
    if (hasInterface1())
    {
      lastDistance_ = si->distance(interface1Inside_, interface2Inside_);
    }
  }

  /** \brief Helper to determine wether interface 1 has been found */
  bool hasInterface1() const
  {
    return interface1Inside_ != nullptr;
  }

  /** \brief Helper to determine wether interface 2 has been found */
  bool hasInterface2() const
  {
    return interface2Inside_ != nullptr;
  }

  base::State* getInsideInterfaceOfV1(std::size_t v1, std::size_t v2) const
  {
    if (v1 < v2)
      return interface1Inside_;
    else if (v1 > v2)
      return interface2Inside_;

    throw Exception("InterfaceHash", "Vertices are the same index");
    return NULL;
  }

  base::State* getInsideInterfaceOfV2(std::size_t v1, std::size_t v2) const
  {
    if (v1 < v2)
      return interface2Inside_;
    else if (v1 > v2)
      return interface1Inside_;

    throw Exception("InterfaceHash", "Vertices are the same index");
    return NULL;
  }

  base::State* getOutsideInterfaceOfV1(std::size_t v1, std::size_t v2) const
  {
    if (v1 < v2)
      return interface1Outside_;
    else if (v1 > v2)
      return interface2Outside_;

    throw Exception("InterfaceHash", "Vertices are the same index");
    return NULL;
  }

  base::State* getOutsideInterfaceOfV2(std::size_t v1, std::size_t v2) const
  {
    if (v1 < v2)
      return interface2Outside_;
    else if (v1 > v2)
      return interface1Outside_;

    throw Exception("InterfaceHash", "Vertices are the same index");
    return NULL;
  }

  double getLastDistance() const
  {
    return lastDistance_;
  }

  base::State* getInterface1Inside()
  {
    return interface1Inside_;
  }

  base::State* getInterface1Outside()
  {
    return interface1Outside_;
  }

  base::State* getInterface2Inside()
  {
    return interface2Inside_;
  }

  base::State* getInterface2Outside()
  {
    return interface2Outside_;
  }

private:
  // Note: interface1 is between this vertex v and the vertex with the lower index v'
  // Note: interface2 is between this vertex v and the vertex with the higher index v''

  base::State* interface1Inside_;   // Lies inside the visibility region of the vertex and supports its interface
  base::State* interface1Outside_;  // Lies outside the visibility region of the vertex and supports its interface
                                    // (sigma)

  base::State* interface2Inside_;   // Lies inside the visibility region of the vertex and supports its interface.
  base::State* interface2Outside_;  // Lies outside the visibility region of the vertex and supports its interface
                                    // (sigma)

  /** \brief Last known distance between the two interfaces supported by points_ and sigmas. */
  double lastDistance_;

  // TODO: Remember when iData has failed to shortcut prexisting shorcut because came up with same lastDistance

};  // end class InterfaceData

}  // namespace bolt
}  // namespace tools
}  // namespace ompl

#endif  // OMPL_TOOLS_BOLT_INTERFACE_DATA_
