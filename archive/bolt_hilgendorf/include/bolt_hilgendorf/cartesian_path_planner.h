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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Dave Coleman
   Desc:   Creates a cartesian path to be inserted into a planning roadmap
*/

/** TEMPLATE NOTES
    SHORT_NAME - verticle_test
    CLASS_NAME - VerticleApproachTest
    PACKAGE_NAME - moveit_experimental
    THEN make ifndef all caps with Alt-U  (and Alt-F to skip the #define)
 */

#ifndef PACKAGE_NAME_SHORT_NAME_H
#define PACKAGE_NAME_SHORT_NAME_H

// ROS
#include <ros/ros.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace PACKAGE_NAME
{
class CLASS_NAME
{
public:
  /**
   * \brief Constructor
   */
  CLASS_NAME() : name_("SHORT_NAME"), nh_("~")
  {
    // Load rosparams
    // ros::NodeHandle rpnh(nh_, name_);
    // std::size_t error = 0;
    // error += !rosparam_shortcuts::get(name_, rpnh, "control_rate", control_rate_);
    // add more parameters here to load if desired
    // rosparam_shortcuts::shutdownIfError(name_, error);

    ROS_INFO_STREAM_NAMED(name_, "CLASS_NAME Ready.");
  }

private:
  // --------------------------------------------------------

  // The short name of this class
  std::string name_;

  // A shared node handle
  ros::NodeHandle nh_;

};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<CLASS_NAME> CLASS_NAMEPtr;
typedef boost::shared_ptr<const CLASS_NAME> CLASS_NAMEConstPtr;

}  // namespace PACKAGE_NAME
#endif  // PACKAGE_NAME_SHORT_NAME_H
