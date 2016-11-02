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
   Desc:   Helper function for loading rosparameter settings into the
           OMPL Bolt algorithm from different applications
*/

#ifndef MOVEIT_OMPL_OMPL_ROSPARAM_H_
#define MOVEIT_OMPL_OMPL_ROSPARAM_H_

// ROS
#include <ros/ros.h>

#include <bolt_core/Bolt.h>


namespace bolt_moveit
{
/** \brief Load settings for Bolt via rosparam server */
void loadOMPLParameters(ros::NodeHandle nh, const std::string &name, ompl::tools::bolt::BoltPtr bolt);

/**
 * \brief Creates a directory names *database_direction* in the user's *home* folder, and inside that creates a file
 *        named *database_name.ompl*
 * \param file_path - result to generate
 * \param file_name - name of file to create
 * \param home_directory - name of folder to save in user directory
 * \return true on success
 */
bool getFilePath(std::string &file_path, const std::string &file_name, const std::string &home_directory);

}  // namespace

#endif  // MOVEIT_OMPL_OMPL_ROSPARAM_H_
