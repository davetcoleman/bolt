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
 *   * Neither the name of University of Colorado nor the names of its
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
   Desc:   Load series of lines from file
           Future possibility: https://github.com/memononen/nanosvg/blob/master/src/nanosvg.h
*/

#ifndef BOLT_MOVEIT_PATH_LOADER_H
#define BOLT_MOVEIT_PATH_LOADER_H

// ROS
#include <ros/ros.h>

// Boost
#include <boost/filesystem.hpp>

// C++
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>

namespace bolt_moveit
{
class PathLoader
{
public:
  /** \brief Constructor */
  PathLoader(const std::string &package_path)
  {
    ROS_INFO_STREAM_NAMED(name_, "PathLoader Ready.");

    // Get file name
    if (!getFilePath(package_path, file_path_, "2d_path.csv", "config"))
      exit(-1);
  }

  /** \brief Get a set of paths, one for each end effector */
  bool get2DPath(std::vector<EigenSTL::vector_Affine3d> &path, bool debug = false)
  {
    if (!boost::filesystem::exists(file_path_))
    {
      ROS_WARN_STREAM_NAMED(name_, "File not found: " << file_path_);
      return false;
    }
    std::ifstream input_file(file_path_);

    std::string line;

    std::size_t path_id = 0;
    path.resize(path_id + 1);
    while (std::getline(input_file, line))
    {
      // Check if new path has been started
      if (line == "---")
      {
        ROS_DEBUG_STREAM_NAMED(name_, "New path found");
        path_id++;
        path.resize(path_id + 1);
        continue;
      }

      std::stringstream lineStream(line);
      std::string cell;

      Eigen::Affine3d point = Eigen::Affine3d::Identity();

      // For each item/column
      for (std::size_t i = 0; i < 3; ++i)
      {
        // Get a variable
        if (!std::getline(lineStream, cell, ','))
        {
          ROS_ERROR_STREAM_NAMED(name_, "Missing variable " << i);
          return false;
        }

        try
        {
          trim(cell);
          point.translation()[i] = boost::lexical_cast<double>(cell.c_str());
        }
        catch (...)
        {
          ROS_ERROR_STREAM_NAMED(name_, "Failed to cast value '" << cell << "' to double");
          return false;
        }
      }  // for

      path[path_id].push_back(point);
    }  // while

    if (debug)
      printPath(path);

    return true;
  }

  void printPath(std::vector<EigenSTL::vector_Affine3d> &path)
  {
    std::cout << "Printing path: " << std::endl;
    for (std::size_t j = 0; j < path.size(); ++j)
    {
      for (std::size_t i = 0; i < path[j].size(); ++i)
      {
        const Eigen::Affine3d &point = path[j][i];
        std::cout << "  Point: " << i << " x: " << point.translation().x() << " y: " << point.translation().y()
                  << std::endl;
      }
      std::cout << "--- " << std::endl;
    }
  }

  bool getFilePath(const std::string &package_path, std::string &file_path, const std::string &file_name,
                   const std::string &subdirectory) const

  {
    namespace fs = boost::filesystem;

    // Check that the directory exists, if not, create it
    fs::path rootPath = fs::path(package_path);
    rootPath = rootPath / fs::path(subdirectory);

    boost::system::error_code returnedError;
    fs::create_directories(rootPath, returnedError);

    if (returnedError)
    {
      // did not successfully create directories
      ROS_ERROR("Unable to create directory %s", subdirectory.c_str());
      return false;
    }

    // directories successfully created, append the group name as the file name
    rootPath = rootPath / fs::path(file_name);
    file_path = rootPath.string();
    // ROS_DEBUG_STREAM_NAMED(name_, "Config file: " << file_path);

    return true;
  }

private:
  // trim from start (in place)
  static inline void ltrim(std::string &s)
  {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
  }

  // trim from end (in place)
  static inline void rtrim(std::string &s)
  {
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
  }

  // trim from both ends (in place)
  static inline void trim(std::string &s)
  {
    ltrim(s);
    rtrim(s);
  }

  // --------------------------------------------------------

  // The short name of this class
  std::string name_ = "path_loader";

  std::string file_path_;

};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<PathLoader> PathLoaderPtr;
typedef boost::shared_ptr<const PathLoader> PathLoaderConstPtr;

}  // namespace bolt_moveit
#endif  // BOLT_MOVEIT_PATH_LOADER_H
