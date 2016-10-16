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
   Desc:   Sparse experience database for storing and reusing past path plans
           Assumes "name_" exists in every class for the namespace
*/

#ifndef OMPL_TOOLS_BOLT_DEBUG_H
#define OMPL_TOOLS_BOLT_DEBUG_H

#include <ostream>
#include <bolt_core/ThrowAssert.h>

// clang-format off

// BOLT_DEBUG()
#ifdef ENABLE_DEBUG_MACRO
#define BOLT_DEBUG(indent, flag, stream)                                \
  do                                                                    \
  {                                                                     \
    std::stringstream o; o << stream;                                   \
    if (flag)                                                           \
      std::cout << ANSI_COLOR_GREEN << std::string(indent, ' ') << o.str() << ANSI_COLOR_RESET << std::endl; \
  } while (0)
#else
#define BOLT_DEBUG(indent, flag, stream)        \
  do {} while (0)
#endif

// BOLT_COLOR_DEBUG()
#ifdef ENABLE_DEBUG_MACRO
#define BOLT_COLOR_DEBUG(indent, flag, stream, color)                   \
      do                                                                \
      {                                                                 \
        std::stringstream o; o << stream;                               \
        if (flag)                                                       \
          std::cout << color << std::string(indent, ' ') << o.str() << ANSI_COLOR_RESET << std::endl; \
      } while (0)
#else
#define BOLT_COLOR_DEBUG(indent, flag, stream, color)   \
  do {} while (0)
#endif

// BOLT_FUNC() - use only for function names, auto adds 2 to the indent value
#ifdef ENABLE_DEBUG_MACRO
#define BOLT_FUNC(indent, flag, stream)                   \
      do                                                                \
      {                                                                 \
        std::stringstream o; o << stream;                               \
        indent = indent + 2;                                            \
        if (flag)                                                       \
          std::cout << ANSI_COLOR_CYAN << std::string(indent, ' ') << name_ << ":" << o.str() << ANSI_COLOR_RESET << std::endl; \
        indent = indent + 2;                                            \
      } while (0)
#else
#define BOLT_FUNC(indent, flag, stream)   \
  do {} while (0)
#endif

// BOLT_INFO()
#define BOLT_INFO(indent, flag, stream)                                \
  do                                                                    \
  {                                                                     \
    std::stringstream o; o << stream;                                   \
    if (flag)                                                           \
      std::cout << std::string(indent, ' ') << o.str() << std::endl;    \
  } while (0)

// BOLT_COLOR_INFO()
#define BOLT_COLOR_INFO(indent, flag, stream, color)                   \
    do                                                                  \
    {                                                                   \
      std::stringstream o; o << stream;                                 \
      if (flag)                                                         \
        std::cout << color << std::string(indent, ' ') << o.str() << ANSI_COLOR_RESET << std::endl; \
    } while (0)

// BOLT_ERROR()
#define BOLT_ERROR(indent, stream) BOLT_COLOR_INFO(indent, true, stream, ANSI_COLOR_RED);

// BOLT_WARN()
#define BOLT_WARN(indent, flag, stream) BOLT_COLOR_INFO(indent, flag, stream, ANSI_COLOR_YELLOW);

// OTHER COLORS
#define BOLT_GREEN(indent, flag, stream) BOLT_COLOR_DEBUG(indent, flag, stream, ANSI_COLOR_GREEN);
#define BOLT_BLUE(indent, flag, stream) BOLT_COLOR_DEBUG(indent, flag, stream, ANSI_COLOR_BLUE);
#define BOLT_MAGENTA(indent, flag, stream) BOLT_COLOR_DEBUG(indent, flag, stream, ANSI_COLOR_MAGENTA);
// Note: Cyan is also used for function headers
#define BOLT_CYAN(indent, flag, stream) BOLT_COLOR_DEBUG(indent, flag, stream, ANSI_COLOR_CYAN);
// clang-format on

#endif  // OMPL_TOOLS_BOLT_DEBUG_H
