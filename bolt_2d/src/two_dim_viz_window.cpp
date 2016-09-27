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
   Desc:   Tools for displaying OMPL components in Rviz
*/

#include <bolt_2d/two_dim_viz_window.h>

// ROS
#include <ros/ros.h>
#include <graph_msgs/GeometryGraph.h>
#include <graph_msgs/Edges.h>

// Boost
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

// OMPL
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>

#include <moveit/macros/deprecation.h>

namespace ot = ompl::tools;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace bnu = boost::numeric::ublas;

namespace bolt_2d
{
TwoDimVizWindow::TwoDimVizWindow(rviz_visual_tools::RvizVisualToolsPtr visuals, ompl::base::SpaceInformationPtr si)
  : name_("two_dim_viz_window"), visuals_(visuals), si_(si)
{
  // with this OMPL interface to Rviz all pubs must be manually triggered
  // visuals_->enableBatchPublishing(true);

  ROS_DEBUG_STREAM_NAMED(name_, "Initializing TwoDimVizWindow()");
}

void TwoDimVizWindow::state(const ompl::base::State* state, ot::VizSizes size, ot::VizColors color, double extra_data)
{
  Eigen::Vector3d point = stateToPoint(state);

  // Optional - show state above lines - looks good in 2D but not 3D TODO(davetcoleman): how to auto choose this
  if (si_->getStateDimension() == 2)
    point.z() += 0.5;

  switch (size)
  {
    case ompl::tools::XXSMALL:
      visuals_->publishSphere(std::move(point), visuals_->intToRvizColor(color), rvt::XXXSMALL);
      break;
    case ompl::tools::XSMALL:
      visuals_->publishSphere(std::move(point), visuals_->intToRvizColor(color), rvt::XXSMALL);
      break;
    case ompl::tools::SMALL:
      visuals_->publishSphere(std::move(point), visuals_->intToRvizColor(color), rvt::XSMALL);
      break;
    case ompl::tools::MEDIUM:
      visuals_->publishSphere(std::move(point), visuals_->intToRvizColor(color), rvt::SMALL);
      break;
    case ompl::tools::LARGE:
      visuals_->publishSphere(std::move(point), visuals_->intToRvizColor(color), rvt::MEDIUM);
      break;
    case ompl::tools::XLARGE:
      visuals_->publishSphere(std::move(point), visuals_->intToRvizColor(color), rvt::LARGE);
      break;
    case ompl::tools::XXLARGE:
      visuals_->publishSphere(std::move(point), visuals_->intToRvizColor(color), rvt::XLARGE);
      break;
    case ompl::tools::VARIABLE_SIZE:
      // Visual tools has a scaling feature that will mess up the exact scaling we desire, so we out-smart it
      // extra_data /= visuals_->getGlobalScale();
      // visuals_->publishSphere(point, visuals_->intToRvizColor(color), extra_data * 2);
      {
        extra_data = sqrt(2 * pow(extra_data, 2));
        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        pose.translation() = point;
        pose = pose * Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ());

        visuals_->publishCuboid(pose, extra_data, extra_data, extra_data, visuals_->intToRvizColor(color));
      }
      break;
    case ompl::tools::SCALE:
    {
      const double percent = (extra_data - min_edge_cost_) / (max_edge_cost_ - min_edge_cost_);
      double radius = ((max_state_radius_ - min_state_radius_) * percent + min_state_radius_);

      // Visual tools has a scaling feature that will mess up the exact scaling we desire, so we out-smart it
      radius /= visuals_->getGlobalScale();

      geometry_msgs::Vector3 scale;
      scale.x = radius;
      scale.y = radius;
      scale.z = radius;
      visuals_->publishSphere(visuals_->convertPointToPose(point), visuals_->getColorScale(percent), scale);
    }
    break;
    case ompl::tools::SMALL_TRANSLUCENT:
      OMPL_WARN("SMALL_TRANSLUCENT in two_dim_viz_window... I think this should be removed");
      visuals_->publishSphere(point, rvt::TRANSLUCENT_LIGHT, extra_data);
      break;
    case ompl::tools::ROBOT:
      std::cout << "skipping visualizing robot state " << std::endl;
      break;
    default:
      ROS_ERROR_STREAM_NAMED(name_, "vizState2D: Invalid state size value " << size);
  }
}

void TwoDimVizWindow::states(std::vector<const ompl::base::State*> states, std::vector<ot::VizColors> colors,
                             ot::VizSizes size)
{
  // Cache spheres
  EigenSTL::vector_Vector3d sphere_points;
  std::vector<rvt::colors> sphere_colors;

  for (std::size_t i = 0; i < states.size(); ++i)
  {
    // Convert OMPL state to vector3
    Eigen::Vector3d point = stateToPoint(states[i]);

    // Optional - show state above lines - looks good in 2D but not 3D TODO(davetcoleman): how to auto choose this
    if (si_->getStateDimension() == 2)
      point.z() += 0.5;

    sphere_points.push_back(point);

    // Convert OMPL color to Rviz color
    sphere_colors.push_back(visuals_->intToRvizColor(colors[i]));
  }

  // Publish - TwoDimVizWindow scales are one smaller than MoveItVizWindow
  visuals_->publishSpheres(std::move(sphere_points), std::move(sphere_colors), visuals_->intToRvizScale(size - 1));
}

void TwoDimVizWindow::edge(const ompl::base::State* stateA, const ompl::base::State* stateB, ot::VizSizes size,
                           ot::VizColors color)
{
  Eigen::Vector3d pointA = stateToPoint(stateA);
  Eigen::Vector3d pointB = stateToPoint(stateB);

  double radius;
  switch (size)
  {
    case ompl::tools::SMALL:
      radius = 0.1;
      break;
    case ompl::tools::MEDIUM:
      radius = 0.2;
      break;
    case ompl::tools::LARGE:
      radius = 0.5;
      break;
    default:
      OMPL_ERROR("Unknown size");
      exit(-1);
  }

  visuals_->publishCylinder(pointA, pointB, visuals_->intToRvizColor(color), radius);
}

void TwoDimVizWindow::edge(const ompl::base::State* stateA, const ompl::base::State* stateB, double cost)
{
  // Error check
  if (si_->getStateSpace()->equalStates(stateA, stateB))
  {
    ROS_WARN_STREAM_NAMED(name_, "Unable to visualize edge because states are the same");
    visuals_->publishSphere(stateToPoint(stateA), rvt::RED, rvt::XLARGE);
    visuals_->trigger();
    ros::Duration(0.01).sleep();
    return;
  }

  // Convert input cost
  double percent = (cost - min_edge_cost_) / (max_edge_cost_ - min_edge_cost_);

  // Swap colors
  if (invert_edge_cost_)
  {
    percent = 1 - percent;
  }

  // const double radius = percent / 6.0 + 0.15;
  const double radius = (max_edge_radius_ - min_edge_radius_) * percent + min_edge_radius_;

  if (false)
    std::cout << "cost: " << cost << " min_edge_cost_: " << min_edge_cost_ << " max_edge_cost_: " << max_edge_cost_
              << " percent: " << percent << " radius: " << radius << std::endl;

  publishEdge(stateA, stateB, visuals_->getColorScale(percent), radius);
}

void TwoDimVizWindow::path(ompl::geometric::PathGeometric* path, ompl::tools::VizSizes type, ompl::tools::VizColors vertexColor, ompl::tools::VizColors edgeColor)
{
  // Convert
  const og::PathGeometric& geometric_path = *path;  // static_cast<og::PathGeometric&>(*path);

  switch (type)
  {
    case ompl::tools::SMALL:  // Basic line with vertiices
      publish2DPath(geometric_path, visuals_->intToRvizColor(edgeColor), min_edge_radius_);
      publishSpheres(geometric_path, visuals_->intToRvizColor(vertexColor), rvt::SMALL);
      break;
    case ompl::tools::MEDIUM:  // Basic line with vertiices
      publish2DPath(geometric_path, visuals_->intToRvizColor(edgeColor), (max_edge_radius_ - min_edge_radius_) / 2.0);
      publishSpheres(geometric_path, visuals_->intToRvizColor(vertexColor), rvt::MEDIUM);
      break;
    case ompl::tools::LARGE:  // Basic line with vertiices
      publish2DPath(geometric_path, visuals_->intToRvizColor(edgeColor), max_edge_radius_);
      publishSpheres(geometric_path, visuals_->intToRvizColor(vertexColor), rvt::LARGE);
      break;
    case ompl::tools::ROBOT:  // Playback motion for real robot
      // do nothing in this space
      break;
    default:
      ROS_ERROR_STREAM_NAMED(name_, "Invalid vizPath type value " << type);
  }
}

void TwoDimVizWindow::trigger(std::size_t queueSize)
{
  visuals_->triggerEvery(queueSize);
}

void TwoDimVizWindow::deleteAllMarkers()
{
  visuals_->deleteAllMarkers();
}

void TwoDimVizWindow::spin()
{
  std::cout << "Spinning ROS node... " << std::endl;
  ros::spin();
}

bool TwoDimVizWindow::shutdownRequested()
{
  if (!ros::ok())
  {
    std::cout << "Shutting down process by request of ros::ok()" << std::endl;
    return true;
  }
  return false;
}

// From bolt_2d ------------------------------------------------------

bool TwoDimVizWindow::publishPPMImage(ompl::PPM& ppm, bool static_id)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = visuals_->getBaseFrame();
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  marker.ns = "background";

  // Set the marker type.
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

  // Set the marker action.  Options are ADD and DELETE
  marker.action = visualization_msgs::Marker::ADD;

  static std::size_t marker_id = 0;
  if (static_id)
  {
    marker.id = 0;
  }
  else
  {
    marker_id++;
    marker.id = marker_id;
  }

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = -0.25;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color = visuals_->getColor(rvt::BLACK);

  static const double MAX_COLOR = 255.0 * 3 - 2.0 * std::numeric_limits<double>::epsilon();

  // Visualize Results -------------------------------------------------------------------------------------------------
  for (std::size_t x = 0; x < ppm.getWidth(); ++x)
  {
    for (std::size_t y = 0; y < ppm.getHeight(); ++y)
    {
      ompl::PPM::Color& map_color = ppm.getPixel(x, y);

      std_msgs::ColorRGBA color;
      if (map_color.red + map_color.green + map_color.blue >= MAX_COLOR)
      {
        // continue;  // transparent, do not publish

        // set single color
        color = visuals_->getColor(rvt::WHITE);  // color doesn't matter since alpha is 0
        color.a = 0.0;
      }
      else
      {
        // set single color
        color = visuals_->getColor(rvt::DARK_GREY);
        // color = visuals_->getColor(rvt::BLACK);
        color.a = 1.0;
      }

      // Make right and down triangle
      // Check that we are not on the far right or bottom
      if (!(x + 1 >= ppm.getWidth() || y + 1 >= ppm.getHeight()))
      {
        // Top left triangle
        publishTriangle(x, y, &marker, color);
        publishTriangle(x + 1, y, &marker, color);
        publishTriangle(x + 1, y + 1, &marker, color);

        // Bottom right triangle
        publishTriangle(x, y, &marker, color);
        publishTriangle(x, y + 1, &marker, color);
        publishTriangle(x + 1, y + 1, &marker, color);
      }
    }
  }

  // Send to Rviz
  return visuals_->publishMarker(marker);
}

bool TwoDimVizWindow::publishTriangle(int x, int y, visualization_msgs::Marker* marker, std_msgs::ColorRGBA color)
{
  // Point
  temp_point_.y = x;
  temp_point_.x = y;
  temp_point_.z = 0.1;  // all costs become zero in flat world

  marker->points.push_back(temp_point_);
  marker->colors.push_back(color);

  return true;
}

bool TwoDimVizWindow::publishEdge(const ob::State* stateA, const ob::State* stateB, const std_msgs::ColorRGBA& color,
                                  const double radius)
{
  return visuals_->publishCylinder(stateToPoint(stateA), stateToPoint(stateB), color, radius / 2.0);
}

bool TwoDimVizWindow::publishSpheres(const og::PathGeometric& path, const rvt::colors& color, double scale,
                                     const std::string& ns)
{
  geometry_msgs::Vector3 scale_vector;
  scale_vector.x = scale;
  scale_vector.y = scale;
  scale_vector.z = scale;
  return publishSpheres(path, color, scale_vector, ns);
}

bool TwoDimVizWindow::publishSpheres(const og::PathGeometric& path, const rvt::colors& color, const rvt::scales scale,
                                     const std::string& ns)
{
  return publishSpheres(path, color, visuals_->getScale(scale), ns);
}

bool TwoDimVizWindow::publishSpheres(const og::PathGeometric& path, const rvt::colors& color,
                                     const geometry_msgs::Vector3& scale, const std::string& ns)
{
  std::vector<geometry_msgs::Point> points;
  for (std::size_t i = 0; i < path.getStateCount(); ++i)
    points.push_back(visuals_->convertPoint(stateToPoint(path.getState(i))));

  return visuals_->publishSpheres(points, color, scale, ns);
}

bool TwoDimVizWindow::publish2DPath(const og::PathGeometric& path, const rvt::colors& color, const double thickness,
                                    const std::string& ns)
{
  // Error check
  if (path.getStateCount() <= 0)
  {
    ROS_WARN_STREAM_NAMED(name_, "No states found in path");
    return false;
  }

  // Initialize first vertex
  Eigen::Vector3d prev_vertex = stateToPoint(path.getState(0));
  Eigen::Vector3d this_vertex;

  // Convert path coordinates
  for (std::size_t i = 1; i < path.getStateCount(); ++i)
  {
    // Get current coordinates
    this_vertex = stateToPoint(path.getState(i));

    // Create line
    visuals_->publishCylinder(prev_vertex, this_vertex, color, thickness, ns);

    // Save these coordinates for next line
    prev_vertex = this_vertex;
  }

  return true;
}

Eigen::Vector3d TwoDimVizWindow::stateToPoint(const ob::ScopedState<> state)
{
  return stateToPoint(state.get());
}

Eigen::Vector3d TwoDimVizWindow::stateToPoint(const ob::State* state)
{
  if (!state)
  {
    ROS_FATAL_NAMED(name_, "No state found for vertex");
    exit(1);
  }

  // Convert to RealVectorStateSpace
  const ob::RealVectorStateSpace::StateType* real_state =
      static_cast<const ob::RealVectorStateSpace::StateType*>(state);

  // Create point
  temp_eigen_point_.x() = real_state->values[0];
  temp_eigen_point_.y() = real_state->values[1];

  if (si_->getStateSpace()->getDimension() == 2)
    temp_eigen_point_.z() = level_scale_ * si_->getStateSpace()->getLevel(state);
  else
    temp_eigen_point_.z() = real_state->values[2];

  return temp_eigen_point_;
}

bool TwoDimVizWindow::publishState(const ob::State* state, const rvt::colors& color, const rvt::scales scale,
                                   const std::string& ns)
{
  return visuals_->publishSphere(stateToPoint(state), color, scale, ns);
}

bool TwoDimVizWindow::publishState(const ob::State* state, const rvt::colors& color, const double scale,
                                   const std::string& ns)
{
  return visuals_->publishSphere(stateToPoint(state), color, scale, ns);
}

bool TwoDimVizWindow::publishState(const ob::ScopedState<> state, const rvt::colors& color, const rvt::scales scale,
                                   const std::string& ns)
{
  return visuals_->publishSphere(stateToPoint(state), color, scale, ns);
}

bool TwoDimVizWindow::publishState(const ob::ScopedState<> state, const rvt::colors& color, double scale,
                                   const std::string& ns)
{
  return visuals_->publishSphere(stateToPoint(state), color, scale, ns);
}

bool TwoDimVizWindow::publishState(const ob::ScopedState<> state, const rvt::colors& color,
                                   const geometry_msgs::Vector3& scale, const std::string& ns)
{
  return visuals_->publishSphere(stateToPoint(state), color, scale.x, ns);
}

bool TwoDimVizWindow::publishSampleRegion(const ob::ScopedState<>& state_area, const double& distance)
{
  temp_point_.x = state_area[0];
  temp_point_.y = state_area[1];
  temp_point_.z = state_area[2];

  visuals_->publishSphere(temp_point_, rvt::BLACK, rvt::REGULAR, "sample_region");  // mid point
  // outer sphere (x2 b/c its a radius, x0.1 to make it look nicer)
  return visuals_->publishSphere(temp_point_, rvt::TRANSLUCENT, rvt::REGULAR, "sample_region");
}

}  // namespace bolt_2d
