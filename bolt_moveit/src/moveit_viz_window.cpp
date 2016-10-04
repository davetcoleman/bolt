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

#include <bolt_moveit/moveit_viz_window.h>

// ROS
#include <ros/ros.h>
#include <graph_msgs/GeometryGraph.h>
#include <graph_msgs/Edges.h>

// OMPL
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>

// For converting OMPL state to a MoveIt robot state
#include <moveit_ompl/model_based_state_space.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/macros/deprecation.h>

namespace ot = ompl::tools;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace bolt_moveit
{
MoveItVizWindow::MoveItVizWindow(moveit_visual_tools::MoveItVisualToolsPtr visuals, ob::SpaceInformationPtr si)
  : ot::VizWindow(si), name_("moveit_viz_window"), visuals_(visuals)
{
  // with this OMPL interface to Rviz all pubs must be manually triggered
  // visuals_->enableBatchPublishing(false);

  visuals_->loadSharedRobotState();

  ROS_DEBUG_STREAM_NAMED(name_, "Initializing MoveItVizWindow()");
}

void MoveItVizWindow::state(const ob::State* state, ot::VizSizes size, ot::VizColors color, double extra_data,
                            ob::SpaceInformationPtr si)
{
  // We do not use stateToPoint() because the publishRobotState() function might need the robot state in this function
  moveit_ompl::ModelBasedStateSpacePtr mb_state_space =
      std::static_pointer_cast<moveit_ompl::ModelBasedStateSpace>(si->getStateSpace());

  // We must use the root_robot_state here so that the virtual_joint isn't affected
  mb_state_space->copyToRobotState(*visuals_->getRootRobotState(), state);

  // Check for this size first, because we can skip forward kinematics
  if (size == ompl::tools::ROBOT)  // Show actual robot in custom color
  {
    visuals_->publishRobotState(visuals_->getRootRobotState(), visuals_->intToRvizColor(color));
    return;
  }

  for (const moveit::core::LinkModel* link : eef_link_models_)
  {
    Eigen::Affine3d pose = visuals_->getRootRobotState()->getGlobalLinkTransform(link);
    switch (size)
    {
      case ompl::tools::SMALL:
        visuals_->publishSphere(pose, visuals_->intToRvizColor(color), rvt::SMALL);
        break;
      case ompl::tools::MEDIUM:
        visuals_->publishSphere(pose, visuals_->intToRvizColor(color), rvt::REGULAR);
        break;
      case ompl::tools::LARGE:
        visuals_->publishSphere(pose, visuals_->intToRvizColor(color), rvt::LARGE);
        break;
      case ompl::tools::VARIABLE_SIZE:  // Medium purple, translucent outline
        // Visual tools has a scaling feature that will mess up the exact scaling we desire, so we out-smart it
        extra_data /= visuals_->getGlobalScale();
        visuals_->publishSphere(visuals_->convertPose(pose), visuals_->intToRvizColor(color), extra_data * 2);
        break;
      case ompl::tools::SCALE:  // Display sphere based on value between 0-100
      {
        const double percent = (extra_data - min_edge_cost_) / (max_edge_cost_ - min_edge_cost_);
        double radius = ((max_state_radius_ - min_state_radius_) * percent + min_state_radius_);

        // Visual tools has a scaling feature that will mess up the exact scaling we desire, so we out-smart it
        radius /= visuals_->getGlobalScale();

        geometry_msgs::Vector3 scale;
        scale.x = radius;
        scale.y = radius;
        scale.z = radius;
        visuals_->publishSphere(pose, visuals_->getColorScale(percent), scale);
      }
      break;
      default:
        ROS_ERROR_STREAM_NAMED(name_, "vizStateRobot: Invalid state type value");
    }  // end switch
  }    // end for
}

void MoveItVizWindow::states(std::vector<const ob::State*> states, std::vector<ot::VizColors> colors, ot::VizSizes size)
{
  if (size == ompl::tools::ROBOT)
  {
    ROS_WARN_STREAM_NAMED(name_, "Not implemented for moveit_viz_window");
    return;
  }

  // Cache spheres
  EigenSTL::vector_Vector3d sphere_points;
  std::vector<rvt::colors> sphere_colors;

  for (std::size_t i = 0; i < states.size(); ++i)
  {
    for (const moveit::core::LinkModel* link : eef_link_models_)
    {
      // Convert OMPL state to vector3
      sphere_points.push_back(stateToPoint(states[i], link));

      // Convert OMPL color to Rviz color
      sphere_colors.push_back(visuals_->intToRvizColor(colors[i]));
    }
  }

  // Publish
  visuals_->publishSpheres(std::move(sphere_points), std::move(sphere_colors), visuals_->intToRvizScale(size));
}

void MoveItVizWindow::edge(const ob::State* stateA, const ob::State* stateB, double cost)
{
  // Error check
  if (si_->getStateSpace()->equalStates(stateA, stateB))
  {
    ROS_WARN_STREAM_NAMED(name_, "Unable to visualize edge because states are the same");
    for (const moveit::core::LinkModel* link : eef_link_models_)
    {
      visuals_->publishSphere(stateToPoint(stateA, link), rvt::RED, rvt::XLARGE);
    }
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

  const double radius = (max_edge_radius_ - min_edge_radius_) * percent + min_edge_radius_;

  if (false)
  {
    std::cout << "cost: " << cost << " min_edge_cost_: " << min_edge_cost_ << " max_edge_cost_: " << max_edge_cost_
              << " percent: " << percent << " radius: " << radius << std::endl;
    std::cout << "max edge r: " << max_edge_radius_ << " min edge r: " << min_edge_radius_ << std::endl;
  }

  for (const moveit::core::LinkModel* link : eef_link_models_)
  {
    visuals_->publishLine(stateToPoint(stateA, link), stateToPoint(stateB, link), visuals_->getColorScale(percent),
                          radius / 2.0);
  }
}

void MoveItVizWindow::edge(const ob::State* stateA, const ob::State* stateB, ot::VizSizes size, ot::VizColors color)
{
  for (const moveit::core::LinkModel* link : eef_link_models_)
  {
    visuals_->publishCylinder(stateToPoint(stateA, link), stateToPoint(stateB, link), visuals_->intToRvizColor(color),
                              visuals_->intToRvizScale(size));
  }
}

void MoveItVizWindow::path(ompl::geometric::PathGeometric* path, ompl::tools::VizSizes type,
                           ompl::tools::VizColors vertexColor, ompl::tools::VizColors edgeColor)
{
  // Convert
  const og::PathGeometric& geometric_path = *path;  // static_cast<og::PathGeometric&>(*path);

  switch (type)
  {
    case ompl::tools::SMALL:  // Basic line with vertiices
      publish3DPath(geometric_path, visuals_->intToRvizColor(edgeColor), min_edge_radius_);
      publishSpheres(geometric_path, visuals_->intToRvizColor(vertexColor), rvt::SMALL);
      break;
    case ompl::tools::MEDIUM:  // Basic line with vertiices
      publish3DPath(geometric_path, visuals_->intToRvizColor(edgeColor), (max_edge_radius_ - min_edge_radius_) / 2.0);
      publishSpheres(geometric_path, visuals_->intToRvizColor(vertexColor), rvt::SMALL);
      break;
    case ompl::tools::LARGE:  // Basic line with vertiices
      publish3DPath(geometric_path, visuals_->intToRvizColor(edgeColor), max_edge_radius_);
      publishSpheres(geometric_path, visuals_->intToRvizColor(vertexColor), rvt::SMALL);
      break;
    case ompl::tools::ROBOT:  // Playback motion for real robot
      // Check that jmg_ was set
      if (!jmg_)
        ROS_ERROR_STREAM_NAMED(name_, "Joint model group has not been set");

      publishTrajectoryPath(geometric_path, jmg_, true /*wait_for_trajectory*/);
      break;
    default:
      ROS_ERROR_STREAM_NAMED(name_, "Invalid vizPath type value " << type);
  }
}

void MoveItVizWindow::trigger(std::size_t queueSize)
{
  visuals_->triggerEvery(queueSize);
}

void MoveItVizWindow::deleteAllMarkers()
{
  visuals_->deleteAllMarkers();
}

void MoveItVizWindow::spin()
{
  std::cout << "Spinning ROS node... " << std::endl;
  ros::spin();
}

bool MoveItVizWindow::shutdownRequested()
{
  if (!ros::ok())
  {
    std::cout << "Shutting down process by request of ros::ok()" << std::endl;
    return true;
  }
  return false;
}

// From bolt_moveit ------------------------------------------------------

void MoveItVizWindow::publishSpheres(const og::PathGeometric& path, const rvt::colors& color, double scale,
                                     const std::string& ns)
{
  geometry_msgs::Vector3 scale_vector;
  scale_vector.x = scale;
  scale_vector.y = scale;
  scale_vector.z = scale;
  publishSpheres(path, color, scale_vector, ns);
}

void MoveItVizWindow::publishSpheres(const og::PathGeometric& path, const rvt::colors& color, const rvt::scales scale,
                                     const std::string& ns)
{
  publishSpheres(path, color, visuals_->getScale(scale), ns);
}

void MoveItVizWindow::publishSpheres(const og::PathGeometric& path, const rvt::colors& color,
                                     const geometry_msgs::Vector3& scale, const std::string& ns)
{
  std::vector<geometry_msgs::Point> points;
  for (std::size_t i = 0; i < path.getStateCount(); ++i)
    for (const moveit::core::LinkModel* link : eef_link_models_)
      points.push_back(visuals_->convertPoint(stateToPoint(path.getState(i), link)));

  visuals_->publishSpheres(points, color, scale, ns);
}

void MoveItVizWindow::publishRobotState(const ob::State* state)
{
  moveit_ompl::ModelBasedStateSpacePtr mb_state_space =
      std::static_pointer_cast<moveit_ompl::ModelBasedStateSpace>(si_->getStateSpace());

  // Convert to robot state
  mb_state_space->copyToRobotState(*visuals_->getSharedRobotState(), state);

  // Show the robot visualized in Rviz
  visuals_->publishRobotState(visuals_->getSharedRobotState());
}

void MoveItVizWindow::publishTrajectoryPath(const og::PathGeometric& path, const robot_model::JointModelGroup* jmg,
                                            const bool blocking)
{
  // Convert to MoveIt! format
  robot_trajectory::RobotTrajectoryPtr traj;
  double speed = 0.01;
  if (!convertPath(path, jmg, traj, speed))
  {
    return;
  }

  visuals_->publishTrajectoryPath(*traj, blocking);
}

void MoveItVizWindow::publish3DPath(const og::PathGeometric& path, const rvt::colors& color, const double thickness,
                                    const std::string& ns)
{
  std::cout << "publish3DPath() " << std::endl;

  // Error check
  if (path.getStateCount() <= 0)
  {
    ROS_WARN_STREAM_NAMED(name_, "No states found in path");
    return;
  }

  for (const moveit::core::LinkModel* link : eef_link_models_)
  {
    // Initialize first vertex
    Eigen::Vector3d prev_vertex = stateToPoint(path.getState(0), link);
    Eigen::Vector3d this_vertex;

    // Convert path coordinates
    for (std::size_t i = 1; i < path.getStateCount(); ++i)
    {
      // Get current coordinates
      this_vertex = stateToPoint(path.getState(i), link);
      // Create line
      visuals_->publishCylinder(prev_vertex, this_vertex, color, thickness, ns);

      // Save these coordinates for next line
      prev_vertex = this_vertex;
    }
  }
}

Eigen::Vector3d MoveItVizWindow::stateToPoint(const ob::ScopedState<> state, const moveit::core::LinkModel* eef_link)
{
  return stateToPoint(state.get(), eef_link);
}

Eigen::Vector3d MoveItVizWindow::stateToPoint(const ob::State* state, const moveit::core::LinkModel* eef_link)
{
  if (!state)
  {
    ROS_FATAL_NAMED(name_, "No state found for vertex");
    exit(1);
  }

  // Make sure a robot state is available
  //visuals_->loadSharedRobotState();

  // Get StateSpace
  moveit_ompl::ModelBasedStateSpacePtr mb_state_space =
      std::static_pointer_cast<moveit_ompl::ModelBasedStateSpace>(si_->getStateSpace());

  // Convert to robot state
  mb_state_space->copyToRobotState(*visuals_->getRootRobotState(), state);

  // Get pose
  Eigen::Affine3d pose = visuals_->getRootRobotState()->getGlobalLinkTransform(eef_link);

  return pose.translation();
}

void MoveItVizWindow::publishState(const ob::State* state, const rvt::colors& color, const rvt::scales scale,
                                   const std::string& ns)
{
  for (const moveit::core::LinkModel* link : eef_link_models_)
    visuals_->publishSphere(stateToPoint(state, link), color, scale, ns);
}

void MoveItVizWindow::publishState(const ob::State* state, const rvt::colors& color, const double scale,
                                   const std::string& ns)
{
  for (const moveit::core::LinkModel* link : eef_link_models_)
    visuals_->publishSphere(stateToPoint(state, link), color, scale, ns);
}

void MoveItVizWindow::publishState(const ob::ScopedState<> state, const rvt::colors& color, const rvt::scales scale,
                                   const std::string& ns)
{
  for (const moveit::core::LinkModel* link : eef_link_models_)
    visuals_->publishSphere(stateToPoint(state, link), color, scale, ns);
}

void MoveItVizWindow::publishState(const ob::ScopedState<> state, const rvt::colors& color, double scale,
                                   const std::string& ns)
{
  for (const moveit::core::LinkModel* link : eef_link_models_)
    visuals_->publishSphere(stateToPoint(state, link), color, scale, ns);
}

void MoveItVizWindow::publishState(const ob::ScopedState<> state, const rvt::colors& color,
                                   const geometry_msgs::Vector3& scale, const std::string& ns)
{
  for (const moveit::core::LinkModel* link : eef_link_models_)
    visuals_->publishSphere(stateToPoint(state, link), color, scale.x, ns);
}

void MoveItVizWindow::publishSampleRegion(const ob::ScopedState<>& state_area, const double& distance)
{
  temp_point_.x = state_area[0];
  temp_point_.y = state_area[1];
  temp_point_.z = state_area[2];

  visuals_->publishSphere(temp_point_, rvt::BLACK, rvt::REGULAR, "sample_region");  // mid point
  // outer sphere (x2 b/c its a radius, x0.1 to make it look nicer)
  visuals_->publishSphere(temp_point_, rvt::TRANSLUCENT, rvt::REGULAR, "sample_region");
}

bool MoveItVizWindow::convertPath(const og::PathGeometric& path, const robot_model::JointModelGroup* jmg,
                                  robot_trajectory::RobotTrajectoryPtr& traj, double speed)
{
  // Error check
  if (path.getStateCount() <= 0)
  {
    ROS_WARN_STREAM_NAMED(name_, "No states found in path");
    return false;
  }

  // New trajectory
  traj.reset(new robot_trajectory::RobotTrajectory(visuals_->getRobotModel(), jmg));
  moveit::core::RobotState state(*visuals_->getSharedRobotState());  // TODO(davetcoleman):do i need to copy this?

  // Get correct type of space
  moveit_ompl::ModelBasedStateSpacePtr mb_state_space =
      std::static_pointer_cast<moveit_ompl::ModelBasedStateSpace>(si_->getStateSpace());

  // Convert solution to MoveIt! format, reversing the solution
  // for (std::size_t i = path.getStateCount(); i > 0; --i)
  for (std::size_t i = 0; i < path.getStateCount(); ++i)
  {
    // Convert format
    mb_state_space->copyToRobotState(state, path.getState(i));

    // Add to trajectory
    traj->addSuffixWayPoint(state, speed);
  }

  return true;
}

}  // namespace bolt_moveit
