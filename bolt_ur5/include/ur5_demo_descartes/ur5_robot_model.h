/*
 * ur5_robot_model.h
 *
 *  Created on: Apr 15, 2015
 *      Author: ros-devel
 */

#ifndef UR5_DEMO_DESCARTES_UR5_ROBOT_MODEL_H_
#define UR5_DEMO_DESCARTES_UR5_ROBOT_MODEL_H_

#include <descartes_moveit/moveit_state_adapter.h>
#include <ros/ros.h>
#include <urdf/model.h>
#include <eigen_conversions/eigen_kdl.h>
#include <tf_conversions/tf_kdl.h>
#include <ur5_demo_descartes/ur_moveit_plugin.h>
#include <ur5_demo_descartes/ur_kin.h>


namespace ur5_demo_descartes
{

DESCARTES_CLASS_FORWARD(UR5RobotModel);

const std::string UR5_BASE_LINK = "base_link";
const std::string UR5_TIP_LINK = "ee_link";

class UR5RobotModel: public descartes_moveit::MoveitStateAdapter, public ur_kinematics::URKinematicsPlugin
{
public:
  UR5RobotModel(const std::string& prefix = "");
  virtual ~UR5RobotModel();

  virtual bool initialize(const std::string &robot_description, const std::string& group_name,
                            const std::string& world_frame,const std::string& tcp_frame);

  virtual bool initialize(robot_model::RobotModelConstPtr robot_model, const std::string& group_name,
                          const std::string& world_frame, const std::string& tcp_frame);

  virtual bool initializeHelper(const std::string& group_name, const std::string& world_frame,
                                const std::string& tcp_frame);

  virtual bool getAllIK(const Eigen::Affine3d &pose, std::vector<std::vector<double> > &joint_poses) const;

  descartes_core::Frame world_to_base_;// world to arm base
  descartes_core::Frame tool_to_tip_; // from arm tool to robot tool
  std::string prefix_;
  const std::string name_ = "ur5_robot_model";
};

} /* namespace ur5_demo_descartes */

#endif /* UR5_DEMO_DESCARTES_UR5_ROBOT_MODEL_H_ */
