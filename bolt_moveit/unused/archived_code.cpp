bool visualizeMoveItCartPath(const Eigen::Affine3d &start_pose);
bool computeFullMoveItTrajectory(std::vector<moveit::core::RobotStatePtr> &trajectory);

bool CartPathPlanner::visualizeMoveItCartPath(const Eigen::Affine3d &start_pose)
{
  // Plan cartesian path
  trajectory_.clear();
  Eigen::Vector3d rotated_direction;
  rotated_direction << 0, -1, 0;
  double desired_distance = 0.5;  // Distance to move
  double max_step = 0.01;  // Resolution of trajectory, the maximum distance in Cartesian space between consecutive
                           // points on the resulting path

  // this is the Cartesian pose we start from, and we move in the direction indicated
  trajectory_.push_back(moveit::core::RobotStatePtr(new moveit::core::RobotState(*imarker_state_)));

  // The target pose is built by applying a translation to the start pose for the desired direction and distance
  Eigen::Affine3d target_pose = start_pose;
  target_pose.translation() += rotated_direction * desired_distance;

  // Create new state
  moveit::core::RobotState robot_state(*imarker_state_);
  // Decide how many steps we will need for this trajectory
  double distance = (target_pose.translation() - start_pose.translation()).norm();
  unsigned int steps = 5 + (unsigned int)floor(distance / max_step);

  std::vector<double> dist_vector;
  double total_dist = 0.0;
  double last_valid_percentage = 0.0;
  Eigen::Quaterniond start_quaternion(start_pose.rotation());
  Eigen::Quaterniond target_quaternion(target_pose.rotation());

  imarker_cartesian_->getVisualTools()->enableBatchPublishing();
  imarker_cartesian_->getVisualTools()->deleteAllMarkers();
  for (unsigned int i = 1; i <= steps; ++i)
  {
    double percentage = (double)i / (double)steps;

    Eigen::Affine3d pose(start_quaternion.slerp(percentage, target_quaternion));
    pose.translation() = percentage * target_pose.translation() + (1 - percentage) * start_pose.translation();

    // Visualize
    imarker_cartesian_->getVisualTools()->publishArrow(pose, rvt::ORANGE, rvt::SMALL, 0.02, i);

    if (robot_state.setFromIK(parent_->jmg_, pose, parent_->ee_link_->getName(), 1, 0.0))
    {
      trajectory_.push_back(moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_state)));

      // compute the distance to the previous point (infinity norm)
      double dist_prev_point = trajectory_.back()->distance(*trajectory_[trajectory_.size() - 2], parent_->jmg_);
      dist_vector.push_back(dist_prev_point);
      total_dist += dist_prev_point;
    }
    else
      break;
    last_valid_percentage = percentage;
  }
  imarker_cartesian_->getVisualTools()->triggerBatchPublishAndDisable();

  // std::cout << "last_valid_percentage: " << last_valid_percentage << std::endl;

  if (!last_valid_percentage)
  {
    ROS_WARN_STREAM_NAMED(name_, "No solution found");
    return false;
  }

  // Visualize path
  // double speed = 0.001;
  // bool blocking = true;
  // imarker_cartesian_->getVisualTools()->publishTrajectoryPath(trajectory_, parent_->jmg_, speed, blocking);

  return true;
}

bool CartPathPlanner::computeFullMoveItTrajectory(std::vector<moveit::core::RobotStatePtr> &trajectory)
{
  if (trajectory_.empty())
  {
    imarker_state_ = imarker_cartesian_->getRobotState();
    Eigen::Affine3d start_pose = imarker_state_->getGlobalLinkTransform(parent_->ee_link_);
    visualizeMoveItCartPath(start_pose);
  }

  if (trajectory_.empty())
    return false;

  trajectory = trajectory_;
  return true;
}
