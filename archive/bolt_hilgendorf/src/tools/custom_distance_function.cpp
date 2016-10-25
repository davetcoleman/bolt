// TODO: this was a project I started but abandoned for now

double customDistanceFunction(const ompl::base::State *state1, const ompl::base::State *state2)
{
  std::cout << "no one should call this " << std::endl;
  exit(-1);

  space_->copyToRobotState(*from_state_, state1);
  space_->copyToRobotState(*to_state_, state2);

  const Eigen::Affine3d from_pose = from_state_->getGlobalLinkTransform(ee_link_);
  const Eigen::Affine3d to_pose = to_state_->getGlobalLinkTransform(ee_link_);

  return getPoseDistance(from_pose, to_pose);
}

double getPoseDistance(const Eigen::Affine3d &from_pose, const Eigen::Affine3d &to_pose)
{
  const double translation_dist = (from_pose.translation() - to_pose.translation()).norm();
  // const double distance_wrist_to_finger = 0.25; // meter

  const Eigen::Quaterniond from(from_pose.rotation());
  const Eigen::Quaterniond to(to_pose.rotation());

  // std::cout << "From: " << from.x() << ", " << from.y() << ", " << from.z() << ", " << from.w() << std::endl;
  // std::cout << "To: " << to.x() << ", " << to.y() << ", " << to.z() << ", " << to.w() << std::endl;

  double rotational_dist = arcLength(from, to);  // * distance_wrist_to_finger;

  std::cout << "  Translation_Dist: " << std::fixed << std::setprecision(4) << translation_dist
            << " rotational_dist: " << rotational_dist << std::endl;

  return rotational_dist + translation_dist;
}

double arcLength(const Eigen::Quaterniond &from, const Eigen::Quaterniond &to)
{
  static const double MAX_QUATERNION_NORM_ERROR = 1e-9;
  double dq = fabs(from.x() * to.x() + from.y() * to.y() + from.z() * to.z() + from.w() * to.w());
  if (dq > 1.0 - MAX_QUATERNION_NORM_ERROR)
    return 0.0;
  else
    return acos(dq);
}
