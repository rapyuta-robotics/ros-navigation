#include <base_local_planner/align_with_path_function.h>
#include <math.h>       /* atan2 */
#include <angles/angles.h>

namespace base_local_planner {

const double ANGLE_THRESHOLD = 0.8;
const double PENALIZING_WEIGHT = 20.0;
const double MIN_GOAL_DIST_SQ = 0.7;

void AlignWithPathFunction::setTargetPoses(std::vector<geometry_msgs::PoseStamped>& target_poses, const geometry_msgs::PoseStamped& global_pose) {
  setScale(0);

  const int num_points = target_poses.size();
  if (num_points <= 0) {
    return;
  }
  // todo: decide which point to pick
  geometry_msgs::PoseStamped path_node =  *(target_poses.begin() + std::min(7, num_points-1));
  path_yaw_ = 2 * atan2 (path_node.pose.orientation.z, path_node.pose.orientation.w);
  goal_x_ = target_poses.back().pose.position.x;
  goal_y_ = target_poses.back().pose.position.y;

  const double& px = global_pose.pose.position.x;
  const double& py = global_pose.pose.position.y;
  const double squared_dist = (goal_x_ - px) * (goal_x_ - px) + (goal_y_ - py) * (goal_y_ - py);

  if (squared_dist < MIN_GOAL_DIST_SQ) {
    return;
  }

  const double current_yaw = 2 * atan2 (global_pose.pose.orientation.z, global_pose.pose.orientation.w);
  const double current_yaw_diff = fabs(angles::normalize_angle(path_yaw_ - current_yaw));

  if (current_yaw_diff < ANGLE_THRESHOLD) {
	return;
  }

  setScale(PENALIZING_WEIGHT);
}

bool AlignWithPathFunction::prepare() {
  return true;
}

double AlignWithPathFunction::scoreTrajectory(Trajectory &traj) {
  if (traj.getPointsSize() <= 0) {
    return M_PI;
  }

  double px, py, pth;
  traj.getEndpoint(px, py, pth);

  const double end_yaw_diff = fabs(angles::normalize_angle(path_yaw_ - pth));
  return end_yaw_diff;
}

} /* namespace base_local_planner */
