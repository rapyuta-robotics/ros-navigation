#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/trajectory_cost_function.h>

namespace base_local_planner
{
class AlignWithPathFunction : public base_local_planner::TrajectoryCostFunction
{
public:
  AlignWithPathFunction() = default;

  void setTargetPoses(std::vector<geometry_msgs::PoseStamped>& target_poses,
                      const geometry_msgs::PoseStamped& global_pose);

  bool prepare();

  double scoreTrajectory(Trajectory& traj);

private:
  double path_yaw_;
  double goal_x_;
  double goal_y_;
};

} /* namespace base_local_planner */
