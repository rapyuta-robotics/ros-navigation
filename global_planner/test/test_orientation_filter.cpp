#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include <angles/angles.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "global_planner/orientation_filter.h"

namespace
{
using geometry_msgs::msg::PoseStamped;
using global_planner::OrientationFilter;
using global_planner::OrientationMode;

constexpr double kEps = 1e-9;

PoseStamped pose(double x, double y, double yaw = 0.0)
{
  PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  p.pose.orientation.x = q.x();
  p.pose.orientation.y = q.y();
  p.pose.orientation.z = q.z();
  p.pose.orientation.w = q.w();
  return p;
}

double yaw(const PoseStamped& p)
{
  return tf2::getYaw(p.pose.orientation);
}

/** Path through @p points with identity orientations, except the goal yaw on the last pose. */
std::vector<PoseStamped> path(const std::vector<std::pair<double, double>>& points, double goal_yaw)
{
  std::vector<PoseStamped> poses;
  for (const auto& [x, y] : points)
  {
    poses.push_back(pose(x, y));
  }
  poses.back() = pose(points.back().first, points.back().second, goal_yaw);
  return poses;
}

std::vector<PoseStamped> straightX(int n, double goal_yaw)
{
  std::vector<std::pair<double, double>> points;
  for (int i = 0; i < n; ++i)
  {
    points.emplace_back(0.1 * i, 0.0);
  }
  return path(points, goal_yaw);
}

std::vector<PoseStamped> filtered(OrientationMode mode, const PoseStamped& start, std::vector<PoseStamped> poses)
{
  OrientationFilter filter;
  filter.setMode(mode);
  filter.setWindowSize(1);
  filter.processPath(start, poses);
  return poses;
}

void expectInteriorYaw(const std::vector<PoseStamped>& poses, double expected)
{
  for (std::size_t i = 0; i + 1 < poses.size(); ++i)
  {
    EXPECT_NEAR(angles::shortest_angular_distance(yaw(poses[i]), expected), 0.0, kEps) << "pose " << i;
  }
}

}  // namespace

TEST(OrientationFilter, EveryModeKeepsTheGoalOrientationOnTheLastPose)
{
  for (int mode = 0; mode <= 9; ++mode)
  {
    const auto poses = filtered(static_cast<OrientationMode>(mode), pose(0.0, 0.0, 0.3), straightX(6, 1.0));
    EXPECT_NEAR(yaw(poses.back()), 1.0, kEps) << "mode " << mode;
  }
}

TEST(OrientationFilter, NoneLeavesTheOrientationsUntouched)
{
  const auto poses = filtered(global_planner::NONE, pose(0.0, 0.0, 0.3), straightX(5, 1.0));
  expectInteriorYaw(poses, 0.0);
}

TEST(OrientationFilter, ForwardBackwardLeftwardRightwardOffsetTheTangent)
{
  const PoseStamped start = pose(0.0, 0.0, 0.3);
  expectInteriorYaw(filtered(global_planner::FORWARD, start, straightX(5, 1.0)), 0.0);
  expectInteriorYaw(filtered(global_planner::BACKWARD, start, straightX(5, 1.0)), M_PI);
  expectInteriorYaw(filtered(global_planner::LEFTWARD, start, straightX(5, 1.0)), -M_PI_2);
  expectInteriorYaw(filtered(global_planner::RIGHTWARD, start, straightX(5, 1.0)), M_PI_2);
}

TEST(OrientationFilter, BidirectionalFlipsOnlyWhenTurningExceedsPi)
{
  // Robot and goal facing backward along a +x path: turning 2*pi forward, so it drives backward.
  expectInteriorYaw(filtered(global_planner::BIDIRECTIONAL, pose(0.0, 0.0, M_PI), straightX(6, M_PI)), M_PI);
  // Facing forward: no flip.
  expectInteriorYaw(filtered(global_planner::BIDIRECTIONAL, pose(0.0, 0.0, 0.0), straightX(6, 0.0)), 0.0);
  // Turning exactly pi is not more than pi: no flip.
  expectInteriorYaw(filtered(global_planner::BIDIRECTIONAL, pose(0.0, 0.0, M_PI), straightX(6, 0.0)), 0.0);
}

TEST(OrientationFilter, BidirectionalNeverFlipsTwoPosePaths)
{
  expectInteriorYaw(filtered(global_planner::BIDIRECTIONAL, pose(0.0, 0.0, M_PI), straightX(2, M_PI)), 0.0);
}

TEST(OrientationFilter, BidirectionalUsesTheSecondPoseFromEachEndForFiveOrMorePoses)
{
  // Pose 1 points backward (pi), pose 2 forward (0); the goal yaw adds 0.2 of turning.
  const std::vector<std::pair<double, double>> points{ { 0.0, 0.0 }, { -0.5, 0.0 }, { -1.0, 0.0 }, { 3.0, 0.0 },
                                                       { 4.0, 0.0 } };
  const auto poses = filtered(global_planner::BIDIRECTIONAL, pose(0.0, 0.0, 0.0), path(points, 0.2));
  // With pose 1 the turning would be pi + 0.2 (flip); with pose 2 it is 0.2 (no flip).
  EXPECT_NEAR(yaw(poses[1]), M_PI, kEps);
  EXPECT_NEAR(yaw(poses[2]), 0.0, kEps);

  // Four poses use the first and last neighbours: pose 1 at pi gives pi + 0.2 of turning, so it flips.
  const std::vector<std::pair<double, double>> four{ { 0.0, 0.0 }, { -0.5, 0.0 }, { -1.0, 0.0 }, { 3.0, 0.0 } };
  const auto flipped = filtered(global_planner::BIDIRECTIONAL, pose(0.0, 0.0, 0.0), path(four, 0.2));
  EXPECT_NEAR(std::fabs(yaw(flipped[1])), 0.0, kEps);
}

TEST(OrientationFilter, FixedOrientationCopiesTheStartOrientation)
{
  expectInteriorYaw(filtered(global_planner::FIXEDORIENTATION, pose(0.0, 0.0, 0.7), straightX(5, 1.0)), 0.7);
}

TEST(OrientationFilter, SidewardPicksTheSideThatTurnsLess)
{
  // Facing +y on a +x path: the robot moves rightward (yaw = tangent + pi/2).
  expectInteriorYaw(filtered(global_planner::SIDEWARD, pose(0.0, 0.0, M_PI_2), straightX(6, M_PI_2)), M_PI_2);
  // Facing -y: leftward (yaw = tangent - pi/2).
  expectInteriorYaw(filtered(global_planner::SIDEWARD, pose(0.0, 0.0, -M_PI_2), straightX(6, -M_PI_2)), -M_PI_2);
  // Symmetric case: equal turning both ways picks rightward.
  expectInteriorYaw(filtered(global_planner::SIDEWARD, pose(0.0, 0.0, 0.0), straightX(6, 0.0)), M_PI_2);
}

TEST(OrientationFilter, InterpolateKeepsStartAndGoalOrientation)
{
  const auto poses = filtered(global_planner::INTERPOLATE, pose(0.0, 0.0, 0.0), straightX(5, 1.0));
  const std::vector<double> expected{ 0.0, 0.25, 0.5, 0.75, 1.0 };
  for (std::size_t i = 0; i < poses.size(); ++i)
  {
    EXPECT_NEAR(yaw(poses[i]), expected[i], kEps) << "pose " << i;
  }
}

TEST(OrientationFilter, ForwardThenInterpolateKeepsTheIndexIncrementQuirk)
{
  // +y leg then +x leg: the last straightaway starts at pose 3 (yaw 0).
  const std::vector<std::pair<double, double>> points{ { 0.0, 0.0 }, { 0.0, 1.0 }, { 0.0, 2.0 },
                                                       { 1.0, 2.0 }, { 2.0, 2.0 }, { 3.0, 2.0 } };
  const auto poses = filtered(global_planner::FORWARDTHENINTERPOLATE, pose(0.0, 0.0, 0.1), path(points, 0.5));
  EXPECT_NEAR(yaw(poses[0]), 0.1, kEps);  // start orientation
  EXPECT_NEAR(yaw(poses[1]), M_PI_2, kEps);
  EXPECT_NEAR(yaw(poses[2]), M_PI_4, kEps);
  EXPECT_NEAR(yaw(poses[3]), 0.0, kEps);  // interpolation start keeps its orientation
  // start_yaw + increment * i with i = 4 (not i - 3): 0 + 0.25 * 4.
  EXPECT_NEAR(yaw(poses[4]), 1.0, kEps);
  EXPECT_NEAR(yaw(poses[5]), 0.5, kEps);  // goal orientation
}
