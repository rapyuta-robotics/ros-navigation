#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <angles/angles.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rr_nav_msgs/msg/get_path_outcome.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "global_planner/planner_core.h"

namespace
{
using geometry_msgs::msg::PoseStamped;
using global_planner::PlannerConfig;
using global_planner::PlannerCore;
using global_planner::PlannerOptions;
using Outcome = rr_nav_msgs::msg::GetPathOutcome;

PoseStamped pose(double x, double y, double yaw = 0.0, const std::string& frame = "map")
{
  PoseStamped p;
  p.header.frame_id = frame;
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

/** Core recording the publish hooks. */
class RecordingCore : public PlannerCore
{
public:
  using PlannerCore::PlannerCore;

  /** Number of potential publications. */
  int potential_count = 0;
  /** Number of plan publications. */
  int plan_count = 0;
  /** Number of footprint radii publications. */
  int radii_count = 0;
  /** Number of footprint radii deletions. */
  int clear_count = 0;
  /** Last published potential grid. */
  nav_msgs::msg::OccupancyGrid last_potential;

protected:
  void publishPlan(const std::vector<PoseStamped>&) override
  {
    ++plan_count;
  }
  void publishPotential(const float* potential) override
  {
    ++potential_count;
    last_potential = potentialGrid(potential);
  }
  void showFootprintRadii() override
  {
    ++radii_count;
  }
  void clearFootprintRadii() override
  {
    ++clear_count;
  }
};

class PlannerCoreTest : public ::testing::Test
{
protected:
  /** 5 m x 5 m map of free cells. */
  void makeMap(double resolution = 0.05, unsigned char value = nav2_costmap_2d::FREE_SPACE)
  {
    const auto cells = static_cast<unsigned int>(std::lround(5.0 / resolution));
    costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(cells, cells, resolution, 0.0, 0.0, value);
  }

  RecordingCore& core(const PlannerOptions& options = PlannerOptions(), const PlannerConfig& config = PlannerConfig())
  {
    if (!costmap_)
    {
      makeMap();
    }
    core_ = std::make_unique<RecordingCore>(costmap_.get(), "map", options, config,
                                            std::make_shared<rclcpp::Clock>(), rclcpp::get_logger("test"));
    return *core_;
  }

  void setCost(double x, double y, unsigned char cost)
  {
    unsigned int mx, my;
    ASSERT_TRUE(costmap_->worldToMap(x, y, mx, my));
    costmap_->setCost(mx, my, cost);
  }

  /** Fills the cells [x0, x1] x [y0, y1] (cell indices). */
  void fill(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned char cost)
  {
    for (unsigned int x = x0; x <= x1; ++x)
    {
      for (unsigned int y = y0; y <= y1; ++y)
      {
        costmap_->setCost(x, y, cost);
      }
    }
  }

  /** Lethal square ring of cells around (cx, cy) at Chebyshev distance r. */
  void ring(unsigned int cx, unsigned int cy, unsigned int r)
  {
    fill(cx - r, cy - r, cx + r, cy - r, nav2_costmap_2d::LETHAL_OBSTACLE);
    fill(cx - r, cy + r, cx + r, cy + r, nav2_costmap_2d::LETHAL_OBSTACLE);
    fill(cx - r, cy - r, cx - r, cy + r, nav2_costmap_2d::LETHAL_OBSTACLE);
    fill(cx + r, cy - r, cx + r, cy + r, nav2_costmap_2d::LETHAL_OBSTACLE);
  }

  uint32_t plan(const PoseStamped& start, const PoseStamped& goal, double tolerance)
  {
    std::string message;
    return core_->makePlan(start, goal, tolerance, plan_, message);
  }

  std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  std::unique_ptr<RecordingCore> core_;
  std::vector<PoseStamped> plan_;
};

}  // namespace

TEST_F(PlannerCoreTest, FreeSpacePlanStartsAtTheStartAndEndsWithTheGoalPose)
{
  core();
  const PoseStamped goal = pose(3.01, 2.01, 0.7);
  ASSERT_EQ(plan(pose(1.01, 1.01), goal, 0.0), Outcome::SUCCESS);
  ASSERT_GE(plan_.size(), 3u);
  EXPECT_EQ(plan_.back().pose, goal.pose);
  for (const auto& p : plan_)
  {
    EXPECT_EQ(p.header.frame_id, "map");
  }
  EXPECT_EQ(core_->plan_count, 1);
  EXPECT_EQ(core_->potential_count, 1);
  EXPECT_EQ(core_->radii_count, 1);
}

TEST_F(PlannerCoreTest, StartAndGoalOnCellBordersStayInTheirCells)
{
  // Exact binary resolution so that the border coordinates are exact.
  makeMap(0.0625);
  core();
  const double res = 0.0625;
  const PoseStamped start = pose(16 * res, 16 * res);
  const PoseStamped goal = pose(48 * res, 32 * res);
  ASSERT_EQ(plan(start, goal, 0.0), Outcome::SUCCESS);
  ASSERT_GE(plan_.size(), 3u);
  // First pose: fractional start + half a cell; second to last: fractional goal + half a cell.
  EXPECT_DOUBLE_EQ(plan_.front().pose.position.x, 16.5 * res);
  EXPECT_DOUBLE_EQ(plan_.front().pose.position.y, 16.5 * res);
  const auto& before_goal = plan_[plan_.size() - 2].pose.position;
  EXPECT_DOUBLE_EQ(before_goal.x, 48.5 * res);
  EXPECT_DOUBLE_EQ(before_goal.y, 32.5 * res);
  unsigned int mx, my;
  ASSERT_TRUE(costmap_->worldToMap(before_goal.x, before_goal.y, mx, my));
  EXPECT_EQ(mx, 48u);
  EXPECT_EQ(my, 32u);
}

TEST_F(PlannerCoreTest, BlockedStartFromTheInscribedCost)
{
  for (const unsigned char cost : { 253, 254, 255 })
  {
    makeMap();
    core();
    setCost(1.01, 1.01, cost);
    EXPECT_EQ(plan(pose(1.01, 1.01), pose(3.01, 1.01), 0.0), Outcome::BLOCKED_START) << int(cost);
    EXPECT_TRUE(plan_.empty());
    EXPECT_EQ(core_->potential_count, 0);
  }
  makeMap();
  core();
  setCost(1.01, 1.01, 252);
  EXPECT_EQ(plan(pose(1.01, 1.01), pose(3.01, 1.01), 0.0), Outcome::SUCCESS);
}

TEST_F(PlannerCoreTest, BlockedGoalWithZeroToleranceBeforeAnyPublication)
{
  core();
  setCost(3.01, 1.01, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(plan(pose(1.01, 1.01), pose(3.01, 1.01), 0.0), Outcome::BLOCKED_GOAL);
  EXPECT_EQ(core_->potential_count, 0);
  EXPECT_EQ(core_->radii_count, 0);
  EXPECT_EQ(core_->plan_count, 0);
}

TEST_F(PlannerCoreTest, FreeButUnreachableGoalWithZeroToleranceIsBlockedGoal)
{
  makeMap();
  core();
  ring(60, 60, 5);  // encloses the goal cell (60, 60)
  EXPECT_EQ(plan(pose(1.01, 1.01), pose(3.01, 3.01), 0.0), Outcome::BLOCKED_GOAL);
  // The potential and the radii are published before reporting the blocked goal.
  EXPECT_EQ(core_->potential_count, 1);
  EXPECT_EQ(core_->radii_count, 1);
  EXPECT_EQ(core_->plan_count, 0);
  EXPECT_EQ(core_->last_potential.info.width, 100u);
}

TEST_F(PlannerCoreTest, FreeGoalCellWithNothingReachableInToleranceIsNoPathFound)
{
  makeMap();
  core();
  ring(60, 60, 10);  // 0.5 m around the goal, tolerance 0.3 m inside the enclosure
  EXPECT_EQ(plan(pose(1.01, 1.01), pose(3.01, 3.01), 0.3), Outcome::NO_PATH_FOUND);
  EXPECT_TRUE(plan_.empty());
  EXPECT_EQ(core_->plan_count, 1);  // the empty plan is published
}

TEST_F(PlannerCoreTest, BlockedGoalCellWithFreeUnreachableCellsInToleranceIsBlockedGoal)
{
  makeMap();
  core();
  ring(60, 60, 10);
  setCost(3.01, 3.01, nav2_costmap_2d::LETHAL_OBSTACLE);  // only the goal cell decides
  EXPECT_EQ(plan(pose(1.01, 1.01), pose(3.01, 3.01), 0.3), Outcome::BLOCKED_GOAL);
  EXPECT_EQ(core_->potential_count, 1);
}

TEST_F(PlannerCoreTest, ToleranceSearchPicksTheNearestReachableSampleInsideTheDisc)
{
  makeMap(0.05, nav2_costmap_2d::LETHAL_OBSTACLE);
  core();
  fill(10, 18, 90, 22, nav2_costmap_2d::FREE_SPACE);  // corridor y in [0.90, 1.15), start inside
  const PoseStamped goal = pose(3.0, 1.55, 0.4);
  ASSERT_EQ(plan(pose(0.61, 1.01), goal, 0.5), Outcome::SUCCESS);
  const auto& end = plan_.back();
  const double distance = std::hypot(end.pose.position.x - goal.pose.position.x,
                                     end.pose.position.y - goal.pose.position.y);
  EXPECT_LT(distance, 0.5);
  // Nearest corridor cell row is y = 22 ([1.10, 1.15)): below the goal, 0.40 m to 0.45 m away, within one
  // sampling step (half a cell) of the goal x.
  EXPECT_NEAR(end.pose.position.x, goal.pose.position.x, 0.025);
  EXPECT_GE(distance, 0.40);
  EXPECT_LT(distance, 0.45);
  EXPECT_EQ(end.pose.orientation, goal.pose.orientation);
  unsigned int mx, my;
  ASSERT_TRUE(costmap_->worldToMap(end.pose.position.x, end.pose.position.y, mx, my));
  EXPECT_EQ(costmap_->getCost(mx, my), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(PlannerCoreTest, ToleranceSearchIsADiscNotASquare)
{
  makeMap(0.05, nav2_costmap_2d::LETHAL_OBSTACLE);
  core();
  fill(10, 18, 40, 22, nav2_costmap_2d::FREE_SPACE);  // corridor ends at x < 2.05, y < 1.15
  const PoseStamped start = pose(0.61, 1.01);
  // Nearest free corner is 0.4 m away on both axes: 0.566 m, inside the 0.5 m square but outside the disc.
  const PoseStamped goal = pose(2.45, 1.55);
  EXPECT_EQ(plan(start, goal, 0.5), Outcome::BLOCKED_GOAL);
  EXPECT_EQ(plan(start, goal, 0.6), Outcome::SUCCESS);
}

TEST_F(PlannerCoreTest, CostsBelowLethalArePassable)
{
  const PoseStamped start = pose(1.01, 2.51);
  const PoseStamped goal = pose(4.01, 2.51);
  makeMap();
  core();
  fill(50, 0, 50, 99, 252);  // wall of 252 across the map
  EXPECT_EQ(plan(start, goal, 0.0), Outcome::SUCCESS);

  makeMap();
  core();
  fill(50, 0, 50, 99, 253);
  EXPECT_EQ(plan(start, goal, 0.0), Outcome::BLOCKED_GOAL);
}

TEST_F(PlannerCoreTest, UnknownCellsFollowAllowUnknown)
{
  const PoseStamped start = pose(1.01, 2.51);
  const PoseStamped goal = pose(4.01, 2.51);
  PlannerConfig config;
  config.lethal_cost = 253;
  PlannerOptions options;

  makeMap();
  options.allow_unknown = true;
  core(options, config);
  fill(50, 0, 50, 99, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_EQ(plan(start, goal, 0.0), Outcome::SUCCESS);

  makeMap();
  options.allow_unknown = false;
  core(options, config);
  fill(50, 0, 50, 99, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_EQ(plan(start, goal, 0.0), Outcome::BLOCKED_GOAL);
}

TEST_F(PlannerCoreTest, OutOfMapStartOrGoal)
{
  core();
  EXPECT_EQ(plan(pose(-0.1, 1.0), pose(3.0, 1.0), 0.0), Outcome::OUT_OF_MAP);
  EXPECT_EQ(plan(pose(1.0, 1.0), pose(5.1, 1.0), 0.0), Outcome::OUT_OF_MAP);
}

TEST_F(PlannerCoreTest, FrameMismatchIsInvalidStartOrGoal)
{
  core();
  EXPECT_EQ(plan(pose(1.0, 1.0), pose(3.0, 1.0, 0.0, "odom"), 0.0), Outcome::INVALID_GOAL);
  EXPECT_EQ(plan(pose(1.0, 1.0, 0.0, "odom"), pose(3.0, 1.0), 0.0), Outcome::INVALID_START);
}

TEST_F(PlannerCoreTest, ReconfigureClearsTheRadiiWhenDisabled)
{
  core();
  PlannerConfig config;
  config.show_footprint_radii = false;
  core_->reconfigure(config);
  EXPECT_EQ(core_->clear_count, 1);
  ASSERT_EQ(plan(pose(1.01, 1.01), pose(3.01, 1.01), 0.0), Outcome::SUCCESS);
  EXPECT_EQ(core_->radii_count, 0);

  config.show_footprint_radii = true;
  config.publish_potential = false;
  core_->reconfigure(config);
  EXPECT_EQ(core_->clear_count, 1);
  ASSERT_EQ(plan(pose(1.01, 1.01), pose(3.01, 1.01), 0.0), Outcome::SUCCESS);
  EXPECT_EQ(core_->radii_count, 1);
  EXPECT_EQ(core_->potential_count, 1);  // only the first plan published it
}

TEST_F(PlannerCoreTest, OrientationModeAppliesToThePlan)
{
  core();
  const PoseStamped start = pose(1.01, 1.01);
  const PoseStamped goal = pose(4.01, 2.01, 0.3);
  ASSERT_EQ(plan(start, goal, 0.0), Outcome::SUCCESS);
  const auto forward = plan_;
  PlannerConfig config;
  config.orientation_mode = global_planner::BACKWARD;
  core_->reconfigure(config);
  ASSERT_EQ(plan(start, goal, 0.0), Outcome::SUCCESS);
  ASSERT_EQ(plan_.size(), forward.size());
  // Same poses, driven backward: every yaw but the goal's turns by pi.
  for (std::size_t i = 0; i + 1 < plan_.size(); ++i)
  {
    EXPECT_EQ(plan_[i].pose.position, forward[i].pose.position);
    const double turn = angles::shortest_angular_distance(tf2::getYaw(forward[i].pose.orientation),
                                                          tf2::getYaw(plan_[i].pose.orientation));
    EXPECT_NEAR(std::fabs(turn), M_PI, 1e-6) << "pose " << i;
  }
  EXPECT_EQ(plan_.back().pose, goal.pose);
}
