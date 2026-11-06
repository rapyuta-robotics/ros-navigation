#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include "global_planner/planner_core.h"

namespace
{
using geometry_msgs::msg::PoseStamped;
using global_planner::PlanStatus;
using global_planner::PlannerConfig;
using global_planner::PlannerCore;
using global_planner::PlannerOptions;

PoseStamped pose(double x, double y, double yaw = 0.0)
{
  PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

/** Core recording the potential publications. */
class RecordingCore : public PlannerCore
{
public:
  using PlannerCore::PlannerCore;

  /** Number of potential publications. */
  int potential_count = 0;
  /** Last published potential grid. */
  nav_msgs::msg::OccupancyGrid last_potential;

protected:
  void publishPotential(const float* potential) override
  {
    ++potential_count;
    last_potential = potentialGrid(potential);
  }
};

class PlannerCoreTest : public ::testing::Test
{
protected:
  /** 5 m x 5 m map of @p value cells. */
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

  PlanStatus plan(const PoseStamped& start, const PoseStamped& goal, double tolerance)
  {
    return core_->makePlan(start, goal, tolerance, plan_);
  }

  /** Cost of the cell containing @p p. */
  unsigned char costAt(const PoseStamped& p)
  {
    unsigned int mx, my;
    EXPECT_TRUE(costmap_->worldToMap(p.pose.position.x, p.pose.position.y, mx, my));
    return costmap_->getCost(mx, my);
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
  ASSERT_EQ(plan(pose(1.01, 1.01), goal, 0.0), PlanStatus::Success);
  ASSERT_GE(plan_.size(), 3u);
  EXPECT_EQ(plan_.back().pose, goal.pose);
  for (const auto& p : plan_)
  {
    EXPECT_EQ(p.header.frame_id, "map");
  }
  for (std::size_t i = 0; i + 1 < plan_.size(); ++i)
  {
    EXPECT_EQ(plan_[i].pose.orientation, PoseStamped().pose.orientation) << "pose " << i;
  }
  EXPECT_EQ(core_->potential_count, 1);
}

TEST_F(PlannerCoreTest, StartAndGoalOnCellBordersStayInTheirCells)
{
  // Exact binary resolution so that the border coordinates are exact.
  makeMap(0.0625);
  core();
  const double res = 0.0625;
  const PoseStamped start = pose(16 * res, 16 * res);
  const PoseStamped goal = pose(48 * res, 32 * res);
  ASSERT_EQ(plan(start, goal, 0.0), PlanStatus::Success);
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

TEST_F(PlannerCoreTest, OutOfMapStartOrGoalIsReportedBeforeTheSearch)
{
  core();
  EXPECT_EQ(plan(pose(-0.1, 1.0), pose(3.0, 1.0), 0.0), PlanStatus::StartOutsideMap);
  EXPECT_EQ(plan(pose(1.0, 1.0), pose(5.1, 1.0), 0.0), PlanStatus::GoalOutsideMap);
  // The upper map edge belongs to the outside, the last cell to the inside.
  EXPECT_EQ(plan(pose(1.0, 1.0), pose(5.0, 1.0), 0.0), PlanStatus::GoalOutsideMap);
  EXPECT_EQ(plan(pose(1.0, 1.0), pose(1.0, 5.0), 0.0), PlanStatus::GoalOutsideMap);
  EXPECT_TRUE(plan_.empty());
  EXPECT_EQ(core_->potential_count, 0);
}

TEST_F(PlannerCoreTest, LastCellIsPlannableOnlyWithoutTheOutline)
{
  const PoseStamped start = pose(1.01, 1.01);
  const PoseStamped goal = pose(4.99, 1.01);  // last column
  PlannerOptions options;

  makeMap();
  options.outline_map = true;
  core(options);
  EXPECT_EQ(plan(start, goal, 0.0), PlanStatus::NoPotential);
  EXPECT_EQ(costAt(goal), nav2_costmap_2d::LETHAL_OBSTACLE);

  makeMap();
  // The expansion reads the rows above and below each cell: keep the first and last rows lethal.
  fill(0, 0, 99, 0, nav2_costmap_2d::LETHAL_OBSTACLE);
  fill(0, 99, 99, 99, nav2_costmap_2d::LETHAL_OBSTACLE);
  options.outline_map = false;
  core(options);
  ASSERT_EQ(plan(start, goal, 0.0), PlanStatus::Success);
  EXPECT_EQ(plan_.back().pose, goal.pose);
  EXPECT_EQ(costAt(goal), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(PlannerCoreTest, UnreachableGoalWithZeroToleranceHasNoPotential)
{
  core();
  ring(60, 60, 5);  // encloses the goal cell (60, 60)
  EXPECT_EQ(plan(pose(1.01, 1.01), pose(3.01, 3.01), 0.0), PlanStatus::NoPotential);
  EXPECT_TRUE(plan_.empty());
  // The potential is published before reporting the failure.
  EXPECT_EQ(core_->potential_count, 1);
  EXPECT_EQ(core_->last_potential.info.width, 100u);
}

TEST_F(PlannerCoreTest, NothingReachableInsideTheToleranceDiscHasNoPotential)
{
  core();
  ring(60, 60, 10);  // 0.5 m around the goal, tolerance 0.3 m inside the enclosure
  EXPECT_EQ(plan(pose(1.01, 1.01), pose(3.01, 3.01), 0.3), PlanStatus::NoPotential);
  EXPECT_TRUE(plan_.empty());

  setCost(3.01, 3.01, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(plan(pose(1.01, 1.01), pose(3.01, 3.01), 0.3), PlanStatus::NoPotential);
}

TEST_F(PlannerCoreTest, ToleranceAtTheReductionTermSamplesNothingAndTerminates)
{
  // Goal coordinates below 1 m make the tolerance reduction exactly 1e-6, so the disc radius and the step are 0.
  core();
  ring(18, 18, 3);
  EXPECT_EQ(plan(pose(2.01, 2.01), pose(0.91, 0.91), 1e-6), PlanStatus::NoPotential);
  EXPECT_EQ(plan(pose(2.01, 2.01), pose(0.91, 0.91), 0.5e-6), PlanStatus::NoPotential);
}

TEST_F(PlannerCoreTest, LethalGoalCellIsDisplacedInsideTheTolerance)
{
  core();
  setCost(3.01, 1.01, nav2_costmap_2d::LETHAL_OBSTACLE);
  const PoseStamped goal = pose(3.01, 1.01, 0.4);
  EXPECT_EQ(plan(pose(1.01, 1.01), goal, 0.0), PlanStatus::NoPotential);
  ASSERT_EQ(plan(pose(1.01, 1.01), goal, 0.3), PlanStatus::Success);
  const auto& end = plan_.back();
  EXPECT_LT(std::hypot(end.pose.position.x - goal.pose.position.x, end.pose.position.y - goal.pose.position.y), 0.3);
  EXPECT_EQ(end.pose.orientation, goal.pose.orientation);
  EXPECT_LT(costAt(end), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(PlannerCoreTest, ToleranceSearchPicksTheNearestReachableSampleInsideTheDisc)
{
  makeMap(0.05, nav2_costmap_2d::LETHAL_OBSTACLE);
  core();
  fill(10, 18, 90, 22, nav2_costmap_2d::FREE_SPACE);  // corridor y in [0.90, 1.15), start inside
  const PoseStamped goal = pose(3.0, 1.55, 0.4);
  ASSERT_EQ(plan(pose(0.61, 1.01), goal, 0.5), PlanStatus::Success);
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
  EXPECT_EQ(costAt(end), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(PlannerCoreTest, ToleranceSearchIsADiscNotASquare)
{
  makeMap(0.05, nav2_costmap_2d::LETHAL_OBSTACLE);
  core();
  fill(10, 18, 40, 22, nav2_costmap_2d::FREE_SPACE);  // corridor ends at x < 2.05, y < 1.15
  const PoseStamped start = pose(0.61, 1.01);
  // Nearest free corner is 0.4 m away on both axes: 0.566 m, inside the 0.5 m square but outside the disc.
  const PoseStamped goal = pose(2.45, 1.55);
  EXPECT_EQ(plan(start, goal, 0.5), PlanStatus::NoPotential);
  EXPECT_EQ(plan(start, goal, 0.6), PlanStatus::Success);
}

TEST_F(PlannerCoreTest, CostsBelowLethalArePassable)
{
  const PoseStamped start = pose(1.01, 2.51);
  const PoseStamped goal = pose(4.01, 2.51);
  makeMap();
  core();
  fill(50, 0, 50, 99, 252);  // wall of 252 across the map
  EXPECT_EQ(plan(start, goal, 0.0), PlanStatus::Success);

  makeMap();
  core();
  fill(50, 0, 50, 99, 253);
  EXPECT_EQ(plan(start, goal, 0.0), PlanStatus::NoPotential);
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
  EXPECT_EQ(plan(start, goal, 0.0), PlanStatus::Success);

  makeMap();
  options.allow_unknown = false;
  core(options, config);
  fill(50, 0, 50, 99, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_EQ(plan(start, goal, 0.0), PlanStatus::NoPotential);
}

TEST_F(PlannerCoreTest, PotentialGridIsScaledAndFollowsPublishPotential)
{
  core();
  fill(70, 0, 70, 99, nav2_costmap_2d::LETHAL_OBSTACLE);  // cuts off x >= 3.5 m
  ASSERT_EQ(plan(pose(1.01, 1.01), pose(3.01, 1.01), 0.0), PlanStatus::Success);
  ASSERT_EQ(core_->potential_count, 1);
  const auto& grid = core_->last_potential;
  EXPECT_EQ(grid.header.frame_id, "map");
  EXPECT_EQ(grid.info.width, 100u);
  EXPECT_EQ(grid.info.height, 100u);
  EXPECT_FLOAT_EQ(grid.info.resolution, 0.05f);
  ASSERT_EQ(grid.data.size(), 100u * 100u);
  EXPECT_EQ(grid.data[20 * 100 + 20], 0);    // start cell
  EXPECT_EQ(grid.data[20 * 100 + 80], -1);   // unreachable
  EXPECT_EQ(grid.data[0], -1);               // outlined border
  int8_t max = 0;
  for (const auto value : grid.data)
  {
    max = std::max(max, value);
  }
  EXPECT_EQ(max, 100);  // publish_scale

  PlannerConfig config;
  config.publish_potential = false;
  core_->reconfigure(config);
  ASSERT_EQ(plan(pose(1.01, 1.01), pose(3.01, 1.01), 0.0), PlanStatus::Success);
  EXPECT_EQ(core_->potential_count, 1);
}

TEST_F(PlannerCoreTest, ExtractionFailureAfterALegalPotentialLeavesTheReturnedPlanEmpty)
{
  // The grid path stops at the lowest neighbor; with the start near the far corner of its cell, the lowest potential
  // lies in the diagonal cell and the trace oscillates around the start cell until it gives up.
  PlannerOptions options;
  options.use_grid_path = true;
  core(options);
  const double res = 0.05;
  EXPECT_EQ(plan(pose(20.99 * res, 20.99 * res), pose(3.01, 1.01), 0.0), PlanStatus::ExtractionFailed);
  EXPECT_TRUE(plan_.empty());
  EXPECT_EQ(core_->potential_count, 1);

  EXPECT_EQ(plan(pose(20.01 * res, 20.01 * res), pose(3.01, 1.01), 0.0), PlanStatus::Success);
  EXPECT_FALSE(plan_.empty());
}
