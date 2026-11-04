#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <typeinfo>
#include <thread>
#include <vector>

#include <angles/angles.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_core/planner_exceptions.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rr_nav_msgs/msg/get_path_outcome.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "global_planner/global_planner.h"

using namespace std::chrono_literals;

namespace
{
using geometry_msgs::msg::PoseStamped;
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

bool notCanceled()
{
  return false;
}

/** Expects @p call to throw exactly @p E with the message tagged with @p code. */
template <typename E>
void expectTagged(const std::function<void()>& call, uint32_t code)
{
  try
  {
    call();
    ADD_FAILURE() << "no exception thrown";
  }
  catch (const std::exception& e)
  {
    EXPECT_EQ(typeid(e), typeid(E)) << "unexpected exception type for: " << e.what();
    const std::string tag = "[rr_code=" + std::to_string(code) + "] ";
    EXPECT_EQ(std::string(e.what()).rfind(tag, 0), 0u) << e.what();
  }
}

/** Expects the same poses as @p reference with every yaw but the goal's turned by @p offset. */
void expectYawOffset(const nav_msgs::msg::Path& reference, const nav_msgs::msg::Path& path, double offset)
{
  ASSERT_EQ(path.poses.size(), reference.poses.size());
  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i)
  {
    EXPECT_EQ(path.poses[i].pose.position, reference.poses[i].pose.position);
    const double turn = angles::shortest_angular_distance(tf2::getYaw(reference.poses[i].pose.orientation),
                                                          tf2::getYaw(path.poses[i].pose.orientation));
    EXPECT_NEAR(std::fabs(turn), offset, 1e-6) << "pose " << i;
  }
  EXPECT_EQ(path.poses.back().pose, reference.poses.back().pose);
}

class GlobalPlannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    makeNode({});
  }

  void TearDown() override
  {
    if (planner_)
    {
      planner_->deactivate();
      planner_->cleanup();
    }
    planner_.reset();
    costmap_ros_.reset();
    node_.reset();
  }

  void makeNode(const std::vector<rclcpp::Parameter>& overrides)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    node_ = std::make_shared<nav2::LifecycleNode>("planner_tester", "", options);
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap", "/ns", false);
    costmap_ros_->configure();
    costmap_ros_->getCostmap()->resizeMap(100, 100, 0.05, 0.0, 0.0);
  }

  void configure()
  {
    planner_ = std::make_unique<global_planner::GlobalPlanner>();
    planner_->configure(node_, "GlobalPlanner", nullptr, costmap_ros_);
    planner_->activate();
  }

  nav_msgs::msg::Path createPlan(const PoseStamped& start, const PoseStamped& goal,
                                 const std::vector<PoseStamped>& viapoints = {})
  {
    return planner_->createPlan(start, goal, viapoints, notCanceled);
  }

  bool setParameter(const rclcpp::Parameter& parameter)
  {
    return node_->set_parameter(parameter).successful;
  }

  /** Lethal block of cells [x0, x1] x [y0, y1]. */
  void block(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned char cost)
  {
    for (unsigned int x = x0; x <= x1; ++x)
    {
      for (unsigned int y = y0; y <= y1; ++y)
      {
        costmap_ros_->getCostmap()->setCost(x, y, cost);
      }
    }
  }

  /** Spins @p listener until @p done or a timeout; true if done. */
  static bool spinUntil(const rclcpp::Node::SharedPtr& listener, const std::function<bool()>& done)
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(listener);
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!done() && std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    return done();
  }

  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<global_planner::GlobalPlanner> planner_;
};

}  // namespace

TEST(GlobalPlannerPlugin, IsExportedForNav2Core)
{
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core", "nav2_core::GlobalPlanner");
  const auto classes = loader.getDeclaredClasses();
  EXPECT_NE(std::find(classes.begin(), classes.end(), "global_planner::GlobalPlanner"), classes.end());
  EXPECT_NO_THROW(loader.createSharedInstance("global_planner::GlobalPlanner"));
}

TEST_F(GlobalPlannerTest, NotConfiguredIsTaggedNotInitialized)
{
  global_planner::GlobalPlanner planner;
  expectTagged<nav2_core::PlannerException>(
      [&] { planner.createPlan(pose(1.0, 1.0), pose(3.0, 1.0), {}, notCanceled); }, Outcome::NOT_INITIALIZED);
}

TEST_F(GlobalPlannerTest, SuccessReturnsAPlanInTheCostmapFrame)
{
  configure();
  const PoseStamped goal = pose(3.01, 2.01, 0.5);
  const auto path = createPlan(pose(1.01, 1.01), goal);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_EQ(path.header.frame_id, costmap_ros_->getGlobalFrameID());
  EXPECT_EQ(path.poses.back().pose, goal.pose);
}

TEST_F(GlobalPlannerTest, OutcomesThrowTaggedNav2Exceptions)
{
  configure();
  const PoseStamped start = pose(1.01, 1.01);
  const PoseStamped goal = pose(3.01, 3.01);
  expectTagged<nav2_core::StartOutsideMapBounds>([&] { createPlan(pose(-1.0, 1.0), goal); }, Outcome::OUT_OF_MAP);
  expectTagged<nav2_core::GoalOutsideMapBounds>([&] { createPlan(start, pose(6.0, 1.0)); }, Outcome::OUT_OF_MAP);
  expectTagged<nav2_core::PlannerException>([&] { createPlan(start, pose(3.0, 1.0, 0.0, "odom")); },
                                            Outcome::INVALID_GOAL);
  expectTagged<nav2_core::PlannerException>([&] { createPlan(pose(1.0, 1.0, 0.0, "odom"), goal); },
                                            Outcome::INVALID_START);

  block(20, 20, 20, 20, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  expectTagged<nav2_core::StartOccupied>([&] { createPlan(start, goal); }, Outcome::BLOCKED_START);
  block(20, 20, 20, 20, nav2_costmap_2d::FREE_SPACE);

  block(60, 60, 60, 60, nav2_costmap_2d::LETHAL_OBSTACLE);
  expectTagged<nav2_core::GoalOccupied>([&] { createPlan(start, goal); }, Outcome::BLOCKED_GOAL);
  block(60, 60, 60, 60, nav2_costmap_2d::FREE_SPACE);

  // Enclosed free goal with a tolerance that stays inside the enclosure.
  block(50, 50, 70, 50, nav2_costmap_2d::LETHAL_OBSTACLE);
  block(50, 70, 70, 70, nav2_costmap_2d::LETHAL_OBSTACLE);
  block(50, 50, 50, 70, nav2_costmap_2d::LETHAL_OBSTACLE);
  block(70, 50, 70, 70, nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(setParameter(rclcpp::Parameter("GlobalPlanner.tolerance", 0.3)));
  expectTagged<nav2_core::NoValidPathCouldBeFound>([&] { createPlan(start, goal); }, Outcome::NO_PATH_FOUND);
  // With zero tolerance the same free but unreachable goal is reported blocked.
  ASSERT_TRUE(setParameter(rclcpp::Parameter("GlobalPlanner.tolerance", 0.0)));
  expectTagged<nav2_core::GoalOccupied>([&] { createPlan(start, goal); }, Outcome::BLOCKED_GOAL);
}

TEST_F(GlobalPlannerTest, ViapointsAreRejectedAsInvalidGoal)
{
  configure();
  expectTagged<nav2_core::PlannerException>(
      [&] { createPlan(pose(1.01, 1.01), pose(3.01, 1.01), { pose(2.0, 1.0) }); }, Outcome::INVALID_GOAL);
}

TEST_F(GlobalPlannerTest, ToleranceParameterAppliesToTheNextPlan)
{
  configure();
  block(55, 15, 65, 25, nav2_costmap_2d::LETHAL_OBSTACLE);  // goal cell (60, 20) inside, 0.25 m to the edge
  const PoseStamped start = pose(1.01, 1.01);
  const PoseStamped goal = pose(3.01, 1.01, 0.2);
  expectTagged<nav2_core::GoalOccupied>([&] { createPlan(start, goal); }, Outcome::BLOCKED_GOAL);
  ASSERT_TRUE(setParameter(rclcpp::Parameter("GlobalPlanner.tolerance", 0.4)));
  const auto path = createPlan(start, goal);
  ASSERT_FALSE(path.poses.empty());
  const auto& end = path.poses.back().pose;
  EXPECT_LT(std::hypot(end.position.x - goal.pose.position.x, end.position.y - goal.pose.position.y), 0.4);
  EXPECT_EQ(end.orientation, goal.pose.orientation);
}

TEST_F(GlobalPlannerTest, OrientationModeParameterAppliesToTheNextPlan)
{
  configure();
  const PoseStamped start = pose(1.01, 1.01);
  const PoseStamped goal = pose(4.01, 1.01);
  const auto forward = createPlan(start, goal);
  ASSERT_TRUE(setParameter(rclcpp::Parameter("GlobalPlanner.orientation_mode", 4)));
  expectYawOffset(forward, createPlan(start, goal), M_PI);
}

TEST_F(GlobalPlannerTest, ParameterRangesAndReadOnlyParametersAreEnforced)
{
  configure();
  EXPECT_FALSE(setParameter(rclcpp::Parameter("GlobalPlanner.orientation_mode", 10)));
  EXPECT_FALSE(setParameter(rclcpp::Parameter("GlobalPlanner.orientation_mode", -1)));
  EXPECT_TRUE(setParameter(rclcpp::Parameter("GlobalPlanner.orientation_mode", 9)));
  EXPECT_FALSE(setParameter(rclcpp::Parameter("GlobalPlanner.lethal_cost", 0)));
  EXPECT_FALSE(setParameter(rclcpp::Parameter("GlobalPlanner.lethal_cost", 256)));
  EXPECT_FALSE(setParameter(rclcpp::Parameter("GlobalPlanner.neutral_cost", 0)));
  EXPECT_FALSE(setParameter(rclcpp::Parameter("GlobalPlanner.cost_factor", 5.5)));
  EXPECT_TRUE(setParameter(rclcpp::Parameter("GlobalPlanner.cost_factor", 0.55)));
  EXPECT_FALSE(setParameter(rclcpp::Parameter("GlobalPlanner.orientation_window_size", 0)));
  EXPECT_FALSE(setParameter(rclcpp::Parameter("GlobalPlanner.tolerance", -0.1)));
  EXPECT_FALSE(setParameter(rclcpp::Parameter("GlobalPlanner.orientation_mode", "forward")));
  for (const auto& name : { "old_navfn_behavior", "use_quadratic", "use_dijkstra", "use_grid_path", "allow_unknown",
                            "outline_map" })
  {
    const std::string full = std::string("GlobalPlanner.") + name;
    EXPECT_FALSE(setParameter(rclcpp::Parameter(full, !node_->get_parameter(full).as_bool()))) << name;
  }
  EXPECT_FALSE(setParameter(rclcpp::Parameter("GlobalPlanner.publish_scale", 50)));
}

TEST_F(GlobalPlannerTest, OutOfRangeParameterFailsConfigure)
{
  makeNode({ rclcpp::Parameter("GlobalPlanner.orientation_mode", 12) });
  global_planner::GlobalPlanner planner;
  EXPECT_ANY_THROW(planner.configure(node_, "GlobalPlanner", nullptr, costmap_ros_));
}

TEST_F(GlobalPlannerTest, LoadTimeParametersComeFromOverrides)
{
  makeNode({ rclcpp::Parameter("GlobalPlanner.allow_unknown", false),
             rclcpp::Parameter("GlobalPlanner.neutral_cost", 100),
             rclcpp::Parameter("GlobalPlanner.cost_factor", 0.55) });
  configure();
  block(50, 0, 50, 99, nav2_costmap_2d::NO_INFORMATION);
  expectTagged<nav2_core::GoalOccupied>([&] { createPlan(pose(1.01, 2.51), pose(4.01, 2.51)); },
                                        Outcome::BLOCKED_GOAL);
}

TEST_F(GlobalPlannerTest, CreatePlanHoldsTheCostmapLockAndBlocksParameterUpdates)
{
  configure();
  const PoseStamped start = pose(1.01, 1.01);
  const PoseStamped goal = pose(4.01, 1.01);
  const auto forward = createPlan(start, goal);
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_ros_->getCostmap()->getMutex());
  auto planning = std::async(std::launch::async, [&] { return createPlan(start, goal); });
  EXPECT_EQ(planning.wait_for(300ms), std::future_status::timeout);
  auto setting = std::async(std::launch::async, [&] {
    return setParameter(rclcpp::Parameter("GlobalPlanner.orientation_mode", 4));
  });
  EXPECT_EQ(setting.wait_for(300ms), std::future_status::timeout);
  costmap_lock.unlock();

  const auto path = planning.get();
  EXPECT_TRUE(setting.get());
  // The plan in progress kept the forward orientation; the update applies to the next plan.
  expectYawOffset(forward, path, 0.0);
  expectYawOffset(forward, createPlan(start, goal), M_PI);
}

TEST_F(GlobalPlannerTest, PublishesUnderThePluginName)
{
  configure();
  EXPECT_EQ(node_->count_publishers("/GlobalPlanner/plan"), 1u);
  EXPECT_EQ(node_->count_publishers("/GlobalPlanner/potential"), 1u);
  EXPECT_EQ(node_->count_publishers("/GlobalPlanner/footprint_radii"), 1u);

  auto listener = std::make_shared<rclcpp::Node>("listener");
  nav_msgs::msg::Path::SharedPtr plan;
  nav_msgs::msg::OccupancyGrid::SharedPtr potential;
  std::vector<visualization_msgs::msg::Marker> markers;
  auto plan_sub = listener->create_subscription<nav_msgs::msg::Path>(
      "/GlobalPlanner/plan", 10, [&](nav_msgs::msg::Path::SharedPtr msg) { plan = msg; });
  auto potential_sub = listener->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/GlobalPlanner/potential", 10, [&](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { potential = msg; });
  auto radii_sub = listener->create_subscription<visualization_msgs::msg::Marker>(
      "/GlobalPlanner/footprint_radii", 10,
      [&](visualization_msgs::msg::Marker::SharedPtr msg) { markers.push_back(*msg); });
  ASSERT_TRUE(spinUntil(listener, [&] {
    return node_->count_subscribers("/GlobalPlanner/plan") == 1 &&
           node_->count_subscribers("/GlobalPlanner/potential") == 1 &&
           node_->count_subscribers("/GlobalPlanner/footprint_radii") == 1;
  }));

  const auto path = createPlan(pose(1.01, 1.01), pose(3.01, 1.01));
  ASSERT_TRUE(spinUntil(listener, [&] { return plan && potential && markers.size() == 2; }));
  EXPECT_EQ(plan->poses.size(), path.poses.size());
  EXPECT_EQ(potential->info.width, 100u);
  EXPECT_EQ(markers[0].ns, "inscribed_radius");
  EXPECT_EQ(markers[1].ns, "circumscribed_radius");
  EXPECT_EQ(markers[0].header.frame_id, costmap_ros_->getBaseFrameID());

  markers.clear();
  ASSERT_TRUE(setParameter(rclcpp::Parameter("GlobalPlanner.show_footprint_radii", false)));
  ASSERT_TRUE(spinUntil(listener, [&] { return !markers.empty(); }));
  EXPECT_EQ(markers[0].action, visualization_msgs::msg::Marker::DELETEALL);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
