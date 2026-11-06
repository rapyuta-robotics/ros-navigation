#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <typeinfo>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_core/planner_exceptions.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "global_planner/global_planner.h"

using namespace std::chrono_literals;

namespace
{
using geometry_msgs::msg::PoseStamped;

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

bool notCanceled()
{
  return false;
}

/** Expects @p call to throw exactly @p E with an untagged message. */
template <typename E>
void expectThrows(const std::function<void()>& call)
{
  try
  {
    call();
    ADD_FAILURE() << "no exception thrown";
  }
  catch (const std::exception& e)
  {
    EXPECT_EQ(typeid(e), typeid(E)) << "unexpected exception type for: " << e.what();
    EXPECT_EQ(std::string(e.what()).find("[rr_code="), std::string::npos) << e.what();
  }
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

  void configure(const std::string& name = "GlobalPlanner")
  {
    planner_ = std::make_unique<global_planner::GlobalPlanner>();
    planner_->configure(node_, name, nullptr, costmap_ros_);
    planner_->activate();
  }

  nav_msgs::msg::Path createPlan(const PoseStamped& start, const PoseStamped& goal)
  {
    return planner_->createPlan(start, goal, {}, notCanceled);
  }

  bool setParameter(const rclcpp::Parameter& parameter)
  {
    return node_->set_parameter(parameter).successful;
  }

  /** Cells [x0, x1] x [y0, y1] set to @p cost. */
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

TEST_F(GlobalPlannerTest, CreatePlanBeforeConfigureThrows)
{
  global_planner::GlobalPlanner planner;
  expectThrows<nav2_core::PlannerException>(
      [&] { planner.createPlan(pose(1.0, 1.0), pose(3.0, 1.0), {}, notCanceled); });
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

TEST_F(GlobalPlannerTest, FailuresThrowUntaggedNav2Exceptions)
{
  configure();
  const PoseStamped start = pose(1.01, 1.01);
  const PoseStamped goal = pose(3.01, 3.01);
  expectThrows<nav2_core::StartOutsideMapBounds>([&] { createPlan(pose(-1.0, 1.0), goal); });
  expectThrows<nav2_core::GoalOutsideMapBounds>([&] { createPlan(start, pose(6.0, 1.0)); });

  block(60, 60, 60, 60, nav2_costmap_2d::LETHAL_OBSTACLE);
  expectThrows<nav2_core::NoValidPathCouldBeFound>([&] { createPlan(start, goal); });
  block(60, 60, 60, 60, nav2_costmap_2d::FREE_SPACE);

  // Enclosed free goal with a tolerance that stays inside the enclosure.
  block(50, 50, 70, 50, nav2_costmap_2d::LETHAL_OBSTACLE);
  block(50, 70, 70, 70, nav2_costmap_2d::LETHAL_OBSTACLE);
  block(50, 50, 50, 70, nav2_costmap_2d::LETHAL_OBSTACLE);
  block(70, 50, 70, 70, nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_TRUE(setParameter(rclcpp::Parameter("GlobalPlanner.tolerance", 0.3)));
  expectThrows<nav2_core::NoValidPathCouldBeFound>([&] { createPlan(start, goal); });
}

TEST_F(GlobalPlannerTest, ExtractionFailureReturnsAnEmptyPath)
{
  makeNode({ rclcpp::Parameter("GlobalPlanner.use_grid_path", true) });
  configure();
  // Start near the far corner of its cell: the grid path trace oscillates around the start cell and gives up.
  const auto path = createPlan(pose(20.99 * 0.05, 20.99 * 0.05), pose(3.01, 1.01));
  EXPECT_TRUE(path.poses.empty());
  EXPECT_EQ(path.header.frame_id, costmap_ros_->getGlobalFrameID());
}

TEST_F(GlobalPlannerTest, ToleranceParameterAppliesToTheNextPlan)
{
  configure();
  block(55, 15, 65, 25, nav2_costmap_2d::LETHAL_OBSTACLE);  // goal cell (60, 20) inside, 0.25 m to the edge
  const PoseStamped start = pose(1.01, 1.01);
  const PoseStamped goal = pose(3.01, 1.01, 0.2);
  expectThrows<nav2_core::NoValidPathCouldBeFound>([&] { createPlan(start, goal); });
  ASSERT_TRUE(setParameter(rclcpp::Parameter("GlobalPlanner.tolerance", 0.4)));
  const auto path = createPlan(start, goal);
  ASSERT_FALSE(path.poses.empty());
  const auto& end = path.poses.back().pose;
  EXPECT_LT(std::hypot(end.position.x - goal.pose.position.x, end.position.y - goal.pose.position.y), 0.4);
  EXPECT_EQ(end.orientation, goal.pose.orientation);
}

TEST_F(GlobalPlannerTest, ParameterRangesAndReadOnlyParametersAreEnforced)
{
  configure();
  EXPECT_FALSE(setParameter(rclcpp::Parameter("GlobalPlanner.lethal_cost", 0)));
  EXPECT_FALSE(setParameter(rclcpp::Parameter("GlobalPlanner.lethal_cost", 256)));
  EXPECT_TRUE(setParameter(rclcpp::Parameter("GlobalPlanner.lethal_cost", 254)));
  EXPECT_FALSE(setParameter(rclcpp::Parameter("GlobalPlanner.neutral_cost", 0)));
  EXPECT_FALSE(setParameter(rclcpp::Parameter("GlobalPlanner.cost_factor", 5.5)));
  EXPECT_TRUE(setParameter(rclcpp::Parameter("GlobalPlanner.cost_factor", 0.55)));
  EXPECT_FALSE(setParameter(rclcpp::Parameter("GlobalPlanner.tolerance", -0.1)));
  EXPECT_FALSE(setParameter(rclcpp::Parameter("GlobalPlanner.tolerance", "wide")));
  EXPECT_TRUE(setParameter(rclcpp::Parameter("GlobalPlanner.publish_potential", false)));
  for (const auto& name : { "old_navfn_behavior", "use_quadratic", "use_dijkstra", "use_grid_path", "allow_unknown",
                            "outline_map" })
  {
    const std::string full = std::string("GlobalPlanner.") + name;
    EXPECT_FALSE(setParameter(rclcpp::Parameter(full, !node_->get_parameter(full).as_bool()))) << name;
  }
  EXPECT_FALSE(setParameter(rclcpp::Parameter("GlobalPlanner.publish_scale", 50)));
}

TEST_F(GlobalPlannerTest, OnlyAlgorithmParametersAreDeclared)
{
  configure();
  for (const auto& name : { "tolerance", "lethal_cost", "neutral_cost", "cost_factor", "publish_potential",
                            "old_navfn_behavior", "use_quadratic", "use_dijkstra", "use_grid_path", "allow_unknown",
                            "publish_scale", "outline_map" })
  {
    EXPECT_TRUE(node_->has_parameter(std::string("GlobalPlanner.") + name)) << name;
  }
  for (const auto& name : { "orientation_mode", "orientation_window_size", "show_footprint_radii" })
  {
    EXPECT_FALSE(node_->has_parameter(std::string("GlobalPlanner.") + name)) << name;
  }
}

TEST_F(GlobalPlannerTest, OutOfRangeParameterFailsConfigure)
{
  makeNode({ rclcpp::Parameter("GlobalPlanner.lethal_cost", 300) });
  global_planner::GlobalPlanner planner;
  EXPECT_ANY_THROW(planner.configure(node_, "GlobalPlanner", nullptr, costmap_ros_));
}

TEST_F(GlobalPlannerTest, LoadTimeParametersComeFromOverrides)
{
  makeNode({ rclcpp::Parameter("GlobalPlanner.allow_unknown", false),
             rclcpp::Parameter("GlobalPlanner.neutral_cost", 100),
             rclcpp::Parameter("GlobalPlanner.cost_factor", 0.55) });
  configure();
  EXPECT_EQ(node_->get_parameter("GlobalPlanner.neutral_cost").as_int(), 100);
  block(50, 0, 50, 99, nav2_costmap_2d::NO_INFORMATION);
  expectThrows<nav2_core::NoValidPathCouldBeFound>([&] { createPlan(pose(1.01, 2.51), pose(4.01, 2.51)); });
}

TEST_F(GlobalPlannerTest, CreatePlanHoldsTheCostmapLockAndBlocksParameterUpdates)
{
  configure();
  block(55, 15, 65, 25, nav2_costmap_2d::LETHAL_OBSTACLE);
  const PoseStamped start = pose(1.01, 1.01);
  const PoseStamped goal = pose(3.01, 1.01);
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_ros_->getCostmap()->getMutex());
  auto planning = std::async(std::launch::async, [&] { return createPlan(start, goal); });
  EXPECT_EQ(planning.wait_for(300ms), std::future_status::timeout);
  auto setting = std::async(std::launch::async, [&] {
    return setParameter(rclcpp::Parameter("GlobalPlanner.tolerance", 0.4));
  });
  EXPECT_EQ(setting.wait_for(300ms), std::future_status::timeout);
  costmap_lock.unlock();

  // The plan in progress kept the zero tolerance; the update applies to the next plan.
  EXPECT_THROW(planning.get(), nav2_core::NoValidPathCouldBeFound);
  EXPECT_TRUE(setting.get());
  EXPECT_FALSE(createPlan(start, goal).poses.empty());
}

TEST_F(GlobalPlannerTest, PublishesOnlyThePotentialUnderTheNestedPluginName)
{
  configure("GlobalPlanner.primary_planner");
  const std::string topic = "/GlobalPlanner/primary_planner/potential";
  EXPECT_EQ(node_->count_publishers(topic), 1u);
  for (const auto& [name, types] : node_->get_topic_names_and_types())
  {
    EXPECT_FALSE(name.ends_with("/plan")) << name;
    EXPECT_FALSE(name.ends_with("/footprint_radii")) << name;
  }

  auto listener = std::make_shared<rclcpp::Node>("listener");
  nav_msgs::msg::OccupancyGrid::SharedPtr potential;
  auto potential_sub = listener->create_subscription<nav_msgs::msg::OccupancyGrid>(
      topic, 10, [&](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { potential = msg; });
  ASSERT_TRUE(spinUntil(listener, [&] { return node_->count_subscribers(topic) == 1; }));

  createPlan(pose(1.01, 1.01), pose(3.01, 1.01));
  ASSERT_TRUE(spinUntil(listener, [&] { return potential != nullptr; }));
  EXPECT_EQ(potential->info.width, 100u);
  EXPECT_EQ(potential->header.frame_id, costmap_ros_->getGlobalFrameID());
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
