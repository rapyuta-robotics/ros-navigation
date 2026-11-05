#include <global_planner/global_planner.h>

#include <algorithm>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <nav2_core/planner_exceptions.hpp>
#include <pluginlib/class_list_macros.hpp>

//register this planner as a nav2_core::GlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav2_core::GlobalPlanner)

namespace global_planner {

PublishingPlannerCore::PublishingPlannerCore(nav2_costmap_2d::Costmap2DROS& costmap_ros,
                                             const PlannerOptions& options, const PlannerConfig& config,
                                             rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger,
                                             nav2::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr potential_pub) :
        PlannerCore(costmap_ros.getCostmap(), costmap_ros.getGlobalFrameID(), options, config, std::move(clock),
                    std::move(logger)),
        potential_pub_(std::move(potential_pub)) {
}

void PublishingPlannerCore::publishPotential(const float* potential)
{
    // The full-map grid is only built when someone listens.
    if (potential_pub_->get_subscription_count() == 0)
        return;
    potential_pub_->publish(potentialGrid(potential));
}

GlobalPlanner::GlobalPlanner() = default;

GlobalPlanner::~GlobalPlanner() = default;

void GlobalPlanner::configure(const nav2::LifecycleNode::WeakPtr& parent, std::string name,
                              nav2::TransformBuffer::SharedPtr /*tf*/,
                              std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    auto node = parent.lock();
    if (!node)
        throw std::runtime_error("GlobalPlanner: unable to lock the parent node");
    name_ = std::move(name);
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    costmap_ros_ = std::move(costmap_ros);

    param_handler_ = std::make_unique<ParameterHandler>(node, name_, logger_);
    const Parameters params = *param_handler_->getParams();

    // Topics live under the plugin name, with nested plugin names such as GlobalPlanner.primary_planner mapped to
    // GlobalPlanner/primary_planner.
    std::string topic_prefix = name_;
    std::replace(topic_prefix.begin(), topic_prefix.end(), '.', '/');
    potential_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(topic_prefix + "/potential");

    core_ = std::make_unique<PublishingPlannerCore>(*costmap_ros_, params.options, params.config, clock_, logger_,
                                                    potential_pub_);

    param_handler_->setApplyCallback([this](const Parameters& updated) { core_->reconfigure(updated.config); });
    // Registered from configure to cleanup so that updates made while inactive are not lost.
    param_handler_->activate();
}

void GlobalPlanner::cleanup() {
    if (param_handler_)
        param_handler_->deactivate();
    core_.reset();
    param_handler_.reset();
    potential_pub_.reset();
    costmap_ros_.reset();
}

void GlobalPlanner::activate() {
    potential_pub_->on_activate();
}

void GlobalPlanner::deactivate() {
    potential_pub_->on_deactivate();
}

nav_msgs::msg::Path GlobalPlanner::createPlan(const geometry_msgs::msg::PoseStamped& start,
                                              const geometry_msgs::msg::PoseStamped& goal,
                                              const std::vector<geometry_msgs::msg::PoseStamped>& /*viapoints*/,
                                              std::function<bool()> /*cancel_checker*/) {
    if (!core_)
        throw nav2_core::PlannerException("GlobalPlanner: createPlan called before configure");

    std::lock_guard<std::mutex> params_lock(param_handler_->getMutex());
    const double tolerance = param_handler_->getParams()->tolerance;
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_ros_->getCostmap()->getMutex());

    nav_msgs::msg::Path path;
    switch (core_->makePlan(start, goal, tolerance, path.poses)) {
        case PlanStatus::StartOutsideMap:
            throw nav2_core::StartOutsideMapBounds("The start is outside the global costmap");
        case PlanStatus::GoalOutsideMap:
            throw nav2_core::GoalOutsideMapBounds("The goal is outside the global costmap");
        case PlanStatus::NoPotential:
            throw nav2_core::NoValidPathCouldBeFound(
                "No potential at the goal or within the tolerance of " + std::to_string(tolerance) + " m");
        case PlanStatus::Success:
        case PlanStatus::ExtractionFailed:
            break;
    }
    path.header.frame_id = costmap_ros_->getGlobalFrameID();
    path.header.stamp = clock_->now();
    return path;
}

} //end namespace global_planner
