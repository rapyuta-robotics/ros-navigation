#include <global_planner/global_planner.h>

#include <algorithm>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <nav2_core/planner_exceptions.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rr_nav_msgs/msg/get_path_outcome.hpp>

//register this planner as a nav2_core::GlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav2_core::GlobalPlanner)

namespace global_planner {

using GetPathOutcome = rr_nav_msgs::msg::GetPathOutcome;

namespace {

// Same "[rr_code=<N>] <message>" tag as the rr_nav_adapter error codec, which decodes it from error_msg.
std::string tag(uint32_t code, const std::string& message)
{
    return "[rr_code=" + std::to_string(code) + "] " + message;
}

}  // namespace

PublishingPlannerCore::PublishingPlannerCore(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
                                             const PlannerOptions& options, const PlannerConfig& config,
                                             rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger,
                                             nav2::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub,
                                             nav2::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr potential_pub,
                                             nav2::Publisher<visualization_msgs::msg::Marker>::SharedPtr fp_radii_pub) :
        PlannerCore(costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID(), options, config, std::move(clock),
                    std::move(logger)),
        costmap_ros_(std::move(costmap_ros)), plan_pub_(std::move(plan_pub)),
        potential_pub_(std::move(potential_pub)), fp_radii_pub_(std::move(fp_radii_pub)) {
}

void PublishingPlannerCore::publishPlan(const std::vector<geometry_msgs::msg::PoseStamped>& path) {
    //create a message for the plan
    nav_msgs::msg::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = clock_->now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_->publish(gui_path);
}

void PublishingPlannerCore::publishPotential(const float* potential)
{
    // The full-map grid is only built when someone listens.
    if (potential_pub_->get_subscription_count() == 0)
        return;
    potential_pub_->publish(potentialGrid(potential));
}

void PublishingPlannerCore::showFootprintRadii()
{
    const auto [inscribed_radius, circumscribed_radius] =
        nav2_costmap_2d::calculateMinAndMaxDistances(costmap_ros_->getRobotFootprint());

    // show the inscribed radius (in blue)
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = costmap_ros_->getBaseFrameID();
    marker.header.stamp = clock_->now();
    marker.ns = "inscribed_radius";
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.points = nav2_costmap_2d::makeFootprintFromRadius(inscribed_radius);
    marker.points.push_back(marker.points.front());  // close the polygon
    marker.scale.x = 0.01;
    marker.color.b = 1.0;
    marker.color.a = 0.5;
    fp_radii_pub_->publish(marker);

    // repeat for the circumscribed radius (in red)
    marker.ns = "circumscribed_radius";
    marker.points = nav2_costmap_2d::makeFootprintFromRadius(circumscribed_radius);
    marker.points.push_back(marker.points.front());  // close the polygon
    marker.color.r = 1.0;
    marker.color.b = 0.0;
    fp_radii_pub_->publish(marker);
}

void PublishingPlannerCore::clearFootprintRadii()
{
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  fp_radii_pub_->publish(marker);
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

    // Topics live under the plugin name on the planner node, e.g. GlobalPlanner/plan; nested plugin names such as
    // GlobalPlanner.primary_planner map to GlobalPlanner/primary_planner.
    std::string topic_prefix = name_;
    std::replace(topic_prefix.begin(), topic_prefix.end(), '.', '/');
    plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(topic_prefix + "/plan");
    potential_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(topic_prefix + "/potential");
    fp_radii_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(topic_prefix + "/footprint_radii");

    core_ = std::make_unique<PublishingPlannerCore>(costmap_ros_, params.options, params.config, clock_, logger_,
                                                    plan_pub_, potential_pub_, fp_radii_pub_);

    param_handler_->setApplyCallback([this](const Parameters& updated) { core_->reconfigure(updated.config); });
    // Registered from configure to cleanup so that updates made while inactive are not lost.
    param_handler_->activate();
}

void GlobalPlanner::cleanup() {
    if (param_handler_)
        param_handler_->deactivate();
    core_.reset();
    param_handler_.reset();
    plan_pub_.reset();
    potential_pub_.reset();
    fp_radii_pub_.reset();
    costmap_ros_.reset();
}

void GlobalPlanner::activate() {
    plan_pub_->on_activate();
    potential_pub_->on_activate();
    fp_radii_pub_->on_activate();
}

void GlobalPlanner::deactivate() {
    plan_pub_->on_deactivate();
    potential_pub_->on_deactivate();
    fp_radii_pub_->on_deactivate();
}

nav_msgs::msg::Path GlobalPlanner::createPlan(const geometry_msgs::msg::PoseStamped& start,
                                              const geometry_msgs::msg::PoseStamped& goal,
                                              const std::vector<geometry_msgs::msg::PoseStamped>& viapoints,
                                              std::function<bool()> /*cancel_checker*/) {
    if (!core_) {
        const std::string message =
            "This planner has not been initialized yet, but it is being used, please call configure() before use";
        RCLCPP_ERROR_STREAM(logger_, message);
        throw nav2_core::PlannerException(tag(GetPathOutcome::NOT_INITIALIZED, message));
    }
    if (!viapoints.empty()) {
        const std::string message = "This planner does not support viapoints";
        RCLCPP_ERROR_STREAM(logger_, message);
        throw nav2_core::PlannerException(tag(GetPathOutcome::INVALID_GOAL, message));
    }

    std::lock_guard<std::mutex> params_lock(param_handler_->getMutex());
    const double tolerance = param_handler_->getParams()->tolerance;
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_ros_->getCostmap()->getMutex());

    std::vector<geometry_msgs::msg::PoseStamped> plan;
    std::string message;
    uint32_t outcome;
    try {
        outcome = core_->makePlan(start, goal, tolerance, plan, message);
    } catch (const std::exception& e) {
        throw nav2_core::PlannerException(tag(GetPathOutcome::INTERNAL_ERROR, e.what()));
    }
    if (outcome != GetPathOutcome::SUCCESS)
        throwOutcome(outcome, start, message);

    nav_msgs::msg::Path path;
    path.header.frame_id = costmap_ros_->getGlobalFrameID();
    path.header.stamp = clock_->now();
    path.poses = std::move(plan);
    return path;
}

void GlobalPlanner::throwOutcome(uint32_t outcome, const geometry_msgs::msg::PoseStamped& start,
                                 const std::string& message) const {
    const std::string what = tag(outcome, message);
    switch (outcome) {
        case GetPathOutcome::BLOCKED_START:
            throw nav2_core::StartOccupied(what);
        case GetPathOutcome::BLOCKED_GOAL:
            throw nav2_core::GoalOccupied(what);
        case GetPathOutcome::NO_PATH_FOUND:
            throw nav2_core::NoValidPathCouldBeFound(what);
        case GetPathOutcome::OUT_OF_MAP: {
            // makePlan checks the start first, so a start inside the map means the goal is outside.
            unsigned int mx, my;
            if (!costmap_ros_->getCostmap()->worldToMap(start.pose.position.x, start.pose.position.y, mx, my))
                throw nav2_core::StartOutsideMapBounds(what);
            throw nav2_core::GoalOutsideMapBounds(what);
        }
        default:
            throw nav2_core::PlannerException(what);
    }
}

} //end namespace global_planner
