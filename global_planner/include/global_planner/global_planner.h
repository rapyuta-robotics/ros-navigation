#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_GLOBAL_PLANNER_H

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <nav2_ros_common/publisher.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <global_planner/parameter_handler.h>
#include <global_planner/planner_core.h>

namespace global_planner {

/** PlannerCore that publishes the plan, the potential and the footprint radii. */
class PublishingPlannerCore : public PlannerCore {
    public:
        /** Plans on the costmap of @p costmap_ros and publishes with the given publishers. */
        PublishingPlannerCore(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
                              const PlannerOptions& options, const PlannerConfig& config,
                              rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger,
                              nav2::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub,
                              nav2::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr potential_pub,
                              nav2::Publisher<visualization_msgs::msg::Marker>::SharedPtr fp_radii_pub);

    protected:
        /** Publishes the plan on GlobalPlanner/plan. */
        void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped>& path) override;
        /** Publishes the potential on GlobalPlanner/potential if it has subscribers. */
        void publishPotential(const float* potential) override;
        /** Publishes the inscribed and circumscribed radii markers. */
        void showFootprintRadii() override;
        /** Publishes a DELETEALL marker. */
        void clearFootprintRadii() override;

    private:
        /** Costmap ROS wrapper providing the footprint and the base frame. */
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        /** Publisher of the plan. */
        nav2::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
        /** Publisher of the potential. */
        nav2::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr potential_pub_;
        /** Publisher of the footprint radii markers. */
        nav2::Publisher<visualization_msgs::msg::Marker>::SharedPtr fp_radii_pub_;
};

/** nav2_core::GlobalPlanner plugin running the global_planner navigation function on the global costmap. */
class GlobalPlanner : public nav2_core::GlobalPlanner {
    public:
        /** Creates an unconfigured planner. */
        GlobalPlanner();
        /** Destroys the planner. */
        ~GlobalPlanner() override;

        /** Declares the parameters, creates the publishers and the planner core. */
        void configure(const nav2::LifecycleNode::WeakPtr& parent, std::string name,
                       nav2::TransformBuffer::SharedPtr tf,
                       std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
        /** Releases the planner core, the publishers and the parameter callbacks. */
        void cleanup() override;
        /** Activates the publishers. */
        void activate() override;
        /** Deactivates the publishers. */
        void deactivate() override;

        /** Plans with the tolerance parameter; returns a non-empty plan or throws a tagged nav2_core exception. */
        nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start,
                                       const geometry_msgs::msg::PoseStamped& goal,
                                       const std::vector<geometry_msgs::msg::PoseStamped>& viapoints,
                                       std::function<bool()> cancel_checker) override;

    private:
        /** Throws the nav2_core exception matching a GetPathOutcome code, with the code tagged in the message. */
        [[noreturn]] void throwOutcome(uint32_t outcome, const geometry_msgs::msg::PoseStamped& start,
                                       const std::string& message) const;

        /** Name of the plugin instance. */
        std::string name_;
        /** Logger of the parent node. */
        rclcpp::Logger logger_{rclcpp::get_logger("global_planner")};
        /** Clock of the parent node. */
        rclcpp::Clock::SharedPtr clock_;
        /** Global costmap. */
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        /** Parameters and the mutex shared by createPlan and parameter updates. */
        std::unique_ptr<ParameterHandler> param_handler_;
        /** Publisher of the plan. */
        nav2::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
        /** Publisher of the potential. */
        nav2::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr potential_pub_;
        /** Publisher of the footprint radii markers. */
        nav2::Publisher<visualization_msgs::msg::Marker>::SharedPtr fp_radii_pub_;
        /** The planner. */
        std::unique_ptr<PublishingPlannerCore> core_;
};

} //end namespace global_planner

#endif
