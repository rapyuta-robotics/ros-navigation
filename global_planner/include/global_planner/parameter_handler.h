#ifndef GLOBAL_PLANNER_PARAMETER_HANDLER_H
#define GLOBAL_PLANNER_PARAMETER_HANDLER_H

#include <functional>
#include <string>
#include <vector>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <nav2_util/parameter_handler.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/parameter.hpp>
#include <global_planner/planner_core.h>

namespace global_planner {

/** Parameters of the GlobalPlanner plugin, declared as <plugin name>.<parameter>. */
struct Parameters {
    /** Read-only settings, read once at configure. */
    PlannerOptions options;
    /** Runtime-writable settings, applied immediately. */
    PlannerConfig config;
    /** Goal tolerance in meters used by the next plan. */
    double tolerance = 0.0;
};

/** Declares the plugin parameters and applies runtime updates under the plugin mutex. */
class ParameterHandler : public nav2_util::ParameterHandler<Parameters> {
    public:
        /** Declares the <plugin_name>.* parameters; descriptors enforce ranges and read-only flags. */
        ParameterHandler(const nav2::LifecycleNode::SharedPtr& node, const std::string& plugin_name,
                         const rclcpp::Logger& logger);

        /** Sets the function called under the mutex with the parameters after a runtime-writable setting changed. */
        void setApplyCallback(std::function<void(const Parameters&)> apply);

    protected:
        /** Accepts every update; the parameter descriptors already reject invalid ones. */
        rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
            const std::vector<rclcpp::Parameter>& parameters) override;

        /** Stores the updated values and calls the apply function when a setting other than tolerance changed. */
        void updateParametersCallback(const std::vector<rclcpp::Parameter>& parameters) override;

    private:
        /** Parameter prefix: the plugin name followed by a dot. */
        std::string prefix_;
        /** Called after every accepted update. */
        std::function<void(const Parameters&)> apply_;
};

} //end namespace global_planner

#endif
