#include <global_planner/parameter_handler.h>

#include <limits>
#include <mutex>
#include <utility>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>

namespace global_planner {

namespace {

rcl_interfaces::msg::ParameterDescriptor readOnly(const std::string& description)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor integerRange(const std::string& description, int64_t from, int64_t to)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = from;
    descriptor.integer_range[0].to_value = to;
    return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor floatingRange(const std::string& description, double from, double to)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = from;
    descriptor.floating_point_range[0].to_value = to;
    return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor plain(const std::string& description)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    return descriptor;
}

}  // namespace

ParameterHandler::ParameterHandler(const nav2::LifecycleNode::SharedPtr& node, const std::string& plugin_name,
                                   const rclcpp::Logger& logger) :
        nav2_util::ParameterHandler<Parameters>(node, logger), prefix_(plugin_name + ".") {
    PlannerOptions& o = params_.options;
    o.old_navfn_behavior = node->declare_or_get_parameter(prefix_ + "old_navfn_behavior", o.old_navfn_behavior,
        readOnly("Exactly mirror behavior of navfn"));
    o.use_quadratic = node->declare_or_get_parameter(prefix_ + "use_quadratic", o.use_quadratic,
        readOnly("Use the quadratic approximation of the potential"));
    o.use_dijkstra = node->declare_or_get_parameter(prefix_ + "use_dijkstra", o.use_dijkstra,
        readOnly("Use Dijkstra's algorithm, otherwise A*"));
    o.use_grid_path = node->declare_or_get_parameter(prefix_ + "use_grid_path", o.use_grid_path,
        readOnly("Create a path that follows the grid boundaries, otherwise use gradient descent"));
    o.allow_unknown = node->declare_or_get_parameter(prefix_ + "allow_unknown", o.allow_unknown,
        readOnly("Allow the planner to plan through unknown space"));
    o.publish_scale = node->declare_or_get_parameter(prefix_ + "publish_scale", o.publish_scale,
        readOnly("Scale of the published potential"));
    o.outline_map = node->declare_or_get_parameter(prefix_ + "outline_map", o.outline_map,
        readOnly("Outline the global costmap with lethal obstacles"));

    PlannerConfig& c = params_.config;
    c.lethal_cost = node->declare_or_get_parameter(prefix_ + "lethal_cost", c.lethal_cost,
        integerRange("Lethal Cost", 1, 255));
    c.neutral_cost = node->declare_or_get_parameter(prefix_ + "neutral_cost", c.neutral_cost,
        integerRange("Neutral Cost", 1, 255));
    c.cost_factor = node->declare_or_get_parameter(prefix_ + "cost_factor", c.cost_factor,
        floatingRange("Factor to multiply each cost from costmap by", 0.01, 5.0));
    c.publish_potential = node->declare_or_get_parameter(prefix_ + "publish_potential", c.publish_potential,
        plain("Publish Potential Costmap"));

    params_.tolerance = node->declare_or_get_parameter(prefix_ + "tolerance", params_.tolerance,
        floatingRange("How many meters the planner can relax the goal constraint in x and y if the goal is obstructed",
                      0.0, std::numeric_limits<double>::max()));
}

void ParameterHandler::setApplyCallback(std::function<void(const Parameters&)> apply)
{
    std::lock_guard<std::mutex> lock(mutex_);
    apply_ = std::move(apply);
}

rcl_interfaces::msg::SetParametersResult ParameterHandler::validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter>& /*parameters*/)
{
    // Ranges and read-only parameters are enforced by the parameter descriptors.
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void ParameterHandler::updateParametersCallback(const std::vector<rclcpp::Parameter>& parameters)
{
    std::lock_guard<std::mutex> lock(mutex_);
    bool config_changed = false;
    for (const auto& parameter : parameters) {
        const std::string& name = parameter.get_name();
        if (name.rfind(prefix_, 0) != 0)
            continue;
        const std::string key = name.substr(prefix_.size());
        PlannerConfig& c = params_.config;
        if (key == "lethal_cost")
            c.lethal_cost = parameter.as_int();
        else if (key == "neutral_cost")
            c.neutral_cost = parameter.as_int();
        else if (key == "cost_factor")
            c.cost_factor = parameter.as_double();
        else if (key == "publish_potential")
            c.publish_potential = parameter.as_bool();
        else if (key == "tolerance") {
            params_.tolerance = parameter.as_double();
            continue;
        }
        else
            continue;
        config_changed = true;
    }
    if (config_changed && apply_)
        apply_(params_);
}

} //end namespace global_planner
