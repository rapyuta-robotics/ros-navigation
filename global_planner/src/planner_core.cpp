/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <global_planner/planner_core.h>
#include <nav2_costmap_2d/cost_values.hpp>
#include <rclcpp/logging.hpp>

#include <global_planner/dijkstra.h>
#include <global_planner/astar.h>
#include <global_planner/grid_path.h>
#include <global_planner/gradient_path.h>
#include <global_planner/quadratic_calculator.h>

#include <algorithm>
#include <cassert>
#include <cfloat>
#include <utility>

namespace global_planner {

void PlannerCore::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

PlannerCore::PlannerCore(nav2_costmap_2d::Costmap2D* costmap, std::string frame_id, const PlannerOptions& options,
                         const PlannerConfig& config, rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger) :
        costmap_(costmap), frame_id_(std::move(frame_id)), allow_unknown_(options.allow_unknown),
        clock_(std::move(clock)), logger_(std::move(logger)) {
    unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();

    old_navfn_behavior_ = options.old_navfn_behavior;
    if(!old_navfn_behavior_)
        convert_offset_ = 0.5;
    else
        convert_offset_ = 0.0;

    if (options.use_quadratic)
        p_calc_ = std::make_unique<QuadraticCalculator>(cx, cy);
    else
        p_calc_ = std::make_unique<PotentialCalculator>(cx, cy);

    if (options.use_dijkstra)
    {
        auto de = std::make_unique<DijkstraExpansion>(p_calc_.get(), cx, cy);
        if(!old_navfn_behavior_)
            de->setPreciseStart(true);
        planner_ = std::move(de);
    }
    else
        planner_ = std::make_unique<AStarExpansion>(p_calc_.get(), cx, cy);

    if (options.use_grid_path)
        path_maker_ = std::make_unique<GridPath>(p_calc_.get());
    else
        path_maker_ = std::make_unique<GradientPath>(p_calc_.get());

    planner_->setHasUnknown(allow_unknown_);
    publish_scale_ = options.publish_scale;
    outline_map_ = options.outline_map;

    reconfigure(config);
}

PlannerCore::~PlannerCore() = default;

void PlannerCore::reconfigure(const PlannerConfig& config) {
    planner_->setLethalCost(config.lethal_cost);
    path_maker_->setLethalCost(config.lethal_cost);
    planner_->setNeutralCost(config.neutral_cost);
    planner_->setFactor(config.cost_factor);
    publish_potential_ = config.publish_potential;
}

void PlannerCore::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool PlannerCore::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution;
    my = (wy - origin_y) / resolution;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

PlanStatus PlannerCore::makePlan(const geometry_msgs::msg::PoseStamped& start,
                                 const geometry_msgs::msg::PoseStamped& goal, double tolerance,
                                 std::vector<geometry_msgs::msg::PoseStamped>& plan) {
    std::lock_guard<std::mutex> lock(mutex_);

    //clear the plan, just in case
    plan.clear();

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        RCLCPP_ERROR(logger_, "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return PlanStatus::StartOutsideMap;
    }
    if(old_navfn_behavior_){
        start_x = start_x_i;
        start_y = start_y_i;
    }else{
        worldToMap(wx, wy, start_x, start_y);
    }

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        RCLCPP_ERROR(logger_, "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal");
        return PlanStatus::GoalOutsideMap;
    }
    if(old_navfn_behavior_){
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }else{
        worldToMap(wx, wy, goal_x, goal_y);
    }

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);
    potential_array_ = std::make_unique_for_overwrite<float[]>(nx * ny);

    if(outline_map_)
        outlineMap(costmap_->getCharMap(), nx, ny, nav2_costmap_2d::LETHAL_OBSTACLE);

    bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                    nx * ny * 2, potential_array_.get());


    geometry_msgs::msg::PoseStamped best_pose = goal;
    if (!found_legal) {
      // calculatePotentials did not result in a valid potential at the goal cell:
      // Check if any cells within tolerance around the goal have a valid potential:
      // if so, set found_legal to true and displace the goal,
      // such the getPlanFromPotential will generate a path to the displaced goal
      double resolution = costmap_->getResolution();
      geometry_msgs::msg::PoseStamped p = goal;

      double best_sdist = DBL_MAX;

      // reduce tolerance to ensure all poses are clearly inside the tolerance
      // TODO: use constexpr T max( std::initializer_list<T> ilist ); (since C++14)
      const double tol_reduction = 1e-6 * std::max(std::max(1.0, tolerance), std::max(std::abs(p.pose.position.x), std::abs(p.pose.position.y)));
      const double reduced_tolerance = tolerance - tol_reduction;

      // sample denser than cell to not unnecessarily use up tolerance, and ensure at least 5 sampled points
      const double step_size = std::min(0.5 * resolution, 0.999*reduced_tolerance);

      unsigned int mx, my;
      // A non-positive step would never advance the sampling.
      for(double dy = -reduced_tolerance; step_size > 0 && dy <= reduced_tolerance; dy += step_size){
        p.pose.position.y = goal.pose.position.y + dy;
        const double dx = std::sqrt(reduced_tolerance*reduced_tolerance - dy*dy);
        for(p.pose.position.x = goal.pose.position.x - dx; p.pose.position.x <= goal.pose.position.x + dx; p.pose.position.x += step_size){
            if(costmap_->worldToMap(p.pose.position.x, p.pose.position.y, mx, my)) {
              unsigned int index = my * nx + mx;
              double potential = potential_array_[index];
              double sdist = sq_distance(p, goal);
              RCLCPP_FATAL_STREAM_EXPRESSION(logger_, sdist > tolerance, "sampled pose is " << sdist << " away from the goal, which is above tolerance " << tolerance);
              assert(sdist <= tolerance);
              if(potential < POT_HIGH && sdist < best_sdist && sdist < tolerance){
                best_sdist = sdist;
                best_pose = p;
                found_legal = true;
              }
            }
        }
      }

      if (found_legal) {
        if(old_navfn_behavior_){
          goal_x = goal_x_i;
          goal_y = goal_y_i;
        }else{
          worldToMap(best_pose.pose.position.x, best_pose.pose.position.y, goal_x, goal_y);
        }
      }
    }

    if(!old_navfn_behavior_)
        planner_->clearEndpoint(costmap_->getCharMap(), potential_array_.get(), goal_x_i, goal_y_i, 2);
    if(publish_potential_)
        publishPotential(potential_array_.get());

    if (found_legal) {
        //extract the plan
        if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, best_pose, plan)) {
            //make sure the goal we push on has the same timestamp as the rest of the plan
            best_pose.header.stamp = clock_->now();
            plan.push_back(best_pose);
        } else {
            RCLCPP_ERROR(logger_, "Failed to get a plan from potential when a legal potential was found. This shouldn't happen");
        }
    }else{
        RCLCPP_ERROR(logger_, "Failed to get a plan");
    }

    potential_array_.reset();
    if (!found_legal)
        return PlanStatus::NoPotential;
    return plan.empty() ? PlanStatus::ExtractionFailed : PlanStatus::Success;
}

bool PlannerCore::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                      const geometry_msgs::msg::PoseStamped& goal,
                                       std::vector<geometry_msgs::msg::PoseStamped>& plan) {
    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    if (!path_maker_->getPath(potential_array_.get(), start_x, start_y, goal_x, goal_y, path)) {
        RCLCPP_ERROR(logger_, "NO PATH!");
        return false;
    }

    rclcpp::Time plan_time = clock_->now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
    if(old_navfn_behavior_){
            plan.push_back(goal);
    }
    return !plan.empty();
}

void PlannerCore::publishPotential(const float* /*potential*/)
{
}

nav_msgs::msg::OccupancyGrid PlannerCore::potentialGrid(const float* potential_array) const
{
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::msg::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = clock_->now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        float potential = potential_array[i];
        if (potential < POT_HIGH) {
            if (potential > max) {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++) {
        if (potential_array[i] >= POT_HIGH) {
            grid.data[i] = -1;
        } else {
            if (fabs(max) < DBL_EPSILON) {
                grid.data[i] = -1;
            } else {
                grid.data[i] = potential_array[i] * publish_scale_ / max;
            }
        }
    }
    return grid;
}

} //end namespace global_planner
