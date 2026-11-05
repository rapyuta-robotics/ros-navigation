#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H
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
#define POT_HIGH 1.0e10        // unassigned cell potential
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <global_planner/potential_calculator.h>
#include <global_planner/expander.h>
#include <global_planner/traceback.h>

namespace global_planner {

class Expander;
class GridPath;

/** Settings read once when the planner is created. */
struct PlannerOptions {
    /** Reproduce the navfn coordinate convention (no half-cell offset, no precise start). */
    bool old_navfn_behavior = false;
    /** Use the quadratic potential approximation instead of the simple one. */
    bool use_quadratic = true;
    /** Use Dijkstra's algorithm instead of A*. */
    bool use_dijkstra = true;
    /** Follow the grid boundaries instead of the gradient when extracting the path. */
    bool use_grid_path = false;
    /** Allow planning through unknown cells. */
    bool allow_unknown = true;
    /** Scale of the published potential values. */
    int publish_scale = 100;
    /** Write lethal cost into the costmap border before planning. */
    bool outline_map = true;
};

/** Settings that can change between plans. */
struct PlannerConfig {
    /** Lethal cost. */
    int lethal_cost = 253;
    /** Neutral cost. */
    int neutral_cost = 50;
    /** Factor to multiply each cost from costmap by. */
    double cost_factor = 3.0;
    /** Publish the potential after every plan. */
    bool publish_potential = true;
};

/** Result of PlannerCore::makePlan. */
enum class PlanStatus {
    /** The plan reaches the goal or the displaced goal. */
    Success,
    /** The start is outside the costmap. */
    StartOutsideMap,
    /** The goal is outside the costmap. */
    GoalOutsideMap,
    /** Neither the goal cell nor any sample in the tolerance disc has a potential. */
    NoPotential,
    /** A legal potential was found but the path extraction failed; the plan is empty. */
    ExtractionFailed,
};

/**
 * @class PlannerCore
 * @brief Provides a ROS wrapper for the global_planner planner which runs a fast, interpolated navigation function on a costmap.
 */

class PlannerCore {
    public:
        /** Plans on @p costmap in @p frame_id, without a ROS node; @p clock stamps the plan and the potential. */
        PlannerCore(nav2_costmap_2d::Costmap2D* costmap, std::string frame_id, const PlannerOptions& options,
                    const PlannerConfig& config, rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger);

        /**
         * @brief  Default deconstructor for the PlannerCore object
         */
        virtual ~PlannerCore();

        /** Applies runtime settings to the next plan. */
        void reconfigure(const PlannerConfig& config);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param tolerance If the goal is obstructed, how many meters the planner can relax the constraint
         *        in x and y before failing
         * @param plan The plan... filled by the planner
         * @return Outcome of the search and of the path extraction
         */
        PlanStatus makePlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
                            double tolerance, std::vector<geometry_msgs::msg::PoseStamped>& plan);

        /**
         * @brief Compute a plan to a goal after the potential for a start point has already been computed (Note: You should call computePotential first)
         * @param start_x
         * @param start_y
         * @param end_x
         * @param end_y
         * @param goal The goal pose to create a plan to
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool getPlanFromPotential(double start_x, double start_y, double end_x, double end_y,
                                  const geometry_msgs::msg::PoseStamped& goal,
                                  std::vector<geometry_msgs::msg::PoseStamped>& plan);

    protected:
        /** Publishes the potential; the node-free planner publishes nothing. */
        virtual void publishPotential(const float* potential);

        /** Occupancy grid of the potential, scaled to publish_scale. */
        nav_msgs::msg::OccupancyGrid potentialGrid(const float* potential) const;

        /**
         * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
         */
        nav2_costmap_2d::Costmap2D* costmap_ = nullptr;
        std::string frame_id_;
        bool allow_unknown_;
        /** Clock stamping the plan and the potential. */
        rclcpp::Clock::SharedPtr clock_;
        /** Logger for the planner messages. */
        rclcpp::Logger logger_;

    private:
        inline double sq_distance(const geometry_msgs::msg::PoseStamped& p1, const geometry_msgs::msg::PoseStamped& p2){
          double dx = p1.pose.position.x - p2.pose.position.x;
          double dy = p1.pose.position.y - p2.pose.position.y;
          return std::hypot(dx, dy);
        }
        void mapToWorld(double mx, double my, double& wx, double& wy);
        bool worldToMap(double wx, double wy, double& mx, double& my);

        std::mutex mutex_;

        std::unique_ptr<PotentialCalculator> p_calc_;
        std::unique_ptr<Expander> planner_;
        std::unique_ptr<Traceback> path_maker_;

        bool publish_potential_;
        int publish_scale_;

        void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);

        std::unique_ptr<float[]> potential_array_;

        bool old_navfn_behavior_;
        float convert_offset_;

        bool outline_map_;

};

} //end namespace global_planner

#endif
