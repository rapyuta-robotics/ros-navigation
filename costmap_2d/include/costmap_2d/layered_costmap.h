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
#ifndef COSTMAP_2D_LAYERED_COSTMAP_H_
#define COSTMAP_2D_LAYERED_COSTMAP_H_

#include <costmap_2d/cost_values.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d.h>
#include <vector>
#include <string>

namespace costmap_2d
{
class Layer;

/**
 * @class LayeredCostmap
 * @brief Instantiates different layer plugins and aggregates them into one score
 */
class LayeredCostmap
{
public:
  /**
   * @brief  Constructor for a timed costmap
   */
  LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown, double prediction_time = 0.0, double timestep = 0.0);

  /**
   * @brief  Destructor
   */
  ~LayeredCostmap();

  /**
   * @brief  Update the underlying costmap with new data.
   * If you want to update the map outside of the update loop that runs, you can call this.
   */
  void updateMap(double robot_x, double robot_y, double robot_yaw);

  inline const std::string& getGlobalFrameID() const noexcept
  {
    return global_frame_;
  }

  void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y,
                 bool size_locked = false);

  void getUpdatedBounds(double& minx, double& miny, double& maxx, double& maxy, double t = 0)
  {
    int n;
    if (!timestep_)
      n = 0;
    else
      n = std::min((int)timed_bounds_.size()-1, int(t/timestep_));
    minx = timed_bounds_[n].minx;
    miny = timed_bounds_[n].miny;
    maxx = timed_bounds_[n].maxx;
    maxy = timed_bounds_[n].maxy;
  }

  bool isCurrent();

  costmap_2d::Costmap2D* getCostmap(double t = 0.0);

  double getTimestep() const;

  bool isRolling()
  {
    return rolling_window_;
  }

  bool isTrackingUnknown()
  {
    return getCostmap()->getDefaultValue() == costmap_2d::NO_INFORMATION;
  }

  std::vector<boost::shared_ptr<Layer> >* getPlugins()
  {
    return &plugins_;
  }

  void addPlugin(boost::shared_ptr<Layer> plugin)
  {
    plugins_.push_back(plugin);
  }

  bool isSizeLocked()
  {
    return size_locked_;
  }

  void getBounds(unsigned int* x0, unsigned int* xn, unsigned int* y0, unsigned int* yn, double t = 0.0)
  {
    int n;
    if (!timestep_)
      n = 0;
    else
      n = std::min((int)timed_bounds_.size()-1, int(t/timestep_));
    *x0 = timed_bounds_[n].bx0;
    *xn = timed_bounds_[n].bxn;
    *y0 = timed_bounds_[n].by0;
    *yn = timed_bounds_[n].byn;
  }

  bool isInitialized()
  {
      return initialized_;
  }

  /** @brief Updates the stored footprint, updates the circumscribed
   * and inscribed radii, and calls onFootprintChanged() in all
   * layers. */
  void setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec);

  /** @brief Returns the latest footprint stored with setFootprint(). */
  const std::vector<geometry_msgs::Point>& getFootprint() { return footprint_; }

  /** @brief The radius of a circle centered at the origin of the
   * robot which just surrounds all points on the robot's
   * footprint.
   *
   * This is updated by setFootprint(). */
  double getCircumscribedRadius() { return circumscribed_radius_; }

  /** @brief The radius of a circle centered at the origin of the
   * robot which is just within all points and edges of the robot's
   * footprint.
   *
   * This is updated by setFootprint(). */
  double getInscribedRadius() { return inscribed_radius_; }

protected:
  /**
   * ! Function copied from Costmap2D !
   * To-Do: Modify function for this use-case
   * @brief  Copy a region of a source map into a destination map
   * @param  source_map The source map
   * @param sm_lower_left_x The lower left x point of the source map to start the copy
   * @param sm_lower_left_y The lower left y point of the source map to start the copy
   * @param sm_size_x The x size of the source map
   * @param  dest_map The destination map
   * @param dm_lower_left_x The lower left x point of the destination map to start the copy
   * @param dm_lower_left_y The lower left y point of the destination map to start the copy
   * @param dm_size_x The x size of the destination map
   * @param region_size_x The x size of the region to copy
   * @param region_size_y The y size of the region to copy
   */
  template<typename data_type>
    void copyMapRegion(data_type* source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y,
                       unsigned int sm_size_x, data_type* dest_map, unsigned int dm_lower_left_x,
                       unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x,
                       unsigned int region_size_y)
    {
      // we'll first need to compute the starting points for each map
      data_type* sm_index = source_map + (sm_lower_left_y * sm_size_x + sm_lower_left_x);
      data_type* dm_index = dest_map + (dm_lower_left_y * dm_size_x + dm_lower_left_x);

      // now, we'll copy the source map into the destination map
      for (unsigned int i = 0; i < region_size_y; ++i)
      {
        memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));
        sm_index += sm_size_x;
        dm_index += dm_size_x;
      }
    }

private:

  // Struct to store the bounds for each timed costmaps
  struct Costmap2DBounds
  {
    double minx, miny, maxx, maxy;
    int bx0, bxn, by0, byn;
  };
  
  std::vector<Costmap2D> timed_costmaps_;
  std::vector<Costmap2DBounds> timed_bounds_;

  Costmap2D static_costmap_;
  std::string global_frame_;

  bool rolling_window_;  /// < @brief Whether or not the costmap should roll with the robot

  bool current_;

  double static_minx_, static_miny_, static_maxx_, static_maxy_;  // Bounds for static costmap
  unsigned int static_bx0_, static_bxn_, static_by0_, static_byn_;
  double timestep_, prediction_time_; 

  std::vector<boost::shared_ptr<Layer> > plugins_;

  bool initialized_;
  bool size_locked_;
  double circumscribed_radius_, inscribed_radius_;
  std::vector<geometry_msgs::Point> footprint_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_LAYERED_COSTMAP_H_
