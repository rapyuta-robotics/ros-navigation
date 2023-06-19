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
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/footprint.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>

using std::vector;

namespace costmap_2d
{

LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown, double prediction_time, double timestep) :
    static_costmap_(),
    timestep_(timestep),
    prediction_time_(prediction_time),
    global_frame_(global_frame),
    rolling_window_(rolling_window),
    current_(false),
    static_minx_(0.0),
    static_miny_(0.0),
    static_maxx_(0.0),
    static_maxy_(0.0),
    static_bx0_(0),
    static_bxn_(0),
    static_by0_(0),
    static_byn_(0),
    initialized_(false),
    size_locked_(false),
    circumscribed_radius_(1.0),
    inscribed_radius_(0.1)
{
  if (!timestep_ || !prediction_time_)
    timed_costmaps_.resize(1);
  else
    timed_costmaps_.resize(ceil(prediction_time_/timestep_));
  
  timed_bounds_.resize(timed_costmaps_.size());
  for (auto bounds : timed_bounds_)
  {
    bounds.minx = bounds.maxx = bounds.miny = bounds.maxy =  0.0;
    bounds.bx0 = bounds.bxn = bounds.by0 = bounds.byn = 0;
  }

  if (track_unknown)
  {
    static_costmap_.setDefaultValue(NO_INFORMATION);
    for(auto& costmap : timed_costmaps_)
      costmap.setDefaultValue(NO_INFORMATION);
  }
  else
  {
    static_costmap_.setDefaultValue(FREE_SPACE);
    for(auto& costmap : timed_costmaps_)
      costmap.setDefaultValue(FREE_SPACE);
  }
}

LayeredCostmap::~LayeredCostmap()
{
  while (plugins_.size() > 0)
  {
    plugins_.pop_back();
  }
}

void LayeredCostmap::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                               double origin_y, bool size_locked)
{  
  size_locked_ = size_locked;
  
  boost::unique_lock<Costmap2D::mutex_t> lock(*(static_costmap_.getMutex()));
  static_costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  for(Costmap2D& costmap : timed_costmaps_)
  {
    boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap.getMutex()));
    costmap.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  }
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
        ++plugin)
  {
    (*plugin)->matchSize();
  }
}

void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  boost::unique_lock<Costmap2D::mutex_t> lock(*(static_costmap_.getMutex())); // Uneccessary to lock?

  // if we're using a rolling buffer costmap_... we need to update the origin using the robot's position
  if (rolling_window_)
  {
    double new_origin_x = robot_x - timed_costmaps_.front().getSizeInMetersX() / 2;
    double new_origin_y = robot_y - timed_costmaps_.front().getSizeInMetersY() / 2;
    static_costmap_.updateOrigin(new_origin_x, new_origin_y);
    for(costmap_2d::Costmap2D& costmap : timed_costmaps_)
      costmap.updateOrigin(new_origin_x, new_origin_y);
  }
  
  if (plugins_.size() == 0)
    return;

  vector<boost::shared_ptr<Layer> >::iterator current_plugin;

  static_minx_ = static_miny_ = 1e30;
  static_maxx_ = static_maxy_ = -1e30;

  // In this first loop we create a 'static_costmap' that we later copy into the timed costmaps, so
  // we don't have to recompute costs that don't change over time 
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    current_plugin = plugin; // Save the last plugin we looked at to know where to start the timed loop

    if(!(*plugin)->isEnabled()) 
    {
      continue;
    }
    // We can't just skip timed plugins, since later plugins costs depend on previous ones, so a layers 
    // cost could change over timed even if it is itself not timed (e.g. inflation layer) -> break.
    else if((*plugin)->isTimed())
    {
      break;
    }

    double prev_minx = static_minx_;
    double prev_miny = static_miny_;
    double prev_maxx = static_maxx_;
    double prev_maxy = static_maxy_;
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &static_minx_, &static_miny_, &static_maxx_, &static_maxy_);
    if (static_minx_ > prev_minx || static_miny_ > prev_miny || static_maxx_ < prev_maxx || static_maxy_ < prev_maxy)
    {
      ROS_WARN_THROTTLE(1.0, "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
                        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
                        prev_minx, prev_miny, prev_maxx , prev_maxy,
                        static_minx_, static_miny_, static_maxx_ , static_maxy_,
                        (*plugin)->getName().c_str());
    }
    // ROS_INFO_STREAM((*plugin)->getName());
    // ROS_ERROR("Updating area x: [%f, %f] y: [%f, %f] ", static_minx_, static_maxx_, static_miny_, static_maxy_);

  }

  int static_x0, static_xn, static_y0, static_yn;
  static_costmap_.worldToMapEnforceBounds(static_minx_, static_miny_, static_x0, static_y0);  
  static_costmap_.worldToMapEnforceBounds(static_maxx_, static_maxy_, static_xn, static_yn);

  static_x0 = std::max(0, static_x0);
  static_xn = std::min(int(static_costmap_.getSizeInCellsX()), static_xn + 1);
  static_y0 = std::max(0, static_y0);
  static_yn = std::min(int(static_costmap_.getSizeInCellsY()), static_yn + 1);

  // ROS_DEBUG("Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);
  // ROS_ERROR("Updating area x: [%f, %f] y: [%f, %f] ", static_minx_, static_maxx_, static_miny_, static_maxy_);

  if (static_xn > static_x0 && static_yn > static_y0)
  {
    static_costmap_.resetMap(static_x0, static_y0, static_xn, static_yn); // Reset all Maps here? 
    for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
        ++plugin)
    {
      if(!(*plugin)->isEnabled())
        continue;
      else if((*plugin)->isTimed())
        break;
      else 
        (*plugin)->updateCosts(static_costmap_, static_x0, static_y0, static_xn, static_yn);
    }
  }

  // In this second loop we create the actual 'timed_costmaps'.
  // We loop through the timesteps, and compute bounds and costs for each timed costmap 
  // based on the time index i given by the time i = t / timestep.
  for (size_t i = 0; i < timed_costmaps_.size(); ++i)
  {
    Costmap2D& costmap = timed_costmaps_[i];
    Costmap2DBounds& bounds = timed_bounds_[i];

    // We set the timed bounds to the static bounds to include changes in the static layers 
    bounds.minx = static_minx_;
    bounds.miny = static_miny_;
    bounds.maxx = static_maxx_;
    bounds.maxy = static_maxy_;
    double prev_minx = bounds.minx;
    double prev_miny = bounds.miny;
    double prev_maxx = bounds.maxx;
    double prev_maxy = bounds.maxy;

    boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap.getMutex())); 
    
    for (vector<boost::shared_ptr<Layer> >::iterator plugin = current_plugin; plugin != plugins_.end();
        ++plugin)
    {
      // Currently we have some layers that are time dependent but don't have a timed logic yet. This means
      // that we don't want to paint these layers in the static_costmap, otherwise we paint it in all of our timed
      // snapshots. Instead, we paint it only in the first costmap where t = 0, and skip these plugins for t > 0.
      // E.g.: Obstacle Layer -> Since the obstacle layer paints all observation, it would include dynamic obstacles
      // at their current position. We don't want to paint these dynamic obstacle at their current position in future
      // costmaps, otherwise we would have to manually remove them again.
      // A proper implementation of this depends on the logic of the perception component.....
      if ((i > 0 && (*plugin)->isTimedFront()))
        continue;
        
      (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, 
                              &bounds.minx, &bounds.miny, 
                              &bounds.maxx, &bounds.maxy);
      if (bounds.minx > prev_minx || bounds.miny > prev_miny || 
          bounds.maxx < prev_maxx || bounds.maxy < prev_maxy)
      {
        ROS_WARN_THROTTLE(1.0, "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
                          "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
                          prev_minx, prev_miny, prev_maxx , prev_maxy,
                          bounds.minx, bounds.miny, 
                          bounds.maxx , bounds.maxy,
                          (*plugin)->getName().c_str());
      }

      // ROS_INFO_STREAM((*plugin)->getName());
      // ROS_INFO_STREAM("Updating area x: [%f, %f] y: [%f, %f], i: %d i: %f ", bounds.minx, bounds.maxx, bounds.miny, bounds.maxy, i, i*timestep_);

    }
    int x0, xn, y0, yn;
    costmap.worldToMapEnforceBounds(bounds.minx, bounds.miny, x0, y0);   
    costmap.worldToMapEnforceBounds(bounds.maxx, bounds.maxy, xn, yn);

    x0 = std::max(0, x0);
    xn = std::min(int(costmap.getSizeInCellsX()), xn + 1);
    y0 = std::max(0, y0);
    yn = std::min(int(costmap.getSizeInCellsY()), yn + 1);

    // ROS_ERROR("Updating area x: [%d, %d] y: [%d, %d], i: %d i: %f ", x0, xn, y0, yn, i, i*timestep_);

    if (xn < x0 || yn < y0)
      continue;
    
    // Instead of resetting the costmap within the bounds, we copy the region of the static costmap we precomputed.
    // The region includes all changes from both the static and timed layers.
    copyMapRegion(static_costmap_.getCharMap(), x0, y0, static_costmap_.getSizeInCellsX(), costmap.getCharMap(), x0, y0, costmap.getSizeInCellsX(), xn - x0, yn - y0);

    for (vector<boost::shared_ptr<Layer> >::iterator plugin = current_plugin; plugin != plugins_.end();
        ++plugin)
    {
      if ((i > 0 && (*plugin)->isTimedFront()))
        continue;
    
      // ROS_INFO_STREAM((*plugin)->getName() << " " << i);
      (*plugin)->updateCosts(costmap, x0, y0, xn, yn); // i is not used here atm...
    }

    bounds.bx0 = x0;
    bounds.bxn = xn;
    bounds.by0 = y0;
    bounds.byn = yn;
  }
  
  initialized_ = true;
}

bool LayeredCostmap::isCurrent()
{
  current_ = true;
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    if((*plugin)->isEnabled())
      current_ = current_ && (*plugin)->isCurrent();
  }
  return current_;
}

costmap_2d::Costmap2D* LayeredCostmap::getCostmap(double t)
{
  if (timed_costmaps_.empty())
    return nullptr;
  else if(!timestep_)
    return &timed_costmaps_.front();
  int i = std::min((int)timed_costmaps_.size()-1, int(t/timestep_));  
  return &timed_costmaps_[i];
}

// we're not using this rn...
double LayeredCostmap::getTimestep() const
{
  return timestep_;
}


void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec)
{
  footprint_ = footprint_spec;
  costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);

  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    (*plugin)->onFootprintChanged();
  }
}

}  // namespace costmap_2d
