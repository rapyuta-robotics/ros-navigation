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
    timestep_(timestep),
    prediction_time_(prediction_time),
    global_frame_(global_frame),
    rolling_window_(rolling_window),
    current_(false),
    minx_(0.0),
    miny_(0.0),
    maxx_(0.0),
    maxy_(0.0),
    bx0_(0),
    bxn_(0),
    by0_(0),
    byn_(0),
    initialized_(false),
    size_locked_(false),
    circumscribed_radius_(1.0),
    inscribed_radius_(0.1)
{
  if (!timestep_ || !prediction_time_)
    timed_costmaps_.resize(1);
  else
    timed_costmaps_.resize(ceil(prediction_time_/timestep_));
  
  for(auto& costmap : timed_costmaps_)
  {
    if (track_unknown)
      costmap.setDefaultValue(NO_INFORMATION);
    else
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
  for(costmap_2d::Costmap2D& costmap : timed_costmaps_)
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
  boost::unique_lock<Costmap2D::mutex_t> lock(*(timed_costmaps_.front().getMutex())); 

  // if we're using a rolling buffer costmap_... we need to update the origin using the robot's position
  if (rolling_window_)
  {
    double new_origin_x = robot_x - timed_costmaps_.front().getSizeInMetersX() / 2;
    double new_origin_y = robot_y - timed_costmaps_.front().getSizeInMetersY() / 2;
    for(costmap_2d::Costmap2D& costmap : timed_costmaps_)
      costmap.updateOrigin(new_origin_x, new_origin_y);
  }

  if (plugins_.size() == 0)
    return;

  minx_ = miny_ = 1e30;
  maxx_ = maxy_ = -1e30;

  // To-Do: Maybe store all static layers in a costmap object then 
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    // {1} calculate bounds for first costmap but ALL layers, default obstacle layer updateBounds with t = 0)
    // or only calculate bounds if (!plugin->isTimed()) and add obstacle bounds in time loop
    if(!(*plugin)->isEnabled() || (*plugin)->isTimed())
      continue;

    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy)
    {
      ROS_WARN_THROTTLE(1.0, "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
                        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
                        prev_minx, prev_miny, prev_maxx , prev_maxy,
                        minx_, miny_, maxx_ , maxy_,
                        (*plugin)->getName().c_str());
    }
  }

  // To-Do: if timed layers use costmap converter to paint into costmap and costmap converter only sees front()
  // that means the bounds don't need to be updated for the timed layers? we can just take the same bounds as front()?
  int x0, xn, y0, yn;
  timed_costmaps_.front().worldToMapEnforceBounds(minx_, miny_, x0, y0);  
  timed_costmaps_.front().worldToMapEnforceBounds(maxx_, maxy_, xn, yn);

  x0 = std::max(0, x0);
  xn = std::min(int(timed_costmaps_.front().getSizeInCellsX()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(int(timed_costmaps_.front().getSizeInCellsY()), yn + 1);

  ROS_DEBUG("Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);

  if (xn < x0 || yn < y0)
  {
    return; // To-Do: Do we need to change this?
  }
  
  timed_costmaps_.front().resetMap(x0, y0, xn, yn); // {1} Reset all Maps here? 
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
      ++plugin)
  {
    if((*plugin)->isEnabled() && !(*plugin)->isTimed())
      (*plugin)->updateCosts(timed_costmaps_.front(), x0, y0, xn, yn);
  }

  bx0_ = x0;
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;

  // Idea for costmap converter implementation:
  // Each update step move timed_costmaps one down, so timed_costmap(t=2) becomes timed_costmap(t=1)...
  // Then update costs keeping noise from previous predictions. 
  // Maybe implement a function like resetMap() but instead of setting to 0 subtract from previous cost?
  // Or make some function cost depends on prev_cost, new_cost & t ????
  // Influence of previous prediction should be higher the further in the future the timed_costmap is.
  // ok but this is probably unneccessary...

  double t = 0;

  for(costmap_2d::Costmap2D& costmap : timed_costmaps_)
  {
    if(t == 0)
    {
      t += timestep_;
      continue; // Put above code here?
    }
    boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap.getMutex())); 
    minx_ = miny_ = 1e30;
    maxx_ = maxy_ = -1e30;

    for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
        ++plugin)
    {
      if((*plugin)->isTimed()) 
        continue;

      double prev_minx = minx_;
      double prev_miny = miny_;
      double prev_maxx = maxx_;
      double prev_maxy = maxy_;
      (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_, t);
      if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy)
      {
        ROS_WARN_THROTTLE(1.0, "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
                          "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
                          prev_minx, prev_miny, prev_maxx , prev_maxy,
                          minx_, miny_, maxx_ , maxy_,
                          (*plugin)->getName().c_str());
      }
    }

    int x0, xn, y0, yn;
    costmap.worldToMapEnforceBounds(minx_, miny_, x0, y0);   
    costmap.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);

    x0 = std::max(0, x0);
    xn = std::min(int(costmap.getSizeInCellsX()), xn + 1);
    y0 = std::max(0, y0);
    yn = std::min(int(costmap.getSizeInCellsY()), yn + 1);

    ROS_DEBUG("Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);

    if (xn < x0 || yn < y0)
      continue;

    costmap.resetMap(x0, y0, xn, yn);
    for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins_.begin(); plugin != plugins_.end();
        ++plugin)
    {
      if((*plugin)->isTimed())
        (*plugin)->updateCosts(costmap, x0, y0, xn, yn, t); // Time is not used here atm...
    }

    bx0_ = x0;
    bxn_ = xn;
    by0_ = y0;
    byn_ = yn;

    t += timestep_;
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
  if(!timestep_)
    return &timed_costmaps_.front();
  int n = std::min((int)timed_costmaps_.size()-1, int(t/timestep_));
  return &timed_costmaps_[n];
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
