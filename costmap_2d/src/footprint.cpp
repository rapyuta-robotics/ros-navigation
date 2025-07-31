/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <unordered_set>

#include <costmap_2d/costmap_math.h>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <costmap_2d/footprint.h>
#include <costmap_2d/array_parser.h>
#include <geometry_msgs/Point32.h>

namespace costmap_2d
{

void calculateMinAndMaxDistances(const std::vector<geometry_msgs::Point>& footprint, double& min_dist, double& max_dist)
{
  min_dist = std::numeric_limits<double>::max();
  max_dist = 0.0;

  if (footprint.size() <= 2)
  {
    return;
  }

  for (unsigned int i = 0; i < footprint.size() - 1; ++i)
  {
    // check the distance from the robot center point to the first vertex
    double vertex_dist = distance(0.0, 0.0, footprint[i].x, footprint[i].y);
    double edge_dist = distanceToLine(0.0, 0.0, footprint[i].x, footprint[i].y,
                                      footprint[i + 1].x, footprint[i + 1].y);
    min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
  }

  // we also need to do the last vertex and the first vertex
  double vertex_dist = distance(0.0, 0.0, footprint.back().x, footprint.back().y);
  double edge_dist = distanceToLine(0.0, 0.0, footprint.back().x, footprint.back().y,
                                      footprint.front().x, footprint.front().y);
  min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
  max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
}

BoundingRect minBoundingRect(const std::vector<geometry_msgs::Point>& points)
{
  Eigen::MatrixX2d hull_points_2d(points.size(), 2);  // empty 2 column array
  for (size_t i = 0; i < points.size(); ++i)
    hull_points_2d.row(i) << points[i].x, points[i].y;
  ROS_DEBUG_STREAM("Input convex hull points:\n" << hull_points_2d);

  // Compute edges (x2-x1,y2-y1)
  Eigen::MatrixX2d edges(hull_points_2d.rows() - 1, 2);  // empty 2 column array
  edges.setZero();
  for (size_t i = 0; i < edges.rows(); ++i)
  {
    double edge_x = hull_points_2d(i + 1, 0) - hull_points_2d(i, 0);
    double edge_y = hull_points_2d(i + 1, 1) - hull_points_2d(i, 1);
    edges.row(i) << edge_x, edge_y;
  }
  ROS_DEBUG_STREAM("Edges:\n" << edges);

  // Calculate edge angles with atan2(y/x)
  std::vector<double> edge_angles(edges.rows());  // empty 1 column array
  for (size_t i = 0; i < edge_angles.size(); ++i)
    edge_angles[i] = std::atan2(edges.row(i)[1], edges.row(i)[0]);
  ROS_DEBUG_STREAM("Edge angles:\n" << toString(edge_angles));

  // Check for angles in 1st quadrant
  for (size_t i = 0; i < edge_angles.size(); ++i)
    edge_angles[i] = std::fmod(edge_angles[i] + M_PI, M_PI_2);  // want strictly positive answers
  ROS_DEBUG_STREAM("Edge angles in 1st Quadrant:\n" << toString(edge_angles));

  // Remove duplicate angles
  std::unordered_set<float> s;
  auto end = std::remove_if(edge_angles.begin(), edge_angles.end(), [&s](double v) { return !s.insert(v).second; });
  edge_angles.erase(end, edge_angles.end());
  ROS_DEBUG_STREAM("Unique edge angles:\n" << toString(edge_angles));

  // Test each angle to find bounding box with the smallest area
  // rot_angle, area, width, height, min_x, max_x, min_y, max_y
  std::array<double, 8> min_bbox{ 0.0, DBL_MAX, 0.0, 0.0, 0.0, 0.0, 0.0, 0 };
  ROS_DEBUG_STREAM("Testing " << edge_angles.size() << " possible rotations for bounding box...");
  for (size_t i = 0; i < edge_angles.size(); ++i)
  {
    // Create rotation matrix to shift points to baseline
    // R = [ cos(theta)      , cos(theta-PI/2)
    //       cos(theta+PI/2) , cos(theta)     ]
    // clang-format off
    Eigen::Matrix<double, 2, 2> R;
    R << std::cos(edge_angles[i]), std::cos(edge_angles[i] - M_PI_2),
         std::cos(edge_angles[i] + M_PI_2), std::cos(edge_angles[i]);
    // clang-format on
    ROS_DEBUG_STREAM("Rotation matrix for " << edge_angles[i] << " is\n" << R);

    // Apply this rotation to convex hull points
    Eigen::MatrixX2d rot_points = (R * hull_points_2d.transpose()).transpose();  // 2x2 * 2xn
    ROS_DEBUG_STREAM("Rotated hull points are\n" << rot_points);

    // Find min/max x,y points
    const double min_x = rot_points.col(0).minCoeff();
    const double max_x = rot_points.col(0).maxCoeff();
    const double min_y = rot_points.col(1).minCoeff();
    const double max_y = rot_points.col(1).maxCoeff();
    ROS_DEBUG_STREAM("Min x: " << min_x << " Max x: " << max_x << "   Min y: " << min_y << " Max y: " << max_y);

    // Calculate height/width/area of this bounding rectangle
    const double width = max_x - min_x;
    const double height = max_y - min_y;
    const double area = width * height;
    ROS_DEBUG_STREAM("Bounding box " << i << ":  width: " << width << " height: " << height << "  area: " << area);

    // Store the smallest rect found first (a simple convex hull might have 2 answers with same area)
    // Note that we require a non-neglectable difference to favor smaller rotations
    if (min_bbox[1] - area > 1e-3)
    {
      ROS_DEBUG_STREAM("Area " << min_bbox[1] << " -> " << area);
      min_bbox = { edge_angles[i], area, width, height, min_x, max_x, min_y, max_y };
    }
  }
  // Re-create rotation matrix for smallest rect
  // clang-format off
  const double angle = min_bbox[0];
  Eigen::Matrix<double, 2, 2> R;
  R << std::cos(angle), std::cos(angle - M_PI_2),
       std::cos(angle + M_PI_2), std::cos(angle);
  // clang-format on
  ROS_DEBUG_STREAM("Projection matrix:\n" << R);

  // Project convex hull points onto rotated frame
  Eigen::MatrixX2d proj_points = (R * hull_points_2d.transpose()).transpose();  // 2x2 * 2xn
  ROS_DEBUG_STREAM("Project hull points are\n" << proj_points);

  // min/max x,y points are against baseline
  const double min_x = min_bbox[4];
  const double max_x = min_bbox[5];
  const double min_y = min_bbox[6];
  const double max_y = min_bbox[7];
  ROS_DEBUG_STREAM("Min x: " << min_x << " Max x: " << max_x << "   Min y: " << min_y << " Max y: " << max_y);

  // Calculate center point and project onto rotated frame
  Eigen::Vector2d center{ (min_x + max_x) / 2.0, (min_y + max_y) / 2.0 };
  Eigen::Vector2d center_point = center.transpose() * R;
  ROS_DEBUG_STREAM("Bounding box center point:\n" << center_point);

  // Calculate corner points and project onto rotated frame
  Eigen::Matrix<double, 4, 2> corner_points;  //// = zeros((4, 2))  // empty 2 column array
  corner_points.row(0) = (Eigen::Vector2d{ max_x, min_y }.transpose() * R).transpose();
  corner_points.row(1) = (Eigen::Vector2d{ min_x, min_y }.transpose() * R).transpose();
  corner_points.row(2) = (Eigen::Vector2d{ min_x, max_y }.transpose() * R).transpose();
  corner_points.row(3) = (Eigen::Vector2d{ max_x, max_y }.transpose() * R).transpose();
  ROS_DEBUG_STREAM("Bounding box corner points:\n" << corner_points);

  ROS_DEBUG_STREAM("Angle of rotation: " << angle << " rad  " << angle * (180 / M_PI) << " deg");

  BoundingRect result;
  result.rot_angle = angle;
  result.area = min_bbox[1];
  result.width = min_bbox[2];
  result.height = min_bbox[3];
  result.center.x = center_point.x();
  result.center.y = center_point.y();
  for (int i = 0; i < corner_points.rows(); ++i)
    result.corners[i] = toPoint(corner_points.row(i));

  return result;
}

geometry_msgs::Point32 toPoint32(geometry_msgs::Point pt)
{
  geometry_msgs::Point32 point32;
  point32.x = pt.x;
  point32.y = pt.y;
  point32.z = pt.z;
  return point32;
}

geometry_msgs::Point toPoint(geometry_msgs::Point32 pt)
{
  geometry_msgs::Point point;
  point.x = pt.x;
  point.y = pt.y;
  point.z = pt.z;
  return point;
}

geometry_msgs::Point toPoint(Eigen::Vector2d pt)
{
  geometry_msgs::Point point;
  point.x = pt.x();
  point.y = pt.y();
  return point;
}

geometry_msgs::Polygon toPolygon(std::vector<geometry_msgs::Point> pts)
{
  geometry_msgs::Polygon polygon;
  for (int i = 0; i < pts.size(); i++){
    polygon.points.push_back(toPoint32(pts[i]));
  }
  return polygon;
}

std::vector<geometry_msgs::Point> toPointVector(geometry_msgs::Polygon polygon)
{
  std::vector<geometry_msgs::Point> pts;
  for (int i = 0; i < polygon.points.size(); i++)
  {
    pts.push_back(toPoint(polygon.points[i]));
  }
  return pts;
}

std::string toString(const std::vector<double>& numbers)
{
  std::stringstream ss;
  std::for_each(numbers.begin(), numbers.end(), [&](double nb) { ss << nb << " "; });
  return ss.str();
}

void transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec,
                        std::vector<geometry_msgs::Point>& oriented_footprint)
{
  // build the oriented footprint at a given location
  oriented_footprint.clear();
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i)
  {
    geometry_msgs::Point new_pt;
    new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    oriented_footprint.push_back(new_pt);
  }
}

void transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec,
                        geometry_msgs::PolygonStamped& oriented_footprint)
{
  // build the oriented footprint at a given location
  oriented_footprint.polygon.points.clear();
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i)
  {
    geometry_msgs::Point32 new_pt;
    new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    oriented_footprint.polygon.points.push_back(new_pt);
  }
}

void padFootprint(std::vector<geometry_msgs::Point>& footprint, double padding)
{
  // pad footprint in place
  for (unsigned int i = 0; i < footprint.size(); i++)
  {
    geometry_msgs::Point& pt = footprint[ i ];
    pt.x += sign0(pt.x) * padding;
    pt.y += sign0(pt.y) * padding;
  }
}


std::vector<geometry_msgs::Point> makeFootprintFromRadius(double radius)
{
  std::vector<geometry_msgs::Point> points;

  // Loop over 16 angles around a circle making a point each time
  int N = 16;
  geometry_msgs::Point pt;
  for (int i = 0; i < N; ++i)
  {
    double angle = i * 2 * M_PI / N;
    pt.x = cos(angle) * radius;
    pt.y = sin(angle) * radius;

    points.push_back(pt);
  }

  return points;
}


bool makeFootprintFromString(const std::string& footprint_string, std::vector<geometry_msgs::Point>& footprint)
{
  std::string error;
  std::vector<std::vector<float> > vvf = parseVVF(footprint_string, error);

  if (error != "")
  {
    ROS_ERROR("Error parsing footprint parameter: '%s'", error.c_str());
    ROS_ERROR("  Footprint string was '%s'.", footprint_string.c_str());
    return false;
  }

  // convert vvf into points.
  if (vvf.size() < 3)
  {
    ROS_ERROR("You must specify at least three points for the robot footprint, reverting to previous footprint.");
    return false;
  }
  footprint.reserve(vvf.size());
  for (unsigned int i = 0; i < vvf.size(); i++)
  {
    if (vvf[ i ].size() == 2)
    {
      geometry_msgs::Point point;
      point.x = vvf[ i ][ 0 ];
      point.y = vvf[ i ][ 1 ];
      point.z = 0;
      footprint.push_back(point);
    }
    else
    {
      ROS_ERROR("Points in the footprint specification must be pairs of numbers.  Found a point with %d numbers.",
                 int(vvf[ i ].size()));
      return false;
    }
  }

  return true;
}



std::vector<geometry_msgs::Point> makeFootprintFromParams(ros::NodeHandle& nh, bool* use_radius)
{
  std::string full_param_name;
  std::string full_radius_param_name;
  std::vector<geometry_msgs::Point> points;

  if (nh.searchParam("footprint", full_param_name))
  {
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    nh.getParam(full_param_name, footprint_xmlrpc);
    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString &&
        footprint_xmlrpc != "" && footprint_xmlrpc != "[]")
    {
      if (makeFootprintFromString(std::string(footprint_xmlrpc), points))
      {
        writeFootprintToParam(nh, points);
        if (use_radius)
        {
          *use_radius = false;
        }
        return points;
      }
    }
    else if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      points = makeFootprintFromXMLRPC(footprint_xmlrpc, full_param_name);
      writeFootprintToParam(nh, points);
      if (use_radius)
      {
        *use_radius = false;
      }
      return points;
    }
  }

  if (nh.searchParam("robot_radius", full_radius_param_name))
  {
    double robot_radius;
    nh.param(full_radius_param_name, robot_radius, 1.234);
    points = makeFootprintFromRadius(robot_radius);
    nh.setParam("robot_radius", robot_radius);
    if (use_radius)
    {
      *use_radius = true;
    }
  }
  // Else neither param was found anywhere this knows about, so
  // defaults will come from dynamic_reconfigure stuff, set in
  // cfg/Costmap2D.cfg and read in this file in reconfigureCB().
  return points;
}

void writeFootprintToParam(ros::NodeHandle& nh, const std::vector<geometry_msgs::Point>& footprint)
{
  std::ostringstream oss;
  bool first = true;
  for (unsigned int i = 0; i < footprint.size(); i++)
  {
    geometry_msgs::Point p = footprint[ i ];
    if (first)
    {
      oss << "[[" << p.x << "," << p.y << "]";
      first = false;
    }
    else
    {
      oss << ",[" << p.x << "," << p.y << "]";
    }
  }
  oss << "]";
  nh.setParam("footprint", oss.str().c_str());
}

double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
{
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
      value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    std::string& value_string = value;
    ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
               full_param_name.c_str(), value_string.c_str());
    throw std::runtime_error("Values in the footprint specification must be numbers");
  }
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

std::vector<geometry_msgs::Point> makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc,
                                const std::string& full_param_name)
{
  // Make sure we have an array of at least 3 elements.
  if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      footprint_xmlrpc.size() < 3)
  {
    ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
               full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
    throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                             "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
  }

  std::vector<geometry_msgs::Point> footprint;
  geometry_msgs::Point pt;

  for (int i = 0; i < footprint_xmlrpc.size(); ++i)
  {
    // Make sure each element of the list is an array of size 2. (x and y coordinates)
    XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];
    if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        point.size() != 2)
    {
      ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                 full_param_name.c_str());
      throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                               "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }

    pt.x = getNumberFromXMLRPC(point[ 0 ], full_param_name);
    pt.y = getNumberFromXMLRPC(point[ 1 ], full_param_name);

    footprint.push_back(pt);
  }
  return footprint;
}

}  // end namespace costmap_2d
