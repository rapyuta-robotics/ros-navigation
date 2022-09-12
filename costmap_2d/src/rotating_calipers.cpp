///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of OpenCV Foundation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the OpenCV Foundation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//

#include <cfloat>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
using Point = geometry_msgs::Point;

#include <costmap_2d/footprint.h>

namespace costmap_2d
{

struct MinAreaState
{
  int bottom;
  int left;
  float height;
  float width;
  float base_a;
  float base_b;
};

enum
{
  CALIPERS_MAXHEIGHT = 0,
  CALIPERS_MINAREARECT = 1,
  CALIPERS_MAXDIST = 2
};

/*F///////////////////////////////////////////////////////////////////////////////////////
 //    Name:    rotatingCalipers
 //    Purpose:
 //      Rotating calipers algorithm with some applications
 //
 //    Context:
 //    Parameters:
 //      points      - convex hull vertices ( any orientation )
 //      n           - number of vertices
 //      mode        - concrete application of algorithm
 //                    can be  CV_CALIPERS_MAXDIST   or
 //                            CV_CALIPERS_MINAREARECT
 //      left, bottom, right, top - indexes of extremal points
 //      out         - output info.
 //                    In case CV_CALIPERS_MAXDIST it points to float value -
 //                    maximal height of polygon.
 //                    In case CV_CALIPERS_MINAREARECT
 //                    ((CvPoint2D32f*)out)[0] - corner
 //                    ((CvPoint2D32f*)out)[1] - vector1
 //                    ((CvPoint2D32f*)out)[0] - corner2
 //
 //                      ^
 //                      |
 //              vector2 |
 //                      |
 //                      |____________\
 //                    corner         /
 //                               vector1
 //
 //    Returns:
 //    Notes:
 //F*/

/* we will use usual cartesian coordinates */
void rotatingCalipers(const std::vector<Point>& points)
{
  float min_area = FLT_MAX;
  float max_dist = 0;
  char buffer[32] = {};
  int i, k;
  /*  modern equivalents
  std::vector<float> abuf(points.size() * 3);
  std::vector<float>& inv_vect_length = abuf;
  std::vector<Point> vect(inv_vect_length + n);
  */
  float abuf[points.size() * 3];
  float* inv_vect_length = abuf;
  Point* vect = (Point*)(inv_vect_length + points.size());
  int left = 0, bottom = 0, right = 0, top = 0;
  int seq[4] = { -1, -1, -1, -1 };

  /* rotating calipers sides will always have coordinates
   (a,b) (-b,a) (-a,-b) (b, -a)
   */
  /* this is a first base vector (a,b) initialized by (1,0) */
  float orientation = 0;
  float base_a;
  float base_b = 0;

  float left_x, right_x, top_y, bottom_y;
  Point pt0 = points[0];

  left_x = right_x = pt0.x;
  top_y = bottom_y = pt0.y;

  for (i = 0; i < points.size(); i++)
  {
    double dx, dy;

    if (pt0.x < left_x)
      left_x = pt0.x, left = i;

    if (pt0.x > right_x)
      right_x = pt0.x, right = i;

    if (pt0.y > top_y)
      top_y = pt0.y, top = i;

    if (pt0.y < bottom_y)
      bottom_y = pt0.y, bottom = i;

    Point pt = points[(i + 1) & (i + 1 < points.size() ? -1 : 0)];

    dx = pt.x - pt0.x;
    dy = pt.y - pt0.y;

    vect[i].x = (float)dx;
    vect[i].y = (float)dy;
    inv_vect_length[i] = (float)(1. / std::sqrt(dx * dx + dy * dy));

    pt0 = pt;
  }

  // find convex hull orientation
  {
    double ax = vect[points.size() - 1].x;
    double ay = vect[points.size() - 1].y;

    for (i = 0; i < points.size(); i++)
    {
      double bx = vect[i].x;
      double by = vect[i].y;

      double convexity = ax * by - ay * bx;

      if (convexity != 0)
      {
        orientation = (convexity > 0) ? 1.f : (-1.f);
        break;
      }
      ax = bx;
      ay = by;
    }
    ROS_ASSERT(orientation != 0);
  }
  base_a = orientation;

  /*****************************************************************************************/
  /*                         init calipers position                                        */
  seq[0] = bottom;
  seq[1] = right;
  seq[2] = top;
  seq[3] = left;
  /*****************************************************************************************/
  /*                         Main loop - evaluate angles and rotate calipers               */

  /* all of edges will be checked while rotating calipers by 90 degrees */
  for (k = 0; k < points.size(); k++)
  {
    /* sinus of minimal angle */
    /*float sinus;*/

    /* compute cosine of angle between calipers side and polygon edge */
    /* dp - dot product */
    float dp0 = base_a * vect[seq[0]].x + base_b * vect[seq[0]].y;
    float dp1 = -base_b * vect[seq[1]].x + base_a * vect[seq[1]].y;
    float dp2 = -base_a * vect[seq[2]].x - base_b * vect[seq[2]].y;
    float dp3 = base_b * vect[seq[3]].x - base_a * vect[seq[3]].y;

    float cosalpha = dp0 * inv_vect_length[seq[0]];
    float maxcos = cosalpha;

    /* number of calipers edges, that has minimal angle with edge */
    int main_element = 0;

    /* choose minimal angle */
    cosalpha = dp1 * inv_vect_length[seq[1]];
    maxcos = (cosalpha > maxcos) ? (main_element = 1, cosalpha) : maxcos;
    cosalpha = dp2 * inv_vect_length[seq[2]];
    maxcos = (cosalpha > maxcos) ? (main_element = 2, cosalpha) : maxcos;
    cosalpha = dp3 * inv_vect_length[seq[3]];
    maxcos = (cosalpha > maxcos) ? (main_element = 3, cosalpha) : maxcos;

    /*rotate calipers*/
    {
      // get next base
      int pindex = seq[main_element];
      float lead_x = vect[pindex].x * inv_vect_length[pindex];
      float lead_y = vect[pindex].y * inv_vect_length[pindex];
      switch (main_element)
      {
        case 0:
          base_a = lead_x;
          base_b = lead_y;
          break;
        case 1:
          base_a = lead_y;
          base_b = -lead_x;
          break;
        case 2:
          base_a = -lead_x;
          base_b = -lead_y;
          break;
        case 3:
          base_a = -lead_y;
          base_b = lead_x;
          break;
        default:
          throw ros::Exception("main_element should be 0, 1, 2 or 3");
      }
    }
    /* change base point of main edge */
    seq[main_element] += 1;
    seq[main_element] = (seq[main_element] == points.size()) ? 0 : seq[main_element];

    /* now main element lies on edge aligned to calipers side */

    /* find opposite element i.e. transform  */
    /* 0->2, 1->3, 2->0, 3->1                */
    int opposite_el = main_element ^ 2;

    float dx = points[seq[opposite_el]].x - points[seq[main_element]].x;
    float dy = points[seq[opposite_el]].y - points[seq[main_element]].y;
    float dist;

    if (main_element & 1)
      dist = (float)fabs(dx * base_a + dy * base_b);
    else
      dist = (float)fabs(dx * (-base_b) + dy * base_a);

    if (dist > max_dist)
      max_dist = dist;
  }

  //        out[0] = max_dist;
}

}  // namespace costmap_2d
