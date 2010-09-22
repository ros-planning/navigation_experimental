/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *
 */

/**
 * \file 
 * 
 * Filter out isolated points
 *
 * \author Bhaskara Marthi
 */

#include <filters/filter_base.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <boost/circular_buffer.hpp>

namespace isolated_point_filter
{

namespace sm=sensor_msgs;
namespace gm=geometry_msgs;
using std::string;
using std::vector;



class IsolatedPointFilter : public filters::FilterBase<sm::PointCloud>
{
public:

  IsolatedPointFilter ();
  bool configure ();
  bool update (const sm::PointCloud& in, sm::PointCloud& out);

private:

  unsigned getIndex (int x, int y) const;
  void resetStamps ();
  bool withinBounds (const gm::Point32& p) const;
  void getCoords (const gm::Point32& p, int* x, int* y) const;
  template <class T>
  void readParam (const string& name, T& place);

  ros::NodeHandle nh_;
  tf::TransformListener tf_;

  // params
  int nx_;
  int ny_;
  double resolution_;
  string frame_;
  string fixed_frame_;
  double x_min_;
  double y_min_;
  double x_max_;
  double y_max_;
  double z_max_;
  int r_big_;
  int r_small_;
  int negate_;
  unsigned buffer_size_;
  

  // state
  unsigned counter_;
  vector<unsigned> stamps_;
  boost::circular_buffer<sm::PointCloud> cloud_buffer_; // Because the 'clouds' in our case are just lines
  
};

IsolatedPointFilter::IsolatedPointFilter () :
  tf_(nh_), fixed_frame_("odom_combined"), z_max_(10), buffer_size_(1), counter_(0), cloud_buffer_(1)
{}

template <class T>
void IsolatedPointFilter::readParam (const string& name, T& place)
{
  bool found = getParam(name, place);
  if (!found) {
    ROS_FATAL_STREAM ("Did not find parameter " << name);
    ROS_BREAK();
  }
}

bool IsolatedPointFilter::configure ()
{
  readParam("frame", frame_);
  readParam("resolution", resolution_);
  readParam("x_min", x_min_);
  readParam("x_max", x_max_);
  readParam("y_min", y_min_);
  readParam("y_max", y_max_);
  getParam("z_max", z_max_);
  readParam("inner_radius", r_small_);
  readParam("outer_radius", r_big_);
  getParam("negate", negate_);
  getParam("buffer_size", buffer_size_);
  getParam("fixed_frame", fixed_frame_);
  nx_ = ceil((x_max_-x_min_)/resolution_);
  ny_ = ceil((y_max_-y_min_)/resolution_);
  stamps_.resize(nx_*ny_);
  cloud_buffer_.resize(buffer_size_);
  ROS_INFO_STREAM ("Initialized isolated_point_filter.  Frame is " << frame_ << ", x bounds are " << x_min_ <<
                   ", " << x_max_ << "; y bounds are " << y_min_ << ", " << y_max_ << " points above " <<
                   z_max_ << " will never be considered isolated; box radii are " << r_small_ << ", " <<
                   r_big_  << " grid size is " << nx_ << " x " << ny_ << ", bufsize = " << buffer_size_);
  ROS_ASSERT_MSG (x_min_ < x_max_ && y_min_ < y_max_ && r_small_ < r_big_ && 0 <= r_small_ && 0 < z_max_,
                  "Params don't pass sanity check");
  return true;
}

void IsolatedPointFilter::resetStamps ()
{
  fill(stamps_.begin(), stamps_.end(), 0);
}

unsigned IsolatedPointFilter::getIndex (const int x, const int y) const
{
  ROS_ASSERT ((x>=0) && (x<nx_) && (y>=0) && (y<ny_));
  return ny_*x + y;
}

bool IsolatedPointFilter::withinBounds (const gm::Point32& p) const
{
  return ((p.x > x_min_) && (p.x < x_max_) && (p.y > y_min_) && (p.y < y_max_) &&
          (p.z < z_max_));
}

void IsolatedPointFilter::getCoords (const gm::Point32& p, int* x, int* y) const
{
  *x = floor((p.x-x_min_)/resolution_);
  *y = floor((p.y-y_min_)/resolution_);
  ROS_ASSERT ((*x>=0) && (*x < nx_) && (*y>=0) && (*y < ny_));
}


bool IsolatedPointFilter::update (const sm::PointCloud& in, sm::PointCloud& out)
{
  out.points.resize(0);
  out.header.frame_id = frame_;
  out.header.stamp = in.header.stamp;
  if (counter_==0) {
    resetStamps();
  }
  counter_++;
  ROS_DEBUG_STREAM_NAMED ("update", "In update loop " << counter_ <<
                          " on point cloud with " << in.points.size() << " points");


  try {
    sm::PointCloud transformed;
    tf_.transformPointCloud (fixed_frame_, in, transformed);
    cloud_buffer_.push_back(transformed);
  }
  catch (tf::TransformException& e) {
    ROS_WARN_STREAM ("Received tf exception " << e.what() << "; skipping cloud");
    return false;
  }
  if (cloud_buffer_.size() < buffer_size_) {
    // skip
    // out = *(--cloud_buffer_.end());
    return true;
  }
  else {

    sm::PointCloud combined, last;
    try {
      tf_.transformPointCloud (frame_, *(--cloud_buffer_.end()), last);
      BOOST_FOREACH (const sm::PointCloud& c, cloud_buffer_) {
        sm::PointCloud transformed;
        tf_.transformPointCloud (frame_, c, transformed);
        BOOST_FOREACH (const gm::Point32& p, transformed.points) {
          combined.points.push_back(p);
        }
      }
    }
    catch (tf::TransformException& e) {
      if (!negate_) {
        ROS_DEBUG_STREAM_NAMED ("transforms", "Received tf exception " << e.what () << "; passing along cloud");
        out.points = last.points;
      }
      return true;
    }

    // mark each cell that is occupied by a point in this cloud
    BOOST_FOREACH (const gm::Point32& p, combined.points) {
      if (withinBounds(p)) {
        int x, y;
        getCoords(p, &x, &y);
        stamps_[getIndex(x, y)] = counter_;
      }
    }

    BOOST_FOREACH (const gm::Point32& p, last.points) {
      bool is_isolated;
      if (withinBounds(p)) {
        int x0, y0;
        bool found=false;
        getCoords(p, &x0, &y0);
        for (int x = x0-r_big_; x <= x0+r_big_; x++) {
          if ((x<0) || (x>=nx_))
            continue; // Out of map bounds
          for (int y = y0-r_big_; y <= y0+r_big_; y++) {
            // OK, (x,y) now represents a point near (x0, y0)
          
            // Skip it if it's outside map bounds
            if ((y<0) || (y>=ny_))
              continue; 
          
            // Skip it if it's within the inner square at (x0, y0)
            if ((x>=x0-r_small_) && (x<=x0+r_small_) &&
                (y>=y0-r_small_) && (y<=y0+r_small_))
              continue; 

            // Else if it has a point, that means (x0, y0) has a neighbor
            if (stamps_[getIndex(x, y)] == counter_) {
              found = true;
              break;
            }
          }
          if (found)
            break;
        }
        if (found)
          is_isolated = false;
        else {
          is_isolated = true;
          ROS_DEBUG_STREAM_NAMED ("isolated", " found isolated point " << p);
        }
      }
    
      // Points not in the filter bounds are always kept
      else 
        is_isolated = false;

      if ((is_isolated && negate_) || (!is_isolated && !negate_))
        out.points.push_back(p);
    }

    out.header.frame_id = frame_;
    out.header.stamp = in.header.stamp;
    ROS_DEBUG_STREAM_NAMED ("update", "Update loop finished with " << out.points.size() << " points in "
                            << frame_ << " frame");
  }
  return true;
}
 

} // namespace

PLUGINLIB_DECLARE_CLASS(isolated_point_filter, IsolatedPointFilter,
                        isolated_point_filter::IsolatedPointFilter,
                        filters::FilterBase<sensor_msgs::PointCloud>)

