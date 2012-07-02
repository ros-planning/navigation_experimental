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
 * Ros node that provides an implementation of the FollowBaseTrajectory action.
 * Unlike the MoveBase action, this action allows passing in an entire trajectory
 * which will be followed.
 *
 * \author Bhaskara Marthi
 */

#include <eband_local_planner/FollowBaseTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <eband_local_planner/eband_local_planner_ros.h>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>

namespace al=actionlib;
namespace gm=geometry_msgs;
namespace cmap=costmap_2d;
namespace eband=eband_local_planner;

using std::string;
using boost::format;

namespace eband_local_planner
{

typedef FollowBaseTrajectoryGoalConstPtr GoalPtr;
typedef FollowBaseTrajectoryResult Result;
typedef boost::mutex::scoped_lock Lock;

Result result (const string& msg)
{
  Result r;
  r.msg = msg;
  return r;
}

Result result (const format& f)
{
  return result(f.str());
}

// Activates costmap within this scope and pauses it when leaving
struct ScopedCostmapActivate
{
  ScopedCostmapActivate (cmap::Costmap2DROS* cmap) : cmap(cmap)
  {
    cmap->start();
  }

  ~ScopedCostmapActivate ()
  {
    cmap->pause();
  }

  cmap::Costmap2DROS* cmap;
};

// Holds ros node state
class Node
{
public:
  
  Node ();
  void execute (const GoalPtr& goal);

private:

  ros::Rate control_rate_;
  ros::Duration control_timeout_;

  ros::Time last_valid_control_;
  
  boost::mutex mutex_;
  ros::NodeHandle nh_;
  tf::TransformListener tf_;
  al::SimpleActionServer<FollowBaseTrajectoryAction> as_;
  cmap::Costmap2DROS costmap_ros_;
  ros::Publisher vel_pub_;
  eband::EBandPlannerROS tc_;

};

inline gm::Twist zeroVelocity ()
{
  return gm::Twist();
}


Node::Node () :
  control_rate_(10.0), control_timeout_(2.0),
  as_(nh_, "follow_base_trajectory", boost::bind(&Node::execute, this, _1), false),
  costmap_ros_("elastic_band_planner", tf_),
  vel_pub_(nh_.advertise<gm::Twist>("cmd_vel", 10))
{
  Lock l(mutex_);
  costmap_ros_.pause();
  
  // Initialize the 'trajectory controller' that will do the work
  tc_.initialize("elastic_band_planner", &tf_, &costmap_ros_);
  as_.start();
  ROS_INFO ("Eband node initialized");
}

void Node::execute (const GoalPtr& goal)
{
  // Make it so costmap is active within body of this function
  ScopedCostmapActivate cmap_activate(&costmap_ros_);
  Lock l(mutex_);

  const unsigned n=goal->path.size();
  if (n<2)
  {
    as_.setAborted(result(format("Path had length %1%") % n));
    return;
  }
  // Now guaranteed path has length > 1
  
  ROS_DEBUG_STREAM_NAMED ("control", "Received trajectory of length " << n <<
                          " going from " << goal->path[0].pose << " to "
                          << goal->path[n-1].pose);
  if (!tc_.setPlan(goal->path))
  {
    as_.setAborted(result("Failed setting plan for controller;"
                          "see stdout for error message"));
    return;
  }
  // Now guaranteed that controller plan was set correctly

  // Main loop
  last_valid_control_ = ros::Time::now();
  while (ros::ok() && !tc_.isGoalReached())
  {
    gm::Twist vel;
    if (!tc_.computeVelocityCommands(vel))
    {
      ROS_DEBUG_STREAM_NAMED ("control", "Didn't find valid control");
      vel_pub_.publish(zeroVelocity());
      if (ros::Time::now() - last_valid_control_ > control_timeout_)
      {
        as_.setAborted(result("Timed out finding a valid control"));
        return;
      }
      continue;
    }

    // Successfully got command
    ROS_DEBUG_STREAM_THROTTLE_NAMED (1.0, "control", "Sending control " << vel);
    vel_pub_.publish(vel);
    last_valid_control_ = ros::Time::now();
    ros::spinOnce();
    control_rate_.sleep();
  }

  ROS_DEBUG_STREAM_NAMED ("control", "Succeeded");
  as_.setSucceeded();
}

} // namespace

int main (int argc, char** argv)
{
  ros::init(argc, argv, "eband_action");
  ros::NodeHandle nh;
  eband_local_planner::Node node;
  ros::spin();
  return 0;
}
