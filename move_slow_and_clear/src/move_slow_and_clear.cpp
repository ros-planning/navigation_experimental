/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*********************************************************************/
#include <move_slow_and_clear/move_slow_and_clear.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(move_slow_and_clear, MoveSlowAndClear, move_slow_and_clear::MoveSlowAndClear,
    nav_core::RecoveryBehavior)

namespace move_slow_and_clear
{
  MoveSlowAndClear::MoveSlowAndClear():global_costmap_(NULL), local_costmap_(NULL), initialized_(false){}

  void MoveSlowAndClear::initialize (std::string n, tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* global_costmap,
      costmap_2d::Costmap2DROS* local_costmap)
  {
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    ros::NodeHandle private_nh_("~/" + n);
    private_nh_.param("clearing_distance", clearing_distance_, 0.5);
    private_nh_.param("limited_speed", limited_speed_, 0.25);

    std::string planner_namespace;
    private_nh_.param("planner_namespace", planner_namespace, std::string("DWAPlannerROS"));

    planner_nh_ = ros::NodeHandle("~/" + planner_namespace);

    initialized_ = true;
  }

  void MoveSlowAndClear::runBehavior()
  {
    if(!initialized_)
    {
      ROS_ERROR("This recovery behavior has not been initialized, doing nothing.");
      return;
    }

    ROS_DEBUG("Running move slow and clear behavior");
    tf::Stamped<tf::Pose> global_pose, local_pose;
    global_costmap_->getRobotPose(global_pose);
    local_costmap_->getRobotPose(local_pose);

    std::vector<geometry_msgs::Point> global_poly, local_poly;

    geometry_msgs::Point pt;
    for(int i = -1; i <= 1; i+=2)
    {
      pt.x = global_pose.getOrigin().x() + i * clearing_distance_;
      pt.y = global_pose.getOrigin().y() + i * clearing_distance_;
      global_poly.push_back(pt);

      pt.x = global_pose.getOrigin().x() + i * clearing_distance_;
      pt.y = global_pose.getOrigin().y() + -1.0 * i * clearing_distance_;
      global_poly.push_back(pt);

      pt.x = local_pose.getOrigin().x() + i * clearing_distance_;
      pt.y = local_pose.getOrigin().y() + i * clearing_distance_;
      local_poly.push_back(pt);

      pt.x = local_pose.getOrigin().x() + i * clearing_distance_;
      pt.y = local_pose.getOrigin().y() + -1.0 * i * clearing_distance_;
      local_poly.push_back(pt);
    }

    //clear the desired space in both costmaps
    global_costmap_->setConvexPolygonCost(global_poly, costmap_2d::FREE_SPACE);
    local_costmap_->setConvexPolygonCost(local_poly, costmap_2d::FREE_SPACE);

    //get the old maximum speed for the robot... we'll want to set it back
    if(!planner_nh_.getParam("max_trans_vel", old_speed_))
    {
      ROS_ERROR("The planner %s, does not have the parameter max_trans_vel", planner_nh_.getNamespace().c_str());
    }

    //limit the speed of the robot until it moves a certain distance
    setRobotSpeed(limited_speed_);
    distance_check_timer_ = private_nh_.createTimer(ros::Duration(0.1), &MoveSlowAndClear::distanceCheck, this);
  }

  void MoveSlowAndClear::distanceCheck(const ros::TimerEvent& e)
  {
    ROS_ERROR("Timer called");
  }

  void MoveSlowAndClear::setRobotSpeed(double speed)
  {
    std::ostringstream command;
    command << "rosrun dynamic_reconfigure dynparam set " << planner_nh_.getNamespace() << " max_trans_vel " << speed;
    ROS_ERROR("%s", command.str().c_str());
    if(system(command.str().c_str()) < 0)
    {
      ROS_ERROR("Something went wrong in the system call to dynparam");
    }
  }

};
