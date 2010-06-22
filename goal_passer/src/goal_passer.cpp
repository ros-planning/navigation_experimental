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
#include <goal_passer/goal_passer.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_DECLARE_CLASS(goal_passer, GoalPasser, goal_passer::GoalPasser, nav_core::BaseGlobalPlanner)

namespace goal_passer {
  void GoalPasser::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    costmap_ros_ = costmap_ros;
    costmap_ros_->stop();
  }

  bool GoalPasser::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    plan.clear();
    if(last_goal_.header.stamp == goal.header.stamp && last_goal_.pose.position.x == goal.pose.position.x && last_goal_.pose.position.y == goal.pose.position.y
        && last_goal_.pose.position.z == goal.pose.position.z && last_goal_.pose.orientation.x == goal.pose.orientation.x
        && last_goal_.pose.orientation.y == goal.pose.orientation.y && last_goal_.pose.orientation.z == goal.pose.orientation.z
        && last_goal_.pose.orientation.w == goal.pose.orientation.w){
      return false;
    }

    plan.push_back(goal);
    last_goal_ = goal;

    return true;
  }
};
