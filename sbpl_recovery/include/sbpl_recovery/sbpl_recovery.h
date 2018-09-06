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
#ifndef SBPL_RECOVERY_SBPL_RECOVERY_H_
#define SBPL_RECOVERY_SBPL_RECOVERY_H_

#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <pose_follower/pose_follower.h>
#include <sbpl_lattice_planner/sbpl_lattice_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <boost/thread.hpp>
#include <base_local_planner/goal_functions.h>

namespace sbpl_recovery
{
  class SBPLRecovery : public nav_core::RecoveryBehavior
  {
    public:
      SBPLRecovery();

      // Initialize the parameters of the behavior
      void initialize (std::string n, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* global_costmap,
          costmap_2d::Costmap2DROS* local_costmap);

      // Run the behavior
      void runBehavior();

    private:
      void planCB(const nav_msgs::Path::ConstPtr& plan);
      double sqDistance(const geometry_msgs::PoseStamped& p1, 
          const geometry_msgs::PoseStamped& p2);
      std::vector<geometry_msgs::PoseStamped> makePlan();

      costmap_2d::Costmap2DROS* global_costmap_;
      costmap_2d::Costmap2DROS* local_costmap_;
      tf::TransformListener* tf_;
      sbpl_lattice_planner::SBPLLatticePlanner global_planner_;
      pose_follower::PoseFollower local_planner_;
      bool initialized_;
      ros::Subscriber plan_sub_;
      ros::Publisher vel_pub_;
      boost::mutex plan_mutex_;
      nav_msgs::Path plan_;
      double control_frequency_, sq_planning_distance_, controller_patience_;
      int planning_attempts_, attempts_per_run_;
      bool use_local_frame_;
  };

};
#endif
