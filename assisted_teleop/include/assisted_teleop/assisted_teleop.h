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
#ifndef ASSISTED_TELEOP_ASSISTED_TELEOP_H_
#define ASSISTED_TELEOP_ASSISTED_TELEOP_H_
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <boost/thread.hpp>
#include <Eigen/Core>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


namespace assisted_teleop {
  class AssistedTeleop {
    public:
      AssistedTeleop();
      ~AssistedTeleop();
    private:
      void velCB(const geometry_msgs::TwistConstPtr& vel);
      void controlLoop();

      tf2_ros::Buffer tf_;
      tf2_ros::TransformListener tfl_;
      costmap_2d::Costmap2DROS costmap_ros_;
      double controller_frequency_;
      base_local_planner::TrajectoryPlannerROS planner_;
      boost::mutex mutex_;
      geometry_msgs::Twist cmd_vel_;
      boost::thread* planning_thread_;
      double theta_range_;
      int num_th_samples_, num_x_samples_;
      ros::Publisher pub_;
      ros::Subscriber sub_;
      double collision_trans_speed_, collision_rot_speed_;
  };
};
#endif
