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
#ifndef POSE_BASE_CONTROLLER_POSE_BASE_CONTROLLER_H_
#define POSE_BASE_CONTROLLER_POSE_BASE_CONTROLLER_H_
#include <ros/ros.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <boost/thread.hpp>

namespace pose_base_controller {
  class PoseBaseController {
    private:
      typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

    public:
      PoseBaseController();

      ~PoseBaseController() {}

      void execute(const move_base_msgs::MoveBaseGoalConstPtr& user_goal);
      bool controlLoop(const move_base_msgs::MoveBaseGoal& current_goal);
      tf2::Stamped<tf2::Transform> getRobotPose();

      inline double sign(double n){
        return n < 0.0 ? -1.0 : 1.0;
      }

      geometry_msgs::Twist diff2D(const tf2::Transform& pose1, const tf2::Transform& pose2);
      geometry_msgs::Twist limitTwist(const geometry_msgs::Twist& twist);
      double headingDiff(double pt_x, double pt_y, double x, double y, double heading);
      move_base_msgs::MoveBaseGoal goalToFixedFrame(const move_base_msgs::MoveBaseGoal& goal);

    private:
      void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
      bool stopped();

      MoveBaseActionServer action_server_;
      tf2_ros::Buffer tf_;
      tf2_ros::TransformListener tfl_;
      ros::Publisher vel_pub_;
      double K_trans_, K_rot_, tolerance_trans_, tolerance_rot_;
      double tolerance_timeout_, freq_;
      double max_vel_lin_, max_vel_th_;
      double min_vel_lin_, min_vel_th_;
      double min_in_place_vel_th_, in_place_trans_vel_;
      double transform_tolerance_;
      std::string fixed_frame_, base_frame_;
      bool holonomic_;
      boost::mutex odom_lock_;
      ros::Subscriber odom_sub_;
      nav_msgs::Odometry base_odom_;
      double trans_stopped_velocity_, rot_stopped_velocity_;
  };
};
#endif
