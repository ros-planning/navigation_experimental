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
#include <pose_base_controller/pose_base_controller.h>

namespace pose_base_controller {
  PoseBaseController::PoseBaseController() : action_server_(ros::NodeHandle(), 
                                        "pose_base_controller", 
                                        boost::bind(&PoseBaseController::controlLoop, this, _1),
                                        false){
    ros::NodeHandle node_private("~");
    node_private.param("k_trans", K_trans_, 1.0);
    node_private.param("k_rot", K_rot_, 1.0);

    node_private.param("tolerance_trans", tolerance_trans_, 0.02);
    node_private.param("tolerance_rot", tolerance_rot_, 0.04);
    node_private.param("tolerance_timeout", tolerance_timeout_, 0.5);

    node_private.param("fixed_frame", fixed_frame_, std::string("odom_combined"));
    node_private.param("base_frame", base_frame_, std::string("base_link"));

    node_private.param("max_vel_lin", max_vel_lin_, 0.9);
    node_private.param("max_vel_th", max_vel_th_, 1.4);

    node_private.param("min_vel_lin", min_vel_lin_, 0.0);
    node_private.param("min_vel_th", min_vel_th_, 0.0);
    node_private.param("frequency", freq_, 100.0);
    node_private.param("transform_tolerance", transform_tolerance_, 0.5);

    ros::NodeHandle node;
    vel_pub_ = node.advertise<geometry_msgs::Twist>("base_controller/command", 10);

    action_server_.start();
  }

  tf::Stamped<tf::Pose> PoseBaseController::getRobotPose(){
    tf::Stamped<tf::Pose> global_pose, robot_pose;
    global_pose.setIdentity();
    robot_pose.setIdentity();
    robot_pose.frame_id_ = base_frame_;
    robot_pose.stamp_ = ros::Time();

    tf_.transformPose(fixed_frame_, robot_pose, global_pose);
    //ROS_INFO("Delay: %f", (global_pose.stamp_ - ros::Time::now()).toSec());
    return global_pose;
  }

  void PoseBaseController::controlLoop(const move_base_msgs::MoveBaseGoalConstPtr& current_goal){
    if(freq_ == 0.0)
      return;

    ros::Rate r(freq_);
    ros::Time goal_reached_time = ros::Time::now();
    while(ros::ok()){
      if(action_server_.isPreemptRequested()){
        action_server_.setPreempted();
        return;
      }
      ROS_DEBUG("At least looping");
      tf::Stamped<tf::Pose> target_pose;
      tf::poseStampedMsgToTF(current_goal->target_pose, target_pose);


      //get the current pose of the robot in the fixed frame
      tf::Stamped<tf::Pose> robot_pose;
      try {
        robot_pose = getRobotPose();
        //make sure that the transform to the dance frame isn't too out of date
        if(robot_pose.stamp_ + ros::Duration(transform_tolerance_) < ros::Time::now()){
          ROS_WARN("The %s frame to %s frame transform is more than %.2f seconds old, will not move",
              fixed_frame_.c_str(), base_frame_.c_str(), transform_tolerance_);
          geometry_msgs::Twist empty_twist;
          vel_pub_.publish(empty_twist);
          action_server_.setAborted();
          return;
        }
      }
      catch(tf::TransformException &ex){
        ROS_ERROR("Can't transform: %s\n", ex.what());
        geometry_msgs::Twist empty_twist;
        vel_pub_.publish(empty_twist);
        action_server_.setAborted();
        return;
      }
      ROS_DEBUG("PoseBaseController: current robot pose %f %f ==> %f", robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), tf::getYaw(robot_pose.getRotation()));
      ROS_DEBUG("PoseBaseController: target robot pose %f %f ==> %f", target_pose.getOrigin().x(), target_pose.getOrigin().y(), tf::getYaw(target_pose.getRotation()));

      //get the difference between the two poses
      geometry_msgs::Twist diff = diff2D(target_pose, robot_pose);
      ROS_DEBUG("PoseBaseController: diff %f %f ==> %f", diff.linear.x, diff.linear.y, diff.angular.z);

      //publish the desired velocity command to the base
      vel_pub_.publish(limitTwist(diff));

      //if we haven't reached our goal... we'll update time
      if (fabs(diff.linear.x) > tolerance_trans_ || fabs(diff.linear.y) > tolerance_trans_ || fabs(diff.angular.z) > tolerance_rot_)
        goal_reached_time = ros::Time::now();

      //check if we've reached our goal for long enough to succeed
      if(goal_reached_time + ros::Duration(tolerance_timeout_) < ros::Time::now()){
        action_server_.setSucceeded();
        geometry_msgs::Twist empty_twist;
        vel_pub_.publish(empty_twist);
        return;
      }

      r.sleep();
    }
  }

  geometry_msgs::Twist PoseBaseController::diff2D(const tf::Pose& pose1, const tf::Pose& pose2)
  {
    geometry_msgs::Twist res;
    tf::Pose diff = pose2.inverse() * pose1;
    res.linear.x = diff.getOrigin().x();
    res.linear.y = diff.getOrigin().y();
    res.angular.z = tf::getYaw(diff.getRotation());
    return res;
  }


  geometry_msgs::Twist PoseBaseController::limitTwist(const geometry_msgs::Twist& twist)
  {
    geometry_msgs::Twist res = twist;
    res.linear.x *= K_trans_;
    res.linear.y *= K_trans_;
    res.angular.z *= K_rot_;

    //make sure to bound things by our velocity limits
    double lin_overshoot = sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y) / max_vel_lin_;
    if (lin_overshoot > 1.0) 
    {
      res.linear.x /= lin_overshoot;
      res.linear.y /= lin_overshoot;
    }
    if (fabs(res.angular.z) > max_vel_th_) res.angular.z = max_vel_th_ * sign(res.angular.z);
    if (fabs(res.angular.z) < min_vel_th_) res.angular.z = min_vel_th_ * sign(res.angular.z);

    ROS_DEBUG("Angular command %f", res.angular.z);
    return res;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_base_controller");

  pose_base_controller::PoseBaseController pbc;

  ros::spin();
  return 0;
}
