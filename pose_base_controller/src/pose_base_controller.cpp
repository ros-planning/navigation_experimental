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
PoseBaseController::PoseBaseController() : tfl_(tf_),
                                           action_server_(ros::NodeHandle(),
                                                          "pose_base_controller",
                                        boost::bind(&PoseBaseController::execute, this, _1),
                                                          false) {
    ros::NodeHandle node_private("~");
    node_private.param("k_trans", K_trans_, 1.0);
    node_private.param("k_rot", K_rot_, 1.0);

    node_private.param("tolerance_trans", tolerance_trans_, 0.02);
    node_private.param("tolerance_rot", tolerance_rot_, 0.04);
    node_private.param("tolerance_timeout", tolerance_timeout_, 0.5);

    node_private.param("fixed_frame", fixed_frame_, std::string("odom_combined"));
    node_private.param("base_frame", base_frame_, std::string("base_link"));

    node_private.param("holonomic", holonomic_, true);

    node_private.param("max_vel_lin", max_vel_lin_, 0.9);
    node_private.param("max_vel_th", max_vel_th_, 1.4);

    node_private.param("min_vel_lin", min_vel_lin_, 0.0);
    node_private.param("min_vel_th", min_vel_th_, 0.0);
    node_private.param("min_in_place_vel_th", min_in_place_vel_th_, 0.0);
    node_private.param("in_place_trans_vel", in_place_trans_vel_, 0.0);
    node_private.param("frequency", freq_, 100.0);
    node_private.param("transform_tolerance", transform_tolerance_, 0.5);

    node_private.param("trans_stopped_velocity", trans_stopped_velocity_, 1e-4);
    node_private.param("rot_stopped_velocity", rot_stopped_velocity_, 1e-4);

    ros::NodeHandle node;
    odom_sub_ = node.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&PoseBaseController::odomCallback, this, _1));
    vel_pub_ = node.advertise<geometry_msgs::Twist>("base_controller/command", 10);

    action_server_.start();
    ROS_DEBUG("Started server");
  }

  void PoseBaseController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //we assume that the odometry is published in the frame of the base
    boost::mutex::scoped_lock lock(odom_lock_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    ROS_DEBUG("In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
        base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
  }

  double PoseBaseController::headingDiff(double x, double y, double pt_x, double pt_y, double heading)
  {
    double v1_x = x - pt_x;
    double v1_y = y - pt_y;
    double v2_x = cos(heading);
    double v2_y = sin(heading);

    double perp_dot = v1_x * v2_y - v1_y * v2_x;
    double dot = v1_x * v2_x + v1_y * v2_y;

    //get the signed angle
    double vector_angle = atan2(perp_dot, dot);

    return -1.0 * vector_angle;
  }

  tf2::Stamped<tf2::Transform> PoseBaseController::getRobotPose(){
    geometry_msgs::PoseStamped global_pose, robot_pose;
    robot_pose.pose.orientation.w = 1.0;
    robot_pose.header.frame_id = base_frame_;

    tf_.transform(robot_pose, global_pose, fixed_frame_);
    //ROS_INFO("Delay: %f", (global_pose.header.stamp - ros::Time::now()).toSec());

    tf2::Stamped<tf2::Transform> global_pose_tf;
    tf2::fromMsg(global_pose, global_pose_tf);
    return global_pose_tf;
  }

  move_base_msgs::MoveBaseGoal PoseBaseController::goalToFixedFrame(const move_base_msgs::MoveBaseGoal& goal){
    move_base_msgs::MoveBaseGoal fixed_goal;
    geometry_msgs::PoseStamped pose;
    pose = goal.target_pose;

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    pose.header.stamp = ros::Time();
    
    try{
      tf_.transform(pose, fixed_goal.target_pose, fixed_frame_);
    }
    catch(tf2::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          pose.header.frame_id.c_str(), fixed_frame_.c_str(), ex.what());
      return goal;
    }

    return fixed_goal;
  }

  void PoseBaseController::execute(const move_base_msgs::MoveBaseGoalConstPtr& user_goal){
    bool success = false;

    move_base_msgs::MoveBaseGoal goal = goalToFixedFrame(*user_goal);

    success = controlLoop(goal);

    //based on the control loop's exit status... we'll set our goal status
    if(success){
      //wait until we're stopped before returning success
      ros::Rate r(10.0);
      while(!stopped()){
        geometry_msgs::Twist empty_twist;
        vel_pub_.publish(empty_twist);
        r.sleep();
      }
      action_server_.setSucceeded();
    }
    else{
      //if a preempt was requested... the control loop exits for that reason
      if(action_server_.isPreemptRequested())
        action_server_.setPreempted();
      else
        action_server_.setAborted();
    }
  }

  bool PoseBaseController::stopped(){
    //copy over the odometry information
    nav_msgs::Odometry base_odom;
    {
      boost::mutex::scoped_lock lock(odom_lock_);
      base_odom = base_odom_;
    }

    return fabs(base_odom.twist.twist.angular.z) <= rot_stopped_velocity_
      && fabs(base_odom.twist.twist.linear.x) <= trans_stopped_velocity_
      && fabs(base_odom.twist.twist.linear.y) <= trans_stopped_velocity_;
  }

  bool PoseBaseController::controlLoop(const move_base_msgs::MoveBaseGoal& current_goal){
    if(freq_ == 0.0)
      return false;

    ros::Rate r(freq_);
    ros::Time goal_reached_time = ros::Time::now();
    while(ros::ok()){
      if(action_server_.isPreemptRequested()){
        return false;
      }
      ROS_DEBUG("At least looping");
      tf2::Stamped<tf2::Transform> target_pose;
      tf2::convert(current_goal.target_pose, target_pose);

      //get the current pose of the robot in the fixed frame
      tf2::Stamped<tf2::Transform> robot_pose;
      try {
        robot_pose = getRobotPose();
        //make sure that the transform to the dance frame isn't too out of date
        if(robot_pose.stamp_ + ros::Duration(transform_tolerance_) < ros::Time::now()){
          ROS_WARN("The %s frame to %s frame transform is more than %.2f seconds old, will not move",
              fixed_frame_.c_str(), base_frame_.c_str(), transform_tolerance_);
          geometry_msgs::Twist empty_twist;
          vel_pub_.publish(empty_twist);
          return false;
        }
      }
      catch(tf2::TransformException &ex){
        ROS_ERROR("Can't transform: %s\n", ex.what());
        geometry_msgs::Twist empty_twist;
        vel_pub_.publish(empty_twist);
        return false;
      }
      ROS_DEBUG("PoseBaseController: current robot pose %f %f ==> %f", robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), tf2::getYaw(robot_pose.getRotation()));
      ROS_DEBUG("PoseBaseController: target robot pose %f %f ==> %f", target_pose.getOrigin().x(), target_pose.getOrigin().y(), tf2::getYaw(target_pose.getRotation()));

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
        geometry_msgs::Twist empty_twist;
        vel_pub_.publish(empty_twist);
        return true;
      }

      r.sleep();
    }
    return false;
  }

  geometry_msgs::Twist PoseBaseController::diff2D(const tf2::Transform& pose1, const tf2::Transform& pose2)
  {
    geometry_msgs::Twist res;
    tf2::Transform diff = pose2.inverse() * pose1;
    res.linear.x = diff.getOrigin().x();
    res.linear.y = diff.getOrigin().y();
    res.angular.z = tf2::getYaw(diff.getRotation());

    if(holonomic_ || (fabs(res.linear.x) <= tolerance_trans_ && fabs(res.linear.y) <= tolerance_trans_))
      return res;

    //in the case that we're not rotating to our goal position and we have a non-holonomic robot
    //we'll need to command a rotational velocity that will help us reach our desired heading
    
    //we want to compute a goal based on the heading difference between our pose and the target
    double yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(), 
        pose2.getOrigin().x(), pose2.getOrigin().y(), tf2::getYaw(pose2.getRotation()));

    //we'll also check if we can move more effectively backwards
    double neg_yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(), 
        pose2.getOrigin().x(), pose2.getOrigin().y(), M_PI + tf2::getYaw(pose2.getRotation()));

    //check if its faster to just back up
    if(fabs(neg_yaw_diff) < fabs(yaw_diff)){
      ROS_DEBUG("Negative is better: %.2f", neg_yaw_diff);
      yaw_diff = neg_yaw_diff;
    }

    //compute the desired quaterion
    tf2::Quaternion rot_diff;
    rot_diff.setRPY(0.0, 0.0, yaw_diff);
    tf2::Quaternion rot = pose2.getRotation() * rot_diff;
    tf2::Transform new_pose = pose1;
    new_pose.setRotation(rot);

    diff = pose2.inverse() * new_pose;
    res.linear.x = diff.getOrigin().x();
    res.linear.y = diff.getOrigin().y();
    res.angular.z = tf2::getYaw(diff.getRotation());
    return res;
  }


  geometry_msgs::Twist PoseBaseController::limitTwist(const geometry_msgs::Twist& twist)
  {
    geometry_msgs::Twist res = twist;
    res.linear.x *= K_trans_;
    if(!holonomic_)
      res.linear.y = 0.0;
    else    
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

    if(fabs(res.linear.x) < in_place_trans_vel_ && fabs(res.linear.y) < in_place_trans_vel_){
      if (fabs(res.angular.z) < min_in_place_vel_th_) res.angular.z = min_in_place_vel_th_ * sign(res.angular.z);
    }

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
