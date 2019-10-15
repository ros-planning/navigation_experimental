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
#include <nav_msgs/Path.h>
#include <pose_follower/pose_follower.h>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(pose_follower::PoseFollower, nav_core::BaseLocalPlanner)

namespace pose_follower {
  PoseFollower::PoseFollower(): tf_(NULL), costmap_ros_(NULL) {}

  PoseFollower::~PoseFollower() {
    if (dsrv_)
      delete dsrv_;
  }

  void PoseFollower::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    current_waypoint_ = 0;
    goal_reached_time_ = ros::Time::now();
    ros::NodeHandle node_private("~/" + name);

    collision_planner_.initialize(name + "/collision_planner", tf_, costmap_ros_);

    //set this to true if you're using a holonomic robot
    node_private.param("holonomic", holonomic_, true);

    global_plan_pub_ = node_private.advertise<nav_msgs::Path>("global_plan", 1);

    ros::NodeHandle node;
    odom_sub_ = node.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&PoseFollower::odomCallback, this, _1));

    dsrv_ = new dynamic_reconfigure::Server<pose_follower::PoseFollowerConfig>(
        ros::NodeHandle(node_private));
    dynamic_reconfigure::Server<pose_follower::PoseFollowerConfig>::CallbackType cb =
        boost::bind(&PoseFollower::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    ROS_DEBUG("Initialized");
  }

  void PoseFollower::reconfigureCB(pose_follower::PoseFollowerConfig &config, uint32_t level) {
    max_vel_lin_ = config.max_vel_lin;
    max_vel_th_ = config.max_vel_th;
    min_vel_lin_ = config.min_vel_lin;
    min_vel_th_ = config.min_vel_th;
    min_in_place_vel_th_ = config.min_in_place_vel_th;
    in_place_trans_vel_ = config.in_place_trans_vel;
    trans_stopped_velocity_ = config.trans_stopped_velocity;
    rot_stopped_velocity_ = config.rot_stopped_velocity;

    tolerance_trans_ = config.tolerance_trans;
    tolerance_rot_ = config.tolerance_rot;
    tolerance_timeout_ = config.tolerance_timeout;

    samples_ = config.samples;
    allow_backwards_ = config.allow_backwards;
    turn_in_place_first_ = config.turn_in_place_first;
    max_heading_diff_before_moving_ = config.max_heading_diff_before_moving;

    K_trans_ = config.k_trans;
    K_rot_ = config.k_rot;
  }

  void PoseFollower::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //we assume that the odometry is published in the frame of the base
    boost::mutex::scoped_lock lock(odom_lock_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    ROS_DEBUG("In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
        base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
  }

  double PoseFollower::headingDiff(double x, double y, double pt_x, double pt_y, double heading)
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

  bool PoseFollower::stopped(){
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

  void PoseFollower::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path,
                               const ros::Publisher &pub) {
    // given an empty path we won't do anything
    if (path.empty())
      return;

    // create a path message
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
      gui_path.poses[i] = path[i];
    }
    pub.publish(gui_path);
  }

  bool PoseFollower::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    //get the current pose of the robot in the fixed frame
    geometry_msgs::PoseStamped robot_pose;
    if(!costmap_ros_->getRobotPose(robot_pose)){
      ROS_ERROR("Can't get robot pose");
      geometry_msgs::Twist empty_twist;
      cmd_vel = empty_twist;
      return false;
    }

    //we want to compute a velocity command based on our current waypoint
    geometry_msgs::PoseStamped target_pose = global_plan_[current_waypoint_];
    ROS_DEBUG("PoseFollower: current robot pose %f %f ==> %f", robot_pose.pose.position.x, robot_pose.pose.position.y, tf2::getYaw(robot_pose.pose.orientation));
    ROS_DEBUG("PoseFollower: target robot pose %f %f ==> %f", target_pose.pose.position.x, target_pose.pose.position.y, tf2::getYaw(target_pose.pose.orientation));

    //get the difference between the two poses
    geometry_msgs::Twist diff = diff2D(target_pose.pose, robot_pose.pose);
    ROS_DEBUG("PoseFollower: diff %f %f ==> %f", diff.linear.x, diff.linear.y, diff.angular.z);

    geometry_msgs::Twist limit_vel = limitTwist(diff);

    geometry_msgs::Twist test_vel = limit_vel;
    bool legal_traj = collision_planner_.checkTrajectory(test_vel.linear.x, test_vel.linear.y, test_vel.angular.z, true);

    double scaling_factor = 1.0;
    double ds = scaling_factor / samples_;

    //let's make sure that the velocity command is legal... and if not, scale down
    if(!legal_traj){
      for(int i = 0; i < samples_; ++i){
        test_vel.linear.x = limit_vel.linear.x * scaling_factor;
        test_vel.linear.y = limit_vel.linear.y * scaling_factor;
        test_vel.angular.z = limit_vel.angular.z * scaling_factor;
        test_vel = limitTwist(test_vel);
        if(collision_planner_.checkTrajectory(test_vel.linear.x, test_vel.linear.y, test_vel.angular.z, false)){
          legal_traj = true;
          break;
        }
        scaling_factor -= ds;
      }
    }

    if(!legal_traj){
      ROS_ERROR("Not legal (%.2f, %.2f, %.2f)", limit_vel.linear.x, limit_vel.linear.y, limit_vel.angular.z);
      geometry_msgs::Twist empty_twist;
      cmd_vel = empty_twist;
      return false;
    }

    //if it is legal... we'll pass it on
    cmd_vel = test_vel;

    bool in_goal_position = false;
    while(fabs(diff.linear.x) <= tolerance_trans_ &&
          fabs(diff.linear.y) <= tolerance_trans_ &&
	  fabs(diff.angular.z) <= tolerance_rot_)
    {
      if(current_waypoint_ < global_plan_.size() - 1)
      {
        current_waypoint_++;
        target_pose = global_plan_[current_waypoint_];
        diff = diff2D(target_pose.pose, robot_pose.pose);
      }
      else
      {
        ROS_INFO("Reached goal: %d", current_waypoint_);
        in_goal_position = true;
        break;
      }
    }

    //if we're not in the goal position, we need to update time
    if(!in_goal_position)
      goal_reached_time_ = ros::Time::now();

    //check if we've reached our goal for long enough to succeed
    if(goal_reached_time_ + ros::Duration(tolerance_timeout_) < ros::Time::now()){
      geometry_msgs::Twist empty_twist;
      cmd_vel = empty_twist;
    }

    return true;
  }

  bool PoseFollower::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan){
    current_waypoint_ = 0;
    goal_reached_time_ = ros::Time::now();
    if(!transformGlobalPlan(*tf_, global_plan, *costmap_ros_, costmap_ros_->getGlobalFrameID(), global_plan_)){
      ROS_ERROR("Could not transform the global plan to the frame of the controller");
      return false;
    }

    ROS_DEBUG("global plan size: %lu", global_plan_.size());
    publishPlan(global_plan_, global_plan_pub_);
    return true;
  }

  bool PoseFollower::isGoalReached(){
    return goal_reached_time_ + ros::Duration(tolerance_timeout_) < ros::Time::now() && stopped();
  }

  geometry_msgs::Twist PoseFollower::diff2D(const geometry_msgs::Pose& pose1_msg,
                                            const geometry_msgs::Pose& pose2_msg)
  {
    tf2::Transform pose1, pose2;
    tf2::convert(pose1_msg, pose1);
    tf2::convert(pose2_msg, pose2);
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
    if (allow_backwards_) 
    {
      double neg_yaw_diff = headingDiff(pose1.getOrigin().x(), pose1.getOrigin().y(), 
					pose2.getOrigin().x(), pose2.getOrigin().y(), M_PI + tf2::getYaw(pose2.getRotation()));

      //check if its faster to just back up
      if(fabs(neg_yaw_diff) < fabs(yaw_diff)){
        ROS_DEBUG("Negative is better: %.2f", neg_yaw_diff);
        yaw_diff = neg_yaw_diff;
      }
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


  geometry_msgs::Twist PoseFollower::limitTwist(const geometry_msgs::Twist& twist)
  {
    geometry_msgs::Twist res = twist;
    res.linear.x *= K_trans_;
    if(!holonomic_)
      res.linear.y = 0.0;
    else    
      res.linear.y *= K_trans_;
    res.angular.z *= K_rot_;

    //if turn_in_place_first is true, see if we need to rotate in place to face our goal first
    if (turn_in_place_first_ && fabs(twist.angular.z) > max_heading_diff_before_moving_)
    {
      res.linear.x = 0;
      res.linear.y = 0;
      if (fabs(res.angular.z) > max_vel_th_) res.angular.z = max_vel_th_ * sign(res.angular.z);
      if (fabs(res.angular.z) < min_in_place_vel_th_) res.angular.z = min_in_place_vel_th_ * sign(res.angular.z);
      return res;
    }

    //make sure to bound things by our velocity limits
    double lin_overshoot = sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y) / max_vel_lin_;
    double lin_undershoot = min_vel_lin_ / sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y);
    if (lin_overshoot > 1.0) 
    {
      res.linear.x /= lin_overshoot;
      res.linear.y /= lin_overshoot;
    }

    //we only want to enforce a minimum velocity if we're not rotating in place
    if(lin_undershoot > 1.0)
    {
      res.linear.x *= lin_undershoot;
      res.linear.y *= lin_undershoot;
    }

    if (fabs(res.angular.z) > max_vel_th_) res.angular.z = max_vel_th_ * sign(res.angular.z);
    if (fabs(res.angular.z) < min_vel_th_) res.angular.z = min_vel_th_ * sign(res.angular.z);
    if (std::isnan(res.linear.x))
        res.linear.x = 0.0;
    if (std::isnan(res.linear.y))
        res.linear.y = 0.0;

    //we want to check for whether or not we're desired to rotate in place
    if(sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y) < in_place_trans_vel_){
      if (fabs(res.angular.z) < min_in_place_vel_th_) res.angular.z = min_in_place_vel_th_ * sign(res.angular.z);
      res.linear.x = 0.0;
      res.linear.y = 0.0;
    }

    ROS_DEBUG("Angular command %f", res.angular.z);
    return res;
  }

  bool PoseFollower::transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
      std::vector<geometry_msgs::PoseStamped>& transformed_plan){
    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

    transformed_plan.clear();

    try{
      if (global_plan.empty())
      {
        ROS_ERROR("Recieved plan with zero length");
        return false;
      }

      geometry_msgs::TransformStamped transform;
      transform = tf.lookupTransform(global_frame, ros::Time(),
                                     plan_pose.header.frame_id, plan_pose.header.stamp,
                                     plan_pose.header.frame_id);
      tf2::Stamped<tf2::Transform> tf_transform;
      tf2::convert(transform, tf_transform);

      tf2::Stamped<tf2::Transform> tf_pose;
      geometry_msgs::PoseStamped newer_pose;
      //now we'll transform until points are outside of our distance threshold
      for(unsigned int i = 0; i < global_plan.size(); ++i){
        const geometry_msgs::PoseStamped& pose = global_plan[i];
        tf2::convert(pose, tf_pose);
        tf_pose.setData(tf_transform * tf_pose);
        tf_pose.stamp_ = tf_transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        tf2::toMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);
      }
    }
    catch(tf2::LookupException& ex) {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch(tf2::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch(tf2::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (!global_plan.empty())
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }

    return true;
  }
};
