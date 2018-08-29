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
 * Recovery behavior based on executing a particular twist
 *
 * \author Bhaskara Marthi
 */

#ifndef TWIST_RECOVERY_TWIST_RECOVERY_H
#define TWIST_RECOVERY_TWIST_RECOVERY_H

#include <nav_core/recovery_behavior.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose2D.h>

namespace twist_recovery
{

/// Recovery behavior that takes a given twist and tries to execute it for up to
/// d seconds, or until reaching an obstacle.  
class TwistRecovery : public nav_core::RecoveryBehavior
{
public:
  
  /// Doesn't do anything: initialize is where the actual work happens
  TwistRecovery();

  ~TwistRecovery();

  /// Initialize the parameters of the behavior
  void initialize (std::string n, tf::TransformListener* tf,
                   costmap_2d::Costmap2DROS* global_costmap,
                   costmap_2d::Costmap2DROS* local_costmap);

  /// Run the behavior
  void runBehavior();

private:

  geometry_msgs::Pose2D getCurrentLocalPose () const;
  geometry_msgs::Twist scaleGivenAccelerationLimits (const geometry_msgs::Twist& twist, const double time_remaining) const;
  double nonincreasingCostInterval (const geometry_msgs::Pose2D& current, const geometry_msgs::Twist& twist) const;
  double normalizedPoseCost (const geometry_msgs::Pose2D& pose) const;
  geometry_msgs::Twist transformTwist (const geometry_msgs::Pose2D& pose) const;

  ros::NodeHandle nh_;
  costmap_2d::Costmap2DROS* global_costmap_;
  costmap_2d::Costmap2DROS* local_costmap_;
  std::string name_;
  tf::TransformListener* tf_;
  ros::Publisher pub_;
  bool initialized_;

  // Memory owned by this object
  // Mutable because footprintCost is not declared const
  mutable base_local_planner::CostmapModel* world_model_;

  geometry_msgs::Twist base_frame_twist_;
  
  double duration_;
  double linear_speed_limit_;
  double angular_speed_limit_;
  double linear_acceleration_limit_;
  double angular_acceleration_limit_;
  double controller_frequency_;
  double simulation_inc_;
  
  
};

} // namespace twist_recovery

#endif // include guard
