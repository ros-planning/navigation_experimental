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
#ifndef DWA_LOCAL_PLANNER_DWA_PLANNER_H_
#define DWA_LOCAL_PLANNER_DWA_PLANNER_H_
#include <queue>
#include <vector>
#include <Eigen/Core>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/map_grid.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <dwa_local_planner/velocity_iterator.h>

#include <dynamic_reconfigure/server.h>
#include <dwa_local_planner/DWAPlannerConfig.h>

namespace dwa_local_planner {
  class DWAPlanner {
    public:
      DWAPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      ~DWAPlanner() {delete world_model_;}

      Eigen::Vector3f computeNewPositions(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, double dt);
      void generateTrajectory(Eigen::Vector3f pos, const Eigen::Vector3f& vel, base_local_planner::Trajectory& traj, bool two_point_scoring);
      base_local_planner::Trajectory computeTrajectories(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel);
      bool checkTrajectory(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel);
      base_local_planner::Trajectory findBestPath(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel, 
          tf::Stamped<tf::Pose>& drive_velocities);
      void updatePlan(const std::vector<geometry_msgs::PoseStamped>& new_plan);
      Eigen::Vector3f getAccLimits() { return acc_lim_; }
      double getSimPeriod() { return sim_period_; }
      

    private:
      void reconfigureCB(DWAPlannerConfig &config, uint32_t level);
      double footprintCost(const Eigen::Vector3f& pos, double scale);
      void selectBestTrajectory(base_local_planner::Trajectory* &best, base_local_planner::Trajectory* &comp);
      void resetOscillationFlags();
      void resetOscillationFlagsIfPossible(const Eigen::Vector3f& pos, const Eigen::Vector3f& prev);
      bool setOscillationFlags(base_local_planner::Trajectory* t);
      double headingDiff(double gx, double gy, const Eigen::Vector3f& pos);

      inline double squareDist(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
        return (p1.pose.position.x - p2.pose.position.x) * (p1.pose.position.x - p2.pose.position.x)
          + (p1.pose.position.y - p2.pose.position.y) * (p1.pose.position.y - p2.pose.position.y);
      }

      inline Eigen::Vector3f getMaxSpeedToStopInTime(double time){
        return acc_lim_ * std::max(time, 0.0);
      }

      inline double getStopTime(const Eigen::Vector3f& vel){
        Eigen::Vector3f stop_vec = -2.0 * (vel.cwise() / acc_lim_);
        double max_time = stop_vec[0];
        for(unsigned int i = 1; i < 3; ++i){
          max_time = std::max(max_time, (double)stop_vec[i]);
        }
        return max_time;
      }

      inline double yFromElipse(double a, double b, double x){
        double y_squared = (1.0 - (x * x) / (a * a)) * (b * b);
        if(y_squared < 0.0)
          return 0.0;
        return sqrt(y_squared);
      }

      double sign(double x){
        return x < 0.0 ? -1.0 : 1.0;
      }

      int getHeadingLookaheadIndex(double dist, const Eigen::Vector3f& pos);
      bool oscillationCheck(const Eigen::Vector3f& vel);

      base_local_planner::MapGrid map_, front_map_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D costmap_;
      double stop_time_buffer_;
      double pdist_scale_, gdist_scale_, occdist_scale_, heading_scale_;
      Eigen::Vector3f acc_lim_, vsamples_, prev_stationary_pos_;
      std::vector<geometry_msgs::Point> footprint_spec_;
      base_local_planner::CostmapModel* world_model_;
      double sim_time_, sim_granularity_;
      double max_vel_x_, min_vel_x_;
      double max_vel_y_, min_vel_y_, min_vel_trans_, max_vel_trans_;
      double max_vel_th_, min_vel_th_, min_rot_vel_;
      double sim_period_;
      base_local_planner::Trajectory traj_one_, traj_two_;
      bool strafe_pos_only_, strafe_neg_only_, strafing_pos_, strafing_neg_;
      bool rot_pos_only_, rot_neg_only_, rotating_pos_, rotating_neg_;
      bool forward_pos_only_, forward_neg_only_, forward_pos_, forward_neg_;
      double oscillation_reset_dist_;
      double heading_lookahead_, forward_point_distance_;
      double scaling_speed_, max_scaling_factor_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      dynamic_reconfigure::Server<DWAPlannerConfig> dsrv_;
      boost::mutex configuration_mutex_;
      bool penalize_negative_x_;
  };
};
#endif
