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
#include <ros/ros.h>

namespace dwa_local_planner {
  class DWAPlanner {
    public:
      DWAPlanner() : map_(10, 10){}

      ~DWAPlanner() {}

      void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      Eigen::Vector3f computeNewPositions(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, double dt);
      void generateTrajectory(Eigen::Vector3f pos, const Eigen::Vector3f& vel, base_local_planner::Trajectory& traj);
      void computeTrajectories(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel);
      

    private:
      //void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

      double footprintCost(const Eigen::Vector3f& pos, double scale);
      void selectBestTrajectory(base_local_planner::Trajectory* best, base_local_planner::Trajectory* comp);

      inline Eigen::Vector3f getMaxSpeedToStopInTime(double time){
        return -0.5 * acc_lim_ * std::max(time, 0.0);
      }

      base_local_planner::MapGrid map_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D costmap_;
      tf::TransformListener* tf_;
      double stop_time_buffer_;
      double pdist_scale_, gdist_scale_, occdist_scale_;
      Eigen::Vector3f acc_lim_, vsamples_;
      std::vector<geometry_msgs::Point> footprint_spec_;
      base_local_planner::CostmapModel* world_model_;
      double sim_time_, sim_granularity_;
      double max_vel_x_, min_vel_x_;
      double max_vel_y_, min_vel_y_, min_in_place_vel_y_;
      double max_vel_th_, min_vel_th_, min_in_place_vel_th_;
      double sim_period_;
      base_local_planner::Trajectory traj_one_, traj_two_;
  };
};
#endif
