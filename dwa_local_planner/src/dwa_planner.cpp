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
#include <dwa_local_planner/dwa_planner.h>

namespace dwa_local_planner {
  void DWAPlanner::initialize(std::string name, tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros){
  }

  Eigen::Vector3f DWAPlanner::computeNewPositions(const Eigen::Vector3f& pos, 
      const Eigen::Vector3f& vel, double dt){
    Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
    new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
    new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
    new_pos[2] = pos[2] + vel[2] * dt;
    return new_pos;
  }

  double DWAPlanner::footprintCost(const Eigen::Vector3f& pos, double scale){
    double cos_th = cos(pos[2]);
    double sin_th = sin(pos[2]);

    vector<geometry_msgs::Point> scaled_oriented_footprint;
    for(unsigned int i  = 0; i < footprint_spec_.size(); ++i){
      geometry_msgs::Point new_pt;
      new_pt.x = x_i + (scale * footprint_spec_[i].x * cos_th - scale * footprint_spec_[i].y * sin_th);
      new_pt.y = y_i + (scale * footprint_spec_[i].x * sin_th + scale * footprint_spec_[i].y * cos_th);
      scaled_oriented_footprint.push_back(new_pt);
    }

    geometry_msgs::Point robot_position;
    robot_position.x = pos[0];
    robot_position.y = pos[1];

    //check if the footprint is legal
    double footprint_cost = world_model_.footprintCost(robot_position, scaled_oriented_footprint, inscribed_radius_, circumscribed_radius_);

    return footprint_cost;
  }

  double trajectoryCost(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, base_local_planner::Trajectory& traj){
    int num_steps = ceil(sim_time_ / sim_granularity);
    double dt = sim_sime_ / num_steps_;

    double path_dist = 0.0;
    double goal_dist = 0.0;
    double occ_cost = 0.0;

    for(int i = 0; i < num_steps; ++i){
      //get the mapp coordinates of a point
      unsigned int cell_x, cell_y;

      if(!costmap_.worldToMap(pos[0], pos[1], cell_x, cell_y)){
        //we're off the map
        traj.cost_ = -1.0;
        return;
      }

      //we want to find the cost of the footprint
      double footprint_cost = footprintCost(pos[0], pos[1], pos[2], scale);

      //if the footprint hits an obstacle... the trajectory is invalid
      if(footprint_cost < 0){
        traj.cost_ = -1.0;
        return;
      }

      occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_.getCost(cell_x, cell_y)));
      double cell_pdist = map_(cell_x, cell_y).path_dist;
      double cell_gdist = map_(cell_x, cell_y).goal_dist;

      path_dist = cell_pdist;
      goal_dist = cell_gdist;

      //if a point on this trajectory has no clear path to the goal... it is invalid
      if(impossible_cost <= goal_dist || impossible_cost <= path_dist){
        cost = -2.0;
      }

    }

  }
};
