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

    std::vector<geometry_msgs::Point> scaled_oriented_footprint;
    for(unsigned int i  = 0; i < footprint_spec_.size(); ++i){
      geometry_msgs::Point new_pt;
      new_pt.x = pos[0] + (scale * footprint_spec_[i].x * cos_th - scale * footprint_spec_[i].y * sin_th);
      new_pt.y = pos[1] + (scale * footprint_spec_[i].x * sin_th + scale * footprint_spec_[i].y * cos_th);
      scaled_oriented_footprint.push_back(new_pt);
    }

    geometry_msgs::Point robot_position;
    robot_position.x = pos[0];
    robot_position.y = pos[1];

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(robot_position, scaled_oriented_footprint, costmap_.getInscribedRadius(), costmap_.getCircumscribedRadius());

    return footprint_cost;
  }

  void DWAPlanner::generateTrajectory(Eigen::Vector3f pos, const Eigen::Vector3f& vel, base_local_planner::Trajectory& traj){
    double impossible_cost = map_.map_.size();

    //compute the number of steps we'll take
    int num_steps = ceil(sim_time_ / sim_granularity_);

    //compute a timestep that is guaranteed to be less than our granularity and will end at the desired time
    double dt = sim_time_ / num_steps;
    double time = 0.0;

    //initialize the costs for the trajectory
    double path_dist = 0.0;
    double goal_dist = 0.0;
    double occ_cost = 0.0;

    //create a potential trajectory... it might be reused so we'll make sure to reset it
    traj.resetPoints();
    traj.xv_ = vel[0];
    traj.yv_ = vel[1];
    traj.thetav_ = vel[2];
    traj.cost_ = -1.0;

    //if we're not actualy going to simulate... we may as well just return now
    if(num_steps == 0)
      return;

    //we want to check against the absolute value of the velocities for collisions later
    Eigen::Vector3f abs_vel = vel.cwise().abs();

    //simulate the trajectory and check for collisions, updating costs along the way
    for(int i = 0; i < num_steps; ++i){
      //get the mapp coordinates of a point
      unsigned int cell_x, cell_y;

      //we won't allow trajectories that go off the map... shouldn't happen that often anyways
      if(!costmap_.worldToMap(pos[0], pos[1], cell_x, cell_y)){
        //we're off the map
        traj.cost_ = -1.0;
        return;
      }

      //TODO:Compute scale properly
      double scale = 1.0;

      //we want to find the cost of the footprint
      double footprint_cost = footprintCost(pos, scale);

      //if the footprint hits an obstacle... we'll check if we can stop before we hit it... given the time to get there
      if(footprint_cost < 0){
        //we want to compute the max allowable speeds to be able to stop
        //to be safe... we'll make sure we can stop some time before we actually hit
        Eigen::Vector3f max_vel = getMaxSpeedToStopInTime(time - stop_time_buffer_);

        //check if we can stop in time
        if(abs_vel[0] <= max_vel[0] && abs_vel[1] < max_vel[1] && abs_vel[2] < max_vel[2]){
          //if we can, then we'll just break out of the loop here.. no point in checking future points
          break;
        }
        else{
          //if we can't... then this trajectory is invalid
          traj.cost_ = -1.0;
          return;
        }
      }

      //compute the costs for this point on the trajectory
      occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_.getCost(cell_x, cell_y)));
      path_dist = map_(cell_x, cell_y).path_dist;
      goal_dist = map_(cell_x, cell_y).goal_dist;

      //if a point on this trajectory has no clear path to the goal... it is invalid
      if(impossible_cost <= goal_dist || impossible_cost <= path_dist){
        traj.cost_ = -2.0; //-2.0 means that we were blocked because propagation failed
        return;
      }

      //add the point to the trajectory so we can draw it later if we want
      traj.addPoint(pos[0], pos[1], pos[2]);

      //update the position of the robot using the velocities passed in
      pos = computeNewPositions(pos, vel, dt);
      time += dt;
    }

    //compute the final cost
    traj.cost_ = pdist_scale_ * path_dist + gdist_scale_ * goal_dist + occdist_scale_ * occ_cost;
  }
};
