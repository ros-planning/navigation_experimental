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
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    ros::NodeHandle pn("~/" + name);

    double acc_lim_x, acc_lim_y, acc_lim_th;
    pn.param("acc_lim_x", acc_lim_x, 2.5);
    pn.param("acc_lim_y", acc_lim_y, 2.5);
    pn.param("acc_lim_th", acc_lim_th, 3.2);

    acc_lim_[0] = acc_lim_x;
    acc_lim_[1] = acc_lim_y;
    acc_lim_[2] = acc_lim_th;

    pn.param("max_vel_x", max_vel_x_, 0.5);
    pn.param("min_vel_x", min_vel_x_, 0.1);

    pn.param("max_vel_y", max_vel_x_, 0.2);
    pn.param("min_vel_y", min_vel_x_, -0.2);
    pn.param("min_in_place_strafe_vel", min_in_place_vel_y_, 0.1);

    pn.param("max_rotational_vel", max_vel_th_, 1.0);
    min_vel_th_ = -1.0 * max_vel_th_;

    pn.param("min_in_place_rotational_vel", min_in_place_vel_th_, 0.4);

    pn.param("sim_time", sim_time_, 1.0);
    pn.param("sim_granularity", sim_granularity_, 0.025);
    pn.param("path_distance_bias", pdist_scale_, 0.6);
    pn.param("goal_distance_bias", gdist_scale_, 0.8);
    pn.param("occdist_scale", occdist_scale_, 0.01);

    pn.param("stop_time_buffer", stop_time_buffer_, 0.2);

    int vx_samp, vy_samp, vth_samp;
    pn.param("vx_samples", vx_samp, 3);
    pn.param("vy_samples", vy_samp, 3);
    pn.param("vth_samples", vth_samp, 20);

    if(vx_samp <= 0){
      ROS_WARN("You've specified that you don't want to sample in the x dimension. We'll at least assume that you want to sample zero... so we're going to set vx_samples to 1 instead");
      vx_samp = 1;
    }

    if(vy_samp <= 0){
      ROS_WARN("You've specified that you don't want to sample in the y dimension. We'll at least assume that you want to sample zero... so we're going to set vy_samples to 1 instead");
      vy_samp = 1;
    }

    if(vth_samp <= 0){
      ROS_WARN("You've specified that you don't want to sample in the th dimension. We'll at least assume that you want to sample zero... so we're going to set vth_samples to 1 instead");
      vth_samp = 1;
    }

    pn.param("sim_period", sim_period_, 0.1);

    vsamples_[0] = vx_samp;
    vsamples_[1] = vy_samp;
    vsamples_[2] = vth_samp;

    footprint_spec_ = costmap_ros_->getRobotFootprint();

  }

  Eigen::Vector3f DWAPlanner::computeNewPositions(const Eigen::Vector3f& pos, 
      const Eigen::Vector3f& vel, double dt){
    Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
    new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
    new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
    new_pos[2] = pos[2] + vel[2] * dt;
    return new_pos;
  }
  
  void DWAPlanner::selectBestTrajectory(base_local_planner::Trajectory* best, base_local_planner::Trajectory* comp){
    //check if the comp trajectory is better than the current best and, if so, swap them
    if(comp->cost_ >= 0 && (comp->cost_ < best->cost_ || best->cost_ < 0)){
      base_local_planner::Trajectory* swap = best;
      best = comp;
      comp = swap;
    }
  }

  void DWAPlanner::computeTrajectories(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel){
    //compute the feasible velocity space based on the rate at which we run
    Eigen::Vector3f max_vel = Eigen::Vector3f::Zero();
    max_vel[0] = std::min(max_vel_x_, vel[0] + acc_lim_[0] * sim_period_);
    max_vel[1] = std::min(max_vel_y_, vel[1] + acc_lim_[1] * sim_period_);
    max_vel[2] = std::min(max_vel_th_, vel[2] + acc_lim_[2] * sim_period_);

    Eigen::Vector3f min_vel = Eigen::Vector3f::Zero();
    min_vel[0] = std::max(min_vel_x_, vel[0] - acc_lim_[0] * sim_period_);
    min_vel[1] = std::max(min_vel_y_, vel[1] - acc_lim_[1] * sim_period_);
    min_vel[2] = std::max(min_vel_th_, vel[2] - acc_lim_[2] * sim_period_);

    //we want to sample the velocity space regularly
    Eigen::Vector3f dv = (max_vel - min_vel).cwise() / vsamples_;

    //keep track of the best trajectory seen so far... we'll re-use two memeber vars for efficiency
    base_local_planner::Trajectory* best_traj = &traj_one_;
    best_traj->cost_ = 1.0;

    base_local_planner::Trajectory* comp_traj = &traj_two_;
    comp_traj->cost_ = 1.0;

    Eigen::Vector3f vel_samp = Eigen::Vector3f::Zero();

    //first... we'll make sure to simulate the case where y and th are zero
    vel_samp[0] = min_vel_x_;
    for(unsigned int i = 0; i < vsamples_[0]; ++i){
      //simulate the trajectory and select it if its better
      generateTrajectory(pos, vel, *comp_traj);
      selectBestTrajectory(best_traj, comp_traj);
      vel_samp[0] += dv[0];
    }

    vel_samp[0] = min_vel_x_;
    //Next, we'll just loop through the trajectories as normal
    for(unsigned int i = 0; i < vsamples_[0]; ++i){
      //we only want to set the y velocity if we're sampling more than one point
      if(vsamples_[1] > 1)
        vel_samp[1] = min_vel_y_;
      else
        vel_samp[1] = 0.0;

      for(unsigned int j = 0; j < vsamples_[1]; ++j){
        //we only want to set the th velocity if we're sampling more than one point
        if(vsamples_[2] > 1)
          vel_samp[2] = min_vel_th_;
        else
          vel_samp[2] = 0.0;

        for(unsigned int k = 0; k < vsamples_[2]; ++k){
          //simulate the trajectory and select it if its better
          generateTrajectory(pos, vel, *comp_traj);
          selectBestTrajectory(best_traj, comp_traj);
          vel_samp[2] += dv[2];
        }
        vel_samp[1] += dv[1];
      }
      vel_samp[0] += dv[0];
    }

    //we want to make sure to simulate strafing without motion in the x direction
    if(vsamples_[1] > 1){
      vel_samp[0] = 0.0;
      vel_samp[1] = min_vel_y_;
      vel_samp[2] = 0.0;

      for(unsigned int i = 0; i < vsamples_[1]; ++i){
        //we won't sample trajectories that don't meet the minimum speed requirements
        if(fabs(vel_samp[1]) < min_in_place_vel_y_){
          vel_samp[1] += dv[1];
          continue;
        }

        //TODO:simulate make sure to check for oscillation
      }
    }

    //and we have the special case where we want to simulate in-place rotations
    if(vsamples_[2] > 1){
      vel_samp[0] = 0.0;
      vel_samp[1] = 0.0;
      vel_samp[2] = min_vel_th_;

      for(unsigned int i = 0; i < vsamples_[2]; ++i){
        //we won't sample trajectories that don't meet the minimum speed requirements
        if(fabs(vel_samp[2]) < min_in_place_vel_th_){
          vel_samp[2] += dv[2];
          continue;
        }

        //TODO: simulate and make sure to differentiate between rotation scores and check oscillation
      }
    }

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

    double vmag = sqrt(vel[0] * vel[0] + vel[1] * vel[1]);

    //compute the number of steps we must take along this trajectory to be "safe"

    //compute the number of steps we'll take
    int num_steps = ceil(std::max((vmag * sim_time_) / sim_granularity_, fabs(vel[2]) / sim_granularity_));

    //compute a timestep
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
        if(abs_vel[0] < max_vel[0] && abs_vel[1] < max_vel[1] && abs_vel[2] < max_vel[2]){
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
