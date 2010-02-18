/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
* Author: Sachin Chitta
*********************************************************************/
#ifndef ANTI_COLLISION_BASE_CONTROLLER_H_
#define ANTI_COLLISION_BASE_CONTROLLER_H_

#include <vector>
#include <math.h>
#include <ros/console.h>
#include <angles/angles.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

//we'll take in a path as a vector of poses
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

//for some datatypes
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <assisted_teleop/trajectory.h>

namespace anti_collision_base_controller {
  /**
   * @class AntiCollisionBaseController
   * @brief Computes control velocities for a robot given a costmap, a plan, and the robot's position in the world.
   */
  class AntiCollisionBaseController{
    //    friend class AntiCollisionBaseController; //Need this for gtest to work
    public:
      /**
       * @brief  Constructs an anti-collision base controller
       */
      AntiCollisionBaseController();

      /**
       * @brief  Destructs a trajectory controller
       */
      ~AntiCollisionBaseController();

      tf::TransformListener tf_;

      void joyCallBack(const geometry_msgs::TwistConstPtr& msg);

      void odomCallback(const nav_msgs::OdometryConstPtr& msg);

      void spin();

    private:

      /**
       * @brief  Generate and scor7e a single trajectory
       * @param x The x position of the robot
       * @param y The y position of the robot
       * @param theta The orientation of the robot
       * @param vx The x velocity of the robot
       * @param vy The y velocity of the robot
       * @param vtheta The theta velocity of the robot
       * @param vx_samp The x velocity used to seed the trajectory
       * @param vy_samp The y velocity used to seed the trajectory
       * @param vtheta_samp The theta velocity used to seed the trajectory
       * @param acc_x The x acceleration limit of the robot
       * @param acc_y The y acceleration limit of the robot
       * @param acc_theta The theta acceleration limit of the robot
       * @param traj Will be set to the generated trajectory with its associated score
       * @param sim_time_local The time to simulate forward for
       */
      bool generateTrajectory(double x, double y, double theta,
			      double vx, double vy, double vtheta,
			      double vx_samp, double vy_samp, double vtheta_samp,
			      double acc_x, double acc_y, double acc_theta,
			      base_local_planner::Trajectory& traj, double sim_time_local);
      /**
       * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
       * @param x_i The x position of the robot
       * @param y_i The y position of the robot
       * @param theta_i The orientation of the robot
       * @return
       */
      double footprintCost(double x_i, double y_i, double theta_i);

      costmap_2d::Costmap2D costmap_; ///< @brief Provides access to cost map information

      std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief The footprint specification of the robot

      double inscribed_radius_, circumscribed_radius_, inflation_radius_; ///< @brief The inscribed and circumscribed radii of the robot

      double sim_time_; ///< @brief The number of seconds each trajectory is "rolled-out"
      double sim_granularity_; ///< @brief The distance between simulation points

      int vx_samples_; ///< @brief The number of samples we'll take in the x dimenstion of the control space
      int vtheta_samples_; ///< @brief The number of samples we'll take in the theta dimension of the control space
      double acc_lim_x_, acc_lim_y_, acc_lim_theta_; ///< @brief The acceleration limits of the robot

      double max_vel_x_, min_vel_x_, max_vel_th_, min_vel_th_, min_in_place_vel_th_; ///< @brief Velocity limits for the controller

      double controller_frequency_;

      std::string global_frame_, robot_base_frame_;

      /**
       * @brief  Compute x position based on velocity
       * @param  xi The current x position
       * @param  vx The current x velocity
       * @param  vy The current y velocity
       * @param  theta The current orientation
       * @param  dt The timestep to take
       * @return The new x position
       */
      inline double computeNewXPosition(double xi, double vx, double vy, double theta, double dt){
        return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
      }

      /**
       * @brief  Compute y position based on velocity
       * @param  yi The current y position
       * @param  vx The current x velocity
       * @param  vy The current y velocity
       * @param  theta The current orientation
       * @param  dt The timestep to take
       * @return The new y position
       */
      inline double computeNewYPosition(double yi, double vx, double vy, double theta, double dt){
        return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
      }

      /**
       * @brief  Compute orientation based on velocity
       * @param  thetai The current orientation
       * @param  vth The current theta velocity
       * @param  dt The timestep to take
       * @return The new orientation
       */
      inline double computeNewThetaPosition(double thetai, double vth, double dt){
        return thetai + vth * dt;
      }

      //compute velocity based on acceleration
      /**
       * @brief  Compute velocity based on acceleration
       * @param vg The desired velocity, what we're accelerating up to
       * @param vi The current velocity
       * @param a_max An acceleration limit
       * @param  dt The timestep to take
       * @return The new velocity
       */
      inline double computeNewVelocity(double vg, double vi, double a_max, double dt){
        if(vg >= 0)
          return std::min(vg, vi + a_max * dt);
        return std::max(vg, vi - a_max * dt);
      }

      base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use

      ros::Publisher base_cmd_pub_; ///< @brief A publisher for base commands

      ros::NodeHandle ros_node_; ///< @brief A node handle to the ROS node

      ros::Subscriber joy_sub_; ///< @brief A subscriber for commands from the joystick

      ros::Subscriber odom_sub_; ///< @brief A subscriber for odometry messages

      void computeSafeVelocity(double x, double y, double theta, double vx_current, double vy_current, double vtheta_current, double vx_desired, double vy_desired, double vtheta_desired, double &vx_result, double &vy_result, double &vtheta_result);

      costmap_2d::Costmap2DROS *costmap_ros_;

      geometry_msgs::Twist vel_desired_, base_odom_;
      boost::mutex vel_desired_mutex_, base_odom_mutex_;

      bool getRobotPose(double &x, double &y, double &theta);

      ros::Time last_cmd_received_;

      double timeout_;
      double min_vel_cmd_x_, min_vel_cmd_y_, min_vel_cmd_theta_;

      trajectory::Trajectory *sim_trajectory_;
  };
};

#endif
