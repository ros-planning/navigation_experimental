/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
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
* Author: Christian Connette
*********************************************************************/

#ifndef EBAND_TRAJECTORY_CONTROLLER_H_
#define EBAND_TRAJECTORY_CONTROLLER_H_

#include <ros/ros.h>
#include <ros/assert.h>

// classes which are part of this package
#include <eband_local_planner/conversions_and_types.h>
#include <eband_local_planner/eband_visualization.h>

// geometry_msg
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

// nav_msgs
#include <nav_msgs/Odometry.h>

// geometry
#include <angles/angles.h>
#include <tf/tf.h>

// PID control library
#include <control_toolbox/pid.h>


namespace eband_local_planner{


/**
 * @class EBandPlanner
 * @brief Implements the Elastic Band Method for SE2-Manifold (mobile Base)
 */
class EBandTrajectoryCtrl{

	public:

		/**
		 * @brief Default constructor
		 */
		EBandTrajectoryCtrl();

		/**
		 * @brief Constructs the elastic band object
		 * @param name The name to give this instance of the elastic band local planner
		 * @param costmap The cost map to use for assigning costs to trajectories
		 */
		EBandTrajectoryCtrl(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

		/**
		 * @brief  Destructor
		 */
		~EBandTrajectoryCtrl();

		/**
		 * @brief Initializes the elastic band class by accesing costmap and loading parameters
		 * @param name The name to give this instance of the trajectory planner
		 * @param costmap The cost map to use for assigning costs to trajectories
		 */
		void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

		/**
		 * @brief passes a reference to the eband visualization object which can be used to visualize the band optimization
		 * @param pointer to visualization object
		 */
		void setVisualization(boost::shared_ptr<EBandVisualization> target_visual);

		/**
		 * @brief This sets the elastic_band to the trajectory controller
		 * @param reference via which the band is passed
		 */
		bool setBand(const std::vector<Bubble>& elastic_band);

		/**
		 * @brief This sets the robot velocity as provided by odometry
		 * @param reference via which odometry is passed
		 */
		bool setOdometry(const nav_msgs::Odometry& odometry);

		/**
		 * @brief calculates a twist feedforward command from the current band
		 * @param refernce to the twist cmd
		 */
		bool getTwist(geometry_msgs::Twist& twist_cmd);


	private:

		// pointer to external objects (do NOT delete object)
		costmap_2d::Costmap2DROS* costmap_ros_; ///<@brief pointer to costmap
		boost::shared_ptr<EBandVisualization> target_visual_; // pointer to visualization object

    control_toolbox::Pid pid_;

		// parameters
                double k_p_, k_nu_, ctrl_freq_;
		double acc_max_, virt_mass_;
		double max_vel_lin_, max_vel_th_, min_vel_lin_, min_vel_th_;
		double min_in_place_vel_th_;
		double in_place_trans_vel_;
		double tolerance_trans_, tolerance_rot_, tolerance_timeout_;
		double acc_max_trans_, acc_max_rot_;
                double rotation_correction_threshold_; // We'll do rotation correction if we're at least this far from the goal

		// flags
                bool initialized_, band_set_, visualization_;

		// data
		std::vector<Bubble> elastic_band_;
		geometry_msgs::Twist odom_vel_;
		geometry_msgs::Twist last_vel_;
		geometry_msgs::Pose ref_frame_band_;

		///@brief defines sign of a double
		inline double sign(double n)
		{
			return n < 0.0 ? -1.0 : 1.0;
		}

		/**
		 * @brief Transforms Pose of frame 1 and 2 into reference frame and gets difference of frame 1 and 2
		 * @param refernce to pose of frame1
		 * @param reference to pose of frame2
		 * @param reference to pose of reference frame
		 * @return vector from frame1 to frame2 in coordinates of the reference frame 
		 */
		geometry_msgs::Twist getFrame1ToFrame2InRefFrame(const geometry_msgs::Pose& frame1, const geometry_msgs::Pose& frame2, const geometry_msgs::Pose& ref_frame);
		
		/**
		 * @param Transforms twist into a given reference frame
		 * @param Twist that shall be transformed
		 * @param refernce to pose of frame1
		 * @param reference to pose of frame2
		 * @return transformed twist
		 */
		geometry_msgs::Twist transformTwistFromFrame1ToFrame2(const geometry_msgs::Twist& curr_twist, const geometry_msgs::Pose& frame1,
																const geometry_msgs::Pose& frame2);

		/**
		 * @brief limits the twist to the allowed range
		 * @param reference to unconstrained twist
		 * @return twist in allowed range
		 */
		geometry_msgs::Twist limitTwist(const geometry_msgs::Twist& twist);

		/**
		 * @brief gets the max velocity allowed within this bubble depending on size of the bubble and pose and size of the following bubble
		 * @param number of the bubble of interest within the band
		 * @param band in which the bubble is in
		 * @return absolute value of maximum allowed velocity within this bubble
		 */
		double getBubbleTargetVel(const int& target_bub_num, const std::vector<Bubble>& band, geometry_msgs::Twist& VelDir);

};
};
#endif






