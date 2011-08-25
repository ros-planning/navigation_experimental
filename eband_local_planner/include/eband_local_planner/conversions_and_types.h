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

#ifndef EBAND_CONVERSIONS_AND_TYPES_H_
#define EBAND_CONVERSIONS_AND_TYPES_H_

#include <ros/ros.h>

// msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>

// transforms
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>


namespace eband_local_planner{


// typedefs

///<@brief defines a bubble - pose of center & radius of according hypersphere (expansion)
struct Bubble
{
	geometry_msgs::PoseStamped center;
	double expansion;
};

enum AddAtPosition {add_front, add_back};


// functions

/**
 * @brief Converts a frame of type Pose to type Pose2D (mainly -> conversion of orientation from quaternions to euler angles)
 * @param Pose which shall be converted
 * @param References to converted ROS Pose2D frmae
 */
void PoseToPose2D(const geometry_msgs::Pose pose, geometry_msgs::Pose2D& pose2D);


/**
 * @brief Converts a frame of type Pose to type Pose2D (mainly -> conversion of orientation from euler angles to quaternions, -> z-coordinate is set to zero)
 * @param References to converted ROS Pose2D frame
 * @param Pose2D which shall be converted
 */
void Pose2DToPose(geometry_msgs::Pose& pose, const geometry_msgs::Pose2D pose2D);


/**
 * @brief  Transforms the global plan of the robot from the planner frame to the local frame. This replaces the transformGlobalPlan as defined in the base_local_planner/goal_functions.h main difference is that it additionally outputs counter indicating which part of the plan has been transformed.
 * @param tf A reference to a transform listener
 * @param global_plan The plan to be transformed
 * @param costmap A reference to the costmap being used so the window size for transforming can be computed
 * @param global_frame The frame to transform the plan to
 * @param transformed_plan Populated with the transformed plan
 * @param number of start and end frame counted from the end of the global plan
 */
bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
							const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame, 
							std::vector<geometry_msgs::PoseStamped>& transformed_plan, std::vector<int>& start_end_counts_from_end);

};
#endif

