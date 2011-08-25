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


#include <eband_local_planner/conversions_and_types.h>


namespace eband_local_planner{


void PoseToPose2D(const geometry_msgs::Pose pose, geometry_msgs::Pose2D& pose2D)
{
	// use tf-pkg to convert angles
    tf::Pose pose_tf;
	
	// convert geometry_msgs::PoseStamped to tf::Pose
	tf::poseMsgToTF(pose, pose_tf);

	// now get Euler-Angles from pose_tf
    double useless_pitch, useless_roll, yaw;
    pose_tf.getBasis().getEulerYPR(yaw, useless_pitch, useless_roll);

	// normalize angle
	yaw = angles::normalize_angle(yaw);

	// and set to pose2D
	pose2D.x = pose.position.x;
	pose2D.y = pose.position.y;
	pose2D.theta = yaw;

	return;
}


void Pose2DToPose(geometry_msgs::Pose& pose, const geometry_msgs::Pose2D pose2D)
{
	// use tf-pkg to convert angles
	tf::Quaternion frame_quat;

	// transform angle from euler-angle to quaternion representation
   	frame_quat = tf::createQuaternionFromYaw(pose2D.theta);

	// set position
	pose.position.x = pose2D.x;
	pose.position.y = pose2D.y;
	pose.position.z = 0.0;

	// set quaternion
	pose.orientation.x = frame_quat.x();
	pose.orientation.y = frame_quat.y();
	pose.orientation.z = frame_quat.z();
	pose.orientation.w = frame_quat.w();

	return;
}


bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan, 
						const costmap_2d::Costmap2DROS& costmap, const std::string& global_frame, 
						std::vector<geometry_msgs::PoseStamped>& transformed_plan, std::vector<int>& start_end_counts)
{
	const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

	// initiate refernce variables
	transformed_plan.clear();

	try
	{
		if (!global_plan.size() > 0)
		{
			ROS_ERROR("Recieved plan with zero length");
			return false;
		}

		tf::StampedTransform transform;
		tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, 
							plan_pose.header.frame_id, transform);

		//let's get the pose of the robot in the frame of the plan
		tf::Stamped<tf::Pose> robot_pose;
		robot_pose.setIdentity();
		robot_pose.frame_id_ = costmap.getBaseFrameID();
		robot_pose.stamp_ = ros::Time();
		tf.transformPose(plan_pose.header.frame_id, robot_pose, robot_pose);

		//we'll keep points on the plan that are within the window that we're looking at
		double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0, costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);

		unsigned int i = 0;
		double sq_dist_threshold = dist_threshold * dist_threshold;
		double sq_dist = DBL_MAX;

		// --- start - modification w.r.t. base_local_planner
		// initiate start_end_count
		std::vector<int> start_end_count;
		start_end_count.assign(2, 0);

		// we know only one direction and that is forward! - initiate search with previous start_end_counts
		// this is neccesserry to work with the sampling based planners - path may severall time enter and leave moving window
		ROS_ASSERT( (start_end_counts.at(0) > 0) && (start_end_counts.at(0) <= (int) global_plan.size()) );
		i = (unsigned int) global_plan.size() - (unsigned int) start_end_counts.at(0);
		// --- end - modification w.r.t. base_local_planner

		//we need to loop to a point on the plan that is within a certain distance of the robot
		while(i < (unsigned int)global_plan.size() && sq_dist > sq_dist_threshold)
		{
			double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
			double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
			sq_dist = x_diff * x_diff + y_diff * y_diff;

			// --- start - modification w.r.t. base_local_planner
			// not yet in reach - get next frame
			if( sq_dist > sq_dist_threshold )
				++i;
			else
				// set counter for start of transformed intervall - from back as beginning of plan might be prunned
				start_end_count.at(0) = (int) (((unsigned int) global_plan.size()) - i);
			// --- end - modification w.r.t. base_local_planner
		}


		tf::Stamped<tf::Pose> tf_pose;
		geometry_msgs::PoseStamped newer_pose;

		//now we'll transform until points are outside of our distance threshold
		while(i < (unsigned int)global_plan.size() && sq_dist < sq_dist_threshold)
		{
			double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
			double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
			sq_dist = x_diff * x_diff + y_diff * y_diff;

			const geometry_msgs::PoseStamped& pose = global_plan[i];
			poseStampedMsgToTF(pose, tf_pose);
			tf_pose.setData(transform * tf_pose);
			tf_pose.stamp_ = transform.stamp_;
			tf_pose.frame_id_ = global_frame;
			poseStampedTFToMsg(tf_pose, newer_pose);

			transformed_plan.push_back(newer_pose);

			// --- start - modification w.r.t. base_local_planner
			// set counter for end of transformed intervall - from back as beginning of plan might be prunned
			start_end_count.at(1) = int (((unsigned int) global_plan.size()) - i);
			// --- end - modification w.r.t. base_local_planner

			++i;
		}

		// --- start - modification w.r.t. base_local_planner
		// write to reference variable
		start_end_counts = start_end_count;
		// --- end - modification w.r.t. base_local_planner
    }
	catch(tf::LookupException& ex)
	{
		ROS_ERROR("No Transform available Error: %s\n", ex.what());
		return false;
	}
	catch(tf::ConnectivityException& ex)
	{
		ROS_ERROR("Connectivity Error: %s\n", ex.what());
		return false;
	}
	catch(tf::ExtrapolationException& ex)
	{
		ROS_ERROR("Extrapolation Error: %s\n", ex.what());
		if (global_plan.size() > 0)
			ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

		return false;
	}

	return true;
}


}
