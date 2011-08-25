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


#include <eband_local_planner/eband_visualization.h>


namespace eband_local_planner{


EBandVisualization::EBandVisualization() : initialized_(false) {}


EBandVisualization::EBandVisualization(ros::NodeHandle& pn, costmap_2d::Costmap2DROS* costmap_ros)
{
	initialize(pn, costmap_ros);
}


EBandVisualization::~EBandVisualization() {}


void EBandVisualization::initialize(ros::NodeHandle& pn, costmap_2d::Costmap2DROS* costmap_ros)
{
	// check if visualization already initialized
	if(!initialized_)
	{
		// read parameters from parameter server
		pn.param("marker_lifetime", marker_lifetime_, 0.5);

		// advertise topics
		one_bubble_pub_ = pn.advertise<visualization_msgs::Marker>("eband_visualization", 1);
		// although we want to publish MarkerArrays we have to advertise Marker topic first -> rviz searchs relative to this
		bubble_pub_ = pn.advertise<visualization_msgs::MarkerArray>("eband_visualization_array", 1);

		// copy adress of costmap and Transform Listener (handed over from move_base) -> to get robot shape
		costmap_ros_ = costmap_ros;

		initialized_ = true;
	}
	else
	{
		ROS_WARN("Trying to initialize already initialized visualization, doing nothing.");
	}
}


void EBandVisualization::publishBand(std::string marker_name_space, std::vector<Bubble> band)
{
	// check if visualization was initialized
	if(!initialized_)
	{
		ROS_ERROR("Visualization not yet initialized, please call initialize() before using visualization");
		return;
	}

	visualization_msgs::MarkerArray eband_msg;
	eband_msg.markers.resize(band.size());

	visualization_msgs::MarkerArray eband_heading_msg;
	eband_heading_msg.markers.resize(band.size());
	std::string marker_heading_name_space = marker_name_space;
	marker_heading_name_space.append("_heading");

	// convert elastic band to msg
	for(int i = 0; i < ((int) band.size()); i++)
	{
		// convert bubbles in eband to marker msg
		bubbleToMarker(band[i], eband_msg.markers[i], marker_name_space, i, green);

		// convert bubbles in eband to marker msg
		bubbleHeadingToMarker(band[i], eband_heading_msg.markers[i], marker_heading_name_space, i, green);
	}

	// publish
	bubble_pub_.publish(eband_msg);
	bubble_pub_.publish(eband_heading_msg);
}


void EBandVisualization::publishBubble(std::string marker_name_space, int marker_id, Bubble bubble)
{
	// check if visualization was initialized
	if(!initialized_)
	{
		ROS_ERROR("Visualization not yet initialized, please call initialize() before using visualization");
		return;
	}

	visualization_msgs::Marker bubble_msg;

	// convert bubble to marker msg
	bubbleToMarker(bubble, bubble_msg, marker_name_space, marker_id, green);

	// publish
	one_bubble_pub_.publish(bubble_msg);
}


void EBandVisualization::publishBubble(std::string marker_name_space, int marker_id, Color marker_color, Bubble bubble)
{
	// check if visualization was initialized
	if(!initialized_)
	{
		ROS_ERROR("Visualization not yet initialized, please call initialize() before using visualization");
		return;
	}

	visualization_msgs::Marker bubble_msg;

	// convert bubble to marker msg
	bubbleToMarker(bubble, bubble_msg, marker_name_space, marker_id, marker_color);

	// publish
	one_bubble_pub_.publish(bubble_msg);
}


void EBandVisualization::publishForceList(std::string marker_name_space, std::vector<geometry_msgs::WrenchStamped> forces, std::vector<Bubble> band)
{
	// check if visualization was initialized
	if(!initialized_)
	{
		ROS_ERROR("Visualization not yet initialized, please call initialize() before using visualization");
		return;
	}

	visualization_msgs::MarkerArray forces_msg;
	forces_msg.markers.resize(forces.size());

	//before converting to msg - check whether internal, external or resulting forces - switch color
	Color marker_color = green;
	if(marker_name_space.compare("internal_forces") == 0)
		marker_color = blue;

	if(marker_name_space.compare("external_forces") == 0)
		marker_color = red;

	if(marker_name_space.compare("resulting_forces") == 0)
		marker_color = green;

	for(int i = 0; i < ((int) forces.size()); i++)
	{
		// convert wrenches in force list into marker msg
		forceToMarker(forces[i], band[i].center.pose, forces_msg.markers[i], marker_name_space, i, marker_color);
	}

	// publish
	bubble_pub_.publish(forces_msg);
}

void EBandVisualization::publishForce(std::string marker_name_space, int id, Color marker_color, geometry_msgs::WrenchStamped force, Bubble bubble)
{
	// check if visualization was initialized
	if(!initialized_)
	{
		ROS_ERROR("Visualization not yet initialized, please call initialize() before using visualization");
		return;
	}

	visualization_msgs::Marker force_msg;

	// convert wrenches in force list into marker msg
	forceToMarker(force, bubble.center.pose, force_msg, marker_name_space, id, marker_color);

	// publish
	one_bubble_pub_.publish(force_msg);
}


void EBandVisualization::bubbleToMarker(Bubble bubble, visualization_msgs::Marker& marker, std::string marker_name_space, int marker_id, Color marker_color)
{
	geometry_msgs::Pose2D tmp_pose2d;

	// header
	marker.header.stamp = ros::Time::now();
	marker.header.frame_id = bubble.center.header.frame_id;

	// identifier and cmds
	marker.ns = marker_name_space;
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;

	// body
	marker.pose = bubble.center.pose;
	// get theta-angle to display as elevation
	PoseToPose2D(bubble.center.pose, tmp_pose2d);
	marker.pose.position.z = tmp_pose2d.theta * costmap_ros_->getCircumscribedRadius();
	// scale ~ diameter --> is 2x expansion ~ radius
	marker.scale.x = 2.0*bubble.expansion;
	marker.scale.y = 2.0*bubble.expansion;
	marker.scale.z = 2.0*bubble.expansion;

	// color (rgb)
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	switch(marker_color)
	{
		case red:	{ marker.color.r = 1.0f; break; }
		case green:	{ marker.color.g = 1.0f; break; }
		case blue:	{ marker.color.b = 1.0f; break; }
	}
	// transparency (alpha value < 1 : displays marker transparent)
	marker.color.a = 0.25;

	// lifetime of this marker
	marker.lifetime = ros::Duration(marker_lifetime_);
}


void EBandVisualization::bubbleHeadingToMarker(Bubble bubble, visualization_msgs::Marker& marker, std::string marker_name_space, int marker_id, Color marker_color)
{
	geometry_msgs::Pose2D tmp_pose2d;

	// header
	marker.header.stamp = ros::Time::now();
	marker.header.frame_id = bubble.center.header.frame_id;

	// identifier and cmds
	marker.ns = marker_name_space;
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;

	// body
	marker.pose = bubble.center.pose;
	// get theta-angle to display as elevation
	PoseToPose2D(bubble.center.pose, tmp_pose2d);
	marker.pose.position.z = tmp_pose2d.theta * costmap_ros_->getCircumscribedRadius();
	// scale ~ diameter --> is 2x expansion ~ radius
	marker.scale.x = 0.9;
	marker.scale.y = 0.45;
	marker.scale.z = 0.45;

	// color (rgb)
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	switch(marker_color)
	{
		case red:	{ marker.color.r = 1.0f; break; }
		case green:	{ marker.color.g = 1.0f; break; }
		case blue:	{ marker.color.b = 1.0f; break; }
	}
	// transparency (alpha value < 1 : displays marker transparent)
	marker.color.a = 1.0;

	// lifetime of this marker
	marker.lifetime = ros::Duration(marker_lifetime_);
}


void EBandVisualization::forceToMarker(geometry_msgs::WrenchStamped wrench, geometry_msgs::Pose wrench_origin, visualization_msgs::Marker& marker, std::string marker_name_space, int marker_id, Color marker_color)
{
	geometry_msgs::Pose2D tmp_pose2d;

	// header
	marker.header.stamp = ros::Time::now();
	marker.header.frame_id = wrench.header.frame_id;

	// identifier and cmds
	marker.ns = marker_name_space;
	marker.id = marker_id;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;

	// body - origin
	marker.pose.position = wrench_origin.position;
	// get theta-angle to display as elevation
	PoseToPose2D(wrench_origin, tmp_pose2d);
	marker.pose.position.z = tmp_pose2d.theta * costmap_ros_->getCircumscribedRadius();

	// body - orientation of vector (calc. quaternion that transform xAxis into force-vector)
	// check if force different from zero
	if( (wrench.wrench.force.x != 0) || (wrench.wrench.force.y != 0) || (wrench.wrench.torque.z != 0) )
	{
		// find AxisAngle Representation then convert to Quaternion
		// (Axis = normalize(vec1) cross normalize(vec2) // cos(Angle) = normalize(vec1) dot normalize(vec2))
		Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
		Eigen::Vector3d target_vec( wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.torque.z / costmap_ros_->getCircumscribedRadius() );
		Eigen::Vector3d rotation_axis(1.0, 0.0, 0.0);
		double rotation_angle = 0.0;
		// get Axis orthogonal to both vectors ()
		x_axis.normalize(); // unneccessary but just in case
		target_vec.normalize();
		if(!(x_axis == target_vec))
		{
			// vector not identical - cross-product defined
			rotation_axis = x_axis.cross(target_vec);
			rotation_angle = x_axis.dot(target_vec);
			rotation_angle = acos(rotation_angle);
		}
		// create AngleAxis representation
		rotation_axis.normalize(); // normalize vector -> otherwise AngleAxis will be invalid!
                const double s = sin(rotation_angle/2);
                const double c = cos(rotation_angle/2);
                Eigen::Quaterniond rotate_quat(c, s*rotation_axis.x(), s*rotation_axis.y(), s*rotation_axis.z());
	
		// transform quaternion back from Eigen to ROS
		tf::Quaternion orientation_tf;
		geometry_msgs::Quaternion orientation_msg;
		tf::RotationEigenToTF(rotate_quat, orientation_tf);
		tf::quaternionTFToMsg(orientation_tf, orientation_msg);
	
		// finally set orientation of marker
		marker.pose.orientation = orientation_msg;

		// scale ~ diameter --> is 2x expansion ~ radius
		double scale = sqrt( (wrench.wrench.force.x * wrench.wrench.force.x) + (wrench.wrench.force.y * wrench.wrench.force.y) +
						( (wrench.wrench.torque.z/costmap_ros_->getCircumscribedRadius())*(wrench.wrench.torque.z/costmap_ros_->getCircumscribedRadius()) ) );
		marker.scale.x = scale; //1.0;
		marker.scale.y = scale; //1.0;
		marker.scale.z = scale; //1.0;
	
		// color (rgb)
		marker.color.r = 0.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;
		switch(marker_color)
		{
			case red:	{ marker.color.r = 1.0f; break; }
			case green:	{ marker.color.g = 1.0f; break; }
			case blue:	{ marker.color.b = 1.0f; break; }
		}
		// transparency (alpha value < 1 : displays marker transparent)
		marker.color.a = 1.25;
	}
	else
	{
		// force on this bubble is zero -> make marker invisible
		marker.pose.orientation = wrench_origin.orientation; // just take orientation of bubble as dummy-value

		// scale
		marker.scale.x = 0.0;
		marker.scale.y = 0.0;
		marker.scale.z = 0.0;

		// color (rgb)
		marker.color.r = 0.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;

		// transparency (alpha value < 1 : displays marker transparent)
		marker.color.a = 0.0;
	}

	// lifetime of this marker
	marker.lifetime = ros::Duration(marker_lifetime_);
}

};

