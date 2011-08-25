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

#ifndef EBAND_VISUALIZATION_H_
#define EBAND_VISUALIZATION_H_


#include <ros/ros.h>

// classes wich are part of this pkg
#include <eband_local_planner/conversions_and_types.h>

// msgs
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// transforms
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>

// Eigen library for geometric operation
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace eband_local_planner{

/**
 * @class ConversionsAndTypes
 * @brief Implements type-convrsions and types used by elastic-band optimizer and according ros-wrapper class
 */
class EBandVisualization{

	public:

		// typedefs
		enum Color {blue, red, green};


		// methods

		/**
		 * @brief Default constructor
		 */
		EBandVisualization();

		/**
		 * @brief Construct and initializes eband visualization
		 */
		EBandVisualization(ros::NodeHandle& pn, costmap_2d::Costmap2DROS* costmap_ros);

		/**
		 * @brief Default destructor
		 */
		~EBandVisualization();

		/**
		 * @brief Initializes the visualization class
		 * @param name The name to give this instance (important for publishing)
		 */
		void initialize(ros::NodeHandle& pn, costmap_2d::Costmap2DROS* costmap_ros);

		/**
		 * @brief publishes the bubbles (Position and Expansion) in a band as Marker-Array
		 * @param the name space under which the markers shall be bublished
		 * @param the shape of the markers
		 * @param the band which shall be published
		 */
		void publishBand(std::string marker_name_space, std::vector<Bubble> band);

		/**
		 * @brief publishes a single bubble as a Marker
		 * @param the name space under which the markers shall be bublished
		 * @param the shape of the markers
		 * @param the bubble which shall be published
		 */
		void publishBubble(std::string marker_name_space, int marker_id, Bubble bubble);

		/**
		 * @brief publishes a single bubble as a Marker
		 * @param the name space under which the markers shall be bublished
		 * @param the shape of the markers
		 * @param the bubble which shall be published
		 * @param color in which the bubble shall be displayed
		 */
		void publishBubble(std::string marker_name_space, int marker_id, Color marker_color, Bubble bubble);

		/**
		 * @brief publishes the list of forces along the band as Marker-Array
		 * @param the name space under which the markers shall be bublished
		 * @param the shape of the markers
		 * @param the list of forces which shall be published
		 * @param the list of bubbles on which the forces act (needed to get origin of force-vector)
		 */
		void publishForceList(std::string marker_name_space, std::vector<geometry_msgs::WrenchStamped> forces, std::vector<Bubble> band);

		/**
		 * @brief publishes a single force as a Marker
		 * @param the name space under which the markers shall be bublished
		 * @param the shape of the markers
		 * @param the force which shall be published
		 */
		void publishForce(std::string marker_name_space, int id, Color marker_color, geometry_msgs::WrenchStamped force, Bubble bubble);

	private:
		
		// external objects
		costmap_2d::Costmap2DROS* costmap_ros_; ///<@brief pointer to costmap - needed to retrieve information about robot geometry

		// Topics & Services
		ros::Publisher bubble_pub_; ///<@brief publishes markers to visualize bubbles of elastic band ("modified global plan")
		ros::Publisher one_bubble_pub_; ///<@brief publishes markers to visualize bubbles of elastic band ("modified global plan")

		// flags
		bool initialized_;

		// parameters
		double marker_lifetime_;


		// methods

		/**
		 * @brief converts a bubble into a Marker msg - this is visualization-specific
		 * @param the bubble to convert
		 * @param reference to hand back the marker
		 * @param name space under which the marker shall be published
		 * @param object id of the marker in its name space
		 */
		void bubbleToMarker(Bubble bubble, visualization_msgs::Marker& marker, std::string marker_name_space, int marker_id, Color marker_color);

		/**
		 * @brief converts the haeding of a bubble into a Arrow-Marker msg - this is visualization-specific
		 * @param the bubble to convert
		 * @param reference to hand back the marker
		 * @param name space under which the marker shall be published
		 * @param object id of the marker in its name space
		 */
		void bubbleHeadingToMarker(Bubble bubble, visualization_msgs::Marker& marker, std::string marker_name_space, int marker_id, Color marker_color);

		/**
		 * @brief converts a wrench into a Marker msg - this is visualization-specific
		 * @param the wrench to convert
		 * @param origin of force or wrench
		 * @param reference to hand back the marker
		 * @param name space under which the marker shall be published
		 * @param object id of the marker in its name space
		 * @param color in which the marker shall be displayed
		 */
		void forceToMarker(geometry_msgs::WrenchStamped wrench, geometry_msgs::Pose wrench_origin, visualization_msgs::Marker& marker, std::string marker_name_space, int marker_id, Color marker_color);

};
};
#endif
