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

#include <eband_local_planner/eband_trajectory_controller.h>
#include <tf/transform_datatypes.h>


namespace eband_local_planner{

using std::min;
using std::max;


EBandTrajectoryCtrl::EBandTrajectoryCtrl() : costmap_ros_(NULL), initialized_(false), band_set_(false), visualization_(false) {}


EBandTrajectoryCtrl::EBandTrajectoryCtrl(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false), band_set_(false), visualization_(false)
{
	// initialize planner
	initialize(name, costmap_ros);

  // Initialize pid object (note we'll be further clamping its output)
  pid_.initPid(1, 0, 0, 10, -10);
}


EBandTrajectoryCtrl::~EBandTrajectoryCtrl() {}


void EBandTrajectoryCtrl::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{

	// check if trajectory controller is already initialized
	if(!initialized_)
	{
		// create Node Handle with name of plugin (as used in move_base for loading)
		ros::NodeHandle node_private("~/" + name);
	
		// read parameters from parameter server
		node_private.param("max_vel_lin", max_vel_lin_, 0.6);
		node_private.param("max_vel_th", max_vel_th_, 1.5);

		node_private.param("min_vel_lin", min_vel_lin_, 0.1);
		node_private.param("min_vel_th", min_vel_th_, 0.0);

		node_private.param("min_in_place_vel_th", min_in_place_vel_th_, 0.0);
		node_private.param("in_place_trans_vel", in_place_trans_vel_, 0.0);

		node_private.param("tolerance_trans", tolerance_trans_, 0.02);
		node_private.param("tolerance_rot", tolerance_rot_, 0.04);
		node_private.param("tolerance_timeout", tolerance_timeout_, 0.5);

		node_private.param("k_prop", k_p_, 4.0);
		node_private.param("k_damp", k_nu_, 3.5);
		node_private.param("Ctrl_Rate", ctrl_freq_, 10.0); // TODO retrieve this from move base parameters

		node_private.param("max_acceleration", acc_max_, 0.5);
		node_private.param("virtual_mass", virt_mass_, 0.75);

		node_private.param("max_translational_acceleration", acc_max_trans_, 0.5);
		node_private.param("max_rotational_acceleration", acc_max_rot_, 1.5);

                node_private.param("rotation_correction_threshold", rotation_correction_threshold_, 0.5);

		// copy adress of costmap and Transform Listener (handed over from move_base)
		costmap_ros_ = costmap_ros;

		// init velocity for interpolation
		last_vel_.linear.x = 0.0;
		last_vel_.linear.y = 0.0;
		last_vel_.linear.z = 0.0;
		last_vel_.angular.x = 0.0;
		last_vel_.angular.y = 0.0;
		last_vel_.angular.z = 0.0;

		// set the general refernce frame to that in which the band is given
		geometry_msgs::Pose2D tmp_pose2D;
		tmp_pose2D.x = 0.0;
		tmp_pose2D.y = 0.0;
		tmp_pose2D.theta = 0.0;
		Pose2DToPose(ref_frame_band_, tmp_pose2D);

		// set initialized flag
		initialized_ = true;
	}
	else
	{
		ROS_WARN("This planner has already been initialized, doing nothing.");
	}
}


void EBandTrajectoryCtrl::setVisualization(boost::shared_ptr<EBandVisualization> target_visual)
{
	target_visual_ = target_visual;

	visualization_ = true;
}

bool EBandTrajectoryCtrl::setBand(const std::vector<Bubble>& elastic_band)
{
	elastic_band_ = elastic_band;
	band_set_ = true;
	return true;
}


bool EBandTrajectoryCtrl::setOdometry(const nav_msgs::Odometry& odometry)
{
	odom_vel_.linear.x = odometry.twist.twist.linear.x;
	odom_vel_.linear.y = odometry.twist.twist.linear.y;
	odom_vel_.linear.z = 0.0;
	odom_vel_.angular.x = 0.0;
	odom_vel_.angular.y = 0.0;
	odom_vel_.angular.z = odometry.twist.twist.angular.z;

	return true;
}

// Return the angular difference between the direction we're pointing
// and the direction we want to move in
double angularDiff (const geometry_msgs::Twist& heading,
                    const geometry_msgs::Pose& pose)
{
  const double pi = 3.14159265;
  const double t1 = atan2(heading.linear.y, heading.linear.x);
  const double t2 = tf::getYaw(pose.orientation);
  const double d = t1-t2;

  if (fabs(d)<pi)
    return d;
  else if (d<0)
    return d+2*pi;
  else
    return d-2*pi;
}


bool EBandTrajectoryCtrl::getTwist(geometry_msgs::Twist& twist_cmd)
{
	// init twist cmd to be handed back to caller
	geometry_msgs::Twist robot_cmd, bubble_diff, control_deviation;
	robot_cmd.linear.x = 0.0;
	robot_cmd.linear.y = 0.0;
	robot_cmd.linear.z = 0.0;
	robot_cmd.angular.x = 0.0;
	robot_cmd.angular.y = 0.0;
	robot_cmd.angular.z = 0.0;

	// make sure command vector is clean
	twist_cmd = robot_cmd;
	control_deviation = robot_cmd;

	// check if plugin initialized
	if(!initialized_)
	{
		ROS_ERROR("Requesting feedforward command from not initialized planner. Please call initialize() before using this planner");
		return false;
	}

	// check there is a plan at all (minimum 1 frame in this case, as robot + goal = plan)
	if( (!band_set_) || (elastic_band_.size() < 2) )
	{
		ROS_WARN("Requesting feedforward command from empty band.");
		return false;
	}

	// calc intersection of bubble-radius with sequence of vector connecting the bubbles

	// get distance to target from bubble-expansion
	double scaled_radius = 0.7 * elastic_band_.at(0).expansion;

	// get difference and distance between bubbles in odometry frame
	double bubble_distance, ang_pseudo_dist;
	bubble_diff = getFrame1ToFrame2InRefFrame(elastic_band_.at(0).center.pose,
												elastic_band_.at(1).center.pose,
													ref_frame_band_);
	ang_pseudo_dist = bubble_diff.angular.z * costmap_ros_->getCircumscribedRadius();
	bubble_distance = sqrt( (bubble_diff.linear.x * bubble_diff.linear.x) + (bubble_diff.linear.y * bubble_diff.linear.y) +
						(ang_pseudo_dist * ang_pseudo_dist) );

	if(visualization_)
	{
		target_visual_->publishBubble("ctrl_target", 1, target_visual_->blue, elastic_band_.at(0));
		target_visual_->publishBubble("ctrl_target", 2, target_visual_->blue, elastic_band_.at(1));
	}

	// by default our control deviation is the difference between the bubble centers
	double abs_ctrl_dev;
	control_deviation = bubble_diff;


	ang_pseudo_dist = control_deviation.angular.z * costmap_ros_->getCircumscribedRadius();
	abs_ctrl_dev = sqrt( (control_deviation.linear.x * control_deviation.linear.x) +
								(control_deviation.linear.y * control_deviation.linear.y) +
									(ang_pseudo_dist * ang_pseudo_dist) );

	// yet depending on the expansion of our bubble we might want to adapt this point
	if(scaled_radius < bubble_distance)
	{
		// triviale case - simply scale bubble_diff
		double scale_difference = scaled_radius / bubble_distance;
		bubble_diff.linear.x *= scale_difference;
		bubble_diff.linear.y *= scale_difference;
		bubble_diff.angular.z *= scale_difference;
		// set controls
		control_deviation = bubble_diff;
	}
	
	// if scaled_radius = bubble_distance -- we have nothing to do at all

	if(scaled_radius > bubble_distance)
	{
		// o.k. now we have to do a little bit more -> check next but one bubble
		if(elastic_band_.size() > 2)
		{
			// get difference between next and next but one bubble
			double next_bubble_distance;
			geometry_msgs::Twist next_bubble_diff;
			next_bubble_diff = getFrame1ToFrame2InRefFrame(elastic_band_.at(1).center.pose,
															elastic_band_.at(2).center.pose,
																ref_frame_band_);
			ang_pseudo_dist = next_bubble_diff.angular.z * costmap_ros_->getCircumscribedRadius();
			next_bubble_distance = sqrt( (next_bubble_diff.linear.x * next_bubble_diff.linear.x) +
											(next_bubble_diff.linear.y * next_bubble_diff.linear.y) +
												(ang_pseudo_dist * ang_pseudo_dist) );

			if(scaled_radius > (bubble_distance + next_bubble_distance) )
			{
				// we should normally not end up here - but just to be sure
				control_deviation.linear.x = bubble_diff.linear.x + next_bubble_diff.linear.x;
				control_deviation.linear.y = bubble_diff.linear.y + next_bubble_diff.linear.y;
				control_deviation.angular.z = bubble_diff.angular.z + next_bubble_diff.angular.z;
				// done
				if(visualization_)
					target_visual_->publishBubble("ctrl_target", 3, target_visual_->red, elastic_band_.at(2));
			}
			else
			{
				if(visualization_)
					target_visual_->publishBubble("ctrl_target", 3, target_visual_->red, elastic_band_.at(2));

				// we want to calculate intersection point of bubble ...
				// ... and vector connecting the following bubbles
				double b_distance, cosine_at_bub;
				double vec_prod, norm_vec1, norm_vec2;
				double ang_pseudo_dist1, ang_pseudo_dist2;

				// get distance between next bubble center and intersection point
				ang_pseudo_dist1 = bubble_diff.angular.z * costmap_ros_->getCircumscribedRadius();
				ang_pseudo_dist2 = next_bubble_diff.angular.z * costmap_ros_->getCircumscribedRadius();
				// careful! - we need this sign because of the direction of the vectors and the definition of the vector-product
				vec_prod = - ( (bubble_diff.linear.x * next_bubble_diff.linear.x) +
								(bubble_diff.linear.y * next_bubble_diff.linear.y) +
									(ang_pseudo_dist1 * ang_pseudo_dist2) );

				norm_vec1 = sqrt( (bubble_diff.linear.x * bubble_diff.linear.x) +
								(bubble_diff.linear.y * bubble_diff.linear.y) +
									(ang_pseudo_dist1 * ang_pseudo_dist1) );

				norm_vec2 = sqrt( (next_bubble_diff.linear.x * next_bubble_diff.linear.x) +
								(next_bubble_diff.linear.y * next_bubble_diff.linear.y) +
									(ang_pseudo_dist2 * ang_pseudo_dist2) );

				// reform the cosine-rule
				cosine_at_bub = vec_prod / norm_vec1 / norm_vec2;
				b_distance = bubble_distance * cosine_at_bub + sqrt( scaled_radius*scaled_radius -
								bubble_distance*bubble_distance * (1.0 - cosine_at_bub*cosine_at_bub) );

				// get difference vector from next_bubble to intersection point
				double scale_next_difference = b_distance / next_bubble_distance;
				next_bubble_diff.linear.x *= scale_next_difference;
				next_bubble_diff.linear.y *= scale_next_difference;
				next_bubble_diff.angular.z *= scale_next_difference;

				// and finally get the control deviation
				control_deviation.linear.x = bubble_diff.linear.x + next_bubble_diff.linear.x;
				control_deviation.linear.y = bubble_diff.linear.y + next_bubble_diff.linear.y;
				control_deviation.angular.z = bubble_diff.angular.z + next_bubble_diff.angular.z;
				// done
			}
		}
	}

	// plot control deviation
	ang_pseudo_dist = control_deviation.angular.z * costmap_ros_->getCircumscribedRadius();
	abs_ctrl_dev = sqrt( (control_deviation.linear.x * control_deviation.linear.x) +
								(control_deviation.linear.y * control_deviation.linear.y) +
									(ang_pseudo_dist * ang_pseudo_dist) );


	if(visualization_)
	{
		// compose bubble from ctrl-target
		geometry_msgs::Pose2D tmp_bubble_2d, curr_bubble_2d;
		geometry_msgs::Pose tmp_pose;
		// init bubble for visualization
		Bubble new_bubble = elastic_band_.at(0);
		PoseToPose2D(elastic_band_.at(0).center.pose, curr_bubble_2d);
		tmp_bubble_2d.x = curr_bubble_2d.x + control_deviation.linear.x;
		tmp_bubble_2d.y = curr_bubble_2d.y + control_deviation.linear.y;
		tmp_bubble_2d.theta = curr_bubble_2d.theta + control_deviation.angular.z;
		Pose2DToPose(tmp_pose, tmp_bubble_2d);
		new_bubble.center.pose = tmp_pose;
		new_bubble.expansion = 0.1; // just draw a small bubble
		target_visual_->publishBubble("ctrl_target", 0, target_visual_->red, new_bubble);
	}


        const geometry_msgs::Point& goal = (--elastic_band_.end())->center.pose.position;
        const double dx = elastic_band_.at(0).center.pose.position.x - goal.x;
        const double dy = elastic_band_.at(0).center.pose.position.y - goal.y;
        const double dist_to_goal = sqrt(dx*dx + dy*dy);
        
        // Assuming we're far enough from the final goal, make sure to rotate so
        // we're facing the right way
        if (dist_to_goal > rotation_correction_threshold_)
        {
        
          const double angular_diff = angularDiff(control_deviation, elastic_band_.at(0).center.pose);
          const double vel = pid_.updatePid(-angular_diff, ros::Duration(1/ctrl_freq_));
          const double mult = fabs(vel) > max_vel_th_ ? max_vel_th_/fabs(vel) : 1.0;
          control_deviation.angular.z = vel*mult;
          const double abs_vel = fabs(control_deviation.angular.z);

          ROS_DEBUG_THROTTLE_NAMED (1.0, "angle_correction",
                                    "Angular diff is %.2f and desired angular "
                                    "vel is %.2f.  Initial translation velocity "
                                    "is %.2f, %.2f", angular_diff,
                                    control_deviation.angular.z,
                                    control_deviation.linear.x,
                                    control_deviation.linear.y);
          const double trans_mult = max(0.01, 1.0 - abs_vel/max_vel_th_); // There are some weird tf errors if I let it be 0
          control_deviation.linear.x *= trans_mult;
          control_deviation.linear.y *= trans_mult;
          ROS_DEBUG_THROTTLE_NAMED (1.0, "angle_correction",
                                    "Translation multiplier is %.2f and scaled "
                                    "translational velocity is %.2f, %.2f",
                                    trans_mult, control_deviation.linear.x,
                                    control_deviation.linear.y);
        }
        else
          ROS_DEBUG_THROTTLE_NAMED (1.0, "angle_correction",
                                    "Not applying angle correction because "
                                    "distance to goal is %.2f", dist_to_goal);
                                    
        


	// now the actual control procedure start (using attractive Potentials)
	geometry_msgs::Twist desired_velocity, currbub_maxvel_dir;
	double desvel_abs, desvel_abs_trans, currbub_maxvel_abs;
	double scale_des_vel;
	desired_velocity = robot_cmd;
	currbub_maxvel_dir = robot_cmd;
	
	// calculate "equilibrium velocity" (Khatib86 - Realtime Obstacle Avoidance)
	desired_velocity.linear.x = k_p_/k_nu_ * control_deviation.linear.x;
	desired_velocity.linear.y = k_p_/k_nu_ * control_deviation.linear.y;
	desired_velocity.angular.z = k_p_/k_nu_ * control_deviation.angular.z;

	//robot_cmd = desired_velocity;

	// get max vel for current bubble
	int curr_bub_num = 0;
	currbub_maxvel_abs = getBubbleTargetVel(curr_bub_num, elastic_band_, currbub_maxvel_dir);

	// if neccessarry scale desired vel to stay lower than currbub_maxvel_abs
	ang_pseudo_dist = desired_velocity.angular.z * costmap_ros_->getCircumscribedRadius();
	desvel_abs = sqrt( (desired_velocity.linear.x * desired_velocity.linear.x) +
							(desired_velocity.linear.y * desired_velocity.linear.y) +
								(ang_pseudo_dist * ang_pseudo_dist) );
	if(desvel_abs > currbub_maxvel_abs)
	{
		scale_des_vel = currbub_maxvel_abs / desvel_abs;
		desired_velocity.linear.x *= scale_des_vel;
		desired_velocity.linear.y *= scale_des_vel;
		desired_velocity.angular.z *= scale_des_vel;
	}

	// make sure to stay within velocity bounds for the robot
	desvel_abs_trans = sqrt( (desired_velocity.linear.x * desired_velocity.linear.x) + (desired_velocity.linear.y * desired_velocity.linear.y) );
	// for translation
	if(desvel_abs_trans > max_vel_lin_)
	{
		scale_des_vel = max_vel_lin_ / desvel_abs_trans;
		desired_velocity.linear.x *= scale_des_vel;
		desired_velocity.linear.y *= scale_des_vel;
		// to make sure we are staying inside the bubble also scale rotation
		desired_velocity.angular.z *= scale_des_vel;
	}

	// for rotation
	if(fabs(desired_velocity.angular.z) > max_vel_th_)
	{
		scale_des_vel = max_vel_th_ / fabs(desired_velocity.angular.z);
		desired_velocity.angular.z *= scale_des_vel;
		// to make sure we are staying inside the bubble also scale translation
		desired_velocity.linear.x *= scale_des_vel;
		desired_velocity.linear.y *= scale_des_vel;
	}

	// calculate resulting force (accel. resp.) (Khatib86 - Realtime Obstacle Avoidance)
	geometry_msgs::Twist acc_desired;
	acc_desired = robot_cmd;
	acc_desired.linear.x = (1.0/virt_mass_) * k_nu_ * (desired_velocity.linear.x - last_vel_.linear.x);
	acc_desired.linear.y = (1.0/virt_mass_) * k_nu_ * (desired_velocity.linear.y - last_vel_.linear.y);
	acc_desired.angular.z = (1.0/virt_mass_) * k_nu_ * (desired_velocity.angular.z - last_vel_.angular.z);

	// constrain acceleration
	double scale_acc;
	double abs_acc_trans = sqrt( (acc_desired.linear.x*acc_desired.linear.x) + (acc_desired.linear.y*acc_desired.linear.y) );
	if(abs_acc_trans > acc_max_trans_)
	{
		scale_acc = acc_max_trans_ / abs_acc_trans;
		acc_desired.linear.x *= scale_acc;
		acc_desired.linear.y *= scale_acc;
		// again - keep relations - stay in bubble
		acc_desired.angular.z *= scale_acc;
	}

	if(fabs(acc_desired.angular.z) > acc_max_rot_)
	{
		scale_acc = fabs(acc_desired.angular.z) / acc_max_rot_;
		acc_desired.angular.z *= scale_acc;
		// again - keep relations - stay in bubble
		acc_desired.linear.x *= scale_acc;
		acc_desired.linear.y *= scale_acc;
	}

	// and get velocity-cmds by integrating them over one time-step
	last_vel_.linear.x = last_vel_.linear.x + acc_desired.linear.x / ctrl_freq_;
	last_vel_.linear.y = last_vel_.linear.y + acc_desired.linear.y / ctrl_freq_;
	last_vel_.angular.z = last_vel_.angular.z + acc_desired.angular.z / ctrl_freq_;


	// we are almost done now take into accoun stick-slip and similar nasty things

	// last checks - limit current twist cmd (upper and lower bounds)
	last_vel_ = limitTwist(last_vel_);

	// finally set robot_cmd (to non-zero value)
	robot_cmd = last_vel_;

	// now convert into robot-body frame
	robot_cmd = transformTwistFromFrame1ToFrame2(robot_cmd, ref_frame_band_, elastic_band_.at(0).center.pose);

	// check whether we reached the end of the band
	int curr_target_bubble = 1;
	while(fabs(bubble_diff.linear.x) <= tolerance_trans_ &&
			fabs(bubble_diff.linear.y) <= tolerance_trans_ &&
			fabs(bubble_diff.angular.z) <= tolerance_rot_)
	{
		if(curr_target_bubble < ((int) elastic_band_.size()) - 1)
		{
                  curr_target_bubble++;
                  // transform next target bubble into robot-body frame
                  // and get difference to robot bubble
                  bubble_diff = getFrame1ToFrame2InRefFrame(elastic_band_.at(0).center.pose, elastic_band_.at(curr_target_bubble).center.pose,
                                                            ref_frame_band_);
		}
		else
		{
                  ROS_DEBUG_THROTTLE_NAMED (1.0, "controller_state",
                                            "Goal reached with distance %.2f, %.2f, %.2f"
                                            "; sending zero velocity",
                                            bubble_diff.linear.x, bubble_diff.linear.y,
                                            bubble_diff.angular.z);
                  // goal position reached
                  robot_cmd.linear.x = 0.0;
                  robot_cmd.linear.y = 0.0;
                  robot_cmd.angular.z = 0.0;
                  // reset velocity
                  last_vel_.linear.x = 0.0;
                  last_vel_.linear.y = 0.0;
                  last_vel_.angular.z = 0.0;
                  break;
		}
	}

	twist_cmd = robot_cmd;
	
	return true;
}


double EBandTrajectoryCtrl::getBubbleTargetVel(const int& target_bub_num, const std::vector<Bubble>& band, geometry_msgs::Twist& VelDir)
{
	// init reference for direction vector
	VelDir.linear.x = 0.0;
	VelDir.linear.y = 0.0;
	VelDir.linear.z = 0.0;
	VelDir.angular.x = 0.0;
	VelDir.angular.y = 0.0;
	VelDir.angular.z = 0.0;

	// if we are looking at the last bubble - target vel is always zero
	if(target_bub_num >= ((int) band.size() - 1))
		return 0.0;


	// otherwise check for max_vel calculated from current bubble size
	double v_max_curr_bub, v_max_next_bub;
	double bubble_distance, angle_to_pseudo_vel, delta_vel_max;
	geometry_msgs::Twist bubble_diff;

	// distance for braking s = 0.5*v*v/a
	v_max_curr_bub = sqrt(2 * elastic_band_.at(target_bub_num).expansion * acc_max_);

	// get distance to next bubble center
	ROS_ASSERT( (target_bub_num >= 0) && ((target_bub_num +1) < (int) band.size()) );
	bubble_diff = getFrame1ToFrame2InRefFrame(band.at(target_bub_num).center.pose, band.at(target_bub_num + 1).center.pose,
												ref_frame_band_);
	angle_to_pseudo_vel = bubble_diff.angular.z * costmap_ros_->getCircumscribedRadius();

	bubble_distance = sqrt( (bubble_diff.linear.x * bubble_diff.linear.x) + (bubble_diff.linear.y * bubble_diff.linear.y) +
							(angle_to_pseudo_vel * angle_to_pseudo_vel) );

	// calculate direction vector - norm of diference
	VelDir.linear.x =bubble_diff.linear.x/bubble_distance;
	VelDir.linear.y =bubble_diff.linear.y/bubble_distance;
	VelDir.angular.z =bubble_diff.angular.z/bubble_distance;

	// if next bubble outside this one we will always be able to break fast enough
	if(bubble_distance > band.at(target_bub_num).expansion )
		return v_max_curr_bub;


	// next bubble center inside this bubble - take into account restrictions on next bubble
	int next_bub_num = target_bub_num + 1;
	geometry_msgs::Twist dummy_twist;
	v_max_next_bub = getBubbleTargetVel(next_bub_num, band, dummy_twist); // recursive call

	// if velocity at next bubble bigger (or equal) than our velocity - we are on the safe side
	if(v_max_next_bub >= v_max_curr_bub)
		return v_max_curr_bub;


	// otherwise max. allowed vel is next vel + plus possible reduction on the way between the bubble-centers
	delta_vel_max = sqrt(2 * bubble_distance * acc_max_);
	v_max_curr_bub = v_max_next_bub + delta_vel_max;

	return v_max_curr_bub;
}


geometry_msgs::Twist EBandTrajectoryCtrl::getFrame1ToFrame2InRefFrame(const geometry_msgs::Pose& frame1, const geometry_msgs::Pose& frame2, const geometry_msgs::Pose& ref_frame)
{

	geometry_msgs::Pose2D frame1_pose2D, frame2_pose2D, ref_frame_pose2D;
	geometry_msgs::Pose2D frame1_pose2D_rf, frame2_pose2D_rf;
	geometry_msgs::Twist frame_diff;

	// transform all frames to Pose2d
	PoseToPose2D(frame1, frame1_pose2D);
	PoseToPose2D(frame2, frame2_pose2D);
	PoseToPose2D(ref_frame, ref_frame_pose2D);

	// transform frame1 into ref frame
	frame1_pose2D_rf.x = (frame1_pose2D.x - ref_frame_pose2D.x) * cos(ref_frame_pose2D.theta) +
							(frame1_pose2D.y - ref_frame_pose2D.y) * sin(ref_frame_pose2D.theta);
	frame1_pose2D_rf.y = -(frame1_pose2D.x - ref_frame_pose2D.x) * sin(ref_frame_pose2D.theta) +
							(frame1_pose2D.y - ref_frame_pose2D.y) * cos(ref_frame_pose2D.theta);
	frame1_pose2D_rf.theta = frame1_pose2D.theta - ref_frame_pose2D.theta;
	frame1_pose2D_rf.theta = angles::normalize_angle(frame1_pose2D_rf.theta);
	// transform frame2 into ref frame
	frame2_pose2D_rf.x = (frame2_pose2D.x - ref_frame_pose2D.x) * cos(ref_frame_pose2D.theta) +
							(frame2_pose2D.y - ref_frame_pose2D.y) * sin(ref_frame_pose2D.theta);
	frame2_pose2D_rf.y = -(frame2_pose2D.x - ref_frame_pose2D.x) * sin(ref_frame_pose2D.theta) +
							(frame2_pose2D.y - ref_frame_pose2D.y) * cos(ref_frame_pose2D.theta);
	frame2_pose2D_rf.theta = frame2_pose2D.theta - ref_frame_pose2D.theta;
	frame2_pose2D_rf.theta = angles::normalize_angle(frame2_pose2D_rf.theta);

	// get differences
	frame_diff.linear.x = frame2_pose2D_rf.x - frame1_pose2D_rf.x;
	frame_diff.linear.y = frame2_pose2D_rf.y - frame1_pose2D_rf.y;
	frame_diff.linear.z = 0.0;
	frame_diff.angular.x = 0.0;
	frame_diff.angular.y = 0.0;
	frame_diff.angular.z = frame2_pose2D_rf.theta - frame1_pose2D_rf.theta;
	// normalize angle
	frame_diff.angular.z = angles::normalize_angle(frame_diff.angular.z);

	return frame_diff;
}


geometry_msgs::Twist EBandTrajectoryCtrl::transformTwistFromFrame1ToFrame2(const geometry_msgs::Twist& curr_twist,
															const geometry_msgs::Pose& frame1, const geometry_msgs::Pose& frame2)
{
	geometry_msgs::Pose2D frame1_pose2D, frame2_pose2D;
	geometry_msgs::Twist tmp_transformed;
	double delta_ang;

	tmp_transformed = curr_twist;

	// transform all frames to Pose2d
	PoseToPose2D(frame1, frame1_pose2D);
	PoseToPose2D(frame2, frame2_pose2D);

	// get orientation diff of frames
	delta_ang = frame2_pose2D.theta - frame1_pose2D.theta;
	delta_ang = angles::normalize_angle(delta_ang);

	// tranform twist
	tmp_transformed.linear.x = curr_twist.linear.x * cos(delta_ang) + curr_twist.linear.y * sin(delta_ang);
	tmp_transformed.linear.y = -curr_twist.linear.x * sin(delta_ang) + curr_twist.linear.y * cos(delta_ang);

	return tmp_transformed;
}


geometry_msgs::Twist EBandTrajectoryCtrl::limitTwist(const geometry_msgs::Twist& twist)
{
	geometry_msgs::Twist res = twist;

	//make sure to bound things by our velocity limits
	double lin_overshoot = sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y) / max_vel_lin_;
	double lin_undershoot = min_vel_lin_ / sqrt(res.linear.x * res.linear.x + res.linear.y * res.linear.y);
	if (lin_overshoot > 1.0) 
	{
		res.linear.x /= lin_overshoot;
		res.linear.y /= lin_overshoot;
		// keep relations
		res.angular.z /= lin_overshoot;
	}

	//we only want to enforce a minimum velocity if we're not rotating in place
	if(lin_undershoot > 1.0)
	{
		res.linear.x *= lin_undershoot;
		res.linear.y *= lin_undershoot;
		// we cannot keep relations here for stability reasons
	}

	if (fabs(res.angular.z) > max_vel_th_)
	{
		double scale = max_vel_th_/fabs(res.angular.z);
		//res.angular.z = max_vel_th_ * sign(res.angular.z);
		res.angular.z *= scale;
		// keep relations
		res.linear.x *= scale;
		res.linear.y *= scale;
	}

	if (fabs(res.angular.z) < min_vel_th_) res.angular.z = min_vel_th_ * sign(res.angular.z);
	// we cannot keep relations here for stability reasons

	//we want to check for whether or not we're desired to rotate in place
	if(sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y) < in_place_trans_vel_)
	{
		if (fabs(res.angular.z) < min_in_place_vel_th_)
			res.angular.z = min_in_place_vel_th_ * sign(res.angular.z);

		res.linear.x = 0.0;
		res.linear.y = 0.0;
	}

	ROS_DEBUG("Angular command %f", res.angular.z);
	return res;
}


}
