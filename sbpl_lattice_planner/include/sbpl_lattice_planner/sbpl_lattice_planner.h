#ifndef SBPL_LATTICE_PLANNER_H
#define SBPL_LATTICE_PLANNER_H

#include <iostream>
#include <vector>

using namespace std;

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>

// sbpl headers
#include <sbpl/headers.h>

// global representation
#include <nav_core/base_global_planner.h>

namespace sbpl_lattice_planner{

class SBPLLatticePlanner : public nav_core::BaseGlobalPlanner{
public:
  
  /**
   * @brief  Default constructor for the NavFnROS object
   */
  SBPLLatticePlanner();

  
  /**
   * @brief  Constructor for the SBPLLatticePlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  SBPLLatticePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);


  /**
   * @brief  Initialization function for the SBPLLatticePlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  virtual void initialize(std::string name, 
                          costmap_2d::Costmap2DROS* costmap_ros);
  
  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose 
   * @param goal The goal pose 
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        std::vector<geometry_msgs::PoseStamped>& plan);

  virtual ~SBPLLatticePlanner(){};

private:
  unsigned char costMapCostToSBPLCost(unsigned char newcost);
  void publishStats(int solution_cost, int solution_size, 
                    const geometry_msgs::PoseStamped& start, 
                    const geometry_msgs::PoseStamped& goal);

  unsigned char computeCircumscribedCost();

  static void transformFootprintToEdges(const geometry_msgs::Pose& robot_pose,
                                        const std::vector<geometry_msgs::Point>& footprint,
                                        std::vector<geometry_msgs::Point>& out_footprint);

  void getFootprintList(const std::vector<EnvNAVXYTHETALAT3Dpt_t>& sbpl_path, const std::string& path_frame_id,
                        visualization_msgs::Marker& ma);

  bool initialized_;

  SBPLPlanner* planner_;
  EnvironmentNAVXYTHETALAT* env_;
  
  std::string planner_type_; /**< sbpl method to use for planning.  choices are ARAPlanner and ADPlanner */

  double allocated_time_; /**< amount of time allowed for search */
  double initial_epsilon_; /**< initial epsilon for beginning the anytime search */

  std::string environment_type_; /** what type of environment in which to plan.  choices are 2D and XYThetaLattice. */ 
  std::string cost_map_topic_; /** what topic is being used for the costmap topic */

  bool forward_search_; /** whether to use forward or backward search */
  std::string primitive_filename_; /** where to find the motion primitives for the current robot */
  int force_scratch_limit_; /** the number of cells that have to be changed in the costmap to force the planner to plan from scratch even if its an incremental planner */

  unsigned char lethal_obstacle_;
  unsigned char inscribed_inflated_obstacle_;
  unsigned char circumscribed_cost_;
  unsigned char sbpl_cost_multiplier_;

  bool publish_footprint_path_;
  int visualizer_skip_poses_;

  bool allow_unknown_;

  double goal_blend_distance_; /**< length of the plan tail blended into the exact goal pose, 0 disables blending */

  std::string name_;
  costmap_2d::Costmap2DROS* costmap_ros_; /**< manages the cost map for us */
  std::vector<geometry_msgs::Point> footprint_;
  unsigned int current_env_width_;
  unsigned int current_env_height_;

  ros::Publisher plan_pub_;
  ros::Publisher stats_publisher_;
  ros::Publisher sbpl_plan_footprint_pub_;
};

/**
 * @brief Blends the tail of a plan into the exact requested goal pose.
 *
 * sbpl_lattice_planner leaves out the last intermediate pose of every motion
 * primitive (it coincides with the first pose of the next one), so its plans
 * end one sample short of the final primitive, and its goal is snapped to the
 * lattice (5 cm cells, 22.5 degree headings). The DWB local planner steers to
 * the last pose of the plan, which therefore ends up to half a cell, one
 * primitive sample (up to 4.4 cm) and half a heading (11.25 degrees) away from
 * where the robot was sent. The last motion of the plan (up to
 * goal_blend_distance of it) is blended into the exact goal, so that a
 * differential drive robot can correct the offset while still moving instead
 * of facing a sideways jump at the very end. In-place-turn poses at the plan
 * end (consecutive poses with identical x/y) are blended with w = 1 when the
 * walk back from the plan end also includes at least one non-zero step, so a
 * trailing turn is translated to the goal position instead of being left
 * behind at the pre-blend position. (An all-turn tail, or a blend distance
 * smaller than the preceding step, leaves the accumulated length at zero: every
 * pose except the final one, which is set to the goal, stays unchanged.)
 *
 * @param goal The exact requested goal pose.
 * @param goal_blend_distance Length of the plan tail to blend, in meters. A
 *        value <= 0.0 (and an empty plan) leaves the plan unchanged.
 * @param plan The plan to modify in place; its last pose is set to goal.pose.
 */
void blendIntoGoal(const geometry_msgs::PoseStamped& goal, double goal_blend_distance,
                   std::vector<geometry_msgs::PoseStamped>& plan);
};

#endif

