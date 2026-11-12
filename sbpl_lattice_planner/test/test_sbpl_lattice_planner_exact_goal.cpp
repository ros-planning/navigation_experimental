/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, DFKI GmbH
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
 *   * Neither the name of the copyright holder nor the names of its
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
 */

/*
 * Tests for the exact-goal blending of sbpl_lattice_planner.
 *
 * SBPL's ConvertStateIDPathintoXYThetaPath() (a) starts every plan at the
 * center of the lattice cell containing the requested start, (b) ends at the
 * lattice-snapped goal cell (5 cm cells, 16 headings = 22.5 deg) and (c) drops
 * the last intermediate pose of every motion primitive, so the final plan pose
 * is one sample short of the final primitive endpoint. With goal_blend_distance
 * set, the planner is supposed to blend the end of the plan into the exact
 * requested goal pose.
 *
 * The first three integration tests run with blending disabled (the default,
 * and explicitly 0.0): two reproduce the documented endpoint problem and one
 * guards the zero parameter. The fourth is the acceptance test for
 * goal_blend_distance and runs with blending enabled. Below them, the BlendUnit*
 * tests exercise blendIntoGoal() directly.
 */

#include <angles/angles.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sbpl_lattice_planner/sbpl_lattice_planner.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace
{
// unicycle_highcost_5cm.mprim: 5 cm cells, 16 headings.
const double CELL_SIZE = 0.05;

// Deterministic 10 m x 10 m free costmap.
const unsigned int MAP_CELLS = 200;

geometry_msgs::PoseStamped makePose(double x, double y, double yaw)
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation.x = orientation.x();
  pose.pose.orientation.y = orientation.y();
  pose.pose.orientation.z = orientation.z();
  pose.pose.orientation.w = orientation.w();
  return pose;
}

double positionError(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
{
  return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}

double yawError(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
{
  return std::abs(angles::shortest_angular_distance(tf2::getYaw(a.orientation), tf2::getYaw(b.orientation)));
}

void setCostmapParams(ros::NodeHandle& root_nh)
{
  ros::NodeHandle nh(root_nh, "costmap");
  nh.setParam("global_frame", "map");
  nh.setParam("robot_base_frame", "base_link");
  nh.setParam("robot_radius", 0.2);
  nh.setParam("rolling_window", false);
  nh.setParam("track_unknown_space", false);
  // An empty list (as opposed to a missing parameter) keeps Costmap2DROS from
  // loading its pre-Hydro default layers (obstacle + inflation).
  XmlRpc::XmlRpcValue plugins;
  plugins.setSize(0);
  nh.setParam("plugins", plugins);
}

void setPlannerParams(ros::NodeHandle& root_nh, const std::string& name, bool blending_enabled)
{
  ros::NodeHandle nh(root_nh, name);
  nh.setParam("planner_type", "ARAPlanner");
  nh.setParam("environment_type", "XYThetaLattice");
  nh.setParam("allocated_time", 0.5);
  nh.setParam("initial_epsilon", 3.0);
  nh.setParam("forward_search", false);
  nh.setParam("lethal_obstacle", 20);
  nh.setParam("force_scratch_limit", 500);
  nh.setParam("nominalvel_mpersecs", 0.8);
  nh.setParam("timetoturn45degsinplace_secs", 0.6);
  nh.setParam("allow_unknown", true);
  nh.setParam("publish_footprint_path", false);
  if (blending_enabled)
  {
    nh.setParam("goal_blend_distance", 0.3);
  }
}

class SbplLatticePlannerExactGoalTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    ros::NodeHandle root_nh("~");
    setCostmapParams(root_nh);
    setPlannerParams(root_nh, "SBPLLatticePlanner", false);
    setPlannerParams(root_nh, "SBPLLatticePlannerBlend", true);

    // Costmap2DROS blocks in its constructor until map -> base_link is
    // available, so insert an identity transform before creating it. The
    // transform-only buffer has no listener thread, so without
    // setUsingDedicatedThread(true) its canTransform(..., timeout) overload
    // (which Costmap2DROS uses) always returns false and the constructor
    // would wait forever.
    tf_buffer_.reset(new tf2_ros::Buffer());
    tf_buffer_->setUsingDedicatedThread(true);
    geometry_msgs::TransformStamped map_to_base_link;
    map_to_base_link.header.stamp = ros::Time::now();
    map_to_base_link.header.frame_id = "map";
    map_to_base_link.child_frame_id = "base_link";
    map_to_base_link.transform.rotation.w = 1.0;
    tf_buffer_->setTransform(map_to_base_link, "test");
  }

  static void TearDownTestSuite()
  {
    tf_buffer_.reset();
  }

  void SetUp() override
  {
    costmap_ros_.reset(new costmap_2d::Costmap2DROS("costmap", *tf_buffer_));
    // Size the costmap explicitly instead of relying on the dynamic
    // reconfigure defaults; LayeredCostmap::resizeMap() fills it with
    // FREE_SPACE (track_unknown_space is false). No start() needed.
    costmap_ros_->getLayeredCostmap()->resizeMap(MAP_CELLS, MAP_CELLS, CELL_SIZE, 0.0, 0.0);
  }

  void TearDown() override
  {
    costmap_ros_.reset();
  }

  std::unique_ptr<sbpl_lattice_planner::SBPLLatticePlanner> makePlanner(const std::string& name)
  {
    return std::unique_ptr<sbpl_lattice_planner::SBPLLatticePlanner>(
        new sbpl_lattice_planner::SBPLLatticePlanner(name, costmap_ros_.get()));
  }

  static std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
};

std::unique_ptr<tf2_ros::Buffer> SbplLatticePlannerExactGoalTest::tf_buffer_;

// The goal does not lie on the lattice: it is 0.009 m / 0.008 m and 0.08 rad
// away from the snapped cell (cell center 2.025, 1.025, heading 0). On top of
// that, SBPL's path conversion drops the last intermediate pose of every
// primitive, so the plan demonstrably ends away from the requested goal.
TEST_F(SbplLatticePlannerExactGoalTest, ReproductionOffLatticeGoal)
{
  std::unique_ptr<sbpl_lattice_planner::SBPLLatticePlanner> planner = makePlanner("SBPLLatticePlanner");
  const geometry_msgs::PoseStamped start = makePose(1.0, 1.0, 0.0);
  const geometry_msgs::PoseStamped goal = makePose(2.034, 1.017, 0.08);
  std::vector<geometry_msgs::PoseStamped> plan;

  ASSERT_TRUE(planner->makePlan(start, goal, plan));
  ASSERT_FALSE(plan.empty());

  const double pos_err = positionError(plan.back().pose, goal.pose);
  const double yaw_err = yawError(plan.back().pose, goal.pose);
  printf("ReproductionOffLatticeGoal: %zu plan poses, endpoint position error %.6f m, yaw error %.6f rad\n",
         plan.size(), pos_err, yaw_err);

  ASSERT_GT(pos_err, 0.005);
  ASSERT_LT(pos_err, 0.1);
  ASSERT_GT(yaw_err, 0.01);
  ASSERT_LT(yaw_err, 0.25);
}

// The goal lies exactly on the lattice (cell center 2.025, 1.025, heading 0),
// so the lattice snap contributes nothing; the endpoint error that remains is
// caused by the dropped last intermediate pose of the final primitive and
// isolates that part of the problem.
TEST_F(SbplLatticePlannerExactGoalTest, ReproductionOnLatticeGoal)
{
  std::unique_ptr<sbpl_lattice_planner::SBPLLatticePlanner> planner = makePlanner("SBPLLatticePlanner");
  const geometry_msgs::PoseStamped start = makePose(1.0, 1.0, 0.0);
  const geometry_msgs::PoseStamped goal = makePose(2.025, 1.025, 0.0);
  std::vector<geometry_msgs::PoseStamped> plan;

  ASSERT_TRUE(planner->makePlan(start, goal, plan));
  ASSERT_FALSE(plan.empty());

  const double pos_err = positionError(plan.back().pose, goal.pose);
  const double yaw_err = yawError(plan.back().pose, goal.pose);
  printf("ReproductionOnLatticeGoal: %zu plan poses, endpoint position error %.6f m, yaw error %.6f rad\n", plan.size(),
         pos_err, yaw_err);

  ASSERT_GT(pos_err, 0.001);
  ASSERT_LT(pos_err, 0.05);
  ASSERT_LT(yaw_err, 0.25);
}

// goal_blend_distance = 0.0 must keep the old behavior (same endpoint error as
// the default configuration above).
TEST_F(SbplLatticePlannerExactGoalTest, BlendDisabledWithZeroParam)
{
  // Set before the planner is constructed, since initialize() reads it.
  ros::NodeHandle root_nh("~");
  root_nh.setParam("SBPLLatticePlanner/goal_blend_distance", 0.0);

  std::unique_ptr<sbpl_lattice_planner::SBPLLatticePlanner> planner = makePlanner("SBPLLatticePlanner");
  const geometry_msgs::PoseStamped start = makePose(1.0, 1.0, 0.0);
  const geometry_msgs::PoseStamped goal = makePose(2.034, 1.017, 0.08);
  std::vector<geometry_msgs::PoseStamped> plan;

  ASSERT_TRUE(planner->makePlan(start, goal, plan));
  ASSERT_FALSE(plan.empty());

  const double pos_err = positionError(plan.back().pose, goal.pose);
  const double yaw_err = yawError(plan.back().pose, goal.pose);
  printf("BlendDisabledWithZeroParam: %zu plan poses, endpoint position error %.6f m, yaw error %.6f rad\n",
         plan.size(), pos_err, yaw_err);

  ASSERT_GT(pos_err, 0.005);
  ASSERT_LT(pos_err, 0.1);
  ASSERT_GT(yaw_err, 0.01);
  ASSERT_LT(yaw_err, 0.25);
}

// Acceptance test for goal_blend_distance = 0.3: the plan must end exactly at
// the requested goal pose, while its beginning stays at the lattice projection
// of the requested start (offset 0.0354 m, i.e. half a cell diagonal, exactly
// as without blending). The tail must stay smooth: strictly decreasing distance
// to the goal and at most 0.1 m between adjacent poses over the final 20 poses.
TEST_F(SbplLatticePlannerExactGoalTest, BlendIntoGoalEnabled)
{
  std::unique_ptr<sbpl_lattice_planner::SBPLLatticePlanner> planner = makePlanner("SBPLLatticePlannerBlend");
  const geometry_msgs::PoseStamped start = makePose(1.0, 1.0, 0.0);
  const geometry_msgs::PoseStamped goal = makePose(2.034, 1.017, 0.08);
  std::vector<geometry_msgs::PoseStamped> plan;

  ASSERT_TRUE(planner->makePlan(start, goal, plan));
  ASSERT_FALSE(plan.empty());

  const double start_pos_err = positionError(plan.front().pose, start.pose);
  const double start_yaw_err = yawError(plan.front().pose, start.pose);
  printf("BlendIntoGoalEnabled: %zu plan poses, plan start position error %.6f m, yaw error %.6f rad "
         "(vs requested start, unchanged by blending)\n",
         plan.size(), start_pos_err, start_yaw_err);
  ASSERT_GT(start_pos_err, 0.001);
  ASSERT_LT(start_pos_err, 0.05);
  ASSERT_LT(start_yaw_err, 0.25);

  const double pos_err = positionError(plan.back().pose, goal.pose);
  const double yaw_err = yawError(plan.back().pose, goal.pose);
  printf("BlendIntoGoalEnabled: endpoint position error %.9f m, yaw error %.9f rad\n", pos_err, yaw_err);
  ASSERT_LT(pos_err, 1e-9);
  ASSERT_LT(yaw_err, 1e-9);

  ASSERT_GE(plan.size(), 20u);
  const size_t tail_start = plan.size() - 20;
  for (size_t i = tail_start + 1; i < plan.size(); ++i)
  {
    const double previous_distance = positionError(plan[i - 1].pose, goal.pose);
    const double current_distance = positionError(plan[i].pose, goal.pose);
    ASSERT_LT(current_distance, previous_distance) << "distance to goal not strictly decreasing at plan pose " << i;

    const double spacing = positionError(plan[i].pose, plan[i - 1].pose);
    ASSERT_LT(spacing, 0.1) << "adjacent plan poses too far apart at plan pose " << i;
  }
}

// Synthetic straight-line plan: 21 poses from (0, 0) to (1, 0) in 0.05 m steps.
std::vector<geometry_msgs::PoseStamped> makeStraightLinePlan(double yaw)
{
  std::vector<geometry_msgs::PoseStamped> plan;
  for (int i = 0; i <= 20; ++i)
  {
    plan.push_back(makePose(0.05 * i, 0.0, yaw));
  }
  return plan;
}

// Nominal path length of `steps` consecutive 0.05 m steps. Accumulated by
// repeated addition because 6 * 0.05 evaluates to 0.30000000000000004, which
// would fail a <= 0.3 m blend-distance check.
double stepSum(size_t steps)
{
  double length = 0.0;
  for (size_t i = 0; i < steps; ++i)
  {
    length += CELL_SIZE;
  }
  return length;
}

// The tail is offset affinely: pose i moves by w * (dx, dy, dyaw), with
// w = 1 - distance_to_end / tail_length, and the last pose is the goal. The
// blend distance 0.26 m is deliberately off the step boundary: five steps cover
// 0.25 m (the blended tail), six steps would need 0.30 m.
TEST(SbplLatticePlannerBlendIntoGoalUnit, BlendUnitAffineOffset)
{
  std::vector<geometry_msgs::PoseStamped> plan = makeStraightLinePlan(0.0);
  const std::vector<geometry_msgs::PoseStamped> original = plan;
  const geometry_msgs::PoseStamped goal = makePose(1.02, 0.01, 0.3);

  const double blend_distance = 0.26;
  sbpl_lattice_planner::blendIntoGoal(goal, blend_distance, plan);

  ASSERT_EQ(plan.size(), original.size());

  // The final pose is exactly the requested goal.
  EXPECT_LT(positionError(plan.back().pose, goal.pose), 1e-9);
  EXPECT_LT(yawError(plan.back().pose, goal.pose), 1e-9);
  EXPECT_LT(std::abs(tf2::getYaw(plan.back().pose.orientation) - 0.3), 1e-9);

  const double dx = goal.pose.position.x - original.back().pose.position.x;
  const double dy = goal.pose.position.y - original.back().pose.position.y;
  const double dyaw = angles::shortest_angular_distance(tf2::getYaw(original.back().pose.orientation),
                                                        tf2::getYaw(goal.pose.orientation));
  EXPECT_LT(std::abs(dyaw - 0.3), 1e-9);

  // The walk back from the plan end stops once the next step would exceed the
  // blend distance: five 0.05 m steps (0.25 m <= 0.26 m) are the blended tail,
  // the sixth step (0.30 m > 0.26 m) is not.
  const size_t tail_poses = 5;
  const double tail_length = stepSum(tail_poses);
  ASSERT_LE(tail_length, blend_distance);
  ASSERT_GT(tail_length + CELL_SIZE, blend_distance);
  const size_t first_blended = plan.size() - 1 - tail_poses;
  ASSERT_EQ(first_blended, 15u);

  for (size_t i = 0; i < plan.size(); ++i)
  {
    const double arc_distance = CELL_SIZE * static_cast<double>(plan.size() - 1 - i);
    if (i < first_blended)
    {
      // Poses 0..14 are farther than goal_blend_distance from the plan end and
      // must not be touched at all.
      EXPECT_LT(positionError(plan[i].pose, original[i].pose), 1e-12) << "pose " << i << " should be unchanged";
      EXPECT_LT(yawError(plan[i].pose, original[i].pose), 1e-12) << "pose " << i << " should be unchanged";
    }
    else if (i + 1 < plan.size())
    {
      // Poses 15..19 are blended by an affine offset towards the goal; pose 15
      // sits exactly tail_length from the end, so it has w = 0.
      const double w = 1.0 - arc_distance / tail_length;
      const double expected_x = original[i].pose.position.x + w * dx;
      const double expected_y = original[i].pose.position.y + w * dy;
      const double expected_yaw = tf2::getYaw(original[i].pose.orientation) + w * dyaw;
      EXPECT_LT(std::hypot(plan[i].pose.position.x - expected_x, plan[i].pose.position.y - expected_y), 1e-9)
          << "pose " << i;
      EXPECT_LT(std::abs(angles::shortest_angular_distance(tf2::getYaw(plan[i].pose.orientation), expected_yaw)), 1e-9)
          << "pose " << i;
    }
  }
}

// A plan that ends with an in-place turn must not degenerate: the
// zero-displacement steps contribute no length but are part of the tail walk,
// so the trailing turn gets w = 1 and is translated to the goal position (with
// its yaw shifted by the full dyaw) instead of the plan jumping sideways in its
// last step.
TEST(SbplLatticePlannerBlendIntoGoalUnit, BlendUnitTrailingInPlaceTurn)
{
  // 16 straight poses at x = CELL_SIZE * i, yaw 0. The last one is the last
  // straight pose and defines the plan-end xy.
  const size_t straight_poses = 16;
  std::vector<geometry_msgs::PoseStamped> plan;
  for (size_t i = 0; i < straight_poses; ++i)
  {
    plan.push_back(makePose(CELL_SIZE * static_cast<double>(i), 0.0, 0.0));
  }
  const size_t last_straight = straight_poses - 1;
  const double end_xy = CELL_SIZE * static_cast<double>(last_straight);
  // Three in-place turn poses at the plan-end xy with yaw 0.0, 0.2, 0.4.
  const double turn_yaws[] = {0.0, 0.2, 0.4};
  const size_t turn_poses = sizeof(turn_yaws) / sizeof(turn_yaws[0]);
  for (size_t k = 0; k < turn_poses; ++k)
  {
    plan.push_back(makePose(end_xy, 0.0, turn_yaws[k]));
  }
  const size_t first_turn = last_straight + 1;
  const std::vector<geometry_msgs::PoseStamped> original = plan;

  const geometry_msgs::PoseStamped goal = makePose(0.79, 0.01, 0.45);
  // Deliberately off the step boundary: at 0.30 m, whether the sixth step still
  // fits depends on a 1-ULP difference in the accumulated length
  // (0.30000000000000004 > 0.30 versus 0.29999999999999993 <= 0.30).
  const double blend_distance = 0.31;
  sbpl_lattice_planner::blendIntoGoal(goal, blend_distance, plan);

  ASSERT_EQ(plan.size(), original.size());
  ASSERT_EQ(plan.size(), straight_poses + turn_poses);

  // The final pose is exactly the requested goal.
  EXPECT_LT(positionError(plan.back().pose, goal.pose), 1e-9);
  EXPECT_LT(yawError(plan.back().pose, goal.pose), 1e-9);

  // Walking back from the plan end: the three zero-displacement steps
  // (18->17->16->15) add no length, then CELL_SIZE steps accumulate until the
  // next one would exceed the blend distance. Six steps cover 0.30 m and fit
  // into 0.31 m; the seventh would reach 0.35 m and does not. The two
  // assertions pin the derived step count for the given blend_distance.
  const size_t tail_steps = static_cast<size_t>(blend_distance / CELL_SIZE);
  const double tail_length = stepSum(tail_steps);
  ASSERT_LE(tail_length, blend_distance);
  ASSERT_GT(tail_length + CELL_SIZE, blend_distance);

  // Pose `first_blended` is the boundary anchor of the tail: it sits exactly
  // tail_length from the plan end, so its weight is 1 - tail_length /
  // tail_length = 0 and it stays unchanged.
  const size_t first_blended = last_straight - tail_steps;

  const double dx = goal.pose.position.x - original.back().pose.position.x;
  const double dy = goal.pose.position.y - original.back().pose.position.y;
  const double dyaw = angles::shortest_angular_distance(tf2::getYaw(original.back().pose.orientation),
                                                        tf2::getYaw(goal.pose.orientation));
  EXPECT_LT(std::abs(dyaw - 0.05), 1e-9);

  // Poses outside the blended tail, plus the anchor (w = 0): unchanged.
  for (size_t i = 0; i <= first_blended; ++i)
  {
    EXPECT_LT(positionError(plan[i].pose, original[i].pose), 1e-12) << "pose " << i << " should be unchanged";
    EXPECT_LT(yawError(plan[i].pose, original[i].pose), 1e-12) << "pose " << i << " should be unchanged";
  }

  // Straight poses between the anchor and the last straight pose follow the
  // affine rule: w = 1 - arc_distance / tail_length with the path distance to
  // the plan end.
  for (size_t i = first_blended + 1; i < last_straight; ++i)
  {
    const double arc_distance = positionError(original[i].pose, original.back().pose);
    const double w = 1.0 - arc_distance / tail_length;
    const double expected_x = original[i].pose.position.x + w * dx;
    const double expected_y = original[i].pose.position.y + w * dy;
    const double expected_yaw = tf2::getYaw(original[i].pose.orientation) + w * dyaw;
    EXPECT_LT(std::hypot(plan[i].pose.position.x - expected_x, plan[i].pose.position.y - expected_y), 1e-9)
        << "pose " << i;
    EXPECT_LT(std::abs(angles::shortest_angular_distance(tf2::getYaw(plan[i].pose.orientation), expected_yaw)), 1e-9)
        << "pose " << i;
  }

  // The tail poses whose xy already equals the plan-end xy (the last straight
  // pose and the turn poses) have w = 1: their position is exactly the goal
  // position and their yaw is shifted by the full dyaw.
  for (size_t i = last_straight; i < plan.size(); ++i)
  {
    EXPECT_LT(positionError(plan[i].pose, goal.pose), 1e-9) << "pose " << i;
    const double expected_yaw = tf2::getYaw(original[i].pose.orientation) + dyaw;
    EXPECT_LT(std::abs(angles::shortest_angular_distance(tf2::getYaw(plan[i].pose.orientation), expected_yaw)), 1e-9)
        << "pose " << i;
  }

  // The in-place turn is preserved: the three turn poses end at
  // turn_yaws[k] + dyaw, a monotone sequence, instead of collapsing to one
  // heading.
  for (size_t k = 0; k < turn_poses; ++k)
  {
    const double expected_yaw = turn_yaws[k] + dyaw;
    EXPECT_LT(
        std::abs(angles::shortest_angular_distance(tf2::getYaw(plan[first_turn + k].pose.orientation), expected_yaw)),
        1e-9)
        << "pose " << first_turn + k;
  }
  EXPECT_LT(std::abs(tf2::getYaw(plan[first_turn].pose.orientation) - 0.05), 1e-9);
  EXPECT_LT(std::abs(tf2::getYaw(plan[first_turn + 1].pose.orientation) - 0.25), 1e-9);
  EXPECT_LT(std::abs(tf2::getYaw(plan[first_turn + 2].pose.orientation) - 0.45), 1e-9);
  for (size_t i = last_straight; i + 1 < plan.size(); ++i)
  {
    const double current_yaw = tf2::getYaw(plan[i].pose.orientation);
    const double next_yaw = tf2::getYaw(plan[i + 1].pose.orientation);
    EXPECT_GE(next_yaw - current_yaw, -1e-12) << "tail yaw not monotone at pose " << i;
  }
}

// Yaw 3.0 -> -3.0 must use the +0.28319 rad short way around, not 6.0+ rad.
TEST(SbplLatticePlannerBlendIntoGoalUnit, BlendUnitShortestAngularDistance)
{
  std::vector<geometry_msgs::PoseStamped> plan = makeStraightLinePlan(3.0);
  const std::vector<geometry_msgs::PoseStamped> original = plan;
  const geometry_msgs::PoseStamped goal = makePose(1.02, 0.01, -3.0);

  const double blend_distance = 0.31;
  sbpl_lattice_planner::blendIntoGoal(goal, blend_distance, plan);

  ASSERT_EQ(plan.size(), original.size());

  EXPECT_LT(positionError(plan.back().pose, goal.pose), 1e-9);
  EXPECT_LT(std::abs(tf2::getYaw(plan.back().pose.orientation) - (-3.0)), 1e-9);

  const double dyaw = angles::shortest_angular_distance(3.0, -3.0);
  EXPECT_LT(std::abs(dyaw - 0.283185307179586), 1e-9);

  // The walk back from the plan end accumulates CELL_SIZE steps: six of them
  // cover 0.30 m and fit into 0.31 m, the seventh would not. The blended tail
  // therefore starts at pose 14 (w = 0); the weight of an interior pose uses
  // that walk length, not the blend distance.
  const size_t tail_steps = static_cast<size_t>(blend_distance / CELL_SIZE);
  const double tail_length = stepSum(tail_steps);
  ASSERT_LE(tail_length, blend_distance);
  ASSERT_GT(tail_length + CELL_SIZE, blend_distance);
  const size_t first_blended = plan.size() - 1 - tail_steps;
  ASSERT_EQ(first_blended, 14u);

  // Pose 16 is four steps (0.20 m) from the plan end: w = 1 - 0.20 / 0.30.
  const size_t i = 16;
  const double arc_distance = CELL_SIZE * static_cast<double>(plan.size() - 1 - i);
  const double w = 1.0 - arc_distance / tail_length;
  const double expected_yaw = 3.0 + w * dyaw;
  EXPECT_LT(std::abs(angles::shortest_angular_distance(tf2::getYaw(plan[i].pose.orientation), expected_yaw)), 1e-9);
}

// goal_blend_distance = 0.0 is a no-op, including not setting the last pose to
// the goal.
TEST(SbplLatticePlannerBlendIntoGoalUnit, BlendUnitZeroDistanceNoOp)
{
  std::vector<geometry_msgs::PoseStamped> plan = makeStraightLinePlan(0.0);
  const std::vector<geometry_msgs::PoseStamped> original = plan;
  const geometry_msgs::PoseStamped goal = makePose(1.02, 0.01, 0.3);

  sbpl_lattice_planner::blendIntoGoal(goal, 0.0, plan);

  ASSERT_EQ(plan.size(), original.size());
  for (size_t i = 0; i < plan.size(); ++i)
  {
    EXPECT_LT(positionError(plan[i].pose, original[i].pose), 1e-12) << "pose " << i;
    EXPECT_LT(yawError(plan[i].pose, original[i].pose), 1e-12) << "pose " << i;
  }
}

// A blend distance below one step size (0.01 m < 0.05 m) makes the walk back
// break immediately: only the final pose changes (it becomes the goal), every
// other pose stays bit-identical.
TEST(SbplLatticePlannerBlendIntoGoalUnit, BlendUnitBlendDistanceBelowStepSize)
{
  std::vector<geometry_msgs::PoseStamped> plan = makeStraightLinePlan(0.0);
  const std::vector<geometry_msgs::PoseStamped> original = plan;
  const geometry_msgs::PoseStamped goal = makePose(1.02, 0.01, 0.3);

  const double blend_distance = 0.01;
  ASSERT_LT(blend_distance, CELL_SIZE);
  sbpl_lattice_planner::blendIntoGoal(goal, blend_distance, plan);

  ASSERT_EQ(plan.size(), original.size());
  EXPECT_LT(positionError(plan.back().pose, goal.pose), 1e-12);
  EXPECT_LT(yawError(plan.back().pose, goal.pose), 1e-12);

  for (size_t i = 0; i + 1 < plan.size(); ++i)
  {
    EXPECT_LT(positionError(plan[i].pose, original[i].pose), 1e-12) << "pose " << i;
    EXPECT_LT(yawError(plan[i].pose, original[i].pose), 1e-12) << "pose " << i;
  }
}

// A one-pose plan becomes exactly the goal pose and does not crash.
TEST(SbplLatticePlannerBlendIntoGoalUnit, BlendUnitSinglePosePlan)
{
  std::vector<geometry_msgs::PoseStamped> plan;
  plan.push_back(makePose(0.0, 0.0, 0.0));
  const geometry_msgs::PoseStamped goal = makePose(1.02, 0.01, 0.3);

  sbpl_lattice_planner::blendIntoGoal(goal, 0.3, plan);

  ASSERT_EQ(plan.size(), 1u);
  EXPECT_LT(positionError(plan.back().pose, goal.pose), 1e-12);
  EXPECT_LT(yawError(plan.back().pose, goal.pose), 1e-12);
}

// An empty plan stays empty and does not crash.
TEST(SbplLatticePlannerBlendIntoGoalUnit, BlendUnitEmptyPlan)
{
  std::vector<geometry_msgs::PoseStamped> plan;
  const geometry_msgs::PoseStamped goal = makePose(1.02, 0.01, 0.3);

  sbpl_lattice_planner::blendIntoGoal(goal, 0.3, plan);

  EXPECT_TRUE(plan.empty());
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_sbpl_lattice_planner_exact_goal");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
