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
 * The first three tests reproduce the documented endpoint problem and must pass
 * with blending disabled (the default, and explicitly 0.0). The last test is
 * the acceptance test for goal_blend_distance and fails until the blending is
 * implemented.
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

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_sbpl_lattice_planner_exact_goal");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
