/*
 * Copyright 2021 TierIV. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>

#include <autoware_utils/trajectory/trajectory.h>

#include <tf2/LinearMath/Quaternion.h>

namespace
{
autoware_planning_msgs::Path generateTestPath()
{
  // Generate Straight Path
  autoware_planning_msgs::Path path;

  for (double x = 0.0; x <= 10.0; x += 1.0) {
    autoware_planning_msgs::PathPoint p;
    p.pose.position = autoware_utils::createPoint(x, 0.0, 0.0);
    tf2::convert(autoware_utils::createQuaternionFromRPY(0.0, 0.0, 0.0), p.pose.orientation);

    path.points.push_back(p);
  }

  return path;
}

autoware_planning_msgs::Trajectory generateTestTrajectory()
{
  // Generate Straight Trajectory
  autoware_planning_msgs::Trajectory traj;

  for (double x = 0.0; x <= 10.0; x += 1.0) {
    autoware_planning_msgs::TrajectoryPoint p;
    p.pose.position = autoware_utils::createPoint(x, 0.0, 0.0);
    tf2::convert(autoware_utils::createQuaternionFromRPY(0.0, 0.0, 0.0), p.pose.orientation);

    traj.points.push_back(p);
  }

  return traj;
}

}  // namespace

TEST(Trajectory, calcPathClosestPointIndex)
{
  using autoware_utils::findClosestIndex;

  autoware_planning_msgs::Path path = generateTestPath();
  autoware_planning_msgs::Trajectory traj = generateTestTrajectory();

  // Initial Point
  {
    geometry_msgs::Point p = autoware_utils::createPoint(0.0, 0.0, 0.0);
    auto closest_path_point = findClosestIndex(path, p);
    auto closest_traj_point = findClosestIndex(traj, p);
    EXPECT_EQ(*closest_path_point, 0);
    EXPECT_EQ(*closest_traj_point, 0);
  }

  // End Point
  {
    geometry_msgs::Point p = autoware_utils::createPoint(10.0, 0.0, 0.0);
    auto closest_path_point = findClosestIndex(path, p);
    auto closest_traj_point = findClosestIndex(traj, p);
    EXPECT_EQ(*closest_path_point, 10);
    EXPECT_EQ(*closest_traj_point, 10);
  }

  // Middle Point
  {
    geometry_msgs::Point p = autoware_utils::createPoint(0.5, 0.0, 0.0);
    auto closest_path_point = findClosestIndex(path, p);
    auto closest_traj_point = findClosestIndex(traj, p);
    EXPECT_EQ(*closest_path_point, 0);
    EXPECT_EQ(*closest_traj_point, 0);
  }

  // Arbitrary Point inside the path
  {
    geometry_msgs::Point p = autoware_utils::createPoint(4.0, 0.0, 0.0);
    auto closest_path_point = findClosestIndex(path, p);
    auto closest_traj_point = findClosestIndex(traj, p);
    EXPECT_EQ(*closest_path_point, 4);
    EXPECT_EQ(*closest_traj_point, 4);
  }

  // Point outside the path on the left
  {
    geometry_msgs::Point p = autoware_utils::createPoint(-4.0, 5.0, 0.0);
    auto closest_path_point = findClosestIndex(path, p);
    auto closest_traj_point = findClosestIndex(traj, p);
    EXPECT_EQ(*closest_path_point, 0);
    EXPECT_EQ(*closest_traj_point, 0);
  }

  // Point outside the path on the right
  {
    geometry_msgs::Point p = autoware_utils::createPoint(100.0, -3.0, 0.0);
    auto closest_path_point = findClosestIndex(path, p);
    auto closest_traj_point = findClosestIndex(traj, p);
    EXPECT_EQ(*closest_path_point, 10);
    EXPECT_EQ(*closest_traj_point, 10);
  }

  // Arbitrary Point inside the path
  {
    geometry_msgs::Point p = autoware_utils::createPoint(2.431, 1.391, 0.0);
    auto closest_path_point = findClosestIndex(path, p);
    auto closest_traj_point = findClosestIndex(traj, p);
    EXPECT_EQ(*closest_path_point, 2);
    EXPECT_EQ(*closest_traj_point, 2);
  }

  // Inf Point
  {
    geometry_msgs::Point p = autoware_utils::createPoint(
      std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0);
    auto closest_path_point = findClosestIndex(path, p);
    auto closest_traj_point = findClosestIndex(traj, p);
    EXPECT_TRUE(closest_path_point);
    EXPECT_TRUE(closest_traj_point);
  }

  // Empty Point
  {
    geometry_msgs::Point p = autoware_utils::createPoint(0.0, 0.0, 0.0);
    autoware_planning_msgs::Path empty_path;
    autoware_planning_msgs::Trajectory empty_traj;
    auto closest_path_point = findClosestIndex(empty_path, p);
    auto closest_traj_point = findClosestIndex(empty_traj, p);
    EXPECT_FALSE(closest_path_point);
    EXPECT_FALSE(closest_traj_point);
  }
}

TEST(Trajectory, calcTrajectoryClosestPoint)
{
  using autoware_utils::findClosestIndex;

  autoware_planning_msgs::Path path = generateTestPath();
  autoware_planning_msgs::Trajectory traj = generateTestTrajectory();

  // Initial Pose
  {
    geometry_msgs::Pose p;
    p.position = autoware_utils::createPoint(0.0, 0.0, 0.0);
    tf2::convert(autoware_utils::createQuaternionFromRPY(0.0, 0.0, 0.0), p.orientation);
    auto closest_path_point = findClosestIndex(path, p);
    auto closest_traj_point = findClosestIndex(traj, p);
    EXPECT_EQ(*closest_path_point, 0);
    EXPECT_EQ(*closest_traj_point, 0);
  }

  // Initial Pose with 90 degrees angle deviations
  {
    geometry_msgs::Pose p;
    p.position = autoware_utils::createPoint(0.0, 0.0, 0.0);
    tf2::convert(autoware_utils::createQuaternionFromRPY(0.0, 0.0, M_PI / 2), p.orientation);
    auto closest_path_point = findClosestIndex(path, p, M_PI);
    auto closest_traj_point = findClosestIndex(traj, p, M_PI);
    EXPECT_EQ(*closest_path_point, 0);
    EXPECT_EQ(*closest_traj_point, 0);
  }

  // Initial Pose with 180 degrees angle deviations
  {
    geometry_msgs::Pose p;
    p.position = autoware_utils::createPoint(0.0, 0.0, 0.0);
    tf2::convert(autoware_utils::createQuaternionFromRPY(0.0, 0.0, M_PI), p.orientation);
    auto closest_path_point = findClosestIndex(path, p, M_PI);
    auto closest_traj_point = findClosestIndex(traj, p, M_PI);
    EXPECT_FALSE(closest_path_point);
    EXPECT_FALSE(closest_traj_point);
  }

  // Initial Pose with 270 degrees angle deviations
  {
    geometry_msgs::Pose p;
    p.position = autoware_utils::createPoint(0.0, 0.0, 0.0);
    tf2::convert(autoware_utils::createQuaternionFromRPY(0.0, 0.0, 1.5 * M_PI), p.orientation);
    auto closest_path_point = findClosestIndex(path, p, M_PI);
    auto closest_traj_point = findClosestIndex(traj, p, M_PI);
    EXPECT_EQ(*closest_path_point, 0);
    EXPECT_EQ(*closest_traj_point, 0);
  }

  // End Point
  {
    geometry_msgs::Pose p;
    p.position = autoware_utils::createPoint(10.0, 0.0, 0.0);
    tf2::convert(autoware_utils::createQuaternionFromRPY(0.0, 0.0, 0.0), p.orientation);
    auto closest_path_point = findClosestIndex(path, p);
    auto closest_traj_point = findClosestIndex(traj, p);
    EXPECT_EQ(*closest_path_point, 10);
    EXPECT_EQ(*closest_traj_point, 10);
  }

  // Middle Point
  {
    geometry_msgs::Pose p;
    p.position = autoware_utils::createPoint(0.5, 0.0, 0.0);
    tf2::convert(autoware_utils::createQuaternionFromRPY(0.0, 0.0, 0.0), p.orientation);
    auto closest_path_point = findClosestIndex(path, p);
    auto closest_traj_point = findClosestIndex(traj, p);
    EXPECT_EQ(*closest_path_point, 0);
    EXPECT_EQ(*closest_traj_point, 0);
  }

  // Arbitrary Point inside the path
  {
    geometry_msgs::Pose p;
    p.position = autoware_utils::createPoint(4.0, 0.0, 0.0);
    tf2::convert(autoware_utils::createQuaternionFromRPY(0.0, 0.0, 0.0), p.orientation);
    auto closest_path_point = findClosestIndex(path, p);
    auto closest_traj_point = findClosestIndex(traj, p);
    EXPECT_EQ(*closest_path_point, 4);
    EXPECT_EQ(*closest_traj_point, 4);
  }

  // Point outside the path on the left
  {
    geometry_msgs::Pose p;
    p.position = autoware_utils::createPoint(-4.0, 5.0, 0.0);
    tf2::convert(autoware_utils::createQuaternionFromRPY(0.0, 0.0, 0.0), p.orientation);
    auto closest_path_point = findClosestIndex(path, p);
    auto closest_traj_point = findClosestIndex(traj, p);
    EXPECT_EQ(*closest_path_point, 0);
    EXPECT_EQ(*closest_traj_point, 0);
  }

  // Point outside the path on the right
  {
    geometry_msgs::Pose p;
    p.position = autoware_utils::createPoint(100.0, -3.0, 0.0);
    tf2::convert(autoware_utils::createQuaternionFromRPY(0.0, 0.0, 0.0), p.orientation);
    auto closest_path_point = findClosestIndex(path, p);
    auto closest_traj_point = findClosestIndex(traj, p);
    EXPECT_EQ(*closest_path_point, 10);
    EXPECT_EQ(*closest_traj_point, 10);
  }

  // Arbitrary Point inside the path
  {
    geometry_msgs::Pose p;
    p.position = autoware_utils::createPoint(2.431, 1.391, 0.0);
    tf2::convert(autoware_utils::createQuaternionFromRPY(0.0, 0.0, 0.0), p.orientation);
    auto closest_path_point = findClosestIndex(path, p);
    auto closest_traj_point = findClosestIndex(traj, p);
    EXPECT_EQ(*closest_path_point, 2);
    EXPECT_EQ(*closest_traj_point, 2);
  }

  // Inf Point
  {
    geometry_msgs::Pose p;
    p.position = autoware_utils::createPoint(
      std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), 0.0);
    tf2::convert(autoware_utils::createQuaternionFromRPY(0.0, 0.0, 0.0), p.orientation);
    auto closest_path_point = findClosestIndex(path, p);
    auto closest_traj_point = findClosestIndex(traj, p);
    EXPECT_FALSE(closest_path_point);
    EXPECT_FALSE(closest_traj_point);
  }

  // Empty Point
  {
    geometry_msgs::Pose p;
    p.position = autoware_utils::createPoint(0.0, 0.0, 0.0);
    tf2::convert(autoware_utils::createQuaternionFromRPY(0.0, 0.0, 0.0), p.orientation);
    autoware_planning_msgs::Path empty_path;
    autoware_planning_msgs::Trajectory empty_traj;
    auto closest_path_point = findClosestIndex(empty_path, p);
    auto closest_traj_point = findClosestIndex(empty_traj, p);
    EXPECT_FALSE(closest_path_point);
    EXPECT_FALSE(closest_traj_point);
  }
}
