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

#include <autoware_utils/geometry/boost_geometry.h>
#include <autoware_utils/trajectory/trajectory.h>

#include <tf2/LinearMath/Quaternion.h>

namespace
{
using autoware_planning_msgs::Trajectory;
using autoware_utils::createPoint;
using autoware_utils::createQuaternionFromRPY;
using autoware_utils::transformPoint;

constexpr double epsilon = 1e-6;

geometry_msgs::Pose createPose(double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::Pose p;
  p.position = createPoint(x, y, z);
  tf2::convert(createQuaternionFromRPY(roll, pitch, yaw), p.orientation);
  return p;
}

template <class T>
T generateTestTrajectory(
  const size_t num_points, const double point_interval, const double vel = 0.0,
  const double init_theta = 0.0, const double delta_theta = 0.0)
{
  using Point = typename T::_points_type::value_type;

  T traj;
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = init_theta + i * delta_theta;
    const double x = i * point_interval * std::cos(theta);
    const double y = i * point_interval * std::sin(theta);

    Point p;
    p.pose = createPose(x, y, 0.0, 0.0, 0.0, theta);
    p.twist.linear.x = vel;
    traj.points.push_back(p);
  }

  return traj;
}

template <class T>
void updateTrajectoryVelocityAt(T & points, const size_t idx, const double vel)
{
  points.at(idx).twist.linear.x = vel;
}
}  // namespace

TEST(trajectory, validateNonEmpty)
{
  using autoware_utils::validateNonEmpty;

  // Empty
  EXPECT_THROW(validateNonEmpty(Trajectory{}.points), std::invalid_argument);

  // Non-empty
  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  EXPECT_NO_THROW(validateNonEmpty(traj.points));
}

TEST(trajectory, searchZeroVelocityIndex)
{
  using autoware_utils::searchZeroVelocityIndex;

  // Empty
  EXPECT_THROW(searchZeroVelocityIndex(Trajectory{}.points), std::invalid_argument);

  // No zero velocity point
  {
    const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    EXPECT_FALSE(searchZeroVelocityIndex(traj.points));
  }

  // Only start point is zero
  {
    const size_t idx_ans = 0;

    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    updateTrajectoryVelocityAt(traj.points, idx_ans, 0.0);

    EXPECT_EQ(*searchZeroVelocityIndex(traj.points), idx_ans);
  }

  // Only end point is zero
  {
    const size_t idx_ans = 9;

    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    updateTrajectoryVelocityAt(traj.points, idx_ans, 0.0);

    EXPECT_EQ(*searchZeroVelocityIndex(traj.points), idx_ans);
  }

  // Only middle point is zero
  {
    const size_t idx_ans = 5;

    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    updateTrajectoryVelocityAt(traj.points, idx_ans, 0.0);

    EXPECT_EQ(*searchZeroVelocityIndex(traj.points), idx_ans);
  }

  // Two points are zero
  {
    const size_t idx_ans = 3;

    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    updateTrajectoryVelocityAt(traj.points, idx_ans, 0.0);
    updateTrajectoryVelocityAt(traj.points, 6, 0.0);

    EXPECT_EQ(*searchZeroVelocityIndex(traj.points), idx_ans);
  }

  // Negative velocity point is before zero velocity point
  {
    const size_t idx_ans = 3;

    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    updateTrajectoryVelocityAt(traj.points, 2, -1.0);
    updateTrajectoryVelocityAt(traj.points, idx_ans, 0.0);

    EXPECT_EQ(*searchZeroVelocityIndex(traj.points), idx_ans);
  }
}

TEST(trajectory, findNearestIndex_StraightTrajectory)
{
  using autoware_utils::findNearestIndex;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_THROW(
    findNearestIndex(Trajectory{}.points, geometry_msgs::Point{}), std::invalid_argument);

  // Start point
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(0.0, 0.0, 0.0)), 0);

  // End point
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(9.0, 0.0, 0.0)), 9);

  // Boundary conditions
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(0.5, 0.0, 0.0)), 0);
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(0.51, 0.0, 0.0)), 1);

  // Point before start point
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(-4.0, 5.0, 0.0)), 0);

  // Point after end point
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(100.0, -3.0, 0.0)), 9);

  // Random cases
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(2.4, 1.3, 0.0)), 2);
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(4.0, 0.0, 0.0)), 4);
}

TEST(trajectory, findNearestIndex_CurvedTrajectory)
{
  using autoware_utils::findNearestIndex;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 0.0, 0.0, 0.1);

  // Random cases
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(5.1, 3.4, 0.0)), 6);
}

TEST(trajectory, findNearestIndexWithYawThreshold)
{
  using autoware_utils::findNearestIndex;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_THROW(
    findNearestIndex(Trajectory{}.points, geometry_msgs::Pose{}, {}), std::invalid_argument);

  // Start point
  EXPECT_EQ(*findNearestIndex(traj.points, createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), M_PI), 0);

  // Start point with 180 degrees angle deviations
  EXPECT_FALSE(findNearestIndex(traj.points, createPose(0.0, 0.0, 0.0, 0.0, 0.0, M_PI), M_PI));

  // Start point with 270(-90) degrees angle deviations
  EXPECT_EQ(
    *findNearestIndex(traj.points, createPose(0.0, 0.0, 0.0, 0.0, 0.0, 1.5 * M_PI), M_PI), 0);

  // End point
  EXPECT_EQ(*findNearestIndex(traj.points, createPose(9.0, 0.0, 0.0, 0.0, 0.0, 0.0), M_PI), 9);

  // Boundary conditions
  EXPECT_EQ(*findNearestIndex(traj.points, createPose(0.5, 0.0, 0.0, 0.0, 0.0, 0.0), M_PI), 0);
  EXPECT_EQ(*findNearestIndex(traj.points, createPose(0.51, 0.0, 0.0, 0.0, 0.0, 0.0), M_PI), 1);

  // Point before start point
  EXPECT_EQ(*findNearestIndex(traj.points, createPose(-4.0, 5.0, 0.0, 0.0, 0.0, 0.0), M_PI), 0);

  // Point after end point
  EXPECT_EQ(*findNearestIndex(traj.points, createPose(100.0, -3.0, 0.0, 0.0, 0.0, 0.0), M_PI), 9);

  // Random cases
  EXPECT_EQ(*findNearestIndex(traj.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.0)), 2);
  EXPECT_EQ(*findNearestIndex(traj.points, createPose(4.0, 0.0, 0.0, 0.0, 0.0, 0.0), M_PI), 4);
}

TEST(trajectory, findNearestSegmentIndex)
{
  using autoware_utils::findNearestSegmentIndex;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_THROW(findNearestSegmentIndex(Trajectory{}.points, {}), std::invalid_argument);

  // Start point
  EXPECT_EQ(findNearestSegmentIndex(traj.points, createPoint(0.0, 0.0, 0.0)), 0);

  // End point
  EXPECT_EQ(findNearestSegmentIndex(traj.points, createPoint(9.0, 0.0, 0.0)), 8);

  // Boundary conditions
  EXPECT_EQ(findNearestSegmentIndex(traj.points, createPoint(1.0, 0.0, 0.0)), 0);
  EXPECT_EQ(findNearestSegmentIndex(traj.points, createPoint(1.1, 0.0, 0.0)), 1);

  // Point before start point
  EXPECT_EQ(findNearestSegmentIndex(traj.points, createPoint(-4.0, 5.0, 0.0)), 0);

  // Point after end point
  EXPECT_EQ(findNearestSegmentIndex(traj.points, createPoint(100.0, -3.0, 0.0)), 8);

  // Random cases
  EXPECT_EQ(findNearestSegmentIndex(traj.points, createPoint(2.4, 1.0, 0.0)), 2);
  EXPECT_EQ(findNearestSegmentIndex(traj.points, createPoint(4.0, 0.0, 0.0)), 3);

  // Two nearest trajectory points are not the edges of the nearest segment.
  std::vector<geometry_msgs::Point> sparse_points{
    createPoint(0.0, 0.0, 0.0),
    createPoint(10.0, 0.0, 0.0),
    createPoint(11.0, 0.0, 0.0),
  };
  EXPECT_EQ(findNearestSegmentIndex(sparse_points, createPoint(9.0, 1.0, 0.0)), 0);
}

TEST(trajectory, calcLongitudinalOffsetToSegment_StraightTrajectory)
{
  using autoware_utils::calcLongitudinalOffsetToSegment;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_THROW(calcLongitudinalOffsetToSegment(Trajectory{}.points, {}, {}), std::invalid_argument);

  // Out of range
  EXPECT_THROW(calcLongitudinalOffsetToSegment(traj.points, -1, {}), std::out_of_range);
  EXPECT_THROW(
    calcLongitudinalOffsetToSegment(traj.points, traj.points.size() - 1, {}), std::out_of_range);

  // Too close points in trajectory
  {
    const auto invalid_traj = generateTestTrajectory<Trajectory>(10, 1e-10);
    const auto p = createPoint(3.0, 0.0, 0.0);
    EXPECT_THROW(calcLongitudinalOffsetToSegment(invalid_traj.points, 3, p), std::runtime_error);
  }

  // Same point
  EXPECT_NEAR(
    calcLongitudinalOffsetToSegment(traj.points, 3, createPoint(3.0, 0.0, 0.0)), 0.0, epsilon);

  // Point before start point
  EXPECT_NEAR(
    calcLongitudinalOffsetToSegment(traj.points, 6, createPoint(-3.9, 3.0, 0.0)), -9.9, epsilon);

  // Point after start point
  EXPECT_NEAR(
    calcLongitudinalOffsetToSegment(traj.points, 7, createPoint(13.3, -10.0, 0.0)), 6.3, epsilon);

  // Random cases
  EXPECT_NEAR(
    calcLongitudinalOffsetToSegment(traj.points, 2, createPoint(4.3, 7.0, 0.0)), 2.3, epsilon);
  EXPECT_NEAR(
    calcLongitudinalOffsetToSegment(traj.points, 8, createPoint(1.0, 3.0, 0.0)), -7, epsilon);
}

TEST(trajectory, calcLongitudinalOffsetToSegment_CurveTrajectory)
{
  using autoware_utils::calcLongitudinalOffsetToSegment;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 0.0, 0.0, 0.1);

  // Random cases
  EXPECT_NEAR(
    calcLongitudinalOffsetToSegment(traj.points, 2, createPoint(2.0, 0.5, 0.0)), 0.083861449,
    epsilon);
}

TEST(trajectory, calcSignedArcLengthFromIndexToIndex)
{
  using autoware_utils::calcSignedArcLength;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_THROW(calcSignedArcLength(Trajectory{}.points, {}, {}), std::invalid_argument);

  // Out of range
  EXPECT_THROW(calcSignedArcLength(traj.points, -1, 1), std::out_of_range);
  EXPECT_THROW(calcSignedArcLength(traj.points, 0, traj.points.size() + 1), std::out_of_range);

  // Same point
  EXPECT_NEAR(calcSignedArcLength(traj.points, 3, 3), 0, epsilon);

  // Forward
  EXPECT_NEAR(calcSignedArcLength(traj.points, 0, 3), 3, epsilon);

  // Backward
  EXPECT_NEAR(calcSignedArcLength(traj.points, 9, 5), -4, epsilon);
}

TEST(trajectory, calcSignedArcLengthFromPointToIndex)
{
  using autoware_utils::calcSignedArcLength;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_THROW(calcSignedArcLength(Trajectory{}.points, {}, {}), std::invalid_argument);

  // Same point
  EXPECT_NEAR(calcSignedArcLength(traj.points, createPoint(3.0, 0.0, 0.0), 3), 0, epsilon);

  // Forward
  EXPECT_NEAR(calcSignedArcLength(traj.points, createPoint(0.0, 0.0, 0.0), 3), 3, epsilon);

  // Backward
  EXPECT_NEAR(calcSignedArcLength(traj.points, createPoint(9.0, 0.0, 0.0), 5), -4, epsilon);

  // Point before start point
  EXPECT_NEAR(calcSignedArcLength(traj.points, createPoint(-3.9, 3.0, 0.0), 6), 9.9, epsilon);

  // Point after end point
  EXPECT_NEAR(calcSignedArcLength(traj.points, createPoint(13.3, -10.0, 0.0), 7), -6.3, epsilon);

  // Random cases
  EXPECT_NEAR(calcSignedArcLength(traj.points, createPoint(1.0, 3.0, 0.0), 9), 8, epsilon);
  EXPECT_NEAR(calcSignedArcLength(traj.points, createPoint(4.3, 7.0, 0.0), 2), -2.3, epsilon);
}

TEST(trajectory, calcSignedArcLengthFromPointToPoint)
{
  using autoware_utils::calcSignedArcLength;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_THROW(calcSignedArcLength(Trajectory{}.points, {}, {}), std::invalid_argument);

  // Same point
  {
    const auto p = createPoint(3.0, 0.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p, p), 0, epsilon);
  }

  // Forward
  {
    const auto p1 = createPoint(0.0, 0.0, 0.0);
    const auto p2 = createPoint(3.0, 1.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, p2), 3, epsilon);
  }

  // Backward
  {
    const auto p1 = createPoint(8.0, 0.0, 0.0);
    const auto p2 = createPoint(9.0, 0.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, p2), 1, epsilon);
  }

  // Point before start point
  {
    const auto p1 = createPoint(-3.9, 3.0, 0.0);
    const auto p2 = createPoint(6.0, -10.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, p2), 9.9, epsilon);
  }

  // Point after end point
  {
    const auto p1 = createPoint(7.0, -5.0, 0.0);
    const auto p2 = createPoint(13.3, -10.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, p2), 6.3, epsilon);
  }

  // Point before start point and after end point
  {
    const auto p1 = createPoint(-4.3, 10.0, 0.0);
    const auto p2 = createPoint(13.8, -1.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, p2), 18.1, epsilon);
  }

  // Random cases
  {
    const auto p1 = createPoint(1.0, 3.0, 0.0);
    const auto p2 = createPoint(9.0, -1.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, p2), 8, epsilon);
  }
  {
    const auto p1 = createPoint(4.3, 7.0, 0.0);
    const auto p2 = createPoint(2.0, 3.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, p2), -2.3, epsilon);
  }
}
