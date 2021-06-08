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

#pragma once

#include <limits>

#include <boost/optional.hpp>

#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>

#include <autoware_utils/geometry/geometry.h>
#include <autoware_utils/geometry/pose_deviation.h>

namespace autoware_utils
{
template <class T>
boost::optional<size_t> findClosestIndex(const T & points, const geometry_msgs::Point & point)
{
  if (points.empty()) {
    return boost::none;
  }

  double dist_min = std::numeric_limits<double>::max();
  size_t index_min = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto dist = calcDistance2d(points.at(i), point);
    if (dist < dist_min) {
      dist_min = dist;
      index_min = i;
    }
  }
  return boost::optional<size_t>(index_min);
}

template <>
inline boost::optional<size_t> findClosestIndex(
  const autoware_planning_msgs::Path & path, const geometry_msgs::Point & point)
{
  return findClosestIndex(path.points, point);
}

template <>
inline boost::optional<size_t> findClosestIndex(
  const autoware_planning_msgs::Trajectory & traj, const geometry_msgs::Point & point)
{
  return findClosestIndex(traj.points, point);
}

template <class T>
boost::optional<size_t> findClosestIndex(
  const T & points, const geometry_msgs::Pose & pose,
  const double delta_yaw_threshold = std::numeric_limits<double>::max())
{
  if (points.empty()) {
    return boost::none;
  }

  double dist_min = std::numeric_limits<double>::max();
  bool is_closest_found = false;
  size_t index_min = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto delta_yaw = calcYawDeviation(getPose(points.at(i)), pose);
    if (std::fabs(delta_yaw) < delta_yaw_threshold) {
      const auto dist = calcDistance2d(points.at(i), pose);
      if (dist < dist_min) {
        dist_min = dist;
        index_min = i;
        is_closest_found = true;
      }
    }
  }
  return is_closest_found ? boost::optional<size_t>(index_min) : boost::none;
}

template <>
inline boost::optional<size_t> findClosestIndex(
  const autoware_planning_msgs::Path & path, const geometry_msgs::Pose & pose,
  const double delta_yaw_threshold)
{
  return findClosestIndex(path.points, pose, delta_yaw_threshold);
}

template <>
inline boost::optional<size_t> findClosestIndex(
  const autoware_planning_msgs::Trajectory & traj, const geometry_msgs::Pose & pose,
  const double delta_yaw_threshold)
{
  return findClosestIndex(traj.points, pose, delta_yaw_threshold);
}
}  // namespace autoware_utils
