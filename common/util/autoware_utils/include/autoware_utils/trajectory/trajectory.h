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
#include <stdexcept>

#include <boost/optional.hpp>

#include <autoware_utils/geometry/geometry.h>
#include <autoware_utils/geometry/pose_deviation.h>

namespace autoware_utils
{
template <class T>
void validateNonEmpty(const T & points)
{
  if (points.empty()) {
    throw std::invalid_argument("Points is empty.");
  }
}

template <class T>
boost::optional<size_t> searchZeroVelocityIndex(const T & points_with_twist)
{
  validateNonEmpty(points_with_twist);

  constexpr double epsilon = 1e-3;
  for (size_t i = 0; i < points_with_twist.size(); ++i) {
    if (std::fabs(points_with_twist.at(i).twist.linear.x) < epsilon) {
      return i;
    }
  }

  return {};
}

template <class T>
size_t findNearestIndex(const T & points, const geometry_msgs::Point & point)
{
  validateNonEmpty(points);

  double dist_min = std::numeric_limits<double>::max();
  size_t index_min = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto dist = calcDistance2d(points.at(i), point);
    if (dist < dist_min) {
      dist_min = dist;
      index_min = i;
    }
  }
  return index_min;
}

template <class T>
boost::optional<size_t> findNearestIndex(
  const T & points, const geometry_msgs::Pose & pose,
  const double yaw_threshold = std::numeric_limits<double>::max())
{
  validateNonEmpty(points);

  double dist_min = std::numeric_limits<double>::max();
  bool is_nearest_found = false;
  size_t index_min = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto delta_yaw = calcYawDeviation(getPose(points.at(i)), pose);
    if (std::fabs(delta_yaw) < yaw_threshold) {
      const auto dist = calcDistance2d(points.at(i), pose);
      if (dist < dist_min) {
        dist_min = dist;
        index_min = i;
        is_nearest_found = true;
      }
    }
  }
  return is_nearest_found ? boost::optional<size_t>(index_min) : boost::none;
}

/**
  * @brief calculate length along trajectory from seg_idx point to nearest point to p_target on trajectory
  *        If seg_idx point is after that nearest point, length is negative
  * @param points points of trajectory, path, ...
  * @param seg_idx segment index of point at beginning of length
  * @param p_target target point at end of length
  * @return signed length
  */
template <class T>
double calcLongitudinalOffsetToSegment(
  const T & points, const size_t seg_idx, const geometry_msgs::Point & p_target)
{
  validateNonEmpty(points);

  const auto p_front = getPoint(points.at(seg_idx));
  const auto p_back = getPoint(points.at(seg_idx + 1));

  const Eigen::Vector3d segment_vec{p_back.x - p_front.x, p_back.y - p_front.y, 0};
  const Eigen::Vector3d target_vec{p_target.x - p_front.x, p_target.y - p_front.y, 0};

  constexpr double epsilon = 1e-6;
  if (segment_vec.norm() < epsilon) {
    throw std::runtime_error("Too close points are given.");
  }

  return segment_vec.dot(target_vec) / segment_vec.norm();
}

/**
  * @brief find nearest segment index to point
  *        segment is straight path between two continuous points of trajectory
  *        When point is on a trajectory point whose index is nearest_idx, return nearest_idx - 1
  * @param points points of trajectory
  * @param point point to which to find nearest segment index
  * @return nearest index
  */
template <class T>
size_t findNearestSegmentIndex(const T & points, const geometry_msgs::Point & point)
{
  const size_t nearest_idx = findNearestIndex(points, point);

  if (nearest_idx == 0) {
    return 0;
  } else if (nearest_idx == points.size() - 1) {
    return points.size() - 2;
  }

  const double signed_length = calcLongitudinalOffsetToSegment(points, nearest_idx, point);

  if (signed_length <= 0) {
    return nearest_idx - 1;
  }

  return nearest_idx;
}

/**
  * @brief calcSignedArcLength from index to index
  */
template <class T>
double calcSignedArcLength(const T & points, const size_t src_idx, const size_t dst_idx)
{
  validateNonEmpty(points);

  if (src_idx > dst_idx) {
    return -calcSignedArcLength(points, dst_idx, src_idx);
  }

  double dist_sum = 0.0;
  for (size_t i = src_idx; i < dst_idx; ++i) {
    dist_sum += calcDistance2d(points.at(i), points.at(i + 1));
  }
  return dist_sum;
}

/**
  * @brief calcSignedArcLength from point to index
  */
template <class T>
double calcSignedArcLength(
  const T & points, const geometry_msgs::Point & src_point, const size_t & dst_idx)
{
  validateNonEmpty(points);

  const size_t src_seg_idx = findNearestSegmentIndex(points, src_point);

  const double signed_length_on_traj = calcSignedArcLength(points, src_seg_idx, dst_idx);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);

  return signed_length_on_traj - signed_length_src_offset;
}

/**
  * @brief calcSignedArcLength from point to point
  */
template <class T>
double calcSignedArcLength(
  const T & points, const geometry_msgs::Point & src_point, const geometry_msgs::Point & dst_point)
{
  validateNonEmpty(points);

  const size_t src_seg_idx = findNearestSegmentIndex(points, src_point);
  const size_t dst_seg_idx = findNearestSegmentIndex(points, dst_point);

  const double signed_length_on_traj = calcSignedArcLength(points, src_seg_idx, dst_seg_idx);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);
  const double signed_length_dst_offset =
    calcLongitudinalOffsetToSegment(points, dst_seg_idx, dst_point);

  return signed_length_on_traj - signed_length_src_offset + signed_length_dst_offset;
}
}  // namespace autoware_utils
