/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include "behavior_path_planner/path_shifter/path_shifter.hpp"

#include "autoware_utils/autoware_utils.h"
#include "behavior_path_planner/utilities.hpp"
#include "spline_interpolation/spline_interpolation.h"

namespace
{
// for debug
std::string toStr(const geometry_msgs::Point & p)
{
  return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " + std::to_string(p.z) + ")";
}
std::string toStr(const behavior_path_planner::ShiftPoint & p)
{
  return "start point = " + toStr(p.start.position) + ", end point = " + toStr(p.end.position) +
         ", start idx = " + std::to_string(p.start_idx) +
         ", end idx = " + std::to_string(p.end_idx) + ", length = " + std::to_string(p.length);
}
}  // namespace

namespace behavior_path_planner
{
using autoware_planning_msgs::PathPointWithLaneId;
using autoware_planning_msgs::PathWithLaneId;

PathShifter::PathShifter() {}

void PathShifter::setPath(const PathWithLaneId & path)
{
  reference_path_ = path;
  is_index_aligned_ = false;  // shift_point index has to be updated for new path.
}
void PathShifter::addShiftPoint(const ShiftPoint & point)
{
  shift_points_.push_back(point);
  is_index_aligned_ = false;  // shift_point index has to be updated for new shift points.
}

bool PathShifter::generate(ShiftedPath * shifted_path)
{
  ROS_DEBUG_STREAM("PathShifter::generate start!");

  // Guard
  if (reference_path_.points.empty()) {
    ROS_ERROR_STREAM("reference path is empty.");
    return false;
  }

  shifted_path->path = reference_path_;
  shifted_path->shift_length.resize(reference_path_.points.size(), 0.0);

  if (shift_points_.empty()) {
    ROS_DEBUG_STREAM("shift_points_ is empty. Return reference with base offset.");
    shiftBaseLength(shifted_path, base_offset_);
    return true;
  }

  if (!is_index_aligned_) updateShiftPointIndices();

  // Sort shift points since applyShift function only supports sorted points
  if (!sortShiftPointsAlongPath(reference_path_)) {
    ROS_ERROR_STREAM("has duplicated points. Failed!");
    return false;
  }

  if (shift_points_.front().start_idx == 0) {
    ROS_WARN_STREAM(
      "shift start point is at the edge of path. It could cause undesired result."
      " Maybe path is too short for backward?");
  }

  // Calculate shifted path (linear shifter is only for debug, will be deprecated.)
  constexpr bool USE_SPLINE_SHIFTER = true;
  USE_SPLINE_SHIFTER ? applySplineShifter(shifted_path) : applyLinearShifter(shifted_path);

  // DEBUG
  ROS_DEBUG_STREAM("PathShifter::generate end. shift_points_.size = " << shift_points_.size());

  return true;
}

void PathShifter::applyLinearShifter(ShiftedPath * shifted_path)
{
  const auto arclength_arr = util::calcPathArcLengthArray(reference_path_);

  shiftBaseLength(shifted_path, base_offset_);

  constexpr double epsilon = 1.0e-8;  // to avoid 0 division

  // For all shift_points,
  double current_shift = base_offset_;
  for (const auto & shift_point : shift_points_) {
    const double delta_shift = shift_point.length - current_shift;
    const auto shifting_arclength = std::max(
      arclength_arr.at(shift_point.end_idx) - arclength_arr.at(shift_point.start_idx), epsilon);

    // For all path.points,
    for (size_t i = 0; i < shifted_path->path.points.size(); ++i) {
      // Set shift length.
      double ith_shift_length;
      if (i < shift_point.start_idx) {
        ith_shift_length = 0.0;
      } else if (shift_point.start_idx <= i && i <= shift_point.end_idx) {
        auto dist_from_start = arclength_arr.at(i) - arclength_arr.at(shift_point.start_idx);
        ith_shift_length = (dist_from_start / shifting_arclength) * delta_shift;
      } else {
        ith_shift_length = delta_shift;
      }

      // Apply shifting.
      addLateralOffsetOnIndexPoint(shifted_path, ith_shift_length, i);
    }
    current_shift = shift_point.length;
  }

  return;
}

void PathShifter::applySplineShifter(ShiftedPath * shifted_path)
{
  const auto arclength_arr = util::calcPathArcLengthArray(reference_path_);

  shiftBaseLength(shifted_path, base_offset_);

  constexpr double epsilon = 1.0e-8;  // to avoid 0 division

  spline_interpolation::SplineInterpolator spline;

  // For all shift_points,
  double current_shift = base_offset_;
  for (const auto & shift_point : shift_points_) {
    const double delta_shift = shift_point.length - current_shift;
    const auto shifting_arclength = std::max(
      arclength_arr.at(shift_point.end_idx) - arclength_arr.at(shift_point.start_idx), epsilon);

    // TODO(Watanabe) write docs.
    // These points are defined to achieve the constant-jerk shifting (see the header description).
    const std::vector<double> base_distance = {0.0, shifting_arclength / 4.0,
                                               shifting_arclength * 3.0 / 4.0, shifting_arclength};
    const std::vector<double> base_length = {0.0, delta_shift / 12.0, delta_shift * 11.0 / 12.0,
                                             delta_shift};

    std::vector<double> query_distance, query_length;

    // For all path.points,
    for (size_t i = shift_point.start_idx + 1; i < shift_point.end_idx; ++i) {
      const double dist_from_start = arclength_arr.at(i) - arclength_arr.at(shift_point.start_idx);
      query_distance.push_back(dist_from_start);
    }
    if (!spline.interpolate(
          base_distance, base_length, query_distance, query_length,
          spline_interpolation::Method::PCG)) {
      ROS_ERROR("spline failed!!");
    }

    // Apply shifting.
    {
      size_t i = shift_point.start_idx + 1;
      for (const auto & itr : query_length) {
        addLateralOffsetOnIndexPoint(shifted_path, itr, i);
        ++i;
      }
    }
    for (size_t i = shift_point.end_idx; i < shifted_path->path.points.size(); ++i) {
      addLateralOffsetOnIndexPoint(shifted_path, delta_shift, i);
    }
    current_shift = shift_point.length;
  }
  return;
}

std::vector<double> PathShifter::calcLateralJerk()
{
  const auto arclength_arr = util::calcPathArcLengthArray(reference_path_);

  constexpr double epsilon = 1.0e-8;  // to avoid 0 division

  std::vector<double> lateral_jerk;

  // TODO(Watanabe) write docs.
  double current_shift = base_offset_;
  for (const auto & shift_point : shift_points_) {
    const double delta_shift = shift_point.length - current_shift;
    const auto shifting_arclength = std::max(
      arclength_arr.at(shift_point.end_idx) - arclength_arr.at(shift_point.start_idx), epsilon);
    lateral_jerk.push_back((delta_shift / 2.0) / std::pow(shifting_arclength / 4.0, 3.0));
    current_shift = shift_point.length;
  }

  return lateral_jerk;
}

bool PathShifter::calcShiftPointFromArcLength(
  const PathWithLaneId & path, const geometry_msgs::Point & origin, double dist_to_start,
  double dist_to_end, double shift_length, ShiftPoint * shift_point)
{
  if (dist_to_end < dist_to_start) {
    ROS_ERROR_STREAM("dist_to_end must be longer than dist_to_start. Failed to create shift point");
    return false;
  }

  if (path.points.empty()) return false;

  const auto origin_idx = autoware_utils::findNearestIndex(path.points, origin);
  const auto arclength_from_origin = util::calcPathArcLengthArray(path, origin_idx);

  if (dist_to_end > arclength_from_origin.back()) {
    ROS_ERROR_STREAM(
      "path is too short to calculate shift point: path length from origin = "
      << arclength_from_origin.back() << ", desired dist_to_end = " << dist_to_end);
  }

  bool is_start_found = false;
  bool is_end_found = false;
  const auto getPathPointFromOrigin = [&](size_t idx_from_origin) {
    return path.points.at(idx_from_origin + origin_idx).point.pose;
  };
  for (size_t i = 0; i < arclength_from_origin.size() - 1; ++i) {
    if (!is_start_found && arclength_from_origin.at(i + 1) > dist_to_start) {
      shift_point->start = getPathPointFromOrigin(i);
      is_start_found = true;
    }
    if (!is_end_found && arclength_from_origin.at(i + 1) > dist_to_end) {
      shift_point->end = getPathPointFromOrigin(i);
      is_end_found = true;
      break;
    }
  }

  if (!is_start_found) {
    ROS_ERROR_STREAM(
      "Even start point has not found. You should stop the shifting. Failed to create shift point");
    return false;
  }

  if (is_start_found && !is_end_found) {
    ROS_ERROR_STREAM("failed to find end point. Set end point to path.back().");
    shift_point->end = path.points.back().point.pose;
  }

  shift_point->length = shift_length;
  ROS_DEBUG("Shift Point is generated from arclength: shift_length = %f", shift_length);
  return true;
}

void PathShifter::updateShiftPointIndices()
{
  for (auto & p : shift_points_) {
    p.start_idx = autoware_utils::findNearestIndex(reference_path_.points, p.start.position);
    p.end_idx = autoware_utils::findNearestIndex(reference_path_.points, p.end.position);
  }
  is_index_aligned_ = true;
}

bool PathShifter::checkShiftPointsAlignment(const std::vector<ShiftPoint> & shift_points) const
{
  for (const auto & p : shift_points) {
    ROS_DEBUG("shift point = %s", toStr(p).c_str());
  }

  for (const auto & c : shift_points) {
    if (c.start_idx > c.end_idx) {
      ROS_ERROR("shift_point must satisfy start_idx <= end_idx.");
      return false;
    }
  }

  return true;
}

bool PathShifter::sortShiftPointsAlongPath(const PathWithLaneId & path)
{
  if (shift_points_.empty()) {
    ROS_DEBUG_STREAM("shift_points_ is empty. do nothing.");
    return true;
  }

  const auto & unsorted_shift_points = shift_points_;

  // Calc indices sorted by "shift start point index" order
  std::vector<size_t> sorted_indices(unsorted_shift_points.size());
  std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
  std::sort(sorted_indices.begin(), sorted_indices.end(), [&](size_t i1, size_t i2) {
    return unsorted_shift_points[i1].start_idx < unsorted_shift_points[i2].start_idx;
  });

  // Set shift points and index by sorted_indices
  std::vector<ShiftPoint> sorted_shift_points;
  for (const auto sorted_i : sorted_indices) {
    sorted_shift_points.push_back(unsorted_shift_points.at(sorted_i));
  }

  // Check if the shift points are sorted correctly
  if (!checkShiftPointsAlignment(sorted_shift_points)) {
    ROS_ERROR_STREAM("Failed to sort shift points..!!");
    return false;
  }

  // set to member
  addShiftPoints(sorted_shift_points);

  // Debug
  for (const auto & p : unsorted_shift_points) {
    ROS_DEBUG_STREAM("unsorted_shift_points: " << toStr(p));
  }
  for (const auto & i : sorted_indices) {
    ROS_DEBUG_STREAM("sorted_indices i = " << i);
  }
  for (const auto & p : sorted_shift_points) {
    ROS_DEBUG_STREAM("sorted_shift_points: in order: " << toStr(p));
  }
  ROS_DEBUG("PathShifter::sortShiftPointsAlongPath end.");

  return true;
}

void PathShifter::removeBehindShiftPointAndSetBaseOffset(const geometry_msgs::Point & base_point)
{
  const auto base_idx = autoware_utils::findNearestIndex(reference_path_.points, base_point);

  // If shift_point.end is behind the base_point, remove the shift_point and
  // set its shift_length to the base_offset.
  std::vector<ShiftPoint> new_shift_points;
  std::vector<ShiftPoint> removed_shift_points;
  for (const auto & sp : shift_points_) {
    (sp.end_idx > base_idx) ? new_shift_points.push_back(sp) : removed_shift_points.push_back(sp);
  }

  double new_base_offset = base_offset_;
  if (!removed_shift_points.empty()) {
    const auto last_removed_sp = std::max_element(
      removed_shift_points.begin(), removed_shift_points.end(),
      [](auto & a, auto & b) { return a.end_idx > b.end_idx; });
    new_base_offset = last_removed_sp->length;
  }

  // remove accumulated floating noise
  if (std::abs(new_base_offset) < 1.0e-4) new_base_offset = 0.0;

  ROS_DEBUG("shift_points_ size: %lu -> %lu", shift_points_.size(), new_shift_points.size());

  addShiftPoints(new_shift_points);

  setBaseOffset(new_base_offset);
}

void PathShifter::addLateralOffsetOnIndexPoint(
  ShiftedPath * path, double offset, size_t index) const
{
  if (fabs(offset) < 1.0e-8) return;

  auto & p = path->path.points.at(index).point.pose;
  double yaw = tf2::getYaw(p.orientation);
  p.position.x -= std::sin(yaw) * offset;
  p.position.y += std::cos(yaw) * offset;

  path->shift_length.at(index) += offset;
}

void PathShifter::shiftBaseLength(ShiftedPath * path, double offset) const
{
  constexpr double BASE_OFFSET_THR = 1.0e-4;
  if (std::abs(offset) > BASE_OFFSET_THR) {
    for (size_t i = 0; i < path->path.points.size(); ++i) {
      addLateralOffsetOnIndexPoint(path, offset, i);
    }
  }
}

}  // namespace behavior_path_planner
