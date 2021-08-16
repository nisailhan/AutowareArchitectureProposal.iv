/*
 * Copyright 2021 Tier IV, Inc. All rights reserved.
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
 *
 */

#include "behavior_path_planner/path_utilities.hpp"
#include "autoware_utils/autoware_utils.h"
#include "behavior_path_planner/utilities.hpp"
#include "lanelet2_extension/utility/message_conversion.h"
#include "lanelet2_extension/utility/query.h"
#include "lanelet2_extension/utility/utilities.h"
#include "opencv2/opencv.hpp"
#include "spline_interpolation/spline_interpolation.h"
#include "tf2/utils.h"

namespace behavior_path_planner
{
namespace util
{
using autoware_perception_msgs::PredictedPath;
using autoware_planning_msgs::PathPointWithLaneId;
using autoware_planning_msgs::PathWithLaneId;

/**
 * @brief calc path arclength on each points from start point to end point.
 */
std::vector<double> calcPathArcLengthArray(const PathWithLaneId & path, size_t start, size_t end)
{
  std::vector<double> out;

  double sum = 0.0;
  out.push_back(sum);

  start = std::max(start + 1, static_cast<size_t>(1));
  end = std::min(end, path.points.size());

  for (size_t i = start; i < end; ++i) {
    sum += autoware_utils::calcDistance2d(path.points.at(i).point, path.points.at(i - 1).point);
    out.push_back(sum);
  }
  return out;
}

/**
 * @brief calc path arclength from start point to end point.
 */
double calcPathArcLength(const PathWithLaneId & path, size_t start, size_t end)
{
  if (path.points.size() < 2) return 0.0;

  // swap
  bool is_negative_direction = false;
  if (start > end) {
    std::swap(start, end);
    is_negative_direction = true;
  }

  start = std::max(start, static_cast<size_t>(1));
  end = std::min(end, path.points.size());

  double sum = 0.0;
  for (size_t i = start; i < end; ++i) {
    sum += autoware_utils::calcDistance2d(path.points.at(i).point, path.points.at(i - 1).point);
  }

  return is_negative_direction ? -sum : sum;
}

/**
 * @brief resamplePathWithSpline
 */
PathWithLaneId resamplePathWithSpline(const PathWithLaneId & path, double interval)
{
  const auto base_points = calcPathArcLengthArray(path);
  const auto sampling_points = rangeVector(0.0, interval, base_points.back());

  if (base_points.empty() || sampling_points.empty()) return path;

  std::vector<double> base_x, base_y, base_z;
  for (const auto & p : path.points) {
    base_x.push_back(p.point.pose.position.x);
    base_y.push_back(p.point.pose.position.y);
    base_z.push_back(p.point.pose.position.z);
  }

  std::vector<double> resampled_x, resampled_y, resampled_z;
  {
    spline_interpolation::SplineInterpolator spline;
    if (
      !spline.interpolate(base_points, base_x, sampling_points, resampled_x) ||
      !spline.interpolate(base_points, base_y, sampling_points, resampled_y) ||
      !spline.interpolate(base_points, base_z, sampling_points, resampled_z)) {
      throw std::runtime_error("spline failed in behavior_path_planner avoidance module");
    }
  }

  PathWithLaneId resampled_path;
  resampled_path.header = path.header;
  resampled_path.drivable_area = path.drivable_area;

  // For Point X, Y, Z
  for (size_t i = 0; i < sampling_points.size(); ++i) {
    PathPointWithLaneId p;
    p.point.pose.position =
      autoware_utils::createPoint(resampled_x.at(i), resampled_y.at(i), resampled_z.at(i));
    resampled_path.points.push_back(p);
  }

  // For LaneIds, Type, Twist
  //
  // ------|----|----|----|----|----|----|-------> resampled
  //      [0]  [1]  [2]  [3]  [4]  [5]  [6]
  //
  // --------|---------------|----------|---------> base
  //        [0]             [1]        [2]
  //
  // resampled[0~3] = base[0]
  // resampled[4~5] = base[1]
  // resampled[6] = base[2]
  //
  size_t base_idx = 0;
  for (size_t i = 0; i < resampled_path.points.size(); ++i) {
    while (base_idx < base_points.size() - 1 && sampling_points.at(i) > base_points.at(base_idx)) {
      ++base_idx;
    }
    size_t ref_idx = std::max(static_cast<int>(base_idx) - 1, 0);
    if (i == resampled_path.points.size() - 1) ref_idx = base_points.size() - 1;  // for last point
    auto & p = resampled_path.points.at(i);
    p.lane_ids = path.points.at(ref_idx).lane_ids;
    p.point.type = path.points.at(ref_idx).point.type;
    p.point.twist = path.points.at(ref_idx).point.twist;
  }

  // For Yaw
  for (size_t i = 0; i < resampled_path.points.size() - 1; ++i) {
    const auto & p0 = resampled_path.points.at(i).point.pose.position;
    const auto & p1 = resampled_path.points.at(i + 1).point.pose.position;
    const double yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
    resampled_path.points.at(i).point.pose.orientation =
      tf2::toMsg(autoware_utils::createQuaternionFromRPY(0.0, 0.0, yaw));
  }
  if (resampled_path.points.size() > 2) {
    resampled_path.points.back().point.pose.orientation =
      resampled_path.points.at(resampled_path.points.size() - 2).point.pose.orientation;
  }

  return resampled_path;
}

autoware_planning_msgs::Path toPath(const autoware_planning_msgs::PathWithLaneId & input)
{
  autoware_planning_msgs::Path output;
  output.header = input.header;
  output.drivable_area = input.drivable_area;
  output.points.resize(input.points.size());
  for (size_t i = 0; i < input.points.size(); ++i) {
    output.points.at(i) = input.points.at(i).point;
  }
  return output;
}

size_t getIdxByArclength(
  const autoware_planning_msgs::PathWithLaneId & path, const geometry_msgs::Point & origin,
  const double signed_arc)
{
  const auto closest_idx = autoware_utils::findNearestIndex(path.points, origin);

  size_t index = closest_idx;
  double sum_length = 0.0;
  if (signed_arc >= 0.0) {
    for (; index < path.points.size() - 1; ++index) {
      sum_length +=
        autoware_utils::calcDistance2d(path.points.at(index), path.points.at(index + 1));
      if (sum_length > signed_arc) break;
    }
  } else {
    for (; index > 0; --index) {
      sum_length -=
        autoware_utils::calcDistance2d(path.points.at(index), path.points.at(index - 1));
      if (sum_length < signed_arc) break;
    }
  }
  return index;
}

}  // namespace util
}  // namespace behavior_path_planner
