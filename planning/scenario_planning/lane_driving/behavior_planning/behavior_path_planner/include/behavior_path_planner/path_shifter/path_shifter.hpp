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

#pragma once

#include <string>
#include <vector>

#include "autoware_utils/ros/marker_helper.h"
#include "ros/ros.h"

#include "autoware_perception_msgs/DynamicObjectArray.h"
#include "autoware_planning_msgs/PathWithLaneId.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Polygon.h"
#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "visualization_msgs/MarkerArray.h"

namespace behavior_path_planner
{

using autoware_planning_msgs::PathPointWithLaneId;
using autoware_planning_msgs::PathWithLaneId;

struct ShiftPoint
{
  geometry_msgs::Pose start;  // shift start point in absolute coordinate
  geometry_msgs::Pose end;    // shift start point in absolute coordinate
  double length;               // absolute shift length related to the reference path
  size_t start_idx;            // associated start-point index for the reference path
  size_t end_idx;              // associated end-point index for the reference path
};

struct ShiftedPath
{
  PathWithLaneId path;
  std::vector<double> shift_length;
};

class PathShifter
{
public:
  PathShifter();

  // setter & getter

  /**
   * @brief  Set reference path.
   */
  void setPath(const PathWithLaneId & path);

  /**
   * @brief  Add shift point. You don't have to care about the start/end_idx.
   */
  void addShiftPoint(const ShiftPoint & point);

  std::vector<ShiftPoint> getShiftPoints() const { return shift_points_; }
  PathWithLaneId getReferencePath() const { return reference_path_; }
  size_t getShiftPointsSize() const { return shift_points_.size(); }

  double getBaseOffset() const { return base_offset_; }

  /**
   * @brief  Generate a shifted path according to the given reference path and shift points.
   * @return False if the path is empty or shift points have conflicts.
   */
  bool generate(ShiftedPath * shift_path);

  /**
   * @brief Remove behind shift points and add the removed offset to the base_offset_.
   * @details The previous offset information is stored in the base_offset_.
   *          This should be called after generate().
   */
  void removeBehindShiftPointAndSetBaseOffset(const geometry_msgs::Point & base);

  ////////////////////////////////////////
  // Utility Functions
  ////////////////////////////////////////

  static double calcLongitudinalDistFromJerk(
    const double lateral, const double jerk, const double velocity)
  {
    const double j = std::abs(jerk);
    const double l = std::abs(lateral);
    const double v = std::abs(velocity);
    if (j < 1.0e-8) return 1.0e10;  // TODO(Horibe) maybe invalid arg?

    return 4.0 * std::pow(0.5 * l / j, 1.0 / 3.0) * std::pow(v, 3);
  }

  double getTotalShiftLength() const
  {
    double sum = base_offset_;
    for (const auto & p : shift_points_) {
      sum += p.length;
    }
    return sum;
  }

  double getLastShiftLength() const
  {
    if (shift_points_.empty()) return base_offset_;

    // TODO(Horibe) enable this with const
    // if (!is_index_aligned_) {
    //   updateShiftPointIndices();
    // }
    const auto furthest = std::max_element(
      shift_points_.begin(), shift_points_.end(),
      [](auto & a, auto & b) { return a.end_idx > b.end_idx; });

    return furthest->length;
  }

  /**
   * @brief  Calculate the theoretical lateral jerk by spline shifting for current shift_points_.
   * @return Jerk array. THe size is same as the shift points.
   */
  std::vector<double> calcLateralJerk();

  /**
   * @brief  Calculate shift point from path arclength for start and end point.
   */
  static bool calcShiftPointFromArcLength(
    const PathWithLaneId & path, const geometry_msgs::Point & origin, double dist_to_start,
    double dist_to_end, double shift_length, ShiftPoint * shift_point);

private:
  // The reference path along which the shift will be performed.
  PathWithLaneId reference_path_;

  // Shift points used for shifted-path generation.
  std::vector<ShiftPoint> shift_points_;

  // The amount of shift length to the entire path.
  double base_offset_ = 0.0;

  // Flag to check the path index is aligned. (cleared when new path or shift points are received)
  bool is_index_aligned_ = false;

  /**
   * @brief Calculate path index for shift_points and set is_index_aligned_ to true.
   */
  void updateShiftPointIndices();

  /**
   * @brief Sort the points in order from the front of the path.
   */
  bool sortShiftPointsAlongPath(const PathWithLaneId & path);

  /**
   * @brief Generate shifted path from reference_path_ and shift_points_ with linear shifting.
   */
  void applyLinearShifter(ShiftedPath * shifted_path);

  /**
   * @brief Generate shifted path from reference_path_ and shift_points_ with spline_based shifting.
   * @details Calculate the shift so that the horizontal jerk remains constant. This is achieved by
   *          dividing the shift interval into four parts and apply a cubic spline to them.
   *          The resultant shifting shape is closed to the Clothoid curve.
   */
  void applySplineShifter(ShiftedPath * shifted_path);

  ////////////////////////////////////////
  // Helper Functions
  ////////////////////////////////////////

  /**
   * @brief Check if the shift points are aligned in order and have no conflict range.
   */
  bool checkShiftPointsAlignment(const std::vector<ShiftPoint> & shift_points) const;

  void addLateralOffsetOnIndexPoint(ShiftedPath * point, double offset, size_t index) const;

  void shiftBaseLength(ShiftedPath * point, double offset) const;

  void setBaseOffset(const double val)
  {
    ROS_DEBUG("base_offset is changed: %f -> %f", base_offset_, val);
    base_offset_ = val;
  }

  void addShiftPoints(const std::vector<ShiftPoint> & val)
  {
    ROS_DEBUG("shift points are changed: size %lu -> %lu", shift_points_.size(), val.size());
    shift_points_ = val;
  }
};

}  // namespace behavior_path_planner
