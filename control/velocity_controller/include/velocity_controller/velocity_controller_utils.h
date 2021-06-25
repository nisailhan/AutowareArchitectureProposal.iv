/*
 * Copyright 2018 Tier IV, Inc. All rights reserved.
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

#ifndef VELOCITY_CONTROLLER_UTILS
#define VELOCITY_CONTROLLER_UTILS

#include <cmath>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <tf2/utils.h>
#include <boost/optional.hpp>
#include "autoware_planning_msgs/Trajectory.h"
#include "autoware_utils/geometry/geometry.h"
#include "autoware_utils/trajectory/trajectory.h"

namespace velocity_controller_utils
{
bool isValidTrajectory(const autoware_planning_msgs::Trajectory & traj);

double calcStopDistance(
  const geometry_msgs::Point & current_pos, const autoware_planning_msgs::Trajectory & traj);

double getPitchByPose(const geometry_msgs::Quaternion & quaternion);

double getPitchByTraj(
  const autoware_planning_msgs::Trajectory & msg, const size_t closest_idx,
  const double wheel_base);

double calcElevationAngle(const geometry_msgs::Point & p_from, const geometry_msgs::Point & p_to);

geometry_msgs::Pose calcPoseAfterTimeDelay(
  const geometry_msgs::Pose & current_pose, const double delay_time, const double current_vel);

double lerp(const double src_val, const double dst_val, const double ratio);

template <class T>
T lerpXYZ(const T & p_from, const T & p_to, const double ratio)
{
  T point;
  point.x = lerp(p_from.x, p_to.x, ratio);
  point.y = lerp(p_from.x, p_to.y, ratio);
  point.z = lerp(p_from.x, p_to.z, ratio);
  return point;
}

geometry_msgs::Quaternion lerpOrientation(
  const geometry_msgs::Quaternion & o_from, const geometry_msgs::Quaternion & o_to,
  const double ratio);

template <class T>
autoware_planning_msgs::TrajectoryPoint lerpTrajectoryPoint(
  const T & points, const geometry_msgs::Point & point)
{
  autoware_planning_msgs::TrajectoryPoint interpolated_point;

  const size_t closest_seg_idx = autoware_utils::findNearestSegmentIndex(points, point);

  const double len_to_interpolated =
    autoware_utils::calcLongitudinalOffsetToSegment(points, closest_seg_idx, point);
  const double len_segment =
    autoware_utils::calcSignedArcLength(points, closest_seg_idx, closest_seg_idx + 1);
  const double interpolate_ratio = len_to_interpolated / len_segment;

  {
    const size_t i = closest_seg_idx;

    interpolated_point.pose.position =
      lerpXYZ(points.at(i).pose.position, points.at(i + 1).pose.position, interpolate_ratio);
    interpolated_point.pose.orientation = lerpOrientation(
      points.at(i).pose.orientation, points.at(i + 1).pose.orientation, interpolate_ratio);
    interpolated_point.twist.linear =
      lerpXYZ(points.at(i).twist.linear, points.at(i + 1).twist.linear, interpolate_ratio);
    interpolated_point.twist.angular =
      lerpXYZ(points.at(i).twist.angular, points.at(i + 1).twist.angular, interpolate_ratio);
    interpolated_point.accel.linear =
      lerpXYZ(points.at(i).accel.linear, points.at(i + 1).accel.linear, interpolate_ratio);
    interpolated_point.accel.angular =
      lerpXYZ(points.at(i).accel.angular, points.at(i + 1).accel.angular, interpolate_ratio);
  }

  return interpolated_point;
}
}  // namespace velocity_controller_utils

#endif
