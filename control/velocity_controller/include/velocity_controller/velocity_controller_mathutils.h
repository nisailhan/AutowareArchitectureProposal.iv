/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#ifndef VELOCITY_CONTROLLER_MATHUTILS
#define VELOCITY_CONTROLLER_MATHUTILS

#include <cmath>

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <boost/optional.hpp>
#include "autoware_planning_msgs/Trajectory.h"
#include "autoware_utils/geometry/geometry.h"
#include "autoware_utils/math/normalization.h"

namespace vcutils
{
boost::optional<int> searchZeroVelocityIdx(const autoware_planning_msgs::Trajectory & trajectory);
boost::optional<double> calcLengthOnWaypoints(
  const autoware_planning_msgs::Trajectory & path, const unsigned int source_idx,
  const unsigned int target_idx);
int calcClosestWaypoint(
  const autoware_planning_msgs::Trajectory & traj, const geometry_msgs::Pose & pose);
int calcClosestWaypoint(
  const autoware_planning_msgs::Trajectory & trajectory, const geometry_msgs::Pose & pose,
  const double delta_yaw_threshold);
template <class Point>
bool judgePoseOverPoint(
  const Point & pose, const autoware_planning_msgs::Trajectory & traj, const int target_idx);
template <class Point>
std::pair<double, double> calcTwoPointsInterpolatedLength(
  const Point & p_target, const Point & p_from, const Point & p_to);
void searchClosestTwoPoints(
  const geometry_msgs::Pose & pose, const autoware_planning_msgs::Trajectory & traj,
  signed int & min_idx, signed int & max_idx, double & min_idx_length, double & max_idx_length);
void searchClosestTwoPoints(
  const geometry_msgs::Pose & pose, const autoware_planning_msgs::Trajectory & traj,
  signed int & min_idx, signed int & max_idx);
boost::optional<double> calcTrajectoryLengthFromPose(
  const geometry_msgs::Pose & pose, const autoware_planning_msgs::Trajectory & traj,
  unsigned int target_idx);
boost::optional<double> calcStopDistance(
  const geometry_msgs::Pose & current_pose, const autoware_planning_msgs::Trajectory & traj);

double calcPitch(const geometry_msgs::Pose & p1, const geometry_msgs::Pose & p2);
boost::optional<int> calcClosestWithThr(
  const autoware_planning_msgs::Trajectory & trajectory, const geometry_msgs::Pose & pose,
  const double angle_thr, const double dist_thr);
geometry_msgs::Point transformToRelativeCoordinate2D(
  const geometry_msgs::Point & point, const geometry_msgs::Pose & origin);
}  // namespace vcutils

#endif
