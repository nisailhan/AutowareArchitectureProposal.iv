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
 */

#pragma once
#include <iostream>
#include <map>
#include <numeric>

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf2/utils.h>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

#include "autoware_planning_msgs/Trajectory.h"
#include "autoware_utils/geometry/geometry.h"
#include "autoware_utils/trajectory/trajectory.h"

namespace motion_velocity_smoother
{
namespace trajectory_utils
{
autoware_planning_msgs::TrajectoryPoint calcInterpolatedTrajectoryPoint(
  const autoware_planning_msgs::Trajectory & trajectory, const geometry_msgs::Pose & target_pose);

boost::optional<autoware_planning_msgs::Trajectory> extractPathAroundIndex(
  const autoware_planning_msgs::Trajectory & trajectory, const size_t index,
  const double & ahead_length, const double & behind_length);

double calcArcLength(
  const autoware_planning_msgs::Trajectory & trajectory, const int idx1, const int idx2);

std::vector<double> calcArclengthArray(const autoware_planning_msgs::Trajectory & trajectory);

std::vector<double> calcTrajectoryIntervalDistance(
  const autoware_planning_msgs::Trajectory & trajectory);

boost::optional<std::vector<double>> calcTrajectoryCurvatureFrom3Points(
  const autoware_planning_msgs::Trajectory & trajectory, const size_t & idx_dist);

void setZeroVelocity(autoware_planning_msgs::Trajectory & trajectory);

double getMaxVelocity(const autoware_planning_msgs::Trajectory & trajectory);

double getMaxAbsVelocity(const autoware_planning_msgs::Trajectory & trajectory);

void applyMaximumVelocityLimit(
  const size_t from, const size_t to, const double max_vel,
  autoware_planning_msgs::Trajectory & trajectory);

boost::optional<size_t> searchZeroVelocityIdx(
  const autoware_planning_msgs::Trajectory & trajectory);

boost::optional<autoware_planning_msgs::Trajectory> applyLinearInterpolation(
  const std::vector<double> & base_index,
  const autoware_planning_msgs::Trajectory & base_trajectory,
  const std::vector<double> & out_index);

bool calcStopDistWithJerkConstraints(
  const double v0, const double a0, const double jerk_acc, const double jerk_dec,
  const double min_acc, const double target_vel, std::map<double, double> & jerk_profile,
  double & stop_dist);

bool isValidStopDist(
  const double v_end, const double a_end, const double v_target, const double a_target,
  const double v_margin, const double a_margin);

boost::optional<autoware_planning_msgs::Trajectory> applyDecelFilterWithJerkConstraint(
  const autoware_planning_msgs::Trajectory & input, const size_t start_index, const double v0,
  const double a0, const double min_acc, const double decel_target_vel,
  const std::map<double, double> & jerk_profile);

boost::optional<std::tuple<double, double, double, double>> updateStateWithJerkConstraint(
  const double v0, const double a0, const std::map<double, double> & jerk_profile, const double t);

}  // namespace trajectory_utils
}  // namespace motion_velocity_smoother
