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

#include <limits>
#include <vector>

#include <boost/optional.hpp>

#include "autoware_planning_msgs/Trajectory.h"

#include "autoware_utils/geometry/geometry.h"
#include "autoware_utils/trajectory/trajectory.h"

#include "motion_velocity_smoother/resample.hpp"
#include "motion_velocity_smoother/trajectory_utils.hpp"

namespace motion_velocity_smoother
{
class SmootherBase
{
public:
  struct BaseParam
  {
    double max_accel;  // max acceleration in planning [m/s2] > 0
    double min_decel;  // min deceleration in planning [m/s2] < 0
    double max_jerk;
    double min_jerk;
    double max_lateral_accel;            // max lateral acceleration [m/ss] > 0
    double min_curve_velocity;           // min velocity at curve [m/s]
    double decel_distance_before_curve;  // distance before slow down for lateral acc at a curve
    double decel_distance_after_curve;   // distance after slow down for lateral acc at a curve
    resampling::ResampleParam resample_param;
  };

  virtual ~SmootherBase() = default;
  virtual bool apply(
    const double initial_vel, const double initial_acc,
    const autoware_planning_msgs::Trajectory & input, autoware_planning_msgs::Trajectory & output,
    std::vector<autoware_planning_msgs::Trajectory> & debug_trajectories) = 0;

  virtual boost::optional<autoware_planning_msgs::Trajectory> applyLateralAccelerationFilter(
    const autoware_planning_msgs::Trajectory & input) const;

  virtual boost::optional<autoware_planning_msgs::Trajectory> resampleTrajectory(
    const autoware_planning_msgs::Trajectory & input, const double v_current,
    const int closest_id) const;

  double getMaxAccel() const;
  double getMinDecel() const;
  double getMaxJerk() const;
  double getMinJerk() const;

  void setParam(const BaseParam & param);

protected:
  BaseParam base_param_;
};
}  // namespace motion_velocity_smoother
