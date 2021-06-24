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

#include <cmath>

#include "motion_velocity_smoother/smoother/smoother_base.hpp"
#include "motion_velocity_smoother/trajectory_utils.hpp"

namespace motion_velocity_smoother
{
void SmootherBase::setParam(const BaseParam & param) { base_param_ = param; }

double SmootherBase::getMaxAccel() const { return base_param_.max_accel; }

double SmootherBase::getMinDecel() const { return base_param_.min_decel; }

double SmootherBase::getMaxJerk() const { return base_param_.max_jerk; }

double SmootherBase::getMinJerk() const { return base_param_.min_jerk; }

boost::optional<autoware_planning_msgs::Trajectory> SmootherBase::applyLateralAccelerationFilter(
  const autoware_planning_msgs::Trajectory & input) const
{
  if (input.points.empty()) {
    return boost::none;
  }

  if (input.points.size() < 3) {
    return boost::optional<autoware_planning_msgs::Trajectory>(
      input);  // cannot calculate lateral acc. do nothing.
  }

  // Interpolate with constant interval distance for lateral acceleration calculation.
  constexpr double points_interval = 0.1;  // [m]
  std::vector<double> out_arclength;
  const std::vector<double> in_arclength = trajectory_utils::calcArclengthArray(input);
  for (double s = 0; s < in_arclength.back(); s += points_interval) {
    out_arclength.push_back(s);
  }
  auto output = trajectory_utils::applyLinearInterpolation(in_arclength, input, out_arclength);
  if (!output) {
    ROS_WARN("[MotionVelocitySmoother]: interpolation failed at lateral acceleration filter.");
    return boost::none;
  }
  output->points.back().twist = input.points.back().twist;  // keep the final speed.

  constexpr double curvature_calc_dist = 5.0;  // [m] calc curvature with 5m away points
  const size_t idx_dist =
    static_cast<size_t>(std::max(static_cast<int>((curvature_calc_dist) / points_interval), 1));

  // Calculate curvature assuming the trajectory points interval is constant
  const auto curvature_v = trajectory_utils::calcTrajectoryCurvatureFrom3Points(*output, idx_dist);
  if (!curvature_v) return boost::optional<autoware_planning_msgs::Trajectory>(input);

  //  Decrease speed according to lateral G
  const size_t before_decel_index =
    static_cast<size_t>(std::round(base_param_.decel_distance_before_curve / points_interval));
  const size_t after_decel_index =
    static_cast<size_t>(std::round(base_param_.decel_distance_after_curve / points_interval));
  const double max_lateral_accel_abs = std::fabs(base_param_.max_lateral_accel);

  for (size_t i = 0; i < output->points.size(); ++i) {
    double curvature = 0.0;
    const size_t start = i > before_decel_index ? i - before_decel_index : 0;
    const size_t end = std::min(output->points.size(), i + after_decel_index);
    for (size_t j = start; j < end; ++j) {
      curvature = std::max(curvature, std::fabs(curvature_v->at(j)));
    }
    double v_curvature_max = std::sqrt(max_lateral_accel_abs / std::max(curvature, 1.0E-5));
    v_curvature_max = std::max(v_curvature_max, base_param_.min_curve_velocity);
    if (output->points.at(i).twist.linear.x > v_curvature_max) {
      output->points.at(i).twist.linear.x = v_curvature_max;
    }
  }
  return output;
}

boost::optional<autoware_planning_msgs::Trajectory> SmootherBase::resampleTrajectory(
  const autoware_planning_msgs::Trajectory & input, const double v_current,
  const int closest_id) const
{
  // Arc length from the initial point to the closest point
  const double front_arclength_value = trajectory_utils::calcArcLength(input, 0, closest_id);

  // Get the nearest point where velocity is zero
  auto zero_vel_id =
    autoware_utils::searchZeroVelocityIndex(input.points, closest_id, input.points.size());
  // Arc length from the closest point to the point where velocity is zero
  double zero_vel_arclength_value = base_param_.max_trajectory_length;
  if (zero_vel_id) {
    zero_vel_arclength_value = std::min(
      zero_vel_arclength_value,
      autoware_utils::calcSignedArcLength(input.points, closest_id, *zero_vel_id));
  }

  //Get the resample size from the closest point
  const std::vector<double> in_arclength = trajectory_utils::calcArclengthArray(input);
  const double Nt = base_param_.resample_time / std::max(base_param_.resample_dt, 0.001);
  const double ds_nominal =
    std::max(v_current * base_param_.resample_dt, base_param_.min_trajectory_interval_distance);
  const double Ns = base_param_.min_trajectory_length / std::max(ds_nominal, 0.001);
  const double N = std::max(Nt, Ns);
  std::vector<double> out_arclength;

  // Step1. Resample front trajectory
  constexpr double front_ds = 0.1;
  for (double ds = 0.0; ds <= front_arclength_value; ds += front_ds) {
    out_arclength.push_back(ds);
  }
  if (std::fabs(out_arclength.back() - front_arclength_value) < 1e-3) {
    out_arclength.back() = front_arclength_value;
  } else {
    out_arclength.push_back(front_arclength_value);
  }

  // Step2. Resample behind trajectory
  double dist_i = 0.0;
  bool is_zero_point_included = false;
  bool is_endpoint_included = false;
  for (size_t i = 1; static_cast<double>(i) <= N; ++i) {
    double ds = ds_nominal;
    if (i > Nt) {
      // if the planning time is not enough to see the desired distance, change the interval distance to see far.
      ds = std::max(4.0, 0.5 * v_current);
    }

    dist_i += ds;

    // Check if the distance is longer than max_trajectory_length
    if (dist_i > base_param_.max_trajectory_length) {
      break;  // distance is over max.
    }

    // Check if the distance is longer than input arclength
    if (dist_i + front_arclength_value >= in_arclength.back()) {
      is_endpoint_included = true;  // distance is over input endpoint.
      break;
    }

    // Check if the distance is longer than minimum_trajectory_length
    if (i > Nt && dist_i >= base_param_.min_trajectory_length) {
      if (
        std::fabs(
          out_arclength.back() - (base_param_.min_trajectory_length + front_arclength_value)) <
        1e-3) {
        out_arclength.back() = base_param_.min_trajectory_length + front_arclength_value;
      } else {
        out_arclength.push_back(base_param_.min_trajectory_length + front_arclength_value);
      }
      break;
    }

    // Handle Close Stop Point
    if (dist_i > zero_vel_arclength_value && !is_zero_point_included) {
      if (std::fabs(dist_i - zero_vel_arclength_value) > 1e-3) {
        // dist_i is much bigger than zero_vel_arclength_value
        if (
          !out_arclength.empty() &&
          std::fabs(out_arclength.back() - (zero_vel_arclength_value + front_arclength_value)) <
            1e-3) {
          out_arclength.back() = zero_vel_arclength_value + front_arclength_value;
        } else {
          out_arclength.push_back(zero_vel_arclength_value + front_arclength_value);
        }
      } else {
        // dist_i is close to the zero_vel_arclength_value
        dist_i = zero_vel_arclength_value;
      }

      is_zero_point_included = true;
    }

    out_arclength.push_back(dist_i + front_arclength_value);
  }

  auto output = trajectory_utils::applyLinearInterpolation(in_arclength, input, out_arclength);
  if (!output) {
    ROS_WARN(
      "[MotionVelocitySmoother]: fail trajectory interpolation. size : in_arclength = %lu, "
      "input = %lu, out_arclength = %lu",
      in_arclength.size(), input.points.size(), out_arclength.size());
    return boost::none;
  }

  // add end point directly to consider the endpoint velocity.
  if (is_endpoint_included) {
    constexpr double ep_dist = 1.0E-3;
    if (autoware_utils::calcDistance2d(output->points.back(), input.points.back()) < ep_dist) {
      output->points.back() = input.points.back();
    } else {
      output->points.push_back(input.points.back());
    }
  }
  output->points.back().twist.linear.x = 0.0;

  return output;
}
}  // namespace motion_velocity_smoother
