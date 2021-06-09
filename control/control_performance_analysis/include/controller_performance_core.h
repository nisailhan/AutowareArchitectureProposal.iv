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

#ifndef CONTROL_PERFORMANCE_MEASUREMENT_CONTROLLER_PERFORMANCE_CORE_H
#define CONTROL_PERFORMANCE_MEASUREMENT_CONTROLLER_PERFORMANCE_CORE_H

#include <geometry_msgs/PoseArray.h>
#include "autoware_control_msgs/ControlCommandStamped.h"
#include "autoware_planning_msgs/Trajectory.h"
#include "control_performance_utils.h"

struct TargetPerformanceMsgVars
{
  double lateral_error;
  double heading_error;
  double control_effort_energy;
  double error_energy;
  double value_approximation;
  double curvature_estimate;
  double curvature_estimate_pp;
  double lateral_error_velocity;
  double lateral_error_acceleration;
};

class ControlPerformanceCore
{
public:
  ControlPerformanceCore();

  explicit ControlPerformanceCore(double wheelbase, double curvature_interval_length);

  ~ControlPerformanceCore() = default;

  // Setters
  void setCurrentPose(const geometry_msgs::Pose & msg);
  void setCurrentWaypoints(const autoware_planning_msgs::Trajectory & trajectory);
  void setCurrentVelocities(const geometry_msgs::Twist & twist_msg);
  void setCurrentControValue(const autoware_control_msgs::ControlCommandStamped & msg);
  void setInterpolatedPose(geometry_msgs::Pose & interpolated_pose);

  void findCurveRefIdx();
  std::pair<bool, int32_t> findClosestPrevWayPointIdx_path_direction();
  double estimateCurvature();
  double estimatePurePursuitCurvature();

  // Getters
  bool isDataReady() const;
  std::pair<bool, TargetPerformanceMsgVars> getPerformanceVars();
  geometry_msgs::Pose getPrevWPPose() const;
  std::pair<bool, geometry_msgs::Pose> calculateClosestPose();

private:
  double wheelbase_;
  double curvature_interval_length_;

  // Variables Received Outside
  std::shared_ptr<geometry_msgs::PoseArray> current_waypoints_ptr_;
  std::shared_ptr<geometry_msgs::Pose> current_vec_pose_ptr_;
  std::shared_ptr<std::vector<double>> current_velocities_ptr_;  // [Vx, Heading rate]
  std::shared_ptr<autoware_control_msgs::ControlCommandStamped> current_control_ptr_;

  // Variables computed
  std::unique_ptr<int32_t> idx_prev_wp_;       // the waypoint index, vehicle
  std::unique_ptr<int32_t> idx_curve_ref_wp_;  // index of waypoint corresponds to front axle center
  std::unique_ptr<int32_t> idx_next_wp_;       //  the next waypoint index, vehicle heading to
  std::unique_ptr<TargetPerformanceMsgVars> prev_target_vars_{};
  std::shared_ptr<geometry_msgs::Pose> interpolated_pose_ptr_;
  // V = xPx' ; Value function from DARE Lyap matrix P
  Eigen::Matrix2d const lyap_P_ = (Eigen::MatrixXd(2, 2) << 2.342, 8.60, 8.60, 64.29).finished();
  double const contR{10.0};  // Control weight in LQR
};

#endif  //CONTROL_PERFORMANCE_MEASUREMENT_CONTROLLER_PERFORMANCE_CORE_H
