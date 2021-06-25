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

#include <vector>

#include <ros/ros.h>

#include "autoware_planning_msgs/Trajectory.h"
#include "autoware_utils/geometry/geometry.h"
#include "autoware_utils/trajectory/trajectory.h"
#include "osqp_interface/osqp_interface.h"

#include "motion_velocity_smoother/smoother/smoother_base.hpp"

namespace motion_velocity_smoother
{
class JerkFilteredSmoother : public SmootherBase
{
public:
  struct Param
  {
    double jerk_weight;
    double over_v_weight;
    double over_a_weight;
    double over_j_weight;
  };

  explicit JerkFilteredSmoother(const Param & p);

  bool apply(
    const double initial_vel, const double initial_acc,
    const autoware_planning_msgs::Trajectory & input, autoware_planning_msgs::Trajectory & output,
    std::vector<autoware_planning_msgs::Trajectory> & debug_trajectories) override;

  void setParam(const Param & param);

private:
  Param smoother_param_;
  osqp::OSQPInterface qp_solver_;

  autoware_planning_msgs::Trajectory forwardJerkFilter(
    const double v0, const double a0, const double a_max, const double j_max,
    const autoware_planning_msgs::Trajectory & input) const;
  autoware_planning_msgs::Trajectory backwardJerkFilter(
    const double v0, const double a0, const double a_min, const double j_min,
    const autoware_planning_msgs::Trajectory & input) const;
  autoware_planning_msgs::Trajectory mergeFilteredTrajectory(
    const double v0, const double a0, const double a_min, const double j_min,
    const autoware_planning_msgs::Trajectory & forward_filtered,
    const autoware_planning_msgs::Trajectory & backward_filtered) const;
};
}  // namespace motion_velocity_smoother
