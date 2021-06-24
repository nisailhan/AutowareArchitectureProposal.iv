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

#ifndef TURN_SIGNAL_DECIDER_HPP
#define TURN_SIGNAL_DECIDER_HPP

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "tf2_ros/transform_listener.h"

#include "autoware_planning_msgs/PathWithLaneId.h"
#include "autoware_vehicle_msgs/TurnSignal.h"
#include "lanelet2_core/LaneletMap.h"

#include <memory>

#include "behavior_path_planner/route_handler.hpp"

namespace behavior_path_planner
{
class TurnSignalDecider
{
public:
  TurnSignalDecider(){};

  autoware_vehicle_msgs::TurnSignal getTurnSignal(
    const autoware_planning_msgs::PathWithLaneId & path, const geometry_msgs::Pose & current_pose,
    const behavior_path_planner::RouteHandler & route_handler,
    const autoware_vehicle_msgs::TurnSignal & turn_signal_plan, const double plan_distance) const;

  void setParameters(const double base_link2front, const double intersection_search_distance)
  {
    base_link2front_ = base_link2front;
    intersection_search_distance_ = intersection_search_distance;
  };

private:
  std::pair<autoware_vehicle_msgs::TurnSignal, double> getIntersectionTurnSignal(
    const autoware_planning_msgs::PathWithLaneId & path, const geometry_msgs::Pose & current_pose,
    const behavior_path_planner::RouteHandler & route_handler) const;

  // data
  double intersection_search_distance_ = 0.0;
  double base_link2front_ = 0.0;
};
}  // namespace behavior_path_planner

#endif
