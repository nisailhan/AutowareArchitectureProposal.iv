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

#include "autoware_utils/autoware_utils.h"

#include "behavior_path_planner/turn_signal_decider.hpp"
#include "behavior_path_planner/utilities.hpp"

namespace behavior_path_planner
{
autoware_vehicle_msgs::TurnSignal TurnSignalDecider::getTurnSignal(
  const autoware_planning_msgs::PathWithLaneId & path, const geometry_msgs::Pose & current_pose,
  const behavior_path_planner::RouteHandler & route_handler,
  const autoware_vehicle_msgs::TurnSignal & turn_signal_plan, const double plan_distance) const
{
  auto turn_signal = turn_signal_plan;

  // If the distance to intersection is nearer than path change point,
  // use turn signal for turning at the intersection
  const auto intersection_result = getIntersectionTurnSignal(path, current_pose, route_handler);
  const auto intersection_turn_signal = intersection_result.first;
  const auto intersection_distance = intersection_result.second;

  if (intersection_distance < plan_distance) turn_signal.data = intersection_turn_signal.data;

  // Set time stamp
  turn_signal.header.stamp = ros::Time::now();
  turn_signal.header.frame_id = "base_link";

  return turn_signal;
}

std::pair<autoware_vehicle_msgs::TurnSignal, double> TurnSignalDecider::getIntersectionTurnSignal(
  const autoware_planning_msgs::PathWithLaneId & path, const geometry_msgs::Pose & current_pose,
  const behavior_path_planner::RouteHandler & route_handler) const
{
  autoware_vehicle_msgs::TurnSignal turn_signal;
  turn_signal.data = autoware_vehicle_msgs::TurnSignal::NONE;
  double distance = std::numeric_limits<double>::max();

  if (path.points.empty()) {
    return std::make_pair(turn_signal, distance);
  }

  // Get frenet coordinate of current_pose on path
  util::FrenetCoordinate3d vehicle_pose_frenet;
  if (!util::convertToFrenetCoordinate3d(path, current_pose.position, &vehicle_pose_frenet)) {
    ROS_ERROR_THROTTLE(5, "failed to convert vehicle pose into frenet coordinate");
    return std::make_pair(turn_signal, distance);
  }

  // Get nearest intersection and decide turn signal
  double accumulated_distance = 0;

  auto prev_point = path.points.front();
  auto prev_lane_id = lanelet::InvalId;
  for (const auto & path_point : path.points) {
    accumulated_distance += autoware_utils::calcDistance3d(prev_point.point, path_point.point);
    prev_point = path_point;
    const double distance_from_vehicle_front =
      accumulated_distance - vehicle_pose_frenet.length - base_link2front_;
    if (distance_from_vehicle_front < 0.0) {
      continue;
    }
    // TODO Route Handler should be a library.
    for (const auto & lane : route_handler.getLaneletsFromIds(path_point.lane_ids)) {
      if (lane.id() == prev_lane_id) {
        continue;
      }
      prev_lane_id = lane.id();

      if (
        lane.attributeOr("turn_signal_distance", std::numeric_limits<double>::max()) <
        distance_from_vehicle_front) {
        if (1 < path_point.lane_ids.size() && lane.id() == path_point.lane_ids.back()) continue;
      }
      if (lane.attributeOr("turn_direction", std::string("none")) == "left") {
        turn_signal.data = autoware_vehicle_msgs::TurnSignal::LEFT;
        distance = distance_from_vehicle_front;
        return std::make_pair(turn_signal, distance);
      }
      if (lane.attributeOr("turn_direction", std::string("none")) == "right") {
        turn_signal.data = autoware_vehicle_msgs::TurnSignal::RIGHT;
        distance = distance_from_vehicle_front;
        return std::make_pair(turn_signal, distance);
      }
    }
    if (distance_from_vehicle_front > intersection_search_distance_) {
      return std::make_pair(turn_signal, distance);
    }
  }
  return std::make_pair(turn_signal, distance);
}
}  // namespace behavior_path_planner
