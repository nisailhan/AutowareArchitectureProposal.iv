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
#include <scene_module/intersection/manager.h>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include "utilization/util.h"

namespace
{
std::vector<lanelet::ConstLanelet> getLaneletsOnPath(
  const autoware_planning_msgs::PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map)
{
  std::vector<lanelet::ConstLanelet> lanelets;

  for (const auto & p : path.points) {
    const auto lane_id = p.lane_ids.at(0);
    const auto lane = lanelet_map->laneletLayer.get(lane_id);
    if (!lanelet::utils::contains(lanelets, lane)) {
      lanelets.push_back(lane);
    }
  }

  return lanelets;
}

std::set<int64_t> getLaneIdSetOnPath(const autoware_planning_msgs::PathWithLaneId & path)
{
  std::set<int64_t> lane_id_set;

  for (const auto & p : path.points) {
    for (const auto & lane_id : p.lane_ids) {
      lane_id_set.insert(lane_id);
    }
  }

  return lane_id_set;
}

}  // namespace

IntersectionModuleManager::IntersectionModuleManager()
: SceneModuleManagerInterface(getModuleName())
{
  ros::NodeHandle pnh("~");
  const std::string ns(getModuleName());

  pnh.param(ns + "/state_transit_margin_time", intersection_param_.state_transit_margin_time, 2.0);
  pnh.param(ns + "/decel_velocity", intersection_param_.decel_velocity, 30.0 / 3.6);
  pnh.param(ns + "/path_expand_width", intersection_param_.path_expand_width, 2.0);
  pnh.param(ns + "/stop_line_margin", intersection_param_.stop_line_margin, 1.0);
  pnh.param(ns + "/stuck_vehicle_detect_dist", intersection_param_.stuck_vehicle_detect_dist, 3.0);
  pnh.param(ns + "/stuck_vehicle_ignore_dist", intersection_param_.stuck_vehicle_ignore_dist, 5.0) +
    planner_data_->base_link2front;
  pnh.param(ns + "/stuck_vehicle_vel_thr", intersection_param_.stuck_vehicle_vel_thr, 3.0 / 3.6);
  pnh.param(ns + "/intersection_velocity", intersection_param_.intersection_velocity, 10.0 / 3.6);
  pnh.param(ns + "/intersection_max_accel", intersection_param_.intersection_max_acc, 0.5);
  pnh.param(ns + "/detection_area_length", intersection_param_.detection_area_length, 200.0);
  pnh.param(ns + "/intersection_max_accel", intersection_param_.intersection_max_acc, 0.5);
  pnh.param(ns + "/walkway/external_input_timeout", intersection_param_.external_input_timeout, 1.0);
  pnh.param(ns + "/merge_from_private_area/stop_duration_sec", merge_from_private_area_param_.stop_duration_sec, 1.0);
  merge_from_private_area_param_.intersection_param = intersection_param_;
}

void IntersectionModuleManager::launchNewModules(
  const autoware_planning_msgs::PathWithLaneId & path)
{
  const auto lanelets = getLaneletsOnPath(path, planner_data_->lanelet_map);
  for (size_t i = 0; i < lanelets.size(); i++) {
    const auto ll = lanelets.at(i);
    const auto lane_id = ll.id();
    const auto module_id = lane_id;

    if (isModuleRegistered(module_id)) {
      continue;
    }

    // Is intersection?
    const std::string turn_direction = ll.attributeOr("turn_direction", "else");
    const auto is_intersection =
      turn_direction == "right" || turn_direction == "left" || turn_direction == "straight";
    if (!is_intersection) {
      continue;
    }

    // Is merging from private road?
    if (i + 1 < lanelets.size()) {
      const auto next_lane = lanelets.at(i + 1);
      const std::string lane_location = ll.attributeOr("location", "else");
      const std::string next_lane_location = next_lane.attributeOr("location", "else");
      if (lane_location == "private" && next_lane_location != "private") {
        registerModule(std::make_shared<MergeFromPrivateRoadModule>(
          module_id, lane_id, planner_data_, merge_from_private_area_param_));
      }
    }

    registerModule(
      std::make_shared<IntersectionModule>(module_id, lane_id, planner_data_, intersection_param_));
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
IntersectionModuleManager::getModuleExpiredFunction(
  const autoware_planning_msgs::PathWithLaneId & path)
{
  const auto lane_id_set = getLaneIdSetOnPath(path);

  return [lane_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return lane_id_set.count(scene_module->getModuleId()) == 0;
  };
}
