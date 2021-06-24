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

#pragma once

#include <string>
#include <vector>

#include "autoware_utils/ros/marker_helper.h"
#include "behavior_path_planner/scene_module/avoidance/avoidance_module.hpp"
#include "behavior_path_planner/path_shifter/path_shifter.hpp"


#include "autoware_perception_msgs/DynamicObjectArray.h"
#include "autoware_planning_msgs/PathWithLaneId.h"

#include "geometry_msgs/Polygon.h"
#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "visualization_msgs/MarkerArray.h"

namespace marker_utils
{
visualization_msgs::MarkerArray createShiftPointMarkerArray(
  const std::vector<behavior_path_planner::ShiftPoint> & shift_points, const double base_shift,
  const std::string & ns, const double r, const double g, const double b);

visualization_msgs::MarkerArray createFrenetPointMarkerArray(
  const std::vector<behavior_path_planner::Frenet> & frenet_points,
  const autoware_planning_msgs::PathWithLaneId & path, const geometry_msgs::Point & ego_point,
  const std::string & ns, const double r, const double g, const double b);

visualization_msgs::MarkerArray createLaneletsAreaMarkerArray(
  const std::vector<lanelet::ConstLanelet> & lanelets, const std::string & ns,
  const double r, const double g, const double b);

visualization_msgs::MarkerArray createLaneletPolygonsMarkerArray(
  const std::vector<lanelet::CompoundPolygon3d> & polygons, const std::string & ns,
  const int64_t lane_id);

visualization_msgs::MarkerArray createPolygonMarkerArray(
  const geometry_msgs::Polygon & polygon, const std::string & ns, const int64_t lane_id,
  const double r, const double g, const double b);

visualization_msgs::MarkerArray createObjectsMarkerArray(
  const autoware_perception_msgs::DynamicObjectArray & objects, const std::string & ns,
  const int64_t lane_id, const double r, const double g, const double b);

visualization_msgs::MarkerArray createPathMarkerArray(
  const autoware_planning_msgs::PathWithLaneId & path, const std::string & ns,
  const int64_t lane_id, const double r, const double g, const double b);

visualization_msgs::MarkerArray createVirtualWallMarkerArray(
  const geometry_msgs::Pose & pose, const int64_t lane_id, const std::string & stop_factor);

visualization_msgs::MarkerArray createPoseMarkerArray(
  const geometry_msgs::Pose & pose, const std::string & ns, const int64_t id, const double r,
  const double g, const double b);

}  // namespace marker_utils
