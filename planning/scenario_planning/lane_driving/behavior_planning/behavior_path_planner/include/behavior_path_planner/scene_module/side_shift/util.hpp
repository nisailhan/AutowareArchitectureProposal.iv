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

#pragma once

#include <memory>
#include <string>

#include "autoware_lanelet2_msgs/MapBin.h"
#include "autoware_perception_msgs/DynamicObjectArray.h"
#include "autoware_planning_msgs/Path.h"
#include "autoware_planning_msgs/PathWithLaneId.h"
#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_extension/utility/query.h"
#include "lanelet2_extension/utility/utilities.h"
#include "ros/ros.h"
#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"

namespace behavior_path_planner
{
void setOrientation(autoware_planning_msgs::PathWithLaneId * path);

bool getStartAvoidPose(
  const autoware_planning_msgs::PathWithLaneId & path, const double start_distance,
  const size_t nearest_idx, geometry_msgs::Pose * start_avoid_pose);

bool isAlmostZero(double v);

geometry_msgs::Point transformToGrid(
  const geometry_msgs::Point & pt, const double longitudinal_offset, const double lateral_offset,
  const double yaw, const geometry_msgs::TransformStamped & geom_tf);

}  // namespace behavior_path_planner
