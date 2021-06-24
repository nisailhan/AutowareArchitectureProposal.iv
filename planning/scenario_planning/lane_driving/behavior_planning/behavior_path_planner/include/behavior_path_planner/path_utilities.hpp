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

#ifndef BEHAVIOR_PATH_PLANNER_PATH_UTILITIES_HPP
#define BEHAVIOR_PATH_PLANNER_PATH_UTILITIES_HPP

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "autoware_perception_msgs/DynamicObjectArray.h"
#include "autoware_planning_msgs/Path.h"
#include "autoware_planning_msgs/PathWithLaneId.h"

#include "boost/geometry/geometries/box.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "boost/geometry/geometries/polygon.hpp"
#include "boost/geometry/geometry.hpp"

#include "lanelet2_core/geometry/Lanelet.h"
#include "lanelet2_routing/Route.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_routing/RoutingGraphContainer.h"

#include <limits>
#include <vector>

#include "opencv2/opencv.hpp"

#include "behavior_path_planner/route_handler.hpp"

namespace behavior_path_planner
{
namespace util
{
std::vector<double> calcPathArcLengthArray(
  const autoware_planning_msgs::PathWithLaneId & path, size_t start, size_t end);

double calcPathArcLength(
  const autoware_planning_msgs::PathWithLaneId & path, size_t start, size_t end);

autoware_planning_msgs::PathWithLaneId resamplePathWithSpline(
  const autoware_planning_msgs::PathWithLaneId & path, double interval);

autoware_planning_msgs::Path toPath(const autoware_planning_msgs::PathWithLaneId & input);

size_t getIdxByArclength(
  const autoware_planning_msgs::PathWithLaneId & path, const geometry_msgs::Point & origin,
  const double signed_arc);

}  // namespace util
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER_PATH_UTILITIES_HPP
