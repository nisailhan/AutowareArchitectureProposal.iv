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

#ifndef BEHAVIOR_PATH_PLANNER_DATA_MANAGER_HPP
#define BEHAVIOR_PATH_PLANNER_DATA_MANAGER_HPP

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"
#include "tf2_ros/transform_listener.h"

#include "autoware_lanelet2_msgs/MapBin.h"
#include "autoware_perception_msgs/DynamicObjectArray.h"
#include "autoware_planning_msgs/PathWithLaneId.h"
#include "autoware_planning_msgs/Route.h"

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_traffic_rules/TrafficRulesFactory.h"

#include <memory>
#include <string>

#include "behavior_path_planner/parameters.hpp"
#include "behavior_path_planner/route_handler.hpp"

namespace behavior_path_planner
{
struct BoolStamped
{
  explicit BoolStamped(bool in_data) : data(in_data) {}
  bool data = false;
  ros::Time stamp;
};

struct ModuleNameStamped
{
  std::string module_name = "NONE";
  ros::Time stamp;
};

struct Approval
{
  BoolStamped is_approved{false};
  ModuleNameStamped is_force_approved{};
};

struct PlannerData
{
  geometry_msgs::PoseStamped::ConstPtr self_pose{};
  geometry_msgs::TwistStamped::ConstPtr self_velocity{};
  autoware_perception_msgs::DynamicObjectArray::ConstPtr dynamic_object{};
  std::shared_ptr<autoware_planning_msgs::PathWithLaneId> reference_path{
    std::make_shared<autoware_planning_msgs::PathWithLaneId>()};
  std::shared_ptr<autoware_planning_msgs::PathWithLaneId> prev_output_path{
    std::make_shared<autoware_planning_msgs::PathWithLaneId>()};
  BehaviorPathPlannerParameters parameters{};
  lanelet::ConstLanelets current_lanes{};
  std::shared_ptr<RouteHandler> route_handler{std::make_shared<RouteHandler>()};
  Approval approval{};
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER_DATA_MANAGER_HPP
