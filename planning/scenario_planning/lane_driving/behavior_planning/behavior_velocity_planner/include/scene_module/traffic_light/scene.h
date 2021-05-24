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

#include <memory>
#include <string>
#include <unordered_map>

#include <boost/assert.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/scene_module_interface.h>

class TrafficLightModule : public SceneModuleInterface
{
public:
  enum class State { APPROACH, GO_OUT };
  enum class Input { PERCEPTION, EXTERNAL, NONE };  // EXTERNAL: FOA, V2X, etc.

  struct DebugData
  {
    double base_link2front;
    std::vector<std::tuple<
      std::shared_ptr<const lanelet::TrafficLight>, autoware_perception_msgs::TrafficLightState>>
      tl_state;  // TODO: replace tuple with struct
    std::vector<geometry_msgs::Pose> stop_poses;
    geometry_msgs::Pose first_stop_pose;
    std::vector<geometry_msgs::Pose> dead_line_poses;
    std::vector<geometry_msgs::Point> traffic_light_points;
  };

  struct PlannerParam
  {
    double stop_margin;
    double tl_state_timeout;
    double external_tl_state_timeout;
    bool enable_pass_judge;
  };

public:
  TrafficLightModule(
    const int64_t module_id, const lanelet::TrafficLight & traffic_light_reg_elem,
    lanelet::ConstLanelet lane, const PlannerParam & planner_param);

  bool modifyPathVelocity(
    autoware_planning_msgs::PathWithLaneId * path,
    autoware_planning_msgs::StopReason * stop_reason) override;

  visualization_msgs::MarkerArray createDebugMarkerArray() override;

  inline autoware_perception_msgs::LookingTrafficLightState getTrafficLightState() const
  {
    return looking_tl_state_;
  };
  inline State getTrafficLightModuleState() const { return state_; };
  inline Input getTrafficLightModuleInput() const { return input_; };

private:
  int64_t lane_id_;

  bool getBackwardPointFromBasePoint(
    const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
    const Eigen::Vector2d & base_point, const double backward_length,
    Eigen::Vector2d & output_point);

  bool insertTargetVelocityPoint(
    const autoware_planning_msgs::PathWithLaneId & input,
    const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<double>> &
      stop_line,
    const double & margin, const double & velocity,
    autoware_planning_msgs::PathWithLaneId & output);

  bool getHighestConfidenceTrafficLightState(
    const lanelet::ConstLineStringsOrPolygons3d & traffic_lights,
    autoware_perception_msgs::TrafficLightStateStamped & highest_confidence_tl_state);

  bool isOverDeadLine(
    const geometry_msgs::Pose & self_pose,
    const autoware_planning_msgs::PathWithLaneId & input_path, const size_t & dead_line_point_idx,
    const Eigen::Vector2d & dead_line_point, const double dead_line_range);

  bool updateTrafficLightState(const lanelet::ConstLineStringsOrPolygons3d traffic_lights);

  bool isStopRequired(const autoware_perception_msgs::TrafficLightState & tl_state);

  bool finalJudgeStopRequired(
    const bool & stoppable, const double & reachable_distance, const double & signed_arc_length);

  bool createTargetPoint(
    const autoware_planning_msgs::PathWithLaneId & input,
    const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<double>> &
      stop_line,
    const double & margin, size_t & target_point_idx, Eigen::Vector2d & target_point);

  bool hasLamp(
    const autoware_perception_msgs::TrafficLightState & tl_state, const uint8_t & lamp_color);

  geometry_msgs::Point getTrafficLightPosition(
    const lanelet::ConstLineStringOrPolygon3d traffic_light);

  bool getExternalTrafficLightState(
    const lanelet::ConstLineStringsOrPolygons3d & traffic_lights,
    autoware_perception_msgs::TrafficLightStateStamped & external_tl_state);

  // Key Feature
  const lanelet::TrafficLight & traffic_light_reg_elem_;
  lanelet::ConstLanelet lane_;

  // State
  State state_;

  // Input
  Input input_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;

  // prevent paththrough chattering
  bool is_prev_state_stop_;

  // Traffic Light State
  autoware_perception_msgs::LookingTrafficLightState looking_tl_state_;
};
