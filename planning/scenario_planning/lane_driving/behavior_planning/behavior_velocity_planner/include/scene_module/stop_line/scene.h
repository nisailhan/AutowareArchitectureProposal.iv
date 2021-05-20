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
#include <unordered_map>
#include <vector>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/scene_module_interface.h>
#include <utilization/boost_geometry_helper.h>

class StopLineModule : public SceneModuleInterface
{
public:
  enum class State { APPROACH, STOPPED, START };

  struct SegmentIndexWithPose
  {
    size_t index;
    geometry_msgs::Pose pose;
  };

  struct SegmentIndexWithPoint2d
  {
    size_t index;
    Point2d point;
  };

  struct SegmentIndexWithOffset
  {
    size_t index;
    double offset;
  };

  struct DebugData
  {
    double base_link2front;
    boost::optional<geometry_msgs::Pose> stop_pose;
  };

  struct PlannerParam
  {
    double stop_margin;
    double stop_check_dist;
    double stop_duration_sec;
  };

public:
  StopLineModule(
    const int64_t module_id, const lanelet::ConstLineString3d & stop_line,
    const PlannerParam & planner_param);

  bool modifyPathVelocity(
    autoware_planning_msgs::PathWithLaneId * path,
    autoware_planning_msgs::StopReason * stop_reason) override;

  visualization_msgs::MarkerArray createDebugMarkerArray() override;

private:
  int64_t module_id_;

  geometry_msgs::Point getCenterOfStopLine(const lanelet::ConstLineString3d & stop_line);

  boost::optional<StopLineModule::SegmentIndexWithPoint2d> findCollision(
    const autoware_planning_msgs::PathWithLaneId & path, const LineString2d & stop_line);

  boost::optional<StopLineModule::SegmentIndexWithOffset> findOffsetSegment(
    const autoware_planning_msgs::PathWithLaneId & path,
    const StopLineModule::SegmentIndexWithPoint2d & collision);

  boost::optional<StopLineModule::SegmentIndexWithPose> calcStopPose(
    const autoware_planning_msgs::PathWithLaneId & path,
    const boost::optional<StopLineModule::SegmentIndexWithOffset> & offset_segment);

  autoware_planning_msgs::PathWithLaneId insertStopPose(
    const autoware_planning_msgs::PathWithLaneId & path,
    const StopLineModule::SegmentIndexWithPose & insert_index_with_pose,
    autoware_planning_msgs::StopReason * stop_reason);

  lanelet::ConstLineString3d stop_line_;
  State state_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;
};
