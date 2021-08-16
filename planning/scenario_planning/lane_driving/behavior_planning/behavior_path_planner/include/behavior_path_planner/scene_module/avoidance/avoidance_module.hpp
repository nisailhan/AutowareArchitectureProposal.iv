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

#ifndef BEHAVIOR_PATH_PLANNER_AVOIDANCE_MODULE_HPP
#define BEHAVIOR_PATH_PLANNER_AVOIDANCE_MODULE_HPP

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

#include "autoware_perception_msgs/DynamicObjectArray.h"
#include "autoware_planning_msgs/Path.h"
#include "autoware_planning_msgs/PathWithLaneId.h"
#include "autoware_vehicle_msgs/TurnSignal.h"

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "behavior_path_planner/path_shifter/path_shifter.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"

namespace behavior_path_planner
{
using autoware_planning_msgs::PathPointWithLaneId;
using autoware_planning_msgs::PathWithLaneId;

struct AvoidanceParameters
{
  // Vehicles whose distance to the center of the path is less than this will not be considered for avoidance.
  double threshold_distance_object_is_on_center;

  // vehicles with speed greater than this will not be avoided
  double threshold_speed_object_is_stopped;

  // distance to avoid object detection
  double object_check_forward_distance;

  // continue to detect backward vehicles as avoidance targets until they are this distance away
  double object_check_backward_distance;

  // we want to keep this lateral margin when avoiding
  double lateral_collision_margin;

  // we want to complete collision avoidance this distance in advance
  double longitudinal_collision_margin;

  // start avoidance after this time to avoid sudden path change
  double time_to_start_avoidance;

  // Even if the vehicle speed is zero, avoidance will start after a distance of this much.
  double min_distance_to_start_avoidance;

  // time to start avoidance TODO(Horibe): will be changed to jerk constraint later
  double time_avoiding;

  // minimum distance while avoiding TODO(Horibe): will be changed to jerk constraint later
  double min_distance_avoiding;

  // Even if the obstacle is very large, it will not avoid more than this length.
  double max_shift_length;

  // even if the vehicle speed is zero, avoidance will end after this much distance.
  double min_distance_avoidance_end_to_object;

  // we want to end the avoidance a few seconds before we pass by the avoided target.
  double time_avoidance_end_to_object;
};

struct Frenet
{
  Frenet(double lat, double lon) : lateral(lat), longitudinal(lon) {}
  Frenet() = default;
  double lateral = 0.0;
  double longitudinal = 0.0;
};

struct DebugData
{
  autoware_perception_msgs::DynamicObjectArray target_objects;
  std::shared_ptr<lanelet::ConstLanelets> expanded_lanelets;
  std::shared_ptr<lanelet::ConstLanelets> current_lanelets;
  std::vector<Frenet> raw_shift_points;
  std::vector<Frenet> modified_shift_points;
  std::vector<ShiftPoint> current_shift_points;
  std::vector<ShiftPoint> new_shift_points;
};

struct ObjectData  // avoidance target
{
  ObjectData() = default;
  ObjectData(
    const autoware_perception_msgs::DynamicObject & obj, double lat, double lon, double overhang)
  : object(obj), lateral(lat), longitudinal(lon), overhang_dist(overhang) {}
  autoware_perception_msgs::DynamicObject object;
  double lateral;
  double longitudinal;
  double overhang_dist;
};

struct AvoidancePlanningData
{
  geometry_msgs::Pose reference_pose;  // unshifted pose
  PathWithLaneId reference_path;
  lanelet::ConstLanelets current_lanelets;
  std::vector<ObjectData> objects;
};

class AvoidanceModule : public SceneModuleInterface
{
public:
  explicit AvoidanceModule(const std::string & name, const AvoidanceParameters & parameters);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BT::NodeStatus updateState() override;
  BehaviorModuleOutput plan() override;
  PathWithLaneId planCandidate() const override;
  BehaviorModuleOutput planWaitingApproval() override;
  void onEntry() override;
  void onExit() override;
  void updateData() override;

  void setParameters(const AvoidanceParameters & parameters);

private:
  AvoidanceParameters parameters_;
  AvoidancePlanningData avoidance_data_;
  std::shared_ptr<ShiftedPath> prev_output_ = nullptr;
  mutable DebugData debug_data_;

  PathShifter path_shifter_;

  void initVariables();

  AvoidancePlanningData calcAvoidancePlanningData() const;

  std::vector<ShiftPoint> calcShiftPointsInFrenet(DebugData & debug) const;

  boost::optional<ShiftPoint> calcIntersectionShiftPoint(const AvoidancePlanningData & data) const;

  boost::optional<ShiftPoint> findNewShiftPoint(
    const std::vector<ShiftPoint> & shift_points, const PathShifter & shifter) const;
  void addShiftPointIfApproved(const ShiftPoint & point);

  ShiftedPath generateAvoidancePath(PathShifter & shifter) const;
  void extendDrivableArea(ShiftedPath * shifted_path, double margin) const;

  void postProcess(PathShifter & shifter) const;

  // turn signal
  TurnSignalInfo calcTurnSignalInfo(const ShiftedPath & path) const;

  // helper
  std::vector<Frenet> endPointsToFrenet(const PathWithLaneId &path, const PathShifter &shifter) const;

  double getNominalAvoidanceDistance() const;
  double getNominalDistanceToStartAvoid() const;

  bool isTargetObjectType(const autoware_perception_msgs::DynamicObject & object) const;

  PathWithLaneId calcCenterLinePath(
    const std::shared_ptr<const PlannerData> & planner_data,
    const geometry_msgs::PoseStamped & pose) const;

  bool isAvoidancePlanRunning() const
  {
    const bool has_base_offset = std::abs(path_shifter_.getBaseOffset()) > 0.01;
    const bool has_shift_point = (path_shifter_.getShiftPointsSize() > 0);
    return has_base_offset || has_shift_point;
  }

  inline double getEgoSpeed() const
  {
    return std::abs(planner_data_->self_velocity->twist.linear.x);
  }

  geometry_msgs::PoseStamped getUnshiftedEgoPose(const ShiftedPath & prev_path) const;

  inline geometry_msgs::PoseStamped getEgoPose() const { return *(planner_data_->self_pose); }

  inline geometry_msgs::Point getEgoPosition() const
  {
    return planner_data_->self_pose->pose.position;
  }

  ShiftedPath toShiftedPath(const PathWithLaneId & path) const;

  void clipPathLength(PathWithLaneId & path) const;

  void setDebugData(const PathShifter & shifter, const DebugData & debug);
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER_AVOIDANCE_MODULE_HPP
