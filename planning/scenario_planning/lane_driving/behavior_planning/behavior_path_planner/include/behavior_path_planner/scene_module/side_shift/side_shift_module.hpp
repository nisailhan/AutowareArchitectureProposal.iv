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

#ifndef BEHAVIOR_PATH_PLANNER_SIDE_SHIFT_MODULE_HPP
#define BEHAVIOR_PATH_PLANNER_SIDE_SHIFT_MODULE_HPP

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

#include "autoware_planning_msgs/Path.h"
#include "autoware_planning_msgs/PathWithLaneId.h"

#include <memory>
#include <string>

#include "behavior_path_planner/scene_module/scene_module_interface.hpp"

namespace behavior_path_planner
{
struct SideShiftParameters
{
  double min_fix_distance;
  double start_avoid_sec;
  double drivable_area_resolution;
  double drivable_area_width;
  double drivable_area_height;
  double inside_outside_judge_margin;
};

class SideShiftModule : public SceneModuleInterface
{
public:
  explicit SideShiftModule(const std::string & name, const SideShiftParameters & parameters);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BT::NodeStatus updateState() override;
  void updateData() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  autoware_planning_msgs::PathWithLaneId planCandidate() const override;
  void onEntry() override;
  void onExit() override;

  void setParameters(const SideShiftParameters & parameters);

private:
  ros::NodeHandle pnh_;
  ros::Subscriber lateral_offset_subscriber_;

  void initVariables();

  void onLateralOffset(const std_msgs::Float32::ConstPtr & lateral_offset_msg);

  // non-const methods
  // TODO(Horibe) cleanup args
  autoware_planning_msgs::PathWithLaneId refinePath(
    const autoware_planning_msgs::PathWithLaneId & input_path, double lateral_offset,
    bool & start_pose_reset_request, bool & drivable_area_shrink_request,
    double & drivable_area_extention_width, geometry_msgs::Pose & start_avoid_pose) const;

  // const methods
  void extendDrivableArea(
    autoware_planning_msgs::PathWithLaneId & path, double lateral_offset,
    const RouteHandler & route_handler) const;
  bool isVehicleInDrivableArea(const nav_msgs::OccupancyGrid & drivable_area) const;
  void publishPath(const autoware_planning_msgs::PathWithLaneId & path) const;

  // member
  autoware_planning_msgs::PathWithLaneId refined_path_;
  std::shared_ptr<autoware_planning_msgs::PathWithLaneId> reference_path_;
  lanelet::ConstLanelets current_lanelets_;
  SideShiftParameters parameters_;

  // Current lateral offset to shift the reference path.
  double lateral_offset_;

  // lateral offset used for previous planned path
  double prev_planned_lateral_offset_;

  // Triggered when offset is changed, released when start pose is refound.
  bool start_pose_reset_request_;

  // To keep the consistency in each planning.
  geometry_msgs::Pose start_avoid_pose_;

  // Triggered when received smaller lateral offset, released when the drivable area is shrunk
  bool drivable_area_shrink_request_;

  // For the drivable area extension.
  double drivable_area_extention_width_;

  //
  bool is_approval_needed_;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER_SIDE_SHIFT_MODULE_HPP
