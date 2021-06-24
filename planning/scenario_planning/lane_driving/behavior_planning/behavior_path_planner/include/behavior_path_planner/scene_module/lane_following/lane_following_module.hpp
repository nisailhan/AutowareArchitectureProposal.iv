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

#ifndef BEHAVIOR_PATH_PLANNER_LANE_FOLLOWING_MODULE_HPP
#define BEHAVIOR_PATH_PLANNER_LANE_FOLLOWING_MODULE_HPP

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
struct LaneFollowingParameters
{
  double lane_change_prepare_duration;
};

class LaneFollowingModule : public SceneModuleInterface
{
public:
  explicit LaneFollowingModule(
    const std::string & name, const LaneFollowingParameters & parameters);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BT::NodeStatus updateState() override;
  BehaviorModuleOutput plan() override;
  autoware_planning_msgs::PathWithLaneId planCandidate() const override;
  void onEntry() override;
  void onExit() override;

  void setParameters(const LaneFollowingParameters & parameters);

private:
  LaneFollowingParameters parameters_;
  autoware_planning_msgs::PathWithLaneId getReferencePath() const;
  void initParam();
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER_LANE_FOLLOWING_MODULE_HPP
