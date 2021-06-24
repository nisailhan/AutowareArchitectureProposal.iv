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

#ifndef BEHAVIOR_PATH_PLANNER_APPROVAL_HANDLER_HPP
#define BEHAVIOR_PATH_PLANNER_APPROVAL_HANDLER_HPP

#include <memory>
#include <string>

#include "ros/ros.h"

namespace behavior_path_planner
{
class ApprovalHandler
{
public:
  ApprovalHandler() : approval_(false) {}
  void setCurrentApproval(const behavior_path_planner::BoolStamped & approval)
  {
    approval_ = approval;
  }

  bool isApproved() const
  {
    const auto now = ros::Time::now();
    const auto thresh_sec = 0.5;
    if (approval_.data && (now - approval_.stamp).toSec() < thresh_sec) {
      if ((now - last_clear_time_).toSec() > thresh_sec) {
        return true;
      }
    }
    return false;
  }

  bool isWaitingApproval() const { return is_waiting_approval_; }
  void waitApproval() { is_waiting_approval_ = true; }
  void clearWaitApproval() { is_waiting_approval_ = false; }
  void clearApproval() {
    last_clear_time_ = ros::Time::now();
    approval_.data = false;
  }

private:
  behavior_path_planner::BoolStamped approval_;
  bool is_waiting_approval_ = true;
  ros::Time last_clear_time_;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER_SCENE_MODULE_INTERFACE_HPP
