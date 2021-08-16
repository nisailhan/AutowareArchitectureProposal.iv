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

#include <awapi_awiv_adapter/awapi_autoware_state_publisher.h>
#include <regex>

namespace autoware_api
{
AutowareIvAutowareStatePublisher::AutowareIvAutowareStatePublisher()
: nh_(), pnh_("~"), arrived_goal_(false)
{
  // publisher
  pub_state_ = pnh_.advertise<autoware_api_msgs::AwapiAutowareStatus>("output/autoware_status", 1);
}

void AutowareIvAutowareStatePublisher::statePublisher(const AutowareInfo & aw_info)
{
  autoware_api_msgs::AwapiAutowareStatus status;

  //input header
  status.header.frame_id = "base_link";
  status.header.stamp = ros::Time::now();

  // get all info
  getAutowareStateInfo(aw_info.autoware_state_ptr, &status);
  getControlModeInfo(aw_info.control_mode_ptr, &status);
  getGateModeInfo(aw_info.gate_mode_ptr, &status);
  getIsEmergencyInfo(aw_info.is_emergency_ptr, &status);
  getHazardStatusInfo(aw_info.hazard_status_ptr, &status);
  getStopReasonInfo(aw_info.stop_reason_ptr, &status);
  getDiagInfo(aw_info, &status);
  getErrorDiagInfo(aw_info, &status);
  getGlobalRptInfo(aw_info.global_rpt_ptr, &status);

  // publish info
  pub_state_.publish(status);
}

void AutowareIvAutowareStatePublisher::getAutowareStateInfo(
  const autoware_system_msgs::AutowareState::ConstPtr & autoware_state_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!autoware_state_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] autoware_state is nullptr");
    return;
  }

  // get autoware_state
  status->autoware_state = autoware_state_ptr->state;
  status->arrived_goal = isGoal(autoware_state_ptr);
}

void AutowareIvAutowareStatePublisher::getControlModeInfo(
  const autoware_vehicle_msgs::ControlMode::ConstPtr & control_mode_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!control_mode_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] control mode is nullptr");
    return;
  }

  // get control mode
  status->control_mode = control_mode_ptr->data;
}

void AutowareIvAutowareStatePublisher::getGateModeInfo(
  const autoware_control_msgs::GateMode::ConstPtr & gate_mode_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!gate_mode_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] gate mode is nullptr");
    return;
  }

  // get control mode
  status->gate_mode = gate_mode_ptr->data;
}

void AutowareIvAutowareStatePublisher::getIsEmergencyInfo(
  const std_msgs::Bool::ConstPtr & is_emergency_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!is_emergency_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] is_emergency is nullptr");
    return;
  }

  // get emergency
  status->emergency_stopped = is_emergency_ptr->data;
}

void AutowareIvAutowareStatePublisher::getHazardStatusInfo(
  const autoware_system_msgs::HazardStatusStamped::ConstPtr & hazard_status_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!hazard_status_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] hazard_status is nullptr");
    return;
  }

  // get emergency
  status->hazard_status = *hazard_status_ptr;
}

void AutowareIvAutowareStatePublisher::getStopReasonInfo(
  const autoware_planning_msgs::StopReasonArray::ConstPtr & stop_reason_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!stop_reason_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] stop reason is nullptr");
    return;
  }

  status->stop_reason = *stop_reason_ptr;
}

void AutowareIvAutowareStatePublisher::getDiagInfo(
  const AutowareInfo & aw_info, autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!aw_info.diagnostic_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] diagnostics is nullptr");
    return;
  }

  // get diag
  status->diagnostics = extractLeafDiag(aw_info.diagnostic_ptr->status);
}

// This function is tentative and should be replaced with getHazardStatusInfo.
// TODO(Kenji Miyake): Make getErrorDiagInfo users to use getHazardStatusInfo.
void AutowareIvAutowareStatePublisher::getErrorDiagInfo(
  const AutowareInfo & aw_info, autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!aw_info.autoware_state_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] autoware_state is nullptr");
    return;
  }

  if (!aw_info.control_mode_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] control_mode is nullptr");
    return;
  }

  if (!aw_info.diagnostic_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] diagnostics is nullptr");
    return;
  }

  if (!aw_info.hazard_status_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] hazard_status is nullptr");
    return;
  }

  // filter by state
  if (aw_info.autoware_state_ptr->state != autoware_system_msgs::AutowareState::Emergency) {
    status->error_diagnostics = {};
    return;
  }

  // filter by control_mode
  if (aw_info.control_mode_ptr->data == autoware_vehicle_msgs::ControlMode::MANUAL) {
    status->error_diagnostics = {};
    return;
  }

  // get diag
  using diagnostic_msgs::DiagnosticStatus;
  const auto & hazard_status = aw_info.hazard_status_ptr->status;
  std::vector<diagnostic_msgs::DiagnosticStatus> error_diagnostics;

  for (const auto & hazard_diag : hazard_status.diagnostics_spf) {
    auto diag = hazard_diag;
    diag.level = DiagnosticStatus::ERROR;
    error_diagnostics.push_back(hazard_diag);
  }
  for (const auto & hazard_diag : hazard_status.diagnostics_lf) {
    auto diag = hazard_diag;
    diag.level = DiagnosticStatus::ERROR;
    error_diagnostics.push_back(hazard_diag);
  }
  for (const auto & hazard_diag : hazard_status.diagnostics_sf) {
    auto diag = hazard_diag;
    diag.level = DiagnosticStatus::WARN;
    error_diagnostics.push_back(hazard_diag);
  }
  for (const auto & hazard_diag : hazard_status.diagnostics_nf) {
    auto diag = hazard_diag;
    diag.level = DiagnosticStatus::OK;
    error_diagnostics.push_back(hazard_diag);
  }

  // filter leaf diag
  status->error_diagnostics = extractLeafDiag(error_diagnostics);
}

void AutowareIvAutowareStatePublisher::getGlobalRptInfo(
  const pacmod_msgs::GlobalRpt::ConstPtr & global_rpt_ptr,
  autoware_api_msgs::AwapiAutowareStatus * status)
{
  if (!global_rpt_ptr) {
    ROS_DEBUG_STREAM_THROTTLE(5.0, "[AutowareIvAutowareStatePublisher] global_rpt is nullptr");
    return;
  }

  // get global_rpt
  status->autonomous_overriden = global_rpt_ptr->override_active;
}

bool AutowareIvAutowareStatePublisher::isGoal(
  const autoware_system_msgs::AutowareState::ConstPtr & autoware_state)
{
  //rename
  const auto & aw_state = autoware_state->state;

  if (aw_state == autoware_system_msgs::AutowareState::ArrivedGoal) {
    arrived_goal_ = true;
  } else if (
    prev_state_ == autoware_system_msgs::AutowareState::Driving &&
    aw_state == autoware_system_msgs::AutowareState::WaitingForRoute) {
    arrived_goal_ = true;
  }

  if (
    aw_state == autoware_system_msgs::AutowareState::WaitingForEngage ||
    aw_state == autoware_system_msgs::AutowareState::Driving) {
    //cancel goal state
    arrived_goal_ = false;
  }

  prev_state_ = aw_state;

  return arrived_goal_;
}

std::vector<diagnostic_msgs::DiagnosticStatus> AutowareIvAutowareStatePublisher::extractLeafDiag(
  const std::vector<diagnostic_msgs::DiagnosticStatus> & diag_vec)
{
  updateDiagNameSet(diag_vec);

  std::vector<diagnostic_msgs::DiagnosticStatus> leaf_diag_info;
  for (const auto diag : diag_vec) {
    if (isLeaf(diag)) {
      leaf_diag_info.emplace_back(diag);
    }
  }
  return leaf_diag_info;
}

std::string AutowareIvAutowareStatePublisher::splitStringByLastSlash(const std::string & str)
{
  const auto last_slash = str.find_last_of("/");

  if (last_slash == std::string::npos) {
    // if not find slash
    return str;
  }

  return str.substr(0, last_slash);
}

void AutowareIvAutowareStatePublisher::updateDiagNameSet(
  const std::vector<diagnostic_msgs::DiagnosticStatus> & diag_vec)
{
  // set diag name to diag_name_set_
  for (const auto & diag : diag_vec) {
    diag_name_set_.insert(splitStringByLastSlash(diag.name));
  }
}

bool AutowareIvAutowareStatePublisher::isLeaf(const diagnostic_msgs::DiagnosticStatus & diag)
{
  //if not find diag.name in diag set, diag is leaf.
  return diag_name_set_.find(diag.name) == diag_name_set_.end();
}

}  // namespace autoware_api
