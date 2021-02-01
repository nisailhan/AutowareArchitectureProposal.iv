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

#include "external_cmd_selector/external_cmd_selector_node.hpp"

ExternalCmdSelector::ExternalCmdSelector()
{
  // Parameter
  pnh_.param("update_rate", update_rate_, 10.0);
  pnh_.param("initial_selector_mode", initial_selector_mode_, 0);

  // Publisher
  pub_current_selector_mode_ = pnh_.advertise<autoware_control_msgs::RemoteCommandSelectorMode>(
    "output/current_selector_mode", 1);

  pub_raw_control_cmd_ =
    pnh_.advertise<autoware_vehicle_msgs::RawControlCommandStamped>("output/raw_control_cmd", 1);
  pub_shift_cmd_ = pnh_.advertise<autoware_vehicle_msgs::ShiftStamped>("output/shift_cmd", 1);
  pub_turn_signal_cmd_ =
    pnh_.advertise<autoware_vehicle_msgs::TurnSignal>("output/turn_signal_cmd", 1);

  // Subscriber
  sub_local_raw_control_cmd_ = pnh_.subscribe(
    "input/local/raw_control_cmd", 1, &ExternalCmdSelector::onLocalRawControlCmd, this);
  sub_local_shift_cmd_ =
    pnh_.subscribe("input/local/shift_cmd", 1, &ExternalCmdSelector::onLocalShiftCmd, this);
  sub_local_turn_signal_cmd_ = pnh_.subscribe(
    "input/local/turn_signal_cmd", 1, &ExternalCmdSelector::onLocalTurnSignalCmd, this);

  sub_remote_raw_control_cmd_ = pnh_.subscribe(
    "input/remote/raw_control_cmd", 1, &ExternalCmdSelector::onRemoteRawControlCmd, this);
  sub_remote_shift_cmd_ =
    pnh_.subscribe("input/remote/shift_cmd", 1, &ExternalCmdSelector::onRemoteShiftCmd, this);
  sub_remote_turn_signal_cmd_ = pnh_.subscribe(
    "input/remote/turn_signal_cmd", 1, &ExternalCmdSelector::onRemoteTurnSignalCmd, this);

  // Service
  srv_select_external_command_ = pnh_.advertiseService(
    "service/select_external_command", &ExternalCmdSelector::onSelectRemoteCommandService, this);

  // Initialize mode
  current_selector_mode_.data = initial_selector_mode_;

  // Timer
  timer_ = pnh_.createTimer(ros::Rate(update_rate_), &ExternalCmdSelector::onTimer, this);
}

void ExternalCmdSelector::onLocalRawControlCmd(
  const autoware_vehicle_msgs::RawControlCommandStamped::ConstPtr msg)
{
  if (current_selector_mode_.data != autoware_control_msgs::RemoteCommandSelectorMode::LOCAL) {
    return;
  }

  pub_raw_control_cmd_.publish(msg);
}

void ExternalCmdSelector::onLocalShiftCmd(const autoware_vehicle_msgs::ShiftStamped::ConstPtr msg)
{
  if (current_selector_mode_.data != autoware_control_msgs::RemoteCommandSelectorMode::LOCAL) {
    return;
  }

  pub_shift_cmd_.publish(msg);
}

void ExternalCmdSelector::onLocalTurnSignalCmd(
  const autoware_vehicle_msgs::TurnSignal::ConstPtr msg)
{
  if (current_selector_mode_.data != autoware_control_msgs::RemoteCommandSelectorMode::LOCAL) {
    return;
  }

  pub_turn_signal_cmd_.publish(msg);
}

void ExternalCmdSelector::onRemoteRawControlCmd(
  const autoware_vehicle_msgs::RawControlCommandStamped::ConstPtr msg)
{
  if (current_selector_mode_.data != autoware_control_msgs::RemoteCommandSelectorMode::REMOTE) {
    return;
  }

  pub_raw_control_cmd_.publish(msg);
}

void ExternalCmdSelector::onRemoteShiftCmd(const autoware_vehicle_msgs::ShiftStamped::ConstPtr msg)
{
  if (current_selector_mode_.data != autoware_control_msgs::RemoteCommandSelectorMode::REMOTE) {
    return;
  }

  pub_shift_cmd_.publish(msg);
}

void ExternalCmdSelector::onRemoteTurnSignalCmd(
  const autoware_vehicle_msgs::TurnSignal::ConstPtr msg)
{
  if (current_selector_mode_.data != autoware_control_msgs::RemoteCommandSelectorMode::REMOTE) {
    return;
  }

  pub_turn_signal_cmd_.publish(msg);
}

bool ExternalCmdSelector::onSelectRemoteCommandService(
  autoware_control_msgs::RemoteCommandSelect::Request & req,
  autoware_control_msgs::RemoteCommandSelect::Response & res)
{
  current_selector_mode_.data = req.mode.data;
  res.success = true;
  res.message = "Success.";

  return true;
}

void ExternalCmdSelector::onTimer(const ros::TimerEvent & event)
{
  pub_current_selector_mode_.publish(current_selector_mode_);
}
