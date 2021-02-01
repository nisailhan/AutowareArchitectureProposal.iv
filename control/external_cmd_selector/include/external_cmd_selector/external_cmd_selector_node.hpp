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

#include <ros/ros.h>

#include <autoware_control_msgs/RemoteCommandSelect.h>
#include <autoware_control_msgs/RemoteCommandSelectorMode.h>
#include <autoware_vehicle_msgs/RawControlCommandStamped.h>
#include <autoware_vehicle_msgs/ShiftStamped.h>
#include <autoware_vehicle_msgs/TurnSignal.h>

class ExternalCmdSelector
{
public:
  ExternalCmdSelector();

private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle pnh_{"~"};

  // Publisher
  ros::Publisher pub_current_selector_mode_;

  ros::Publisher pub_raw_control_cmd_;
  ros::Publisher pub_shift_cmd_;
  ros::Publisher pub_turn_signal_cmd_;

  // Subscriber
  ros::Subscriber sub_local_raw_control_cmd_;
  ros::Subscriber sub_local_shift_cmd_;
  ros::Subscriber sub_local_turn_signal_cmd_;

  ros::Subscriber sub_remote_raw_control_cmd_;
  ros::Subscriber sub_remote_shift_cmd_;
  ros::Subscriber sub_remote_turn_signal_cmd_;

  void onSelectorModeCmd(const autoware_control_msgs::RemoteCommandSelectorMode::ConstPtr msg);

  void onLocalRawControlCmd(const autoware_vehicle_msgs::RawControlCommandStamped::ConstPtr msg);
  void onLocalShiftCmd(const autoware_vehicle_msgs::ShiftStamped::ConstPtr msg);
  void onLocalTurnSignalCmd(const autoware_vehicle_msgs::TurnSignal::ConstPtr msg);

  void onRemoteRawControlCmd(const autoware_vehicle_msgs::RawControlCommandStamped::ConstPtr msg);
  void onRemoteShiftCmd(const autoware_vehicle_msgs::ShiftStamped::ConstPtr msg);
  void onRemoteTurnSignalCmd(const autoware_vehicle_msgs::TurnSignal::ConstPtr msg);

  // Service
  ros::ServiceServer srv_select_external_command_;
  autoware_control_msgs::RemoteCommandSelectorMode current_selector_mode_;

  bool onSelectRemoteCommandService(
    autoware_control_msgs::RemoteCommandSelect::Request & req,
    autoware_control_msgs::RemoteCommandSelect::Response & res);

  // Timer
  ros::Timer timer_;

  void onTimer(const ros::TimerEvent & event);

  // Parameter
  double update_rate_;
  int initial_selector_mode_;
};
