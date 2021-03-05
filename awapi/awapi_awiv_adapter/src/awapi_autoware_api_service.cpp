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

#include <awapi_awiv_adapter/awapi_autoware_api_service.h>

using autoware_system_msgs::AutowareState;

namespace autoware_api
{
AutowareApiService::AutowareApiService()
{
  // Subscriber
  sub_initial_pose_ =
    pnh_.subscribe("input/initial_pose", 1, &AutowareApiService::onInitialPose, this);
  sub_goal_pose_ = pnh_.subscribe("input/goal_pose", 1, &AutowareApiService::onGoalPose, this);
  sub_route_ = pnh_.subscribe("input/route", 1, &AutowareApiService::onRoute, this);
  sub_vehicle_engage_ =
    pnh_.subscribe("input/vehicle_engage", 1, &AutowareApiService::onVehicleEngage, this);
  sub_engage_ = pnh_.subscribe("input/engage", 1, &AutowareApiService::onEngage, this);

  // Publisher
  pub_initial_pose_ =
    pnh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("output/initial_pose", 1, true);
  pub_goal_pose_ = pnh_.advertise<geometry_msgs::PoseStamped>("output/goal_pose", 1, true);
  pub_route_ = pnh_.advertise<autoware_planning_msgs::Route>("output/route", 1, true);
  pub_autoware_engage_ = pnh_.advertise<std_msgs::Bool>("output/autoware_engage", 1, true);
  pub_vehicle_engage_ = pnh_.advertise<std_msgs::Bool>("output/vehicle_engage", 1, true);
  pub_max_velocity_ = pnh_.advertise<std_msgs::Float32>("output/max_velocity", 1, true);

  // Service
  srv_set_initial_pose_ = pnh_.advertiseService(
    "service/set_initial_pose", &AutowareApiService::onSetInitialPoseService, this);
  srv_set_goal_pose_ =
    pnh_.advertiseService("service/set_goal_pose", &AutowareApiService::onSetGoalPoseService, this);
  srv_set_route_ =
    pnh_.advertiseService("service/set_route", &AutowareApiService::onSetRouteService, this);
  srv_reset_route_ =
    pnh_.advertiseService("service/reset_route", &AutowareApiService::onResetRouteService, this);
  srv_set_vehicle_engage_ = pnh_.advertiseService(
    "service/set_vehicle_engage", &AutowareApiService::onSetVehicleEngageService, this);
  srv_set_engage_ =
    pnh_.advertiseService("service/set_engage", &AutowareApiService::onSetEngageService, this);

  // Service Client
  client_reset_route_ = pnh_.serviceClient<std_srvs::Trigger>("client/reset_route");
}

void AutowareApiService::update(const AutowareInfo & aw_info)
{
  if (aw_info.autoware_state_ptr) {
    autoware_state_ = aw_info.autoware_state_ptr->state;
  }

  if (aw_info.current_max_velocity_ptr) {
    // Ignore when stopped
    if (aw_info.current_max_velocity_ptr->data != 0.0) {
      prev_max_velocity_ = aw_info.current_max_velocity_ptr->data;
    }
  }
}

void AutowareApiService::onInitialPose(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
  if (autoware_state_ == AutowareState::Planning || autoware_state_ == AutowareState::Driving) {
    return;
  }

  pub_initial_pose_.publish(msg);
}

void AutowareApiService::onGoalPose(const geometry_msgs::PoseStamped & msg)
{
  // Check condition
  if (autoware_state_ != AutowareState::WaitingForRoute) {
    return;
  }

  pub_goal_pose_.publish(msg);
}

void AutowareApiService::onRoute(const autoware_planning_msgs::Route & msg)
{
  if (autoware_state_ != AutowareState::WaitingForRoute) {
    return;
  }

  pub_route_.publish(msg);
}

void AutowareApiService::onVehicleEngage(const std_msgs::Bool & msg)
{
  pub_vehicle_engage_.publish(msg);
}

void AutowareApiService::onEngage(const std_msgs::Bool & msg)
{
  if (msg.data == true) {
    if (
      autoware_state_ != AutowareState::WaitingForEngage &&
      autoware_state_ != AutowareState::Driving) {
      return;
    }

    std_msgs::Float32 max_velocity;
    max_velocity.data = prev_max_velocity_;
    pub_max_velocity_.publish(max_velocity);

    pub_autoware_engage_.publish(msg);
    pub_vehicle_engage_.publish(msg);
  } else {
    std_msgs::Float32 max_velocity;
    max_velocity.data = 0.0;
    pub_max_velocity_.publish(max_velocity);
  }
}

bool AutowareApiService::onSetInitialPoseService(
  autoware_api_msgs::SetInitialPose::Request & req,
  autoware_api_msgs::SetInitialPose::Response & res)
{
  // Check condition
  if (
    autoware_state_ != AutowareState::InitializingVehicle &&
    autoware_state_ != AutowareState::WaitingForRoute) {
    res.success = false;
    res.message = "failed to set initial_pose.";
    return true;
  }

  // Set data
  pub_initial_pose_.publish(req.initial_pose);

  // Check result
  // TODO(Kenji Miyake): Implement

  // Set result
  res.success = true;
  res.message = "initial_pose was successfully set.";
  return true;
}

bool AutowareApiService::onSetGoalPoseService(
  autoware_api_msgs::SetGoalPose::Request & req, autoware_api_msgs::SetGoalPose::Response & res)
{
  // Check condition
  if (autoware_state_ != AutowareState::WaitingForRoute) {
    res.success = false;
    res.message = "failed to set goal_pose.";
    return true;
  }

  // Set data
  pub_goal_pose_.publish(req.goal_pose);

  // Check result
  // TODO(Kenji Miyake): Implement

  // Set result
  res.success = true;
  res.message = "goal_pose was successfully set.";
  return true;
}

bool AutowareApiService::onSetRouteService(
  autoware_api_msgs::SetRoute::Request & req, autoware_api_msgs::SetRoute::Response & res)
{
  // Check condition
  if (autoware_state_ != AutowareState::WaitingForRoute) {
    res.success = false;
    res.message = "failed to set route.";
    return true;
  }

  // Set data
  pub_route_.publish(req.route);

  // Check result
  // TODO(Kenji Miyake): Implement

  // Set result
  res.success = true;
  res.message = "route was successfully set.";
  return true;
}

bool AutowareApiService::onResetRouteService(
  std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res)
{
  // Check condition
  if (autoware_state_ != AutowareState::WaitingForEngage) {
    res.success = false;
    res.message = "failed to reset route.";
    return true;
  }

  // Call service
  std_srvs::Trigger srv;
  const auto result = client_reset_route_.call(srv);

  if (result) {
    if (srv.response.success) {
      res.success = true;
      res.message = "route was successfully reset.";
    } else {
      res.success = false;
      res.message = "failed to reset route: " + srv.response.message;
    }
  } else {
    res.success = false;
    res.message = "failed to call service";
  }

  return true;
}

bool AutowareApiService::onSetVehicleEngageService(
  autoware_api_msgs::SetVehicleEngage::Request & req,
  autoware_api_msgs::SetVehicleEngage::Response & res)
{
  // Check condition
  // No condition is required because vehicles always can be activated

  // Set data
  std_msgs::Bool b;
  b.data = req.engage;
  pub_vehicle_engage_.publish(b);

  // Check result
  // TODO(Kenji Miyake): Implement

  // Set result
  res.success = true;
  res.message = "vehicle_engage was successfully set.";
  return true;
}

bool AutowareApiService::onSetEngageService(
  autoware_api_msgs::SetEngage::Request & req, autoware_api_msgs::SetEngage::Response & res)
{
  if (req.engage == true) {
    // Check condition
    if (
      autoware_state_ != AutowareState::WaitingForEngage &&
      autoware_state_ != AutowareState::Driving) {
      res.success = false;
      res.message = "failed to set engage.";
      return true;
    }

    // Set data
    std_msgs::Float32 max_velocity;
    max_velocity.data = prev_max_velocity_;
    pub_max_velocity_.publish(max_velocity);

    std_msgs::Bool b;
    b.data = req.engage;
    pub_autoware_engage_.publish(b);
    pub_vehicle_engage_.publish(b);

    // Check result
    // TODO(Kenji Miyake): Implement

    res.success = true;
    res.message = "engage was successfully set.";
    return true;
  } else {
    // Set data
    std_msgs::Float32 max_velocity;
    max_velocity.data = 0.0;
    pub_max_velocity_.publish(max_velocity);

    // Check result
    // TODO(Kenji Miyake): Implement

    res.success = true;
    res.message = "engage was successfully set.";
    return true;
  }
}

}  // namespace autoware_api
