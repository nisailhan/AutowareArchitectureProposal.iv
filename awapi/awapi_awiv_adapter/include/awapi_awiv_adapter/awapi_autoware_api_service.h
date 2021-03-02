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

#include <ros/ros.h>

#include <autoware_api_msgs/SetEngage.h>
#include <autoware_api_msgs/SetGoalPose.h>
#include <autoware_api_msgs/SetInitialPose.h>
#include <autoware_api_msgs/SetRoute.h>
#include <autoware_api_msgs/SetVehicleEngage.h>
#include <autoware_planning_msgs/Route.h>
#include <autoware_system_msgs/AutowareState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>

#include <awapi_awiv_adapter/awapi_autoware_util.h>

namespace autoware_api
{
class AutowareApiService
{
public:
  AutowareApiService();
  void update(const AutowareInfo & aw_info);

private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle pnh_{"~"};

  // Subscriber
  ros::Subscriber sub_initial_pose_;
  ros::Subscriber sub_goal_pose_;
  ros::Subscriber sub_route_;
  ros::Subscriber sub_vehicle_engage_;
  ros::Subscriber sub_engage_;

  void onInitialPose(const geometry_msgs::PoseWithCovarianceStamped & msg);
  void onGoalPose(const geometry_msgs::PoseStamped & msg);
  void onRoute(const autoware_planning_msgs::Route & msg);
  void onVehicleEngage(const std_msgs::Bool & msg);
  void onEngage(const std_msgs::Bool & msg);

  // Publisher
  ros::Publisher pub_initial_pose_;
  ros::Publisher pub_goal_pose_;
  ros::Publisher pub_route_;
  ros::Publisher pub_autoware_engage_;
  ros::Publisher pub_vehicle_engage_;
  ros::Publisher pub_max_velocity_;

  // Service
  ros::ServiceServer srv_set_initial_pose_;
  ros::ServiceServer srv_set_goal_pose_;
  ros::ServiceServer srv_set_route_;
  ros::ServiceServer srv_reset_route_;
  ros::ServiceServer srv_set_vehicle_engage_;
  ros::ServiceServer srv_set_engage_;

  bool onSetInitialPoseService(
    autoware_api_msgs::SetInitialPose::Request & req,
    autoware_api_msgs::SetInitialPose::Response & res);
  bool onSetGoalPoseService(
    autoware_api_msgs::SetGoalPose::Request & req, autoware_api_msgs::SetGoalPose::Response & res);
  bool onSetRouteService(
    autoware_api_msgs::SetRoute::Request & req, autoware_api_msgs::SetRoute::Response & res);
  bool onResetRouteService(std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res);
  bool onSetVehicleEngageService(
    autoware_api_msgs::SetVehicleEngage::Request & req,
    autoware_api_msgs::SetVehicleEngage::Response & res);
  bool onSetEngageService(
    autoware_api_msgs::SetEngage::Request & req, autoware_api_msgs::SetEngage::Response & res);

  // Service Client
  ros::ServiceClient client_reset_route_;

  // Parameter
  // Nothing

  // Data
  autoware_system_msgs::AutowareState::_state_type autoware_state_ =
    autoware_system_msgs::AutowareState::InitializingVehicle;
  float prev_max_velocity_ = 0.0;
};

}  // namespace autoware_api
