/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#include <algorithm>
#include <deque>
#include <map>
#include <memory>
#include <vector>

#include <boost/optional.hpp>

#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <autoware_api_msgs/CrosswalkStatus.h>
#include <autoware_api_msgs/IntersectionStatus.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_perception_msgs/LookingTrafficLightState.h>
#include <autoware_perception_msgs/TrafficLightStateArray.h>
#include <autoware_perception_msgs/TrafficLightStateStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

class BehaviorVelocityPlannerNode;
struct PlannerData
{
  // tf
  geometry_msgs::PoseStamped current_pose;

  // msgs from callbacks that are used for data-ready
  geometry_msgs::TwistStamped::ConstPtr current_velocity;
  double current_accel;
  static constexpr double velocity_buffer_time_sec = 10.0;
  std::deque<geometry_msgs::TwistStamped> velocity_buffer;
  autoware_perception_msgs::DynamicObjectArray::ConstPtr dynamic_objects;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr no_ground_pointcloud;
  lanelet::LaneletMapPtr lanelet_map;

  // other internal data
  std::map<int, autoware_perception_msgs::TrafficLightStateStamped> traffic_light_id_map;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules;
  lanelet::routing::RoutingGraphPtr routing_graph;
  std::shared_ptr<const lanelet::routing::RoutingGraphContainer> overall_graphs;

  // external data
  std::map<int, autoware_perception_msgs::TrafficLightStateStamped> external_traffic_light_id_map;
  boost::optional<autoware_api_msgs::CrosswalkStatus> external_crosswalk_status_input;
  boost::optional<autoware_api_msgs::IntersectionStatus> external_intersection_status_input;

  // parameters
  double wheel_base;
  double front_overhang;
  double rear_overhang;
  double vehicle_width;
  double base_link2front;
  double vehicle_length;

  // additional parameters
  double max_stop_acceleration_threshold;
  double max_stop_jerk_threshold;
  double delay_response_time;
  double stop_line_extend_length;

  bool isVehicleStopped(const double stop_duration = 0.0) const
  {
    if (velocity_buffer.empty()) return false;

    // Get velocities within stop_duration
    const auto now = ros::Time::now();
    std::vector<double> vs;
    for (const auto & velocity : velocity_buffer) {
      vs.push_back(velocity.twist.linear.x);

      const auto time_diff = now - velocity.header.stamp;
      if (time_diff.toSec() >= stop_duration) {
        break;
      }
    }

    // Check all velocities
    constexpr double stop_velocity = 0.1;
    for (const auto & v : vs) {
      if (v >= stop_velocity) {
        return false;
      }
    }

    return true;
  }

  std::shared_ptr<autoware_perception_msgs::TrafficLightStateStamped> getTrafficLightState(
    const int id) const
  {
    if (traffic_light_id_map.count(id) == 0) {
      return {};
    }
    return std::make_shared<autoware_perception_msgs::TrafficLightStateStamped>(
      traffic_light_id_map.at(id));
  }

  std::shared_ptr<autoware_perception_msgs::TrafficLightStateStamped> getExternalTrafficLightState(
    const int id) const
  {
    if (external_traffic_light_id_map.count(id) == 0) {
      return {};
    }
    return std::make_shared<autoware_perception_msgs::TrafficLightStateStamped>(
      external_traffic_light_id_map.at(id));
  }

private:
  double prev_accel_;
  geometry_msgs::TwistStamped::ConstPtr prev_velocity_;
  double accel_lowpass_gain_;

  void updateCurrentAcc()
  {
    if (prev_velocity_) {
      const double dv = current_velocity->twist.linear.x - prev_velocity_->twist.linear.x;
      const double dt =
        std::max((current_velocity->header.stamp - prev_velocity_->header.stamp).toSec(), 1e-03);
      const double accel = dv / dt;
      // apply lowpass filter
      current_accel = accel_lowpass_gain_ * accel + (1.0 - accel_lowpass_gain_) * prev_accel_;
    } else {
      current_accel = 0.0;
    }

    prev_velocity_ = current_velocity;
    prev_accel_ = current_accel;
  }
  friend BehaviorVelocityPlannerNode;
};
