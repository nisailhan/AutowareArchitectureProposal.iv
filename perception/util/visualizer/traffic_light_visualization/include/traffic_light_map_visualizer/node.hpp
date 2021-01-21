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


#include <visualization_msgs/MarkerArray.h>

#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_perception_msgs/TrafficLightStateArray.h>

#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>
#include <lanelet2_extension/utility/query.h>

#include <string>
#include <memory>

namespace traffic_light
{
class TrafficLightMapVisualizerNode
{
public:
  TrafficLightMapVisualizerNode();
  ~TrafficLightMapVisualizerNode() = default;
  void trafficLightStatesCallback(
    const autoware_perception_msgs::TrafficLightStateArray::ConstPtr & input_tl_states_msg);
  void binMapCallback(const autoware_lanelet2_msgs::MapBin::ConstPtr & input_map_msg);

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher light_marker_pub_;
  ros::Subscriber tl_state_sub_, vector_map_sub_;

  std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems_;
};

}  // namespace traffic_light
