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

#include <traffic_light_map_visualizer/node.hpp>

#include <visualization_msgs/MarkerArray.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_projection/UTM.h>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/visualization/visualization.h>

#include <vector>

namespace
{
void setColor(
  const double r, const double g, const double b, const double a, std_msgs::ColorRGBA & cl)
{
  cl.r = r;
  cl.g = g;
  cl.b = b;
  cl.a = a;
}

bool isAttributeValue(
  const lanelet::ConstPoint3d p, const std::string attr_str, const std::string value_str)
{
  lanelet::Attribute attr = p.attribute(attr_str);
  if (attr.value().compare(value_str) == 0) return true;
  return false;
}

bool isAttributeValue(
  const lanelet::ConstLineString3d l, const std::string attr_str, const int value)
{
  lanelet::Attribute attr = l.attribute(attr_str);
  if (std::stoi(attr.value()) == value) return true;
  return false;
}

void lightAsMarker(
  lanelet::ConstPoint3d p, visualization_msgs::Marker * marker, const std::string ns,
  const ros::Time & current_time)
{
  if (marker == nullptr) {
    ROS_ERROR_STREAM(__FUNCTION__ << ": marker is null pointer!");
    return;
  }

  marker->header.frame_id = "map";
  marker->header.stamp = current_time;
  marker->frame_locked = true;
  marker->ns = ns;
  marker->id = p.id();
  marker->lifetime = ros::Duration(0.2);
  marker->type = visualization_msgs::Marker::SPHERE;
  marker->pose.position.x = p.x();
  marker->pose.position.y = p.y();
  marker->pose.position.z = p.z();
  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = 0.0;
  marker->pose.orientation.w = 1.0;

  float s = 0.3;

  marker->scale.x = s;
  marker->scale.y = s;
  marker->scale.z = s;

  marker->color.r = 0.0f;
  marker->color.g = 0.0f;
  marker->color.b = 0.0f;
  marker->color.a = 0.999f;

  if (isAttributeValue(p, "color", "red")) {
    marker->color.r = 1.0f;
    marker->color.g = 0.0f;
    marker->color.b = 0.0f;
  } else if (isAttributeValue(p, "color", "green")) {
    marker->color.r = 0.0f;
    marker->color.g = 1.0f;
    marker->color.b = 0.0f;
  } else if (isAttributeValue(p, "color", "yellow")) {
    marker->color.r = 1.0f;
    marker->color.g = 1.0f;
    marker->color.b = 0.0f;
  } else {
    marker->color.r = 1.0f;
    marker->color.g = 1.0f;
    marker->color.b = 1.0f;
  }
}
}  // namespace

namespace traffic_light
{
TrafficLightMapVisualizerNode::TrafficLightMapVisualizerNode() : nh_(), pnh_("~")
{
  light_marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("output/traffic_light", 1);
  tl_state_sub_ = pnh_.subscribe(
    "input/tl_state", 1, &TrafficLightMapVisualizerNode::trafficLightStatesCallback, this);
  vector_map_sub_ =
    pnh_.subscribe("input/vector_map", 1, &TrafficLightMapVisualizerNode::binMapCallback, this);
}
void TrafficLightMapVisualizerNode::trafficLightStatesCallback(
  const autoware_perception_msgs::TrafficLightStateArray::ConstPtr & input_tl_states_msg)
{
  visualization_msgs::MarkerArray output_msg;
  ros::Time current_time = ros::Time::now();

#if 0
  for (auto tli = aw_tl_reg_elems_.begin(); tli != aw_tl_reg_elems_.end(); tli++) {
    for (const auto & lsp : (*tli)->trafficLights()) {
      if (lsp.isLineString())  // traffic ligths can either polygons or
      {                        // linestrings
        lanelet::ConstLineString3d ls = static_cast<lanelet::ConstLineString3d>(lsp);
        for (const auto & input_tl_state : input_tl_states_msg->states) {
          if (ls.id() != input_tl_state.id) {
            continue;
          }
          visualization_msgs::Marker marker;
          std_msgs::ColorRGBA color;
          setColor(1.0f, 1.0f, 1.0f, 0.999f, color);
          lanelet::visualization::trafficLight2TriangleMarker(
            ls, &marker, "traffic_light_triangle", color);
          marker.header.frame_id = "map";
          marker.header.stamp = current_time;
          marker.frame_locked = true;
          marker.lifetime = ros::Duration(0.2);
          output_msg.markers.push_back(marker);
        }
      }
    }
  }
#endif

  for (auto tli = aw_tl_reg_elems_.begin(); tli != aw_tl_reg_elems_.end(); tli++) {
    for (auto ls : (*tli)->lightBulbs()) {
      if (!ls.hasAttribute("traffic_light_id")) continue;
      for (const auto & input_tl_state : input_tl_states_msg->states) {
        if (isAttributeValue(ls, "traffic_light_id", input_tl_state.id)) {
          for (auto pt : ls) {
            if (!pt.hasAttribute("color")) continue;

            for (const auto & lamp_state : input_tl_state.lamp_states) {
              visualization_msgs::Marker marker;
              if (
                isAttributeValue(pt, "color", "red") &&
                lamp_state.type == autoware_perception_msgs::LampState::RED) {
                lightAsMarker(pt, &marker, "traffic_light", current_time);
              } else if (
                isAttributeValue(pt, "color", "green") &&
                lamp_state.type == autoware_perception_msgs::LampState::GREEN) {
                lightAsMarker(pt, &marker, "traffic_light", current_time);

              } else if (
                isAttributeValue(pt, "color", "yellow") &&
                lamp_state.type == autoware_perception_msgs::LampState::YELLOW) {
                lightAsMarker(pt, &marker, "traffic_light", current_time);
              } else {
                continue;
              }
              output_msg.markers.push_back(marker);
            }
          }
        }
      }
    }
  }

  light_marker_pub_.publish(output_msg);
}

void TrafficLightMapVisualizerNode::binMapCallback(
  const autoware_lanelet2_msgs::MapBin::ConstPtr & input_map_msg)
{
  lanelet::LaneletMapPtr viz_lanelet_map(new lanelet::LaneletMap);

  lanelet::utils::conversion::fromBinMsg(*input_map_msg, viz_lanelet_map);
  ROS_INFO("Map is loaded\n");

  // get lanelets etc to visualize
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(viz_lanelet_map);
  aw_tl_reg_elems_ = lanelet::utils::query::autowareTrafficLights(all_lanelets);
}
}  // namespace traffic_light
