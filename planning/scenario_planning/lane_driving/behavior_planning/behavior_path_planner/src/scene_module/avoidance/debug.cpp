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

#include "behavior_path_planner/scene_module/avoidance/debug.hpp"
#include "behavior_path_planner/utilities.hpp"
#include "behavior_path_planner/path_utilities.hpp"
#include "tf2/utils.h"

using namespace behavior_path_planner;

namespace marker_utils
{
inline int64_t bitShift(int64_t original_id) { return (original_id << (sizeof(int32_t) * 8 / 2)); }

visualization_msgs::MarkerArray createShiftPointMarkerArray(
  const std::vector<behavior_path_planner::ShiftPoint> & shift_points, const double base_shift,
  const std::string & ns, const double r, const double g, const double b)
{
  int32_t id = 0;
  const auto current_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;

  double current_shift = base_shift;
  // TODO(Horibe) now assuming the shift point is aligned in longitudinal distance order
  for (const auto & sp : shift_points) {
    // ROS_ERROR("sp: s = (%f, %f), g = (%f, %f)", sp.start.x, sp.start.y, sp.end.x, sp.end.y);
    visualization_msgs::Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = ns;
    marker.lifetime = ros::Duration(1.1);
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = autoware_utils::createMarkerScale(0.5, 0.5, 0.5);
    marker.color = autoware_utils::createMarkerColor(r, g, b, 0.7);
    {
      marker.type = visualization_msgs::Marker::CUBE;

      // start point
      auto marker_s = marker;
      marker_s.id = id++;
      marker_s.pose = sp.start;
      util::shiftPose(&marker_s.pose, current_shift);
      msg.markers.push_back(marker_s);

      // end point
      auto marker_e = marker;
      marker_e.id = id++;
      marker_e.pose = sp.end;
      util::shiftPose(&marker_e.pose, sp.length);
      msg.markers.push_back(marker_e);

      // start-to-end line
      auto marker_l = marker;
      marker_l.id = id++;
      marker_l.type = visualization_msgs::Marker::LINE_STRIP;
      marker_l.scale = autoware_utils::createMarkerScale(0.1, 0.0, 0.0);
      marker_l.points.push_back(marker_s.pose.position);
      marker_l.points.push_back(marker_e.pose.position);
      msg.markers.push_back(marker_l);
    }
    current_shift = sp.length;
  }

  return msg;
}

visualization_msgs::MarkerArray createFrenetPointMarkerArray(
  const std::vector<behavior_path_planner::Frenet> & frenet_points,
  const autoware_planning_msgs::PathWithLaneId & path, const geometry_msgs::Point & ego_point,
  const std::string & ns, const double r, const double g, const double b)
{
  const auto length_back_to_ego = -autoware_utils::calcSignedArcLength(path.points, ego_point, 0);
  const auto arclength_arr = util::calcPathArcLengthArray(path);

  auto sorted_points = frenet_points;
  std::sort(sorted_points.begin(), sorted_points.end(), [](auto a, auto b) {
    return a.longitudinal < b.longitudinal;
  });

  int32_t id = 0;
  const auto current_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;
  const auto addMarker = [&](const geometry_msgs::Pose & pose, const double shift) {
    auto shifted_pose = pose;
    util::shiftPose(&shifted_pose, shift);

    visualization_msgs::Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = ns;
    marker.id = id++;
    marker.lifetime = ros::Duration(0.3);
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = autoware_utils::createMarkerScale(0.5, 0.5, 0.5);
    marker.color = autoware_utils::createMarkerColor(r, g, b, 0.7);
    marker.pose = shifted_pose;
    msg.markers.push_back(marker);
  };

  size_t frenet_points_idx = 0;
  for (size_t i = 0; i < arclength_arr.size(); ++i) {
    const double from_ego = arclength_arr.at(i) - length_back_to_ego;
    while (frenet_points_idx < sorted_points.size() &&
           sorted_points.at(frenet_points_idx).longitudinal < from_ego) {
      addMarker(path.points.at(i).point.pose, sorted_points.at(frenet_points_idx).lateral);
      ++frenet_points_idx;
    }
    if (frenet_points_idx == sorted_points.size()) break;
  }

  return msg;
}

visualization_msgs::MarkerArray createLaneletsAreaMarkerArray(
  const std::vector<lanelet::ConstLanelet> & lanelets, const std::string & ns, const double r,
  const double g, const double b)
{
  const auto current_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;

  for (const auto & lanelet : lanelets) {
    visualization_msgs::Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;

    marker.ns = ns;
    marker.id = lanelet.id();
    marker.lifetime = ros::Duration(0.3);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = autoware_utils::createMarkerScale(0.1, 0.0, 0.0);
    marker.color = autoware_utils::createMarkerColor(r, g, b, 0.999);
    for (const auto & p : lanelet.polygon3d()) {
      geometry_msgs::Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      marker.points.push_back(point);
    }
    if (!marker.points.empty()) marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::MarkerArray createLaneletPolygonsMarkerArray(
  const std::vector<lanelet::CompoundPolygon3d> & polygons, const std::string & ns,
  const int64_t lane_id)
{
  const auto current_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;

  int32_t i = 0;
  int32_t uid = bitShift(lane_id);
  for (const auto & polygon : polygons) {
    visualization_msgs::Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;

    marker.ns = ns;
    marker.id = uid + i++;
    marker.lifetime = ros::Duration(0.3);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = autoware_utils::createMarkerScale(0.1, 0.0, 0.0);
    marker.color = autoware_utils::createMarkerColor(0.0, 1.0, 0.0, 0.999);
    for (const auto & p : polygon) {
      geometry_msgs::Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      marker.points.push_back(point);
    }
    if (!marker.points.empty()) marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::MarkerArray createPolygonMarkerArray(
  const geometry_msgs::Polygon & polygon, const std::string & ns, const int64_t lane_id,
  const double r, const double g, const double b)
{
  const auto current_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;

  visualization_msgs::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;

  marker.ns = ns;
  marker.id = lane_id;
  marker.lifetime = ros::Duration(0.3);
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  marker.scale = autoware_utils::createMarkerScale(0.3, 0.0, 0.0);
  marker.color = autoware_utils::createMarkerColor(r, g, b, 0.8);
  for (const auto & p : polygon.points) {
    geometry_msgs::Point point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    marker.points.push_back(point);
  }
  if (!marker.points.empty()) marker.points.push_back(marker.points.front());
  msg.markers.push_back(marker);

  return msg;
}

visualization_msgs::MarkerArray createObjectsMarkerArray(
  const autoware_perception_msgs::DynamicObjectArray & objects, const std::string & ns,
  const int64_t lane_id, const double r, const double g, const double b)
{
  const auto current_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;

  visualization_msgs::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  int32_t uid = bitShift(lane_id);
  int32_t i = 0;
  for (const auto & object : objects.objects) {
    marker.id = uid + i++;
    marker.lifetime = ros::Duration(1.0);
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = object.state.pose_covariance.pose;
    marker.scale = autoware_utils::createMarkerScale(3.0, 1.0, 1.0);
    marker.color = autoware_utils::createMarkerColor(r, g, b, 0.8);
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::MarkerArray createPathMarkerArray(
  const autoware_planning_msgs::PathWithLaneId & path, const std::string & ns,
  const int64_t lane_id, const double r, const double g, const double b)
{
  const auto current_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;
  int32_t uid = bitShift(lane_id);
  int32_t i = 0;
  for (const auto & p : path.points) {
    visualization_msgs::Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = ns;
    marker.id = uid + i++;
    marker.lifetime = ros::Duration(0.3);
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = p.point.pose;
    marker.scale = autoware_utils::createMarkerScale(0.6, 0.3, 0.3);
    if (std::find(p.lane_ids.begin(), p.lane_ids.end(), lane_id) != p.lane_ids.end()) {
      // if p.lane_ids has lane_id
      marker.color = autoware_utils::createMarkerColor(r, g, b, 0.999);
    } else {
      marker.color = autoware_utils::createMarkerColor(0.5, 0.5, 0.5, 0.999);
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::MarkerArray createVirtualWallMarkerArray(
  const geometry_msgs::Pose & pose, const int64_t lane_id, const std::string & stop_factor)
{
  visualization_msgs::MarkerArray msg;

  visualization_msgs::Marker marker_virtual_wall{};
  marker_virtual_wall.header.frame_id = "map";
  marker_virtual_wall.header.stamp = ros::Time::now();
  marker_virtual_wall.ns = "stop_virtual_wall";
  marker_virtual_wall.id = lane_id;
  marker_virtual_wall.lifetime = ros::Duration(0.5);
  marker_virtual_wall.type = visualization_msgs::Marker::CUBE;
  marker_virtual_wall.action = visualization_msgs::Marker::ADD;
  marker_virtual_wall.pose = pose;
  marker_virtual_wall.pose.position.z += 1.0;
  marker_virtual_wall.scale = autoware_utils::createMarkerScale(0.1, 5.0, 2.0);
  marker_virtual_wall.color = autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.5);
  msg.markers.push_back(marker_virtual_wall);

  visualization_msgs::Marker marker_factor_text{};
  marker_factor_text.header.frame_id = "map";
  marker_factor_text.header.stamp = ros::Time::now();
  marker_factor_text.ns = "factor_text";
  marker_factor_text.id = lane_id;
  marker_factor_text.lifetime = ros::Duration(0.5);
  marker_factor_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_factor_text.action = visualization_msgs::Marker::ADD;
  marker_factor_text.pose = pose;
  marker_factor_text.pose.position.z += 2.0;
  marker_factor_text.scale = autoware_utils::createMarkerScale(0.0, 0.0, 1.0);
  marker_factor_text.color = autoware_utils::createMarkerColor(1.0, 1.0, 1.0, 0.999);
  marker_factor_text.text = stop_factor;
  msg.markers.push_back(marker_factor_text);

  return msg;
}

visualization_msgs::MarkerArray createPoseMarkerArray(
  const geometry_msgs::Pose & pose, const std::string & ns, const int64_t id, const double r,
  const double g, const double b)
{
  const auto current_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;

  visualization_msgs::Marker marker_line{};
  marker_line.header.frame_id = "map";
  marker_line.header.stamp = current_time;
  marker_line.ns = ns + "_line";
  marker_line.id = id;
  marker_line.lifetime = ros::Duration(0.3);
  marker_line.type = visualization_msgs::Marker::LINE_STRIP;
  marker_line.action = visualization_msgs::Marker::ADD;
  marker_line.pose.orientation = autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  marker_line.scale = autoware_utils::createMarkerScale(0.1, 0.0, 0.0);
  marker_line.color = autoware_utils::createMarkerColor(r, g, b, 0.999);

  const double yaw = tf2::getYaw(pose.orientation);

  const double a = 3.0;
  geometry_msgs::Point p0;
  p0.x = pose.position.x - a * std::sin(yaw);
  p0.y = pose.position.y + a * std::cos(yaw);
  p0.z = pose.position.z;
  marker_line.points.push_back(p0);

  geometry_msgs::Point p1;
  p1.x = pose.position.x + a * std::sin(yaw);
  p1.y = pose.position.y - a * std::cos(yaw);
  p1.z = pose.position.z;
  marker_line.points.push_back(p1);

  msg.markers.push_back(marker_line);

  return msg;
}

}  // namespace marker_utils
