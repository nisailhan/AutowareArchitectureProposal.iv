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
 */

#include "pointcloud_preprocessor/ground_filter/scan_ground_filter_nodelet.h"

#include <autoware_utils/geometry/geometry.h>
#include <autoware_utils/math/normalization.h>
#include <autoware_utils/math/unit_conversion.h>
#include <autoware_utils/ros/vehicle_info.h>
#include <pluginlib/class_list_macros.h>

#include <pcl_ros/transforms.h>
namespace pointcloud_preprocessor
{
ScanGroundFilterNodelet::ScanGroundFilterNodelet() : tf_listener_(tf_buffer_) {}

bool ScanGroundFilterNodelet::transformPointCloud(
  const std::string & in_target_frame, const sensor_msgs::PointCloud2::ConstPtr & in_cloud_ptr,
  const sensor_msgs::PointCloud2::Ptr & out_cloud_ptr)
{
  if (in_target_frame == in_cloud_ptr->header.frame_id) {
    *out_cloud_ptr = *in_cloud_ptr;
    return true;
  }

  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      in_target_frame, in_cloud_ptr->header.frame_id, in_cloud_ptr->header.stamp,
      ros::Duration(1.0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
  Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(mat, *in_cloud_ptr, *out_cloud_ptr);
  out_cloud_ptr->header.frame_id = in_target_frame;
  return true;
}

void ScanGroundFilterNodelet::convertPointcloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
  std::vector<PointCloudRefVector> & out_radial_ordered_points)
{
  out_radial_ordered_points.resize(radial_dividers_num_);
  PointRef current_point;

  for (size_t i = 0; i < in_cloud->points.size(); ++i) {
    auto radius = static_cast<float>(std::hypot(in_cloud->points[i].x, in_cloud->points[i].y));
    auto theta = static_cast<float>(
      autoware_utils::rad2deg(atan2(in_cloud->points[i].y, in_cloud->points[i].x)));
    if (theta < 0) {
      theta += 360;
    }
    if (theta >= 360) {
      theta -= 360;
    }
    auto radial_div = (size_t)floor(theta / radial_divider_angle_);

    current_point.radius = radius;
    current_point.theta = theta;
    current_point.radial_div = radial_div;
    current_point.point_state = PointLabel::INIT;
    current_point.orig_index = i;
    current_point.orig_point = &in_cloud->points[i];

    // radial divisions
    out_radial_ordered_points[radial_div].emplace_back(current_point);
  }

  // sort by distance
  for (size_t i = 0; i < radial_dividers_num_; ++i) {
    std::sort(
      out_radial_ordered_points[i].begin(), out_radial_ordered_points[i].end(),
      [](const PointRef & a, const PointRef & b) { return a.radius < b.radius; });
  }
}

void ScanGroundFilterNodelet::calcVirtualGroundOrigin(pcl::PointXYZ & point)
{
  VehicleInfo vehicle_info = waitForVehicleInfo();

  point.x = vehicle_info.wheel_base;
  point.y = 0;
  point.z = 0;
  return;
}

void ScanGroundFilterNodelet::classifyPointCloud(
  std::vector<PointCloudRefVector> & in_radial_ordered_clouds,
  pcl::PointIndices & out_no_ground_indices)
{
  out_no_ground_indices.indices.clear();

  const pcl::PointXYZ init_ground_point(0, 0, 0);
  pcl::PointXYZ virtual_ground_point(0, 0, 0);
  calcVirtualGroundOrigin(virtual_ground_point);

  // point classification algorithm
  // sweep through each radial division
  for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) {
    float prev_gnd_radius = 0.0f;
    float prev_point_radius = 0.0f;
    float prev_gnd_slope = 0.0f;
    float prev_gnd_radius_sum = 0.0f;
    float prev_gnd_height_sum = 0.0f;
    float prev_gnd_radius_avg = 0.0f;
    float prev_gnd_height_avg = 0.0f;
    float points_distance = 0.0f;
    uint32_t prev_gnd_point_num = 0;
    float local_slope = 0.0f;
    PointLabel prev_point_label = PointLabel::INIT;
    pcl::PointXYZ prev_gnd_point(0, 0, 0);
    bool is_prev_point_ground = false;
    bool is_current_point_ground = false;
    // loop through each point in the radial div
    for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) {
      const double global_slope_max_angle = global_slope_max_;
      const double local_slope_max_angle = local_slope_max_angle_;
      const double local_slope_max_dist = local_slope_max_dist_;
      auto * p = &in_radial_ordered_clouds[i][j];
      auto * p_prev = &in_radial_ordered_clouds[i][j - 1];

      if (j == 0) {
        bool is_front_side = (p->orig_point->x > virtual_ground_point.x);
        if (use_virtual_ground_point_ && is_front_side) {
          prev_gnd_point = virtual_ground_point;
        } else {
          prev_gnd_point = init_ground_point;
        }
        prev_gnd_radius = std::hypot(prev_gnd_point.x, prev_gnd_point.y);
        prev_gnd_slope = 0.0f;
        prev_gnd_radius_sum = 0.0f;
        prev_gnd_height_sum = 0.0f;
        prev_gnd_radius_avg = 0.0f;
        prev_gnd_height_avg = 0.0f;
        prev_gnd_point_num = 0;
        points_distance = autoware_utils::calcDistance3d(*p->orig_point, prev_gnd_point);
      } else {
        prev_point_radius = p_prev->radius;
        points_distance = autoware_utils::calcDistance3d(*p->orig_point, *p_prev->orig_point);
      }

      float points_2d_distance = p->radius - prev_gnd_radius;
      float height_distance = p->orig_point->z - prev_gnd_point.z;

      // check points which is far enough from previous point
      if (
        // close to the previous point, set point follow label
        points_distance < (p->radius * autoware_utils::deg2rad(radial_divider_angle_) +
                           split_points_distance_tolerance_) &&
        std::fabs(height_distance) < split_height_distance_) {
        p->point_state = PointLabel::POINT_FOLLOW;
      } else {
        // far from the previous point
        float current_height = p->orig_point->z;

        float global_slope = std::atan2(p->orig_point->z, p->radius);
        local_slope = std::atan2(height_distance, points_2d_distance);

        if (global_slope > global_slope_max_angle) {
          // the point is outside of the global slope threshold
          p->point_state = PointLabel::NON_GROUND;
        } else if (local_slope - prev_gnd_slope > local_slope_max_angle) {
          // the point is outside of the local slope threshold
          p->point_state = PointLabel::NON_GROUND;
        } else {
          p->point_state = PointLabel::GROUND;
        }
      }

      if (p->point_state == PointLabel::GROUND) {
        prev_gnd_radius_sum = 0.0f;
        prev_gnd_height_sum = 0.0f;
        prev_gnd_radius_avg = 0.0f;
        prev_gnd_height_avg = 0.0f;
        prev_gnd_point_num = 0;
      }
      if (p->point_state == PointLabel::NON_GROUND) {
        out_no_ground_indices.indices.push_back(p->orig_index);
      } else if (
        (prev_point_label == PointLabel::NON_GROUND) &&
        (p->point_state == PointLabel::POINT_FOLLOW)) {
        p->point_state = PointLabel::NON_GROUND;
        out_no_ground_indices.indices.push_back(p->orig_index);
      } else if (
        (prev_point_label == PointLabel::GROUND) && (p->point_state == PointLabel::POINT_FOLLOW)) {
        p->point_state = PointLabel::GROUND;
      } else {
      }

      // update the ground state
      prev_point_label = p->point_state;
      if (p->point_state == PointLabel::GROUND) {
        prev_gnd_radius = p->radius;
        prev_gnd_point = pcl::PointXYZ(p->orig_point->x, p->orig_point->y, p->orig_point->z);
        prev_gnd_radius_sum += p->radius;
        prev_gnd_height_sum += p->orig_point->z;
        ++prev_gnd_point_num;
        prev_gnd_radius_avg = prev_gnd_radius_sum / prev_gnd_point_num;
        prev_gnd_height_avg = prev_gnd_height_sum / prev_gnd_point_num;
        prev_gnd_slope = std::atan2(prev_gnd_height_avg, prev_gnd_radius_avg);
      }
    }
  }
}

bool ScanGroundFilterNodelet::child_init(ros::NodeHandle & nh, bool & has_service)
{
  // Enable the dynamic reconfigure service
  has_service = true;
  srv_ = boost::make_shared<
    dynamic_reconfigure::Server<pointcloud_preprocessor::ScanGroundFilterConfig> >(nh);
  dynamic_reconfigure::Server<pointcloud_preprocessor::ScanGroundFilterConfig>::CallbackType f =
    boost::bind(&ScanGroundFilterNodelet::config_callback, this, _1, _2);
  srv_->setCallback(f);
  return (true);
}

void ScanGroundFilterNodelet::extractObjectPoints(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, const pcl::PointIndices & in_indices,
  pcl::PointCloud<pcl::PointXYZ>::Ptr out_object_cloud_ptr)
{
  for (const auto & i : in_indices.indices) {
    out_object_cloud_ptr->points.emplace_back(in_cloud_ptr->points[i]);
  }
}

void ScanGroundFilterNodelet::filter(
  const PointCloud2::ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);

  sensor_msgs::PointCloud2::Ptr input_transformed_ptr(new sensor_msgs::PointCloud2);
  bool succeeded = transformPointCloud(base_frame_, input, input_transformed_ptr);
  sensor_frame_ = input->header.frame_id;
  if (!succeeded) {
    ROS_ERROR_STREAM_THROTTLE(
      10, "Failed transform from " << base_frame_ << " to " << input->header.frame_id);
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_transformed_ptr, *current_sensor_cloud_ptr);

  std::vector<PointCloudRefVector> radial_ordered_points;

  radial_dividers_num_ = ceil(360 / radial_divider_angle_);
  convertPointcloud(current_sensor_cloud_ptr, radial_ordered_points);

  pcl::PointIndices no_ground_indices;
  pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  no_ground_cloud_ptr->points.reserve(current_sensor_cloud_ptr->points.size());

  classifyPointCloud(radial_ordered_points, no_ground_indices);

  extractObjectPoints(current_sensor_cloud_ptr, no_ground_indices, no_ground_cloud_ptr);

  sensor_msgs::PointCloud2::Ptr no_ground_cloud_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*no_ground_cloud_ptr, *no_ground_cloud_msg_ptr);

  no_ground_cloud_msg_ptr->header.stamp = input->header.stamp;
  no_ground_cloud_msg_ptr->header.frame_id = base_frame_;
  output = *no_ground_cloud_msg_ptr;
}

void ScanGroundFilterNodelet::subscribe() { Filter::subscribe(); }

void ScanGroundFilterNodelet::unsubscribe() { Filter::unsubscribe(); }

void ScanGroundFilterNodelet::config_callback(
  pointcloud_preprocessor::ScanGroundFilterConfig & config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (base_frame_ != config.base_frame) {
    base_frame_ = config.base_frame;
    NODELET_DEBUG(
      "[%s::config_callback] Setting base_frame to: %s.", getName().c_str(),
      config.base_frame.c_str());
  }
  if (global_slope_max_ != config.global_slope_max) {
    global_slope_max_ = autoware_utils::deg2rad(config.global_slope_max);
    NODELET_DEBUG(
      "[%s::config_callback] Setting global_slope_max to: %f.", getName().c_str(),
      config.global_slope_max);
  }
  if (local_slope_max_angle_ != config.local_slope_max_angle) {
    local_slope_max_angle_ = autoware_utils::deg2rad(config.local_slope_max_angle);
    NODELET_INFO(
      "[%s::config_callback] Setting local_slope_max_angle to: %f.", getName().c_str(),
      config.local_slope_max_angle);
  }
  if (radial_divider_angle_ != config.radial_divider_angle) {
    radial_divider_angle_ = config.radial_divider_angle;
    NODELET_DEBUG(
      "[%s::config_callback] Setting radial_divider_angle to: %f.", getName().c_str(),
      config.radial_divider_angle);
  }
  if (split_points_distance_tolerance_ != config.split_points_distance_tolerance) {
    split_points_distance_tolerance_ = config.split_points_distance_tolerance;
    NODELET_DEBUG(
      "[%s::config_callback] Setting split_points_distance_tolerance to: %f.", getName().c_str(),
      config.split_points_distance_tolerance);
  }
  if (split_height_distance_ != config.split_height_distance) {
    split_height_distance_ = config.split_height_distance;
    NODELET_DEBUG(
      "[%s::config_callback] Setting split_height_distance_ to: %f.", getName().c_str(),
      config.split_height_distance);
  }
  if (use_virtual_ground_point_ != config.use_virtual_ground_point) {
    use_virtual_ground_point_ = config.use_virtual_ground_point;
    NODELET_DEBUG(
      "[%s::config_callback] Setting use_virtual_ground_point to: %d.", getName().c_str(),
      config.use_virtual_ground_point);
  }
}

}  // namespace pointcloud_preprocessor

PLUGINLIB_EXPORT_CLASS(pointcloud_preprocessor::ScanGroundFilterNodelet, nodelet::Nodelet);
