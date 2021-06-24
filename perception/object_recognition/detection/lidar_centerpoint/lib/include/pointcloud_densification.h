/*
 * Copyright 2021 Tier IV, Inc.
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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace centerpoint
{
class PointCloudDensification
{
public:
  PointCloudDensification(std::string base_frame_id, unsigned int pointcloud_cache_size);

  sensor_msgs::PointCloud2 stackPointCloud(const sensor_msgs::PointCloud2 & input_pointcloud_msg);

private:
  geometry_msgs::TransformStamped getTransformStamped(
    const std::string & target_frame, const std::string & source_frame);

  static void setTimeLag(sensor_msgs::PointCloud2 & pointcloud_msg, float diff_timestamp);

  std::string base_frame_id_;
  unsigned int pointcloud_cache_size_ = 0;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::list<sensor_msgs::PointCloud2> pointcloud_cache_;
  std::list<Eigen::Matrix4f> matrix_past2base_cache_;
};

}  // namespace centerpoint
