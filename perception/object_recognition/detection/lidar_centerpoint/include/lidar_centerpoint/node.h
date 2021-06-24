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

#include <autoware_perception_msgs/DynamicObjectWithFeatureArray.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "centerpoint_trt.h"
#include "config.h"
#include "pointcloud_densification.h"

namespace centerpoint
{
class LidarCenterPointNode
{
public:
  LidarCenterPointNode();

private:
  void pointCloudCallback(const sensor_msgs::PointCloud2 & input_pointcloud_msg);

  ros::NodeHandle nh_{};
  ros::NodeHandle pnh_{};
  ros::Subscriber pointcloud_sub_{};
  ros::Publisher objects_pub_{};
  ros::Publisher pointcloud_pub_{};

  float score_threshold_ = 0;
  std::string densification_base_frame_;
  int densification_past_frames_ = 0;
  bool use_vfe_trt_ = false;
  bool use_head_trt_ = false;
  std::string trt_precision_;

  std::string vfe_onnx_path_;
  std::string vfe_engine_path_;
  std::string vfe_pt_path_;
  std::string head_onnx_path_;
  std::string head_engine_path_;
  std::string head_pt_path_;

  std::unique_ptr<PointCloudDensification> densification_ptr_ = nullptr;
  std::unique_ptr<CenterPointTRT> detector_ptr_ = nullptr;
};

}  // namespace centerpoint
