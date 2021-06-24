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

#include "lidar_centerpoint/node.h"

#include <pcl_ros/transforms.h>
#include <memory>

#include "config.h"

namespace centerpoint
{
LidarCenterPointNode::LidarCenterPointNode() : nh_(""), pnh_("~")
{
  pnh_.param<float>("score_threshold", score_threshold_, 0.4);
  pnh_.param<std::string>("densification_base_frame", densification_base_frame_, "map");
  pnh_.param<int>("densification_past_frames", densification_past_frames_, 1);
  pnh_.param<bool>("use_vfe_trt", use_vfe_trt_, false);
  pnh_.param<bool>("use_head_trt", use_head_trt_, true);
  pnh_.param<std::string>("trt_precision", trt_precision_, "fp16");
  pnh_.param<std::string>("vfe_onnx_path", vfe_onnx_path_, "");
  pnh_.param<std::string>("vfe_engine_path", vfe_engine_path_, "");
  pnh_.param<std::string>("vfe_pt_path", vfe_pt_path_, "");
  pnh_.param<std::string>("head_onnx_path", head_onnx_path_, "");
  pnh_.param<std::string>("head_engine_path", head_engine_path_, "");
  pnh_.param<std::string>("head_pt_path", head_pt_path_, "");

  NetworkParam vfe_param(
    vfe_onnx_path_, vfe_engine_path_, vfe_pt_path_, trt_precision_, use_vfe_trt_);
  NetworkParam head_param(
    head_onnx_path_, head_engine_path_, head_pt_path_, trt_precision_, use_head_trt_);
  densification_ptr_ = std::make_unique<PointCloudDensification>(
    densification_base_frame_, densification_past_frames_);
  detector_ptr_ = std::make_unique<CenterPointTRT>(vfe_param, head_param, /*verbose=*/false);

  pointcloud_sub_ =
    pnh_.subscribe("input/pointcloud", 1, &LidarCenterPointNode::pointCloudCallback, this);
  objects_pub_ =
    pnh_.advertise<autoware_perception_msgs::DynamicObjectWithFeatureArray>("output/objects", 1);
  pointcloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("debug/pointcloud_densification", 1);
}

void LidarCenterPointNode::pointCloudCallback(const sensor_msgs::PointCloud2 & input_pointcloud_msg)
{
  if (objects_pub_.getNumSubscribers() < 1 && pointcloud_pub_.getNumSubscribers() < 1) {
    return;
  }

  auto stacked_pointcloud_msg = densification_ptr_->stackPointCloud(input_pointcloud_msg);
  std::vector<float> boxes3d_vec = detector_ptr_->detect(stacked_pointcloud_msg);

  autoware_perception_msgs::DynamicObjectWithFeatureArray output_msg;
  output_msg.header = input_pointcloud_msg.header;
  for (size_t obj_i = 0; obj_i < boxes3d_vec.size() / Config::num_box_features; obj_i++) {
    float score = boxes3d_vec[obj_i * Config::num_box_features + 0];
    if (score < score_threshold_) {
      continue;
    }

    int class_id = (int)boxes3d_vec[obj_i * Config::num_box_features + 1];
    float x = boxes3d_vec[obj_i * Config::num_box_features + 2];
    float y = boxes3d_vec[obj_i * Config::num_box_features + 3];
    float z = boxes3d_vec[obj_i * Config::num_box_features + 4];
    float l = boxes3d_vec[obj_i * Config::num_box_features + 5];
    float w = boxes3d_vec[obj_i * Config::num_box_features + 6];
    float h = boxes3d_vec[obj_i * Config::num_box_features + 7];
    float yaw = boxes3d_vec[obj_i * Config::num_box_features + 8];

    autoware_perception_msgs::DynamicObjectWithFeature feature_obj;
    autoware_perception_msgs::Semantic semantic;
    switch (class_id) {
      case 0:
        semantic.type = autoware_perception_msgs::Semantic::CAR;
        break;
      case 1:
        semantic.type = autoware_perception_msgs::Semantic::PEDESTRIAN;
        break;
      case 2:
        semantic.type = autoware_perception_msgs::Semantic::BICYCLE;
        break;
      default:
        semantic.type = autoware_perception_msgs::Semantic::UNKNOWN;
    }
    semantic.confidence = score;
    feature_obj.object.semantic = semantic;

    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = tf::createQuaternionMsgFromYaw((double)yaw);
    feature_obj.object.state.pose_covariance.pose = pose;

    autoware_perception_msgs::Shape shape;
    shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
    shape.dimensions.x = l;
    shape.dimensions.y = w;
    shape.dimensions.z = h;
    feature_obj.object.shape = shape;

    // Note: object size is referred from multi_object_tracker
    if ((w * l > 2.2 * 5.5) && (w * l <= 2.5 * 7.9)) {
      feature_obj.object.semantic.type = semantic.type = autoware_perception_msgs::Semantic::TRUCK;
    } else if (w * l > 2.5 * 7.9) {
      feature_obj.object.semantic.type = semantic.type = autoware_perception_msgs::Semantic::BUS;
    }

    output_msg.feature_objects.emplace_back(feature_obj);
  }

  if (objects_pub_.getNumSubscribers() > 0) {
    objects_pub_.publish(output_msg);
  }
  if (pointcloud_pub_.getNumSubscribers() > 0) {
    pointcloud_pub_.publish(stacked_pointcloud_msg);
  }
}

}  // namespace centerpoint
