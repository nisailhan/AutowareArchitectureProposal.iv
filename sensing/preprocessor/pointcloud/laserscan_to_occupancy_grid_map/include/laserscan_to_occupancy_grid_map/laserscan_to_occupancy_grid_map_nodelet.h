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

#pragma once

#include <memory>

#include <laser_geometry/laser_geometry.h>
#include <laserscan_to_occupancy_grid_map/occupancy_grid_map.h>
#include <laserscan_to_occupancy_grid_map/updater/occupancy_grid_map_binary_bayes_filter_updater.h>
#include <laserscan_to_occupancy_grid_map/updater/occupancy_grid_map_updater_interface.h>
#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <memory>

namespace occupancy_grid_map
{
class OccupancyGridMapNodelet : public nodelet::Nodelet
{
public:
  OccupancyGridMapNodelet();

private:
  virtual void onInit();
  sensor_msgs::PointCloud2::Ptr convertLaserscanToPointCLoud2(
    const sensor_msgs::LaserScan::ConstPtr & input);
  void onLaserscanPointCloud2WithObstacleAndRaw(
    const sensor_msgs::LaserScan::ConstPtr & input_laserscan_msg,
    const sensor_msgs::PointCloud2::ConstPtr & input_obstacle_msg,
    const sensor_msgs::PointCloud2::ConstPtr & input_raw_msg);
  boost::shared_ptr<nav_msgs::OccupancyGrid> OccupancyGridMapToMsgPtr(
    const std::string & frame_id, const ros::Time & time, const float & robot_pose_z,
    const costmap_2d::Costmap2D & occupancy_grid_map);
  inline void onDummyPointCloud2(const sensor_msgs::LaserScan::ConstPtr & input)
  {
    sensor_msgs::PointCloud2 dummy;
    sensor_msgs::PointCloud2Modifier modifier(dummy);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    dummy.header = input->header;
    passthrough_.add(boost::make_shared<sensor_msgs::PointCloud2>(dummy));
  }

private:
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher occupancy_grid_map_pub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laserscan_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> obstacle_pointcloud_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> raw_pointcloud_sub_;
  message_filters::PassThrough<sensor_msgs::PointCloud2> passthrough_;

  std::shared_ptr<tf2_ros::Buffer> tf2_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::LaserScan, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_ptr_;

  laser_geometry::LaserProjection laserscan2pointcloud_converter_;

  std::shared_ptr<costmap_2d::OccupancyGridMapUpdaterInterface> occupancy_grid_map_updater_ptr_;

  // ROS Parameters
  std::string map_frame_;
  std::string base_link_frame_;
  bool use_height_filter_;

};

}  // namespace occupancy_grid_map
