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

#include <laserscan_to_occupancy_grid_map/cost_value.h>
#include <laserscan_to_occupancy_grid_map/laserscan_to_occupancy_grid_map_nodelet.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_eigen/tf2_eigen.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <autoware_utils/autoware_utils.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pluginlib/class_list_macros.h>

namespace
{
bool transformPointcloud(
  const sensor_msgs::PointCloud2 & input, const tf2_ros::Buffer & tf2,
  const std::string & target_frame, sensor_msgs::PointCloud2 & output)
{
  geometry_msgs::TransformStamped tf_stamped;
  tf_stamped = tf2.lookupTransform(
    target_frame, input.header.frame_id, input.header.stamp, ros::Duration(0.5));
  // transform pointcloud
  Eigen::Matrix4f tf_matrix = tf2::transformToEigen(tf_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(tf_matrix, input, output);
  output.header.stamp =input.header.stamp;
  output.header.frame_id = target_frame;
  return true;
}

void cropPointcloudByHeight(
  const sensor_msgs::PointCloud2 & input, const tf2_ros::Buffer & tf2,
  const std::string & target_frame, const float min_height, const float max_height,
  sensor_msgs::PointCloud2 & output)
{
  // Transformed pointcloud on target frame
  sensor_msgs::PointCloud2 trans_input_tmp;
  const bool is_target_frame = (input.header.frame_id == target_frame);
  if (!is_target_frame) {
    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped = tf2.lookupTransform(
      target_frame, input.header.frame_id, input.header.stamp, ros::Duration(0.5));
    // transform pointcloud
    Eigen::Matrix4f tf_matrix = tf2::transformToEigen(tf_stamped.transform).matrix().cast<float>();
    pcl_ros::transformPointCloud(tf_matrix, input, trans_input_tmp);
  }
  const sensor_msgs::PointCloud2 & trans_input = is_target_frame ? input : trans_input_tmp;

  // Apply height filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(trans_input, "x"),
       iter_y(trans_input, "y"), iter_z(trans_input, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (min_height < *iter_z && *iter_z < max_height) {
      pcl_output->push_back(pcl::PointXYZ(*iter_x, *iter_y, *iter_z));
    }
  }

  // Convert to ros msg
  pcl::toROSMsg(*pcl_output, output);
  output.header = input.header;

  return;
}

geometry_msgs::Pose getPose(
  const std_msgs::Header & source_header, const tf2_ros::Buffer & tf2,
  const std::string & target_frame)
{
  geometry_msgs::Pose pose;
  geometry_msgs::TransformStamped tf_stamped;
  tf_stamped = tf2.lookupTransform(
    target_frame, source_header.frame_id, source_header.stamp, ros::Duration(0.5));
  pose = autoware_utils::transform2pose(tf_stamped.transform);
  return pose;
}
}  // namespace

namespace occupancy_grid_map
{
OccupancyGridMapNodelet::OccupancyGridMapNodelet() {}

void OccupancyGridMapNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  /* params */
  bool input_obstacle_pointcloud, input_obstacle_and_raw_pointcloud;
  double map_length, map_resolution;
  private_nh_.param<std::string>("map_frame", map_frame_, "map");
  private_nh_.param<std::string>("base_link_frame", base_link_frame_, "base_link");
  private_nh_.param<double>("map_length", map_length, 100.0);
  private_nh_.param<double>("map_resolution", map_resolution, 0.5);
  private_nh_.param<bool>("input_obstacle_pointcloud", input_obstacle_pointcloud, true);
  private_nh_.param<bool>("input_obstacle_and_raw_pointcloud", input_obstacle_and_raw_pointcloud, true);
  private_nh_.param<bool>("use_height_filter", use_height_filter_, true);

  /* tf */
  tf2_.reset(new tf2_ros::Buffer());
  tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));

  /* Subscriber and publisher */
  laserscan_sub_.subscribe(private_nh_, "input/laserscan", 1);
  obstacle_pointcloud_sub_.subscribe(private_nh_, "input/obstacle_pointcloud", 1);
  raw_pointcloud_sub_.subscribe(private_nh_, "input/raw_pointcloud", 1);
  // add dummy callback to enable passthrough filter
  laserscan_sub_.registerCallback(
    std::bind(&OccupancyGridMapNodelet::onDummyPointCloud2, this, std::placeholders::_1));
  if (input_obstacle_and_raw_pointcloud)
    sync_ptr_ = std::make_shared<Sync>(
      SyncPolicy(5), laserscan_sub_, obstacle_pointcloud_sub_, raw_pointcloud_sub_);
  else if (input_obstacle_pointcloud)
    sync_ptr_ =
      std::make_shared<Sync>(SyncPolicy(3), laserscan_sub_, obstacle_pointcloud_sub_, passthrough_);
  else
    sync_ptr_ = std::make_shared<Sync>(SyncPolicy(3), laserscan_sub_, passthrough_, passthrough_);

  sync_ptr_->registerCallback(std::bind(
    &OccupancyGridMapNodelet::onLaserscanPointCloud2WithObstacleAndRaw, this, std::placeholders::_1,
    std::placeholders::_2, std::placeholders::_3));
  occupancy_grid_map_pub_ =
    private_nh_.advertise<nav_msgs::OccupancyGrid>("output/occupancy_grid_map", 1);

  /* Occupancy grid */
  occupancy_grid_map_updater_ptr_ = std::make_shared<costmap_2d::OccupancyGridMapBBFUpdater>(
    map_length / map_resolution, map_length / map_resolution, map_resolution);
}

sensor_msgs::PointCloud2::Ptr OccupancyGridMapNodelet::convertLaserscanToPointCLoud2(
  const sensor_msgs::LaserScan::ConstPtr & input)
{
  // check over max range point
  const float max_range =
    static_cast<float>(occupancy_grid_map_updater_ptr_->getSizeInCellsX()) * 0.5f +
    occupancy_grid_map_updater_ptr_->getResolution();
  constexpr float epsilon = 0.001;
  sensor_msgs::LaserScan laserscan = *input;
  laserscan.range_max = max_range;
  for (auto & range : laserscan.ranges) {
    if (max_range < range || std::isinf(range)) {
      range = max_range - epsilon;
    }
  }

  // convert to pointcloud
  sensor_msgs::PointCloud2::Ptr pointcloud_ptr = boost::make_shared<sensor_msgs::PointCloud2>();
  pointcloud_ptr->header = laserscan.header;
  laserscan2pointcloud_converter_.transformLaserScanToPointCloud(
    laserscan.header.frame_id, laserscan, *pointcloud_ptr, *tf2_);

  return pointcloud_ptr;
}

void OccupancyGridMapNodelet::onLaserscanPointCloud2WithObstacleAndRaw(
  const sensor_msgs::LaserScan::ConstPtr & input_laserscan_msg,
  const sensor_msgs::PointCloud2::ConstPtr & input_obstacle_msg,
  const sensor_msgs::PointCloud2::ConstPtr & input_raw_msg)
{
  // Laserscan to pointcloud2
  sensor_msgs::PointCloud2::ConstPtr laserscan_pc_ptr =
    convertLaserscanToPointCLoud2(input_laserscan_msg);

  // Apply height filter
  sensor_msgs::PointCloud2 cropped_obstacle_pc, cropped_raw_pc;
  if (use_height_filter_) {
    constexpr float min_height = -1.0, max_height = 2.0;
    cropPointcloudByHeight(
      *input_obstacle_msg, *tf2_, base_link_frame_, min_height, max_height, cropped_obstacle_pc);
    cropPointcloudByHeight(
      *input_raw_msg, *tf2_, base_link_frame_, min_height, max_height, cropped_raw_pc);
  }
  const sensor_msgs::PointCloud2 & filtered_obstacle_pc =
    use_height_filter_ ? cropped_obstacle_pc : *input_obstacle_msg;
  const sensor_msgs::PointCloud2 & filtered_raw_pc =
    use_height_filter_ ? cropped_raw_pc : *input_raw_msg;

  // Transform pointcloud and get frame pose
  sensor_msgs::PointCloud2 trans_laserscan_pc, trans_obstacle_pc, trans_raw_pc;
  geometry_msgs::Pose pose;
  try {
    transformPointcloud(*laserscan_pc_ptr, *tf2_, map_frame_, trans_laserscan_pc);
    transformPointcloud(filtered_obstacle_pc, *tf2_, map_frame_, trans_obstacle_pc);
    transformPointcloud(filtered_raw_pc, *tf2_, map_frame_, trans_raw_pc);
    pose = getPose(laserscan_pc_ptr->header, *tf2_, map_frame_);
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  // Create oneshot occupancy grid map
  costmap_2d::OccupancyGridMap oneshot_occupancy_grid_map(
    occupancy_grid_map_updater_ptr_->getSizeInCellsX(),
    occupancy_grid_map_updater_ptr_->getSizeInCellsY(),
    occupancy_grid_map_updater_ptr_->getResolution());
  oneshot_occupancy_grid_map.updateOrigin(
    pose.position.x - oneshot_occupancy_grid_map.getSizeInMetersX() / 2,
    pose.position.y - oneshot_occupancy_grid_map.getSizeInMetersY() / 2);
  oneshot_occupancy_grid_map.updateFreespaceCells(trans_raw_pc);
  oneshot_occupancy_grid_map.raytrace2D(trans_laserscan_pc, pose);
  oneshot_occupancy_grid_map.updateOccupiedCells(trans_obstacle_pc);

  // Update with bayes filter
  occupancy_grid_map_updater_ptr_->update(oneshot_occupancy_grid_map);

  // publish
  occupancy_grid_map_pub_.publish(OccupancyGridMapToMsgPtr(
    map_frame_, laserscan_pc_ptr->header.stamp, pose.position.z, *occupancy_grid_map_updater_ptr_));
}

boost::shared_ptr<nav_msgs::OccupancyGrid> OccupancyGridMapNodelet::OccupancyGridMapToMsgPtr(
  const std::string & frame_id, const ros::Time & time, const float & robot_pose_z,
  const costmap_2d::Costmap2D & occupancy_grid_map)
{
  boost::shared_ptr<nav_msgs::OccupancyGrid> msg_ptr(new nav_msgs::OccupancyGrid);

  msg_ptr->header.frame_id = frame_id;
  msg_ptr->header.stamp = time;
  msg_ptr->info.resolution = occupancy_grid_map.getResolution();

  msg_ptr->info.width = occupancy_grid_map.getSizeInCellsX();
  msg_ptr->info.height = occupancy_grid_map.getSizeInCellsY();

  double wx, wy;
  occupancy_grid_map.mapToWorld(0, 0, wx, wy);
  msg_ptr->info.origin.position.x = occupancy_grid_map.getOriginX();
  msg_ptr->info.origin.position.y = occupancy_grid_map.getOriginY();
  msg_ptr->info.origin.position.z = robot_pose_z;
  msg_ptr->info.origin.orientation.w = 1.0;

  msg_ptr->data.resize(msg_ptr->info.width * msg_ptr->info.height);

  unsigned char * data = occupancy_grid_map.getCharMap();
  for (unsigned int i = 0; i < msg_ptr->data.size(); i++) {
    msg_ptr->data[i] = occupancy_cost_value::cost_translation_table[data[i]];
  }
  return msg_ptr;
}

}  // namespace occupancy_grid_map

PLUGINLIB_EXPORT_CLASS(occupancy_grid_map::OccupancyGridMapNodelet, nodelet::Nodelet)
