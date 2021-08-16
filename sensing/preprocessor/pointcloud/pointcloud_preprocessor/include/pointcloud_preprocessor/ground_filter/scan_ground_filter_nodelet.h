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

#include <sensor_msgs/PointCloud2.h>

#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include "pointcloud_preprocessor/ScanGroundFilterConfig.h"
#include "pointcloud_preprocessor/filter.h"

namespace pointcloud_preprocessor
{
class ScanGroundFilterNodelet : public pointcloud_preprocessor::Filter
{
private:
  // classified point label (0: not classified, 1: ground, 2: not ground, 3: follow previous point, 4: unkown(currently not used), 5: virtual ground)
  enum class PointLabel {
    INIT = 0,
    GROUND,
    NON_GROUND,
    POINT_FOLLOW,
    UNKNOWN,
    VIRTUAL_GROUND,
  };
  struct PointRef
  {
    float radius;       // cylindrical coords on XY Plane
    float theta;        // angle deg on XY plane
    size_t radial_div;  // index of the radial division to which this point belongs to
    PointLabel point_state;

    size_t orig_index;  // index of this point in the source pointcloud
    pcl::PointXYZ * orig_point;

    PointRef() : point_state(PointLabel::INIT) {}
  };
  typedef std::vector<PointRef> PointCloudRefVector;

  boost::shared_ptr<dynamic_reconfigure::Server<pointcloud_preprocessor::ScanGroundFilterConfig>>
    srv_;

  void filter(
    const PointCloud2::ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

  void subscribe() override;

  void unsubscribe() override;

  bool child_init(ros::NodeHandle & nh, bool & has_service) override;

  void config_callback(pointcloud_preprocessor::ScanGroundFilterConfig & config, uint32_t level);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string base_frame_;
  std::string sensor_frame_;
  double global_slope_max_;                 // radians
  double local_slope_max_angle_;            // radians
  double local_slope_max_dist_;             // distance threshold in meters
  double radial_divider_angle_;             // distance in rads between dividers
  double split_points_distance_tolerance_;  // distance in meters between concentric divisions
  double
    split_height_distance_;  // minimum height threshold regardless the slope, useful for close points
  bool use_virtual_ground_point_;
  size_t radial_dividers_num_;

  /*!
   * Output transformed PointCloud from in_cloud_ptr->header.frame_id to in_target_frame
   * @param[in] in_target_frame Coordinate system to perform transform
   * @param[in] in_cloud_ptr PointCloud to perform transform
   * @param[out] out_cloud_ptr Resulting transformed PointCloud
   * @retval true transform succeeded
   * @retval false transform failed
   */
  bool transformPointCloud(
    const std::string & in_target_frame, const sensor_msgs::PointCloud2::ConstPtr & in_cloud_ptr,
    const sensor_msgs::PointCloud2::Ptr & out_cloud_ptr);

  /*!
   * Convert pcl::PointCloud to sorted PointCloudRefVector
   * @param[in] in_cloud Input Point Cloud to be organized in radial segments
   * @param[out] out_radial_ordered_points_manager Vector of Points Clouds, each element will contain the points ordered
   */
  void convertPointcloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
    std::vector<PointCloudRefVector> & out_radial_ordered_points_manager);

  /*!
   * Output ground center of front wheels as the virtual ground point
   * @param[out] point Virtual ground origin point
   */
  void calcVirtualGroundOrigin(pcl::PointXYZ & point);

  /*!
   * Classifies Points in the PointCloud as Ground and Not Ground
   * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
   * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud
   */
  void classifyPointCloud(
    std::vector<PointCloudRefVector> & in_radial_ordered_clouds,
    pcl::PointIndices & out_no_ground_indices);

  /*!
   * Returns the resulting complementary PointCloud, one with the points kept and the other removed as indicated
   * in the indices
   * @param in_cloud_ptr Input PointCloud to which the extraction will be performed
   * @param in_indices Indices of the points to be both removed and kept
   * @param out_object_cloud_ptr Resulting PointCloud with the indices kept
   */
  void extractObjectPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, const pcl::PointIndices & in_indices,
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_object_cloud_ptr);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ScanGroundFilterNodelet();
};
}  // namespace pointcloud_preprocessor
