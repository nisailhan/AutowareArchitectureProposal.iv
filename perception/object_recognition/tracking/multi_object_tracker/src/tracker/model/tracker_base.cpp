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
 *
 */

#include "multi_object_tracker/tracker/model/tracker_base.hpp"
#include "multi_object_tracker/utils/utils.hpp"

Tracker::Tracker(const ros::Time & time, const int type)
: uuid_(unique_id::fromRandom()),
  type_(type),
  no_measurement_count_(0),
  total_no_measurement_count_(0),
  total_measurement_count_(1),
  last_update_with_measurement_time_(time)
{
}

bool Tracker::updateWithMeasurement(
  const autoware_perception_msgs::DynamicObject & object, const ros::Time & measurement_time)
{
  no_measurement_count_ = 0;
  ++total_measurement_count_;
  last_update_with_measurement_time_ = measurement_time;
  measure(object, measurement_time);
  return true;
}

bool Tracker::updateWithoutMeasurement()
{
  ++no_measurement_count_;
  ++total_no_measurement_count_;
  return true;
}

geometry_msgs::PoseWithCovariance Tracker::getPoseWithCovariance(const ros::Time & time) const
{
  autoware_perception_msgs::DynamicObject object;
  getEstimatedDynamicObject(time, object);
  return object.state.pose_covariance;
}
