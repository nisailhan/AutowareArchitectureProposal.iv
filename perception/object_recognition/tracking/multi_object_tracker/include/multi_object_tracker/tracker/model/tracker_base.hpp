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
 *
 * v1.0 Yukihiro Saito
 */

#pragma once
#include <autoware_perception_msgs/DynamicObject.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <unique_id/unique_id.h>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

class Tracker
{
protected:
  boost::uuids::uuid getUUID() const { return uuid_; }
  void setType(int type) { type_ = type; }

private:
  boost::uuids::uuid uuid_;
  int type_;
  int no_measurement_count_;
  int total_no_measurement_count_;
  int total_measurement_count_;
  ros::Time last_update_with_measurement_time_;

public:
  Tracker(const ros::Time & time, const int type);
  virtual ~Tracker(){};
  bool updateWithMeasurement(
    const autoware_perception_msgs::DynamicObject & object, const ros::Time & measurement_time);
  bool updateWithoutMeasurement();
  int getType() const { return type_; }
  int getNoMeasurementCount() const { return no_measurement_count_; }
  int getTotalNoMeasurementCount() const { return total_no_measurement_count_; }
  int getTotalMeasurementCount() const { return total_measurement_count_; }
  double getElapsedTimeFromLastUpdate() const
  {
    return (ros::Time::now() - last_update_with_measurement_time_).toSec();
  }
  virtual geometry_msgs::PoseWithCovariance getPoseWithCovariance(const ros::Time & time) const;

  /*
   *　Pure virtual function
   */
protected:
  virtual bool measure(
    const autoware_perception_msgs::DynamicObject & object, const ros::Time & time) = 0;

public:
  virtual bool getEstimatedDynamicObject(
    const ros::Time & time, autoware_perception_msgs::DynamicObject & object) const = 0;
  virtual bool predict(const ros::Time & time) = 0;
};
