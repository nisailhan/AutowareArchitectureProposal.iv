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

#ifndef VELOCITY_CONTROLLER_SMOOTH_STOP
#define VELOCITY_CONTROLLER_SMOOTH_STOP

#include <ros/ros.h>

class SmoothStop
{
public:
  void initTime(const ros::Time & time) { start_time_ = time; }

  void setParams(
    double weak_brake_time, double weak_brake_acc, double increasing_brake_time,
    double increasing_brake_gradient, double stop_brake_time, double stop_brake_acc)
  {
    params_.weak_brake_time = weak_brake_time;
    params_.weak_brake_acc = weak_brake_acc;
    params_.increasing_brake_time = increasing_brake_time;
    params_.increasing_brake_gradient = increasing_brake_gradient;
    params_.stop_brake_time = stop_brake_time;
    params_.stop_brake_acc = stop_brake_acc;
  }

  double calculate() const
  {
    const double elapsed_time = (ros::Time::now() - start_time_).toSec();

    const double t0 = params_.weak_brake_time;
    const double t1 = t0 + params_.increasing_brake_time;
    const double t2 = t1 + params_.stop_brake_time;
    if (elapsed_time < t0) {
      return params_.weak_brake_acc;
    } else if (elapsed_time < t1) {
      const double dt = elapsed_time - t0;
      return params_.weak_brake_acc + params_.increasing_brake_gradient * dt;
    } else if (elapsed_time < t2) {
      return params_.stop_brake_acc;
    } else {
      return params_.stop_brake_acc;
    }
  }

private:
  struct Params
  {
    double weak_brake_time;
    double weak_brake_acc;
    double increasing_brake_time;
    double increasing_brake_gradient;
    double stop_brake_time;
    double stop_brake_acc;
  } params_;

  ros::Time start_time_;
};

#endif
