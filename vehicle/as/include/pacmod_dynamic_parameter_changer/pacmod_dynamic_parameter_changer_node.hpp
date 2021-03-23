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
 */

#pragma once
#include <can_msgs/Frame.h>
#include <pacmod_msgs/SystemRptFloat.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

struct ParamList
{
  double p_gain;
  double i_gain;
  double lugre_fc;
  double rtz_offset;
};

class PacmodDynamicParameterChangerNode
{
private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher debug_pub_;
  ros::Publisher can_pub_;
  ros::Subscriber steer_rpt_sub_;

  ParamList straight_course_param_;
  ParamList curve_course_param_;
  ParamList param_max_rate_;
  ParamList param_min_rate_;
  double min_steer_thresh_;
  double max_steer_thresh_;

  ParamList current_param_list_;
  ros::Time current_param_time_;

public:
  PacmodDynamicParameterChangerNode();
  ~PacmodDynamicParameterChangerNode(){};
  void subSteerRpt(const pacmod_msgs::SystemRptFloat::ConstPtr msg);
  ParamList calculateParam(
    const double current_steer_cmd, const double current_steer, const bool enabled);
  void sendCanMsg(const ParamList param_list);
  void sendDebugMsg(const ParamList param_list);

  ParamList interpolateParam(const ParamList p1, const ParamList p2, const double p1_rate);
  ParamList rateLimit(const ParamList new_param, const ParamList current_param);
  double rateLimit(
    const double new_value, const double current_value, const double delta_time,
    const double max_rate, const double min_rate);
};
