//
//  Copyright 2021 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "raw_vehicle_cmd_converter/node.hpp"

RawVehicleCommandConverter::RawVehicleCommandConverter() : nh_(""), pnh_("~")
{
  /* parameters for accel/brake map */
  std::string csv_path_accel_map, csv_path_brake_map, csv_path_steer_map;
  pnh_.param<std::string>("csv_path_accel_map", csv_path_accel_map, std::string("empty"));
  pnh_.param<std::string>("csv_path_brake_map", csv_path_brake_map, std::string("empty"));
  pnh_.param<std::string>("csv_path_steer_map", csv_path_steer_map, std::string("empty"));
  pnh_.param<bool>("convert_accel_cmd", convert_accel_cmd_, true);
  pnh_.param<bool>("convert_brake_cmd", convert_brake_cmd_, true);
  pnh_.param<bool>("convert_steer_cmd", convert_steer_cmd_, true);
  pnh_.param<double>("max_throttle", max_accel_cmd_, 0.2);
  pnh_.param<double>("max_brake", max_brake_cmd_, 0.8);
  pnh_.param<double>("max_steer", max_steer_cmd_, 10.0);
  pnh_.param<double>("min_steer", min_steer_cmd_, -10.0);
  is_debugging_ = getParam<bool>(pnh_, "is_debugging");
  // for steering steer controller
  use_steer_ff_ = getParam<bool>(pnh_, "use_steer_ff");
  use_steer_fb_ = getParam<bool>(pnh_, "use_steer_fb");
  double kp_steer = getParam<double>(pnh_, "steer_pid/kp");
  double ki_steer = getParam<double>(pnh_, "steer_pid/ki");
  double kd_steer = getParam<double>(pnh_, "steer_pid/kd");
  double max_ret_steer = getParam<double>(pnh_, "steer_pid/max");
  double min_ret_steer = getParam<double>(pnh_, "steer_pid/min");
  double max_ret_p_steer = getParam<double>(pnh_, "steer_pid/max_p");
  double min_ret_p_steer = getParam<double>(pnh_, "steer_pid/min_p");
  double max_ret_i_steer = getParam<double>(pnh_, "steer_pid/max_i");
  double min_ret_i_steer = getParam<double>(pnh_, "steer_pid/min_i");
  double max_ret_d_steer = getParam<double>(pnh_, "steer_pid/max_d");
  double min_ret_d_steer = getParam<double>(pnh_, "steer_pid/min_d");
  double invalid_integration_decay = getParam<double>(pnh_, "steer_pid/invalid_integration_decay");
  ffmap_initialized_ = true;
  if (convert_accel_cmd_) {
    if (!accel_map_.readAccelMapFromCSV(csv_path_accel_map)) {
      ROS_ERROR(
        "Cannot read accelmap. csv path = %s. stop calculation.", csv_path_accel_map.c_str());
      ffmap_initialized_ = false;
    }
  }
  if (convert_brake_cmd_) {
    if (!brake_map_.readBrakeMapFromCSV(csv_path_brake_map)) {
      ROS_ERROR(
        "Cannot read brakemap. csv path = %s. stop calculation.", csv_path_brake_map.c_str());
      ffmap_initialized_ = false;
    }
  }
  if (convert_steer_cmd_) {
    steer_controller_.setDecay(invalid_integration_decay);
    if (!steer_controller_.setFFMap(csv_path_steer_map)) {
      ROS_ERROR(
        "Cannot read steer map. csv path = %s. stop calculation.", csv_path_steer_map.c_str());
      ffmap_initialized_ = false;
    }
    steer_controller_.setFBGains(kp_steer, ki_steer, kd_steer);
    steer_controller_.setFBLimits(
      max_ret_steer, min_ret_steer, max_ret_p_steer, min_ret_p_steer, max_ret_i_steer,
      min_ret_i_steer, max_ret_d_steer, min_ret_d_steer);
  }
  pub_actuation_cmd_ =
    nh_.advertise<autoware_vehicle_msgs::ActuationCommandStamped>("/vehicle/actuation_cmd", 1);
  sub_control_cmd_ =
    nh_.subscribe("/control/control_cmd", 1, &RawVehicleCommandConverter::callbackControlCmd, this);
  sub_velocity_ =
    nh_.subscribe("/localization/twist", 1, &RawVehicleCommandConverter::callbackVelocity, this);
  sub_steering_ = nh_.subscribe(
    "/vehicle/status/steering", 1, &RawVehicleCommandConverter::callbackSteering, this);
  debug_pub_steer_pid_ = nh_.advertise<std_msgs::Float32MultiArray>(
    "/vehicle/raw_vehicle_cmd_converter/debug/steer_pid", 1);
}

void RawVehicleCommandConverter::publishActuationCmd()
{
  if (!ffmap_initialized_) {
    ROS_WARN_COND(is_debugging_, "[RawVehicleCmdConverter] ff map is not initialized");
    return;
  }
  if (!current_twist_ptr_ || !control_cmd_ptr_ || !current_steer_ptr_) {
    ROS_WARN_COND(
      is_debugging_, "[RawVehicleCmdConverter] some of twist/control_cmd/steer pointer is null");
    return;
  }
  double desired_accel_cmd = 0.0;
  double desired_brake_cmd = 0.0;
  double desired_steer_cmd = 0.0;
  autoware_vehicle_msgs::ActuationCommandStamped actuation_cmd;
  const double acc = control_cmd_ptr_->control.acceleration;
  const double vel = current_twist_ptr_->twist.linear.x;
  const double steer = control_cmd_ptr_->control.steering_angle;
  const double steer_rate = control_cmd_ptr_->control.steering_angle_velocity;
  bool accel_cmd_is_zero = true;
  if (convert_accel_cmd_) {
    desired_accel_cmd = calculateAccelMap(vel, acc, accel_cmd_is_zero);
  } else {
    // if conversion is disabled use acceleration as actuation cmd
    desired_accel_cmd = (acc >= 0) ? acc : 0;
  }
  if (convert_brake_cmd_) {
    if (accel_cmd_is_zero) {
      desired_brake_cmd = calculateBrakeMap(vel, acc);
    }
  } else {
    // if conversion is disabled use acceleration as brake cmd
    desired_brake_cmd = (acc < 0) ? acc : 0;
  }
  if (convert_steer_cmd_) {
    desired_steer_cmd = calculateSteer(vel, steer, steer_rate);
  } else {
    // if conversion is disabled use steering angle as steer cmd
    desired_steer_cmd = control_cmd_ptr_->control.steering_angle;
  }
  actuation_cmd.header = control_cmd_ptr_->header;
  actuation_cmd.actuation.accel_cmd = desired_accel_cmd;
  actuation_cmd.actuation.brake_cmd = desired_brake_cmd;
  actuation_cmd.actuation.steer_cmd = desired_steer_cmd;
  pub_actuation_cmd_.publish(actuation_cmd);
}

double RawVehicleCommandConverter::calculateSteer(
  const double vel, const double steering, const double steer_rate)
{
  double steering_output = 0;
  double ff_value = 0;
  double fb_value = 0;
  std::vector<double> pid_contributions(3, 0.0);
  std::vector<double> pid_errors(3, 0.0);
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - prev_time_steer_calculation_).toSec();
  if (std::abs(dt) > 1.0) {
    ROS_WARN_COND(is_debugging_, "[RawVehicleCmdConverter] ignore old topic");
    dt = 0.0;
  }
  prev_time_steer_calculation_ = current_time;
  // feed-forward
  if (use_steer_ff_) {
    ff_value = steer_controller_.calcFFSteer(steer_rate, current_steer_ptr_->data);
  }
  // feedback
  if (use_steer_fb_) {
    fb_value = steer_controller_.calcFBSteer(
      steering, steer_rate, dt, vel, current_steer_ptr_->data, pid_contributions, pid_errors);
  }
  steering_output = ff_value + fb_value;
  // for steer debugging
  {
    debug_steer_.setValues(DebugValues::TYPE::CURR_TIME, current_time.sec);
    debug_steer_.setValues(DebugValues::TYPE::P, pid_contributions.at(0));
    debug_steer_.setValues(DebugValues::TYPE::I, pid_contributions.at(1));
    debug_steer_.setValues(DebugValues::TYPE::D, pid_contributions.at(2));
    debug_steer_.setValues(DebugValues::TYPE::FF, ff_value);
    debug_steer_.setValues(DebugValues::TYPE::FB, fb_value);
    debug_steer_.setValues(DebugValues::TYPE::STEER, steering_output);
    debug_steer_.setValues(DebugValues::TYPE::ERROR_P, pid_errors.at(0));
    debug_steer_.setValues(DebugValues::TYPE::ERROR_I, pid_errors.at(1));
    debug_steer_.setValues(DebugValues::TYPE::ERROR_D, pid_errors.at(2));
    std_msgs::Float32MultiArray msg;
    for (const auto & v : debug_steer_.getValues()) {
      msg.data.push_back(v);
    }
    debug_pub_steer_pid_.publish(msg);
  }
  steering_output = std::max(std::min(max_steer_cmd_, steering_output), min_steer_cmd_);
  return steering_output;
}

double RawVehicleCommandConverter::calculateAccelMap(
  const double current_velocity, const double desired_acc, bool & accel_cmd_is_zero)
{
  double desired_accel_cmd = 0;
  if (!accel_map_.getThrottle(desired_acc, std::abs(current_velocity), desired_accel_cmd)) {
    desired_accel_cmd = 0;
  } else {
    accel_cmd_is_zero = false;
  }
  desired_accel_cmd = std::min(std::max(desired_accel_cmd, 0.0), max_accel_cmd_);
  return desired_accel_cmd;
}

double RawVehicleCommandConverter::calculateBrakeMap(
  const double current_velocity, const double desired_acc)
{
  double desired_brake_cmd = 0;
  brake_map_.getBrake(desired_acc, std::abs(current_velocity), desired_brake_cmd);
  desired_brake_cmd = std::min(std::max(desired_brake_cmd, 0.0), max_brake_cmd_);
  return desired_brake_cmd;
}

void RawVehicleCommandConverter::callbackSteering(
  const autoware_vehicle_msgs::Steering::ConstPtr & msg)
{
  current_steer_ptr_ = msg;
}

void RawVehicleCommandConverter::callbackVelocity(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
  current_twist_ptr_ = msg;
}

void RawVehicleCommandConverter::callbackControlCmd(
  const autoware_control_msgs::ControlCommandStamped::ConstPtr & msg)
{
  control_cmd_ptr_ = msg;
  publishActuationCmd();
}
