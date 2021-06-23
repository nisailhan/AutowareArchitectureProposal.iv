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

#ifndef VEHICLE_RAW_VEHICLE_CMD_CONVERTER_INCLUDE_RAW_VEHICLE_CMD_CONVERTER_NODE_HPP_
#define VEHICLE_RAW_VEHICLE_CMD_CONVERTER_INCLUDE_RAW_VEHICLE_CMD_CONVERTER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_vehicle_msgs/ActuationCommandStamped.h>
#include <autoware_vehicle_msgs/Shift.h>

#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_vehicle_msgs/Steering.h>
#include <raw_vehicle_cmd_converter/accel_map.h>
#include <raw_vehicle_cmd_converter/brake_map.h>
#include <raw_vehicle_cmd_converter/pid.h>
#include <raw_vehicle_cmd_converter/steer_converter.h>
#include <std_msgs/Float32MultiArray.h>

template <class T>
T getParam(const ros::NodeHandle & nh, const std::string & key)
{
  T value;
  const auto result = nh.getParam(key, value);
  return value;
}

class DebugValues
{
public:
  enum class TYPE {
    CURR_TIME = 0,
    P = 1,
    I = 2,
    D = 3,
    FF = 4,
    FB = 5,
    STEER = 6,
    ERROR_P = 7,
    ERROR_I = 8,
    ERROR_D = 9,
    SIZE  // this is the number of enum elements
  };
  std::array<double, static_cast<int>(TYPE::SIZE)> getValues() const { return values_; }
  void setValues(TYPE type, double val) { values_.at(static_cast<int>(type)) = val; }
  void setValues(int type, double val) { values_.at(type) = val; }

private:
  std::array<double, static_cast<int>(TYPE::SIZE)> values_;
};

class RawVehicleCommandConverter
{
public:
  RawVehicleCommandConverter();
  ~RawVehicleCommandConverter() = default;

  ros::NodeHandle nh_;                //!< @brief ros node handle
  ros::NodeHandle pnh_;               //!< @brief private ros node handle
  ros::Publisher pub_actuation_cmd_;  //!< @brief topic publisher for low level vehicle command
  ros::Subscriber sub_velocity_;      //!< @brief subscriber for current velocity
  ros::Subscriber sub_control_cmd_;   //!< @brief subscriber for vehicle command
  ros::Subscriber sub_steering_;      //!< @brief subscriber for steering
  ros::Timer timer_;

  geometry_msgs::TwistStamped::ConstPtr current_twist_ptr_;  // [m/s]
  autoware_vehicle_msgs::Steering::ConstPtr current_steer_ptr_;
  autoware_control_msgs::ControlCommandStamped::ConstPtr control_cmd_ptr_;
  AccelMap accel_map_;
  BrakeMap brake_map_;
  bool ffmap_initialized_;
  double max_accel_cmd_;
  double max_brake_cmd_;
  double max_steer_cmd_;
  double min_steer_cmd_;
  // ros parameter
  bool use_steer_ff_;
  bool use_steer_fb_;
  bool is_debugging_;
  bool convert_accel_cmd_;  //!< @brief use accel or not
  bool convert_brake_cmd_;  //!< @brief use brake or not
  bool convert_steer_cmd_;  //!< @brief use steer or not
  SteerConverter steer_controller_;
  ros::Time prev_time_steer_calculation_;

  double calculateAccelMap(
    const double current_velocity, const double desired_acc, bool & accel_cmd_is_zero);
  double calculateBrakeMap(const double current_velocity, const double desired_acc);
  double calculateSteer(const double vel, const double steering, const double steer_rate);
  void callbackSteering(const autoware_vehicle_msgs::Steering::ConstPtr & msg);
  void callbackControlCmd(const autoware_control_msgs::ControlCommandStamped::ConstPtr & msg);
  void callbackVelocity(const geometry_msgs::TwistStamped::ConstPtr & msg);
  void publishActuationCmd();
  // for debugging
  ros::Publisher debug_pub_steer_pid_;
  DebugValues debug_steer_;
};

#endif  // VEHICLE_RAW_VEHICLE_CMD_CONVERTER_INCLUDE_RAW_VEHICLE_CMD_CONVERTER_NODE_HPP_
