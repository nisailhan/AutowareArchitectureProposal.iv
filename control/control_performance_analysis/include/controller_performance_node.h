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

#ifndef CONTROL_PERFORMANCE_MEASUREMENT_CONTROLLER_PERFORMANCE_NODE_H
#define CONTROL_PERFORMANCE_MEASUREMENT_CONTROLLER_PERFORMANCE_NODE_H
#define EIGEN_MPL2_ONLY

#include <tf2_ros/transform_listener.h>
#include "controller_performance_core.h"
#include "ros/ros.h"

#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_utils/ros/self_pose_listener.h>
#include <autoware_vehicle_msgs/Steering.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// Parameters Struct
struct Param
{
  // Global parameters
  double wheel_base;
  double curvature_interval_length;

  // Control Method Parameters
  double control_period;
};

template <class T>
T waitForParam(const ros::NodeHandle & nh, const std::string & key)
{
  T value;
  ros::Rate rate(1.0);

  while (ros::ok()) {
    const auto result = nh.getParam(key, value);
    if (result) {
      return value;
    }

    ROS_WARN("waiting for parameter `%s` ...", key.c_str());
    rate.sleep();
  }

  return {};
}

class ControlPerformanceNode
{
public:
  ControlPerformanceNode();

private:
  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Subscribers and Local Variable Assignment
  ros::Subscriber sub_trajectory_;        // subscribe to trajectory
  ros::Subscriber sub_control_steering_;  // subscribe to steering control value
  ros::Subscriber sub_velocity_;          // subscribe to velocity
  ros::Subscriber sub_vehicle_steering_;

  // Self Pose listener.
  autoware_utils::SelfPoseListener self_pose_listener_;  // subscribe to pose listener.

  // Publishers
  ros::Publisher pub_error_msg_;  // publish error message

  // Node Methods
  bool isDataReady() const;  // check if data arrive
  static bool isValidTrajectory(const autoware_planning_msgs::Trajectory & traj);
  boost::optional<TargetPerformanceMsgVars> computeTargetPerformanceMsgVars() const;

  // Callback Methods
  void onTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr & msg);
  void publishErrorMsg(const TargetPerformanceMsgVars & control_performance_vars);
  void onControlRaw(const autoware_control_msgs::ControlCommandStampedConstPtr & control_msg);
  void onVecSteeringMeasured(const autoware_vehicle_msgs::SteeringConstPtr & meas_steer_msg);
  void onVelocity(const geometry_msgs::TwistStamped::ConstPtr & msg);

  // Timer - To Publish In Control Period
  ros::Timer timer_publish_;
  void onTimer(const ros::TimerEvent & timer_event);

  // Parameters
  Param param_{};  // wheelbase, control period and feedback coefficients.
  TargetPerformanceMsgVars target_error_vars_{};

  // Subscriber Parameters
  autoware_planning_msgs::Trajectory::ConstPtr current_trajectory_ptr_;  // ConstPtr to local traj.
  autoware_control_msgs::ControlCommandStampedConstPtr current_control_msg_ptr_;
  autoware_vehicle_msgs::SteeringConstPtr current_vec_steering_msg_ptr_;
  geometry_msgs::TwistStamped::ConstPtr current_velocity_ptr_;
  geometry_msgs::PoseStamped::ConstPtr current_pose_;  // pose of the vehicle, x, y, heading

  // TF Parameters - To Listen Current Pose
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Algorithm
  std::unique_ptr<ControlPerformanceCore> control_performance_core_ptr_;
};

#endif  //CONTROL_PERFORMANCE_MEASUREMENT_CONTROLLER_PERFORMANCE_NODE_H
