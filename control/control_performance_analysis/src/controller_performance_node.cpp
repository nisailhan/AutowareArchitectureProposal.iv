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

#include "controller_performance_node.h"
#include "control_performance_analysis/ErrorStamped.h"

namespace
{
control_performance_analysis::ErrorStamped createPerformanceMsgVars(
  const TargetPerformanceMsgVars & target_performance_vars)
{
  control_performance_analysis::ErrorStamped error_msgs{};

  error_msgs.error.lateral_error = target_performance_vars.lateral_error;
  error_msgs.error.heading_error = target_performance_vars.heading_error;
  error_msgs.error.control_effort_energy = target_performance_vars.control_effort_energy;
  error_msgs.error.error_energy = target_performance_vars.error_energy;
  error_msgs.error.value_approximation = target_performance_vars.value_approximation;
  error_msgs.error.curvature_estimate = target_performance_vars.curvature_estimate;
  error_msgs.error.curvature_estimate_pp = target_performance_vars.curvature_estimate_pp;
  error_msgs.error.lateral_error_velocity = target_performance_vars.lateral_error_velocity;
  error_msgs.error.lateral_error_acceleration = target_performance_vars.lateral_error_acceleration;

  return error_msgs;
}
}  // namespace

ControlPerformanceNode::ControlPerformanceNode() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  // Implement Reading Global and Local Variables.
  param_.wheel_base = waitForParam<double>(pnh_, "/vehicle_info/wheel_base");

  // Node Parameters.
  pnh_.param("control_period", param_.control_period, 0.033);
  pnh_.param("curvature_interval_length", param_.curvature_interval_length, 10.0);

  // Prepare error computation class with the wheelbase parameter.
  control_performance_core_ptr_ = std::move(
    std::make_unique<ControlPerformanceCore>(param_.wheel_base, param_.curvature_interval_length));

  // Subscribers.
  sub_trajectory_ =
    pnh_.subscribe("input/reference_trajectory", 1, &ControlPerformanceNode::onTrajectory, this);

  sub_control_steering_ =
    pnh_.subscribe("input/control_raw", 1, &ControlPerformanceNode::onControlRaw, this);

  sub_vehicle_steering_ = pnh_.subscribe(
    "input/measured_steering", 1, &ControlPerformanceNode::onVecSteeringMeasured, this);

  sub_velocity_ =
    pnh_.subscribe("input/current_velocity", 1, &ControlPerformanceNode::onVelocity, this);

  // Publishers
  pub_error_msg_ =
    pnh_.advertise<control_performance_analysis::ErrorStamped>("output/error_stamped", 1);

  // Wait for first self pose
  self_pose_listener_.waitForFirstPose();

  // Timer
  timer_publish_ =
    nh_.createTimer(ros::Duration(param_.control_period), &ControlPerformanceNode::onTimer, this);
}

void ControlPerformanceNode::onTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr & msg)
{
  if (msg->points.size() < 3) {
    ROS_DEBUG("[control_performance_analysis] received path size < 3, is not sufficient.");
    return;
  }

  if (!isValidTrajectory(*msg)) {
    ROS_ERROR("[control_performance_analysis] Trajectory is invalid!, stop computing.");
    return;
  }

  current_trajectory_ptr_ = msg;
}

void ControlPerformanceNode::onControlRaw(
  const autoware_control_msgs::ControlCommandStampedConstPtr & control_msg)
{
  if (!control_msg) {
    ROS_ERROR("[control_performance_analysis] steering signal has not been received yet ...");
    return;
  }
  current_control_msg_ptr_ = control_msg;
}

void ControlPerformanceNode::onVecSteeringMeasured(
  const autoware_vehicle_msgs::SteeringConstPtr & meas_steer_msg)
{
  if (!meas_steer_msg) {
    ROS_WARN_THROTTLE(
      1.0,
      "[control_performance_analysis] waiting for vehicle measured steering "
      "message ...");
    return;
  }
  current_vec_steering_msg_ptr_ = meas_steer_msg;
}

void ControlPerformanceNode::onVelocity(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
  current_velocity_ptr_ = msg;
}

void ControlPerformanceNode::onTimer(const ros::TimerEvent & timer_event)
{
  // Read and Update Current Pose updating  var:current_pose_.
  current_pose_ = self_pose_listener_.getCurrentPose();

  // Check Data Stream
  if (!isDataReady()) {
    // Publish Here
    return;
  }

  // Compute Control Performance Variables.
  auto performanceVars = computeTargetPerformanceMsgVars();
  if (!performanceVars) {
    ROS_ERROR("[control_performance_analysis] steering signal has not been received yet ...");
    return;
  }

  // If successful publish.
  publishErrorMsg(*performanceVars);
}

void ControlPerformanceNode::publishErrorMsg(
  const TargetPerformanceMsgVars & control_performance_vars)
{
  control_performance_analysis::ErrorStamped error_msgs =
    createPerformanceMsgVars(control_performance_vars);

  pub_error_msg_.publish(error_msgs);
}

bool ControlPerformanceNode::isDataReady() const
{
  if (!current_pose_) {
    ROS_WARN_THROTTLE(1.0, "[control_performance_analysis] waiting for current_pose ...");
    return false;
  }

  if (!current_trajectory_ptr_) {
    ROS_WARN_THROTTLE(1.0, "[control_performance_analysis] waiting for trajectory ... ");
    return false;
  }

  if (!current_velocity_ptr_) {
    ROS_WARN_THROTTLE(1.0, "[control_performance_analysis] waiting for current_velocity ...");
    return false;
  }

  if (!current_control_msg_ptr_) {
    ROS_WARN_THROTTLE(
      1.0, "[control_performance_analysis] waiting for current_control_steering_val ...");
    return false;
  }

  return true;
}

/*
   *  - Pass trajectory and current pose to control_performance_analysis -> setCurrentPose()
   *                                                               -> setWayPoints()
   *                                                               -> findClosestPoint
   *                                                               -> computePerformanceVars
   * */

boost::optional<TargetPerformanceMsgVars> ControlPerformanceNode::computeTargetPerformanceMsgVars()
  const
{
  // Set trajectory and current pose of controller_performance_core.
  control_performance_core_ptr_->setCurrentWaypoints(*current_trajectory_ptr_);
  control_performance_core_ptr_->setCurrentPose(current_pose_->pose);
  control_performance_core_ptr_->setCurrentVelocities(current_velocity_ptr_->twist);
  control_performance_core_ptr_->setCurrentControValue(*current_control_msg_ptr_);

  // Find the index of the next waypoint.
  std::pair<bool, int32_t> prev_closest_wp_pose_idx =
    control_performance_core_ptr_->findClosestPrevWayPointIdx_path_direction();

  if (!prev_closest_wp_pose_idx.first) {
    ROS_ERROR("[control_performance_analysis] Cannot find closest waypoint");
    return {};
  }

  // Compute control performance values.
  const std::pair<bool, TargetPerformanceMsgVars> target_performance_vars =
    control_performance_core_ptr_->getPerformanceVars();

  if (!target_performance_vars.first) {
    ROS_ERROR("[control_performance_analysis] Cannot compute control performance vars ...");
    return {};
  }

  return target_performance_vars.second;
}

bool ControlPerformanceNode::isValidTrajectory(const autoware_planning_msgs::Trajectory & traj)
{
  bool check_condition = std::all_of(traj.points.cbegin(), traj.points.cend(), [](auto point) {
    const auto & p = point.pose.position;
    const auto & o = point.pose.orientation;
    const auto & t = point.twist.linear;
    const auto & a = point.accel.linear;

    if (
      !isfinite(p.x) || !isfinite(p.y) || !isfinite(p.z) || !isfinite(o.x) || !isfinite(o.y) ||
      !isfinite(o.z) || !isfinite(o.w) || !isfinite(t.x) || !isfinite(t.y) || !isfinite(t.z) ||
      !isfinite(a.x) || !isfinite(a.y) || !isfinite(a.z)) {
      return false;
    } else {
      return true;
    }
  });

  return check_condition;
}
