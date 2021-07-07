/*
 * Copyright 2018 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not enable this file except in compliance with the License.
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

#include "velocity_controller/velocity_controller.h"

VelocityController::VelocityController()
: nh_(""),
  pnh_("~"),
  current_vel_ptr_(nullptr),
  prev_vel_ptr_(nullptr),
  trajectory_ptr_(nullptr),
  prev_control_time_(nullptr),
  control_state_(ControlState::STOPPED),
  last_running_time_(std::make_shared<ros::Time>(ros::Time::now())),
  prev_ctrl_cmd_(Motion{0.0, 0.0}),
  prev_shift_(Shift::Forward),
  lpf_vel_error_(nullptr),
  lpf_pitch_(nullptr),
  lpf_acc_(nullptr)
{
  // vehicle parameter
  wheel_base_ = waitForParam<double>(pnh_, "/vehicle_info/wheel_base");

  // parameters timer
  pnh_.param("control_rate", control_rate_, 30.0);

  // parameters for delay compensation
  pnh_.param("delay_compensation_time", delay_compensation_time_, 0.17);  // [sec]

  // parameters to enable functions
  pnh_.param("enable_smooth_stop", enable_smooth_stop_, true);
  pnh_.param("enable_overshoot_emergency", enable_overshoot_emergency_, true);
  pnh_.param("enable_slope_compensation", enable_slope_compensation_, false);

  // parameters for state transition
  {
    auto & p = state_transition_params_;
    // drive
    pnh_.param("drive_state_stop_dist", p.drive_state_stop_dist, 0.5);                // [m]
    pnh_.param("drive_state_offset_stop_dist", p.drive_state_offset_stop_dist, 1.0);  // [m]
    // stopping
    pnh_.param("stopping_state_stop_dist", p.stopping_state_stop_dist, 3.0);  // [m]
    // stop
    pnh_.param("stopped_state_entry_vel", p.stopped_state_entry_vel, 0.2);  // [m/s]
    pnh_.param("stopped_state_entry_acc", p.stopped_state_entry_acc, 0.2);  // [m/ss]
    // emergency
    pnh_.param(
      "emergency_state_overshoot_stop_dist", p.emergency_state_overshoot_stop_dist, 1.5);  // [m]
    pnh_.param("emergency_state_traj_trans_dev", p.emergency_state_traj_trans_dev, 3.0);   // [m]
    pnh_.param("emergency_state_traj_rot_dev", p.emergency_state_traj_rot_dev, 0.7);       // [m]
  }

  // parameters for drive state
  {
    // initialize PID gain
    double kp, ki, kd;
    pnh_.param("kp", kp, 0.0);
    pnh_.param("ki", ki, 0.0);
    pnh_.param("kd", kd, 0.0);
    pid_vel_.setGains(kp, ki, kd);

    // initialize PID limits
    double max_pid, min_pid, max_p, min_p, max_i, min_i, max_d, min_d;
    pnh_.param("max_out", max_pid, 0.0);     // [m/s^2]
    pnh_.param("min_out", min_pid, 0.0);     // [m/s^2]
    pnh_.param("max_p_effort", max_p, 0.0);  // [m/s^2]
    pnh_.param("min_p_effort", min_p, 0.0);  // [m/s^2]
    pnh_.param("max_i_effort", max_i, 0.0);  // [m/s^2]
    pnh_.param("min_i_effort", min_i, 0.0);  // [m/s^2]
    pnh_.param("max_d_effort", max_d, 0.0);  // [m/s^2]
    pnh_.param("min_d_effort", min_d, 0.0);  // [m/s^2]
    pid_vel_.setLimits(max_pid, min_pid, max_p, min_p, max_i, min_i, max_d, min_d);

    // set lowpass filter for vel error and pitch
    double lpf_vel_error_gain;
    pnh_.param("lpf_vel_error_gain", lpf_vel_error_gain, 0.9);
    lpf_vel_error_ = std::make_shared<LowpassFilter1d>(0.0, lpf_vel_error_gain);

    pnh_.param(
      "current_vel_threshold_pid_integration", current_vel_threshold_pid_integrate_,
      0.5);  // [m/s]
  }

  // parameters for smooth stop state
  {
    double max_strong_acc, min_strong_acc, weak_acc, weak_stop_acc, strong_stop_acc, max_fast_vel,
      min_running_vel, min_running_acc, weak_stop_time, weak_stop_dist, strong_stop_dist;
    pnh_.param("smooth_stop_max_strong_acc", max_strong_acc, -0.5);    // [m/s^2]
    pnh_.param("smooth_stop_min_strong_acc", min_strong_acc, -1.0);    // [m/s^2]
    pnh_.param("smooth_stop_weak_acc", weak_acc, -0.3);                // [m/s^2]
    pnh_.param("smooth_stop_weak_stop_acc", weak_stop_acc, -0.8);      // [m/s^2]
    pnh_.param("smooth_stop_strong_stop_acc", strong_stop_acc, -3.4);  // [m/s^2]

    pnh_.param("smooth_stop_max_fast_vel", max_fast_vel, 0.5);         // [m/s]
    pnh_.param("smooth_stop_min_running_vel", min_running_vel, 0.01);  // [m/s]
    pnh_.param("smooth_stop_min_running_acc", min_running_acc, 0.01);  // [m/s^2]
    pnh_.param("smooth_stop_weak_stop_time", weak_stop_time, 0.8);     // [s]

    pnh_.param("smooth_stop_weak_stop_dist", weak_stop_dist, -0.3);      // [m]
    pnh_.param("smooth_stop_strong_stop_dist", strong_stop_dist, -0.5);  // [m]

    smooth_stop_.setParams(
      max_strong_acc, min_strong_acc, weak_acc, weak_stop_acc, strong_stop_acc, max_fast_vel,
      min_running_vel, min_running_acc, weak_stop_time, weak_stop_dist, strong_stop_dist);
  }

  // parameters for stop state
  {
    auto & p = stopped_state_params_;
    pnh_.param("stopped_vel", p.vel, 0.0);   // [m/s]
    pnh_.param("stopped_acc", p.acc, -2.0);  // [m/s^2]
  }

  // parameters for emergency state
  {
    auto & p = emergency_state_params_;
    pnh_.param("emergency_vel", p.vel, 0.0);     // [m/s]
    pnh_.param("emergency_acc", p.acc, -2.0);    // [m/s^2]
    pnh_.param("emergency_jerk", p.jerk, -1.5);  // [m/s^3]
  }

  // parameters for acceleration limit
  pnh_.param("max_acc", max_acc_, 2.0);   // [m/s^2]
  pnh_.param("min_acc", min_acc_, -5.0);  // [m/s^2]

  // parameters for jerk limit
  pnh_.param("max_jerk", max_jerk_, 2.0);   // [m/s^3]
  pnh_.param("min_jerk", min_jerk_, -5.0);  // [m/s^3]

  // parameters for slope compensation
  pnh_.param<bool>("use_trajectory_for_pitch_calculation", use_traj_for_pitch_, false);
  double lpf_pitch_gain;
  pnh_.param("lpf_pitch_gain", lpf_pitch_gain, 0.95);
  lpf_pitch_ = std::make_shared<LowpassFilter1d>(0.0, lpf_pitch_gain);
  pnh_.param("max_pitch_rad", max_pitch_rad_, 0.1);   // [rad]
  pnh_.param("min_pitch_rad", min_pitch_rad_, -0.1);  // [rad]

  // dynamic reconfigure
  dynamic_reconfigure_srv_.setCallback(
    boost::bind(&VelocityController::callbackConfig, this, _1, _2));

  // subscriber, publisher and timer
  sub_current_vel_ =
    pnh_.subscribe("current_velocity", 1, &VelocityController::callbackCurrentVelocity, this);
  sub_trajectory_ =
    pnh_.subscribe("current_trajectory", 1, &VelocityController::callbackTrajectory, this);
  pub_control_cmd_ = pnh_.advertise<autoware_control_msgs::ControlCommandStamped>("control_cmd", 1);
  pub_slope_ = pnh_.advertise<std_msgs::Float32>("slope_angle", 1);
  pub_debug_ = pnh_.advertise<std_msgs::Float32MultiArray>("debug_values", 1);
  timer_control_ = nh_.createTimer(
    ros::Duration(1.0 / control_rate_), &VelocityController::callbackTimerControl, this);

  // set lowpass filter for acc
  lpf_acc_ = std::make_shared<LowpassFilter1d>(0.0, 0.2);

  // Wait for first self pose
  self_pose_listener_.waitForFirstPose();
}

void VelocityController::callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
  if (current_vel_ptr_) {
    prev_vel_ptr_ = current_vel_ptr_;
  }
  current_vel_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(*msg);
}

void VelocityController::callbackTrajectory(const autoware_planning_msgs::TrajectoryConstPtr & msg)
{
  if (!velocity_controller_utils::isValidTrajectory(*msg)) {
    ROS_ERROR_THROTTLE(3.0, "[velocity_controller] received invalid trajectory. ignore.");
    return;
  }

  if (msg->points.size() < 2) {
    ROS_WARN_THROTTLE(3.0, "Unexpected trajectory size < 2. Ignored.");
    return;
  }

  trajectory_ptr_ = std::make_shared<autoware_planning_msgs::Trajectory>(*msg);
}

void VelocityController::callbackConfig(
  const velocity_controller::VelocityControllerConfig & config, const uint32_t level)
{
  // delay compensation
  delay_compensation_time_ = config.delay_compensation_time;

  // state transition
  {
    auto & p = state_transition_params_;
    p.drive_state_stop_dist = config.drive_state_stop_dist;
    p.stopping_state_stop_dist = config.stopping_state_stop_dist;
    p.stopped_state_entry_vel = config.stopped_state_entry_vel;
    p.stopped_state_entry_acc = config.stopped_state_entry_acc;
    p.emergency_state_overshoot_stop_dist = config.emergency_state_overshoot_stop_dist;
    p.emergency_state_traj_trans_dev = config.emergency_state_traj_trans_dev;
    p.emergency_state_traj_rot_dev = config.emergency_state_traj_rot_dev;
  }

  // drive state
  pid_vel_.setGains(config.kp, config.ki, config.kd);
  pid_vel_.setLimits(
    config.max_out, config.min_out, config.max_p_effort, config.min_p_effort, config.max_i_effort,
    config.min_i_effort, config.max_d_effort, config.min_d_effort);
  current_vel_threshold_pid_integrate_ = config.current_velocity_threshold_pid_integration;

  // stopping state
  smooth_stop_.setParams(
    config.smooth_stop_max_strong_acc, config.smooth_stop_min_strong_acc,
    config.smooth_stop_weak_acc, config.smooth_stop_weak_stop_acc,
    config.smooth_stop_strong_stop_acc, config.smooth_stop_max_fast_vel,
    config.smooth_stop_min_running_vel, config.smooth_stop_min_running_acc,
    config.smooth_stop_weak_stop_time, config.smooth_stop_weak_stop_dist,
    config.smooth_stop_strong_stop_dist);

  // stop state
  {
    auto & p = stopped_state_params_;
    p.vel = config.stopped_vel;
    p.acc = config.stopped_acc;
  }

  // emergency state
  {
    auto & p = emergency_state_params_;
    p.vel = config.emergency_vel;
    p.acc = config.emergency_acc;
    p.jerk = config.emergency_jerk;
  }

  // acceleration limit
  max_acc_ = config.max_acc;
  min_acc_ = config.min_acc;

  // jerk limit
  max_jerk_ = config.max_jerk;
  min_jerk_ = config.min_jerk;

  // slope compensation
  max_pitch_rad_ = config.max_pitch_rad;
  min_pitch_rad_ = config.min_pitch_rad;
}

void VelocityController::callbackTimerControl(const ros::TimerEvent & event)
{
  // wait for initial pointers
  if (!current_vel_ptr_ || !prev_vel_ptr_ || !trajectory_ptr_) {
    return;
  }

  // calculate current pose and contorl data
  const auto current_pose = self_pose_listener_.getCurrentPose()->pose;
  const auto control_data = getControlData(current_pose);

  // self pose is far from trajectory
  if (control_data.is_far_from_trajectory) {
    control_state_ = ControlState::EMERGENCY;                       // update control state
    const Motion ctrl_cmd = calcEmergencyCtrlCmd(control_data.dt);  // calculate control command
    publishCtrlCmd(ctrl_cmd, control_data.current_motion.vel);      // publish control command
    publishDebugData(ctrl_cmd, control_data, current_pose);         // publish debug data
    return;
  }

  // update control state
  control_state_ = updateControlState(control_state_, current_pose, control_data);

  // calculate control command
  const Motion ctrl_cmd = calcCtrlCmd(control_state_, current_pose, control_data);

  // publish control command
  publishCtrlCmd(ctrl_cmd, control_data.current_motion.vel);

  // publish debug data
  publishDebugData(ctrl_cmd, control_data, current_pose);
}

VelocityController::ControlData VelocityController::getControlData(
  const geometry_msgs::Pose & current_pose)
{
  ControlData control_data;

  // dt
  control_data.dt = getDt();

  // current velocity and acceleration
  control_data.current_motion = getCurrentMotion();

  // closest idx
  const double max_dist = state_transition_params_.emergency_state_traj_trans_dev;
  const double max_yaw = state_transition_params_.emergency_state_traj_rot_dev;
  const auto closest_idx_opt =
    autoware_utils::findNearestIndex(trajectory_ptr_->points, current_pose, max_dist, max_yaw);

  // return here if closest index is not found
  if (!closest_idx_opt) {
    control_data.is_far_from_trajectory = true;
    return control_data;
  }
  control_data.closest_idx = *closest_idx_opt;

  // shift
  control_data.shift = getCurrentShift(control_data.closest_idx);
  if (control_data.shift != prev_shift_) pid_vel_.reset();
  prev_shift_ = control_data.shift;

  // distance to stopline
  control_data.stop_dist =
    velocity_controller_utils::calcStopDistance(current_pose.position, *trajectory_ptr_);

  // pitch
  const double raw_pitch = velocity_controller_utils::getPitchByPose(current_pose.orientation);
  const double traj_pitch = velocity_controller_utils::getPitchByTraj(
    *trajectory_ptr_, control_data.closest_idx, wheel_base_);
  control_data.slope_angle = use_traj_for_pitch_ ? traj_pitch : lpf_pitch_->filter(raw_pitch);
  updatePitchDebugValues(control_data.slope_angle, traj_pitch, raw_pitch);

  return control_data;
}

VelocityController::Motion VelocityController::calcEmergencyCtrlCmd(const double dt) const
{
  const auto & p = emergency_state_params_;
  const double vel = applyDiffLimitFilter(p.vel, prev_ctrl_cmd_.vel, dt, p.acc);
  const double acc = applyDiffLimitFilter(p.acc, prev_ctrl_cmd_.acc, dt, p.jerk);

  ROS_WARN_THROTTLE(3.0, "[Emergency stop] vel: %3.3f, acc: %3.3f", vel, acc);

  return Motion{vel, acc};
}

VelocityController::ControlState VelocityController::updateControlState(
  const ControlState current_control_state, const geometry_msgs::Pose & current_pose,
  const ControlData & control_data)
{
  const double current_vel = control_data.current_motion.vel;
  const double current_acc = control_data.current_motion.acc;
  const double stop_dist = control_data.stop_dist;

  // flags for state transition
  const auto & p = state_transition_params_;

  const bool departure_condition_from_stopping =
    stop_dist > p.drive_state_stop_dist + p.drive_state_offset_stop_dist;
  const bool departure_condition_from_stopped = stop_dist > p.drive_state_stop_dist;

  const bool stopping_condition = stop_dist < p.stopping_state_stop_dist;
  if (
    std::fabs(current_vel) > p.stopped_state_entry_vel ||
    std::fabs(current_acc) > p.stopped_state_entry_acc) {
    last_running_time_ = std::make_shared<ros::Time>(ros::Time::now());
  }
  const bool stopped_condition =
    last_running_time_ ? (ros::Time::now() - *last_running_time_).toSec() > 0.5 : false;

  const bool emergency_condition =
    enable_overshoot_emergency_ && stop_dist < -p.emergency_state_overshoot_stop_dist;

  // transit state
  if (current_control_state == ControlState::DRIVE) {
    if (emergency_condition) {
      return ControlState::EMERGENCY;
    }

    if (enable_smooth_stop_) {
      if (stopping_condition) {
        const double pred_vel_in_target =
          predictedVelocityInTargetPoint(control_data.current_motion, delay_compensation_time_);
        const double pred_stop_dist =
          control_data.stop_dist -
          0.5 * (pred_vel_in_target + current_vel) * delay_compensation_time_;
        smooth_stop_.init(pred_vel_in_target, pred_stop_dist);
        return ControlState::STOPPING;
      }
    } else {
      if (stopped_condition && !departure_condition_from_stopped) {
        return ControlState::STOPPED;
      }
    }
  } else if (current_control_state == ControlState::STOPPING) {
    if (emergency_condition) {
      return ControlState::EMERGENCY;
    }

    if (stopped_condition) {
      return ControlState::STOPPED;
    }

    if (departure_condition_from_stopping) {
      pid_vel_.reset();
      lpf_vel_error_->reset(0.0);
      return ControlState::DRIVE;
    }
  } else if (current_control_state == ControlState::STOPPED) {
    if (departure_condition_from_stopped) {
      pid_vel_.reset();
      lpf_vel_error_->reset(0.0);
      return ControlState::DRIVE;
    }
  } else if (control_state_ == ControlState::EMERGENCY) {
    if (stopped_condition && !emergency_condition) {
      return ControlState::STOPPED;
    }
  }

  return current_control_state;
}

VelocityController::Motion VelocityController::calcCtrlCmd(
  const ControlState & current_control_state, const geometry_msgs::Pose & current_pose,
  const ControlData & control_data)
{
  const size_t closest_idx = control_data.closest_idx;
  const double current_vel = control_data.current_motion.vel;
  const double current_acc = control_data.current_motion.acc;

  // velocity and acceleration command
  Motion ctrl_cmd;
  if (current_control_state == ControlState::DRIVE) {
    // calculate target velocity and acceleration from planning
    const auto closest_interpolated_point = calcInterpolatedTargetValue(
      *trajectory_ptr_, current_pose.position, current_vel, closest_idx);
    const double closest_vel = closest_interpolated_point.twist.linear.x;

    const auto target_pose = velocity_controller_utils::calcPoseAfterTimeDelay(
      current_pose, delay_compensation_time_, current_vel);
    const auto target_interpolated_point =
      calcInterpolatedTargetValue(*trajectory_ptr_, target_pose.position, closest_vel, closest_idx);
    const Motion target_motion =
      Motion{target_interpolated_point.twist.linear.x, target_interpolated_point.accel.linear.x};

    const double pred_vel_in_target =
      predictedVelocityInTargetPoint(control_data.current_motion, delay_compensation_time_);
    debug_values_.setValues(DebugValues::TYPE::PREDICTED_V, pred_vel_in_target);

    ctrl_cmd.vel = target_motion.vel;
    ctrl_cmd.acc = applyVelocityFeedback(target_motion, control_data.dt, pred_vel_in_target);
    ROS_DEBUG(
      "[feedback control]  vel: %3.3f, acc: %3.3f, dt: %3.3f, v_curr: %3.3f, v_ref: %3.3f "
      "feedback_ctrl_cmd.ac: %3.3f",
      ctrl_cmd.vel, ctrl_cmd.acc, control_data.dt, current_vel, target_motion.vel, ctrl_cmd.acc);
  } else if (current_control_state == ControlState::STOPPING) {
    ctrl_cmd.acc = smooth_stop_.calculate(
      control_data.stop_dist, current_vel, current_acc, vel_hist_, delay_compensation_time_);
    ctrl_cmd.vel = stopped_state_params_.vel;

    ROS_DEBUG("[smooth stop]: Smooth stopping. vel: %3.3f, acc: %3.3f", ctrl_cmd.vel, ctrl_cmd.acc);
  } else if (current_control_state == ControlState::STOPPED) {
    ctrl_cmd.vel = stopped_state_params_.vel;
    ctrl_cmd.acc = stopped_state_params_.acc;

    ROS_DEBUG("[Stopped]. vel: %3.3f, acc: %3.3f", ctrl_cmd.vel, ctrl_cmd.acc);
  } else if (current_control_state == ControlState::EMERGENCY) {
    ctrl_cmd = calcEmergencyCtrlCmd(control_data.dt);
  }

  // debug values for vel and acc
  const double filtered_acc_cmd = calcFilteredAcc(ctrl_cmd.acc, control_data);
  return ctrl_cmd;
}

// Do not use closest_idx here
void VelocityController::publishCtrlCmd(const Motion & ctrl_cmd, double current_vel)
{
  // publish control command
  autoware_control_msgs::ControlCommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "base_link";
  cmd.control.velocity = ctrl_cmd.vel;
  cmd.control.acceleration = ctrl_cmd.acc;
  pub_control_cmd_.publish(cmd);

  // store current velocity history
  vel_hist_.push_back({ros::Time::now(), current_vel});
  while (vel_hist_.size() > control_rate_ * 0.5) {
    vel_hist_.erase(vel_hist_.begin());
  }

  prev_ctrl_cmd_ = ctrl_cmd;
}

void VelocityController::publishDebugData(
  const Motion & ctrl_cmd, const ControlData & control_data,
  const geometry_msgs::Pose & current_pose)
{
  // set debug values
  debug_values_.setValues(DebugValues::TYPE::DT, control_data.dt);
  debug_values_.setValues(DebugValues::TYPE::CALCULATED_ACC, control_data.current_motion.acc);
  debug_values_.setValues(DebugValues::TYPE::SHIFT, static_cast<double>(control_data.shift));
  debug_values_.setValues(DebugValues::TYPE::STOP_DIST, control_data.stop_dist);
  debug_values_.setValues(DebugValues::TYPE::CONTROL_STATE, static_cast<double>(control_state_));
  debug_values_.setValues(DebugValues::TYPE::ACC_CMD_PUBLISHED, ctrl_cmd.acc);
  updateDebugVelAcc(ctrl_cmd, current_pose, control_data);

  // publish debug values
  std_msgs::Float32MultiArray debug_msg;
  for (const auto & v : debug_values_.getValues()) {
    debug_msg.data.push_back(v);
  }
  pub_debug_.publish(debug_msg);

  // slope angle
  std_msgs::Float32 slope_msg;
  slope_msg.data = control_data.slope_angle;
  pub_slope_.publish(slope_msg);
}

double VelocityController::getDt()
{
  double dt;
  if (!prev_control_time_) {
    dt = 1.0 / control_rate_;
    prev_control_time_ = std::make_shared<ros::Time>(ros::Time::now());
  } else {
    dt = (ros::Time::now() - *prev_control_time_).toSec();
    *prev_control_time_ = ros::Time::now();
  }
  const double max_dt = 1.0 / control_rate_ * 2.0;
  const double min_dt = 1.0 / control_rate_ * 0.5;
  return std::max(std::min(dt, max_dt), min_dt);
}

VelocityController::Motion VelocityController::getCurrentMotion() const
{
  const double dv = current_vel_ptr_->twist.linear.x - prev_vel_ptr_->twist.linear.x;
  const double dt =
    std::max((current_vel_ptr_->header.stamp - prev_vel_ptr_->header.stamp).toSec(), 1e-03);
  const double accel = dv / dt;

  const double current_vel = current_vel_ptr_->twist.linear.x;
  const double current_acc = lpf_acc_->filter(accel);

  return Motion{current_vel, current_acc};
}

enum VelocityController::Shift VelocityController::getCurrentShift(const size_t closest_idx) const
{
  constexpr double epsilon = 1e-5;

  const double target_vel = trajectory_ptr_->points.at(closest_idx).twist.linear.x;

  if (target_vel > epsilon) {
    return Shift::Forward;
  } else if (target_vel < -epsilon) {
    return Shift::Reverse;
  }

  return prev_shift_;
}

double VelocityController::calcFilteredAcc(const double raw_acc, const ControlData & control_data)
{
  const double acc_max_filtered = applyLimitFilter(raw_acc, max_acc_, min_acc_);
  debug_values_.setValues(DebugValues::TYPE::ACC_CMD_ACC_LIMITED, acc_max_filtered);

  // store ctrl cmd without slope filter
  storeAccelCmd(acc_max_filtered);

  const double acc_slope_filtered =
    applySlopeCompensation(acc_max_filtered, control_data.slope_angle, control_data.shift);
  debug_values_.setValues(DebugValues::TYPE::ACC_CMD_SLOPE_APPLIED, acc_slope_filtered);

  const double acc_jerk_filtered = applyDiffLimitFilter(
    acc_slope_filtered, prev_ctrl_cmd_.acc, control_data.dt, max_jerk_, min_jerk_);
  debug_values_.setValues(DebugValues::TYPE::ACC_CMD_JERK_LIMITED, acc_jerk_filtered);

  return acc_jerk_filtered;
}

void VelocityController::storeAccelCmd(const double accel)
{
  if (control_state_ == ControlState::DRIVE) {
    // convert format
    autoware_control_msgs::ControlCommandStamped cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.control.acceleration = accel;

    // store published ctrl cmd
    ctrl_cmd_vec_.emplace_back(cmd);
  } else {
    //reset command
    ctrl_cmd_vec_.clear();
  }

  // remove unused ctrl cmd
  if (ctrl_cmd_vec_.size() <= 2) {
    return;
  }
  if ((ros::Time::now() - ctrl_cmd_vec_.at(1).header.stamp).toSec() > delay_compensation_time_) {
    ctrl_cmd_vec_.erase(ctrl_cmd_vec_.begin());
  }
}

double VelocityController::applyLimitFilter(
  const double input_val, const double max_val, const double min_val) const
{
  const double limited_val = std::min(std::max(input_val, min_val), max_val);
  return limited_val;
}

double VelocityController::applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double max_val,
  const double min_val) const
{
  const double diff_raw = (input_val - prev_val) / dt;
  const double diff = std::min(std::max(diff_raw, min_val), max_val);
  const double filtered_val = prev_val + diff * dt;
  return filtered_val;
}

double VelocityController::applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double lim_val) const
{
  const double max_val = std::fabs(lim_val);
  const double min_val = -max_val;
  return applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val);
}

double VelocityController::applySlopeCompensation(
  const double input_acc, const double pitch, const Shift shift) const
{
  if (!enable_slope_compensation_) {
    return input_acc;
  }
  const double pitch_limited = std::min(std::max(pitch, min_pitch_rad_), max_pitch_rad_);

  // Acceleration command is always positive independent of direction (= shift) when car is running
  double sign = (shift == Shift::Forward) ? -1 : (shift == Shift::Reverse ? 1 : 0);
  double compensated_acc = input_acc + sign * autoware_utils::gravity * std::sin(pitch_limited);
  return compensated_acc;
}

autoware_planning_msgs::TrajectoryPoint VelocityController::calcInterpolatedTargetValue(
  const autoware_planning_msgs::Trajectory & traj, const geometry_msgs::Point & point,
  const double current_vel, const size_t closest_idx) const
{
  if (traj.points.size() == 1) {
    return traj.points.at(0);
  }

  // If the current position is not within the reference trajectory, enable the edge value.
  // Else, apply linear interpolation
  if (closest_idx == 0) {
    if (autoware_utils::calcSignedArcLength(traj.points, point, 0) > 0) {
      return traj.points.at(0);
    }
  }
  if (closest_idx == traj.points.size() - 1) {
    if (autoware_utils::calcSignedArcLength(traj.points, point, traj.points.size() - 1) < 0) {
      return traj.points.at(traj.points.size() - 1);
    }
  }

  // apply linear interpolation
  return velocity_controller_utils::lerpTrajectoryPoint(traj.points, point);
}

double VelocityController::predictedVelocityInTargetPoint(
  const Motion current_motion, const double delay_compensation_time) const
{
  const double current_vel = current_motion.vel;
  const double current_acc = current_motion.acc;

  if (std::fabs(current_vel) < 1e-01) {  // when velocity is low, no prediction
    return current_vel;
  }

  const double current_vel_abs = std::fabs(current_vel);
  if (ctrl_cmd_vec_.size() == 0) {
    const double pred_vel = current_vel + current_acc * delay_compensation_time;
    // avoid to change sign of current_vel and pred_vel
    return pred_vel > 0 ? std::copysign(pred_vel, current_vel) : 0.0;
  }

  double pred_vel = current_vel_abs;

  const double past_delay_time = ros::Time::now().toSec() - delay_compensation_time;
  for (size_t i = 0; i < ctrl_cmd_vec_.size(); ++i) {
    if ((ros::Time::now() - ctrl_cmd_vec_.at(i).header.stamp).toSec() < delay_compensation_time_) {
      if (i == 0) {
        // size of ctrl_cmd_vec_ is less than delay_compensation_time_
        pred_vel =
          current_vel_abs + ctrl_cmd_vec_.at(i).control.acceleration * delay_compensation_time;
        return pred_vel > 0 ? std::copysign(pred_vel, current_vel) : 0.0;
      }
      // add velocity to accel * dt
      const double acc = ctrl_cmd_vec_.at(i - 1).control.acceleration;
      const double time_to_next_acc = std::min(
        ctrl_cmd_vec_.at(i).header.stamp.toSec() - ctrl_cmd_vec_.at(i - 1).header.stamp.toSec(),
        ctrl_cmd_vec_.at(i).header.stamp.toSec() - past_delay_time);
      pred_vel += acc * time_to_next_acc;
    }
  }

  const double last_acc = ctrl_cmd_vec_.at(ctrl_cmd_vec_.size() - 1).control.acceleration;
  const double time_to_current =
    (ros::Time::now() - ctrl_cmd_vec_.at(ctrl_cmd_vec_.size() - 1).header.stamp).toSec();
  pred_vel += last_acc * time_to_current;

  // avoid to change sign of current_vel and pred_vel
  return pred_vel > 0 ? std::copysign(pred_vel, current_vel) : 0.0;
}

double VelocityController::applyVelocityFeedback(
  const Motion target_motion, const double dt, const double current_vel)
{
  const double current_vel_abs = std::fabs(current_vel);
  const double target_vel_abs = std::fabs(target_motion.vel);
  const bool enable_integration = (current_vel_abs > current_vel_threshold_pid_integrate_);
  const double error_vel_filtered = lpf_vel_error_->filter(target_vel_abs - current_vel_abs);

  std::vector<double> pid_contributions(3);
  const double pid_acc =
    pid_vel_.calculate(error_vel_filtered, dt, enable_integration, pid_contributions);
  const double feedback_acc = target_motion.acc + pid_acc;

  debug_values_.setValues(DebugValues::TYPE::ACC_CMD_PID_APPLIED, feedback_acc);
  debug_values_.setValues(DebugValues::TYPE::ERROR_V_FILTERED, error_vel_filtered);
  debug_values_.setValues(
    DebugValues::TYPE::ACC_CMD_FB_P_CONTRIBUTION, pid_contributions.at(0));  // P
  debug_values_.setValues(
    DebugValues::TYPE::ACC_CMD_FB_I_CONTRIBUTION, pid_contributions.at(1));  // I
  debug_values_.setValues(
    DebugValues::TYPE::ACC_CMD_FB_D_CONTRIBUTION, pid_contributions.at(2));  // D

  return feedback_acc;
}

void VelocityController::updatePitchDebugValues(
  const double pitch, const double traj_pitch, const double raw_pitch)
{
  debug_values_.setValues(DebugValues::TYPE::PITCH_LPF_RAD, pitch);
  debug_values_.setValues(DebugValues::TYPE::PITCH_LPF_DEG, autoware_utils::rad2deg(pitch));
  debug_values_.setValues(DebugValues::TYPE::PITCH_RAW_RAD, raw_pitch);
  debug_values_.setValues(DebugValues::TYPE::PITCH_RAW_DEG, autoware_utils::rad2deg(raw_pitch));
  debug_values_.setValues(DebugValues::TYPE::PITCH_RAW_TRAJ_RAD, traj_pitch);
  debug_values_.setValues(
    DebugValues::TYPE::PITCH_RAW_TRAJ_DEG, autoware_utils::rad2deg(traj_pitch));
}

void VelocityController::updateDebugVelAcc(
  const Motion & ctrl_cmd, const geometry_msgs::Pose & current_pose,
  const ControlData & control_data)
{
  const double current_vel = control_data.current_motion.vel;
  const size_t closest_idx = control_data.closest_idx;

  const auto interpolated_point =
    calcInterpolatedTargetValue(*trajectory_ptr_, current_pose.position, current_vel, closest_idx);

  debug_values_.setValues(DebugValues::TYPE::CURRENT_V, current_vel);
  debug_values_.setValues(DebugValues::TYPE::TARGET_V, ctrl_cmd.vel);
  debug_values_.setValues(DebugValues::TYPE::TARGET_ACC, ctrl_cmd.acc);
  debug_values_.setValues(DebugValues::TYPE::CLOSEST_V, interpolated_point.twist.linear.x);
  debug_values_.setValues(DebugValues::TYPE::CLOSEST_ACC, interpolated_point.accel.linear.x);
  debug_values_.setValues(DebugValues::TYPE::ERROR_V, ctrl_cmd.vel - current_vel);
}
