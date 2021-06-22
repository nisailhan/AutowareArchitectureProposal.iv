/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include <velocity_controller/velocity_controller.h>

VelocityController::VelocityController()
: nh_(""),
  pnh_("~"),
  current_pose_ptr_(nullptr),
  current_vel_ptr_(nullptr),
  prev_vel_ptr_(nullptr),
  trajectory_ptr_(nullptr),
  prev_control_time_(nullptr),
  control_state_(ControlState::STOPPED),
  prev_acc_cmd_(0.0),
  prev_vel_cmd_(0.0),
  last_running_time_(std::make_shared<ros::Time>(ros::Time::now()))
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
    lpf_vel_error_.init(lpf_vel_error_gain);

    pnh_.param(
      "current_vel_threshold_pid_integration", current_vel_threshold_pid_integrate_,
      0.5);  // [m/s]
  }

  // parameters for smooth stop state
  {
    double stop_dist, weak_brake_time, weak_brake_acc, increasing_brake_time,
      increasing_brake_gradient, stop_brake_time, stop_brake_acc;
    pnh_.param("smooth_stop_weak_brake_time", weak_brake_time, 3.0);              // [sec]
    pnh_.param("smooth_stop_weak_brake_acc", weak_brake_acc, -0.4);               // [m/s^2]
    pnh_.param("smooth_stop_increasing_brake_time", increasing_brake_time, 3.0);  // [sec]
    pnh_.param(
      "smooth_stop_increasing_brake_gradient", increasing_brake_gradient, -0.05);  // [m/s^3]
    pnh_.param("smooth_stop_stop_brake_time", stop_brake_time, 2.0);               // [sec]
    pnh_.param("smooth_stop_stop_brake_acc", stop_brake_acc, -1.7);                // [m/s^2]

    smooth_stop_.setParams(
      weak_brake_time, weak_brake_acc, increasing_brake_time, increasing_brake_gradient,
      stop_brake_time, stop_brake_acc);
  }

  // parameters for stop state
  {
    pnh_.param("stopped_vel", stopped_vel_, 0.0);   // [m/s]
    pnh_.param("stopped_acc", stopped_acc_, -2.0);  // [m/s^2]
  }

  // parameters for emergency state
  {
    pnh_.param("emergency_vel", emergency_vel_, 0.0);     // [m/s]
    pnh_.param("emergency_acc", emergency_acc_, -2.0);    // [m/s^2]
    pnh_.param("emergency_jerk", emergency_jerk_, -1.5);  // [m/s^3]
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
  lpf_pitch_.init(lpf_pitch_gain);
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
  lpf_acc_.init(0.2);

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
  if (!isValidTrajectory(*msg)) {
    ROS_ERROR("[velocity_controller] received invalid trajectory. ignore.");
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
  lpf_vel_error_.init(config.lpf_vel_error_gain);
  current_vel_threshold_pid_integrate_ = config.current_velocity_threshold_pid_integration;

  // stopping state
  smooth_stop_.setParams(
    config.smooth_stop_weak_brake_time, config.smooth_stop_weak_brake_acc,
    config.smooth_stop_increasing_brake_time, config.smooth_stop_increasing_brake_gradient,
    config.smooth_stop_stop_brake_time, config.smooth_stop_stop_brake_acc);

  // stop state
  stopped_vel_ = config.stopped_vel;
  stopped_acc_ = config.stopped_acc;

  // emergency state
  emergency_vel_ = config.emergency_vel;
  emergency_acc_ = config.emergency_acc;
  emergency_jerk_ = config.emergency_jerk;

  // acceleration limit
  max_acc_ = config.max_acc;
  min_acc_ = config.min_acc;

  // jerk limit
  max_jerk_ = config.max_jerk;
  min_jerk_ = config.min_jerk;

  // slope compensation
  lpf_pitch_.init(config.lpf_pitch_gain);
  max_pitch_rad_ = config.max_pitch_rad;
  min_pitch_rad_ = config.min_pitch_rad;
}

void VelocityController::callbackTimerControl(const ros::TimerEvent & event)
{
  // wait for initial pointers
  if (!current_vel_ptr_ || !prev_vel_ptr_ || !trajectory_ptr_) {
    return;
  }

  // calculate current pose
  current_pose_ptr_ =
    std::make_shared<geometry_msgs::PoseStamped>(*self_pose_listener_.getCurrentPose());
  const geometry_msgs::Pose current_pose = current_pose_ptr_->pose;

  // calculate current velocity and acceleration
  double current_vel = current_vel_ptr_->twist.linear.x;
  const double dv = current_vel_ptr_->twist.linear.x - prev_vel_ptr_->twist.linear.x;
  const double dt =
    std::max((current_vel_ptr_->header.stamp - prev_vel_ptr_->header.stamp).toSec(), 1e-03);
  const double accel = dv / dt;
  double current_acc = lpf_acc_.filter(accel);
  debug_values_.setValues(DebugValues::TYPE::CALCULATED_ACC, current_acc);

  // calculate closest index of trajectory
  const auto & p = state_transition_params_;
  boost::optional<int> closest_idx = vcutils::calcClosestWithThr(
    *trajectory_ptr_, current_pose, p.emergency_state_traj_rot_dev,
    p.emergency_state_traj_trans_dev);
  boost::optional<double> stop_dist =
    closest_idx ? vcutils::calcStopDistance(current_pose, *trajectory_ptr_) : boost::none;

  // calculate control state
  updateControlState(current_vel, current_acc, stop_dist, closest_idx);
  debug_values_.setValues(DebugValues::TYPE::CTRL_MODE, static_cast<double>(control_state_));

  // calculate command velocity and acceleration
  const CtrlCmd ctrl_cmd = calcCtrlCmd(current_vel, current_acc, *closest_idx);
  debug_values_.setValues(DebugValues::TYPE::ACCCMD_PUBLISHED, ctrl_cmd.acc);

  // publish control command
  publishCtrlCmd(ctrl_cmd.vel, ctrl_cmd.acc);

  prev_acc_cmd_ = ctrl_cmd.acc;
  prev_vel_cmd_ = ctrl_cmd.vel;
}

void VelocityController::updateControlState(
  const double current_vel, const double current_acc, const boost::optional<double> & stop_dist,
  const boost::optional<int> & closest_idx)
{
  const auto & p = state_transition_params_;

  // flags for state transition
  bool departure_condition_from_stopping =
    stop_dist ? *stop_dist > p.drive_state_stop_dist + p.drive_state_offset_stop_dist : false;
  bool departure_condition_from_stopped = stop_dist ? *stop_dist > p.drive_state_stop_dist : false;
  bool stopping_condition = stop_dist ? *stop_dist < p.stopping_state_stop_dist : false;
  if (
    std::fabs(current_vel) > p.stopped_state_entry_vel ||
    std::fabs(current_acc) > p.stopped_state_entry_acc) {
    last_running_time_ = std::make_shared<ros::Time>(ros::Time::now());
  }
  bool stopped_condition =
    last_running_time_ ? (ros::Time::now() - *last_running_time_).toSec() > 0.5 : false;
  bool emergency_condition =
    !closest_idx || (enable_overshoot_emergency_ &&
                     (!stop_dist || *stop_dist < -p.emergency_state_overshoot_stop_dist));

  // transit state
  if (control_state_ == ControlState::DRIVE) {
    if (enable_smooth_stop_) {
      if (stopping_condition) {
        smooth_stop_.initTime(ros::Time::now());
        control_state_ = ControlState::STOPPING;
      }
    } else {
      if (stopped_condition && !departure_condition_from_stopped) {
        control_state_ = ControlState::STOPPED;
      }
    }

    if (emergency_condition) {
      control_state_ = ControlState::EMERGENCY;
    }
  } else if (control_state_ == ControlState::STOPPING) {
    if (departure_condition_from_stopping) {
      control_state_ = ControlState::DRIVE;
      pid_vel_.reset();
      lpf_vel_error_.reset();
    }

    if (stopped_condition) {
      control_state_ = ControlState::STOPPED;
    }

    if (emergency_condition) {
      control_state_ = ControlState::EMERGENCY;
    }
  } else if (control_state_ == ControlState::STOPPED) {
    if (departure_condition_from_stopped) {
      control_state_ = ControlState::DRIVE;
      pid_vel_.reset();
      lpf_vel_error_.reset();
    }

    if (emergency_condition) {
      control_state_ = ControlState::EMERGENCY;
    }
  } else if (control_state_ == ControlState::EMERGENCY) {
    if (stopped_condition && !emergency_condition) {
      control_state_ = ControlState::STOPPED;
    }
  }
}

VelocityController::CtrlCmd VelocityController::calcCtrlCmd(
  const double current_vel, const double current_acc, const int closest_idx)
{
  // dt
  const double dt = getDt();
  debug_values_.setValues(DebugValues::TYPE::DT, dt);

  // velocity and acceleration command
  double vel_cmd, acc_cmd;
  if (control_state_ == ControlState::DRIVE) {
    // calculate target velocity and acceleration from planning
    const double closest_vel = calcInterpolatedTargetValue(
      *trajectory_ptr_, *current_pose_ptr_, current_vel, closest_idx, "twist");
    const double closest_acc = calcInterpolatedTargetValue(
      *trajectory_ptr_, *current_pose_ptr_, current_vel, closest_idx, "accel");
    int target_idx = DelayCompensator::getTrajectoryPointIndexAfterTimeDelay(
      *trajectory_ptr_, closest_idx, delay_compensation_time_, current_vel);
    const geometry_msgs::PoseStamped target_pose = DelayCompensator::calcPoseAfterTimeDelay(
      *current_pose_ptr_, delay_compensation_time_, current_vel);
    double target_vel =
      calcInterpolatedTargetValue(*trajectory_ptr_, target_pose, closest_vel, target_idx, "twist");
    double target_acc =
      calcInterpolatedTargetValue(*trajectory_ptr_, target_pose, closest_vel, target_idx, "accel");

    const double pred_vel_in_target =
      predictedVelocityInTargetPoint(current_vel, current_acc, delay_compensation_time_);
    debug_values_.setValues(DebugValues::TYPE::PREDICTED_V, pred_vel_in_target);

    vel_cmd = target_vel;
    acc_cmd = applyVelocityFeedback(target_acc, target_vel, dt, pred_vel_in_target);
    ROS_DEBUG(
      "[feedback control]  vel: %3.3f, acc: %3.3f, dt: %3.3f, v_curr: %3.3f, v_ref: %3.3f "
      "feedback_acc_cmd: %3.3f",
      vel_cmd, acc_cmd, dt, current_vel, target_vel, acc_cmd);
  } else if (control_state_ == ControlState::STOPPING) {
    acc_cmd = smooth_stop_.calculate();
    vel_cmd = stopped_vel_;

    ROS_DEBUG("[smooth stop]: Smooth stopping. vel: %3.3f, acc: %3.3f", vel_cmd, acc_cmd);
  } else if (control_state_ == ControlState::STOPPED) {
    vel_cmd = stopped_vel_;
    acc_cmd = stopped_acc_;

    ROS_DEBUG("[Stopped]. vel: %3.3f, acc: %3.3f", vel_cmd, acc_cmd);
  } else if (control_state_ == ControlState::EMERGENCY) {
    vel_cmd = applyRateFilter(emergency_vel_, prev_vel_cmd_, dt, emergency_acc_);
    acc_cmd = applyRateFilter(emergency_acc_, prev_acc_cmd_, dt, emergency_jerk_);

    ROS_ERROR_THROTTLE(2.0, "[Emergency stop] vel: %3.3f, acc: %3.3f", vel_cmd, acc_cmd);
  }

  // shift
  const Shift shift = getCurrentShift(vel_cmd);
  if (shift != prev_shift_) pid_vel_.reset();
  prev_shift_ = shift;
  debug_values_.setValues(DebugValues::TYPE::SHIFT, static_cast<double>(shift));

  // pitch
  double pitch = 0.0;
  {
    // TODO(Horibe): closest_idx is indefinite value in EMERGENCY state.
    if (control_state_ != ControlState::EMERGENCY) {
      const double traj_pitch = getPitchByTraj(*trajectory_ptr_, closest_idx);
      const double raw_pitch = getPitchByPose(current_pose_ptr_->pose.orientation);
      pitch = use_traj_for_pitch_ ? traj_pitch : lpf_pitch_.filter(raw_pitch);
      updatePitchDebugValues(pitch, traj_pitch, raw_pitch);
    }

    std_msgs::Float32 msg;
    msg.data = pitch;
    pub_slope_.publish(msg);
  }

  // debug values for vel and acc
  updateDebugVelAcc(current_vel, vel_cmd, acc_cmd, closest_idx);

  if (control_state_ != ControlState::EMERGENCY) {
    double filtered_acc_cmd = calcFilteredAcc(acc_cmd, pitch, dt, shift);
    return CtrlCmd{vel_cmd, filtered_acc_cmd};
  } else {
    return CtrlCmd{vel_cmd, acc_cmd};
  }
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

void VelocityController::publishCtrlCmd(const double vel, const double acc)
{
  // publish control command
  autoware_control_msgs::ControlCommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "base_link";
  cmd.control.velocity = vel;
  cmd.control.acceleration = acc;
  pub_control_cmd_.publish(cmd);

  // publish debug values
  std_msgs::Float32MultiArray msg;
  for (const auto & v : debug_values_.getValues()) {
    msg.data.push_back(v);
  }
  pub_debug_.publish(msg);
}

bool VelocityController::isValidTrajectory(const autoware_planning_msgs::Trajectory & traj) const
{
  for (const auto & points : traj.points) {
    const auto & p = points.pose.position;
    const auto & o = points.pose.orientation;
    const auto & t = points.twist.linear;
    const auto & a = points.accel.linear;
    if (
      !isfinite(p.x) || !isfinite(p.y) || !isfinite(p.z) || !isfinite(o.x) || !isfinite(o.y) ||
      !isfinite(o.z) || !isfinite(o.w) || !isfinite(t.x) || !isfinite(t.y) || !isfinite(t.z) ||
      !isfinite(a.x) || !isfinite(a.y) || !isfinite(a.z)) {
      return false;
    }
  }
  return true;
}

double VelocityController::calcFilteredAcc(
  const double raw_acc, const double pitch, const double dt, const Shift shift)
{
  double acc_max_filtered = applyLimitFilter(raw_acc, max_acc_, min_acc_);
  debug_values_.setValues(DebugValues::TYPE::ACCCMD_ACC_LIMITED, acc_max_filtered);

  // store ctrl cmd without slope filter
  storeAccelCmd(acc_max_filtered);

  double acc_slope_filtered = applySlopeCompensation(acc_max_filtered, pitch, shift);
  debug_values_.setValues(DebugValues::TYPE::ACCCMD_SLOPE_APPLIED, acc_slope_filtered);

  double acc_jerk_filtered =
    applyRateFilter(acc_slope_filtered, prev_acc_cmd_, dt, max_jerk_, min_jerk_);
  debug_values_.setValues(DebugValues::TYPE::ACCCMD_JERK_LIMITED, acc_jerk_filtered);

  return acc_jerk_filtered;
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

enum VelocityController::Shift VelocityController::getCurrentShift(const double target_vel) const
{
  const double ep = 1.0e-5;
  return target_vel > ep ? Shift::Forward : (target_vel < -ep ? Shift::Reverse : prev_shift_);
}

double VelocityController::getPitchByPose(const geometry_msgs::Quaternion & quaternion) const
{
  Eigen::Quaterniond q(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
  Eigen::Vector3d v = q.toRotationMatrix() * Eigen::Vector3d::UnitX();
  double den = std::max(std::sqrt(v.x() * v.x() + v.y() * v.y()), 1.0E-8 /* avoid 0 divide */);
  double pitch = (-1.0) * std::atan2(v.z(), den);
  return pitch;
}

double VelocityController::getPitchByTraj(
  const autoware_planning_msgs::Trajectory & msg, const int32_t closest) const
{
  if (msg.points.size() <= 1 || closest < 0) {
    // cannot calculate pitch
    return 0.0;
  }

  for (int i = closest + 1; i < msg.points.size(); i++) {
    const double dist =
      autoware_utils::calcDistance2d(msg.points.at(closest).pose, msg.points.at(i).pose);
    if (dist > wheel_base_) {
      // closest: rear wheel, i: front wheel
      return vcutils::calcPitch(msg.points.at(closest).pose, msg.points.at(i).pose);
    }
  }

  // close to goal (end of trajectory)
  for (int i = msg.points.size() - 1; i > 0; i--) {
    const double dist =
      autoware_utils::calcDistance2d(msg.points.back().pose, msg.points.at(i).pose);

    if (dist > wheel_base_) {
      // i: rear wheel, msg.points.size()-1: front wheel
      return vcutils::calcPitch(msg.points.at(i).pose, msg.points.back().pose);
    }
  }

  // use full trajectory for calculate pitch
  return vcutils::calcPitch(msg.points.at(0).pose, msg.points.back().pose);
}

double VelocityController::predictedVelocityInTargetPoint(
  const double current_vel, const double current_acc, const double delay_compensation_time) const
{
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
  for (int i = 0; i < ctrl_cmd_vec_.size(); i++) {
    if ((ros::Time::now() - ctrl_cmd_vec_.at(i).header.stamp).toSec() < delay_compensation_time_) {
      if (i == 0) {
        // lack of data
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

double VelocityController::getPointValue(
  const autoware_planning_msgs::TrajectoryPoint & point, const std::string & value_type) const
{
  if (value_type == "twist") {
    return point.twist.linear.x;
  } else if (value_type == "accel") {
    return point.accel.linear.x;
  }

  ROS_WARN_STREAM("value_type in VelocityController::getPointValue is invalid.");
  return 0.0;
}

double VelocityController::calcInterpolatedTargetValue(
  const autoware_planning_msgs::Trajectory & traj, const geometry_msgs::PoseStamped & curr_pose,
  const double current_vel, const int closest, const std::string & value_type) const
{
  const double closest_value = getPointValue(traj.points.at(closest), value_type);

  if (traj.points.size() < 2) {
    return closest_value;
  }

  /* If the current position is at the edge of the reference trajectory, enable the edge value(vel/acc).
   * Else, calc secondary closest index for interpolation */
  int closest_second;
  const auto & closest_pos = traj.points.at(closest).pose;
  geometry_msgs::Point rel_pos =
    vcutils::transformToRelativeCoordinate2D(curr_pose.pose.position, closest_pos);
  if (closest == 0) {
    if (rel_pos.x * current_vel <= 0.0) {
      return closest_value;
    }
    closest_second = 1;
  } else if (closest == static_cast<int>(traj.points.size()) - 1) {
    if (rel_pos.x * current_vel >= 0.0) {
      return closest_value;
    }
    closest_second = traj.points.size() - 2;
  } else {
    const double dist1 =
      autoware_utils::calcDistance2d(closest_pos, traj.points.at(closest - 1).pose);
    const double dist2 =
      autoware_utils::calcDistance2d(closest_pos, traj.points.at(closest + 1).pose);
    closest_second = dist1 < dist2 ? closest - 1 : closest + 1;
  }

  /* apply linear interpolation */
  const double dist_c1 = autoware_utils::calcDistance2d(curr_pose.pose, closest_pos);
  const double dist_c2 =
    autoware_utils::calcDistance2d(curr_pose.pose, traj.points.at(closest_second).pose);
  const double v1 = getPointValue(traj.points.at(closest), value_type);
  const double v2 = getPointValue(traj.points.at(closest_second), value_type);
  const double value_interp = (dist_c1 * v2 + dist_c2 * v1) / (dist_c1 + dist_c2);

  return value_interp;
}

double VelocityController::applyLimitFilter(
  const double input_val, const double max_val, const double min_val) const
{
  const double limited_val = std::min(std::max(input_val, min_val), max_val);
  return limited_val;
}

double VelocityController::applyRateFilter(
  const double input_val, const double prev_val, const double dt, const double max_val,
  const double min_val) const
{
  const double diff_raw = (input_val - prev_val) / dt;
  const double diff = std::min(std::max(diff_raw, min_val), max_val);
  const double filtered_val = prev_val + (diff * dt);
  return filtered_val;
}

double VelocityController::applyRateFilter(
  const double input_val, const double prev_val, const double dt, const double lim_val) const
{
  const double max_val = std::fabs(lim_val);
  const double min_val = -max_val;
  return applyRateFilter(input_val, prev_val, dt, max_val, min_val);
}
double VelocityController::applySlopeCompensation(
  const double input_acc, const double pitch, const Shift shift) const
{
  if (!enable_slope_compensation_) {
    return input_acc;
  }
  constexpr double gravity = 9.80665;
  const double pitch_limited = std::min(std::max(pitch, min_pitch_rad_), max_pitch_rad_);
  double sign = (shift == Shift::Forward) ? -1 : (shift == Shift::Reverse ? 1 : 0);
  double compensated_acc = input_acc + sign * gravity * std::sin(pitch_limited);
  return compensated_acc;
}

double VelocityController::applyVelocityFeedback(
  const double target_acc, const double target_vel, const double dt, const double current_vel)
{
  const double current_vel_abs = std::fabs(current_vel);
  const double target_vel_abs = std::fabs(target_vel);
  const bool enable_integration = (current_vel_abs > current_vel_threshold_pid_integrate_);
  const double error_vel_filtered = lpf_vel_error_.filter(target_vel_abs - current_vel_abs);

  std::vector<double> pid_contributions(3);
  const double pid_acc =
    pid_vel_.calculate(error_vel_filtered, dt, enable_integration, pid_contributions);
  const double feedback_acc = target_acc + pid_acc;

  debug_values_.setValues(DebugValues::TYPE::ACCCMD_PID_APPLIED, feedback_acc);
  debug_values_.setValues(DebugValues::TYPE::ERROR_V_FILTERED, error_vel_filtered);
  debug_values_.setValues(
    DebugValues::TYPE::ACCCMD_FB_P_CONTRIBUTION, pid_contributions.at(0));  // P
  debug_values_.setValues(
    DebugValues::TYPE::ACCCMD_FB_I_CONTRIBUTION, pid_contributions.at(1));  // I
  debug_values_.setValues(
    DebugValues::TYPE::ACCCMD_FB_D_CONTRIBUTION, pid_contributions.at(2));  // D

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
  const double current_vel, const double target_vel, const double target_acc, int closest_idx)
{
  if (closest_idx < 0) return;

  debug_values_.setValues(DebugValues::TYPE::CURR_V, current_vel);
  debug_values_.setValues(DebugValues::TYPE::TARGET_V, target_vel);
  debug_values_.setValues(DebugValues::TYPE::TARGET_ACC, target_acc);
  debug_values_.setValues(
    DebugValues::TYPE::CLOSEST_V,
    calcInterpolatedTargetValue(
      *trajectory_ptr_, *current_pose_ptr_, current_vel, closest_idx, "twist"));
  debug_values_.setValues(
    DebugValues::TYPE::CLOSEST_ACC,
    calcInterpolatedTargetValue(
      *trajectory_ptr_, *current_pose_ptr_, current_vel, closest_idx, "accel"));
  debug_values_.setValues(DebugValues::TYPE::ERROR_V, target_vel - current_vel);
}
