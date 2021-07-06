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

#include "motion_velocity_smoother/motion_velocity_smoother.hpp"
#include <chrono>
#include "motion_velocity_smoother/smoother/jerk_filtered_smoother.hpp"
#include "motion_velocity_smoother/smoother/l2_pseudo_jerk_smoother.hpp"
#include "motion_velocity_smoother/smoother/linf_pseudo_jerk_smoother.hpp"

// clang-format on
namespace motion_velocity_smoother
{
MotionVelocitySmoother::MotionVelocitySmoother() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  // set common params
  node_param_ = getCommonParam();
  pnh_.param<double>(
    "over_stop_velocity_warn_thr", over_stop_velocity_warn_thr_, autoware_utils::kmph2mps(5.0));

  // create smoother
  SmootherBase::BaseParam base_param = getSmootherBaseParam();
  switch (node_param_.algorithm_type) {
    case AlgorithmType::JERK_FILTERED: {
      JerkFilteredSmoother::Param param = getJerkFilteredSmootherParam();
      smoother_ = std::make_shared<JerkFilteredSmoother>(param);
      smoother_->setParam(base_param);

      // Set Publisher for jerk filtered algorithm
      pub_forward_filtered_trajectory_ =
        pnh_.advertise<autoware_planning_msgs::Trajectory>("debug/forward_filtered_trajectory", 1);
      pub_backward_filtered_trajectory_ =
        pnh_.advertise<autoware_planning_msgs::Trajectory>("debug/backward_filtered_trajectory", 1);
      pub_merged_filtered_trajectory_ =
        pnh_.advertise<autoware_planning_msgs::Trajectory>("debug/merged_filtered_trajectory", 1);
      pub_closest_merged_velocity_ =
        pnh_.advertise<std_msgs::Float32>("closest_merged_velocity", 1);
      break;
    }
    case AlgorithmType::L2: {
      L2PseudoJerkSmoother::Param param = getL2PseudoJerkSmootherParam();
      smoother_ = std::make_shared<L2PseudoJerkSmoother>(param);
      smoother_->setParam(base_param);
      break;
    }
    case AlgorithmType::LINF: {
      LinfPseudoJerkSmoother::Param param = getLinfPseudoJerkSmootherParam();
      smoother_ = std::make_shared<LinfPseudoJerkSmoother>(param);
      smoother_->setParam(base_param);
      break;
    }
    default:
      throw std::domain_error("[MotionVelocitySmoother] invalid algorithm");
  }

  // publishers, subscribers
  pub_trajectory_ = pnh_.advertise<autoware_planning_msgs::Trajectory>("output/trajectory", 1);
  pub_velocity_limit_ =
    pnh_.advertise<std_msgs::Float32>("output/current_velocity_limit_mps", 1, true);
  pub_dist_to_stopline_ = pnh_.advertise<std_msgs::Float32>("distance_to_stopline", 1);
  pub_over_stop_velocity_ = pnh_.advertise<std_msgs::Bool>("stop_speed_exceeded", 1);
  sub_current_trajectory_ =
    pnh_.subscribe("input/trajectory", 1, &MotionVelocitySmoother::onCurrentTrajectory, this);
  sub_current_velocity_ =
    pnh_.subscribe("/localization/twist", 1, &MotionVelocitySmoother::onCurrentVelocity, this);
  sub_external_velocity_limit_ = pnh_.subscribe(
    "input/external_velocity_limit_mps", 1, &MotionVelocitySmoother::onExternalVelocityLimit, this);

  // dynamic reconfigure
  dynamic_reconfigure_server_.setCallback(
    boost::bind(&MotionVelocitySmoother::onDynamicReconfigure, this, _1, _2));

  // debug
  pnh_.param<bool>("publish_debug_trajs", publish_debug_trajs_, false);
  debug_closest_velocity_ = pnh_.advertise<std_msgs::Float32>("closest_velocity", 1);
  debug_closest_acc_ = pnh_.advertise<std_msgs::Float32>("closest_acceleration", 1);
  debug_closest_jerk_ = pnh_.advertise<std_msgs::Float32>("closest_jerk", 1);
  debug_closest_max_velocity_ = pnh_.advertise<std_msgs::Float32>("closest_max_velocity", 1);
  debug_calculation_time_ = pnh_.advertise<std_msgs::Float32>("calculation_time", 1);
  pub_trajectory_raw_ =
    pnh_.advertise<autoware_planning_msgs::Trajectory>("debug/trajectory_raw", 1);
  pub_trajectory_vel_lim_ = pnh_.advertise<autoware_planning_msgs::Trajectory>(
    "debug/trajectory_external_velocity_limited", 1);
  pub_trajectory_latacc_filtered_ =
    pnh_.advertise<autoware_planning_msgs::Trajectory>("debug/trajectory_lateral_acc_filtered", 1);
  pub_trajectory_resampled_ =
    pnh_.advertise<autoware_planning_msgs::Trajectory>("debug/trajectory_time_resampled", 1);

  // Wait for first self pose
  self_pose_listener_.waitForFirstPose();

  external_velocity_limit_ = node_param_.max_velocity;
  max_velocity_with_deceleration_ = node_param_.max_velocity;

  // publish default max velocity
  std_msgs::Float32 max_vel_msg;
  max_vel_msg.data = node_param_.max_velocity;
  pub_velocity_limit_.publish(max_vel_msg);
}

void MotionVelocitySmoother::onDynamicReconfigure(
  motion_velocity_smoother::MotionVelocitySmootherConfig & config, uint32_t level)
{
  node_param_.max_velocity = config.max_velocity;
  node_param_.margin_to_insert_external_velocity_limit =
    config.margin_to_insert_external_velocity_limit;
  node_param_.replan_vel_deviation = config.replan_vel_deviation;
  node_param_.engage_velocity = config.engage_velocity;
  node_param_.engage_acceleration = config.engage_acceleration;
  node_param_.engage_exit_ratio = config.engage_exit_ratio;
  node_param_.stopping_velocity = config.stopping_velocity;
  node_param_.stopping_distance = config.stopping_distance;
  node_param_.extract_ahead_dist = config.extract_ahead_dist;
  node_param_.extract_behind_dist = config.extract_behind_dist;
  node_param_.stop_dist_to_prohibit_engage = config.stop_dist_to_prohibit_engage;
  node_param_.delta_yaw_threshold = config.delta_yaw_threshold;

  {
    SmootherBase::BaseParam p;
    p.max_accel = config.max_accel;
    p.min_decel = config.min_decel;
    p.max_jerk = config.max_jerk;
    p.min_jerk = config.min_jerk;
    p.max_lateral_accel = config.max_lateral_accel;
    p.min_curve_velocity = config.min_curve_velocity;
    p.decel_distance_before_curve = config.decel_distance_before_curve;
    p.decel_distance_after_curve = config.decel_distance_after_curve;
    p.resample_param.max_trajectory_length = config.max_trajectory_length;
    p.resample_param.min_trajectory_length = config.min_trajectory_length;
    p.resample_param.resample_time = config.resample_time;
    p.resample_param.dense_resample_dt = config.dense_resample_dt;
    p.resample_param.dense_min_interval_distance = config.dense_min_interval_distance;
    p.resample_param.sparse_resample_dt = config.sparse_resample_dt;
    p.resample_param.sparse_min_interval_distance = config.sparse_min_interval_distance;
    smoother_->setParam(p);
  }

  switch (node_param_.algorithm_type) {
    case AlgorithmType::JERK_FILTERED: {
      JerkFilteredSmoother::Param p;
      p.jerk_weight = config.jerk_weight;
      p.over_v_weight = config.over_v_weight;
      p.over_a_weight = config.over_a_weight;
      p.over_j_weight = config.over_j_weight;
      std::dynamic_pointer_cast<JerkFilteredSmoother>(smoother_)->setParam(p);
      break;
    }
    case AlgorithmType::L2: {
      L2PseudoJerkSmoother::Param p;
      p.pseudo_jerk_weight = config.pseudo_jerk_weight;
      p.over_v_weight = config.over_v_weight;
      p.over_a_weight = config.over_a_weight;
      std::dynamic_pointer_cast<L2PseudoJerkSmoother>(smoother_)->setParam(p);
      break;
    }
    case AlgorithmType::LINF: {
      LinfPseudoJerkSmoother::Param p;
      p.pseudo_jerk_weight = config.pseudo_jerk_weight;
      p.over_v_weight = config.over_v_weight;
      p.over_a_weight = config.over_a_weight;
      std::dynamic_pointer_cast<LinfPseudoJerkSmoother>(smoother_)->setParam(p);
      break;
    }
    default:
      throw std::domain_error("[MotionVelocitySmoother] invalid algorithm");
  }
}

MotionVelocitySmoother::Param MotionVelocitySmoother::getCommonParam() const
{
  Param p;
  pnh_.param<double>("max_velocity", p.max_velocity, 20.0);  // 72.0 kmph
  pnh_.param<double>(
    "margin_to_insert_external_velocity_limit", p.margin_to_insert_external_velocity_limit, 0.3);
  pnh_.param<double>("replan_vel_deviation", p.replan_vel_deviation, 3.0);
  pnh_.param<double>("engage_velocity", p.engage_velocity, 0.3);
  pnh_.param<double>("engage_acceleration", p.engage_acceleration, 0.1);
  pnh_.param<double>("engage_exit_ratio", p.engage_exit_ratio, 0.5);
  p.engage_exit_ratio = std::min(std::max(p.engage_exit_ratio, 0.0), 1.0);
  pnh_.param<double>("stopping_velocity", p.stopping_velocity, autoware_utils::kmph2mps(10.0));
  pnh_.param<double>("stopping_distance", p.stopping_distance, 0.0);
  pnh_.param<double>("extract_ahead_dist", p.extract_ahead_dist, 200.0);
  pnh_.param<double>("extract_behind_dist", p.extract_behind_dist, 3.0);
  pnh_.param<double>("stop_dist_to_prohibit_engage", p.stop_dist_to_prohibit_engage, 1.5);
  pnh_.param<double>("delta_yaw_threshold", p.delta_yaw_threshold, M_PI / 3.0);
  pnh_.param<double>(
    "post_max_trajectory_length", p.post_resample_param.max_trajectory_length, 300.0);
  pnh_.param<double>(
    "post_min_trajectory_length", p.post_resample_param.min_trajectory_length, 30.0);
  pnh_.param<double>("post_resample_time", p.post_resample_param.resample_time, 10.0);
  pnh_.param<double>("post_dense_resample_dt", p.post_resample_param.dense_resample_dt, 0.1);
  pnh_.param<double>(
    "post_dense_min_interval_distance", p.post_resample_param.dense_min_interval_distance, 0.1);
  pnh_.param<double>("post_sparse_resample_dt", p.post_resample_param.sparse_resample_dt, 0.1);
  pnh_.param<double>(
    "post_sparse_min_interval_distance", p.post_resample_param.sparse_min_interval_distance, 1.0);

  {
    std::string algorithm_name;
    pnh_.param<std::string>("algorithm_type", algorithm_name, "JerkFiltered");
    p.algorithm_type = getAlgorithmType(algorithm_name);
  }
  return p;
}

SmootherBase::BaseParam MotionVelocitySmoother::getSmootherBaseParam() const
{
  SmootherBase::BaseParam base_param;
  pnh_.param<double>("max_accel", base_param.max_accel, 2.0);   // 0.11G
  pnh_.param<double>("min_decel", base_param.min_decel, -3.0);  // -0.2G
  pnh_.param<double>("max_jerk", base_param.max_jerk, 0.3);
  pnh_.param<double>("min_jerk", base_param.min_jerk, -0.1);
  pnh_.param<double>("max_lateral_accel", base_param.max_lateral_accel, 0.2);
  pnh_.param<double>("decel_distance_before_curve", base_param.decel_distance_before_curve, 3.5);
  pnh_.param<double>("decel_distance_after_curve", base_param.decel_distance_after_curve, 0.0);
  pnh_.param<double>("min_curve_velocity", base_param.min_curve_velocity, 1.38);
  pnh_.param<double>(
    "max_trajectory_length", base_param.resample_param.max_trajectory_length, 200.0);
  pnh_.param<double>(
    "min_trajectory_length", base_param.resample_param.min_trajectory_length, 30.0);
  pnh_.param<double>("resample_time", base_param.resample_param.resample_time, 10.0);
  pnh_.param<double>("dense_resample_dt", base_param.resample_param.dense_resample_dt, 0.1);
  pnh_.param<double>(
    "dense_min_interval_distance", base_param.resample_param.dense_min_interval_distance, 0.1);
  pnh_.param<double>("sparse_resample_dt", base_param.resample_param.sparse_resample_dt, 0.5);
  pnh_.param<double>(
    "sparse_min_interval_distance", base_param.resample_param.sparse_min_interval_distance, 4.0);
  return base_param;
}

JerkFilteredSmoother::Param MotionVelocitySmoother::getJerkFilteredSmootherParam() const
{
  JerkFilteredSmoother::Param param;
  pnh_.param<double>("jerk_weight", param.jerk_weight, 10.0);
  pnh_.param<double>("over_v_weight", param.over_v_weight, 100000.0);
  pnh_.param<double>("over_a_weight", param.over_a_weight, 5000.0);
  pnh_.param<double>("over_j_weight", param.over_j_weight, 2000.0);
  return param;
}

L2PseudoJerkSmoother::Param MotionVelocitySmoother::getL2PseudoJerkSmootherParam() const
{
  L2PseudoJerkSmoother::Param param;
  pnh_.param<double>("pseudo_jerk_weight", param.pseudo_jerk_weight, 100.0);
  pnh_.param<double>("over_v_weight", param.over_v_weight, 100000.0);
  pnh_.param<double>("over_a_weight", param.over_a_weight, 1000.0);
  return param;
}

LinfPseudoJerkSmoother::Param MotionVelocitySmoother::getLinfPseudoJerkSmootherParam() const
{
  LinfPseudoJerkSmoother::Param param;
  pnh_.param<double>("pseudo_jerk_weight", param.pseudo_jerk_weight, 200.0);
  pnh_.param<double>("over_v_weight", param.over_v_weight, 100000.0);
  pnh_.param<double>("over_a_weight", param.over_a_weight, 5000.0);
  return param;
}

void MotionVelocitySmoother::publishTrajectory(
  const autoware_planning_msgs::Trajectory & trajectory) const
{
  pub_trajectory_.publish(trajectory);
}

void MotionVelocitySmoother::onCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr msg)
{
  current_velocity_ptr_ = msg;
}

void MotionVelocitySmoother::onExternalVelocityLimit(const std_msgs::Float32::ConstPtr msg)
{
  constexpr double eps = 1.0E-04;
  const double margin = node_param_.margin_to_insert_external_velocity_limit;

  // calculate distance and maximum velocity to decelerate to external velocity limit with jerk and acceleration
  // constraints
  if (!prev_output_.points.empty()) {
    // if external velocity limit decreases
    if ((external_velocity_limit_ - msg->data) > eps) {
      if (prev_closest_point_) {
        const double v0 = prev_closest_point_->twist.linear.x;
        const double a0 = prev_closest_point_->accel.linear.x;

        if (isEngageStatus(v0)) {
          max_velocity_with_deceleration_ = external_velocity_limit_;
          external_velocity_limit_dist_ = 0.0;
        } else {
          const double a_min = smoother_->getMinDecel();
          const double j_max = smoother_->getMaxJerk();
          const double j_min = smoother_->getMinJerk();
          double stop_dist;
          std::map<double, double> jerk_profile;
          if (trajectory_utils::calcStopDistWithJerkConstraints(
                v0, a0, j_max, j_min, a_min, msg->data, jerk_profile, stop_dist)) {
            external_velocity_limit_dist_ = stop_dist + margin;
          } else {
            external_velocity_limit_dist_ = stop_dist + margin;
          }
          // If the closest acceleration is positive, velocity will increase until the acceleration becomes zero
          // So we set the maximum increased velocity as the velocity limit
          if (a0 > 0)
            max_velocity_with_deceleration_ = v0 - 0.5 * a0 * a0 / j_min;
          else
            max_velocity_with_deceleration_ = v0;

          if (max_velocity_with_deceleration_ < msg->data) {
            max_velocity_with_deceleration_ = msg->data;
            external_velocity_limit_dist_ = 0.0;
          }
        }
      }
      // if external velocity limit increases
    } else if ((msg->data - external_velocity_limit_) > eps) {
      max_velocity_with_deceleration_ = msg->data;
    }
  }

  external_velocity_limit_ = msg->data;
  pub_velocity_limit_.publish(*msg);
}

void MotionVelocitySmoother::onCurrentTrajectory(
  const autoware_planning_msgs::Trajectory::ConstPtr msg)
{
  base_traj_raw_ptr_ = msg;

  const auto t_start = std::chrono::system_clock::now();
  ROS_DEBUG(
    "[MotionVelocitySmoother] ============================== run start "
    "==============================");

  current_pose_ptr_ = self_pose_listener_.getCurrentPose();

  // guard
  if (!current_pose_ptr_ || !current_velocity_ptr_ || !base_traj_raw_ptr_) {
    ROS_DEBUG(
      "[MotionVelocitySmoother] wait topics : current_pose = %d, current_vel = %d, base_traj = %d",
      (bool)current_pose_ptr_, (bool)current_velocity_ptr_, (bool)base_traj_raw_ptr_);
    return;
  }
  if (base_traj_raw_ptr_->points.empty()) {
    ROS_DEBUG("[MotionVelocitySmoother] received trajectory is empty");
    return;
  }

  // calculate distance to insert external velocity limit
  if (!prev_output_.points.empty()) {
    const double travel_dist = calcTravelDistance();
    external_velocity_limit_dist_ -= travel_dist;
    external_velocity_limit_dist_ = std::max(external_velocity_limit_dist_, 0.0);
    ROS_DEBUG(
      "[MotionVelocitySmoother] run: travel_dist = %f, external_velocity_limit_dist_ = %f",
      travel_dist, external_velocity_limit_dist_);
  }

  // calculate trajectory velocity
  autoware_planning_msgs::Trajectory output = calcTrajectoryVelocity(*base_traj_raw_ptr_);
  if (output.points.empty()) {
    ROS_WARN("[MotionVelocitySmoother] Output Point is empty");
    return;
  }

  // Get the nearest point
  const auto output_closest = autoware_utils::findNearestIndex(
    output.points, current_pose_ptr_->pose, node_param_.delta_yaw_threshold);
  const auto output_closest_point =
    trajectory_utils::calcInterpolatedTrajectoryPoint(output, current_pose_ptr_->pose);
  if (!output_closest) {
    ROS_WARN_THROTTLE(5.0, "[MotionVelocitySmoother] Cannot find closest waypoint for output trajectory");
    return;
  }

  // Resample the optimized trajectory
  auto output_resampled = resampling::resampleTrajectory(
    output, current_velocity_ptr_->twist.linear.x, *output_closest,
    node_param_.post_resample_param);
  if (!output_resampled) {
    ROS_WARN("[MotionVelocitySmoother] Failed to get the resampled output trajectory");
    return;
  }

  // Set 0 at the end of the trajectory
  if (!output_resampled->points.empty()) {
    output_resampled->points.back().twist.linear.x = 0.0;
  }

  // publish message
  output_resampled->header = base_traj_raw_ptr_->header;
  publishTrajectory(*output_resampled);

  // publish debug message
  publishStopDistance(output, *output_closest);
  publishClosestState(output_closest_point);

  prev_output_ = output;
  prev_closest_point_ = output_closest_point;

  const auto t_end = std::chrono::system_clock::now();
  const double elapsed =
    std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start).count();

  // Publish Calculation Time
  std_msgs::Float32 calculation_time_data;
  calculation_time_data.data = elapsed * 1.0e-6;
  debug_calculation_time_.publish(calculation_time_data);
  ROS_DEBUG("[MotionVelocitySmoother] run: calculation time = %f [ms]", elapsed * 1.0e-6);
  ROS_DEBUG(
    "[MotionVelocitySmoother] ============================== run() end "
    "==============================\n\n");
}

autoware_planning_msgs::Trajectory MotionVelocitySmoother::calcTrajectoryVelocity(
  const autoware_planning_msgs::Trajectory & traj_input) const
{
  autoware_planning_msgs::Trajectory output;  // velocity is optimized by qp solver

  // Extract trajectory around self-position with desired forward-backward length
  const auto input_closest = autoware_utils::findNearestIndex(
    traj_input.points, current_pose_ptr_->pose, node_param_.delta_yaw_threshold);
  if (!input_closest) {
    ROS_WARN_THROTTLE(5.0, "[MotionVelocitySmoother] Cannot find the closest point from input trajectory");
    return prev_output_;
  }

  auto traj_extracted = trajectory_utils::extractPathAroundIndex(
    traj_input, *input_closest, node_param_.extract_ahead_dist, node_param_.extract_behind_dist);
  if (!traj_extracted) {
    ROS_WARN("[MotionVelocitySmoother] Fail to extract the path from the input trajectory");
    return prev_output_;
  }

  // Smoother can not handle negative velocity, so multiple -1 to velocity if any trajectory points have reverse
  // velocity
  const bool is_reverse = std::any_of(
    traj_extracted->points.begin(), traj_extracted->points.end(),
    [](auto & pt) { return pt.twist.linear.x < 0; });
  if (is_reverse) {
    for (auto & pt : traj_extracted->points) {
      pt.twist.linear.x *= -1.0;
    }
  }

  // Debug
  if (publish_debug_trajs_) pub_trajectory_raw_.publish(*traj_extracted);

  // Apply external velocity limit
  applyExternalVelocityLimit(*traj_extracted);

  // Change trajectory velocity to zero when current_velocity == 0 & stop_dist is close
  const auto traj_extracted_closest = autoware_utils::findNearestIndex(
    traj_extracted->points, current_pose_ptr_->pose, node_param_.delta_yaw_threshold);
  if (!traj_extracted_closest) {
    ROS_WARN("[MotionVelocitySmoother] Cannot find the closest point from extracted trajectory");
    return prev_output_;
  }

  // Apply velocity to approach stop point
  applyStopApproachingVelocity(*traj_extracted);

  // Debug
  if (publish_debug_trajs_) pub_trajectory_vel_lim_.publish(*traj_extracted);

  // Smoothing velocity
  if (!smoothVelocity(*traj_extracted, output)) {
    return prev_output_;
  }

  // Validation
  const auto output_closest = autoware_utils::findNearestIndex(
    output.points, current_pose_ptr_->pose, node_param_.delta_yaw_threshold);
  if (!output_closest) {
    ROS_WARN("[MotionVelocitySmoother] Cannot find the closest point from the output trajectory");
    return prev_output_;
  }

  // for reverse velocity
  if (is_reverse) {
    for (auto & pt : output.points) {
      pt.twist.linear.x *= -1.0;
    }
  }

  return output;
}

bool MotionVelocitySmoother::smoothVelocity(
  const autoware_planning_msgs::Trajectory & input,
  autoware_planning_msgs::Trajectory & traj_smoothed) const
{
  // Lateral acceleration limit
  const auto traj_lateral_acc_filtered = smoother_->applyLateralAccelerationFilter(input);
  if (!traj_lateral_acc_filtered) {
    return false;
  }

  // Resample trajectory with ego-velocity based interval distance
  const auto traj_pre_resampled_closest = autoware_utils::findNearestIndex(
    traj_lateral_acc_filtered->points, current_pose_ptr_->pose, node_param_.delta_yaw_threshold);
  auto traj_resampled = smoother_->resampleTrajectory(
    *traj_lateral_acc_filtered, current_velocity_ptr_->twist.linear.x, *traj_pre_resampled_closest);
  if (!traj_resampled) {
    ROS_WARN("[MotionVelocitySmoother] Fail to do resampling before the optimization");
    return false;
  }

  // Set 0[m/s] in the terminal point
  if (!traj_resampled->points.empty()) {
    traj_resampled->points.back().twist.linear.x = 0.0;
  }

  // Publish Closest Resample Trajectory Velocity
  publishClosestVelocity(*traj_resampled, current_pose_ptr_->pose, debug_closest_max_velocity_);

  // Calculate initial motion for smoothing
  double initial_vel;
  double initial_acc;
  InitializeType type;
  const auto traj_resampled_closest = autoware_utils::findNearestIndex(
    traj_resampled->points, current_pose_ptr_->pose, node_param_.delta_yaw_threshold);
  if (!traj_resampled_closest) {
    ROS_WARN("[MotionVelocitySmoother] Cannot find closest waypoint for resampled trajectory");
    return false;
  }
  std::tie(initial_vel, initial_acc, type) =
    calcInitialMotion(*traj_resampled, *traj_resampled_closest, prev_output_);

  // Clip trajectory from closest point
  autoware_planning_msgs::Trajectory clipped;
  clipped.header = input.header;
  clipped.points.insert(
    clipped.points.end(), traj_resampled->points.begin() + *traj_resampled_closest,
    traj_resampled->points.end());

  std::vector<autoware_planning_msgs::Trajectory> debug_trajectories;
  if (!smoother_->apply(initial_vel, initial_acc, clipped, traj_smoothed, debug_trajectories)) {
    ROS_WARN("[MotionVelocitySmoother] Fail to solve optimization.");
  }

  traj_smoothed.points.insert(
    traj_smoothed.points.begin(), traj_resampled->points.begin(),
    traj_resampled->points.begin() + *traj_resampled_closest);

  if (!debug_trajectories.empty()) {
    for (auto & debug_trajectory : debug_trajectories) {
      debug_trajectory.points.insert(
        debug_trajectory.points.begin(), traj_resampled->points.begin(),
        traj_resampled->points.begin() + *traj_resampled_closest);
      for (size_t i = 0; i < *traj_resampled_closest; ++i) {
        debug_trajectory.points.at(i).twist.linear.x =
          debug_trajectory.points.at(*traj_resampled_closest).twist.linear.x;
      }
    }
  }

  // Set 0 velocity after input-stop-point
  overwriteStopPoint(*traj_resampled, traj_smoothed);

  // For the endpoint of the trajectory
  if (!traj_smoothed.points.empty()) {
    traj_smoothed.points.back().twist.linear.x = 0.0;
  }

  // Max velocity filter for safety
  trajectory_utils::applyMaximumVelocityLimit(
    *traj_resampled_closest, traj_smoothed.points.size(), node_param_.max_velocity, traj_smoothed);

  // Insert behind velocity for output's consistency
  insertBehindVelocity(*traj_resampled_closest, type, traj_smoothed);

  ROS_DEBUG(
    "[MotionVelocitySmoother] smoothVelocity : traj_smoothed.size() = %lu",
    traj_smoothed.points.size());
  if (publish_debug_trajs_) {
    pub_trajectory_latacc_filtered_.publish(*traj_lateral_acc_filtered);
    pub_trajectory_resampled_.publish(*traj_resampled);
    publishDebugTrajectories(debug_trajectories);
  }

  return true;
}

void MotionVelocitySmoother::insertBehindVelocity(
  const size_t output_closest, const InitializeType type,
  autoware_planning_msgs::Trajectory & output) const
{
  const bool keep_closest_vel_for_behind =
    (type == InitializeType::INIT || type == InitializeType::LARGE_DEVIATION_REPLAN ||
     type == InitializeType::ENGAGING);

  for (size_t i = output_closest - 1; i < output.points.size(); --i) {
    if (keep_closest_vel_for_behind) {
      output.points.at(i).twist.linear.x = output.points.at(output_closest).twist.linear.x;
      output.points.at(i).accel.linear.x = output.points.at(output_closest).accel.linear.x;
    } else {
      const auto prev_output_point =
        trajectory_utils::calcInterpolatedTrajectoryPoint(prev_output_, output.points.at(i).pose);
      output.points.at(i).twist.linear.x = prev_output_point.twist.linear.x;
      output.points.at(i).accel.linear.x = prev_output_point.accel.linear.x;
    }
  }
}

void MotionVelocitySmoother::publishStopDistance(
  const autoware_planning_msgs::Trajectory & trajectory, const size_t closest) const
{
  // stop distance calculation
  const double stop_dist_lim = 50.0;
  double stop_dist = stop_dist_lim;
  const auto stop_idx = autoware_utils::searchZeroVelocityIndex(trajectory.points);
  if (stop_idx) {
    stop_dist = trajectory_utils::calcArcLength(trajectory, closest, *stop_idx);
    stop_dist = closest > *stop_idx ? stop_dist : -stop_dist;
  } else {
    stop_dist = closest > 0 ? stop_dist : -stop_dist;
  }
  std_msgs::Float32 dist_to_stopline;
  dist_to_stopline.data = std::max(-stop_dist_lim, std::min(stop_dist_lim, stop_dist));
  pub_dist_to_stopline_.publish(dist_to_stopline);
}

std::tuple<double, double, MotionVelocitySmoother::InitializeType>
MotionVelocitySmoother::calcInitialMotion(
  const autoware_planning_msgs::Trajectory & input_traj, const size_t input_closest,
  const autoware_planning_msgs::Trajectory & prev_traj) const
{
  const double vehicle_speed = std::fabs(current_velocity_ptr_->twist.linear.x);
  const double target_vel = std::fabs(input_traj.points.at(input_closest).twist.linear.x);

  double initial_vel;
  double initial_acc;
  InitializeType type;

  // first time
  if (prev_traj.points.empty()) {
    initial_vel = vehicle_speed;
    initial_acc = 0.0;
    type = InitializeType::INIT;
    return std::make_tuple(initial_vel, initial_acc, type);
  }

  const auto prev_output_closest_point = trajectory_utils::calcInterpolatedTrajectoryPoint(
    prev_traj, input_traj.points.at(input_closest).pose);

  // when velocity tracking deviation is large
  const double desired_vel = prev_output_closest_point.twist.linear.x;
  const double vel_error = vehicle_speed - std::fabs(desired_vel);
  if (std::fabs(vel_error) > node_param_.replan_vel_deviation) {
    type = InitializeType::LARGE_DEVIATION_REPLAN;
    initial_vel = vehicle_speed;  // use current vehicle speed
    initial_acc = prev_output_closest_point.accel.linear.x;
    ROS_DEBUG(
      "[MotionVelocitySmoother] calcInitialMotion : Large deviation error for speed control. Use "
      "current speed for "
      "initial value, desired_vel = %f, vehicle_speed = %f, vel_error = %f, error_thr = %f",
      desired_vel, vehicle_speed, vel_error, node_param_.replan_vel_deviation);
    return std::make_tuple(initial_vel, initial_acc, type);
  }

  // if current vehicle velocity is low && base_desired speed is high, use engage_velocity for engage vehicle
  const double engage_vel_thr = node_param_.engage_velocity * node_param_.engage_exit_ratio;
  if (vehicle_speed < engage_vel_thr) {
    if (target_vel >= node_param_.engage_velocity) {
      const auto idx = autoware_utils::searchZeroVelocityIndex(input_traj.points);
      const double stop_dist =
        idx ? autoware_utils::calcDistance2d(
                input_traj.points.at(*idx), input_traj.points.at(input_closest))
            : 0.0;
      if (!idx || stop_dist > node_param_.stop_dist_to_prohibit_engage) {
        type = InitializeType::ENGAGING;
        initial_vel = node_param_.engage_velocity;
        initial_acc = node_param_.engage_acceleration;
        ROS_DEBUG(
          "[MotionVelocitySmoother] calcInitialMotion : vehicle speed is low (%.3f), and desired "
          "speed is high (%.3f). Use "
          "engage speed (%.3f) until vehicle speed reaches engage_vel_thr (%.3f). stop_dist = %.3f",
          vehicle_speed, target_vel, node_param_.engage_velocity, engage_vel_thr, stop_dist);
        return std::make_tuple(initial_vel, initial_acc, type);
      } else {
        ROS_DEBUG(
          "[MotionVelocitySmoother] calcInitialMotion : stop point is close (%.3f[m]). no engage.",
          stop_dist);
      }
    } else if (target_vel > 0.0) {
      ROS_WARN_THROTTLE(
        3.0,
        "[MotionVelocitySmoother] calcInitialMotion : target velocity(%.3f[m/s]) is lower than "
        "engage "
        "velocity(%.3f[m/s]). ",
        target_vel, node_param_.engage_velocity);
    }
  }

  // normal update: use closest in prev_output
  type = InitializeType::NORMAL;
  initial_vel = prev_output_closest_point.twist.linear.x;
  initial_acc = prev_output_closest_point.accel.linear.x;
  ROS_DEBUG(
    "[MotionVelocitySmoother] calcInitialMotion : normal update. v0 = %f, a0 = %f, vehicle_speed = "
    "%f, target_vel = %f",
    initial_vel, initial_acc, vehicle_speed, target_vel);
  return std::make_tuple(initial_vel, initial_acc, type);
}

void MotionVelocitySmoother::overwriteStopPoint(
  const autoware_planning_msgs::Trajectory & input,
  autoware_planning_msgs::Trajectory & output) const
{
  const auto stop_idx = autoware_utils::searchZeroVelocityIndex(input.points);

  // check over velocity
  bool is_stop_velocity_exceeded = false;
  double input_stop_vel;
  double output_stop_vel;
  if (stop_idx) {
    double optimized_stop_point_vel = output.points.at(*stop_idx).twist.linear.x;
    is_stop_velocity_exceeded = (optimized_stop_point_vel > over_stop_velocity_warn_thr_);
    input_stop_vel = input.points.at(*stop_idx).twist.linear.x;
    output_stop_vel = output.points.at(*stop_idx).twist.linear.x;
    trajectory_utils::applyMaximumVelocityLimit(*stop_idx, output.points.size(), 0.0, output);
    ROS_DEBUG(
      "[MotionVelocitySmoother] replan : input_stop_idx = %lu, stop velocity : input = %f, output "
      "= %f, thr = %f",
      *stop_idx, input_stop_vel, output_stop_vel, over_stop_velocity_warn_thr_);
  } else {
    input_stop_vel = -1.0;
    output_stop_vel = -1.0;
    ROS_DEBUG(
      "[MotionVelocitySmoother] replan : input_stop_idx = -1, stop velocity : input = %f, output = "
      "%f, thr = %f",
      input_stop_vel, output_stop_vel, over_stop_velocity_warn_thr_);
  }

  {
    std_msgs::Bool msg;
    msg.data = is_stop_velocity_exceeded;
    pub_over_stop_velocity_.publish(msg);
  }
}

void MotionVelocitySmoother::applyExternalVelocityLimit(
  autoware_planning_msgs::Trajectory & traj) const
{
  if (traj.points.size() < 1) return;

  trajectory_utils::applyMaximumVelocityLimit(
    0, traj.points.size(), max_velocity_with_deceleration_, traj);

  const auto closest_idx = autoware_utils::findNearestIndex(
    traj.points, current_pose_ptr_->pose, node_param_.delta_yaw_threshold);
  if (!closest_idx) return;

  double dist = 0.0;
  for (size_t idx = *closest_idx; idx < traj.points.size() - 1; ++idx) {
    dist += autoware_utils::calcDistance2d(traj.points.at(idx), traj.points.at(idx + 1));
    if (dist > external_velocity_limit_dist_) {
      trajectory_utils::applyMaximumVelocityLimit(
        idx + 1, traj.points.size(), external_velocity_limit_, traj);
      return;
    }
  }
  traj.points.back().twist.linear.x =
    std::min(traj.points.back().twist.linear.x, external_velocity_limit_);
  ROS_DEBUG(
    "[MotionVelocitySmoother] externalVelocityLimit : limit_vel = %.3f", external_velocity_limit_);
  return;
}

void MotionVelocitySmoother::applyStopApproachingVelocity(
  autoware_planning_msgs::Trajectory & traj) const
{
  const auto stop_idx = autoware_utils::searchZeroVelocityIndex(traj.points);
  if (!stop_idx) return;  // no stop point.

  double distance_sum = 0.0;
  for (size_t i = *stop_idx - 1; i < traj.points.size(); --i) {  // search backward
    distance_sum += autoware_utils::calcDistance2d(traj.points.at(i), traj.points.at(i + 1));
    if (distance_sum > node_param_.stopping_distance) break;
    if (traj.points.at(i).twist.linear.x > node_param_.stopping_velocity) {
      traj.points.at(i).twist.linear.x = node_param_.stopping_velocity;
    }
  }
}

void MotionVelocitySmoother::publishDebugTrajectories(
  const std::vector<autoware_planning_msgs::Trajectory> & debug_trajectories) const
{
  if (node_param_.algorithm_type == AlgorithmType::JERK_FILTERED) {
    if (debug_trajectories.size() != 3) {
      ROS_WARN("[MotionVelocitySmoother] Size of the debug trajectories is incorrect");
      return;
    }
    pub_forward_filtered_trajectory_.publish(debug_trajectories[0]);
    pub_backward_filtered_trajectory_.publish(debug_trajectories[1]);
    pub_merged_filtered_trajectory_.publish(debug_trajectories[2]);
    publishClosestVelocity(
      debug_trajectories[2], current_pose_ptr_->pose, pub_closest_merged_velocity_);
  }

  return;
}

void MotionVelocitySmoother::publishClosestVelocity(
  const autoware_planning_msgs::Trajectory & trajectory, const geometry_msgs::Pose & current_pose,
  const ros::Publisher & pub) const
{
  const auto closest_point =
    trajectory_utils::calcInterpolatedTrajectoryPoint(trajectory, current_pose);

  std_msgs::Float32 vel_data;
  vel_data.data = std::max(closest_point.twist.linear.x, 0.0);
  pub.publish(vel_data);
}

void MotionVelocitySmoother::publishClosestState(
  const autoware_planning_msgs::TrajectoryPoint & closest_point)
{
  auto publishFloat = [](const double data, const ros::Publisher & pub) {
    std_msgs::Float32 msg;
    msg.data = data;
    pub.publish(msg);
    return;
  };

  const double curr_vel = closest_point.twist.linear.x;
  const double curr_acc = closest_point.accel.linear.x;
  if (!prev_time_) {
    prev_time_ = std::make_shared<ros::Time>(ros::Time::now());
    prev_acc_ = curr_acc;
    return;
  }

  // Calculate jerk
  ros::Time curr_time = ros::Time::now();
  double dt = (curr_time - *prev_time_).toSec();
  double curr_jerk = (curr_acc - prev_acc_) / dt;

  // Publish data
  publishFloat(curr_vel, debug_closest_velocity_);
  publishFloat(curr_acc, debug_closest_acc_);
  publishFloat(curr_jerk, debug_closest_jerk_);

  // Update
  prev_acc_ = curr_acc;
  *prev_time_ = curr_time;
}

MotionVelocitySmoother::AlgorithmType MotionVelocitySmoother::getAlgorithmType(
  const std::string & algorithm_name) const
{
  if (algorithm_name == "JerkFiltered") return AlgorithmType::JERK_FILTERED;
  if (algorithm_name == "L2") return AlgorithmType::L2;
  if (algorithm_name == "Linf") return AlgorithmType::LINF;

  throw std::domain_error("[MotionVelocitySmoother] undesired algorithm is selected.");
  return AlgorithmType::INVALID;
}

double MotionVelocitySmoother::calcTravelDistance() const
{
  const auto closest_point =
    trajectory_utils::calcInterpolatedTrajectoryPoint(prev_output_, current_pose_ptr_->pose);

  if (prev_closest_point_) {
    const double travel_dist = autoware_utils::calcDistance2d(*prev_closest_point_, closest_point);
    return travel_dist;
  }

  return 0.0;
}

bool MotionVelocitySmoother::isEngageStatus(const double target_vel) const
{
  const double vehicle_speed = std::fabs(current_velocity_ptr_->twist.linear.x);
  const double engage_vel_thr = node_param_.engage_velocity * node_param_.engage_exit_ratio;
  return vehicle_speed < engage_vel_thr && target_vel >= node_param_.engage_velocity;
}

}  // namespace motion_velocity_smoother
