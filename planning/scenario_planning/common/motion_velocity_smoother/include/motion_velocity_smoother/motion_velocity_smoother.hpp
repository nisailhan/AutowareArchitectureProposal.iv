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

#pragma once

#include <iostream>
#include <mutex>
#include <string>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include "autoware_planning_msgs/Trajectory.h"
#include "autoware_utils/geometry/geometry.h"
#include "autoware_utils/math/unit_conversion.h"
#include "autoware_utils/ros/self_pose_listener.h"
#include "autoware_utils/trajectory/trajectory.h"
#include "osqp_interface/osqp_interface.h"

#include "motion_velocity_smoother/MotionVelocitySmootherConfig.h"
#include "motion_velocity_smoother/resample.hpp"
#include "motion_velocity_smoother/smoother/jerk_filtered_smoother.hpp"
#include "motion_velocity_smoother/smoother/l2_pseudo_jerk_smoother.hpp"
#include "motion_velocity_smoother/smoother/linf_pseudo_jerk_smoother.hpp"
#include "motion_velocity_smoother/smoother/smoother_base.hpp"
#include "motion_velocity_smoother/trajectory_utils.hpp"

namespace motion_velocity_smoother
{
class MotionVelocitySmoother
{
public:
  MotionVelocitySmoother();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_trajectory_;           //!< @brief publisher for output trajectory
  ros::Publisher pub_over_stop_velocity_;   //!< @brief publisher for over stop velocity warning
  ros::Subscriber sub_current_velocity_;    //!< @brief subscriber for current velocity
  ros::Subscriber sub_current_trajectory_;  //!< @brief subscriber for reference trajectory
  ros::Subscriber sub_external_velocity_limit_;  //!< @brief subscriber for external velocity limit
  tf2_ros::Buffer tf_buffer_;                    //!< @brief tf butter
  tf2_ros::TransformListener tf_listener_;       //!< @brief tf listener

  geometry_msgs::PoseStamped::ConstPtr current_pose_ptr_;           // current vehicle pose
  geometry_msgs::TwistStamped::ConstPtr current_velocity_ptr_;      // current vehicle twist
  autoware_planning_msgs::Trajectory::ConstPtr base_traj_raw_ptr_;  // current base_waypoints
  double external_velocity_limit_;  // current external_velocity_limit
  double
    max_velocity_with_deceleration_;  // maximum velocity with deceleration for external velocity limit
  double external_velocity_limit_dist_ = 0.0;  // distance to set external velocity limit

  autoware_planning_msgs::Trajectory prev_output_;  // previously published trajectory
  boost::optional<autoware_planning_msgs::TrajectoryPoint> prev_closest_point_ =
    {};  // previous trajectory point
         // closest to ego vehicle

  autoware_utils::SelfPoseListener self_pose_listener_;

  enum class AlgorithmType {
    INVALID = 0,
    JERK_FILTERED = 1,
    L2 = 2,
    LINF = 3,
  };

  enum class InitializeType {
    INIT = 0,
    LARGE_DEVIATION_REPLAN = 1,
    ENGAGING = 2,
    NORMAL = 3,
  };

  struct Param
  {
    double max_velocity;                              // max velocity [m/s]
    double margin_to_insert_external_velocity_limit;  // for external velocity limit [m]
    double replan_vel_deviation;  // if speed error exceeds this [m/s], replan from current velocity
    double engage_velocity;       // use this speed when start moving [m/s]
    double engage_acceleration;   // use this acceleration when start moving [m/ss]
    double engage_exit_ratio;     // exit engage sequence when the speed exceeds ratio x engage_vel.
    double stopping_velocity;     // change target velocity to this value before v=0 point.
    double stopping_distance;     // distance for the stopping_velocity
    double extract_ahead_dist;    // forward waypoints distance from current position [m]
    double extract_behind_dist;   // backward waypoints distance from current position [m]
    double stop_dist_to_prohibit_engage;  // prevent to move toward close stop point
    double delta_yaw_threshold;           // for closest index calculation
    resampling::ResampleParam post_resample_param;
    AlgorithmType algorithm_type;  // Option : JerkFiltered, Linf, L2
  };

  std::shared_ptr<SmootherBase> smoother_;

  bool publish_debug_trajs_;  // publish planned trajectories

  double over_stop_velocity_warn_thr_;  // threshold to publish over velocity warn

  Param node_param_;

  // topic callback
  void onCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr msg);

  void onCurrentTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr msg);

  void onExternalVelocityLimit(const std_msgs::Float32::ConstPtr msg);

  // publish methods
  void publishTrajectory(const autoware_planning_msgs::Trajectory & traj) const;

  void publishStopDistance(
    const autoware_planning_msgs::Trajectory & trajectory, const size_t closest) const;

  // non-const methods
  void publishClosestState(const autoware_planning_msgs::TrajectoryPoint & closest_point);

  // const methods
  AlgorithmType getAlgorithmType(const std::string & algorithm_name) const;

  autoware_planning_msgs::Trajectory calcTrajectoryVelocity(
    const autoware_planning_msgs::Trajectory & input) const;

  bool smoothVelocity(
    const autoware_planning_msgs::Trajectory & input,
    autoware_planning_msgs::Trajectory & traj_smoothed) const;

  std::tuple<double, double, InitializeType> calcInitialMotion(
    const autoware_planning_msgs::Trajectory & input_traj, const size_t input_closest,
    const autoware_planning_msgs::Trajectory & prev_traj) const;

  void applyExternalVelocityLimit(autoware_planning_msgs::Trajectory & traj) const;

  void insertBehindVelocity(
    const size_t output_closest, const InitializeType type,
    autoware_planning_msgs::Trajectory & output) const;

  void applyStopApproachingVelocity(autoware_planning_msgs::Trajectory & traj) const;

  void overwriteStopPoint(
    const autoware_planning_msgs::Trajectory & input,
    autoware_planning_msgs::Trajectory & output) const;

  double calcTravelDistance() const;

  bool isEngageStatus(const double target_vel) const;

  // parameter handling
  Param getCommonParam() const;

  SmootherBase::BaseParam getSmootherBaseParam() const;

  JerkFilteredSmoother::Param getJerkFilteredSmootherParam() const;

  L2PseudoJerkSmoother::Param getL2PseudoJerkSmootherParam() const;

  LinfPseudoJerkSmoother::Param getLinfPseudoJerkSmootherParam() const;

  // dynamic reconfigure
  dynamic_reconfigure::Server<motion_velocity_smoother::MotionVelocitySmootherConfig>
    dynamic_reconfigure_server_;

  void onDynamicReconfigure(
    motion_velocity_smoother::MotionVelocitySmootherConfig & config, uint32_t level);

  // debug
  std::shared_ptr<ros::Time> prev_time_;
  double prev_acc_;
  ros::Publisher pub_dist_to_stopline_;
  ros::Publisher pub_trajectory_raw_;
  ros::Publisher pub_velocity_limit_;
  ros::Publisher pub_trajectory_vel_lim_;
  ros::Publisher pub_trajectory_latacc_filtered_;
  ros::Publisher pub_trajectory_resampled_;
  ros::Publisher debug_closest_velocity_;
  ros::Publisher debug_closest_acc_;
  ros::Publisher debug_closest_jerk_;
  ros::Publisher debug_calculation_time_;
};
}  // namespace motion_velocity_smoother
