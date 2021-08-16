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

#ifndef AUTOWARE_CONTROL_VELOCITY_CONTROLLER_H
#define AUTOWARE_CONTROL_VELOCITY_CONTROLLER_H

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include "autoware_utils/geometry/geometry.h"
#include "autoware_utils/math/constants.h"
#include "autoware_utils/math/unit_conversion.h"
#include "autoware_utils/ros/self_pose_listener.h"
#include "autoware_utils/ros/wait_for_param.h"

#include "autoware_control_msgs/ControlCommandStamped.h"
#include "autoware_planning_msgs/Trajectory.h"
#include "signal_processing/lowpass_filter_1d.h"

#include "velocity_controller/debug_values.h"
#include "velocity_controller/pid.h"
#include "velocity_controller/smooth_stop.h"
#include "velocity_controller/velocity_controller_utils.h"

#include <dynamic_reconfigure/server.h>
#include "velocity_controller/VelocityControllerConfig.h"

class VelocityController
{
public:
  VelocityController();

private:
  struct Motion
  {
    double vel = 0.0;
    double acc = 0.0;
  };

  enum class Shift { Forward = 0, Reverse };

  struct ControlData
  {
    bool is_far_from_trajectory = false;
    size_t closest_idx = 0;  // closest_idx = 0 when closest_idx is not found with findNearestIdx
    Motion current_motion;
    Shift shift = Shift::Forward;  // shift is used only to calculate the sign of pitch compensation
    double stop_dist = 0.0;  // signed distance that is positive when car is before the stopline
    double slope_angle = 0.0;
    double dt = 0.0;
  };

  // ros variables
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_current_vel_;
  ros::Subscriber sub_trajectory_;
  ros::Publisher pub_control_cmd_;
  ros::Publisher pub_slope_;
  ros::Publisher pub_debug_;
  ros::Timer timer_control_;

  autoware_utils::SelfPoseListener self_pose_listener_;

  // variables for ros topic
  std::shared_ptr<geometry_msgs::TwistStamped> current_vel_ptr_;
  std::shared_ptr<geometry_msgs::TwistStamped> prev_vel_ptr_;
  std::shared_ptr<autoware_planning_msgs::Trajectory> trajectory_ptr_;

  // dynamic reconfigure
  dynamic_reconfigure::Server<velocity_controller::VelocityControllerConfig>
    dynamic_reconfigure_srv_;

  // vehicle info
  double wheel_base_;

  // control state
  enum class ControlState { DRIVE = 0, STOPPING, STOPPED, EMERGENCY };
  ControlState control_state_;

  // timer callback
  double control_rate_;

  // delay compensation
  double delay_compensation_time_;

  // enable flags
  bool enable_smooth_stop_;
  bool enable_overshoot_emergency_;
  bool enable_slope_compensation_;

  // smooth stop transition
  struct StateTransitionParams
  {
    // drive
    double drive_state_stop_dist;
    double drive_state_offset_stop_dist;
    // stopping
    double stopping_state_stop_dist;
    // stop
    double stopped_state_entry_vel;
    double stopped_state_entry_acc;
    // emergency
    double emergency_state_overshoot_stop_dist;
    double emergency_state_traj_trans_dev;
    double emergency_state_traj_rot_dev;
  };
  StateTransitionParams state_transition_params_;

  // drive
  PIDController pid_vel_;
  std::shared_ptr<LowpassFilter1d> lpf_vel_error_;
  double current_vel_threshold_pid_integrate_;

  // smooth stop
  SmoothStop smooth_stop_;

  // stop
  struct StoppedStateParams
  {
    double vel;
    double acc;
  };
  StoppedStateParams stopped_state_params_;

  // emergency
  struct EmergencyStateParams
  {
    double vel;
    double acc;
    double jerk;
  };
  EmergencyStateParams emergency_state_params_;

  // acceleration limit
  double max_acc_;
  double min_acc_;

  // jerk limit
  double max_jerk_;
  double min_jerk_;

  // slope compensation
  bool use_traj_for_pitch_;
  std::shared_ptr<LowpassFilter1d> lpf_pitch_;
  double max_pitch_rad_;
  double min_pitch_rad_;

  // 1st order lowpass filter for acceleration
  std::shared_ptr<LowpassFilter1d> lpf_acc_;

  // buffer of send command
  std::vector<autoware_control_msgs::ControlCommandStamped> ctrl_cmd_vec_;

  // for calculating dt
  std::shared_ptr<ros::Time> prev_control_time_;

  // shift mode
  Shift prev_shift_;

  // diff limit
  Motion prev_ctrl_cmd_;
  std::vector<std::pair<ros::Time, double>> vel_hist_;

  // debug values
  DebugValues debug_values_;

  std::shared_ptr<ros::Time> last_running_time_;

  // callback
  void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr & msg);
  void callbackTrajectory(const autoware_planning_msgs::TrajectoryConstPtr & msg);
  void callbackConfig(
    const velocity_controller::VelocityControllerConfig & config, const uint32_t level);
  void callbackTimerControl(const ros::TimerEvent & event);

  // main
  ControlData getControlData(const geometry_msgs::Pose & current_pose);
  Motion calcEmergencyCtrlCmd(const double dt) const;
  ControlState updateControlState(
    const ControlState current_control_state, const geometry_msgs::Pose & current_pose,
    const ControlData & control_data);
  Motion calcCtrlCmd(
    const ControlState & current_control_state, const geometry_msgs::Pose & current_pose,
    const ControlData & control_data);
  void publishCtrlCmd(const Motion & ctrl_cmd, const double current_vel);
  void publishDebugData(
    const Motion & ctrl_cmd, const ControlData & control_data,
    const geometry_msgs::Pose & current_pose);

  // control data
  double getDt();
  Motion getCurrentMotion() const;
  enum Shift getCurrentShift(const size_t closest_idx) const;

  // filter acceleration
  double calcFilteredAcc(const double raw_acc, const ControlData & control_data);
  void storeAccelCmd(const double accel);
  double applyLimitFilter(const double input_val, const double max_val, const double min_val) const;
  double applySlopeCompensation(const double acc, const double pitch, const Shift shift) const;
  double applyDiffLimitFilter(
    const double input_val, const double prev_val, const double dt, const double lim_val) const;
  double applyDiffLimitFilter(
    const double input_val, const double prev_val, const double dt, const double max_val,
    const double min_val) const;

  // drive
  autoware_planning_msgs::TrajectoryPoint calcInterpolatedTargetValue(
    const autoware_planning_msgs::Trajectory & traj, const geometry_msgs::Point & point,
    const double current_vel, const size_t closest_idx) const;
  double predictedVelocityInTargetPoint(
    const Motion current_motion, const double delay_compensation_time) const;
  double applyVelocityFeedback(
    const Motion target_motion, const double dt, const double current_vel);

  // debug
  void updatePitchDebugValues(const double pitch, const double traj_pitch, const double raw_pitch);
  void updateDebugVelAcc(
    const Motion & ctrl_cmd, const geometry_msgs::Pose & current_pose,
    const ControlData & control_data);
};

#endif
