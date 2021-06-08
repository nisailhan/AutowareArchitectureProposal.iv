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

#ifndef AUTOWARE_CONTROL_VELOCITY_CONTROLLER_H
#define AUTOWARE_CONTROL_VELOCITY_CONTROLLER_H

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <autoware_utils/geometry/geometry.h>
#include <autoware_utils/math/unit_conversion.h>
#include <autoware_utils/ros/self_pose_listener.h>
#include <autoware_utils/ros/wait_for_param.h>

#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_planning_msgs/Trajectory.h>

#include "debug_values.h"
#include "delay_compensation.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "smooth_stop.h"
#include "velocity_controller_mathutils.h"

#include <dynamic_reconfigure/server.h>
#include "velocity_controller/VelocityControllerConfig.h"

class VelocityController
{
public:
  VelocityController();

private:
  struct CtrlCmd
  {
    double vel;
    double acc;
  };

  // ros variables
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_current_vel_;
  ros::Subscriber sub_trajectory_;
  ros::Publisher pub_control_cmd_;
  ros::Publisher pub_debug_;
  ros::Timer timer_control_;

  autoware_utils::SelfPoseListener self_pose_listener_;

  // pointers for ros topic
  std::shared_ptr<geometry_msgs::PoseStamped> current_pose_ptr_;
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
  } state_transition_params_;

  // drive
  PIDController pid_vel_;
  Lpf1d lpf_vel_error_;
  double current_vel_threshold_pid_integrate_;

  // smooth stop
  SmoothStop smooth_stop_;

  // stop
  double stopped_vel_;
  double stopped_acc_;

  // emergency
  double emergency_vel_;
  double emergency_acc_;
  double emergency_jerk_;

  // acceleration limit
  double max_acc_;
  double min_acc_;

  // jerk limit
  double max_jerk_;
  double min_jerk_;

  // slope compensation
  bool use_traj_for_pitch_;
  Lpf1d lpf_pitch_;
  double max_pitch_rad_;
  double min_pitch_rad_;

  // lowpass filter for acceleration
  Lpf1d lpf_acc_;

  // buffer of send command
  std::vector<autoware_control_msgs::ControlCommandStamped> ctrl_cmd_vec_;

  // for calculating dt
  std::shared_ptr<ros::Time> prev_control_time_;

  // shift mode
  enum Shift {
    Forward = 0,
    Reverse,
  } prev_shift_;

  // diff limit
  double prev_acc_cmd_;
  double prev_vel_cmd_;

  // debug values
  DebugValues debug_values_;

  std::shared_ptr<ros::Time> last_running_time_;

  // callback
  void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr & msg);
  void callbackTrajectory(const autoware_planning_msgs::TrajectoryConstPtr & msg);
  void callbackConfig(
    const velocity_controller::VelocityControllerConfig & config, const uint32_t level);
  void callbackTimerControl(const ros::TimerEvent & event);

  bool isValidTrajectory(const autoware_planning_msgs::Trajectory & traj) const;

  void updateControlState(
    const double current_vel, const double current_acc, const boost::optional<double> & stop_dist,
    const boost::optional<int> & closest_idx);
  CtrlCmd calcCtrlCmd(const double current_vec, const double current_acc, const int closest_idx);

  double getPitchByPose(const geometry_msgs::Quaternion & quaternion) const;
  double getPitchByTraj(
    const autoware_planning_msgs::Trajectory & msg, const int32_t closest) const;
  double getDt();
  enum Shift getCurrentShift(const double target_velocity) const;

  // filter acceleration
  double calcFilteredAcc(
    const double raw_acc, const double pitch, const double dt, const Shift shift);
  void storeAccelCmd(const double accel);
  double applyLimitFilter(const double input_val, const double max_val, const double min_val) const;
  double applySlopeCompensation(const double acc, const double pitch, const Shift shift) const;
  double applyRateFilter(
    const double input_val, const double prev_val, const double dt, const double lim_val) const;
  double applyRateFilter(
    const double input_val, const double prev_val, const double dt, const double max_val,
    const double min_val) const;

  // drive
  double calcInterpolatedTargetValue(
    const autoware_planning_msgs::Trajectory & traj, const geometry_msgs::PoseStamped & curr_pose,
    const double current_vel, const int closest, const std::string & value_type) const;
  double predictedVelocityInTargetPoint(
    const double current_vel, const double current_acc, const double delay_compensation_time) const;
  double getPointValue(
    const autoware_planning_msgs::TrajectoryPoint & point, const std::string & value_type) const;
  double applyVelocityFeedback(
    const double target_acc, const double target_vel, const double dt, const double current_vel);

  // publish
  void publishCtrlCmd(const double vel, const double acc);

  // debug
  void updatePitchDebugValues(const double pitch, const double traj_pitch, const double raw_pitch);
  void updateDebugVelAcc(
    const double current_vel, const double target_vel, const double target_acc, int closest_idx);
};

#endif
