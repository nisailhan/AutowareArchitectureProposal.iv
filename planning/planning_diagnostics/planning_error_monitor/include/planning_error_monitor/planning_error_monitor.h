/*
 * Copyright 2021 Tier IV, Inc.
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

#include <ros/ros.h>

#include <autoware_planning_msgs/Trajectory.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/diagnostic_updater.h>

class PlanningErrorMonitor
{
public:
  PlanningErrorMonitor();
  ~PlanningErrorMonitor() = default;

  void onCurrentTrajectory(const autoware_planning_msgs::TrajectoryConstPtr msg);
  void onTimer(const ros::TimerEvent & event);

  void onTrajectoryPointValueChecker(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void onTrajectoryIntervalChecker(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void onTrajectoryCurvatureChecker(diagnostic_updater::DiagnosticStatusWrapper & stat);

  static bool checkTrajectoryPointValue(
    const autoware_planning_msgs::Trajectory & traj, std::string & error_msg);
  static bool checkTrajectoryInterval(
    const autoware_planning_msgs::Trajectory & traj, const double & interval_threshold,
    std::string & error_msg);
  static bool checkTrajectoryCurvature(
    const autoware_planning_msgs::Trajectory & traj, const double & curvature_threshold,
    std::string & error_msg);

private:
  static bool checkFinite(const autoware_planning_msgs::TrajectoryPoint & p);
  static size_t getIndexAfterDistance(
    const autoware_planning_msgs::Trajectory & traj, const size_t curr_id, const double distance);

  // ROS
  ros::NodeHandle pnh_;
  ros::Subscriber traj_sub_;
  ros::Timer timer_;
  diagnostic_updater::Updater updater_;

  autoware_planning_msgs::TrajectoryConstPtr current_trajectory_;

  // Parameter
  double error_interval_;
  double error_curvature_;
};
