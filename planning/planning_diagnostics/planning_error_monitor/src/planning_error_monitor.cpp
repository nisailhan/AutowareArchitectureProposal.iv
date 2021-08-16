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

#include <planning_error_monitor/planning_error_monitor.h>

#include <autoware_utils/autoware_utils.h>

PlanningErrorMonitor::PlanningErrorMonitor() : pnh_("~")
{
  traj_sub_ =
    pnh_.subscribe("input/trajectory", 1, &PlanningErrorMonitor::onCurrentTrajectory, this);

  updater_.setHardwareID("planning_error_monitor");
  updater_.add(
    "trajectory_point_validation",
    boost::bind(&PlanningErrorMonitor::onTrajectoryPointValueChecker, this, _1));
  updater_.add(
    "trajectory_interval_validation",
    boost::bind(&PlanningErrorMonitor::onTrajectoryIntervalChecker, this, _1));
  updater_.add(
    "trajectory_curvature_validation",
    boost::bind(&PlanningErrorMonitor::onTrajectoryCurvatureChecker, this, _1));

  timer_ = pnh_.createTimer(ros::Duration(0.1), &PlanningErrorMonitor::onTimer, this);

  // Parameter
  pnh_.param<double>("error_interval", error_interval_, 100.0);
  pnh_.param<double>("error_curvature", error_curvature_, 1.0);
}

void PlanningErrorMonitor::onTimer(const ros::TimerEvent & event) { updater_.force_update(); }

void PlanningErrorMonitor::onCurrentTrajectory(const autoware_planning_msgs::TrajectoryConstPtr msg)
{
  current_trajectory_ = msg;
}

void PlanningErrorMonitor::onTrajectoryPointValueChecker(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!current_trajectory_) return;

  std::string error_msg;
  const auto diag_level = checkTrajectoryPointValue(*current_trajectory_, error_msg)
                            ? diagnostic_msgs::DiagnosticStatus::OK
                            : diagnostic_msgs::DiagnosticStatus::ERROR;
  stat.summary(diag_level, error_msg);
}

void PlanningErrorMonitor::onTrajectoryIntervalChecker(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!current_trajectory_) return;

  std::string error_msg;
  const auto diag_level = checkTrajectoryInterval(*current_trajectory_, error_interval_, error_msg)
                            ? diagnostic_msgs::DiagnosticStatus::OK
                            : diagnostic_msgs::DiagnosticStatus::ERROR;
  stat.summary(diag_level, error_msg);
}

void PlanningErrorMonitor::onTrajectoryCurvatureChecker(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!current_trajectory_) return;

  std::string error_msg;
  const auto diag_level =
    checkTrajectoryCurvature(*current_trajectory_, error_curvature_, error_msg)
      ? diagnostic_msgs::DiagnosticStatus::OK
      : diagnostic_msgs::DiagnosticStatus::ERROR;
  stat.summary(diag_level, error_msg);
}

bool PlanningErrorMonitor::checkTrajectoryPointValue(
  const autoware_planning_msgs::Trajectory & traj, std::string & error_msg)
{
  error_msg = "This Trajectory doesn't have any invalid values";
  for (const auto & p : traj.points) {
    if (!checkFinite(p)) {
      error_msg = "This trajectory has an infinite value";
      return false;
    }
  }
  return true;
}

bool PlanningErrorMonitor::checkFinite(const autoware_planning_msgs::TrajectoryPoint & point)
{
  const auto & o = point.pose.orientation;
  const auto & p = point.pose.position;
  const auto & v = point.twist.linear;
  const auto & w = point.twist.angular;
  const auto & a = point.accel.linear;
  const auto & z = point.accel.angular;

  const bool quat_result =
    std::isfinite(o.x) && std::isfinite(o.y) && std::isfinite(o.z) && std::isfinite(o.w);
  const bool p_result = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  const bool v_result = std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  const bool w_result = std::isfinite(w.x) && std::isfinite(w.y) && std::isfinite(w.z);
  const bool a_result = std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
  const bool z_result = std::isfinite(z.x) && std::isfinite(z.y) && std::isfinite(z.z);

  return quat_result && p_result && v_result && w_result && a_result && z_result;
}

bool PlanningErrorMonitor::checkTrajectoryInterval(
  const autoware_planning_msgs::Trajectory & traj, const double & interval_threshold,
  std::string & error_msg)
{
  error_msg = "Trajectory Interval Length is within the expected range";
  for (size_t i = 1; i < traj.points.size(); ++i) {
    double ds = autoware_utils::calcDistance2d(traj.points.at(i), traj.points.at(i - 1));

    if (ds > interval_threshold) {
      error_msg = "Trajectory Interval Length is longer than the expected range";
      return false;
    }
  }
  return true;
}

bool PlanningErrorMonitor::checkTrajectoryCurvature(
  const autoware_planning_msgs::Trajectory & traj, const double & curvature_threshold,
  std::string & error_msg)
{
  error_msg = "This trajectory's curvature is within the expected range";

  // We need at least three points to compute curvature
  if (traj.points.size() < 3) return true;

  constexpr double points_distance = 1.0;

  for (size_t p1_id = 0; p1_id < traj.points.size() - 2; ++p1_id) {
    // Get Point1
    const auto p1 = traj.points.at(p1_id).pose.position;

    // Get Point2
    const auto p2_id = getIndexAfterDistance(traj, p1_id, points_distance);
    const auto p2 = traj.points.at(p2_id).pose.position;

    // Get Point3
    const auto p3_id = getIndexAfterDistance(traj, p2_id, points_distance);
    const auto p3 = traj.points.at(p3_id).pose.position;

    // no need to check for pi, since there is no point with "points_distance" from p1.
    if (p1_id == p2_id || p1_id == p3_id || p2_id == p3_id) break;

    const double curvature = autoware_utils::calcCurvature(p1, p2, p3);

    if (std::fabs(curvature) > curvature_threshold) {
      error_msg = "This Trajectory's curvature has larger value than the expected value";
      return false;
    }
  }
  return true;
}

size_t PlanningErrorMonitor::getIndexAfterDistance(
  const autoware_planning_msgs::Trajectory & traj, const size_t curr_id, const double distance)
{
  // Get Current Trajectory Point
  const autoware_planning_msgs::TrajectoryPoint & curr_p = traj.points.at(curr_id);

  size_t target_id = curr_id;
  double current_distance = 0.0;
  for (size_t traj_id = curr_id + 1; traj_id < traj.points.size(); ++traj_id) {
    current_distance = autoware_utils::calcDistance2d(traj.points.at(traj_id), curr_p);
    if (current_distance >= distance) {
      target_id = traj_id;
      break;
    }
  }

  return target_id;
}
