/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include <velocity_controller/velocity_controller_mathutils.h>

namespace vcutils
{
boost::optional<int> searchZeroVelocityIdx(const autoware_planning_msgs::Trajectory & trajectory)
{
  constexpr double epsilon = 1e-3;
  for (size_t i = 0; i < trajectory.points.size(); i++) {
    if (std::fabs(trajectory.points.at(i).twist.linear.x) < epsilon) {
      return i;
    }
  }
  return {};
}

boost::optional<double> calcLengthOnWaypoints(
  const autoware_planning_msgs::Trajectory & path, const unsigned int source_idx,
  const unsigned int target_idx)
{
  if (
    static_cast<unsigned int>(path.points.size()) - 1 < source_idx ||
    static_cast<unsigned int>(path.points.size()) - 1 < target_idx) {
    return {};
  }

  const unsigned int min_idx = std::min(source_idx, target_idx);
  const unsigned int max_idx = std::max(source_idx, target_idx);
  double dist_sum = 0.0;
  for (unsigned int i = min_idx; i < max_idx; ++i) {
    dist_sum += autoware_utils::calcDistance2d(path.points.at(i), path.points.at(i + 1));
  }
  return dist_sum;
}

int calcClosestWaypoint(
  const autoware_planning_msgs::Trajectory & traj, const geometry_msgs::Pose & pose)
{
  double dist_squared_min = std::numeric_limits<double>::max();
  int idx_min = -1;

  for (int i = 0; i < (int)traj.points.size(); ++i) {
    const double dx = traj.points.at(i).pose.position.x - pose.position.x;
    const double dy = traj.points.at(i).pose.position.y - pose.position.y;
    const double dist_squared = dx * dx + dy * dy;
    if (dist_squared < dist_squared_min) {
      dist_squared_min = dist_squared;
      idx_min = i;
    }
  }
  return idx_min;
}

int calcClosestWaypoint(
  const autoware_planning_msgs::Trajectory & trajectory, const geometry_msgs::Pose & pose,
  const double delta_yaw_threshold)
{
  double dist_squared_min = std::numeric_limits<double>::max();
  int idx_min = -1;

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const double dx = trajectory.points.at(i).pose.position.x - pose.position.x;
    const double dy = trajectory.points.at(i).pose.position.y - pose.position.y;
    const double dist_squared = dx * dx + dy * dy;
    const double traj_point_yaw = tf2::getYaw(trajectory.points.at(i).pose.orientation);
    const double pose_yaw = tf2::getYaw(pose.orientation);
    const double delta_yaw = traj_point_yaw - pose_yaw;
    const double norm_delta_yaw = autoware_utils::normalizeRadian(delta_yaw);
    if (dist_squared < dist_squared_min && std::fabs(norm_delta_yaw) < delta_yaw_threshold) {
      dist_squared_min = dist_squared;
      idx_min = i;
    }
  }
  return idx_min;
}

template <class Point>
bool judgePoseOverPoint(
  const Point & pose, const autoware_planning_msgs::Trajectory & traj, const int target_idx)
{
  geometry_msgs::Point p_target = pose.position;
  geometry_msgs::Point p_pass, p_ortho;
  if (target_idx > 0) {
    p_pass = traj.points.at(target_idx).pose.position;
    p_ortho = traj.points.at(target_idx - 1).pose.position;
  } else {
    p_pass = traj.points.at(target_idx).pose.position;
    p_ortho = traj.points.at(target_idx + 1).pose.position;
  }

  const double a = p_pass.x - p_ortho.x;
  const double b = p_pass.y - p_ortho.y;
  const double c =
    -std::pow(p_pass.x, 2) + p_pass.x * p_ortho.x - std::pow(p_pass.y, 2) + p_pass.y * p_ortho.y;

  const bool same_side =
    ((a * p_ortho.x + b * p_ortho.y + c) * (a * p_target.x + b * p_target.y + c)) > 0;

  if (target_idx > 0) {
    return !same_side;
  } else {
    return same_side;
  }
}

template <class Point>
std::pair<double, double> calcTwoPointsInterpolatedLength(
  const Point & p_target, const Point & p_from, const Point & p_to)
{
  double la = autoware_utils::calcDistance2d(p_from, p_to);
  double lb = autoware_utils::calcDistance2d(p_target, p_to);
  double lc = autoware_utils::calcDistance2d(p_target, p_from);

  if (std::abs(la) < 1e-6) {
    return {0.0, 0.0};
  }

  double length_from = (std::pow(la, 2) + std::pow(lc, 2) - std::pow(lb, 2)) / (2 * la);
  double length_to = (std::pow(la, 2) + std::pow(lb, 2) - std::pow(lc, 2)) / (2 * la);
  return {length_from, length_to};
}

void searchClosestTwoPoints(
  const geometry_msgs::Pose & pose, const autoware_planning_msgs::Trajectory & traj,
  signed int & min_idx, signed int & max_idx, double & min_idx_length, double & max_idx_length)
{
  int closest_idx = calcClosestWaypoint(traj, pose);

  if (closest_idx == 0) {
    min_idx = 0;
    max_idx = 1;
    return;
  } else if (closest_idx == static_cast<int>(traj.points.size() - 1)) {
    min_idx = static_cast<int>(traj.points.size() - 2);
    max_idx = static_cast<int>(traj.points.size() - 1);
    return;
  } else {
    double prev_dist = autoware_utils::calcDistance2d(pose, traj.points.at(closest_idx - 1).pose);
    double next_dist = autoware_utils::calcDistance2d(pose, traj.points.at(closest_idx + 1).pose);

    if (prev_dist <= next_dist) {
      min_idx = closest_idx - 1;
      max_idx = closest_idx;
    } else {
      min_idx = closest_idx;
      max_idx = closest_idx + 1;
    }
    std::pair<double, double> lengths = calcTwoPointsInterpolatedLength(
      pose, traj.points.at(min_idx).pose, traj.points.at(max_idx).pose);
    min_idx_length = lengths.first;
    max_idx_length = lengths.second;
  }
}

void searchClosestTwoPoints(
  const geometry_msgs::Pose & pose, const autoware_planning_msgs::Trajectory & traj,
  signed int & min_idx, signed int & max_idx)
{
  double min_idx_length, max_idx_length;
  searchClosestTwoPoints(pose, traj, min_idx, max_idx, min_idx_length, max_idx_length);
}

boost::optional<double> calcTrajectoryLengthFromPose(
  const geometry_msgs::Pose & pose, const autoware_planning_msgs::Trajectory & traj,
  unsigned int target_idx)
{
  int closest_idx = calcClosestWaypoint(traj, pose);  // TODO経路からある程度の距離以内
  if (closest_idx < 0) {
    return {};
  }

  const bool before_target_idx = !judgePoseOverPoint(pose, traj, target_idx);
  const bool before_traj = !judgePoseOverPoint(pose, traj, 0);
  const bool after_traj = judgePoseOverPoint(pose, traj, traj.points.size() - 1);

  int pose_round_idx;
  double length_to_round_waypoint;
  if (before_traj) {
    pose_round_idx = 0;
    length_to_round_waypoint =
      -calcTwoPointsInterpolatedLength(pose, traj.points.at(0).pose, traj.points.at(1).pose).first;
  } else if (after_traj) {
    pose_round_idx = traj.points.size() - 1;
    length_to_round_waypoint = -calcTwoPointsInterpolatedLength(
                                  pose, traj.points.at(traj.points.size() - 1).pose,
                                  traj.points.at(traj.points.size() - 2).pose)
                                  .first;
  } else {
    int min_idx, max_idx;
    double min_idx_length, max_idx_length;
    searchClosestTwoPoints(pose, traj, min_idx, max_idx, min_idx_length, max_idx_length);

    if (max_idx <= target_idx) {
      pose_round_idx = max_idx;
      length_to_round_waypoint = max_idx_length;
    } else {
      pose_round_idx = min_idx;
      length_to_round_waypoint = min_idx_length;
    }
  }

  boost::optional<double> length_on_waypoints =
    calcLengthOnWaypoints(traj, pose_round_idx, target_idx);
  if (length_on_waypoints) {
    return (*length_on_waypoints + length_to_round_waypoint) * (before_target_idx ? 1 : -1);
  } else {
    return {};
  }
}

boost::optional<double> calcStopDistance(
  const geometry_msgs::Pose & current_pose, const autoware_planning_msgs::Trajectory & traj)
{
  boost::optional<int> stop_idx = searchZeroVelocityIdx(traj);
  if (!stop_idx || traj.points.size() == 1) {
    return {};
  }

  boost::optional<double> length = calcTrajectoryLengthFromPose(current_pose, traj, *stop_idx);
  if (!length) {
    return {};
  }
  return length;
}

double calcPitch(const geometry_msgs::Pose & p1, const geometry_msgs::Pose & p2)
{
  const double dx = p1.position.x - p2.position.x;
  const double dy = p1.position.y - p2.position.y;
  const double dz = p1.position.z - p2.position.z;

  const double dist_2d = std::max(std::hypot(dx, dy), std::numeric_limits<double>::epsilon());
  const double pitch = std::atan2(dz, dist_2d);

  return pitch;
}

// TODO use autoware_utils
boost::optional<int> calcClosestWithThr(
  const autoware_planning_msgs::Trajectory & trajectory, const geometry_msgs::Pose & pose,
  const double angle_thr, const double dist_thr)
{
  double dist_min = std::numeric_limits<double>::max();
  int closest_idx = -1;

  for (int32_t i = 0; i < (int32_t)trajectory.points.size(); ++i) {
    const double ds = autoware_utils::calcDistance2d(trajectory.points.at(i).pose, pose);
    if (ds > dist_thr) continue;

    const double yaw_pose = tf2::getYaw(pose.orientation);
    const double yaw_ref = tf2::getYaw(trajectory.points.at(i).pose.orientation);
    const double yaw_diff = autoware_utils::normalizeRadian(yaw_pose - yaw_ref);

    if (std::fabs(yaw_diff) > angle_thr) continue;

    if (ds < dist_min) {
      dist_min = ds;
      closest_idx = i;
    }
  }

  if (closest_idx < 0) {
    return {};
  }
  return closest_idx;
}

geometry_msgs::Point transformToRelativeCoordinate2D(
  const geometry_msgs::Point & point, const geometry_msgs::Pose & origin)
{
  // translation
  geometry_msgs::Point trans_p;
  trans_p.x = point.x - origin.position.x;
  trans_p.y = point.y - origin.position.y;

  // rotation (use inverse matrix of rotation)
  double yaw = tf2::getYaw(origin.orientation);

  geometry_msgs::Point res;
  res.x = (std::cos(yaw) * trans_p.x) + (std::sin(yaw) * trans_p.y);
  res.y = (-std::sin(yaw) * trans_p.x) + (std::cos(yaw) * trans_p.y);
  res.z = origin.position.z;

  return res;
}
}  // namespace vcutils
