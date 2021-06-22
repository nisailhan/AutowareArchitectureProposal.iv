/*
 * Copyright 2021 TierIV.
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

#include <planning_error_monitor/planning_error_monitor.h>

inline autoware_planning_msgs::Trajectory generateTrajectory(double interval_distance)
{
  autoware_planning_msgs::Trajectory traj;
  for (double s = 0.0; s <= 10.0 * interval_distance; s += interval_distance) {
    autoware_planning_msgs::TrajectoryPoint p;
    p.pose.position.x = s;
    p.twist.linear.x = 1.0;
    traj.points.push_back(p);
  }
  return traj;
}

inline autoware_planning_msgs::Trajectory generateNanTrajectory()
{
  autoware_planning_msgs::Trajectory traj = generateTrajectory(1.0);
  traj.points.front().pose.position.x = NAN;
  return traj;
}

inline autoware_planning_msgs::Trajectory generateInfTrajectory()
{
  autoware_planning_msgs::Trajectory traj = generateTrajectory(1.0);
  traj.points.front().pose.position.x = INFINITY;
  return traj;
}


inline autoware_planning_msgs::Trajectory generateBadCurvatureTrajectory()
{
  autoware_planning_msgs::Trajectory traj;

  double y = 1.5;
  for (double s = 0.0; s <= 10.0; s += 1.0) {
    autoware_planning_msgs::TrajectoryPoint p;
    p.twist.linear.x = 1.0;
    p.pose.position.x = s;
    p.pose.position.y = y;
    y *= -1.0;  // invert sign
    traj.points.push_back(p);
  }

  return traj;
}

