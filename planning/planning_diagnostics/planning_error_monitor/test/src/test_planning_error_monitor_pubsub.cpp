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

#include <gtest/gtest.h>

#include <planning_error_monitor/planning_error_monitor.h>
#include "test_planning_error_monitor_helper.hpp"

constexpr double NOMINAL_INTERVAL = 1.0;
constexpr double ERROR_INTERVAL = 1000.0;
constexpr double ERROR_CURVATURE = 2.0;


class PubSubManager
{
public:
  PubSubManager() : nh_("")
  {
    traj_pub_ = nh_.advertise<autoware_planning_msgs::Trajectory>(
      "/test_planning_error_monitor/input/trajectory", 1);
    diag_sub_ = nh_.subscribe<diagnostic_msgs::DiagnosticArray>(
      "/diagnostics", 1, [this](const diagnostic_msgs::DiagnosticArrayConstPtr & msg) {
        received_diags_.push_back(msg);
      });
  }

  ros::NodeHandle nh_;
  ros::Publisher traj_pub_;
  ros::Subscriber diag_sub_;

  std::vector<diagnostic_msgs::DiagnosticArrayConstPtr> received_diags_;
};

void spinSome()
{
  for (int i = 0; i < 50; ++i) {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
}

bool isAllOK(const std::vector<diagnostic_msgs::DiagnosticArrayConstPtr> & diags)
{
  for (const auto & diag : diags) {
    for (const auto & status : diag->status) {
      if (status.level != diagnostic_msgs::DiagnosticStatus::OK) return false;
    }
  }
  return true;
}

bool hasError(const std::vector<diagnostic_msgs::DiagnosticArrayConstPtr> & diags)
{
  for (const auto & diag : diags) {
    for (const auto & status : diag->status) {
      if (status.level == diagnostic_msgs::DiagnosticStatus::ERROR) return true;
    }
  }
  return false;
}

void runWithOKTrajectory(const autoware_planning_msgs::Trajectory & trajectory)
{
  PlanningErrorMonitor error_monitor;
  PubSubManager manager_;
  EXPECT_GE(manager_.traj_pub_.getNumSubscribers(), 1) << "topic is not connected.";

  manager_.traj_pub_.publish(trajectory);
  spinSome();

  EXPECT_GE(manager_.received_diags_.size(), 1) << "diag has not received!";
  EXPECT_TRUE(isAllOK(manager_.received_diags_));
}

void runWithBadTrajectory(const autoware_planning_msgs::Trajectory & trajectory)
{
  PlanningErrorMonitor error_monitor;
  PubSubManager manager_;
  EXPECT_GE(manager_.traj_pub_.getNumSubscribers(), 1) << "topic is not connected.";

  manager_.traj_pub_.publish(trajectory);
  spinSome();

  EXPECT_GE(manager_.received_diags_.size(), 1) << "diag has not received!";
  EXPECT_TRUE(hasError(manager_.received_diags_));
}


// OK cases
TEST(PlanningErrorMonitor, DiagCheckForNominalTrajectory)
{
  runWithOKTrajectory(generateTrajectory(NOMINAL_INTERVAL));
}


// Bad cases
TEST(PlanningErrorMonitor, DiagCheckForNaNTrajectory)
{
  runWithBadTrajectory(generateNanTrajectory());
}
TEST(PlanningErrorMonitor, DiagCheckForInfTrajectory)
{
  runWithBadTrajectory(generateInfTrajectory());
}
TEST(PlanningErrorMonitor, DiagCheckForTooLongIntervalTrajectory)
{
  runWithBadTrajectory(generateTrajectory(ERROR_INTERVAL));
}

