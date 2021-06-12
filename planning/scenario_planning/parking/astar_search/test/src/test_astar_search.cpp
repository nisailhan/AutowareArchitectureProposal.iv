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
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <array>
#include <fstream>
#include <string>
#include "astar_search/astar_search.h"

using namespace std;
using array3d = array<double, 3>;

geometry_msgs::Pose construct_pose_msg(array<double, 3> pose3d)
{
  geometry_msgs::Pose pose;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, pose3d[2]);
  tf2::convert(quat, pose.orientation);
  pose.position.x = pose3d[0];
  pose.position.y = pose3d[1];
  pose.position.z = 0.0;
  return pose;
}

nav_msgs::OccupancyGrid construct_cost_map(int width, int height, double resolution, int n_padding)
{
  auto costmap_msg = nav_msgs::OccupancyGrid();

  // create info
  auto info = nav_msgs::MapMetaData();
  info.width = width;
  info.height = height;
  info.resolution = resolution;
  info.origin = geometry_msgs::Pose();
  info.origin.orientation.w = 1.0;
  costmap_msg.info = info;

  // create data
  int n_elem = width * height;
  for (int i = 0; i < n_elem; i++) {
    costmap_msg.data.push_back(0.0);
  }

  for (int i = 0; i < n_padding; i++) {
    // fill left
    for (int j = width * i; j <= width * (i + 1); j++) {
      costmap_msg.data[j] = 100.0;
    }
    // fill right
    for (int j = width * (height - n_padding + i); j <= width * (height - n_padding + i + 1); j++) {
      costmap_msg.data[j] = 100.0;
    }
  }

  for (int i = 0; i < height; i++) {
    // fill bottom
    for (int j = i * width; j <= i * width + n_padding; j++) {
      costmap_msg.data[j] = 100.0;
    }
    for (int j = (i + 1) * width - n_padding; j <= (i + 1) * width; j++) {
      costmap_msg.data[j] = 100.0;
    }
  }
  return costmap_msg;
}

bool test_astar(
  array3d start, array3d goal, string file_name, double maximum_turning_radius = 9.0,
  int turning_radius_size = 1)
{
  // set problem configuration
  RobotShape shape{5.5, 2.75, 1.5};

  bool use_back = true;
  bool only_behind_solutions = false;
  double time_limit = 10000000.0;
  double minimum_turning_radius = 9.0;
  // double maximum_turning_radius is a parameter
  // int turning_radius_size is a parameter

  int theta_size = 144;
  double curve_weight = 1.2;
  double reverse_weight = 2;
  double lateral_goal_range = 0.5;
  double longitudinal_goal_range = 2.0;
  double angle_goal_range = 6.0;
  int obstacle_threshold = 100;
  double distance_heuristic_weight = 1.0;

  AstarParam astar_param_{use_back,
                          only_behind_solutions,
                          time_limit,
                          shape,
                          minimum_turning_radius,
                          maximum_turning_radius,
                          turning_radius_size,
                          theta_size,
                          curve_weight,
                          reverse_weight,
                          lateral_goal_range,
                          longitudinal_goal_range,
                          angle_goal_range,
                          obstacle_threshold,
                          distance_heuristic_weight};
  auto astar = AstarSearch(astar_param_);

  auto costmap_msg = construct_cost_map(150, 150, 0.2, 10);
  astar.initializeNodes(costmap_msg);

  const ros::WallTime begin = ros::WallTime::now();
  bool success = astar.makePlan(construct_pose_msg(start), construct_pose_msg(goal));
  const ros::WallTime now = ros::WallTime::now();
  const double msec = (now - begin).toSec() * 1000.0;
  if (success) {
    std::cout << "plan success : " << msec << "[msec]" << std::endl;
  } else {
    std::cout << "plan fail : " << msec << "[msec]" << std::endl;
  }
  auto result = astar.getWaypoints();

  // dump waypoints (will be used for debugging. the dumped file can be loaded by debug_plot.py)
  ofstream file(file_name);
  file << msec << std::endl;
  file << start[0] << ", " << start[1] << ", " << start[2] << std::endl;
  file << goal[0] << ", " << goal[1] << ", " << goal[2] << std::endl;
  for (auto & point : result.waypoints) {
    auto & pos = point.pose.pose.position;
    auto & ori = point.pose.pose.orientation;
    file << pos.x << ", " << pos.y << ", " << tf2::getYaw(ori) << std::endl;
  }
  file.close();

  // backtrace the path and confirm that the entire path is collision-free
  return success && astar.hasFeasibleSolution();
}

TEST(AstarSearchTestSuite, SingleCurvature)
{
  vector<double> goal_xs{8., 12., 16., 26.};
  for (int i = 0; i < goal_xs.size(); i++) {
    array<double, 3> start{6., 4., 0.5 * 3.1415};
    array<double, 3> goal{goal_xs[i], 4., 0.5 * 3.1415};
    string file_name = "/tmp/result_single" + to_string(i) + ".txt";
    EXPECT_TRUE(test_astar(start, goal, file_name));
  }
}

TEST(AstarSearchTestSuite, MultiCurvature)
{
  vector<double> goal_xs{8., 12., 16., 26.};
  double maximum_turning_radius = 14.0;
  int turning_radius_size = 3;
  for (int i = 0; i < goal_xs.size(); i++) {
    array<double, 3> start{6., 4., 0.5 * 3.1415};
    array<double, 3> goal{goal_xs[i], 4., 0.5 * 3.1415};
    string file_name = "/tmp/result_multi" + to_string(i) + ".txt";
    EXPECT_TRUE(test_astar(start, goal, file_name, maximum_turning_radius, turning_radius_size));
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
