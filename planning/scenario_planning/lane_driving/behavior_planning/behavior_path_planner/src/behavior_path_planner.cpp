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
 *
 */

#include "behavior_path_planner/behavior_path_planner.hpp"
#include "autoware_utils/autoware_utils.h"
#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/utilities.hpp"
#include "visualization_msgs/MarkerArray.h"

#include "behavior_path_planner/scene_module/avoidance/avoidance_module.hpp"
#include "behavior_path_planner/scene_module/lane_change/lane_change_module.hpp"
#include "behavior_path_planner/scene_module/side_shift/side_shift_module.hpp"

namespace behavior_path_planner
{
BehaviorPathPlanner::BehaviorPathPlanner() : pnh_("~")
{
  // data_manager
  {
    planner_data_ = std::make_shared<PlannerData>();
    planner_data_->parameters = getCommonParam();
  }

  velocity_subscriber_ =
    pnh_.subscribe("input/velocity", 1, &BehaviorPathPlanner::onVelocity, this);
  perception_subscriber_ =
    pnh_.subscribe("input/perception", 1, &BehaviorPathPlanner::onPerception, this);
  external_approval_subscriber_ =
    pnh_.subscribe("input/external_approval", 1, &BehaviorPathPlanner::onExternalApproval, this);
  force_approval_subscriber_ =
    pnh_.subscribe("input/force_approval", 1, &BehaviorPathPlanner::onForceApproval, this);

  // route_handler
  vector_map_subscriber_ = pnh_.subscribe("input/vector_map", 1, &BehaviorPathPlanner::onMap, this);
  route_subscriber_ = pnh_.subscribe("input/route", 1, &BehaviorPathPlanner::onRoute, this);

  // publisher
  path_publisher_ = pnh_.advertise<autoware_planning_msgs::PathWithLaneId>("output/path", 1);
  path_candidate_publisher_ =
    pnh_.advertise<autoware_planning_msgs::Path>("output/path_candidate", 1);
  turn_signal_publisher_ =
    pnh_.advertise<autoware_vehicle_msgs::TurnSignal>("output/turn_signal_cmd", 1, false);
  debug_drivable_area_publisher_ =
    pnh_.advertise<nav_msgs::OccupancyGrid>("debug/drivable_area", 1);
  debug_path_publisher_ =
    pnh_.advertise<autoware_planning_msgs::Path>("debug/path_for_visualize", 1);

  // For remote operation
  plan_ready_publisher_ =
    pnh_.advertise<autoware_planning_msgs::PathChangeModule>("output/ready", 1);
  plan_running_publisher_ =
    pnh_.advertise<autoware_planning_msgs::PathChangeModuleArray>("output/running", 1);
  force_available_publisher_ =
    pnh_.advertise<autoware_planning_msgs::PathChangeModuleArray>("output/force_available", 1);

  // Debug
  debug_marker_publisher_ = pnh_.advertise<visualization_msgs::MarkerArray>("debug/markers", 1);

  // behavior tree manager
  {
    bt_manager_ = std::make_shared<BehaviorTreeManager>(getBehaviorTreeManagerParam());

    auto side_shift_module = std::make_shared<SideShiftModule>("SideShift", getSideShiftParam());
    bt_manager_->registerSceneModule(side_shift_module);

    auto avoidance_module = std::make_shared<AvoidanceModule>("Avoidance", getAvoidanceParam());
    bt_manager_->registerSceneModule(avoidance_module);

    auto lane_following_module =
      std::make_shared<LaneFollowingModule>("LaneFollowing", getLaneFollowingParam());
    bt_manager_->registerSceneModule(lane_following_module);

    auto lane_change_module =
      std::make_shared<LaneChangeModule>("LaneChange", getLaneChangeParam());
    bt_manager_->registerSceneModule(lane_change_module);

    auto force_lane_change_module =
      std::make_shared<LaneChangeModule>("ForceLaneChange", getLaneChangeParam());
    bt_manager_->registerSceneModule(force_lane_change_module);

    bt_manager_->registerForceApproval("ForceLaneChange");

    bt_manager_->createBehaviorTree();
  }

  // turn signal decider
  {
    double intersection_search_distance;
    pnh_.param("intersection_search_distance", intersection_search_distance, 30.0);
    turn_signal_decider_.setParameters(
      planner_data_->parameters.base_link2front, intersection_search_distance);
  }

  waitForData();

  // Start timer. This must be done after all data (e.g. vehicle pose, velocity) are ready.
  timer_ = pnh_.createTimer(ros::Duration(0.1), &BehaviorPathPlanner::run, this);
}

BehaviorPathPlannerParameters BehaviorPathPlanner::getCommonParam()
{
  // ROS parameters
  BehaviorPathPlannerParameters p;
  pnh_.param("backward_path_length", p.backward_path_length, 5.0);
  pnh_.param("forward_path_length", p.forward_path_length, 100.0);
  pnh_.param(
    "backward_length_buffer_for_end_of_lane", p.backward_length_buffer_for_end_of_lane, 5.0);
  pnh_.param("minimum_lane_change_length", p.minimum_lane_change_length, 8.0);
  pnh_.param("drivable_area_resolution", p.drivable_area_resolution, 0.1);
  pnh_.param("drivable_area_width", p.drivable_area_width, 100.0);
  pnh_.param("drivable_area_height", p.drivable_area_height, 50.0);
  pnh_.param("refine_goal_search_radius_range", p.refine_goal_search_radius_range, 7.5);

  // vehicle info
  pnh_.param("/vehicle_info/vehicle_width", p.vehicle_width, 2.8);
  pnh_.param("/vehicle_info/vehicle_length", p.vehicle_length, 5.0);
  pnh_.param("/vehicle_info/wheel_base", p.wheel_base, 2.8);
  pnh_.param("/vehicle_info/front_overhang", p.front_overhang, 1.0);
  pnh_.param("/vehicle_info/rear_overhang", p.rear_overhang, 1.0);
  p.base_link2front = p.front_overhang + p.wheel_base;
  p.base_link2rear = p.rear_overhang;

  return p;
}

SideShiftParameters BehaviorPathPlanner::getSideShiftParam()
{
  SideShiftParameters p;
  pnh_.param("min_fix_distance", p.min_fix_distance, 20.0);
  pnh_.param("start_avoid_sec", p.start_avoid_sec, 4.0);
  pnh_.param("drivable_area_resolution", p.drivable_area_resolution, 0.1);
  pnh_.param("drivable_area_width", p.drivable_area_width, 100.0);
  pnh_.param("drivable_area_height", p.drivable_area_height, 50.0);
  pnh_.param("inside_outside_judge_margin", p.inside_outside_judge_margin, 0.3);
  return p;
}

AvoidanceParameters BehaviorPathPlanner::getAvoidanceParam()
{
  AvoidanceParameters p;
  pnh_.param(
    "avoidance/threshold_distance_object_is_on_center", p.threshold_distance_object_is_on_center,
    1.0);
  pnh_.param(
    "avoidance/threshold_speed_object_is_stopped", p.threshold_speed_object_is_stopped, 1.0);
  pnh_.param("avoidance/object_check_forward_distance", p.object_check_forward_distance, 100.0);
  pnh_.param("avoidance/object_check_backward_distance", p.object_check_backward_distance, 2.0);
  pnh_.param("avoidance/lateral_collision_margin", p.lateral_collision_margin, 2.0);
  pnh_.param("avoidance/time_to_start_avoidance", p.time_to_start_avoidance, 3.0);
  pnh_.param("avoidance/min_distance_to_start_avoidance", p.min_distance_to_start_avoidance, 10.0);
  pnh_.param("avoidance/time_avoiding", p.time_avoiding, 3.0);
  pnh_.param("avoidance/min_distance_avoiding", p.min_distance_avoiding, 10.0);
  pnh_.param("avoidance/max_shift_length", p.max_shift_length, 1.5);
  pnh_.param(
    "avoidance/min_distance_avoidance_end_to_object", p.min_distance_avoidance_end_to_object, 5.0);
  pnh_.param("avoidance/time_avoidance_end_to_object", p.time_avoidance_end_to_object, 1.0);

  return p;
};

LaneFollowingParameters BehaviorPathPlanner::getLaneFollowingParam()
{
  LaneFollowingParameters p;
  pnh_.param("lane_change_prepare_duration", p.lane_change_prepare_duration, 2.0);
  return p;
};

LaneChangeParameters BehaviorPathPlanner::getLaneChangeParam()
{
  LaneChangeParameters p;
  pnh_.param("lane_change/min_stop_distance", p.min_stop_distance, 5.0);
  pnh_.param("lane_change/stop_time", p.stop_time, 2.0);
  pnh_.param("lane_change/hysteresis_buffer_distance", p.hysteresis_buffer_distance, 2.0);
  pnh_.param("lane_change/lane_change_prepare_duration", p.lane_change_prepare_duration, 2.0);
  pnh_.param("lane_change/lane_changing_duration", p.lane_changing_duration, 4.0);
  pnh_.param("lane_change/lane_change_finish_judge_buffer", p.lane_change_finish_judge_buffer, 3.0);
  pnh_.param("lane_change/minimum_lane_change_velocity", p.minimum_lane_change_velocity, 8.3);
  pnh_.param("lane_change/prediction_duration", p.prediction_duration, 8.0);
  pnh_.param("lane_change/prediction_time_resolution", p.prediction_time_resolution, 0.5);
  pnh_.param("lane_change/static_obstacle_velocity_thresh", p.static_obstacle_velocity_thresh, 0.1);
  pnh_.param("lane_change/maximum_deceleration", p.maximum_deceleration, 1.0);
  pnh_.param("lane_change/lane_change_sampling_num", p.lane_change_sampling_num, 10);
  pnh_.param("lane_change/enable_abort_lane_change", p.enable_abort_lane_change, true);
  pnh_.param(
    "lane_change/enable_collision_check_at_prepare_phase",
    p.enable_collision_check_at_prepare_phase, true);
  pnh_.param(
    "lane_change/use_predicted_path_outside_lanelet", p.use_predicted_path_outside_lanelet, true);
  pnh_.param("lane_change/use_all_predicted_path", p.use_all_predicted_path, false);
  pnh_.param(
    "lane_change/abort_lane_change_velocity_thresh", p.abort_lane_change_velocity_thresh, 0.5);
  pnh_.param(
    "lane_change/abort_lane_change_angle_thresh", p.abort_lane_change_angle_thresh,
    autoware_utils::deg2rad(10.0));
  pnh_.param(
    "lane_change/abort_lane_change_distance_thresh", p.abort_lane_change_distance_thresh, 0.3);
  pnh_.param("lane_change/enable_blocked_by_obstacle", p.enable_blocked_by_obstacle, false);
  pnh_.param("lane_change/lane_change_search_distance", p.lane_change_search_distance, 30.0);

  // validation of parameters
  if (p.lane_change_sampling_num < 1) {
    ROS_FATAL_STREAM(
      "lane_change_sampling_num must be positive integer. Given parameter: "
      << p.lane_change_sampling_num << std::endl
      << "Terminating the program...");
    exit(EXIT_FAILURE);
  }
  if (p.maximum_deceleration < 0.0) {
    ROS_FATAL_STREAM(
      "maximum_deceleration cannot be negative value. Given parameter: "
      << p.maximum_deceleration << std::endl
      << "Terminating the program...");
    exit(EXIT_FAILURE);
  }

  return p;
};

BehaviorTreeManagerParam BehaviorPathPlanner::getBehaviorTreeManagerParam()
{
  BehaviorTreeManagerParam p;
  pnh_.param("bt_tree_config_path", p.bt_tree_config_path, std::string{"default"});
  pnh_.param("groot_zmq_publisher_port", p.groot_zmq_publisher_port, 1666);
  pnh_.param("groot_zmq_server_port", p.groot_zmq_server_port, 1667);
  return p;
}

void BehaviorPathPlanner::waitForData()
{
  // wait until mandatory data is ready
  while (!planner_data_->route_handler->isHandlerReady() && ros::ok()) {
    ROS_WARN_THROTTLE(5, "waiting for route to be ready");
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  while (ros::ok()) {
    if (planner_data_->dynamic_object && planner_data_->self_velocity) {
      break;
    }
    ROS_WARN_THROTTLE(5, "waiting for vehicle pose, vehicle_velocity, and obstacles");
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  self_pose_listener_.waitForFirstPose();
  planner_data_->self_pose = self_pose_listener_.getCurrentPose();
}

void BehaviorPathPlanner::run(const ros::TimerEvent & event)
{
  ROS_DEBUG("----- BehaviorPathPlanner start -----");

  // update planner data
  updateCurrentPose();

  // run behavior planner
  const auto output = bt_manager_->run(planner_data_);

  // path handling
  const auto path = getPath(output);
  const auto path_candidate = getPathCandidate(output);
  planner_data_->prev_output_path = path;

  const auto clipped_path = clipPathByGoal(*path);

  // TODO(Horibe) the path must have points. Needs to be fix.
  if (!clipped_path.points.empty()) {
    path_publisher_.publish(clipped_path);
  } else {
    ROS_ERROR(
      "behavior path output is empty! Stop publish. path = %lu, clipped = %lu", path->points.size(),
      clipped_path.points.size());
  }
  path_candidate_publisher_.publish(util::toPath(*path_candidate));
  // debug_path_publisher_.publish(util::toPath(path));
  debug_drivable_area_publisher_.publish(path->drivable_area);

  // for turn signal
  const auto turn_signal = turn_signal_decider_.getTurnSignal(
    *path, planner_data_->self_pose->pose, *(planner_data_->route_handler),
    output.turn_signal_info.turn_signal, output.turn_signal_info.signal_distance);
  turn_signal_publisher_.publish(turn_signal);

  // for remote operation
  publishModuleStatus(bt_manager_->getModulesStatus());

  publishDebugMarker(bt_manager_->getDebugMarkers());

  ROS_DEBUG("----- behavior path planner end -----\n\n");
}

std::shared_ptr<autoware_planning_msgs::PathWithLaneId> BehaviorPathPlanner::getPath(
  const BehaviorModuleOutput & bt_output)
{
  // TODO(Horibe) do some error handling when path is not available.
  auto path = bt_output.path ? bt_output.path : util::generateCenterLinePath(planner_data_);
  path->header = planner_data_->route_handler->getRouteHeader();
  path->header.stamp = ros::Time::now();
  ROS_DEBUG("BehaviorTreeManager: output is %s.", bt_output.path ? "FOUND" : "NOT FOUND");
  return path;
}

std::shared_ptr<autoware_planning_msgs::PathWithLaneId> BehaviorPathPlanner::getPathCandidate(
  const BehaviorModuleOutput & bt_output)
{
  auto path_candidate = bt_output.path_candidate
                          ? bt_output.path_candidate
                          : std::make_shared<autoware_planning_msgs::PathWithLaneId>();
  path_candidate->header = planner_data_->route_handler->getRouteHeader();
  path_candidate->header.stamp = ros::Time::now();
  ROS_DEBUG(
    "BehaviorTreeManager: path candidate is %s.", bt_output.path_candidate ? "FOUND" : "NOT FOUND");
  return path_candidate;
}

void BehaviorPathPlanner::publishModuleStatus(
  const std::vector<std::shared_ptr<SceneModuleStatus>> & statuses)
{
  auto getModuleType = [](std::string name) {
    if (name == "LaneChange") {
      return autoware_planning_msgs::PathChangeModuleId::LANE_CHANGE;
    } else if (name == "Avoidance") {
      return autoware_planning_msgs::PathChangeModuleId::AVOIDANCE;
    } else if (name == "ForceLaneChange") {
      return autoware_planning_msgs::PathChangeModuleId::FORCE_LANE_CHANGE;
    } else {
      return autoware_planning_msgs::PathChangeModuleId::NONE;
    }
  };

  const auto now = ros::Time::now();

  autoware_planning_msgs::PathChangeModule ready_module;
  autoware_planning_msgs::PathChangeModuleArray running_modules;
  autoware_planning_msgs::PathChangeModuleArray force_available;

  bool is_ready = false;
  for (auto & status : statuses) {
    if (status->status == BT::NodeStatus::RUNNING) {
      autoware_planning_msgs::PathChangeModuleId module;
      module.type = getModuleType(status->module_name);
      running_modules.modules.push_back(module);
    }
    if (status->module_name == "LaneChange") {
      const auto force_approval = planner_data_->approval.is_force_approved;
      if (
        force_approval.module_name == "ForceLaneChange" &&
        (now - force_approval.stamp).toSec() < 0.5) {
        is_ready = true;
        ready_module.module.type = getModuleType("ForceLaneChange");
      }
      if (status->is_requested && !status->is_ready) {
        autoware_planning_msgs::PathChangeModuleId module;
        module.type = getModuleType("ForceLaneChange");
        force_available.modules.push_back(module);
        break;
      }
    }
    if (status->is_ready && status->is_waiting_approval) {
      if (status->module_name == "LaneFollowing" || status->module_name == "SideShift") continue;
      is_ready = true;
      ROS_DEBUG(
        "%s is Ready : ready = %s, is_approved = %s", status->module_name.c_str(),
        status->is_ready ? "true" : "false", status->is_waiting_approval ? "true" : "false");
      ready_module.module.type = getModuleType(status->module_name);
    }
  }

  if (!is_ready) {
    prev_ready_module_name_ = "NONE";
    ready_module.module.type = autoware_planning_msgs::PathChangeModuleId::NONE;
  }

  ready_module.header.stamp = now;
  plan_ready_publisher_.publish(ready_module);

  running_modules.header.stamp = now;
  plan_running_publisher_.publish(running_modules);

  force_available.header.stamp = now;
  force_available_publisher_.publish(force_available);
}

void BehaviorPathPlanner::publishDebugMarker(
  const std::vector<visualization_msgs::MarkerArray> & debug_markers)
{
  visualization_msgs::MarkerArray msg;
  for (const auto & markers : debug_markers) {
    autoware_utils::appendMarkerArray(markers, &msg);
  }
  debug_marker_publisher_.publish(msg);
}

void BehaviorPathPlanner::updateCurrentPose()
{
  auto self_pose = self_pose_listener_.getCurrentPose();
  planner_data_->self_pose = self_pose;
}

void BehaviorPathPlanner::onVelocity(const geometry_msgs::TwistStamped::ConstPtr msg)
{
  planner_data_->self_velocity = msg;
}
void BehaviorPathPlanner::onPerception(
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr msg)
{
  planner_data_->dynamic_object = msg;
}
void BehaviorPathPlanner::onExternalApproval(const std_msgs::Bool & msg)
{
  planner_data_->approval.is_approved.data = msg.data;
  planner_data_->approval.is_approved.stamp = ros::Time::now();
}
void BehaviorPathPlanner::onForceApproval(
  const autoware_planning_msgs::PathChangeModule::ConstPtr & msg)
{
  auto getModuleName = [](autoware_planning_msgs::PathChangeModuleId module) {
    if (module.type == autoware_planning_msgs::PathChangeModuleId::FORCE_LANE_CHANGE) {
      return "ForceLaneChange";
    } else {
      return "NONE";
    }
  };
  planner_data_->approval.is_force_approved.module_name = getModuleName(msg->module);
  planner_data_->approval.is_force_approved.stamp = msg->header.stamp;
}
void BehaviorPathPlanner::onMap(const autoware_lanelet2_msgs::MapBinConstPtr msg)
{
  planner_data_->route_handler->setMap(*msg);
}
void BehaviorPathPlanner::onRoute(const autoware_planning_msgs::RouteConstPtr msg)
{
  const bool is_first_time = !(planner_data_->route_handler->isHandlerReady());

  planner_data_->route_handler->setRoute(*msg);

  // Reset behavior tree when new route is received,
  // so that the each modules do not have to care about the "route jump".
  if (!is_first_time) {
    ROS_DEBUG("new route is received. reset behavior tree.");
    bt_manager_->resetBehaviorTree();
  }
}

autoware_planning_msgs::PathWithLaneId BehaviorPathPlanner::clipPathByGoal(
  const autoware_planning_msgs::PathWithLaneId & path) const
{
  const auto goal = planner_data_->route_handler->getGoalPose();
  const auto goal_lane_id = planner_data_->route_handler->getGoalLaneId();

  geometry_msgs::Pose refined_goal;
  {
    lanelet::ConstLanelet goal_lanelet;
    if (planner_data_->route_handler->getGoalLanelet(&goal_lanelet)) {
      refined_goal = util::refineGoal(goal, goal_lanelet);
    } else {
      refined_goal = goal;
    }
  }

  auto refined_path = util::refinePath(
    planner_data_->parameters.refine_goal_search_radius_range, M_PI * 0.5, path, refined_goal,
    goal_lane_id);
  refined_path.header.frame_id = "map";
  refined_path.header.stamp = ros::Time::now();

  return refined_path;
}

}  // namespace behavior_path_planner
