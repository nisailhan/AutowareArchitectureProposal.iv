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

#ifndef BEHAVIOR_PATH_PLANNER_HPP
#define BEHAVIOR_PATH_PLANNER_HPP

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "tf2_ros/transform_listener.h"

#include "autoware_lanelet2_msgs/MapBin.h"
#include "autoware_perception_msgs/DynamicObjectArray.h"
#include "autoware_planning_msgs/PathChangeModule.h"
#include "autoware_planning_msgs/PathChangeModuleArray.h"
#include "autoware_planning_msgs/PathChangeModuleId.h"
#include "autoware_planning_msgs/PathWithLaneId.h"
#include "autoware_planning_msgs/Route.h"
#include "autoware_planning_msgs/StopReasonArray.h"
#include "autoware_utils/ros/self_pose_listener.h"
#include "autoware_vehicle_msgs/TurnSignal.h"

#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_traffic_rules/TrafficRulesFactory.h"

#include <memory>
#include <vector>

// #include "behavior_path_planner/scene_module/avoidance/avoidance_module.hpp"
#include "behavior_path_planner/scene_module/lane_change/lane_change_module.hpp"
#include "behavior_path_planner/scene_module/lane_following/lane_following_module.hpp"
// #include "behavior_path_planner/scene_module/side_shift/side_shift_module.hpp"

#include "behavior_path_planner/behavior_tree_manager.hpp"
#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/route_handler.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"

namespace behavior_path_planner
{
class BehaviorPathPlanner
{
public:
  BehaviorPathPlanner();

private:
  ros::NodeHandle pnh_;

  ros::Subscriber route_subscriber_;
  ros::Subscriber vector_map_subscriber_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber perception_subscriber_;
  ros::Subscriber external_approval_subscriber_;
  ros::Subscriber force_approval_subscriber_;
  ros::Publisher path_publisher_;
  ros::Publisher path_candidate_publisher_;
  ros::Publisher force_available_publisher_;
  ros::Publisher plan_ready_publisher_;
  ros::Publisher plan_running_publisher_;
  ros::Publisher turn_signal_publisher_;
  ros::Timer timer_;

  std::shared_ptr<PlannerData> planner_data_;
  std::shared_ptr<BehaviorTreeManager> bt_manager_;
  autoware_utils::SelfPoseListener self_pose_listener_;

  std::string prev_ready_module_name_ = "NONE";

  TurnSignalDecider turn_signal_decider_;

  // setup
  void waitForData();

  // parameters
  BehaviorPathPlannerParameters getCommonParam();
  BehaviorTreeManagerParam getBehaviorTreeManagerParam();
  LaneFollowingParameters getLaneFollowingParam();
  LaneChangeParameters getLaneChangeParam();

  // callback
  void onVelocity(const geometry_msgs::TwistStamped::ConstPtr msg);
  void onPerception(const autoware_perception_msgs::DynamicObjectArray::ConstPtr msg);
  void onExternalApproval(const std_msgs::Bool & msg);
  void onForceApproval(const autoware_planning_msgs::PathChangeModule::ConstPtr & msg);
  void onMap(const autoware_lanelet2_msgs::MapBinConstPtr map_msg);
  void onRoute(const autoware_planning_msgs::RouteConstPtr route_msg);

  // (TODO) move to util
  autoware_planning_msgs::PathWithLaneId clipPathByGoal(
    const autoware_planning_msgs::PathWithLaneId & path) const;

  /**
   * @brief Execute behavior tree and publish planned data.
   */
  void run(const ros::TimerEvent & event);

  /**
   * @brief extract path from behavior tree output
   */
  std::shared_ptr<autoware_planning_msgs::PathWithLaneId> getPath(
    const BehaviorModuleOutput & bt_out);

  /**
   * @brief extract path candidate from behavior tree output
   */
  std::shared_ptr<autoware_planning_msgs::PathWithLaneId> getPathCandidate(
    const BehaviorModuleOutput & bt_out);

  /**
   * @brief publish behavior module status mainly for the user interface
   */
  void publishModuleStatus(const std::vector<std::shared_ptr<SceneModuleStatus>> & statuses);

  /**
   * @brief update current pose on the planner_data_
   */
  void updateCurrentPose();

  // debug
private:
  ros::Publisher debug_drivable_area_publisher_;
  ros::Publisher debug_path_publisher_;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER_HPP
