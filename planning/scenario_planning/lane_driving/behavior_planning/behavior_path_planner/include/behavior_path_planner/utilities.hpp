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

#ifndef BEHAVIOR_PATH_PLANNER_UTILITIES_HPP
#define BEHAVIOR_PATH_PLANNER_UTILITIES_HPP

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "autoware_perception_msgs/DynamicObjectArray.h"
#include "autoware_planning_msgs/Path.h"
#include "autoware_planning_msgs/PathWithLaneId.h"
#include "autoware_utils/autoware_utils.h"

#include "boost/geometry/geometries/box.hpp"
#include "boost/geometry/geometries/point_xy.hpp"
#include "boost/geometry/geometries/polygon.hpp"
#include "boost/geometry/geometry.hpp"

#include "lanelet2_core/geometry/Lanelet.h"
#include "lanelet2_routing/Route.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_routing/RoutingGraphContainer.h"

#include <limits>
#include <vector>

#include "opencv2/opencv.hpp"

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/route_handler.hpp"

namespace autoware_utils
{
template <>
inline geometry_msgs::Point getPoint(const autoware_planning_msgs::PathPointWithLaneId & p)
{
  return p.point.pose.position;
}
}  // namespace autoware_utils

namespace behavior_path_planner
{
namespace util
{
using autoware_utils::LineString2d;
using autoware_utils::Point2d;
using autoware_utils::Polygon2d;

struct FrenetCoordinate3d
{
  double length = 0.0;    // longitudinal
  double distance = 0.0;  // lateral
  FrenetCoordinate3d() {}
};

std::vector<geometry_msgs::Point> convertToPointArray(
  const autoware_planning_msgs::PathWithLaneId & path);

double normalizeRadian(const double radian);
double l2Norm(const geometry_msgs::Vector3 vector);

Eigen::Vector3d convertToEigenPt(const geometry_msgs::Point geom_pt);
std::vector<geometry_msgs::Point> convertToGeometryPointArray(
  const autoware_planning_msgs::PathWithLaneId & path);
geometry_msgs::PoseArray convertToGeometryPoseArray(
  const autoware_planning_msgs::PathWithLaneId & path);

autoware_perception_msgs::PredictedPath convertToPredictedPath(
  const autoware_planning_msgs::PathWithLaneId & path, const geometry_msgs::Twist & vehicle_twist,
  const geometry_msgs::Pose & vehicle_pose, const double duration, const double resolution,
  const double acceleration);
autoware_perception_msgs::PredictedPath resamplePredictedPath(
  const autoware_perception_msgs::PredictedPath & input_path, const double resolution,
  const double duration);

bool convertToFrenetCoordinate3d(
  const autoware_planning_msgs::PathWithLaneId & path,
  const geometry_msgs::Point & search_point_geom, FrenetCoordinate3d * frenet_coordinate);

bool convertToFrenetCoordinate3d(
  const std::vector<geometry_msgs::Point> & linestring,
  const geometry_msgs::Point search_point_geom, FrenetCoordinate3d * frenet_coordinate);

geometry_msgs::Pose lerpByPose(
  const geometry_msgs::Pose & p1, const geometry_msgs::Pose & p2, const double t);

geometry_msgs::Point lerpByLength(
  const std::vector<geometry_msgs::Point> & array, const double length);
bool lerpByTimeStamp(
  const autoware_perception_msgs::PredictedPath & path, const ros::Time & t,
  geometry_msgs::Pose * lerped_pt);

double getDistance2d(const geometry_msgs::Pose & p1, const geometry_msgs::Pose & p2);
double getDistance2d(
  const autoware_planning_msgs::PathPointWithLaneId & p1,
  const autoware_planning_msgs::PathPointWithLaneId & p2);
double getDistance3d(const geometry_msgs::Point & p1, const geometry_msgs::Point & p2);

double getDistanceBetweenPredictedPaths(
  const autoware_perception_msgs::PredictedPath & path1,
  const autoware_perception_msgs::PredictedPath & path2, const double start_time,
  const double end_time, const double resolution);

double getDistanceBetweenPredictedPathAndObject(
  const autoware_perception_msgs::DynamicObject & object,
  const autoware_perception_msgs::PredictedPath & path, const double start_time,
  const double end_time, const double resolution);

/**
 * @brief Get index of the obstacles inside the lanelets with start and end length
 * @return Indices corresponding to the obstacle inside the lanelets
 */
std::vector<size_t> filterObjectsByLanelets(
  const autoware_perception_msgs::DynamicObjectArray & objects,
  const lanelet::ConstLanelets & lanelets, const double start_arc_length,
  const double end_arc_length);

/**
 * @brief Get index of the obstacles inside the lanelets
 * @return Indices corresponding to the obstacle inside the lanelets
 */
std::vector<size_t> filterObjectsByLanelets(
  const autoware_perception_msgs::DynamicObjectArray & objects,
  const lanelet::ConstLanelets & target_lanelets);

bool calcObjectPolygon(
  const autoware_perception_msgs::DynamicObject & object, Polygon2d * object_polygon);

std::vector<size_t> filterObjectsByPath(
  const autoware_perception_msgs::DynamicObjectArray & objects,
  const std::vector<size_t> & object_indices,
  const autoware_planning_msgs::PathWithLaneId & ego_path, const double vehicle_width);

const geometry_msgs::Pose refineGoal(
  const geometry_msgs::Pose & goal, const lanelet::ConstLanelet & goal_lanelet);

autoware_planning_msgs::PathWithLaneId refinePath(
  const double search_radius_range, const double search_rad_range,
  const autoware_planning_msgs::PathWithLaneId & input, const geometry_msgs::Pose & goal,
  const int64_t goal_lane_id);
autoware_planning_msgs::PathWithLaneId removeOverlappingPoints(
  const autoware_planning_msgs::PathWithLaneId & input_path);

bool containsGoal(const lanelet::ConstLanelets & lanes, const lanelet::Id & goal_id);

nav_msgs::OccupancyGrid generateDrivableArea(
  const lanelet::ConstLanelets & lanes, const geometry_msgs::PoseStamped & current_pose,
  const double width, const double height, const double resolution, const double vehicle_length,
  const RouteHandler & route_handler);

double getDistanceToEndOfLane(
  const geometry_msgs::Pose & current_pose, const lanelet::ConstLanelets & lanelets);

double getDistanceToNextIntersection(
  const geometry_msgs::Pose & current_pose, const lanelet::ConstLanelets & lanelets);
double getDistanceToCrosswalk(
  const geometry_msgs::Pose & current_pose, const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphContainer & overall_graphs);
double getSignedDistance(
  const geometry_msgs::Pose & current_pose, const geometry_msgs::Pose & goal_pose,
  const lanelet::ConstLanelets & lanelets);

std::vector<uint64_t> getIds(const lanelet::ConstLanelets & lanelets);

autoware_planning_msgs::Path convertToPathFromPathWithLaneId(
  const autoware_planning_msgs::PathWithLaneId & path_with_lane_id);

lanelet::Polygon3d getVehiclePolygon(
  const geometry_msgs::Pose & vehicle_pose, const double vehicle_width,
  const double base_link2front);

autoware_planning_msgs::PathPointWithLaneId insertStopPoint(
  double length, autoware_planning_msgs::PathWithLaneId * path);

double getArcLengthToTargetLanelet(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelet & target_lane,
  const geometry_msgs::Pose & pose);

std::vector<Polygon2d> getTargetLaneletPolygons(
  const lanelet::ConstLanelets & lanelets, const geometry_msgs::Pose & pose,
  const double check_length, const std::string & target_type);

std::vector<Polygon2d> filterObstaclePolygons(
  const std::vector<Polygon2d> & obstacle_polygons,
  const autoware_perception_msgs::DynamicObjectArray & objects,
  const double static_obstacle_velocity_thresh);

double getDistanceToNearestObstaclePolygon(
  const std::vector<Polygon2d> & obstacle_polygons, const geometry_msgs::Pose & pose);

// MEMO: From side shift
bool isUniqueId(const lanelet::Ids & ids, const lanelet::Id & id);

void occupancyGridToImage(const nav_msgs::OccupancyGrid & occupancy_grid, cv::Mat * cv_image);

void imageToOccupancyGrid(const cv::Mat & cv_image, nav_msgs::OccupancyGrid * occupancy_grid);

cv::Point toCVPoint(
  const geometry_msgs::Point & geom_point, const double width_m, const double height_m,
  const double resolution);

// MEMO: From Avoidance
std::vector<double> calcPathArcLengthArray(
  const autoware_planning_msgs::PathWithLaneId & path, size_t start = 0,
  size_t end = std::numeric_limits<size_t>::max());

double calcPathArcLength(
  const autoware_planning_msgs::PathWithLaneId & path, size_t start = 0,
  size_t end = std::numeric_limits<size_t>::max());

// TODO(Horibe) template?
std::vector<double> rangeVector(double start, double diff, double end);

// TODO(Horibe) There is a similar function in route_handler. Check.
std::shared_ptr<autoware_planning_msgs::PathWithLaneId> generateCenterLinePath(
  const std::shared_ptr<const PlannerData> & planner_data);

autoware_perception_msgs::DynamicObjectArray filterObjectsByVelocity(
  const autoware_perception_msgs::DynamicObjectArray & objects, double lim_v);
autoware_perception_msgs::DynamicObjectArray filterObjectsByVelocity(
  const autoware_perception_msgs::DynamicObjectArray & objects, double min_v, double max_v);

void shiftPose(geometry_msgs::Pose * pose, double shift_length);


}  // namespace util
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER_UTILITIES_HPP
