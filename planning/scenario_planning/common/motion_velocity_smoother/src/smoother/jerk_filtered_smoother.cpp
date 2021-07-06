/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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
#include <chrono>
#include <cmath>
#include <numeric>

#include <eigen3/Eigen/Core>

#include "motion_velocity_smoother/smoother/jerk_filtered_smoother.hpp"
#include "motion_velocity_smoother/trajectory_utils.hpp"

namespace motion_velocity_smoother
{
JerkFilteredSmoother::JerkFilteredSmoother(const Param & smoother_param)
: smoother_param_(smoother_param)
{
  qp_solver_.updateMaxIter(20000);
  qp_solver_.updateRhoInterval(0);  // 0 means automatic
  qp_solver_.updateEpsRel(1.0e-4);  // def: 1.0e-4
  qp_solver_.updateEpsAbs(1.0e-8);  // def: 1.0e-4
  qp_solver_.updateVerbose(false);
}

void JerkFilteredSmoother::setParam(const Param & smoother_param)
{
  smoother_param_ = smoother_param;
}

bool JerkFilteredSmoother::apply(
  const double v0, const double a0, const autoware_planning_msgs::Trajectory & input,
  autoware_planning_msgs::Trajectory & output,
  std::vector<autoware_planning_msgs::Trajectory> & debug_trajectories)
{
  output = input;

  if (input.points.empty()) {
    ROS_WARN(
      "[JerkFilteredSmoother] Input Trajectory to the jerk filtered optimization is empty.");
    return false;
  }

  if (input.points.size() == 1) {
    // No need to do optimization
    output.points.front().twist.linear.x = v0;
    output.points.front().accel.linear.x = a0;
    debug_trajectories.resize(3);
    debug_trajectories[0] = output;
    debug_trajectories[1] = output;
    debug_trajectories[2] = output;
    return true;
  }

  if (std::fabs(input.points.front().twist.linear.x) < 0.1) {
    ROS_DEBUG("[JerkFilteredSmoother] v_max[0] < 0.1. assume vehicle stopped. return.");
    return false;
  }

  bool TMP_SHOW_DEBUG_INFO = false;

  const auto ts = std::chrono::system_clock::now();

  const double a_max = base_param_.max_accel;
  const double a_min = base_param_.min_decel;
  const double j_max = base_param_.max_jerk;
  const double j_min = base_param_.min_jerk;
  const double over_j_weight = smoother_param_.over_j_weight;
  const double over_v_weight = smoother_param_.over_v_weight;
  const double over_a_weight = smoother_param_.over_a_weight;

  // jerk filter
  const auto forward_filtered = forwardJerkFilter(v0, std::max(a0, a_min), a_max, j_max, input);
  const auto backward_filtered =
    backwardJerkFilter(input.points.back().twist.linear.x, 0.0, a_min, j_min, input);
  const auto filtered =
    mergeFilteredTrajectory(v0, a0, a_min, j_min, forward_filtered, backward_filtered);

  // Set debug trajectories
  debug_trajectories.resize(3);
  debug_trajectories[0] = forward_filtered;
  debug_trajectories[1] = backward_filtered;
  debug_trajectories[2] = filtered;

  const size_t N = filtered.points.size();

  std::vector<double> interval_dist_arr =
    trajectory_utils::calcTrajectoryIntervalDistance(filtered);

  std::vector<double> v_max_arr(N, 0.0);
  for (size_t i = 0; i < N; ++i) {
    v_max_arr.at(i) = filtered.points.at(i).twist.linear.x;
  }

  /*
   * x = [
   *      b[0], b[1], ..., b[N],               : 0~N
   *      a[0], a[1], .... a[N],               : N~2N
   *      delta[0], ..., delta[N],             : 2N~3N
   *      sigma[0], sigma[1], ...., sigma[N],  : 3N~4N
   *      gamma[0], gamma[1], ..., gamma[N]    : 4N~5N
   *     ]
   *
   * b[i]  : velocity^2
   * delta : 0 < b[i]-delta[i] < max_vel[i]*max_vel[i]
   * sigma : a_min < a[i] - sigma[i] < a_max
   * gamma : jerk_min < pseudo_jerk[i] * ref_vel[i] - gamma[i] < jerk_max
   */
  const uint32_t IDX_B0 = 0;
  const uint32_t IDX_A0 = N;
  const uint32_t IDX_DELTA0 = 2 * N;
  const uint32_t IDX_SIGMA0 = 3 * N;
  const uint32_t IDX_GAMMA0 = 4 * N;

  const uint32_t l_variables = 5 * N;
  const uint32_t l_constraints = 4 * N + 1;

  // the matrix size depends on constraint numbers.
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(l_constraints, l_variables);

  std::vector<double> lower_bound(l_constraints, 0.0);
  std::vector<double> upper_bound(l_constraints, 0.0);

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(l_variables, l_variables);
  std::vector<double> q(l_variables, 0.0);

  /**************************************************************/
  /**************************************************************/
  /**************** design objective function *******************/
  /**************************************************************/
  /**************************************************************/

  // jerk: d(ai)/ds * v_ref -> minimize weight * ((a1 - a0) / ds * v_ref)^2 * ds
  constexpr double ZERO_VEL_THR_FOR_DT_CALC = 0.3;
  const double smooth_weight = smoother_param_.jerk_weight;
  for (size_t i = 0; i < N - 1; ++i) {
    const double ref_vel = v_max_arr.at(i);
    const double interval_dist = std::max(interval_dist_arr.at(i), 0.0001);
    const double w_x_ds_inv = (1.0 / interval_dist) * ref_vel;
    P(IDX_A0 + i, IDX_A0 + i) += smooth_weight * w_x_ds_inv * w_x_ds_inv * interval_dist;
    P(IDX_A0 + i, IDX_A0 + i + 1) -= smooth_weight * w_x_ds_inv * w_x_ds_inv * interval_dist;
    P(IDX_A0 + i + 1, IDX_A0 + i) -= smooth_weight * w_x_ds_inv * w_x_ds_inv * interval_dist;
    P(IDX_A0 + i + 1, IDX_A0 + i + 1) += smooth_weight * w_x_ds_inv * w_x_ds_inv * interval_dist;
  }

  for (size_t i = 0; i < N; ++i) {
    q[IDX_B0 + i] = -1.0;  // |v_max^2 - b| -> minimize (-bi) * ds
    if (i < N - 1) {
      q[IDX_B0 + i] *= std::max(interval_dist_arr.at(i), 0.0001);
    }
    P(IDX_DELTA0 + i, IDX_DELTA0 + i) += over_v_weight;  // over velocity cost
    P(IDX_SIGMA0 + i, IDX_SIGMA0 + i) += over_a_weight;  // over acceleration cost
    P(IDX_GAMMA0 + i, IDX_GAMMA0 + i) += over_j_weight;  // over jerk cost
  }

  /**************************************************************/
  /**************************************************************/
  /**************** design constraint matrix ********************/
  /**************************************************************/
  /**************************************************************/

  /*
  NOTE: The delta allows b to be negative. This is actually invalid because the definition is b=v^2.
  But mathematically, the strict b>0 constraint may make the problem infeasible, such as the case of
  v=0 & a<0. To avoid the infeasibility, we allow b<0. The negative b is dealt as b=0 when it is
  converted to v with sqrt. If the weight of delta^2 is large (the value of delta is very small),
  b is almost 0, and is not a big problem.
  */

  size_t constr_idx = 0;

  // Soft Constraint Velocity Limit: 0 < b - delta < v_max^2
  for (size_t i = 0; i < N; ++i, ++constr_idx) {
    A(constr_idx, IDX_B0 + i) = 1.0;       // b_i
    A(constr_idx, IDX_DELTA0 + i) = -1.0;  // -delta_i
    upper_bound[constr_idx] = v_max_arr.at(i) * v_max_arr.at(i);
    lower_bound[constr_idx] = 0.0;
  }

  // Soft Constraint Acceleration Limit: a_min < a - sigma < a_max
  for (size_t i = 0; i < N; ++i, ++constr_idx) {
    A(constr_idx, IDX_A0 + i) = 1.0;       // a_i
    A(constr_idx, IDX_SIGMA0 + i) = -1.0;  // -sigma_i
    if (i != 0 && v_max_arr.at(i) < std::numeric_limits<double>::epsilon()) {
      upper_bound[constr_idx] = 0.0;
      lower_bound[constr_idx] = 0.0;
    } else {
      upper_bound[constr_idx] = a_max;
      lower_bound[constr_idx] = a_min;
    }
  }

  // Soft Constraint Jerk Limit: jerk_min < pseudo_jerk[i] * ref_vel[i] - gamma[i] < jerk_max
  // -> jerk_min * ds < (a[i+1] - a[i]) * ref_vel[i] - gamma[i] * ds < jerk_max * ds
  for (size_t i = 0; i < N - 1; ++i, ++constr_idx) {
    const double ref_vel = std::max(v_max_arr.at(i), ZERO_VEL_THR_FOR_DT_CALC);
    const double ds = interval_dist_arr.at(i);
    A(constr_idx, IDX_A0 + i) = -ref_vel;     // -a[i] * ref_vel
    A(constr_idx, IDX_A0 + i + 1) = ref_vel;  //  a[i+1] * ref_vel
    A(constr_idx, IDX_GAMMA0 + i) = -ds;      // -gamma[i] * ds
    upper_bound[constr_idx] = j_max * ds;     //  jerk_max * ds
    lower_bound[constr_idx] = j_min * ds;     //  jerk_min * ds
  }

  // b' = 2a ... (b(i+1) - b(i)) / ds = 2a(i)
  for (size_t i = 0; i < N - 1; ++i, ++constr_idx) {
    A(constr_idx, IDX_B0 + i) = -1.0;                            // b(i)
    A(constr_idx, IDX_B0 + i + 1) = 1.0;                         // b(i+1)
    A(constr_idx, IDX_A0 + i) = -2.0 * interval_dist_arr.at(i);  // a(i) * ds
    upper_bound[constr_idx] = 0.0;
    lower_bound[constr_idx] = 0.0;
  }

  // initial condition
  {
    A(constr_idx, IDX_B0) = 1.0;  // b0
    upper_bound[constr_idx] = v0 * v0;
    lower_bound[constr_idx] = v0 * v0;
    ++constr_idx;

    A(constr_idx, IDX_A0) = 1.0;  // a0
    upper_bound[constr_idx] = a0;
    lower_bound[constr_idx] = a0;
    ++constr_idx;
  }

  // execute optimization
  const auto result = qp_solver_.optimize(P, A, q, lower_bound, upper_bound);
  const std::vector<double> optval = std::get<0>(result);

  const auto tf1 = std::chrono::system_clock::now();
  const double dt_ms1 =
    std::chrono::duration_cast<std::chrono::nanoseconds>(tf1 - ts).count() * 1.0e-6;
  ROS_DEBUG("[JerkFilteredSmoother] optimization time = %f [ms]", dt_ms1);

  // get velocity & acceleration
  for (size_t i = 0; i < N; ++i) {
    double b = optval.at(IDX_B0 + i);
    output.points.at(i).twist.linear.x = std::sqrt(std::max(b, 0.0));
    output.points.at(i).accel.linear.x = optval.at(IDX_A0 + i);
  }
  for (size_t i = N; i < output.points.size(); ++i) {
    output.points.at(i).twist.linear.x = 0.0;
    output.points.at(i).accel.linear.x = 0.0;
  }

  const int status_val = std::get<3>(result);
  if (status_val != 1) {
    ROS_ERROR(
      "[JerkFilteredSmoother] optimization failed : %s", qp_solver_.getStatusMessage().c_str());
  }

  if (TMP_SHOW_DEBUG_INFO) {
    // jerk calculation
    std::vector<double> p_jerk, jerk_converted, jerk_ideal;
    for (size_t i = 0; i < input.points.size() - 1; ++i) {
      double v_ref0 = filtered.points.at(i).twist.linear.x;
      double v0 = output.points.at(i).twist.linear.x;
      double a0 = output.points.at(i).accel.linear.x;
      double a1 = output.points.at(i + 1).accel.linear.x;
      double ds = interval_dist_arr.at(i);
      p_jerk.push_back((a1 - a0) / ds);
      jerk_converted.push_back((a1 - a0) / ds * v_ref0);
      jerk_ideal.push_back((a1 - a0) / ds * v0);
    }
    p_jerk.push_back(0.0);
    jerk_converted.push_back(0.0);
    jerk_ideal.push_back(0.0);

    // print
    size_t i = 0;
    auto getVx = [&i](const autoware_planning_msgs::Trajectory & trajectory) {
      return trajectory.points.at(i).twist.linear.x;
    };
    auto getAx = [&i](const autoware_planning_msgs::Trajectory & trajectory) {
      return trajectory.points.at(i).accel.linear.x;
    };
    printf("v0 = %.3f, a0 = %.3f\n", v0, a0);
    for (; i < input.points.size(); ++i) {
      double gamma = optval.at(IDX_GAMMA0 + i);
      printf(
        "i = %lu, input: %.3f, filt_f: (%.3f, %.3f), filt_b: (%.3f, %.3f), filt_fb: (%.3f, %.3f), "
        "opt: (%.3f, %f), p_jerk = %.3f, p_jerk*v_ref: %.3f (min: %.3f, mac: %.3f), p_jerk*v_opt "
        "(to "
        "check): %.3f, gamma: %.3f\n",
        i, getVx(input), getVx(forward_filtered), getAx(forward_filtered), getVx(backward_filtered),
        getAx(backward_filtered), getVx(filtered), getAx(filtered), getVx(output), getAx(output),
        p_jerk.at(i), jerk_converted.at(i), j_min, j_max, jerk_ideal.at(i), gamma);
    }
    printf("\n");
  }

  return true;
}

autoware_planning_msgs::Trajectory JerkFilteredSmoother::forwardJerkFilter(
  const double v0, const double a0, const double a_max, const double j_max,
  const autoware_planning_msgs::Trajectory & input) const
{
  auto applyLimits = [&input](double & v, double & a, size_t i) {
    double v_lim = input.points.at(i).twist.linear.x;
    static constexpr double ep = 1.0e-5;
    if (v > v_lim + ep) {
      v = v_lim;
      a = 0.0;
    }
    if (v <= 0.0) {
      v = a = 0.0;
    }
  };

  auto output = input;

  double current_vel = v0;
  double current_acc = a0;
  applyLimits(current_vel, current_acc, 0);

  output.points.front().twist.linear.x = current_vel;
  output.points.front().accel.linear.x = current_acc;
  for (size_t i = 1; i < input.points.size(); ++i) {
    const double ds = autoware_utils::calcDistance2d(input.points.at(i), input.points.at(i - 1));
    const double max_dt = std::pow(6.0 * ds / j_max, 1.0 / 3.0);  // assuming v0 = a0 = 0.
    const double dt = std::min(ds / std::max(current_vel, 1.0e-6), max_dt);

    current_acc = std::min(current_acc + j_max * dt, a_max);
    current_vel = current_vel + current_acc * dt;
    applyLimits(current_vel, current_acc, i);
    output.points.at(i).twist.linear.x = current_vel;
    output.points.at(i).accel.linear.x = current_acc;
  }
  return output;
}

autoware_planning_msgs::Trajectory JerkFilteredSmoother::backwardJerkFilter(
  const double v0, const double a0, const double a_min, const double j_min,
  const autoware_planning_msgs::Trajectory & input) const
{
  auto input_rev = input;
  std::reverse(input_rev.points.begin(), input_rev.points.end());
  auto filtered = forwardJerkFilter(v0, a0, -a_min, -j_min, input_rev);
  std::reverse(filtered.points.begin(), filtered.points.end());
  return filtered;
}

autoware_planning_msgs::Trajectory JerkFilteredSmoother::mergeFilteredTrajectory(
  const double v0, const double a0, const double a_min, const double j_min,
  const autoware_planning_msgs::Trajectory & forward_filtered,
  const autoware_planning_msgs::Trajectory & backward_filtered) const
{
  autoware_planning_msgs::Trajectory merged;
  merged.header = forward_filtered.header;
  merged.points = forward_filtered.points;

  auto getVx = [](const autoware_planning_msgs::Trajectory & trajectory, int i) {
    return trajectory.points.at(i).twist.linear.x;
  };

  size_t i = 0;

  if (getVx(backward_filtered, 0) < v0) {
    double current_vel = v0;
    double current_acc = a0;
    while (getVx(backward_filtered, i) < current_vel && current_vel <= getVx(forward_filtered, i) &&
           i < merged.points.size() - 1) {
      merged.points.at(i).twist.linear.x = current_vel;

      const double ds = autoware_utils::calcDistance2d(
        forward_filtered.points.at(i + 1), forward_filtered.points.at(i));
      const double max_dt =
        std::pow(6.0 * ds / std::fabs(j_min), 1.0 / 3.0);  // assuming v0 = a0 = 0.
      const double dt = std::min(ds / std::max(current_vel, 1.0e-6), max_dt);

      current_acc = std::max(current_acc + j_min * dt, a_min);
      current_vel = current_vel + current_acc * dt;
      ++i;
    }
  }

  // take smaller velocity point
  for (; i < merged.points.size(); ++i) {
    merged.points.at(i) = (getVx(forward_filtered, i) < getVx(backward_filtered, i))
                            ? forward_filtered.points.at(i)
                            : backward_filtered.points.at(i);
  }
  return merged;
}
}  // namespace motion_velocity_smoother
