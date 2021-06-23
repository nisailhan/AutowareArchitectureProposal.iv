//
//  Copyright 2021 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
#pragma once

#include <algorithm>
#include <iostream>
#include <vector>

class PIDController
{
public:
  PIDController();
  ~PIDController() = default;

  double calculatePID(
    const double error, const double dt, const bool enable_integration,
    std::vector<double> & pid_contributions, std::vector<double> & errors, bool is_debugging);
  void setGains(const double kp, const double ki, const double kd);
  void setLimits(
    const double max_ret, const double min_ret, const double max_ret_p, const double min_ret_p,
    const double max_ret_i, const double min_ret_i, const double max_ret_d, const double min_ret_d);
  void reset();
  void setDecay(const double decay) { invalid_integration_decay_ = decay; };

private:
  // parameters
  double kp_;
  double ki_;
  double kd_;
  double max_ret_p_;
  double min_ret_p_;
  double max_ret_i_;
  double min_ret_i_;
  double max_ret_d_;
  double min_ret_d_;
  double max_ret_;
  double min_ret_;
  // states
  double error_integral_;
  double prev_error_;
  bool is_first_time_;
  double invalid_integration_decay_;
};
