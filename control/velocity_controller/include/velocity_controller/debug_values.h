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

#ifndef AUTOWARE_CONTROL_VELOCITY_CONTROLLER_DEBUG_VALUES_H
#define AUTOWARE_CONTROL_VELOCITY_CONTROLLER_DEBUG_VALUES_H

class DebugValues
{
public:
  enum class TYPE {
    DT = 0,
    CURRENT_V = 1,
    TARGET_V = 2,
    TARGET_ACC = 3,
    CLOSEST_V = 4,
    CLOSEST_ACC = 5,
    SHIFT = 6,
    PITCH_LPF_RAD = 7,
    PITCH_RAW_RAD = 8,
    PITCH_LPF_DEG = 9,
    PITCH_RAW_DEG = 10,
    ERROR_V = 11,
    ERROR_V_FILTERED = 12,
    CONTROL_STATE = 13,
    ACC_CMD_PID_APPLIED = 14,
    ACC_CMD_ACC_LIMITED = 15,
    ACC_CMD_JERK_LIMITED = 16,
    ACC_CMD_SLOPE_APPLIED = 17,
    ACC_CMD_PUBLISHED = 18,
    ACC_CMD_FB_P_CONTRIBUTION = 19,
    ACC_CMD_FB_I_CONTRIBUTION = 20,
    ACC_CMD_FB_D_CONTRIBUTION = 21,
    FLAG_STOPPING = 22,
    FLAG_EMERGENCY_STOP = 23,
    PREDICTED_V = 24,
    CALCULATED_ACC = 25,
    PITCH_RAW_TRAJ_RAD = 26,
    PITCH_RAW_TRAJ_DEG = 27,
    STOP_DIST = 28,
    SIZE  // this is the number of enum elements
  };

  int getValuesIdx(const TYPE type) const { return static_cast<int>(type); }
  std::array<double, static_cast<int>(TYPE::SIZE)> getValues() const { return values_; }

  void setValues(const TYPE type, const double val) { values_.at(static_cast<int>(type)) = val; }
  void setValues(const int type, const double val) { values_.at(type) = val; }

private:
  static constexpr int num_debug_values_ = static_cast<int>(TYPE::SIZE);
  std::array<double, static_cast<int>(TYPE::SIZE)> values_;
};

#endif
