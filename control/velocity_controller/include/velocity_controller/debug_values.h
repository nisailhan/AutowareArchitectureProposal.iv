#ifndef AUTOWARE_CONTROL_VELOCITY_CONTROLLER_DEBUG_VALUES_H
#define AUTOWARE_CONTROL_VELOCITY_CONTROLLER_DEBUG_VALUES_H

class DebugValues
{
public:
  enum class TYPE {
    DT = 0,
    CURR_V = 1,
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
    CTRL_MODE = 13,
    ACCCMD_PID_APPLIED = 14,
    ACCCMD_ACC_LIMITED = 15,
    ACCCMD_JERK_LIMITED = 16,
    ACCCMD_SLOPE_APPLIED = 17,
    ACCCMD_PUBLISHED = 18,
    ACCCMD_FB_P_CONTRIBUTION = 19,
    ACCCMD_FB_I_CONTRIBUTION = 20,
    ACCCMD_FB_D_CONTRIBUTION = 21,
    FLAG_SMOOTH_STOP = 22,
    FLAG_EMERGENCY_STOP = 23,
    PREDICTED_V = 24,
    CALCULATED_ACC = 25,
    PITCH_RAW_TRAJ_RAD = 26,
    PITCH_RAW_TRAJ_DEG = 27,
    SIZE  // this is the number of enum elements
  };

  int getValuesIdx(TYPE type) const { return static_cast<int>(type); }
  std::array<double, static_cast<int>(TYPE::SIZE)> getValues() const { return values_; }

  void setValues(TYPE type, double val) { values_.at(static_cast<int>(type)) = val; }
  void setValues(int type, double val) { values_.at(type) = val; }

private:
  static constexpr int num_debug_values_ = static_cast<int>(TYPE::SIZE);
  std::array<double, static_cast<int>(TYPE::SIZE)> values_;
};

#endif
