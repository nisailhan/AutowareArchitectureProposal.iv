// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <memory>
#include <regex>
#include <string>
#include <utility>
#include <vector>
#include <set>

#define FMT_HEADER_ONLY
#include "fmt/format.h"

#include "autoware_error_monitor/autoware_error_monitor_core.hpp"
#include "autoware_error_monitor/diagnostics_filter.hpp"

namespace
{
int str2level(const std::string & level_str)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  using std::regex_constants::icase;

  if (std::regex_match(level_str, std::regex("warn", icase))) {return DiagnosticStatus::WARN;}
  if (std::regex_match(level_str, std::regex("error", icase))) {return DiagnosticStatus::ERROR;}
  if (std::regex_match(level_str, std::regex("stale", icase))) {return DiagnosticStatus::STALE;}

  throw std::runtime_error(fmt::format("invalid level: {}", level_str));
}

bool isOverLevel(const int & diag_level, const std::string & failure_level_str)
{
  if (failure_level_str == "none") {
    return false;
  }

  return diag_level >= str2level(failure_level_str);
}

std::vector<diagnostic_msgs::msg::DiagnosticStatus> & getTargetDiagnosticsRef(
  const int hazard_level, autoware_system_msgs::msg::HazardStatus * hazard_status)
{
  using autoware_system_msgs::msg::HazardStatus;

  if (hazard_level == HazardStatus::NO_FAULT) {return hazard_status->diagnostics_nf;}
  if (hazard_level == HazardStatus::SAFE_FAULT) {return hazard_status->diagnostics_sf;}
  if (hazard_level == HazardStatus::LATENT_FAULT) {return hazard_status->diagnostics_lf;}
  if (hazard_level == HazardStatus::SINGLE_POINT_FAULT) {return hazard_status->diagnostics_spf;}

  throw std::runtime_error(fmt::format("invalid hazard level: {}", hazard_level));
}

diagnostic_msgs::msg::DiagnosticArray convertHazardStatusToDiagnosticArray(
  rclcpp::Clock::SharedPtr clock, const autoware_system_msgs::msg::HazardStatus & hazard_status)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = clock->now();

  const auto decorateDiag = [](const auto & hazard_diag, const std::string & label) {
      auto diag = hazard_diag;

      diag.message = label + diag.message;

      return diag;
    };

  for (const auto & hazard_diag : hazard_status.diagnostics_nf) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[No Fault]"));
  }
  for (const auto & hazard_diag : hazard_status.diagnostics_sf) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[Safe Fault]"));
  }
  for (const auto & hazard_diag : hazard_status.diagnostics_lf) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[Latent Fault]"));
  }
  for (const auto & hazard_diag : hazard_status.diagnostics_spf) {
    diag_array.status.push_back(decorateDiag(hazard_diag, "[Single Point Fault]"));
  }

  return diag_array;
}

std::set<std::string> getErrorModules(
  const autoware_system_msgs::msg::HazardStatus & hazard_status, const int emergency_hazard_level)
{
  std::set<std::string> error_modules;
  using autoware_system_msgs::msg::HazardStatus;
  if (emergency_hazard_level <= HazardStatus::SINGLE_POINT_FAULT) {
    for (const auto & diag_spf : hazard_status.diagnostics_spf) {
      error_modules.insert(diag_spf.name);
    }
  }
  if (emergency_hazard_level <= HazardStatus::LATENT_FAULT) {
    for (const auto & diag_lf : hazard_status.diagnostics_lf) {
      error_modules.insert(diag_lf.name);
    }
  }
  if (emergency_hazard_level <= HazardStatus::SAFE_FAULT) {
    for (const auto & diag_sf : hazard_status.diagnostics_sf) {
      error_modules.insert(diag_sf.name);
    }
  }

  return error_modules;
}

autoware_system_msgs::msg::HazardStatus createTimeoutHazardStatus()
{
  autoware_system_msgs::msg::HazardStatus hazard_status;
  hazard_status.level = autoware_system_msgs::msg::HazardStatus::SINGLE_POINT_FAULT;
  hazard_status.emergency = true;
  hazard_status.emergency_holding = false;
  diagnostic_msgs::msg::DiagnosticStatus diag;
  diag.name = "autoware_error_monitor/input_data_timeout";
  diag.hardware_id = "autoware_error_monitor";
  diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  hazard_status.diagnostics_spf.push_back(diag);
  return hazard_status;
}

int isInNoFaultCondition(
  const autoware_system_msgs::msg::AutowareState & autoware_state,
  const autoware_control_msgs::msg::GateMode & current_gate_mode)
{
  using autoware_control_msgs::msg::GateMode;
  using autoware_system_msgs::msg::AutowareState;

  const auto is_in_auto_ignore_state =
    (autoware_state.state == AutowareState::INITIALIZING_VEHICLE) ||
    (autoware_state.state == AutowareState::WAITING_FOR_ROUTE) ||
    (autoware_state.state == AutowareState::PLANNING) ||
    (autoware_state.state == AutowareState::FINALIZING);

  if (current_gate_mode.data == GateMode::AUTO && is_in_auto_ignore_state) {
    return true;
  }

  const auto is_in_remote_ignore_state =
    (autoware_state.state == AutowareState::INITIALIZING_VEHICLE) ||
    (autoware_state.state == AutowareState::FINALIZING);

  if (current_gate_mode.data == GateMode::EXTERNAL && is_in_remote_ignore_state) {
    return true;
  }

  return false;
}
}  // namespace

AutowareErrorMonitor::AutowareErrorMonitor()
: Node("autoware_error_monitor")
{
  // Parameter
  params_.update_rate = declare_parameter("update_rate", 10);
  params_.ignore_missing_diagnostics = declare_parameter("ignore_missing_diagnostics", false);
  params_.add_leaf_diagnostics = declare_parameter("add_leaf_diagnostics", true);
  params_.data_ready_timeout = declare_parameter<double>("data_ready_timeout", 30.0);
  params_.diag_timeout_sec = declare_parameter<double>("diag_timeout_sec", 1.0);
  params_.hazard_recovery_timeout = declare_parameter<double>("hazard_recovery_timeout", 5.0);
  params_.emergency_hazard_level = declare_parameter<int>(
    "emergency_hazard_level",
    autoware_system_msgs::msg::HazardStatus::LATENT_FAULT);
  params_.use_emergency_hold = declare_parameter<bool>("use_emergency_hold", false);
  params_.use_emergency_hold_in_manual_driving =
    declare_parameter<bool>("use_emergency_hold_in_manual_driving", false);

  loadRequiredModules(KeyName::autonomous_driving);
  loadRequiredModules(KeyName::remote_control);

  using std::placeholders::_1;
  using std::placeholders::_2;
  // Subscriber
  sub_diag_array_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "input/diag_array", rclcpp::QoS{1},
    std::bind(&AutowareErrorMonitor::onDiagArray, this, _1));
  sub_current_gate_mode_ = create_subscription<autoware_control_msgs::msg::GateMode>(
    "~/input/current_gate_mode", rclcpp::QoS{1},
    std::bind(&AutowareErrorMonitor::onCurrentGateMode, this, _1));
  sub_autoware_state_ = create_subscription<autoware_system_msgs::msg::AutowareState>(
    "~/input/autoware_state", rclcpp::QoS{1},
    std::bind(&AutowareErrorMonitor::onAutowareState, this, _1));
  sub_control_mode_ = create_subscription<autoware_vehicle_msgs::msg::ControlMode>(
    "~/input/control_mode", rclcpp::QoS{1}, std::bind(
      &AutowareErrorMonitor::onControlMode, this, _1));

  // Publisher
  pub_hazard_status_ = create_publisher<autoware_system_msgs::msg::HazardStatusStamped>(
    "~/output/hazard_status", rclcpp::QoS{1});
  pub_diagnostics_err_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "~/output/diagnostics_err", rclcpp::QoS{1});

  // Service
  srv_clear_emergency_ = this->create_service<std_srvs::srv::Trigger>(
    "service/clear_emergency",
    std::bind(&AutowareErrorMonitor::onClearEmergencyService, this, _1, _2));

  // Initialize
  autoware_vehicle_msgs::msg::ControlMode control_mode;
  control_mode.data = autoware_vehicle_msgs::msg::ControlMode::MANUAL;
  control_mode_ = std::make_shared<const autoware_vehicle_msgs::msg::ControlMode>(control_mode);

  // Timer
  initialized_time_ = this->now();
  auto timer_callback = std::bind(&AutowareErrorMonitor::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / params_.update_rate));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void AutowareErrorMonitor::loadRequiredModules(const std::string & key)
{
  const auto param_key = std::string("required_modules.") + key + std::string(".names");

  const auto names = this->declare_parameter<std::vector<std::string>>(param_key);

  if (names.size() == 0) {
    throw std::runtime_error(fmt::format("no parameter found: {}", param_key));
  }

  RequiredModules required_modules;
  required_modules.reserve(names.size());

  DiagLevel default_level{{"sf_at", ""}, {"lf_at", ""}, {"spf_at", ""}};

  for (const auto & module_name : names) {
    // diag_level
    const auto diag_level_key =
      std::string("required_modules.") + key + std::string(".diag_level.") + module_name;
    this->declare_parameters(diag_level_key, default_level);
    DiagLevel diag_level{};
    this->get_parameters(diag_level_key, diag_level);
    // auto_recovery
    const auto auto_recovery_key =
      std::string("required_modules.") + key + std::string(".auto_recovery.") + module_name;
    const bool auto_recovery_approval = this->declare_parameter(auto_recovery_key, true);
    required_modules.emplace_back(module_name, diag_level, auto_recovery_approval);
  }

  required_modules_map_.insert(std::make_pair(key, required_modules));
}

void AutowareErrorMonitor::onDiagArray(
  const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
{
  diag_array_ = msg;

  const auto & header = msg->header;

  for (const auto & diag : msg->status) {
    if (diag_buffer_map_.count(diag.name) == 0) {
      diag_buffer_map_.insert(std::make_pair(diag.name, DiagBuffer{}));
    }

    auto & diag_buffer = diag_buffer_map_.at(diag.name);
    diag_buffer.push_back(DiagStamped{header, diag});

    while (diag_buffer.size() > diag_buffer_size_) {
      diag_buffer.pop_front();
    }
  }
}

void AutowareErrorMonitor::onCurrentGateMode(
  const autoware_control_msgs::msg::GateMode::ConstSharedPtr msg)
{
  current_gate_mode_ = msg;
}

void AutowareErrorMonitor::onAutowareState(
  const autoware_system_msgs::msg::AutowareState::ConstSharedPtr msg)
{
  autoware_state_ = msg;
}

void AutowareErrorMonitor::onControlMode(
  const autoware_vehicle_msgs::msg::ControlMode::ConstSharedPtr msg)
{
  control_mode_ = msg;
}

bool AutowareErrorMonitor::isDataReady()
{
  if (!diag_array_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for diag_array msg...");
    return false;
  }

  if (!current_gate_mode_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for current_gate_mode msg...");
    return false;
  }

  if (!autoware_state_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for autoware_state msg...");
    return false;
  }

  if (!control_mode_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for control_mode msg...");
    return false;
  }
  return true;
}

void AutowareErrorMonitor::onTimer()
{
  if (!isDataReady()) {
    if ((this->now() - initialized_time_).seconds() > params_.data_ready_timeout) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
        "input data is timeout");
      publishHazardStatus(createTimeoutHazardStatus());
    }
    return;
  }

  current_mode_ = current_gate_mode_->data == autoware_control_msgs::msg::GateMode::AUTO ?
    KeyName::autonomous_driving : KeyName::remote_control;

  updateHazardStatus();
  publishHazardStatus(hazard_status_);
}

boost::optional<DiagStamped> AutowareErrorMonitor::getLatestDiag(const std::string & diag_name)
const
{
  if (diag_buffer_map_.count(diag_name) == 0) {
    return {};
  }

  const auto & diag_buffer = diag_buffer_map_.at(diag_name);

  if (diag_buffer.empty()) {
    return {};
  }

  return diag_buffer.back();
}

int AutowareErrorMonitor::getHazardLevel(
  const DiagConfig & required_module, const int diag_level) const
{
  using autoware_system_msgs::msg::HazardStatus;

  if (isOverLevel(diag_level, required_module.spf_at)) {return HazardStatus::SINGLE_POINT_FAULT;}
  if (isOverLevel(diag_level, required_module.lf_at)) {return HazardStatus::LATENT_FAULT;}
  if (isOverLevel(diag_level, required_module.sf_at)) {return HazardStatus::SAFE_FAULT;}

  return HazardStatus::NO_FAULT;
}

void AutowareErrorMonitor::appendHazardDiag(
  const DiagConfig & required_module, const diagnostic_msgs::msg::DiagnosticStatus & hazard_diag,
  autoware_system_msgs::msg::HazardStatus * hazard_status) const
{
  const auto hazard_level = getHazardLevel(required_module, hazard_diag.level);

  auto & target_diagnostics_ref = getTargetDiagnosticsRef(hazard_level, hazard_status);
  target_diagnostics_ref.push_back(hazard_diag);

  if (params_.add_leaf_diagnostics) {
    for (const auto & diag :
      diagnostics_filter::extractLeafChildrenDiagnostics(hazard_diag, diag_array_->status))
    {
      target_diagnostics_ref.push_back(diag);
    }
  }

  hazard_status->level = std::max(hazard_status->level, hazard_level);
}

autoware_system_msgs::msg::HazardStatus AutowareErrorMonitor::judgeHazardStatus() const
{
  using autoware_system_msgs::msg::HazardStatus;
  using diagnostic_msgs::msg::DiagnosticStatus;

  autoware_system_msgs::msg::HazardStatus hazard_status;
  for (const auto & required_module : required_modules_map_.at(current_mode_)) {
    const auto & diag_name = required_module.name;
    const auto latest_diag = getLatestDiag(diag_name);

    // no diag found
    if (!latest_diag) {
      if (!params_.ignore_missing_diagnostics) {
        DiagnosticStatus missing_diag;

        missing_diag.name = diag_name;
        missing_diag.hardware_id = "autoware_error_monitor";
        missing_diag.level = DiagnosticStatus::STALE;
        missing_diag.message = "no diag found";

        appendHazardDiag(required_module, missing_diag, &hazard_status);
      }

      continue;
    }

    // diag level high
    {
      appendHazardDiag(required_module, latest_diag->status, &hazard_status);
    }

    // diag timeout
    {
      const auto time_diff = this->now() - latest_diag->header.stamp;
      if (time_diff.seconds() > params_.diag_timeout_sec) {
        DiagnosticStatus timeout_diag = latest_diag->status;
        timeout_diag.level = DiagnosticStatus::STALE;
        timeout_diag.message = "timeout";

        appendHazardDiag(required_module, timeout_diag, &hazard_status);
      }
    }
  }

  // Ignore error when vehicle is not ready to start
  if (isInNoFaultCondition(*autoware_state_, *current_gate_mode_)) {
    hazard_status.level = autoware_system_msgs::msg::HazardStatus::NO_FAULT;
  }

  return hazard_status;
}

void AutowareErrorMonitor::updateHazardStatus()
{
  const bool prev_emergency_status = hazard_status_.emergency;

  // Create hazard status based on diagnostics
  if (!hazard_status_.emergency_holding) {
    const auto current_hazard_status = judgeHazardStatus();
    hazard_status_.level = current_hazard_status.level;
    hazard_status_.diagnostics_nf = current_hazard_status.diagnostics_nf;
    hazard_status_.diagnostics_sf = current_hazard_status.diagnostics_sf;
    hazard_status_.diagnostics_lf = current_hazard_status.diagnostics_lf;
    hazard_status_.diagnostics_spf = current_hazard_status.diagnostics_spf;
  }

  // Update emergency status
  {
    hazard_status_.emergency = hazard_status_.level >= params_.emergency_hazard_level;
    if (hazard_status_.emergency != prev_emergency_status) {
      emergency_state_switch_time_ = this->now();
    }
  }

  // Update emergency_holding condition
  if (params_.use_emergency_hold) {
    hazard_status_.emergency_holding = isEmergencyHoldingRequired();
  }
}

bool AutowareErrorMonitor::canAutoRecovery() const
{
  const auto error_modules = getErrorModules(hazard_status_, params_.emergency_hazard_level);
  for (const auto & required_module : required_modules_map_.at(current_mode_)) {
    if (required_module.auto_recovery) {
      continue;
    }
    if (error_modules.count(required_module.name) != 0) {
      return false;
    }
  }
  return true;
}

bool AutowareErrorMonitor::isEmergencyHoldingRequired() const
{
  // Does not change holding status until emergency_holding is cleared by service call
  if (hazard_status_.emergency_holding) {
    return true;
  }

  if (!hazard_status_.emergency) {
    return false;
  }

  // Don't hold status if emergency duration within recovery timeout
  const auto emergency_duration = (this->now() - emergency_state_switch_time_).seconds();
  const auto within_recovery_timeout = emergency_duration < params_.hazard_recovery_timeout;
  if (within_recovery_timeout && canAutoRecovery()) {
    return false;
  }

  // Don't hold status during manual driving
  const bool is_manual_driving =
    (control_mode_->data == autoware_vehicle_msgs::msg::ControlMode::MANUAL);
  const auto no_hold_condition =
    (!params_.use_emergency_hold_in_manual_driving && is_manual_driving);
  if (no_hold_condition) {
    return false;
  }

  return true;
}

void AutowareErrorMonitor::publishHazardStatus(
  const autoware_system_msgs::msg::HazardStatus & hazard_status)
{
  autoware_system_msgs::msg::HazardStatusStamped hazard_status_stamped;
  hazard_status_stamped.header.stamp = this->now();
  hazard_status_stamped.status = hazard_status;
  pub_hazard_status_->publish(hazard_status_stamped);
  pub_diagnostics_err_->publish(
    convertHazardStatusToDiagnosticArray(this->get_clock(), hazard_status_stamped.status));
}

bool AutowareErrorMonitor::onClearEmergencyService(
  [[maybe_unused]] std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  hazard_status_.emergency_holding = false;
  updateHazardStatus();
  response->success = true;
  response->message = "Emergency Holding state was cleared.";

  return true;
}
